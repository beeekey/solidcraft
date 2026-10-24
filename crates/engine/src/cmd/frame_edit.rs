//! Editing a frame after it is built: `design.frame_end_cap` (a plate over a member's end),
//! `design.frame_notch` (one member cut by another), `design.frame_change_section` (the same
//! member on a new profile), `design.frame_extend` (a member longer or shorter at one end) and
//! `design.groove_weld` (the gap between two parallel faces filled as a weld body). Each works
//! from the member's own geometry, so it takes any straight member, not only a frame's.

use serde_json::{Value, json};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use super::design::{number, run};
use crate::params::{bad, bool_, str_, vec3};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("design.frame_end_cap", "End Cap", end_cap).at("3DEEE", "STRUCTURE").icon("frame_end_cap").params(
        "member: body name; thickness_mm? (default 5); end?: start | end (default end: the far end along the member's axis) | both; at?: [x, y, z] \
         picks the end nearest this point instead → member, end, thickness_mm, axis, section_mm: [u, v], caps. A plate covering the member's \
         cross-section at its end, joined to it; the axis is the member's longest straight edge",
    ),
    CommandSpec::new("design.frame_notch", "Notch", notch)
        .at("3DEEE", "STRUCTURE")
        .icon("frame_notch")
        .params("member: the body to notch; tool: the member cutting it (kept) → member, tool. One member cut back around another where they cross"),
    CommandSpec::new("design.frame_change_section", "Change Section", change_section).at("3DEEE", "STRUCTURE").icon("frame_change_section").params(
        "member: body name; profile: {family, size} or {family, a, b, t, t2} as design.frame; rotation_deg?, offset? as design.frame; keep?: true \
         leaves the old member visible (default: hidden) → member, replaced_by, length_mm, profile. The member rebuilt on its own axis with \
         the new profile, butt ends",
    ),
    CommandSpec::new("design.frame_extend", "Extend Member", extend).at("3DEEE", "STRUCTURE").icon("frame_extend").params(
        "member: body name; amount_mm: positive extends, negative trims; end?: start | end (default end); at?: [x, y, z] picks the end nearest \
         this point → member, end, amount_mm, length_mm (after). The member's end face pushed out or cut back along its axis",
    ),
    CommandSpec::new("design.frame_mitre", "Mitre", mitre).at("3DEEE", "STRUCTURE").icon("frame_mitre").params(
        "member_a, member_b: two members that meet at a corner → members, corner, plane_normal. Each is cut on the plane halving their corner, \
         so the pair closes in a clean joint; the first keeps the side of the plane its own axis comes from",
    ),
    CommandSpec::new("design.groove_weld", "Groove Weld", groove_weld).at("3DEEE", "STRUCTURE").icon("groove_weld").params(
        "face_a: [x, y, z] on the first face, face_b: [x, y, z] on the second (two parallel flat faces of two bodies, facing each other across \
         the gap); body_a?, body_b?: whose faces (default: the bodies under the points); name? → body (the bead), gap_mm, area_mm2, \
         volume_mm3. The gap filled by extruding the first face to the second: a groove weld's bead as its own body",
    ),
];

fn arr(v: Vec3) -> Value {
    json!([v.x, v.y, v.z])
}

/// A member's axis: the direction of its longest straight edge, through its box's centre, and
/// how far the body reaches along it either way.
struct Axis {
    centre: Vec3,
    dir: Vec3,
    t_min: f64,
    t_max: f64,
    u: Vec3,
    /// The section's extent across the axis: (u_min, u_max, v_min, v_max).
    section: [f64; 4],
}

fn axis_of(s: &Session, member: &str, cmd: &str) -> Result<Axis> {
    let st = s.world_state();
    let b = st.body(member).ok_or_else(|| bad(cmd, format!("no body `{member}`")))?;
    let tol = (b.body.size() * 2e-3).max(1e-3);
    let edges = b.body.edges(tol)?;
    let mut best: Option<(f64, Vec3)> = None;
    for e in &edges {
        let (Some(a), Some(z)) = (e.points.first(), e.points.last()) else { continue };
        // Straight: the polyline's chord is its length.
        let chord = (*z - *a).len();
        if chord < 1e-6 || (e.length - chord).abs() > 1e-3 * chord.max(1.0) {
            continue;
        }
        if best.is_none_or(|(l, _)| chord > l) {
            best = Some((chord, (*z - *a) / chord));
        }
    }
    let (_, dir) = best.ok_or_else(|| bad(cmd, format!("`{member}` has no straight edge to take its axis from")))?;
    let mesh = b.mesh();
    let bounds = mesh.bounds();
    let centre = (bounds.min + bounds.max) * 0.5;
    let helper = if dir.z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
    let u = (helper - dir * helper.dot(dir)).normalized().unwrap_or(Vec3::X);
    let v = dir.cross(u);
    let (mut t_min, mut t_max) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut section = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
    for q in &mesh.positions {
        let rel = *q - centre;
        let t = rel.dot(dir);
        t_min = t_min.min(t);
        t_max = t_max.max(t);
        let (pu, pv) = (rel.dot(u), rel.dot(v));
        section[0] = section[0].min(pu);
        section[1] = section[1].max(pu);
        section[2] = section[2].min(pv);
        section[3] = section[3].max(pv);
    }
    if !(t_max > t_min) {
        return Err(bad(cmd, format!("`{member}` has no length")));
    }
    Ok(Axis { centre, dir, t_min, t_max, u, section })
}

/// Which end: `end` (start | end | both), or the one nearest `at`.
fn ends_of(p: &Value, axis: &Axis, cmd: &str, allow_both: bool) -> Result<Vec<bool>> {
    if let Some(at) = p.get("at").and_then(vec3) {
        let t = (at - axis.centre).dot(axis.dir);
        return Ok(vec![(t - axis.t_max).abs() <= (t - axis.t_min).abs()]);
    }
    match str_(p, "end").map(|e| e.trim().to_ascii_lowercase()).as_deref() {
        None | Some("end") | Some("far") => Ok(vec![true]),
        Some("start") | Some("near") => Ok(vec![false]),
        Some("both") if allow_both => Ok(vec![false, true]),
        Some(_) => Err(bad(cmd, if allow_both { "`end` is start, end or both" } else { "`end` is start or end" })),
    }
}

fn member_of(s: &Session, p: &Value, key: &str, cmd: &str) -> Result<String> {
    match p.get(key) {
        Some(Value::String(n)) => {
            if s.world_state().body(n).is_none() {
                return Err(bad(cmd, format!("no body `{n}`")));
            }
            Ok(n.clone())
        }
        None | Some(Value::Null) if key == "member" => {
            let st = s.world_state();
            let selected: Vec<String> =
                s.selection.iter().filter_map(|x| if let crate::Sel::Body { name } = x { Some(name.clone()) } else { None }).collect();
            let all: Vec<String> = st.bodies.iter().map(|b| b.name.clone()).collect();
            match (selected.as_slice(), all.as_slice()) {
                ([one], _) | (_, [one]) => Ok(one.clone()),
                _ => Err(bad(cmd, "give `member` (or select one body)")),
            }
        }
        _ => Err(bad(cmd, format!("give `{key}`: a body name"))),
    }
}

fn end_cap(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.frame_end_cap";
    let member = member_of(s, p, "member", cmd)?;
    let thickness = number(cmd, p, "thickness_mm", Some(5.0), 0.1..=500.0)?.unwrap_or(5.0);
    let axis = axis_of(s, &member, cmd)?;
    let ends = ends_of(p, &axis, cmd, true)?;
    let [u0, u1, v0, v1] = axis.section;
    let mut caps = 0usize;
    for far in &ends {
        let out = if *far { axis.dir } else { -axis.dir };
        let origin = axis.centre + axis.dir * if *far { axis.t_max } else { axis.t_min };
        // A frame whose normal (x × y) is the outward direction.
        let x = axis.u;
        let y = out.cross(x);
        let (pu, pv) = (|q: f64| q, |q: f64| if *far { q } else { -q });
        run(s, "sketch.create", json!({"plane": {"origin": arr(origin), "x_dir": arr(x), "y_dir": arr(y)}}))?;
        let (a, b) = ([pu(u0), pv(v0).min(pv(v1))], [pu(u1), pv(v0).max(pv(v1))]);
        run(s, "sketch.rectangle.two_point", json!({"p0": a, "p1": b}))?;
        run(s, "sketch.finish", json!({}))?;
        run(s, "solid.extrude", json!({"distance": thickness, "operation": "join", "targets": [member], "profiles": "all"}))?;
        caps += 1;
    }
    Ok(json!({
        "member": member,
        "end": if ends.len() == 2 { "both" } else if ends[0] { "end" } else { "start" },
        "thickness_mm": thickness,
        "axis": arr(axis.dir),
        "section_mm": [u1 - u0, v1 - v0],
        "caps": caps,
    }))
}

fn notch(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.frame_notch";
    let member = member_of(s, p, "member", cmd)?;
    let tool = member_of(s, p, "tool", cmd)?;
    if member == tool {
        return Err(bad(cmd, "`member` and `tool` must be two different bodies"));
    }
    run(s, "solid.combine", json!({"target": member, "tools": [tool], "operation": "cut", "keep_tools": true}))?;
    Ok(json!({"member": member, "tool": tool}))
}

fn change_section(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.frame_change_section";
    let member = member_of(s, p, "member", cmd)?;
    let profile = p.get("profile").cloned().ok_or_else(|| bad(cmd, "give `profile`: {family, size} or {family, a, b, t, t2}"))?;
    let axis = axis_of(s, &member, cmd)?;
    let from = axis.centre + axis.dir * axis.t_min;
    let to = axis.centre + axis.dir * axis.t_max;
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{member} resection"));
    let mut q = json!({"profile": profile, "members": [{"from": arr(from), "to": arr(to)}], "ends": "butt", "trim": false, "name": name});
    for k in ["rotation_deg", "offset"] {
        if let Some(v) = p.get(k) {
            q[k] = v.clone();
        }
    }
    let built = run(s, "design.frame", q)?;
    if !bool_(p, "keep").unwrap_or(false) {
        let _ = s.execute("browser.visibility", &json!({"items": [member], "visible": false}));
    }
    Ok(json!({
        "member": member,
        "replaced_by": built["bodies"].get(0).cloned().unwrap_or(Value::Null),
        "length_mm": axis.t_max - axis.t_min,
        "profile": built["profile"].clone(),
        "component": built["component"].clone(),
    }))
}

fn extend(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.frame_extend";
    let member = member_of(s, p, "member", cmd)?;
    let amount =
        number(cmd, p, "amount_mm", None, -10_000.0..=10_000.0)?.ok_or_else(|| bad(cmd, "give `amount_mm`: positive extends, negative trims"))?;
    if amount.abs() < 1e-6 {
        return Err(bad(cmd, "`amount_mm` must not be zero"));
    }
    let axis = axis_of(s, &member, cmd)?;
    let far = ends_of(p, &axis, cmd, false)?[0];
    let length = axis.t_max - axis.t_min;
    if amount < 0.0 && -amount >= length {
        return Err(bad(cmd, "the trim would remove the whole member"));
    }
    let out = if far { axis.dir } else { -axis.dir };
    let origin = axis.centre + axis.dir * if far { axis.t_max } else { axis.t_min };
    // The end face, a hair inside the body so the pick lands on it.
    let point = origin - out * 1e-3;
    if amount > 0.0 {
        run(s, "solid.extrude", json!({"face": arr(point), "face_body": member, "distance": amount, "operation": "join", "targets": [member]}))?;
    } else {
        run(
            s,
            "solid.extrude",
            json!({"face": arr(point), "face_body": member, "distance": -amount, "operation": "cut", "targets": [member], "direction": "negative"}),
        )?;
    }
    Ok(json!({"member": member, "end": if far { "end" } else { "start" }, "amount_mm": amount, "length_mm": length + amount}))
}

/// Cuts everything of `member` on the `normal` side of the plane through `origin`.
fn cut_half_space(s: &mut Session, member: &str, origin: Vec3, normal: Vec3, reach: f64) -> Result<()> {
    let helper = if normal.z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
    let x = (helper - normal * helper.dot(normal)).normalized().unwrap_or(Vec3::X);
    let y = normal.cross(x);
    run(s, "sketch.create", json!({"plane": {"origin": arr(origin), "x_dir": arr(x), "y_dir": arr(y)}}))?;
    run(s, "sketch.rectangle.two_point", json!({"p0": [-reach, -reach], "p1": [reach, reach]}))?;
    run(s, "sketch.finish", json!({}))?;
    run(s, "solid.extrude", json!({"distance": reach, "operation": "cut", "targets": [member], "profiles": "all"}))?;
    Ok(())
}

fn mitre(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.frame_mitre";
    let a = member_of(s, p, "member_a", cmd)?;
    let b = member_of(s, p, "member_b", cmd)?;
    if a == b {
        return Err(bad(cmd, "`member_a` and `member_b` must be two different bodies"));
    }
    let ax = axis_of(s, &a, cmd)?;
    let bx = axis_of(s, &b, cmd)?;
    // Where the two axes come closest: the corner.
    let w = ax.centre - bx.centre;
    let cos = ax.dir.dot(bx.dir);
    let (d, e) = (ax.dir.dot(w), bx.dir.dot(w));
    let denominator = 1.0 - cos * cos;
    if denominator.abs() < 1e-6 {
        return Err(bad(cmd, "the two members are parallel: there is no corner to mitre"));
    }
    let sa = (cos * e - d) / denominator;
    let tb = (e - cos * d) / denominator;
    let corner = (ax.centre + ax.dir * sa + bx.centre + bx.dir * tb) * 0.5;
    let reach_a = (ax.t_max - ax.t_min) + (ax.section[1] - ax.section[0]) + (ax.section[3] - ax.section[2]);
    let reach_b = (bx.t_max - bx.t_min) + (bx.section[1] - bx.section[0]) + (bx.section[3] - bx.section[2]);
    if sa.abs() > reach_a || tb.abs() > reach_b {
        return Err(bad(cmd, "the two members do not meet: their axes come closest far from both"));
    }
    // From the corner along each member, into it.
    let into = |x: &Axis| if x.dir.dot(x.centre - corner) < 0.0 { -x.dir } else { x.dir };
    let (ua, ub) = (into(&ax), into(&bx));
    let n = (ua - ub).normalized().ok_or_else(|| bad(cmd, "the members run the same way: nothing to mitre"))?;
    let reach = reach_a.max(reach_b) + (corner - ax.centre).len() + (corner - bx.centre).len();
    // The first member keeps the side its axis comes from (n points along it), the second the other.
    cut_half_space(s, &a, corner, -n, reach)?;
    cut_half_space(s, &b, corner, n, reach)?;
    Ok(json!({"members": [a, b], "corner": arr(corner), "plane_normal": arr(n)}))
}

fn groove_weld(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.groove_weld";
    let pa =
        p.get("face_a").and_then(vec3).filter(|q| q.len() <= 1e6).ok_or_else(|| bad(cmd, "`face_a` must be a point [x, y, z] on the first face"))?;
    let pb =
        p.get("face_b").and_then(vec3).filter(|q| q.len() <= 1e6).ok_or_else(|| bad(cmd, "`face_b` must be a point [x, y, z] on the second face"))?;
    let st = s.world_state();
    let body_at = |key: &str, at: Vec3| -> Result<String> {
        match p.get(key) {
            Some(Value::String(n)) if st.body(n).is_some() => Ok(n.clone()),
            Some(Value::String(n)) => Err(bad(cmd, format!("no body `{n}`"))),
            None | Some(Value::Null) => st
                .bodies
                .iter()
                .find(|b| solidcraft_doc::appearance::face_index_at(b, at).is_some())
                .map(|b| b.name.clone())
                .ok_or_else(|| bad(cmd, format!("no body has a face at `{}`", key.replace("body", "face")))),
            Some(_) => Err(bad(cmd, format!("`{key}` must be a body name"))),
        }
    };
    let a = body_at("body_a", pa)?;
    let b = body_at("body_b", pb)?;
    let face_of = |body: &str, at: Vec3, key: &str| -> Result<(Vec3, Vec3, f64)> {
        let bd = st.body(body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
        let face = solidcraft_doc::appearance::face_index_at(bd, at).ok_or_else(|| bad(cmd, format!("`{body}` has no face at `{key}`")))?;
        let tol = (bd.body.size() * 2e-3).max(1e-3);
        let faces = bd.body.faces(tol)?;
        let info = faces.iter().find(|f| f.index == face).ok_or_else(|| bad(cmd, "no face"))?;
        let n = info.plane_normal.and_then(|n| n.normalized()).ok_or_else(|| bad(cmd, format!("`{key}` is not a flat face")))?;
        // The normal that points out of the material.
        let mesh = bd.mesh();
        let n = if mesh.inside_index().contains(at + n * 0.05) { -n } else { n };
        Ok((n, info.centroid, info.area))
    };
    let (na, ca, area) = face_of(&a, pa, "face_a")?;
    let (nb, cb, _) = face_of(&b, pb, "face_b")?;
    if (na.dot(nb).abs() - 1.0).abs() > 1e-3 {
        return Err(bad(cmd, "a groove weld needs two parallel faces"));
    }
    let gap = (cb - ca).dot(na);
    if gap <= 1e-6 {
        return Err(bad(
            cmd,
            if gap.abs() <= 1e-6 {
                "the faces touch: there is no gap to fill"
            } else {
                "the second face lies behind the first: pick the two faces that look at each other across the gap"
            },
        ));
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Groove weld {a}-{b}"));
    run(s, "solid.extrude", json!({"face": arr(pa), "face_body": a, "distance": gap, "operation": "new", "body_name": name}))?;
    Ok(json!({"body": name, "gap_mm": gap, "area_mm2": area, "volume_mm3": area * gap, "between": [a, b]}))
}

#[cfg(test)]
#[path = "frame_edit_tests.rs"]
mod tests;
