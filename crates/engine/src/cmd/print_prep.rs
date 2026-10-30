//! Getting a part ready for the printer, and taking material out of it: `print.elephant_foot`
//! (chamfer the first layer's edges), `print.hole_compensation` (enlarge the bores a printer
//! closes up), `print.split` (cut an oversize body in two with alignment pins),
//! `construct.topology_optimize` (keep the material that carries the load) and `plastic.grill`
//! (a vent of slots between ribs in a flat wall). Each composes the ordinary features (chamfer,
//! cut, split, extrude), so the result stays in the timeline and follows the part.

use serde_json::{Value, json};
use solidcraft_analysis::print_analysis::{self, TopologyMask};
use solidcraft_geom::{Mesh, MeshIndex, Vec3};
use solidcraft_kernel::FaceSurface;

use super::CommandSpec;
use super::design::{number, run};
use super::stress::{body_name, region};
use crate::params::{bad, bool_, str_, vec3};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("print.elephant_foot", "Elephant Foot", elephant_foot).at("3DEEE", "MAKE").icon("elephant_foot").params(
        "body?: name (default: the selected body, or the design's only one); up?: [x, y, z] the build direction (default +Z); distance_mm?: the \
         chamfer on the first-layer edges (0.05 to 5, default 0.3) → body, edges (how many were chamfered), distance_mm, up. The first layer is \
         squashed wider than drawn; chamfering the edges that sit on the plate undoes it, so the part stands on size",
    ),
    CommandSpec::new("print.hole_compensation", "Hole Compensation", hole_compensation).at("3DEEE", "MAKE").icon("hole_compensation").params(
        "body?: name; delta_mm?: how much wider each bore is cut, on the diameter (0.01 to 2, default 0.2); max_diameter_mm?: leave bores wider \
         than this alone (default 0: all) → body, bores (how many were enlarged), diameters_mm (as drawn), delta_mm. A printed bore comes out \
         smaller than drawn; each round bore (a cylindrical face with the void inside it) is cut wider by `delta_mm`",
    ),
    CommandSpec::new("print.split", "Split for Print", split).at("3DEEE", "MAKE").icon("print_split").params(
        "body?: name; axis?: X | Y | Z the cut plane's normal (default Z); station_mm?: where along it (default: the body's middle); \
         pin_diameter_mm? (0.5 to 50, default 4), pin_length_mm? (0.5 to 100, default 6), clearance_mm?: the socket over the pin (0 to 2, default \
         0.3); pins?: false leaves the halves plain → body, bodies: [lower, upper], lower, upper, axis, station_mm, pins (how many were set), \
         pin_diameter_mm, pin_length_mm, clearance_mm, warnings. Cuts a body too big for the printer in two and puts alignment pins on the lower \
         half with sockets in the upper, at the corners of the cut where both halves are solid",
    ),
    CommandSpec::new("construct.topology_optimize", "Topology Optimize", topology_optimize)
        .at("3DEEE", "GENERATIVE")
        .icon("topology_optimize")
        .params(
            "body?: name; fixed: the face the body is held at, load: the face the force acts on (a point, a face name or {face: index}); force_n: \
         [fx, fy, fz] newtons; target_fraction?: of the material to keep (0.05 to 0.95, default 0.5); resolution?: cells along the longest side \
         (6 to 48, default 24); name?: the new body's name (default: `<body> optimised`); hide?: true hides the source → body, kept_fraction, \
         voxels_total, voxels_kept, grid, cell_mm, advice, built. Solves the body's stress on a voxel grid and keeps the most-worked cells of \
         every section along the load path, always connected from the mount to the load; the result is a new mesh body",
        ),
    CommandSpec::new("plastic.grill", "Grill", grill).at("3DEEE", "PARTS").icon("grill").params(
        "face: [x, y, z] a point on the flat wall, the vent's centre; body?: name (default: the body with that face); direction?: [x, y, z] the \
         slots run along it, projected onto the wall (default +X); width_mm? (default 20) along the slots, length_mm? (default 12) across them; \
         rim_width_mm? (default 1.5), rim_height_mm? (default 0: no raised rim); slot_width_mm? (default 2), rib_width_mm? (default 1.5); \
         spar_width_mm?, spar_pitch_mm?: cross ribs square to the slots (default 0: none); depth_mm?: how deep the slots are cut (default: \
         through the wall under the face) → body, face, wall_mm, slots, rim, normal, direction. A vent through a flat wall: slots between ribs \
         inside a rim, cut from a sketch on the wall",
    ),
];

fn up_of(p: &Value, cmd: &str) -> Result<Vec3> {
    match p.get("up") {
        None | Some(Value::Null) => Ok(Vec3::Z),
        Some(v) => vec3(v).and_then(|u| u.normalized()).ok_or_else(|| bad(cmd, "`up` is the build direction [x, y, z], not zero")),
    }
}

fn arr(v: Vec3) -> Value {
    json!([v.x, v.y, v.z])
}

fn elephant_foot(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "print.elephant_foot";
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let up = up_of(p, cmd)?;
    let distance = number(cmd, p, "distance_mm", Some(0.3), 0.05..=5.0)?.unwrap_or(0.3);
    let b = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let tol = (b.body.size() * 2e-3).max(1e-3);
    let edges = b.body.edges(tol)?;
    let station = |q: &Vec3| q.dot(up);
    let bottom = edges.iter().flat_map(|e| e.points.iter()).map(station).fold(f64::INFINITY, f64::min);
    if !bottom.is_finite() {
        return Err(bad(cmd, "the body has no edges"));
    }
    let picks: Vec<Value> = edges
        .iter()
        .filter(|e| !e.points.is_empty() && e.points.iter().all(|q| station(q) <= bottom + 1e-3))
        .map(|e| json!({"body": name, "index": e.index}))
        .collect();
    if picks.is_empty() {
        return Err(bad(cmd, "no edges lie on the first layer: the body does not sit flat on the plate in this build direction"));
    }
    let count = picks.len();
    run(s, "solid.chamfer", json!({"edges": picks, "distance": distance, "body": name}))?;
    Ok(json!({"body": name, "edges": count, "distance_mm": distance, "up": arr(up)}))
}

/// A round bore: a cylindrical face (or several on one axis) with the void inside it.
struct Bore {
    origin: Vec3,
    axis: Vec3,
    radius: f64,
    lo: f64,
    hi: f64,
}

fn hole_compensation(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "print.hole_compensation";
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let delta = number(cmd, p, "delta_mm", Some(0.2), 0.01..=2.0)?.unwrap_or(0.2);
    let max_d = number(cmd, p, "max_diameter_mm", Some(0.0), 0.0..=10_000.0)?.unwrap_or(0.0);
    let b = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let mesh = b.mesh();
    let index = mesh.inside_index();
    let spots = solidcraft_doc::naming::face_spots(b);
    let mut bores: Vec<Bore> = Vec::new();
    for (face, spot) in spots.iter().enumerate() {
        let Some(spot) = spot else { continue };
        let Some(FaceSurface::Cylinder { origin, axis, radius }) = spot.surface else { continue };
        let Some(axis) = axis.normalized() else { continue };
        if !(radius > 0.0) || (max_d > 0.0 && 2.0 * radius > max_d + 1e-9) {
            continue;
        }
        // The void is inside a bore: a point just inside the surface, towards the axis, is empty.
        let rel = spot.point - origin;
        let Some(radial) = (rel - axis * rel.dot(axis)).normalized() else { continue };
        let probe = (0.2 * radius).clamp(0.02, 0.5);
        if index.contains(spot.point - radial * probe) {
            continue; // material inside: a boss or a shaft
        }
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for (t, f) in mesh.triangles.iter().zip(&mesh.tri_face) {
            if *f as usize != face {
                continue;
            }
            for i in t {
                if let Some(q) = mesh.positions.get(*i as usize) {
                    let a = (*q - origin).dot(axis);
                    lo = lo.min(a);
                    hi = hi.max(a);
                }
            }
        }
        if !(hi > lo) {
            continue;
        }
        let same = bores.iter_mut().find(|bb| {
            let d = origin - bb.origin;
            (bb.radius - radius).abs() < 1e-6 && bb.axis.cross(axis).len() < 1e-6 && (d - bb.axis * d.dot(bb.axis)).len() < 1e-6
        });
        match same {
            Some(bore) => {
                let shift = (origin - bore.origin).dot(bore.axis);
                let (glo, ghi) = if bore.axis.dot(axis) > 0.0 { (shift + lo, shift + hi) } else { (shift - hi, shift - lo) };
                bore.lo = bore.lo.min(glo);
                bore.hi = bore.hi.max(ghi);
            }
            None => bores.push(Bore { origin, axis, radius, lo, hi }),
        }
    }
    if bores.is_empty() {
        return Err(bad(cmd, "no round bores to enlarge: the body has no cylindrical face with the void inside it"));
    }
    let mut diameters = Vec::with_capacity(bores.len());
    for bore in &bores {
        // Past an open end, so the wider bore runs out cleanly.
        let probe = (bore.radius * 0.5).max(0.05);
        let (mut lo, mut hi) = (bore.lo, bore.hi);
        if !index.contains(bore.origin + bore.axis * (lo - probe)) {
            lo -= probe;
        }
        if !index.contains(bore.origin + bore.axis * (hi + probe)) {
            hi += probe;
        }
        let base = bore.origin + bore.axis * lo;
        run(
            s,
            "solid.cylinder",
            json!({"radius": bore.radius + delta / 2.0, "height": hi - lo, "base": arr(base), "axis": arr(bore.axis), "operation": "cut", "targets": [name]}),
        )?;
        diameters.push(2.0 * bore.radius);
    }
    Ok(json!({"body": name, "bores": diameters.len(), "diameters_mm": diameters, "delta_mm": delta}))
}

fn component(v: Vec3, axis: usize) -> f64 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

fn split(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "print.split";
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let (axis, axis_name) = match str_(p, "axis").map(|a| a.trim().to_ascii_uppercase()).as_deref() {
        None | Some("") | Some("Z") => (2usize, "Z"),
        Some("X") => (0, "X"),
        Some("Y") => (1, "Y"),
        Some(_) => return Err(bad(cmd, "`axis` is X, Y or Z")),
    };
    let dir = [Vec3::X, Vec3::Y, Vec3::Z][axis];
    let b = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let mesh = b.mesh();
    let bounds = mesh.bounds();
    let (lo_b, hi_b) = (component(bounds.min, axis), component(bounds.max, axis));
    if !(hi_b > lo_b + 1e-6) {
        return Err(bad(cmd, "the body has no extent along that axis"));
    }
    let middle = (lo_b + hi_b) / 2.0;
    let station = number(cmd, p, "station_mm", Some(middle), -1e6..=1e6)?.unwrap_or(middle);
    if station <= lo_b + 1e-3 || station >= hi_b - 1e-3 {
        return Err(bad(cmd, format!("the cut plane misses the body: `station_mm` must lie between {lo_b:.2} and {hi_b:.2}")));
    }
    let pin_d = number(cmd, p, "pin_diameter_mm", Some(4.0), 0.5..=50.0)?.unwrap_or(4.0);
    let pin_l = number(cmd, p, "pin_length_mm", Some(6.0), 0.5..=100.0)?.unwrap_or(6.0);
    let clearance = number(cmd, p, "clearance_mm", Some(0.3), 0.0..=2.0)?.unwrap_or(0.3);
    let want_pins = bool_(p, "pins").unwrap_or(true);
    // The cut section's bounds, from the mesh edges that cross the plane.
    let (u_axis, v_axis) = (if axis == 0 { 1 } else { 0 }, if axis == 2 { 1 } else { 2 });
    let (mut s_min, mut s_max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for t in &mesh.triangles {
        let Some([a, b, c]) = mesh.tri(t) else { continue };
        for (p0, p1) in [(a, b), (b, c), (c, a)] {
            let (s0, s1) = (component(p0, axis) - station, component(p1, axis) - station);
            if s0 * s1 > 0.0 || (s0 - s1).abs() < 1e-12 {
                continue;
            }
            let q = p0 + (p1 - p0) * (s0 / (s0 - s1));
            for k in 0..3 {
                s_min[k] = s_min[k].min(component(q, k));
                s_max[k] = s_max[k].max(component(q, k));
            }
        }
    }
    if !(s_max[u_axis] > s_min[u_axis]) || !(s_max[v_axis] > s_min[v_axis]) {
        return Err(bad(cmd, "the cut plane meets no material"));
    }
    let index = mesh.inside_index();
    let mut origin = Vec3::ZERO;
    match axis {
        0 => origin.x = station,
        1 => origin.y = station,
        _ => origin.z = station,
    }
    run(s, "solid.split_body", json!({"body": name, "plane": {"origin": arr(origin), "normal": arr(dir)}}))?;
    let other = format!("{name} (1)");
    let after = s.world_state();
    let centre = |n: &str| after.body(n).map(|b| b.mesh().bounds()).map(|bb| (component(bb.min, axis) + component(bb.max, axis)) / 2.0);
    let (lower, upper) = match (centre(&name), centre(&other)) {
        (Some(a), Some(b)) if a < b => (name.clone(), other),
        (Some(_), Some(_)) => (other, name.clone()),
        _ => return Err(bad(cmd, "the cut did not leave two bodies")),
    };
    let mut pins = 0usize;
    let mut warnings: Vec<String> = Vec::new();
    if want_pins {
        let inset_u = pin_d.max(0.15 * (s_max[u_axis] - s_min[u_axis]));
        let inset_v = pin_d.max(0.15 * (s_max[v_axis] - s_min[v_axis]));
        let probe = pin_l.max(2.0 * pin_d);
        let mut sites = Vec::new();
        for u in [s_min[u_axis] + inset_u, s_max[u_axis] - inset_u] {
            for v in [s_min[v_axis] + inset_v, s_max[v_axis] - inset_v] {
                let mut at = [0.0; 3];
                at[axis] = station;
                at[u_axis] = u;
                at[v_axis] = v;
                let at = Vec3::new(at[0], at[1], at[2]);
                if sites.iter().any(|q: &Vec3| (*q - at).len() < pin_d) {
                    continue;
                }
                if index.contains(at - dir * probe) && index.contains(at + dir * probe) {
                    sites.push(at);
                }
            }
        }
        for at in &sites {
            run(
                s,
                "solid.cylinder",
                json!({"radius": pin_d / 2.0, "height": pin_l, "base": arr(*at), "axis": arr(dir), "operation": "join", "targets": [lower]}),
            )?;
            run(
                s,
                "solid.cylinder",
                json!({"radius": (pin_d + clearance) / 2.0, "height": pin_l + clearance, "base": arr(*at), "axis": arr(dir), "operation": "cut", "targets": [upper]}),
            )?;
            pins += 1;
        }
        if pins == 0 {
            warnings.push("no spot for alignment pins: the halves are not solid on both sides at the corners of the cut".into());
        }
    }
    Ok(json!({
        "body": name,
        "bodies": [lower, upper],
        "lower": lower,
        "upper": upper,
        "axis": axis_name,
        "station_mm": station,
        "pins": pins,
        "pin_diameter_mm": pin_d,
        "pin_length_mm": pin_l,
        "clearance_mm": clearance,
        "warnings": warnings,
    }))
}

/// The kept cells as a smooth surface.
fn mask_mesh(mask: &TopologyMask) -> Option<Mesh> {
    if mask.kept.len() != mask.grid[0].checked_mul(mask.grid[1])?.checked_mul(mask.grid[2])? {
        return None;
    }
    solidcraft_analysis::implicit::cells_mesh(mask.grid, mask.origin * 1000.0, mask.h * 1000.0, |v| mask.kept.get(v).copied().unwrap_or(false))
}

fn topology_optimize(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "construct.topology_optimize";
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let fixed = p.get("fixed").ok_or_else(|| bad(cmd, "give `fixed`: the face the body is held at"))?;
    let load = p.get("load").ok_or_else(|| bad(cmd, "give `load`: the face the force acts on"))?;
    let fixed = region(s, fixed, &name, cmd)?;
    let load = region(s, load, &name, cmd)?;
    let force = p
        .get("force_n")
        .and_then(vec3)
        .filter(|f| f.len() > 0.0 && f.len() <= 1e9)
        .ok_or_else(|| bad(cmd, "`force_n` must be [fx, fy, fz] newtons, not zero and at most 1e9 N in all"))?;
    let fraction = number(cmd, p, "target_fraction", Some(0.5), 0.05..=0.95)?.unwrap_or(0.5);
    let resolution = number(cmd, p, "resolution", Some(24.0), 6.0..=48.0)?.unwrap_or(24.0) as usize;
    let b = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let mesh = b.mesh();
    let mask = print_analysis::topology_optimize(&mesh, &fixed, &load, force, fraction, resolution)
        .ok_or_else(|| bad(cmd, "the body could not be optimised: check that it is closed and that the fixed and load faces hit material"))?;
    let out = mask_mesh(&mask).ok_or_else(|| bad(cmd, "nothing was kept"))?;
    if out.triangles.is_empty() {
        return Err(bad(cmd, "the kept material is too thin to make a body: raise the target fraction or the resolution"));
    }
    let kit = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{name} optimised"));
    let built = super::design::add_mesh_body(s, &kit, out, cmd)?;
    if bool_(p, "hide").unwrap_or(false) {
        let _ = s.execute("browser.visibility", &json!({"items": [name], "visible": false}));
    }
    Ok(json!({
        "body": name,
        "kept_fraction": mask.kept_fraction,
        "voxels_total": mask.voxels_total,
        "voxels_kept": mask.voxels_kept,
        "grid": mask.grid,
        "cell_mm": mask.h * 1000.0,
        "advice": mask.advice,
        "built": built,
    }))
}

fn grill(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "plastic.grill";
    let face_pt =
        p.get("face").and_then(vec3).filter(|q| q.len() <= 1e6).ok_or_else(|| bad(cmd, "`face` must be a point [x, y, z] on the flat wall"))?;
    let st = s.world_state();
    let name = match p.get("body") {
        None | Some(Value::Null) => st
            .bodies
            .iter()
            .find(|b| solidcraft_doc::appearance::face_index_at(b, face_pt).is_some())
            .map(|b| b.name.clone())
            .ok_or_else(|| bad(cmd, "no body has a face at `face`"))?,
        Some(Value::String(n)) => n.clone(),
        Some(_) => return Err(bad(cmd, "`body` must be a body name")),
    };
    let b = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let face = solidcraft_doc::appearance::face_index_at(b, face_pt).ok_or_else(|| bad(cmd, format!("`{name}` has no face at `face`")))?;
    let tol = (b.body.size() * 2e-3).max(1e-3);
    let faces = b.body.faces(tol)?;
    let n = faces
        .iter()
        .find(|f| f.index == face)
        .and_then(|f| f.plane_normal)
        .and_then(|n| n.normalized())
        .ok_or_else(|| bad(cmd, "a grill goes in a flat wall"))?;
    let mesh = b.mesh();
    let index = mesh.inside_index();
    // The normal that points out of the material.
    let n = if index.contains(face_pt + n * 0.05) { -n } else { n };
    let bvh = MeshIndex::new(&mesh);
    let wall =
        bvh.raycast(&mesh, face_pt - n * 1e-4, -n).map(|(d, _)| d + 1e-4).filter(|d| *d > 1e-6).ok_or_else(|| bad(cmd, "no wall under the face"))?;
    let wanted = p.get("direction").and_then(vec3).unwrap_or(Vec3::X);
    let x = (wanted - n * wanted.dot(n)).normalized().or_else(|| (Vec3::X - n * n.x).normalized()).or_else(|| (Vec3::Y - n * n.y).normalized());
    let x = x.ok_or_else(|| bad(cmd, "`direction` must not be along the wall's normal"))?;
    let y = n.cross(x);
    let width = number(cmd, p, "width_mm", Some(20.0), 0.5..=5000.0)?.unwrap_or(20.0);
    let length = number(cmd, p, "length_mm", Some(12.0), 0.5..=5000.0)?.unwrap_or(12.0);
    let rim_w = number(cmd, p, "rim_width_mm", Some(1.5), 0.0..=500.0)?.unwrap_or(1.5);
    let rim_h = number(cmd, p, "rim_height_mm", Some(0.0), 0.0..=500.0)?.unwrap_or(0.0);
    let slot = number(cmd, p, "slot_width_mm", Some(2.0), 0.1..=500.0)?.unwrap_or(2.0);
    let rib = number(cmd, p, "rib_width_mm", Some(1.5), 0.1..=500.0)?.unwrap_or(1.5);
    let spar_w = number(cmd, p, "spar_width_mm", Some(0.0), 0.0..=500.0)?.unwrap_or(0.0);
    let spar_p = number(cmd, p, "spar_pitch_mm", Some(0.0), 0.0..=5000.0)?.unwrap_or(0.0);
    let depth = number(cmd, p, "depth_mm", Some(wall + 0.05), 0.01..=10_000.0)?.unwrap_or(wall + 0.05);
    let (hw, hl) = (width / 2.0 - rim_w, length / 2.0 - rim_w);
    if hw <= 1e-6 || hl <= slot / 2.0 {
        return Err(bad(cmd, "the boundary is too small for one slot inside the rim"));
    }
    // Slots along x at a pitch across y, centred on the region; spars split them along x.
    let pitch = slot + rib;
    let count = ((2.0 * hl + rib) / pitch).floor() as i64 + 1;
    let mut u_ranges: Vec<(f64, f64)> = vec![(-hw, hw)];
    if spar_w > 0.0 && spar_p > spar_w {
        let m = (hw / spar_p).floor() as i64 + 1;
        let mut gaps: Vec<(f64, f64)> = (-m..=m).map(|q| (q as f64 * spar_p - spar_w / 2.0, q as f64 * spar_p + spar_w / 2.0)).collect();
        gaps.sort_by(|a, b| a.0.total_cmp(&b.0));
        u_ranges.clear();
        let mut start = -hw;
        for (g0, g1) in gaps {
            if g1 <= -hw || g0 >= hw {
                continue;
            }
            if g0 > start {
                u_ranges.push((start, g0.min(hw)));
            }
            start = start.max(g1);
        }
        if start < hw {
            u_ranges.push((start, hw));
        }
    }
    let mut rects: Vec<([f64; 2], [f64; 2])> = Vec::new();
    for k in -count..=count {
        let v0 = (k as f64 * pitch - slot / 2.0).max(-hl);
        let v1 = (k as f64 * pitch + slot / 2.0).min(hl);
        if v1 - v0 < 1e-6 {
            continue;
        }
        for (u0, u1) in &u_ranges {
            if u1 - u0 > 1e-6 {
                rects.push(([*u0, v0], [*u1, v1]));
            }
        }
    }
    if rects.is_empty() {
        return Err(bad(cmd, "the boundary is too small for one slot"));
    }
    let plane = json!({"origin": arr(face_pt), "x_dir": arr(x), "y_dir": arr(y)});
    run(s, "sketch.create", json!({"plane": plane}))?;
    for (p0, p1) in &rects {
        run(s, "sketch.rectangle.two_point", json!({"p0": p0, "p1": p1}))?;
    }
    run(s, "sketch.finish", json!({}))?;
    run(s, "solid.extrude", json!({"distance": depth, "operation": "cut", "targets": [name], "direction": "negative", "profiles": "all"}))?;
    let rim = rim_h > 0.0 && rim_w > 0.0;
    if rim {
        run(s, "sketch.create", json!({"plane": plane}))?;
        run(s, "sketch.rectangle.two_point", json!({"p0": [-width / 2.0, -length / 2.0], "p1": [width / 2.0, length / 2.0]}))?;
        run(s, "sketch.rectangle.two_point", json!({"p0": [-hw, -hl], "p1": [hw, hl]}))?;
        run(s, "sketch.finish", json!({}))?;
        run(
            s,
            "solid.extrude",
            json!({"distance": rim_h, "operation": "join", "targets": [name], "profiles": [{"point": [width / 2.0 - rim_w / 2.0, 0.0]}]}),
        )?;
    }
    Ok(json!({"body": name, "face": face, "wall_mm": wall, "slots": rects.len(), "rim": rim, "normal": arr(n), "direction": arr(x)}))
}

#[cfg(test)]
#[path = "print_prep_tests.rs"]
mod tests;
