//! The bamboo workshop's hardware: `design.bamboo_connector` (a Muffe that several culms are
//! glued into), `design.bamboo_frame` (a bicycle or trailer skeleton built as culms, tubes and
//! connectors with a cut list), `design.bearing` (a catalogue bearing drawn to its boundary
//! dimensions) and `design.damper` (a shock drawn to its mounting dimensions). Each is a
//! component of ordinary features, so it measures, exports and takes the checks.

use std::f64::consts::PI;

use serde_json::{Value, json};
use solidcraft_analysis::bearing::{BearingSize, SIZES, find_bearing};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use super::design::{in_component, number, run};
use crate::params::{bad, bool_, str_, vec3};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("design.bamboo_connector", "Bamboo Connector", connector).at("3DEEE", "PARTS").icon("bamboo_connector").params(
        "arms: [{direction: [x, y, z], diameter_mm: the culm's outside diameter, engagement_mm?: how far it goes in (default twice its \
         diameter)}] one socket per culm (1 to 12); glue_gap_mm?: radial, all round (default 0.3); wall_mm?: the sleeve (default 3); \
         hub_diameter_mm?: the hub (default: as the fattest sleeve); name? → component, occurrence, name, arms, hub_diameter_mm, \
         bond_area_mm2 (the culms' surface in the glue, all arms), bores_mm. A Muffe: a hub with a sleeve per culm; the bores are cut last and \
         run in by exactly the engagement, so a culm bottoms out on hub material",
    ),
    CommandSpec::new("design.bamboo_frame", "Bamboo Frame", frame).at("3DEEE", "PARTS").icon("bamboo_frame").params(
        "kind: bicycle | trailer; bicycle: seat_tube_mm (540), seat_angle_deg (73.5), top_tube_mm (560, horizontal), top_tube_drop_mm (0), \
         head_tube_mm (130), head_angle_deg (71), chain_stay_mm (430), bb_drop_mm (68), dropout_spacing_mm (135), main_diameter_mm (34), \
         top_tube_diameter_mm (default main), stay_diameter_mm (default 0.65 × main), head_tube_diameter_mm (36, metal); trailer: length_mm \
         (900), width_mm (600), cross_members (2), drawbar_mm (700), drawbar_rise_mm (150), drawbar_offset_mm (0), rail_diameter_mm (32), \
         drawbar_diameter_mm (38); both: culm_wall_mm (8), engagement (culm diameters into each socket, default 2), glue_gap_mm (0.3), \
         socket_wall_mm (3), node_spacing_mm (220; 0 for plain tubes), name? → component, occurrence, name, kind, members: [{name, from, to, \
         axis_mm, cut_mm, diameter_mm, material}], junctions, bond_area_mm2, total_cut_mm, bodies. Origin at the bottom bracket (bicycle: X \
         forward, Z up, Y to the rider's right) or the middle of the axle line (trailer). Chain stay length is centre-plane: the stay itself \
         runs out to a dropout half the hub width away, so it is a little longer than the figure",
    ),
    CommandSpec::new("design.bearing", "Bearing", bearing).at("3DEEE", "DRIVE").icon("bearing").params(
        "designation: 625, 608, 6000…6003, 6200…6204, 6802, 6803, 6805, 6806, 6900…6903, Headset 41.8 | 41.0 | 51.8; center?: [x, y, z] (the \
         bore's axis runs along +Z from here); name? → component, occurrence, name, designation, bore_mm, outer_mm, width_mm, dynamic_rating_n, \
         static_rating_n, used_for. The inner and outer rings to the catalogue's boundary dimensions, with a torus standing for the balls",
    ),
    CommandSpec::new("design.damper", "Damper", damper).at("3DEEE", "DRIVE").icon("damper").params(
        "eye_to_eye_mm (165, extended), stroke_mm (40), body_diameter_mm (32), rod_diameter_mm (12), eye_bore_mm (8), eye_width_mm (12), \
         coil_over?: bool (default false), coil_diameter_mm (default 1.6 × body), wire_diameter_mm (4), spring_rate_n_mm? (30), \
         damping_ns_m? (600) — the two ratings are carried in the reply for inspect.damping; name? → component, occurrence, name, \
         eye_to_eye_mm, compressed_mm, stroke_mm, spring_rate_n_mm, damping_ns_m. Drawn along +Z with the lower eye at the origin, to its \
         mounting dimensions, which is what has to fit",
    ),
];

/// The hub radius for sleeves whose fattest is `widest`: well clear of it, so the sleeves meet
/// the sphere at a steep angle rather than grazing it.
fn hub_of(widest: f64) -> f64 {
    (widest * 1.3).max(widest + 3.0)
}

/// How far outside the hub's surface a bore bottoms (mm), so its end disc clears the sphere and
/// the cut meets only the sleeve.
const BORE_OUTSET: f64 = 0.5;

fn arr(v: Vec3) -> Value {
    json!([v.x, v.y, v.z])
}

struct Arm {
    direction: Vec3,
    diameter: f64,
    engagement: f64,
}

/// Builds a connector at `at`: a hub sphere and, as a body of its own, a sleeve per arm with its
/// bore cut coaxially. Sleeves that leave close together simply overlap, which is what keeps the
/// kernel out of shallow-angle booleans. Returns the hub radius, the bond area and the bodies.
fn build_connector(s: &mut Session, name: &str, at: Vec3, arms: &[Arm], gap: f64, wall: f64, hub_diameter: f64) -> Result<(f64, f64, Vec<String>)> {
    let widest = arms.iter().map(|a| a.diameter / 2.0 + gap + wall).fold(0.0, f64::max);
    let hub = if hub_diameter > 0.0 { hub_diameter / 2.0 } else { hub_of(widest) };
    run(s, "solid.sphere", json!({"radius": hub, "center": arr(at), "operation": "new", "body_name": name}))?;
    let mut bodies = vec![name.to_string()];
    let mut bond = 0.0;
    for (k, a) in arms.iter().enumerate() {
        let sleeve = format!("{name} socket {}", k + 1);
        let outer = a.diameter / 2.0 + gap + wall;
        let bore = a.diameter / 2.0 + gap;
        // From well inside the hub to the mouth; the bore bottoms just outside the hub's surface.
        let start = at + a.direction * (hub * 0.5);
        run(
            s,
            "solid.cylinder",
            json!({"radius": outer, "height": hub * 0.5 + a.engagement, "base": arr(start), "axis": arr(a.direction), "operation": "new", "body_name": sleeve}),
        )?;
        let base = at + a.direction * (hub + BORE_OUTSET);
        run(
            s,
            "solid.cylinder",
            json!({"radius": bore, "height": a.engagement - BORE_OUTSET + 0.01, "base": arr(base), "axis": arr(a.direction), "operation": "cut", "targets": [sleeve]}),
        )?;
        bond += PI * a.diameter * a.engagement;
        bodies.push(sleeve);
    }
    Ok((hub, bond, bodies))
}

fn connector(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.bamboo_connector";
    let list = p.get("arms").and_then(Value::as_array).ok_or_else(|| bad(cmd, "`arms` lists {direction, diameter_mm, engagement_mm?} per socket"))?;
    if list.is_empty() || list.len() > 12 {
        return Err(bad(cmd, "`arms` holds 1 to 12 sockets"));
    }
    let gap = number(cmd, p, "glue_gap_mm", Some(0.3), 0.0..=5.0)?.unwrap_or(0.3);
    let wall = number(cmd, p, "wall_mm", Some(3.0), 0.5..=50.0)?.unwrap_or(3.0);
    let hub_diameter = number(cmd, p, "hub_diameter_mm", Some(0.0), 0.0..=1000.0)?.unwrap_or(0.0);
    let mut arms = Vec::with_capacity(list.len());
    for (i, a) in list.iter().enumerate() {
        let direction = a
            .get("direction")
            .and_then(vec3)
            .and_then(|d| d.normalized())
            .ok_or_else(|| bad(cmd, format!("socket {} needs `direction`: [x, y, z], not zero", i + 1)))?;
        let diameter =
            number(cmd, a, "diameter_mm", None, 5.0..=500.0)?.ok_or_else(|| bad(cmd, format!("socket {} needs `diameter_mm` (5 to 500)", i + 1)))?;
        let engagement = number(cmd, a, "engagement_mm", Some(2.0 * diameter), 1.0..=2000.0)?.unwrap_or(2.0 * diameter);
        arms.push(Arm { direction, diameter, engagement });
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Bamboo connector {} sockets", arms.len()));
    let body = name.clone();
    in_component(s, &name, |s| {
        let (hub, bond, bodies) = build_connector(s, &body, Vec3::ZERO, &arms, gap, wall, hub_diameter)?;
        Ok(json!({
            "bodies": bodies,
            "arms": arms.len(),
            "hub_diameter_mm": 2.0 * hub,
            "bond_area_mm2": bond,
            "bores_mm": arms.iter().map(|a| a.diameter + 2.0 * gap).collect::<Vec<_>>(),
            "glue_gap_mm": gap,
            "wall_mm": wall,
        }))
    })
}

struct Segment {
    name: String,
    from: Vec3,
    to: Vec3,
    diameter: f64,
    wall: f64,
    tube: bool,
}

/// A perpendicular to `d`.
fn perpendicular(d: Vec3) -> Vec3 {
    let helper = if d.z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
    (helper - d * helper.dot(d)).normalized().unwrap_or(Vec3::X)
}

/// The half-profile of a culm (radius, along) with collars outside and diaphragms inside.
fn culm_profile(length: f64, butt: f64, tip: f64, butt_wall: f64, tip_wall: f64, spacing: f64, first: f64) -> Vec<[f64; 2]> {
    let outer_at = |t: f64| butt / 2.0 + (tip / 2.0 - butt / 2.0) * t / length;
    let bore_at = |t: f64| outer_at(t) - (butt_wall + (tip_wall - butt_wall) * t / length);
    let mut nodes = Vec::new();
    if spacing > 0.0 {
        let mut t = first;
        while t < length - 1e-9 && nodes.len() < 1000 {
            if t > 1e-9 {
                nodes.push(t);
            }
            t += spacing;
        }
    }
    let (collar, half, diaphragm) = (1.5, 4.0, 2.0);
    let mut pts: Vec<[f64; 2]> = vec![[bore_at(0.0), 0.0], [outer_at(0.0), 0.0]];
    for &t in &nodes {
        if t - half > 0.0 && t + half < length {
            pts.push([outer_at(t - half), t - half]);
            pts.push([outer_at(t) + collar, t]);
            pts.push([outer_at(t + half), t + half]);
        }
    }
    pts.push([outer_at(length), length]);
    pts.push([bore_at(length), length]);
    for &t in nodes.iter().rev() {
        if t - diaphragm > 0.0 && t + diaphragm < length {
            pts.push([bore_at(t + diaphragm), t + diaphragm]);
            pts.push([0.0, t + diaphragm]);
            pts.push([0.0, t - diaphragm]);
            pts.push([bore_at(t - diaphragm), t - diaphragm]);
        }
    }
    pts
}

/// A member revolved about its own axis from `seat` along `d` for `length`.
fn build_member(s: &mut Session, name: &str, seat: Vec3, d: Vec3, length: f64, seg: &Segment, spacing: f64) -> Result<()> {
    let x = perpendicular(d);
    let profile = if seg.tube {
        let (ro, ri) = (seg.diameter / 2.0, seg.diameter / 2.0 - seg.wall);
        vec![[ri, 0.0], [ro, 0.0], [ro, length], [ri, length]]
    } else {
        let tip = (seg.diameter - length / 400.0).max(seg.diameter * 0.8);
        culm_profile(length, seg.diameter, tip, seg.wall, seg.wall * 0.92, spacing, (spacing / 2.0).min(length / 4.0))
    };
    run(s, "sketch.create", json!({"plane": {"origin": arr(seat), "x_dir": arr(x), "y_dir": arr(d)}}))?;
    run(s, "sketch.line", json!({"points": profile, "closed": true}))?;
    run(s, "sketch.finish", json!({}))?;
    run(s, "solid.revolve", json!({"axis": "y", "operation": "new", "body_name": name}))?;
    Ok(())
}

fn frame(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.bamboo_frame";
    let kind = str_(p, "kind").map(|k| k.trim().to_ascii_lowercase()).ok_or_else(|| bad(cmd, "give `kind`: bicycle or trailer"))?;
    let culm_wall = number(cmd, p, "culm_wall_mm", Some(8.0), 1.0..=50.0)?.unwrap_or(8.0);
    let engagement = number(cmd, p, "engagement", Some(2.0), 0.5..=5.0)?.unwrap_or(2.0);
    let gap = number(cmd, p, "glue_gap_mm", Some(0.3), 0.0..=5.0)?.unwrap_or(0.3);
    let socket_wall = number(cmd, p, "socket_wall_mm", Some(3.0), 0.5..=50.0)?.unwrap_or(3.0);
    let spacing = number(cmd, p, "node_spacing_mm", Some(220.0), 0.0..=5000.0)?.unwrap_or(220.0);
    if spacing > 0.0 && spacing < 20.0 {
        return Err(bad(cmd, "`node_spacing_mm` is 0 for plain tubes, or at least 20 mm"));
    }
    let n = |key: &str, default: f64, lo: f64, hi: f64| number(cmd, p, key, Some(default), lo..=hi).map(|v| v.unwrap_or(default));
    let mut segments: Vec<Segment> = Vec::new();
    let mut add = |name: &str, from: Vec3, to: Vec3, diameter: f64, wall: f64, tube: bool| {
        segments.push(Segment { name: name.into(), from, to, diameter, wall, tube });
    };
    match kind.as_str() {
        "bicycle" | "bike" => {
            let seat_tube = n("seat_tube_mm", 540.0, 200.0, 1000.0)?;
            let seat_angle = n("seat_angle_deg", 73.5, 60.0, 85.0)?;
            let top_tube = n("top_tube_mm", 560.0, 200.0, 1000.0)?;
            let drop = n("top_tube_drop_mm", 0.0, -200.0, 300.0)?;
            let head_tube = n("head_tube_mm", 130.0, 50.0, 400.0)?;
            let head_angle = n("head_angle_deg", 71.0, 60.0, 85.0)?;
            let chain_stay = n("chain_stay_mm", 430.0, 200.0, 800.0)?;
            let bb_drop = n("bb_drop_mm", 68.0, -50.0, 150.0)?;
            let dropouts = n("dropout_spacing_mm", 135.0, 50.0, 300.0)?;
            let main = n("main_diameter_mm", 34.0, 15.0, 100.0)?;
            let top_d = n("top_tube_diameter_mm", main, 15.0, 100.0)?;
            let stay_d = n("stay_diameter_mm", (main * 0.65).round(), 10.0, 100.0)?;
            let head_d = n("head_tube_diameter_mm", 36.0, 20.0, 100.0)?;
            if chain_stay <= bb_drop.abs() {
                return Err(bad(cmd, "the chain stay is shorter than the bottom bracket drop, so the rear wheel would have to be inside the frame"));
            }
            let (seat, head) = (seat_angle.to_radians(), head_angle.to_radians());
            let bb = Vec3::ZERO;
            let seat_top = Vec3::new(-seat_tube * seat.cos(), 0.0, seat_tube * seat.sin());
            let head_top = Vec3::new(seat_top.x + top_tube, 0.0, seat_top.z - drop);
            let head_bottom = Vec3::new(head_top.x + head_tube * head.cos(), 0.0, head_top.z - head_tube * head.sin());
            let behind = (chain_stay * chain_stay - bb_drop * bb_drop).sqrt();
            let half = dropouts / 2.0;
            let rear_left = Vec3::new(-behind, -half, bb_drop);
            let rear_right = Vec3::new(-behind, half, bb_drop);
            add("down tube", bb, head_bottom, main, culm_wall, false);
            add("seat tube", bb, seat_top, main, culm_wall, false);
            add("top tube", seat_top, head_top, top_d, culm_wall, false);
            add("head tube", head_bottom, head_top, head_d, 2.0, true);
            add("chain stay (left)", bb, rear_left, stay_d, culm_wall * 0.75, false);
            add("chain stay (right)", bb, rear_right, stay_d, culm_wall * 0.75, false);
            add("seat stay (left)", seat_top, rear_left, stay_d, culm_wall * 0.75, false);
            add("seat stay (right)", seat_top, rear_right, stay_d, culm_wall * 0.75, false);
        }
        "trailer" => {
            let length = n("length_mm", 900.0, 200.0, 5000.0)?;
            let width = n("width_mm", 600.0, 100.0, 3000.0)?;
            let cross = n("cross_members", 2.0, 0.0, 8.0)?.round() as usize;
            let drawbar = n("drawbar_mm", 700.0, 100.0, 3000.0)?;
            let rise = n("drawbar_rise_mm", 150.0, -500.0, 1000.0)?;
            let offset = n("drawbar_offset_mm", 0.0, -1500.0, 1500.0)?;
            let rail = n("rail_diameter_mm", 32.0, 10.0, 100.0)?;
            let drawbar_d = n("drawbar_diameter_mm", 38.0, 10.0, 100.0)?;
            let (hl, hw) = (length / 2.0, width / 2.0);
            let (fl, fr, bl, br) = (Vec3::new(hl, -hw, 0.0), Vec3::new(hl, hw, 0.0), Vec3::new(-hl, -hw, 0.0), Vec3::new(-hl, hw, 0.0));
            add("front rail", fl, fr, rail, culm_wall, false);
            add("back rail", bl, br, rail, culm_wall, false);
            add("left rail", fl, bl, rail, culm_wall, false);
            add("right rail", fr, br, rail, culm_wall, false);
            for i in 1..=cross {
                let x = hl - length * i as f64 / (cross + 1) as f64;
                add(&format!("cross member {i}"), Vec3::new(x, -hw, 0.0), Vec3::new(x, hw, 0.0), rail, culm_wall, false);
            }
            let hitch = Vec3::new(hl + drawbar, -offset, rise);
            add("drawbar", fl, hitch, drawbar_d, culm_wall, false);
            add("drawbar brace", fr, hitch, rail, culm_wall, false);
        }
        _ => return Err(bad(cmd, "`kind` is bicycle or trailer")),
    }
    // Junctions: member ends sharing a point; the hub there is as fat as its fattest sleeve.
    let mut junctions: Vec<(Vec3, Vec<(usize, bool)>)> = Vec::new();
    for (i, seg) in segments.iter().enumerate() {
        for (pt, starts) in [(seg.from, true), (seg.to, false)] {
            match junctions.iter_mut().find(|(q, _)| (*q - pt).len() <= 0.01) {
                Some((_, arms)) => arms.push((i, starts)),
                None => junctions.push((pt, vec![(i, starts)])),
            }
        }
    }
    let hub_radius: Vec<f64> = junctions
        .iter()
        .map(|(_, arms)| arms.iter().map(|(i, _)| segments[*i].diameter / 2.0 + gap + socket_wall).fold(0.0, f64::max))
        .map(hub_of)
        .collect();
    let junction_of = |pt: Vec3| junctions.iter().position(|(q, _)| (*q - pt).len() <= 0.01);
    let mut members_out = Vec::new();
    let mut total_cut = 0.0;
    for seg in &segments {
        let axis = (seg.to - seg.from).len();
        let (Some(a), Some(b)) = (junction_of(seg.from), junction_of(seg.to)) else { return Err(bad(cmd, "a member lost its junction")) };
        let cut = axis - hub_radius[a] - hub_radius[b];
        if !(cut > 0.0) {
            return Err(bad(cmd, format!("'{}' is shorter than the two connectors it has to reach between: there is nothing left to cut", seg.name)));
        }
        total_cut += cut;
        members_out.push(json!({"name": seg.name, "from": arr(seg.from), "to": arr(seg.to), "axis_mm": axis, "cut_mm": cut, "diameter_mm": seg.diameter, "material": if seg.tube { "tube" } else { "culm" }}));
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Bamboo {kind} frame"));
    let kind_name = kind.clone();
    in_component(s, &name, |s| {
        let mut bodies = Vec::new();
        for seg in &segments {
            let d = (seg.to - seg.from).normalized().ok_or_else(|| bad(cmd, "a member of no length is not a member"))?;
            let (Some(a), Some(b)) = (junction_of(seg.from), junction_of(seg.to)) else { return Err(bad(cmd, "a member lost its junction")) };
            let cut = (seg.to - seg.from).len() - hub_radius[a] - hub_radius[b];
            let seat = seg.from + d * hub_radius[a];
            let body = format!("{name} {}", seg.name);
            build_member(s, &body, seat, d, cut, seg, if seg.tube { 0.0 } else { spacing })?;
            bodies.push(body);
        }
        let mut bond = 0.0;
        for (j, (at, arms)) in junctions.iter().enumerate() {
            let list: Vec<Arm> = arms
                .iter()
                .filter_map(|(i, starts)| {
                    let seg = &segments[*i];
                    let along = if *starts { seg.to - seg.from } else { seg.from - seg.to };
                    along.normalized().map(|direction| Arm { direction, diameter: seg.diameter, engagement: engagement * seg.diameter })
                })
                .collect();
            let body = format!("{name} joint {}", j + 1);
            let (_, area, made) = build_connector(s, &body, *at, &list, gap, socket_wall, 2.0 * hub_radius[j])?;
            bond += area;
            bodies.extend(made);
        }
        Ok(json!({
            "kind": kind_name,
            "members": members_out,
            "junctions": junctions.len(),
            "bond_area_mm2": bond,
            "total_cut_mm": total_cut,
            "bodies": bodies,
        }))
    })
}

fn bearing_json(size: &BearingSize) -> Value {
    json!({
        "designation": size.designation,
        "bore_mm": size.bore * 1000.0,
        "outer_mm": size.outer * 1000.0,
        "width_mm": size.width * 1000.0,
        "dynamic_rating_n": size.dynamic_rating,
        "static_rating_n": size.static_rating,
        "contact_angle_deg": size.contact_angle,
        "used_for": size.used_for,
    })
}

fn bearing(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.bearing";
    let designation = str_(p, "designation").ok_or_else(|| bad(cmd, "give `designation`"))?;
    let size = find_bearing(designation)
        .ok_or_else(|| bad(cmd, format!("`designation` is one of {}", SIZES.iter().map(|b| b.designation).collect::<Vec<_>>().join(", "))))?;
    let centre = match p.get("center") {
        None | Some(Value::Null) => Vec3::ZERO,
        Some(v) => vec3(v).filter(|c| c.len() <= 1e6).ok_or_else(|| bad(cmd, "`center` is a point [x, y, z]"))?,
    };
    let (bore, outer, width) = (size.bore * 1000.0, size.outer * 1000.0, size.width * 1000.0);
    let pitch = (bore + outer) / 4.0;
    let ball = (outer / 2.0 - bore / 2.0) * 0.28;
    let inner_outer_r = pitch - ball * 0.85;
    let outer_inner_r = pitch + ball * 0.85;
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Bearing {}", size.designation));
    let mut out = bearing_json(&size);
    in_component(s, &name, |s| {
        let inner = format!("{name} inner ring");
        let outer_ring = format!("{name} outer ring");
        let balls = format!("{name} balls");
        run(s, "solid.cylinder", json!({"radius": inner_outer_r, "height": width, "base": arr(centre), "operation": "new", "body_name": inner}))?;
        run(
            s,
            "solid.cylinder",
            json!({"radius": bore / 2.0, "height": width + 0.2, "base": arr(centre - Vec3::Z * 0.1), "operation": "cut", "targets": [inner]}),
        )?;
        run(s, "solid.cylinder", json!({"radius": outer / 2.0, "height": width, "base": arr(centre), "operation": "new", "body_name": outer_ring}))?;
        run(
            s,
            "solid.cylinder",
            json!({"radius": outer_inner_r, "height": width + 0.2, "base": arr(centre - Vec3::Z * 0.1), "operation": "cut", "targets": [outer_ring]}),
        )?;
        run(
            s,
            "solid.torus",
            json!({"major": pitch, "minor": ball, "center": arr(centre + Vec3::Z * (width / 2.0)), "operation": "new", "body_name": balls}),
        )?;
        out["bodies"] = json!([inner, outer_ring, balls]);
        Ok(out)
    })
}

fn damper(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.damper";
    let n = |key: &str, default: f64, lo: f64, hi: f64| number(cmd, p, key, Some(default), lo..=hi).map(|v| v.unwrap_or(default));
    let eye_to_eye = n("eye_to_eye_mm", 165.0, 20.0, 2000.0)?;
    let stroke = n("stroke_mm", 40.0, 0.0, 1000.0)?;
    let body_d = n("body_diameter_mm", 32.0, 5.0, 300.0)?;
    let rod_d = n("rod_diameter_mm", 12.0, 1.0, 200.0)?;
    let eye_bore = n("eye_bore_mm", 8.0, 1.0, 100.0)?;
    let eye_width = n("eye_width_mm", 12.0, 1.0, 200.0)?;
    let coil_over = bool_(p, "coil_over").unwrap_or(false);
    let coil_d = n("coil_diameter_mm", (body_d * 1.6).round(), 5.0, 500.0)?;
    let wire = n("wire_diameter_mm", 4.0, 0.5, 50.0)?;
    let rate = n("spring_rate_n_mm", 30.0, 0.01, 1e5)?;
    let damping = n("damping_ns_m", 600.0, 0.0, 1e6)?;
    if rod_d >= body_d {
        return Err(bad(cmd, "the rod has to fit inside the body"));
    }
    if stroke >= eye_to_eye {
        return Err(bad(cmd, "the stroke is longer than the damper, so it would pull itself apart"));
    }
    let eye_outer = eye_bore / 2.0 + 4.0;
    let between = eye_to_eye - 2.0 * eye_outer;
    if !(between > 0.0) {
        return Err(bad(cmd, "the two eyes overlap: it is shorter than its own mountings"));
    }
    if coil_over && coil_d <= body_d + wire {
        return Err(bad(cmd, "the coil has to clear the body: raise `coil_diameter_mm`"));
    }
    let body_len = between * 0.55;
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Damper {eye_to_eye:.0} x {stroke:.0}"));
    in_component(s, &name, |s| {
        // Body, rod and the two necks into the eyes as one revolved profile (radius, height):
        // no boolean has to meet a cylinder on a cylinder.
        let (rb, rr) = (body_d / 2.0, rod_d / 2.0);
        let profile: Vec<[f64; 2]> = vec![
            [0.0, 0.0],
            [rr, 0.0],
            [rr, eye_outer],
            [rb, eye_outer],
            [rb, eye_outer + body_len],
            [rr, eye_outer + body_len],
            [rr, eye_to_eye],
            [0.0, eye_to_eye],
        ];
        let points: Vec<[f64; 2]> = profile.iter().map(|[r, z]| [*r, -*z]).collect();
        let body = format!("{name} body");
        run(s, "sketch.create", json!({"plane": "XZ"}))?;
        run(s, "sketch.line", json!({"points": points, "closed": true}))?;
        run(s, "sketch.finish", json!({}))?;
        run(s, "solid.revolve", json!({"axis": "y", "operation": "new", "body_name": body}))?;
        let mut bodies = vec![body];
        for (k, z) in [0.0, eye_to_eye].into_iter().enumerate() {
            // An eye: a disc with its bore along x.
            let eye = format!("{name} eye {}", k + 1);
            run(
                s,
                "solid.cylinder",
                json!({"radius": eye_outer, "height": eye_width, "base": [-eye_width / 2.0, 0, z], "axis": "X", "operation": "new", "body_name": eye}),
            )?;
            run(
                s,
                "solid.cylinder",
                json!({"radius": eye_bore / 2.0, "height": eye_width + 0.4, "base": [-eye_width / 2.0 - 0.2, 0, z], "axis": "X", "operation": "cut", "targets": [eye]}),
            )?;
            bodies.push(eye);
        }
        if coil_over {
            let coil = format!("{name} coil");
            let mean = (coil_d - wire) / 2.0;
            run(
                s,
                "solid.cylinder",
                json!({"radius": mean + wire / 2.0, "height": between, "base": [0, 0, eye_outer], "operation": "new", "body_name": coil}),
            )?;
            run(
                s,
                "solid.cylinder",
                json!({"radius": mean - wire / 2.0, "height": between + 0.4, "base": [0, 0, eye_outer - 0.2], "operation": "cut", "targets": [coil]}),
            )?;
            bodies.push(coil);
        }
        Ok(json!({
            "eye_to_eye_mm": eye_to_eye,
            "compressed_mm": eye_to_eye - stroke,
            "stroke_mm": stroke,
            "body_diameter_mm": body_d,
            "rod_diameter_mm": rod_d,
            "eye_bore_mm": eye_bore,
            "coil_over": coil_over,
            "spring_rate_n_mm": rate,
            "damping_ns_m": damping,
            "bodies": bodies,
        }))
    })
}

#[cfg(test)]
#[path = "bamboo_tests.rs"]
mod tests;
