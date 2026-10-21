//! Mechanical generators (`design.*`): gears with true involute flanks, racks, bevel gears,
//! stepped shafts and springs, each built in its own component from ordinary sketches and
//! features, so the part stays parametric and editable like anything drawn by hand.
//!
//! A gear's tooth is the involute of its base circle: P(t) = r_b (cos t + t sin t, sin t − t cos t),
//! sampled from the base (or root) circle to the tip and fitted as a spline, mirrored about the
//! tooth's centreline, joined by a tip arc and a root arc to the next tooth. A helical gear is
//! a smooth loft through that outline turned progressively along the axis; a herringbone is two
//! half-width helices of opposite hand meeting at the middle; a bevel gear is the ruled loft
//! from the outline at the heel to the same outline shrunk at the toe; an internal (ring) gear
//! is the mirrored involute cut out of a rim.

use std::f64::consts::PI;

use serde_json::{Value, json};
use solidcraft_analysis::construction::{self, Options as KitOptions};
use solidcraft_analysis::drives::{self, Vec2, Wheel};
use solidcraft_doc::{FeatureKind, MeshData};
use solidcraft_geom::{Mesh, Vec3};

use super::CommandSpec;
use crate::params::{bad, bool_, str_};
use crate::{EngineError, Result, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("design.spur_gear", "Spur Gear", spur_gear).at("3DEEE", "GEARS").icon("spur_gear").params(
        "module: mm (0.2 to 50); teeth: 4 to 400; width: mm; pressure_angle_deg? (default 20); bore_mm?: a through hole; helix_deg?: a helical \
         gear's helix angle (up to 60); herringbone?: true for two opposite helices meeting at the middle (helical and herringbone gears are built \
         as mesh bodies with their bore unless exact?: true asks for the lofted B-rep, which takes minutes); internal?: true for a ring gear with the teeth \
         pointing inward, rim_mm? its rim thickness (default 5); name? → component, occurrence, name, pitch_diameter_mm, tip_diameter_mm, root_diameter_mm, \
         base_diameter_mm, circular_pitch_mm",
    ),
    CommandSpec::new("design.gear_rack", "Gear Rack", gear_rack).at("3DEEE", "GEARS").icon("gear_rack").params(
        "module: mm; teeth: 1 to 400; width: mm (the face, along Y); back_mm?: thickness under the teeth (default 2 × module); pressure_angle_deg? \
         (default 20); name? → component, occurrence, name, length_mm, pitch_mm",
    ),
    CommandSpec::new("design.bevel_gear", "Bevel Gear", bevel_gear).at("3DEEE", "GEARS").icon("bevel_gear").params(
        "module: mm; teeth: 6 to 400; pitch_angle_deg: the cone's half angle (3 to 87; 45 for a mitre pair); face_width: mm along the cone; \
         pressure_angle_deg? (default 20); bore_mm?; name? → component, occurrence, name, outer_cone_distance_mm, pitch_diameter_mm",
    ),
    CommandSpec::new("design.shaft", "Shaft", shaft).at("3DEEE", "DRIVE").icon("shaft").params(
        "sections: [{diameter: mm, length: mm, keyway?: {width, depth} mm (a parallel key seat in the middle of the section), groove?: {width, depth} mm \
         (a circlip groove in the middle)}] (1 to 50, chained along +Z from the origin); chamfer_mm?: on the two end faces; name? → component, occurrence, \
         name, length_mm",
    ),
    CommandSpec::new("design.spring", "Spring", spring).at("3DEEE", "DRIVE").icon("coil").params(
        "kind: compression | extension | torsion; mean_diameter: mm; wire_diameter: mm; coils: active coils (compression: with end_coils? closed at each \
         end, default 1, and free_length: mm; extension: whole coils, close wound, with a half loop at each end; torsion: coils with leg_mm? legs, \
         default 20); name? → component, occurrence, name, and for a compression spring its pitch_mm and the spring rate_n_mm from the wire's shear modulus \
         shear_modulus_pa? (default 79e9, steel)",
    ),
];

pub(super) fn run(s: &mut Session, id: &str, p: Value) -> Result<Value> {
    s.execute(id, &p)
}

pub(super) fn number(cmd: &str, p: &Value, key: &str, default: Option<f64>, range: std::ops::RangeInclusive<f64>) -> Result<Option<f64>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v
            .as_f64()
            .filter(|x| x.is_finite() && range.contains(x))
            .map(Some)
            .ok_or_else(|| bad(cmd, format!("`{key}` must be a number from {} to {}", range.start(), range.end()))),
    }
}

pub(super) fn required(cmd: &str, p: &Value, key: &str, range: std::ops::RangeInclusive<f64>) -> Result<f64> {
    number(cmd, p, key, None, range)?.ok_or_else(|| bad(cmd, format!("give `{key}`")))
}

/// The curve ids a sketch command created.
fn curve_ids(v: &Value) -> Vec<Value> {
    v["created"]["curves"].as_array().cloned().unwrap_or_default()
}

/// Runs `build` inside a new component named `name`, restoring the active component afterwards
/// and turning any feature error into the command's error.
pub(super) fn in_component(s: &mut Session, name: &str, build: impl FnOnce(&mut Session) -> Result<Value>) -> Result<Value> {
    let active = s.active_component;
    let c = run(s, "component.create", json!({"name": name}))?;
    let (comp, occ) = (c["component"].as_u64().unwrap_or(0), c["occurrence"].as_u64().unwrap_or(0));
    let built = build(s);
    let _ = run(s, "component.activate", json!({"component": active}));
    let mut out = built?;
    s.refresh();
    if let Some(e) = s.model.results.iter().find_map(|r| r.error.clone()) {
        return Err(EngineError::Other(e));
    }
    out["component"] = json!(comp);
    out["occurrence"] = json!(occ);
    out["name"] = json!(name);
    Ok(out)
}

/// The involute gear outline as sketch commands in the active XY sketch (mm). Returns the
/// radii it used: (pitch, base, tip, root).
pub(super) struct GearOutline {
    pub(super) pitch: f64,
    pub(super) base: f64,
    pub(super) tip: f64,
    pub(super) root: f64,
}

/// Draws one closed gear outline: flanks as fit-point splines, tip and root arcs through their
/// midpoints. `internal` mirrors the involute so the teeth point inward (the tooth thickens
/// with radius, as the complement of a space does).
fn draw_gear_outline(s: &mut Session, cmd: &str, module: f64, teeth: usize, pressure_angle: f64, internal: bool) -> Result<GearOutline> {
    let pitch = module * teeth as f64 / 2.0;
    let base = pitch * pressure_angle.cos();
    let (tip, root) = if internal { (pitch - module, pitch + 1.25 * module) } else { (pitch + module, pitch - 1.25 * module) };
    if (internal && tip <= base * 0.5) || (!internal && root <= 1e-6) {
        return Err(bad(cmd, "the root circle collapsed: too few teeth for this module"));
    }
    let sign = if internal { -1.0 } else { 1.0 };
    let involute = |t: f64, rotation: f64| -> [f64; 2] {
        let x = base * (t.cos() + t * t.sin());
        let y = sign * base * (t.sin() - t * t.cos());
        [x * rotation.cos() - y * rotation.sin(), x * rotation.sin() + y * rotation.cos()]
    };
    let param_at = |r: f64| if r <= base { 0.0 } else { (r * r / (base * base) - 1.0).sqrt() };
    let involute_angle = |t: f64| t - t.atan();
    let pitch_param = param_at(pitch);
    let half_tooth = PI / (2.0 * teeth as f64);
    // The flank is offset so the tooth is symmetric about angle 0; for an internal gear the
    // rollback runs the other way.
    let flank_offset = half_tooth + sign * involute_angle(pitch_param);
    let (near, far) = if internal { (tip.max(base), root) } else { (root.max(base), tip) };
    let (start_param, end_param) = (param_at(near), param_at(far));
    const SAMPLES: usize = 12;
    let tooth_pitch = 2.0 * PI / teeth as f64;
    for tooth in 0..teeth {
        let centre = tooth as f64 * tooth_pitch;
        let (cx, cy) = (centre.cos(), centre.sin());
        let rising: Vec<[f64; 2]> = (0..SAMPLES)
            .map(|i| involute(start_param + (end_param - start_param) * i as f64 / (SAMPLES - 1) as f64, centre - flank_offset))
            .collect();
        // The other flank is the true mirror of the first across the tooth's centreline, run
        // from the far end back to the near one.
        let falling: Vec<[f64; 2]> = rising
            .iter()
            .rev()
            .map(|p| {
                let dot = p[0] * cx + p[1] * cy;
                [2.0 * dot * cx - p[0], 2.0 * dot * cy - p[1]]
            })
            .collect();
        run(s, "sketch.spline.fit_point", json!({"points": rising}))?;
        let (rise_end, fall_start) = (rising[SAMPLES - 1], falling[0]);
        let far_mid = [far * cx, far * cy];
        run(s, "sketch.arc.three_point", json!({"start": rise_end, "end": fall_start, "through": far_mid}))?;
        run(s, "sketch.spline.fit_point", json!({"points": falling}))?;
        let next_centre = centre + tooth_pitch;
        let next_rise_start = involute(start_param, next_centre - flank_offset);
        let gap = centre + tooth_pitch / 2.0;
        let near_mid = [near * gap.cos(), near * gap.sin()];
        run(s, "sketch.arc.three_point", json!({"start": falling[SAMPLES - 1], "end": next_rise_start, "through": near_mid}))?;
    }
    Ok(GearOutline { pitch, base, tip, root })
}

/// A helical gear body as a loft through the outline turned progressively from `turn_from` to
/// `turn_to` degrees between the heights `z0` and `z1`: a smooth surface through five turned
/// sections follows the helix to well within a hundredth of a millimetre at the twists a gear
/// has, and costs a fraction of a twisted sweep's stations.
#[allow(clippy::too_many_arguments)]
fn helical_loft(
    s: &mut Session,
    cmd: &str,
    module: f64,
    teeth: usize,
    pressure: f64,
    z0: f64,
    z1: f64,
    turn_from: f64,
    turn_to: f64,
    operation: &str,
) -> Result<GearOutline> {
    const SECTIONS: usize = 5;
    let mut sections = Vec::with_capacity(SECTIONS);
    let mut first = None;
    for k in 0..SECTIONS {
        let u = k as f64 / (SECTIONS - 1) as f64;
        run(s, "sketch.create", json!({"plane": "XY", "offset": z0 + (z1 - z0) * u}))?;
        let outline = draw_turned_gear_outline(s, cmd, module, teeth, pressure, (turn_from + (turn_to - turn_from) * u).to_radians())?;
        first.get_or_insert(outline);
        let sk = run(s, "sketch.finish", json!({}))?;
        sections.push(json!({"sketch": sk["sketch"]}));
    }
    run(s, "solid.loft", json!({"sections": sections, "operation": operation}))?;
    first.ok_or_else(|| bad(cmd, "no section was drawn"))
}

/// The gear outline as a closed polygon (mm), counter-clockwise, turned by `rotation` radians:
/// the same involute flanks, tip and root arcs `draw_gear_outline` sketches, sampled.
pub(super) fn gear_outline_points(module: f64, teeth: usize, pressure_angle: f64, rotation: f64) -> Option<(Vec<[f64; 2]>, GearOutline)> {
    let pitch = module * teeth as f64 / 2.0;
    gear_outline_points_tip(module, teeth, pressure_angle, rotation, pitch + module)
}

/// The same outline with the tip circle at `tip` (a throated worm wheel's tips follow the worm):
/// `None` when the tip is below the pitch circle or the flanks would meet before it.
pub(super) fn gear_outline_points_tip(
    module: f64,
    teeth: usize,
    pressure_angle: f64,
    rotation: f64,
    tip: f64,
) -> Option<(Vec<[f64; 2]>, GearOutline)> {
    let pitch = module * teeth as f64 / 2.0;
    let base = pitch * pressure_angle.cos();
    let root = pitch - 1.25 * module;
    if root <= 1e-6 || tip <= pitch || !tip.is_finite() {
        return None;
    }
    let involute = |t: f64, turn: f64| -> [f64; 2] {
        let x = base * (t.cos() + t * t.sin());
        let y = base * (t.sin() - t * t.cos());
        [x * turn.cos() - y * turn.sin(), x * turn.sin() + y * turn.cos()]
    };
    let param_at = |r: f64| if r <= base { 0.0 } else { (r * r / (base * base) - 1.0).sqrt() };
    let pitch_param = param_at(pitch);
    let half_tooth = PI / (2.0 * teeth as f64);
    let flank_offset = half_tooth + (pitch_param - pitch_param.atan());
    let near = root.max(base);
    let (start_param, end_param) = (param_at(near), param_at(tip));
    // The tooth's half angle at the tip: the flanks must not have met below it.
    if half_tooth + (pitch_param - pitch_param.atan()) - (end_param - end_param.atan()) <= 1e-3 {
        return None;
    }
    const SAMPLES: usize = 10;
    const ARC: usize = 4;
    let tooth_pitch = 2.0 * PI / teeth as f64;
    let mut points = Vec::with_capacity(teeth * (2 * SAMPLES + 2 * ARC));
    let polar = |r: f64, a: f64| [r * a.cos(), r * a.sin()];
    for tooth in 0..teeth {
        let centre = rotation + tooth as f64 * tooth_pitch;
        let (cx, cy) = (centre.cos(), centre.sin());
        let rising: Vec<[f64; 2]> = (0..SAMPLES)
            .map(|i| involute(start_param + (end_param - start_param) * i as f64 / (SAMPLES - 1) as f64, centre - flank_offset))
            .collect();
        let falling: Vec<[f64; 2]> = rising
            .iter()
            .rev()
            .map(|p| {
                let dot = p[0] * cx + p[1] * cy;
                [2.0 * dot * cx - p[0], 2.0 * dot * cy - p[1]]
            })
            .collect();
        // Up the first flank, over the tip, down the second, along the root to the next tooth.
        points.extend(rising.iter().copied());
        let (a0, a1) = (rising[SAMPLES - 1][1].atan2(rising[SAMPLES - 1][0]), falling[0][1].atan2(falling[0][0]));
        let mut sweep = a1 - a0;
        while sweep < 0.0 {
            sweep += 2.0 * PI;
        }
        for i in 1..=ARC {
            points.push(polar(tip, a0 + sweep * i as f64 / (ARC + 1) as f64));
        }
        points.extend(falling.iter().copied());
        let next = involute(start_param, centre + tooth_pitch - flank_offset);
        let (r0, r1) = (falling[SAMPLES - 1][1].atan2(falling[SAMPLES - 1][0]), next[1].atan2(next[0]));
        let mut sweep = r1 - r0;
        while sweep < 0.0 {
            sweep += 2.0 * PI;
        }
        for i in 1..=ARC {
            points.push(polar(near, r0 + sweep * i as f64 / (ARC + 1) as f64));
        }
    }
    Some((points, GearOutline { pitch, base, tip, root }))
}

/// A helical (or herringbone) gear as one closed mesh: the outline turned station by station up
/// the width, quads between stations, and ring caps between the outline and the bore (or fans to
/// the centre without one). `twists` lists the turn (radians) at each height fraction given.
pub(super) fn helical_gear_mesh(
    module: f64,
    teeth: usize,
    pressure: f64,
    width: f64,
    bore: f64,
    twist_at: &dyn Fn(f64) -> f64,
) -> Option<(Mesh, GearOutline)> {
    helical_gear_mesh_tip(module, teeth, pressure, width, bore, twist_at, None)
}

/// [`helical_gear_mesh`] with the tip circle at each height fraction given by `tip_at` (a worm
/// wheel's throat), the plain tip when `None`.
pub(super) fn helical_gear_mesh_tip(
    module: f64,
    teeth: usize,
    pressure: f64,
    width: f64,
    bore: f64,
    twist_at: &dyn Fn(f64) -> f64,
    tip_at: Option<&dyn Fn(f64) -> f64>,
) -> Option<(Mesh, GearOutline)> {
    let (_, outline) = gear_outline_points(module, teeth, pressure, 0.0)?;
    if bore > 0.0 && bore / 2.0 >= outline.root - 1e-6 {
        return None;
    }
    let stations = (width / module * 2.0).ceil().clamp(8.0, 120.0) as usize;
    let mut mesh = Mesh::default();
    let mut rings: Vec<u32> = Vec::with_capacity(stations + 1);
    let mut n = 0usize;
    for k in 0..=stations {
        let u = k as f64 / stations as f64;
        let tip = tip_at.map_or(outline.tip, |f| f(u));
        let (points, _) = gear_outline_points_tip(module, teeth, pressure, twist_at(u), tip)?;
        n = points.len();
        rings.push(mesh.positions.len() as u32);
        let z = width * u;
        for [x, y] in &points {
            mesh.positions.push(Vec3::new(*x, *y, z));
        }
        if bore > 0.0 {
            for [x, y] in &points {
                let r = bore / 2.0 / x.hypot(*y);
                mesh.positions.push(Vec3::new(x * r, y * r, z));
            }
        }
    }
    let n32 = n as u32;
    let bore_offset = if bore > 0.0 { n32 } else { 0 };
    mesh.normals.resize(mesh.positions.len(), Vec3::ZERO);
    let mut face = 0u32;
    let quad = |mesh: &mut Mesh, a: u32, b: u32, c: u32, d: u32, face: u32| {
        mesh.triangles.push([a, b, c]);
        mesh.triangles.push([a, c, d]);
        mesh.tri_face.push(face);
        mesh.tri_face.push(face);
    };
    for w in 0..stations {
        let (lo, hi) = (rings[w], rings[w + 1]);
        for i in 0..n32 {
            let j = (i + 1) % n32;
            quad(&mut mesh, lo + i, lo + j, hi + j, hi + i, face);
            if bore > 0.0 {
                quad(&mut mesh, lo + bore_offset + j, lo + bore_offset + i, hi + bore_offset + i, hi + bore_offset + j, face + 1);
            }
        }
    }
    face += 2;
    // Caps: bottom faces down (clockwise seen from above), top faces up.
    let (bottom, top) = (rings[0], rings[stations]);
    if bore > 0.0 {
        for i in 0..n32 {
            let j = (i + 1) % n32;
            quad(&mut mesh, bottom + j, bottom + i, bottom + bore_offset + i, bottom + bore_offset + j, face);
            quad(&mut mesh, top + i, top + j, top + bore_offset + j, top + bore_offset + i, face + 1);
        }
    } else {
        let c0 = mesh.positions.len() as u32;
        mesh.positions.push(Vec3::new(0.0, 0.0, 0.0));
        mesh.positions.push(Vec3::new(0.0, 0.0, width));
        mesh.normals.resize(mesh.positions.len(), Vec3::ZERO);
        for i in 0..n32 {
            let j = (i + 1) % n32;
            mesh.triangles.push([c0, bottom + j, bottom + i]);
            mesh.tri_face.push(face);
            mesh.triangles.push([c0 + 1, top + i, top + j]);
            mesh.tri_face.push(face + 1);
        }
    }
    Some((mesh, outline))
}

fn spur_gear(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.spur_gear";
    let module = required(cmd, p, "module", 0.2..=50.0)?;
    let teeth = required(cmd, p, "teeth", 4.0..=400.0)? as usize;
    let width = required(cmd, p, "width", 0.1..=2000.0)?;
    let pressure = number(cmd, p, "pressure_angle_deg", Some(20.0), 10.0..=30.0)?.unwrap_or(20.0).to_radians();
    let bore = number(cmd, p, "bore_mm", Some(0.0), 0.0..=2000.0)?.unwrap_or(0.0);
    let helix = number(cmd, p, "helix_deg", Some(0.0), -60.0..=60.0)?.unwrap_or(0.0);
    let herringbone = bool_(p, "herringbone").unwrap_or(false);
    let internal = bool_(p, "internal").unwrap_or(false);
    let rim = number(cmd, p, "rim_mm", Some(5.0), 0.1..=1000.0)?.unwrap_or(5.0);
    if internal && teeth < 8 {
        return Err(bad(cmd, "a ring gear needs at least 8 teeth"));
    }
    if (herringbone || helix.abs() > 1e-9) && internal {
        return Err(bad(cmd, "a helical ring gear is not built; leave helix_deg and herringbone off with internal"));
    }
    let pitch_d = module * teeth as f64;
    if bore > 0.0 && bore >= pitch_d - 2.5 * module {
        return Err(bad(cmd, "the bore is wider than the gear's root"));
    }
    let kind = if internal {
        "Ring gear"
    } else if herringbone {
        "Herringbone gear"
    } else if helix.abs() > 1e-9 {
        "Helical gear"
    } else {
        "Spur gear"
    };
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{kind} m{module} z{teeth}"));
    let pitch_radius = pitch_d / 2.0;
    let exact = bool_(p, "exact").unwrap_or(false);
    let twist_over = |length: f64| (length * helix.to_radians().tan() / pitch_radius).to_degrees();
    in_component(s, &name, |s| {
        let outline = if internal {
            // The rim first, then the toothed opening cut out of it.
            run(s, "solid.cylinder", json!({"radius": pitch_radius + 1.25 * module + rim, "height": width, "operation": "new"}))?;
            run(s, "sketch.create", json!({"plane": "XY"}))?;
            let outline = draw_gear_outline(s, cmd, module, teeth, pressure, true)?;
            run(s, "sketch.finish", json!({}))?;
            run(s, "solid.extrude", json!({"distance": width, "operation": "cut"}))?;
            outline
        } else if (herringbone || helix.abs() > 1e-9) && !exact {
            // A helical gear's flank is a helicoid, which the kernel can only reach through
            // minutes of lofts or sweeps; so by default it is built directly as a mesh body with
            // the outline turned station by station (a herringbone turning back at the middle),
            // with its bore. `exact: true` asks for the lofted B-rep instead.
            let twist = twist_over(width).to_radians();
            let turn: Box<dyn Fn(f64) -> f64> =
                if herringbone { Box::new(move |u: f64| twist / 2.0 * (1.0 - (2.0 * u - 1.0).abs())) } else { Box::new(move |u: f64| twist * u) };
            let (mesh, outline) = helical_gear_mesh(module, teeth, pressure, width, bore, &*turn)
                .ok_or_else(|| bad(cmd, "the root circle collapsed, or the bore reaches the roots"))?;
            add_mesh_body(s, &name, mesh, cmd)?;
            return Ok(
                json!({"pitch_diameter_mm": 2.0 * outline.pitch, "tip_diameter_mm": 2.0 * outline.tip, "root_diameter_mm": 2.0 * outline.root, "base_diameter_mm": 2.0 * outline.base, "circular_pitch_mm": PI * module, "kind": kind, "mesh_body": true}),
            );
        } else if herringbone {
            // Two half-width helices of opposite hand meeting at the middle.
            let half = width / 2.0;
            let outline = helical_loft(s, cmd, module, teeth, pressure, 0.0, half, 0.0, twist_over(half), "new")?;
            helical_loft(s, cmd, module, teeth, pressure, half, width, twist_over(half), 0.0, "join")?;
            outline
        } else if helix.abs() > 1e-9 {
            helical_loft(s, cmd, module, teeth, pressure, 0.0, width, 0.0, twist_over(width), "new")?
        } else {
            run(s, "sketch.create", json!({"plane": "XY"}))?;
            let outline = draw_gear_outline(s, cmd, module, teeth, pressure, false)?;
            run(s, "sketch.finish", json!({}))?;
            run(s, "solid.extrude", json!({"distance": width, "operation": "new"}))?;
            outline
        };
        if bore > 0.0 && !internal {
            run(s, "solid.cylinder", json!({"diameter": bore, "height": width + 2.0, "base": [0, 0, -1], "operation": "cut"}))?;
        }
        Ok(json!({
            "pitch_diameter_mm": 2.0 * outline.pitch,
            "tip_diameter_mm": 2.0 * outline.tip,
            "root_diameter_mm": 2.0 * outline.root,
            "base_diameter_mm": 2.0 * outline.base,
            "circular_pitch_mm": PI * module,
            "kind": kind,
        }))
    })
}

/// The spur outline turned by `rotation` about the axis, for the second half of a herringbone.
fn draw_turned_gear_outline(s: &mut Session, cmd: &str, module: f64, teeth: usize, pressure_angle: f64, rotation: f64) -> Result<GearOutline> {
    // Drawing is in sketch coordinates; the turn is applied by rotating the whole outline's
    // points, which the drawing routine does through its tooth centres, so offset them all.
    let pitch = module * teeth as f64 / 2.0;
    let base = pitch * pressure_angle.cos();
    let (tip, root) = (pitch + module, pitch - 1.25 * module);
    if root <= 1e-6 {
        return Err(bad(cmd, "the root circle collapsed"));
    }
    let involute = |t: f64, rot: f64| -> [f64; 2] {
        let x = base * (t.cos() + t * t.sin());
        let y = base * (t.sin() - t * t.cos());
        [x * rot.cos() - y * rot.sin(), x * rot.sin() + y * rot.cos()]
    };
    let param_at = |r: f64| if r <= base { 0.0 } else { (r * r / (base * base) - 1.0).sqrt() };
    let involute_angle = |t: f64| t - t.atan();
    let flank_offset = PI / (2.0 * teeth as f64) + involute_angle(param_at(pitch));
    let (start_param, end_param) = (param_at(root.max(base)), param_at(tip));
    const SAMPLES: usize = 12;
    let tooth_pitch = 2.0 * PI / teeth as f64;
    for tooth in 0..teeth {
        let centre = rotation + tooth as f64 * tooth_pitch;
        let (cx, cy) = (centre.cos(), centre.sin());
        let rising: Vec<[f64; 2]> = (0..SAMPLES)
            .map(|i| involute(start_param + (end_param - start_param) * i as f64 / (SAMPLES - 1) as f64, centre - flank_offset))
            .collect();
        let falling: Vec<[f64; 2]> = rising
            .iter()
            .rev()
            .map(|p| {
                let dot = p[0] * cx + p[1] * cy;
                [2.0 * dot * cx - p[0], 2.0 * dot * cy - p[1]]
            })
            .collect();
        run(s, "sketch.spline.fit_point", json!({"points": rising}))?;
        run(s, "sketch.arc.three_point", json!({"start": rising[SAMPLES - 1], "end": falling[0], "through": [tip * cx, tip * cy]}))?;
        run(s, "sketch.spline.fit_point", json!({"points": falling}))?;
        let next = involute(start_param, centre + tooth_pitch - flank_offset);
        let gap = centre + tooth_pitch / 2.0;
        run(s, "sketch.arc.three_point", json!({"start": falling[SAMPLES - 1], "end": next, "through": [root * gap.cos(), root * gap.sin()]}))?;
    }
    Ok(GearOutline { pitch, base, tip, root })
}

fn gear_rack(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.gear_rack";
    let module = required(cmd, p, "module", 0.2..=50.0)?;
    let teeth = required(cmd, p, "teeth", 1.0..=400.0)? as usize;
    let width = required(cmd, p, "width", 0.1..=2000.0)?;
    let back = number(cmd, p, "back_mm", Some(2.0 * module), 0.1..=1000.0)?.unwrap_or(2.0 * module);
    let pressure = number(cmd, p, "pressure_angle_deg", Some(20.0), 10.0..=30.0)?.unwrap_or(20.0).to_radians();
    let pitch = PI * module;
    let (addendum, dedendum) = (module, 1.25 * module);
    let slope = pressure.tan();
    // Half the tooth at height z above the pitch line: straight flanks, the involute of an
    // infinite radius.
    let half = |z: f64| pitch / 4.0 - z * slope;
    if half(addendum) <= 1e-9 {
        return Err(bad(cmd, "the flanks meet before the tip: pressure angle too steep for this module"));
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Rack m{module} z{teeth}"));
    let length = teeth as f64 * pitch;
    in_component(s, &name, |s| {
        // The profile in the XZ plane (sketch y is −Z): teeth up (+Z), back below the pitch line.
        let mut pts: Vec<[f64; 2]> = vec![[0.0, dedendum + back]];
        for i in 0..teeth {
            let centre = (i as f64 + 0.5) * pitch;
            pts.push([centre - half(-dedendum), dedendum]);
            pts.push([centre - half(addendum), -addendum]);
            pts.push([centre + half(addendum), -addendum]);
            pts.push([centre + half(-dedendum), dedendum]);
        }
        pts.push([length, dedendum + back]);
        run(s, "sketch.create", json!({"plane": "XZ"}))?;
        run(s, "sketch.line", json!({"points": pts, "closed": true}))?;
        run(s, "sketch.finish", json!({}))?;
        run(s, "solid.extrude", json!({"distance": width, "operation": "new"}))?;
        Ok(json!({"length_mm": length, "pitch_mm": pitch, "tooth_height_mm": addendum + dedendum}))
    })
}

fn bevel_gear(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.bevel_gear";
    let module = required(cmd, p, "module", 0.2..=50.0)?;
    let teeth = required(cmd, p, "teeth", 6.0..=400.0)? as usize;
    let pitch_angle = required(cmd, p, "pitch_angle_deg", 3.0..=87.0)?.to_radians();
    let face_width = required(cmd, p, "face_width", 0.1..=2000.0)?;
    let pressure = number(cmd, p, "pressure_angle_deg", Some(20.0), 10.0..=30.0)?.unwrap_or(20.0).to_radians();
    let bore = number(cmd, p, "bore_mm", Some(0.0), 0.0..=2000.0)?.unwrap_or(0.0);
    let outer_cone = module * teeth as f64 / (2.0 * pitch_angle.sin());
    if face_width >= outer_cone {
        return Err(bad(cmd, "the face is wider than the cone is long: the teeth would run past the apex"));
    }
    let inner_cone = outer_cone - face_width;
    let shrink = inner_cone / outer_cone;
    let (z_toe, z_heel) = (inner_cone * pitch_angle.cos(), outer_cone * pitch_angle.cos());
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Bevel gear m{module} z{teeth}"));
    in_component(s, &name, |s| {
        // The heel outline at the heel's height and the same outline shrunk to the toe (a gear's
        // outline scales with its module), lofted: the ruled surface between them runs straight
        // to the apex, which is exactly a straight bevel tooth.
        run(s, "sketch.create", json!({"plane": "XY", "offset": z_heel}))?;
        let outline = draw_gear_outline(s, cmd, module, teeth, pressure, false)?;
        let heel = run(s, "sketch.finish", json!({}))?;
        run(s, "sketch.create", json!({"plane": "XY", "offset": z_toe}))?;
        draw_gear_outline(s, cmd, module * shrink, teeth, pressure, false)?;
        let toe = run(s, "sketch.finish", json!({}))?;
        run(s, "solid.loft", json!({"sections": [{"sketch": heel["sketch"]}, {"sketch": toe["sketch"]}], "operation": "new"}))?;
        if bore > 0.0 {
            run(s, "solid.cylinder", json!({"diameter": bore, "height": z_heel - z_toe + 2.0, "base": [0, 0, z_toe - 1.0], "operation": "cut"}))?;
        }
        Ok(json!({
            "outer_cone_distance_mm": outer_cone,
            "inner_cone_distance_mm": inner_cone,
            "pitch_diameter_mm": 2.0 * outline.pitch,
            "heel_z_mm": z_heel,
            "toe_z_mm": z_toe,
        }))
    })
}

fn shaft(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.shaft";
    let sections = p
        .get("sections")
        .and_then(Value::as_array)
        .filter(|a| !a.is_empty() && a.len() <= 50)
        .ok_or_else(|| bad(cmd, "`sections` is a list of 1 to 50 {diameter, length}"))?;
    let chamfer = number(cmd, p, "chamfer_mm", Some(0.0), 0.0..=100.0)?.unwrap_or(0.0);
    struct Section {
        d: f64,
        l: f64,
        keyway: Option<(f64, f64)>,
        groove: Option<(f64, f64)>,
    }
    let mut parsed = Vec::new();
    for sec in sections {
        let d = required(cmd, sec, "diameter", 0.1..=2000.0)?;
        let l = required(cmd, sec, "length", 0.1..=5000.0)?;
        let pair = |key: &str| -> Result<Option<(f64, f64)>> {
            match sec.get(key) {
                None | Some(Value::Null) => Ok(None),
                Some(v) => {
                    let w = required(cmd, v, "width", 0.1..=1000.0)?;
                    let depth = required(cmd, v, "depth", 0.1..=1000.0)?;
                    if w >= l || depth >= d / 2.0 {
                        return Err(bad(cmd, format!("the {key} must be narrower than its section and shallower than its radius")));
                    }
                    Ok(Some((w, depth)))
                }
            }
        };
        parsed.push(Section { d, l, keyway: pair("keyway")?, groove: pair("groove")? });
    }
    let total: f64 = parsed.iter().map(|x| x.l).sum();
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Shaft {} sections", parsed.len()));
    in_component(s, &name, |s| {
        // One revolved outline for the whole stack, grooves included.
        let mut pts: Vec<[f64; 2]> = vec![[0.0, 0.0]];
        let mut z = 0.0;
        for sec in &parsed {
            let r = sec.d / 2.0;
            pts.push([r, z]);
            if let Some((w, depth)) = sec.groove {
                let mid = z + sec.l / 2.0;
                pts.push([r, mid - w / 2.0]);
                pts.push([r - depth, mid - w / 2.0]);
                pts.push([r - depth, mid + w / 2.0]);
                pts.push([r, mid + w / 2.0]);
            }
            z += sec.l;
            pts.push([r, z]);
        }
        pts.push([0.0, z]);
        // Chamfers on the two end faces, when asked for.
        if chamfer > 0.0
            && let (Some(first), Some(last)) = (parsed.first(), parsed.last())
        {
            let c = chamfer.min(first.d / 4.0).min(first.l / 2.0);
            pts[1] = [first.d / 2.0 - c, 0.0];
            pts.insert(2, [first.d / 2.0, c]);
            let n = pts.len();
            let c2 = chamfer.min(last.d / 4.0).min(last.l / 2.0);
            pts[n - 2] = [last.d / 2.0 - c2, z];
            pts.insert(n - 2, [last.d / 2.0, z - c2]);
        }
        let points: Vec<[f64; 2]> = pts.iter().map(|[r, zz]| [*r, -*zz]).collect();
        run(s, "sketch.create", json!({"plane": "XZ"}))?;
        run(s, "sketch.line", json!({"points": points, "closed": true}))?;
        run(s, "sketch.finish", json!({}))?;
        run(s, "solid.revolve", json!({"axis": "y", "operation": "new"}))?;
        // Keyways: a box cut into the top of the section, open to the surface.
        let mut z0 = 0.0;
        for sec in &parsed {
            if let Some((w, depth)) = sec.keyway {
                let r = sec.d / 2.0;
                let mid = z0 + sec.l / 2.0;
                run(
                    s,
                    "solid.box",
                    json!({"length": w, "width": r + 1.0, "height": sec.l.min(w * 4.0), "corner": [-w / 2.0, r - depth, mid - sec.l.min(w * 4.0) / 2.0], "operation": "cut"}),
                )?;
            }
            z0 += sec.l;
        }
        Ok(json!({"length_mm": total, "sections": parsed.len()}))
    })
}

fn spring(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.spring";
    let kind = str_(p, "kind").unwrap_or("compression").to_ascii_lowercase();
    let mean = required(cmd, p, "mean_diameter", 0.5..=2000.0)?;
    let wire = required(cmd, p, "wire_diameter", 0.05..=200.0)?;
    let coils = required(cmd, p, "coils", 0.5..=500.0)?;
    if mean <= wire {
        return Err(bad(cmd, "the coil must be wider than the wire"));
    }
    let shear_modulus = number(cmd, p, "shear_modulus_pa", Some(79e9), 1e6..=1e13)?.unwrap_or(79e9);
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{} spring Ø{mean}", capital(&kind)));
    match kind.as_str() {
        "compression" => {
            let end_coils = number(cmd, p, "end_coils", Some(1.0), 0.0..=5.0)?.unwrap_or(1.0);
            let free_length = required(cmd, p, "free_length", 0.5..=5000.0)?;
            let closed = 1.02 * wire;
            let active_pitch = (free_length - wire - 2.0 * end_coils * closed) / coils;
            if active_pitch <= closed {
                return Err(bad(cmd, "too short for those coils: the active coils would touch"));
            }
            let total = coils + 2.0 * end_coils;
            // Spring rate k = G d⁴ / (8 D³ n) with D the mean diameter and n the active coils.
            let rate = shear_modulus * (wire / 1000.0).powi(4) / (8.0 * (mean / 1000.0).powi(3) * coils) / 1000.0;
            in_component(s, &name, |s| {
                // The closed end coils, the active coils and the closed top, as three coils. Each
                // is its own body: a boolean between two helices meeting face to face costs
                // minutes in the kernel, and the spring is one part to the eye and the scale.
                let mut z = wire / 2.0;
                if end_coils > 0.0 {
                    run(
                        s,
                        "solid.coil",
                        json!({"diameter": mean, "turns": end_coils, "pitch": closed, "section_size": wire, "section": "circular", "base": [0, 0, z], "operation": "new"}),
                    )?;
                    z += end_coils * closed;
                }
                run(
                    s,
                    "solid.coil",
                    json!({"diameter": mean, "turns": coils, "pitch": active_pitch, "section_size": wire, "section": "circular", "base": [0, 0, z], "start_angle": format!("{} deg", (end_coils * 360.0) % 360.0), "operation": "new"}),
                )?;
                z += coils * active_pitch;
                if end_coils > 0.0 {
                    run(
                        s,
                        "solid.coil",
                        json!({"diameter": mean, "turns": end_coils, "pitch": closed, "section_size": wire, "section": "circular", "base": [0, 0, z], "start_angle": format!("{} deg", ((end_coils + coils) * 360.0) % 360.0), "operation": "new"}),
                    )?;
                }
                Ok(json!({"pitch_mm": active_pitch, "rate_n_mm": rate, "turns": total, "free_length_mm": free_length}))
            })
        }
        "extension" => {
            let turns = coils.round().max(1.0);
            in_component(s, &name, |s| {
                run(
                    s,
                    "solid.coil",
                    json!({"diameter": mean, "turns": turns, "pitch": 1.02 * wire, "section_size": wire, "section": "circular", "base": [0, 0, wire / 2.0], "operation": "new"}),
                )?;
                // A hook at each end: a half ring of the coil's radius standing across the axis.
                let top = wire / 2.0 + turns * 1.02 * wire;
                for (z, flip) in [(wire / 2.0, true), (top, false)] {
                    let sk = run(s, "sketch.create", json!({"plane": "XZ", "offset": 0.0}))?;
                    let _ = sk;
                    let dir = if flip { -1.0 } else { 1.0 };
                    let arc = run(
                        s,
                        "sketch.arc.three_point",
                        json!({"start": [mean / 2.0, -z], "end": [-mean / 2.0, -z], "through": [0.0, -(z + dir * mean / 2.0)]}),
                    )?;
                    let curve = curve_ids(&arc).into_iter().next().ok_or_else(|| bad(cmd, "the hook was not drawn"))?;
                    let path = run(s, "sketch.finish", json!({}))?;
                    let path_id = path["sketch"].clone();
                    run(s, "sketch.create", json!({"plane": "XY", "offset": z}))?;
                    run(s, "sketch.circle.center", json!({"center": [mean / 2.0, 0.0], "diameter": wire}))?;
                    let profile = run(s, "sketch.finish", json!({}))?;
                    run(s, "solid.sweep", json!({"sketch": profile["sketch"], "path_sketch": path_id, "path": [curve], "operation": "new"}))?;
                }
                Ok(json!({"turns": turns, "body_length_mm": top + wire / 2.0}))
            })
        }
        "torsion" => {
            let leg = number(cmd, p, "leg_mm", Some(20.0), 0.5..=5000.0)?.unwrap_or(20.0);
            in_component(s, &name, |s| {
                run(
                    s,
                    "solid.coil",
                    json!({"diameter": mean, "turns": coils, "pitch": 1.02 * wire, "section_size": wire, "section": "circular", "base": [0, 0, wire / 2.0], "operation": "new"}),
                )?;
                // Two straight legs tangent to the coil: at the start (angle 0) along −Y, and at
                // the end along the tangent there.
                let end_angle = coils * 2.0 * PI;
                let top = wire / 2.0 + coils * 1.02 * wire;
                let legs = [
                    ([mean / 2.0, 0.0, wire / 2.0], [0.0, -1.0, 0.0]),
                    ([mean / 2.0 * end_angle.cos(), mean / 2.0 * end_angle.sin(), top], [-end_angle.sin(), end_angle.cos(), 0.0]),
                ];
                for (start, dir) in legs {
                    let end = [start[0] + dir[0] * leg, start[1] + dir[1] * leg, start[2]];
                    let axis = [dir[0], dir[1], dir[2]];
                    run(s, "solid.cylinder", json!({"diameter": wire, "height": leg, "base": start, "axis": axis, "operation": "new"}))?;
                    let _ = end;
                }
                Ok(json!({"turns": coils, "leg_mm": leg}))
            })
        }
        other => Err(bad(cmd, format!("`kind` is compression, extension or torsion, not `{other}`"))),
    }
}

// ---------------------------------------------------------------------------------------------
// Drives, profiles, motors, plastic details, bamboo and the construction kit
// ---------------------------------------------------------------------------------------------

pub static MORE_COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("design.pulley", "Pulley", pulley).at("3DEEE", "DRIVE").icon("pulley").params(
        "standard: a belt standard (GT2-2M, GT2-3M, GT2-5M, HTD-3M, HTD-5M, HTD-8M, HTD-14M, T2.5, T5, T10, AT5, AT10, MXL, XL, L, H); teeth: 6 to 300; \
         width: mm; bore_mm?; bore_style?: plain | keyway (key_width_mm?, key_depth_mm?) | set_screw; flanges?: true (flange_mm? proud of the tip, \
         default 1.5); plain_roller?: true for a deflection roller the belt's back runs on, with roller_diameter_mm?; centre?: [x, y, z] mm; name? → \
         component, occurrence, name, pitch_diameter_mm, outside_diameter_mm, root_diameter_mm",
    ),
    CommandSpec::new("design.belt", "Belt", belt).at("3DEEE", "DRIVE").icon("belt").params(
        "standard: a belt standard; width: mm; wheels: [{centre: [x, y] mm, teeth?: n (a toothed pulley) | roller_diameter_mm?: a plain roller, \
         side?: 1 (counter-clockwise, default) | -1 (an idler pressed in from outside)}] (2 to 20, in belt order); z?: mm the belt's mid-plane \
         (default 0); allow_crossing?: true for a crossed flat-belt drive; build?: false to only measure → length_mm, teeth (the standard belt it \
         matches, exact when integral), wraps_deg, teeth_in_mesh, clearance_mm, crossings, and when built the component",
    ),
    CommandSpec::new("design.sprocket", "Sprocket", sprocket).at("3DEEE", "DRIVE").icon("sprocket").params(
        "standard: a chain (05B, 06B, 08B, 10B, 12B, ANSI 25, 35, 40, 50, 60); teeth: 7 to 150; bore_mm?; centre?: [x, y, z] mm; name? → \
         component, occurrence, name, pitch_diameter_mm, tip_diameter_mm, root_diameter_mm, width_mm",
    ),
    CommandSpec::new("design.chain", "Chain", chain_cmd).at("3DEEE", "DRIVE").icon("roller_chain").params(
        "standard: a chain; sprockets: [{centre: [x, y] mm, teeth}, {centre, teeth}] (two); links?: n (default: what fits, rounded up to even); \
         z?: mm; build?: false to only measure (default true, rollers and plates as bodies; up to 400 links) → links, length_mm, \
         centre_distance_for_links_mm, and when built the component",
    ),
    CommandSpec::new("design.tslot", "T-Slot Profile", tslot).at("3DEEE", "DRIVE").icon("tslot").params(
        "series: 20 | 30 | 40; length: mm (1 to 6000); name? → component, occurrence, name, bore_mm",
    ),
    CommandSpec::new("design.stepper_motor", "Stepper Motor", stepper_motor).at("3DEEE", "DRIVE").icon("stepper_motor").params(
        "frame: NEMA 8 | NEMA 11 | NEMA 14 | NEMA 17 | NEMA 23; body_length_mm? (default the frame's common length); shaft_length_mm?; \
         shaft_flat?: true (default) for the flat a grub screw bites; name? → component, occurrence, name, body_mm, bolt_square_mm, bolt_mm, \
         boss_diameter_mm, boss_height_mm, shaft_diameter_mm. The front face is z = 0 with the shaft along +Z",
    ),
    CommandSpec::new("design.snap_hook", "Snap Hook", snap_hook).at("3DEEE", "PARTS").icon("snap_fit").params(
        "width: mm (along Z); arm_length: mm; thickness: mm; hook_depth: mm; hook_length: mm; name? → component, occurrence, name",
    ),
    CommandSpec::new("design.insert_boss", "Heat-Set Insert Boss", insert_boss).at("3DEEE", "PARTS").icon("insert_boss").params(
        "size: M2 | M2.5 | M3 | M4 | M5 | M6; height: mm (at least the bore depth + 1); name? → component, occurrence, name, boss_diameter_mm, \
         hole_diameter_mm, bore_depth_mm",
    ),
    CommandSpec::new("design.bamboo_culm", "Bamboo Culm", bamboo_culm).at("3DEEE", "PARTS").icon("bamboo_culm").params(
        "length: mm; butt_diameter: mm; tip_diameter?: mm (default the butt's); butt_wall: mm; tip_wall?: mm; node_spacing?: mm (0 = no nodes); \
         first_node?: mm from the butt; node_collar?: mm the collar stands proud (default 1.5); node_collar_height?: mm (default 8); \
         diaphragm?: mm thick (default 4; 0 leaves the bore open); name? → component, occurrence, name, nodes, wall_volume_mm3. The culm runs \
         along +Z from the butt at the origin",
    ),
    CommandSpec::new("construct.brickify", "Brickify", brickify).at("3DEEE", "PARTS").icon("brickify").params(
        "body?: name (default: the selected body, or the design's only one); bricks?: true (default) on the stud grid of brick?: brick (8 mm \
         studs, default) | plate (8 mm studs, 3.2 mm thin) | large (16 mm studs); profiles?: true (default: T-slot runs claim straight full \
         sections) of profile_series?: 20 | 30 | 40 (or 2020, 3030, 4040; default 20); min_profile_mm? (default 40, two 2020 cells); rods?: true (default: \
         threaded rods clamp columns of four layers or more) of rod?: M6 | M8 | M10 | M12 (default M8); bolts?: true (default: hex bolts clamp \
         the shorter columns) of bolt?: M3 | M4 | M5 | M6 | M8 (default M4); build?: true (default) adds the kit as a mesh body named name?; \
         hide?: true hides the source body → brick, profile_series, rod, bolt, bill_of_materials: {name: count}, elements, cells_total, \
         cells_covered, coverage, summary, warnings (why a family that is on placed nothing)",
    ),
];

fn standard_of(cmd: &str, p: &Value) -> Result<&'static drives::BeltStandard> {
    let name = str_(p, "standard").ok_or_else(|| bad(cmd, "give `standard`: a belt standard such as GT2-2M or HTD-5M"))?;
    drives::belt_standard(name).ok_or_else(|| bad(cmd, format!("`{name}` is not a belt standard this build knows")))
}

fn centre_of(cmd: &str, p: &Value) -> Result<Vec3> {
    match p.get("centre") {
        None | Some(Value::Null) => Ok(Vec3::ZERO),
        Some(v) => crate::params::vec3(v).ok_or_else(|| bad(cmd, "`centre` is [x, y, z] in mm")),
    }
}

/// The pulley's toothed outline in an XY sketch (mm), centred on the origin: a land across each
/// tip, then down into the groove and back out.
fn draw_pulley_outline(s: &mut Session, standard: &drives::BeltStandard, teeth: usize, c: Vec3) -> Result<(f64, f64, f64)> {
    let mm = 1000.0;
    let pitch_radius = drives::pitch_diameter(standard, teeth) / 2.0 * mm;
    let tip = drives::outside_diameter(standard, teeth) / 2.0 * mm;
    let root = drives::root_diameter(standard, teeth) / 2.0 * mm;
    if root <= 1e-6 {
        return Err(bad("design.pulley", "the groove is deeper than the pulley radius"));
    }
    let trapezoidal = standard.tooth == drives::ToothForm::Trapezoidal;
    let groove_fraction = match standard.tooth {
        drives::ToothForm::Trapezoidal => 0.40,
        drives::ToothForm::Circular => 0.50,
        _ => 0.46,
    };
    let tooth_angle = 2.0 * PI / teeth as f64;
    let groove_half = tooth_angle * groove_fraction / 2.0;
    let root_half = if trapezoidal { groove_half * 0.72 } else { groove_half * 0.86 };
    let at = |r: f64, a: f64| [c.x + r * a.cos(), c.y + r * a.sin()];
    for tooth in 0..teeth {
        let centre = tooth as f64 * tooth_angle;
        let land_start = at(tip, centre + groove_half);
        let land_mid = at(tip, centre + tooth_angle / 2.0);
        let land_end = at(tip, centre + tooth_angle - groove_half);
        run(s, "sketch.arc.three_point", json!({"start": land_start, "end": land_end, "through": land_mid}))?;
        let next = centre + tooth_angle;
        let groove_in = at(root, next - root_half);
        let groove_out = at(root, next + root_half);
        let after = at(tip, next + groove_half);
        if trapezoidal {
            run(s, "sketch.line", json!({"points": [land_end, groove_in, groove_out, after]}))?;
        } else {
            run(s, "sketch.line", json!({"points": [land_end, groove_in]}))?;
            run(s, "sketch.arc.three_point", json!({"start": groove_in, "end": groove_out, "through": at(root, next)}))?;
            run(s, "sketch.line", json!({"points": [groove_out, after]}))?;
        }
    }
    Ok((pitch_radius, tip, root))
}

fn pulley(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.pulley";
    let standard = standard_of(cmd, p)?;
    let teeth = required(cmd, p, "teeth", 6.0..=300.0)? as usize;
    let width = required(cmd, p, "width", 0.5..=500.0)?;
    let bore = number(cmd, p, "bore_mm", Some(0.0), 0.0..=500.0)?.unwrap_or(0.0);
    let style = str_(p, "bore_style").unwrap_or("plain").to_ascii_lowercase();
    let flanges = bool_(p, "flanges").unwrap_or(false);
    let flange = number(cmd, p, "flange_mm", Some(1.5), 0.1..=50.0)?.unwrap_or(1.5);
    let plain = bool_(p, "plain_roller").unwrap_or(false);
    let roller = number(cmd, p, "roller_diameter_mm", Some(20.0), 1.0..=1000.0)?.unwrap_or(20.0);
    let c = centre_of(cmd, p)?;
    let mm = 1000.0;
    let (pitch_d, outside_d, root_d) = if plain {
        (roller, roller, roller)
    } else {
        (drives::pitch_diameter(standard, teeth) * mm, drives::outside_diameter(standard, teeth) * mm, drives::root_diameter(standard, teeth) * mm)
    };
    if bore > 0.0 && bore >= root_d {
        return Err(bad(cmd, "the bore is wider than the pulley's groove root"));
    }
    let (key_w, key_d) = (
        number(cmd, p, "key_width_mm", Some(2.0), 0.1..=50.0)?.unwrap_or(2.0),
        number(cmd, p, "key_depth_mm", Some(1.0), 0.1..=50.0)?.unwrap_or(1.0),
    );
    if style == "keyway" && bore > 0.0 && bore / 2.0 + key_d >= root_d / 2.0 {
        return Err(bad(cmd, "the keyway would break through the groove root"));
    }
    let name = str_(p, "name")
        .map(str::to_string)
        .unwrap_or_else(|| if plain { format!("Roller Ø{roller}") } else { format!("Pulley {} {teeth}T", standard.name) });
    in_component(s, &name, |s| {
        if plain {
            run(s, "solid.cylinder", json!({"diameter": roller, "height": width, "base": c, "operation": "new"}))?;
        } else {
            run(s, "sketch.create", json!({"plane": "XY", "offset": c.z}))?;
            draw_pulley_outline(s, standard, teeth, c)?;
            run(s, "sketch.finish", json!({}))?;
            run(s, "solid.extrude", json!({"distance": width, "operation": "new"}))?;
        }
        if flanges {
            let thickness = (1.5_f64).min(width / 4.0);
            for base in [c.z - thickness, c.z + width] {
                run(
                    s,
                    "solid.cylinder",
                    json!({"diameter": outside_d + 2.0 * flange, "height": thickness, "base": [c.x, c.y, base], "operation": "join"}),
                )?;
            }
        }
        if bore > 0.0 {
            run(s, "solid.cylinder", json!({"diameter": bore, "height": width + 4.0, "base": [c.x, c.y, c.z - 2.0], "operation": "cut"}))?;
            match style.as_str() {
                "keyway" => {
                    run(
                        s,
                        "solid.box",
                        json!({"length": key_w, "width": bore / 2.0 + key_d, "height": width + 4.0, "corner": [c.x - key_w / 2.0, c.y, c.z - 2.0], "operation": "cut"}),
                    )?;
                }
                "set_screw" => {
                    let screw = (3.0_f64).min(bore / 2.0);
                    run(
                        s,
                        "solid.cylinder",
                        json!({"diameter": screw, "height": root_d / 2.0 + 2.0, "base": [c.x, c.y, c.z + width / 2.0], "axis": [0, 1, 0], "operation": "cut"}),
                    )?;
                }
                "plain" => {}
                other => return Err(bad(cmd, format!("`bore_style` is plain, keyway or set_screw, not `{other}`"))),
            }
        }
        Ok(
            json!({"pitch_diameter_mm": pitch_d, "outside_diameter_mm": outside_d, "root_diameter_mm": root_d, "standard": standard.name, "teeth": teeth}),
        )
    })
}

/// The wheels a belt or chain runs over, from the call (mm → m).
fn wheels_of(cmd: &str, p: &Value, standard: &drives::BeltStandard) -> Result<(Vec<Wheel>, Vec<usize>)> {
    let list = p
        .get("wheels")
        .and_then(Value::as_array)
        .filter(|a| a.len() >= 2 && a.len() <= 20)
        .ok_or_else(|| bad(cmd, "`wheels` is a list of 2 to 20 {centre, teeth | roller_diameter_mm, side?}"))?;
    let mut wheels = Vec::new();
    let mut teeth_of = Vec::new();
    for w in list {
        let centre = w.get("centre").and_then(crate::params::vec2).ok_or_else(|| bad(cmd, "each wheel needs `centre`: [x, y] in mm"))?;
        let side = match w.get("side") {
            None | Some(Value::Null) => 1,
            Some(v) => match v.as_i64() {
                Some(1) => 1,
                Some(-1) => -1,
                _ => return Err(bad(cmd, "`side` is 1 or -1")),
            },
        };
        let (radius, teeth) = if let Some(teeth) = number(cmd, w, "teeth", None, 6.0..=300.0)? {
            (drives::cord_radius_on_pulley(standard, teeth as usize), teeth as usize)
        } else {
            let d = number(cmd, w, "roller_diameter_mm", None, 1.0..=1000.0)?
                .ok_or_else(|| bad(cmd, "each wheel needs `teeth` or `roller_diameter_mm`"))?;
            (drives::cord_radius_on_back_idler(standard, d / 2000.0), 0)
        };
        wheels.push(Wheel { centre: Vec2::new(centre.x / 1000.0, centre.y / 1000.0), radius, side });
        teeth_of.push(teeth);
    }
    Ok((wheels, teeth_of))
}

fn belt(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.belt";
    let standard = standard_of(cmd, p)?;
    let width = required(cmd, p, "width", 0.5..=500.0)?;
    let (wheels, teeth_of) = wheels_of(cmd, p, standard)?;
    let z = number(cmd, p, "z", Some(0.0), -1e5..=1e5)?.unwrap_or(0.0);
    let path = drives::belt_path(&wheels, bool_(p, "allow_crossing").unwrap_or(false));
    if !path.valid {
        return Err(bad(cmd, path.problem));
    }
    let mm = 1000.0;
    let exact_teeth = path.length / standard.pitch;
    let mut out = json!({
        "standard": standard.name,
        "length_mm": path.length * mm,
        "teeth": exact_teeth.round() as i64,
        "teeth_exact": exact_teeth,
        "wraps_deg": path.wraps.iter().map(|w| w.to_degrees()).collect::<Vec<_>>(),
        "teeth_in_mesh": teeth_of.iter().zip(&path.wraps).map(|(t, w)| drives::teeth_in_mesh(*t, *w)).collect::<Vec<_>>(),
        "clearance_mm": path.clearance * mm,
        "crossings": path.crossings,
    });
    if !bool_(p, "build").unwrap_or(true) {
        return Ok(out);
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Belt {} {:.0} mm", standard.name, path.length * mm));
    let thickness = standard.belt_thickness * mm;
    let built = in_component(s, &name, |s| {
        // The loop as one sketch: span, then the arc on the next wheel, round the loop.
        let n = wheels.len();
        let sk = run(s, "sketch.create", json!({"plane": "XY", "offset": z}))?;
        let path_id = sk["sketch"].clone();
        let mut curves = Vec::new();
        for i in 0..n {
            let span = path.spans[i];
            let line = run(s, "sketch.line", json!({"points": [[span.from.x * mm, span.from.y * mm], [span.to.x * mm, span.to.y * mm]]}))?;
            curves.extend(curve_ids(&line));
            let next = wheels[(i + 1) % n];
            let wrap = path.wraps[(i + 1) % n];
            if wrap > 1e-9 {
                let arc = run(
                    s,
                    "sketch.arc.center_point",
                    json!({"center": [next.centre.x * mm, next.centre.y * mm], "start": [span.to.x * mm, span.to.y * mm], "sweep": next.side as f64 * wrap.to_degrees()}),
                )?;
                curves.extend(curve_ids(&arc));
            }
        }
        run(s, "sketch.finish", json!({}))?;
        // The section: a rectangle standing on edge across the cord at the first span's start.
        let first = path.spans[0];
        let along = Vec3::new(first.to.x - first.from.x, first.to.y - first.from.y, 0.0).normalized().unwrap_or(Vec3::X);
        let across = Vec3::new(-along.y, along.x, 0.0);
        let origin = Vec3::new(first.from.x * mm, first.from.y * mm, z);
        let profile = run(s, "sketch.create", json!({"plane": {"origin": origin, "x_dir": across, "y_dir": Vec3::Z}}))?;
        run(s, "sketch.rectangle.two_point", json!({"p0": [-thickness / 2.0, -width / 2.0], "p1": [thickness / 2.0, width / 2.0]}))?;
        run(s, "sketch.finish", json!({}))?;
        run(s, "solid.sweep", json!({"sketch": profile["sketch"], "path_sketch": path_id, "path": curves, "operation": "new"}))?;
        Ok(json!({}))
    })?;
    for key in ["component", "occurrence", "name"] {
        out[key] = built[key].clone();
    }
    Ok(out)
}

fn chain_of(cmd: &str, p: &Value) -> Result<&'static drives::ChainStandard> {
    let name = str_(p, "standard").ok_or_else(|| bad(cmd, "give `standard`: a chain such as 08B or ANSI 40"))?;
    drives::chain_standard(name).ok_or_else(|| bad(cmd, format!("`{name}` is not a chain this build knows")))
}

/// A sprocket as one closed outline (mm) at centre `c`: for every tooth gap the roller seat (an
/// arc of the seating radius centred on the pitch circle), the two flanks opening out to the tip
/// circle, and the tip land to the next gap; then one extrusion and the bore.
fn build_sprocket(s: &mut Session, chain: &drives::ChainStandard, teeth: usize, bore: f64, c: Vec3) -> Result<(f64, f64, f64, f64)> {
    let mm = 1000.0;
    let pitch_r = drives::sprocket_pitch_diameter(chain, teeth) / 2.0 * mm;
    let tip_r = drives::sprocket_tip_diameter(chain, teeth) / 2.0 * mm;
    // ISO 606 seating curve radius: 0.505 d1 + 0.069 ∛d1 (mm).
    let d1 = chain.roller_diameter * mm;
    let seat = 0.505 * d1 + 0.069 * d1.cbrt();
    let width = drives::sprocket_tooth_width(chain) * mm;
    let half_tooth = PI / teeth as f64;
    let seat_angle = seat.atan2(pitch_r);
    if seat_angle >= half_tooth {
        return Err(bad("design.sprocket", "too few teeth for this chain: the roller seats run into each other"));
    }
    // Half the mouth at the tip: past the seat's edge, leaving a land of about half the gap.
    let mouth = seat_angle + 0.45 * (half_tooth - seat_angle);
    let seat_r = pitch_r.hypot(seat);
    let at = |r: f64, a: f64| [c.x + r * a.cos(), c.y + r * a.sin()];
    run(s, "sketch.create", json!({"plane": "XY", "offset": c.z - width / 2.0}))?;
    for i in 0..teeth {
        let a = 2.0 * half_tooth * i as f64;
        let tip_in = at(tip_r, a - mouth);
        let seat_in = at(seat_r, a - seat_angle);
        let seat_out = at(seat_r, a + seat_angle);
        let tip_out = at(tip_r, a + mouth);
        run(s, "sketch.line", json!({"points": [tip_in, seat_in]}))?;
        run(s, "sketch.arc.three_point", json!({"start": seat_in, "end": seat_out, "through": at(pitch_r - seat, a)}))?;
        run(s, "sketch.line", json!({"points": [seat_out, tip_out]}))?;
        run(
            s,
            "sketch.arc.three_point",
            json!({"start": tip_out, "end": at(tip_r, a + 2.0 * half_tooth - mouth), "through": at(tip_r, a + half_tooth)}),
        )?;
    }
    run(s, "sketch.finish", json!({}))?;
    run(s, "solid.extrude", json!({"distance": width, "operation": "new"}))?;
    if bore > 0.0 {
        run(s, "solid.cylinder", json!({"diameter": bore, "height": width + 4.0, "base": [c.x, c.y, c.z - width / 2.0 - 2.0], "operation": "cut"}))?;
    }
    Ok((pitch_r * 2.0, tip_r * 2.0, drives::sprocket_root_diameter(chain, teeth) * mm, width))
}

fn sprocket(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.sprocket";
    let chain = chain_of(cmd, p)?;
    let teeth = required(cmd, p, "teeth", 7.0..=150.0)? as usize;
    let bore = number(cmd, p, "bore_mm", Some(0.0), 0.0..=500.0)?.unwrap_or(0.0);
    let c = centre_of(cmd, p)?;
    if bore > 0.0 && bore >= drives::sprocket_root_diameter(chain, teeth) * 1000.0 - 2.0 * chain.pitch * 1000.0 {
        return Err(bad(cmd, "the bore is too wide for the sprocket's rim"));
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Sprocket {} {teeth}T", chain.name));
    in_component(s, &name, |s| {
        let (pitch_d, tip_d, root_d, width) = build_sprocket(s, chain, teeth, bore, c)?;
        Ok(json!({"pitch_diameter_mm": pitch_d, "tip_diameter_mm": tip_d, "root_diameter_mm": root_d, "width_mm": width, "standard": chain.name}))
    })
}

fn chain_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.chain";
    let chain = chain_of(cmd, p)?;
    let list = p
        .get("sprockets")
        .and_then(Value::as_array)
        .filter(|a| a.len() == 2)
        .ok_or_else(|| bad(cmd, "`sprockets` is a list of two {centre: [x, y], teeth}"))?;
    let mut wheels = Vec::new();
    let mut teeth = Vec::new();
    for w in list {
        let centre = w.get("centre").and_then(crate::params::vec2).ok_or_else(|| bad(cmd, "each sprocket needs `centre`: [x, y] in mm"))?;
        let t = required(cmd, w, "teeth", 7.0..=150.0)? as usize;
        wheels.push(Wheel {
            centre: Vec2::new(centre.x / 1000.0, centre.y / 1000.0),
            radius: drives::sprocket_pitch_diameter(chain, t) / 2.0,
            side: 1,
        });
        teeth.push(t);
    }
    let z = number(cmd, p, "z", Some(0.0), -1e5..=1e5)?.unwrap_or(0.0);
    let path = drives::belt_path(&wheels, false);
    if !path.valid {
        return Err(bad(cmd, format!("no chain fits: {}", path.problem)));
    }
    let mm = 1000.0;
    let centres = (wheels[1].centre.x - wheels[0].centre.x).hypot(wheels[1].centre.y - wheels[0].centre.y);
    let fitted = drives::chain_links(chain, teeth[0], teeth[1], centres);
    let links = number(cmd, p, "links", Some(fitted as f64), 4.0..=400.0)?.unwrap_or(fitted as f64) as usize;
    let mut out = json!({
        "standard": chain.name,
        "links": links,
        "links_that_fit": fitted,
        "length_mm": links as f64 * chain.pitch * mm,
        "loop_length_mm": path.length * mm,
        "centre_distance_for_links_mm": drives::chain_centre_distance(chain, teeth[0], teeth[1], links) * mm,
    });
    if !bool_(p, "build").unwrap_or(true) {
        return Ok(out);
    }
    // The loop as arc and line pieces to walk along, rollers evenly round it, a plate pair across
    // every link, inner plates on the even links and outer on the odd, as a real chain alternates.
    struct Piece {
        arc: bool,
        a: Vec2,
        b: Vec2,
        centre: Vec2,
        radius: f64,
        start: f64,
        sweep: f64,
        length: f64,
    }
    let n = wheels.len();
    let mut pieces = Vec::new();
    for (i, w) in wheels.iter().enumerate() {
        let arrive = path.spans[(i + n - 1) % n];
        let leave = path.spans[i];
        let from = (arrive.to.y - w.centre.y).atan2(arrive.to.x - w.centre.x);
        pieces.push(Piece {
            arc: true,
            a: Vec2::default(),
            b: Vec2::default(),
            centre: w.centre,
            radius: w.radius,
            start: from,
            sweep: path.wraps[i] * w.side as f64,
            length: path.wraps[i] * w.radius,
        });
        pieces.push(Piece {
            arc: false,
            a: leave.from,
            b: leave.to,
            centre: Vec2::default(),
            radius: 0.0,
            start: 0.0,
            sweep: 0.0,
            length: leave.length,
        });
    }
    let total: f64 = pieces.iter().map(|p| p.length).sum();
    let point_at = |mut s: f64| -> Vec2 {
        s %= total;
        for piece in &pieces {
            if s <= piece.length {
                let t = if piece.length > 0.0 { (s / piece.length).min(1.0) } else { 0.0 };
                return if piece.arc {
                    let angle = piece.start + piece.sweep * t;
                    Vec2::new(piece.centre.x + piece.radius * angle.cos(), piece.centre.y + piece.radius * angle.sin())
                } else {
                    Vec2::new(piece.a.x + (piece.b.x - piece.a.x) * t, piece.a.y + (piece.b.y - piece.a.y) * t)
                };
            }
            s -= piece.length;
        }
        pieces.first().map(|p| p.a).unwrap_or_default()
    };
    let rollers: Vec<Vec2> = (0..links).map(|k| point_at(total * k as f64 / links as f64)).collect();
    let half_width = chain.inner_width / 2.0 * mm;
    let plate = 0.12 * chain.pitch * mm;
    let h = chain.plate_height / 2.0 * mm;
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Chain {} {links} links", chain.name));
    // A bought part with hundreds of pieces: one mesh body, a closed shell per roller and plate,
    // rather than a boolean per piece.
    let mut mesh = Mesh::default();
    let roller_outline = construction::circle(chain.roller_diameter / 2.0 * mm, 24);
    for (k, r) in rollers.iter().enumerate() {
        construction::push_prism(&mut mesh, &roller_outline, Vec3::new(r.x * mm, r.y * mm, z - half_width), chain.inner_width * mm);
        let q = rollers[(k + 1) % links];
        let (dx, dy) = (q.x - r.x, q.y - r.y);
        let length = dx.hypot(dy);
        if length < 1e-9 {
            continue;
        }
        let (ux, uy) = (dx / length, dy / length);
        // A plate: a stadium round the two pins, as a polygon.
        let mut outline: Vec<[f64; 2]> = Vec::new();
        for e in 0..2 {
            let centre = if e == 0 { q } else { *r };
            let base = uy.atan2(ux) + if e == 0 { -PI / 2.0 } else { PI / 2.0 };
            for j in 0..=12 {
                let a = base + PI * j as f64 / 12.0;
                outline.push([centre.x * mm + h * a.cos(), centre.y * mm + h * a.sin()]);
            }
        }
        let offset = half_width + if k % 2 == 0 { 0.0 } else { plate };
        for side in [-1.0, 1.0] {
            let bottom = if side > 0.0 { z + offset } else { z - offset - plate };
            construction::push_prism(&mut mesh, &outline, Vec3::new(0.0, 0.0, bottom), plate);
        }
    }
    let built = in_component(s, &name, |s| add_mesh_body(s, &name, mesh, cmd))?;
    for key in ["component", "occurrence", "name"] {
        out[key] = built[key].clone();
    }
    Ok(out)
}

fn tslot(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.tslot";
    let series = required(cmd, p, "series", 20.0..=40.0)? as u32;
    let length = required(cmd, p, "length", 1.0..=6000.0)?;
    let (outline, bore) = construction::tslot_outline(series).ok_or_else(|| bad(cmd, "`series` is 20, 30 or 40"))?;
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("T-slot {series}{series} x {length} mm"));
    in_component(s, &name, |s| {
        run(s, "sketch.create", json!({"plane": "XY"}))?;
        run(s, "sketch.line", json!({"points": outline, "closed": true}))?;
        run(s, "sketch.circle.center", json!({"center": [0.0, 0.0], "diameter": bore}))?;
        run(s, "sketch.finish", json!({}))?;
        // The outer region only: the bore circle makes the profile's island a hole.
        run(
            s,
            "solid.extrude",
            json!({"distance": length, "profiles": [{"point": [series as f64 / 4.0 - 1.0, series as f64 / 4.0 - 1.0]}], "operation": "new"}),
        )?;
        Ok(json!({"bore_mm": bore, "series": series, "length_mm": length}))
    })
}

/// NEMA ICS 16 frames: name, body across the flats, bolt square, bolt diameter, boss diameter,
/// boss height, shaft diameter, a common body length, a common shaft length (mm).
const STEPPER_FRAMES: [(&str, f64, f64, f64, f64, f64, f64, f64, f64); 5] = [
    ("NEMA 8", 20.3, 16.0, 2.5, 15.0, 1.5, 4.0, 30.0, 12.0),
    ("NEMA 11", 28.0, 23.0, 2.5, 22.0, 2.0, 5.0, 32.0, 18.0),
    ("NEMA 14", 35.2, 26.0, 3.0, 22.0, 2.0, 5.0, 34.0, 20.0),
    ("NEMA 17", 42.3, 31.0, 3.0, 22.0, 2.0, 5.0, 40.0, 24.0),
    ("NEMA 23", 56.4, 47.14, 5.0, 38.1, 1.6, 6.35, 51.0, 21.0),
];

fn stepper_motor(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.stepper_motor";
    let frame_name = str_(p, "frame").ok_or_else(|| bad(cmd, "give `frame`: NEMA 8, 11, 14, 17 or 23"))?;
    let frame =
        STEPPER_FRAMES.iter().find(|f| f.0.eq_ignore_ascii_case(frame_name.trim())).ok_or_else(|| bad(cmd, "`frame` is NEMA 8, 11, 14, 17 or 23"))?;
    let (name_f, body, bolt_square, bolt, boss_d, boss_h, shaft_d, default_len, default_shaft) = *frame;
    let body_length = number(cmd, p, "body_length_mm", Some(default_len), 5.0..=500.0)?.unwrap_or(default_len);
    let shaft_length = number(cmd, p, "shaft_length_mm", Some(default_shaft), 1.0..=500.0)?.unwrap_or(default_shaft);
    let flat = bool_(p, "shaft_flat").unwrap_or(true);
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{name_f} {body_length:.0} mm"));
    in_component(s, &name, |s| {
        // Front face at z = 0, body behind it, boss and shaft in front along +Z.
        run(
            s,
            "solid.box",
            json!({"length": body, "width": body, "height": body_length, "corner": [-body / 2.0, -body / 2.0, -body_length], "operation": "new"}),
        )?;
        run(s, "solid.cylinder", json!({"diameter": boss_d, "height": boss_h, "operation": "join"}))?;
        run(s, "solid.cylinder", json!({"diameter": shaft_d, "height": shaft_length, "base": [0.0, 0.0, boss_h], "operation": "join"}))?;
        if flat {
            // The flat a grub screw bites on: a slab off one side over the outer two thirds.
            let depth = shaft_d * 0.1;
            let from = boss_h + shaft_length / 3.0;
            run(
                s,
                "solid.box",
                json!({"length": shaft_d, "width": shaft_d, "height": shaft_length - from + 1.0, "corner": [shaft_d / 2.0 - depth, -shaft_d / 2.0, from], "operation": "cut"}),
            )?;
        }
        Ok(
            json!({"frame": name_f, "body_mm": body, "bolt_square_mm": bolt_square, "bolt_mm": bolt, "boss_diameter_mm": boss_d, "boss_height_mm": boss_h, "shaft_diameter_mm": shaft_d, "body_length_mm": body_length, "shaft_length_mm": shaft_length}),
        )
    })
}

fn snap_hook(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.snap_hook";
    let width = required(cmd, p, "width", 0.2..=500.0)?;
    let arm = required(cmd, p, "arm_length", 0.2..=500.0)?;
    let thickness = required(cmd, p, "thickness", 0.1..=100.0)?;
    let depth = required(cmd, p, "hook_depth", 0.1..=100.0)?;
    let hook = required(cmd, p, "hook_length", 0.1..=100.0)?;
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| "Snap hook".to_string());
    in_component(s, &name, |s| {
        // The arm along X, up Y, extruded along Z: a vertical catch face dropping below the arm
        // and an engagement ramp back up to the tip.
        run(s, "sketch.create", json!({"plane": "XY"}))?;
        run(
            s,
            "sketch.line",
            json!({"points": [[0.0, 0.0], [arm, 0.0], [arm, -depth], [arm + hook, 0.0], [arm + hook, thickness], [0.0, thickness]], "closed": true}),
        )?;
        run(s, "sketch.finish", json!({}))?;
        run(s, "solid.extrude", json!({"distance": width, "operation": "new"}))?;
        Ok(json!({"arm_length_mm": arm, "thickness_mm": thickness}))
    })
}

/// Standard brass heat-set inserts: size, hole (insert outer) diameter, boss diameter, bore depth (mm).
const INSERT_BOSSES: [(&str, f64, f64, f64); 6] =
    [("M2", 3.2, 6.4, 4.8), ("M2.5", 3.5, 7.0, 6.5), ("M3", 4.0, 8.0, 7.2), ("M4", 5.6, 11.2, 8.9), ("M5", 6.4, 12.8, 10.3), ("M6", 8.0, 16.0, 13.5)];

fn insert_boss(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.insert_boss";
    let size = str_(p, "size").unwrap_or("M3");
    let (size_name, hole, boss, depth) =
        *INSERT_BOSSES.iter().find(|r| r.0.eq_ignore_ascii_case(size.trim())).ok_or_else(|| bad(cmd, "`size` is M2, M2.5, M3, M4, M5 or M6"))?;
    let height = required(cmd, p, "height", 0.5..=200.0)?;
    if height < depth + 1.0 {
        return Err(bad(cmd, format!("the boss must be at least {} mm tall for an {size_name} insert", depth + 1.0)));
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Insert boss {size_name}"));
    in_component(s, &name, |s| {
        run(s, "solid.cylinder", json!({"diameter": boss, "height": height, "operation": "new"}))?;
        run(s, "solid.cylinder", json!({"diameter": hole, "height": depth + 1.0, "base": [0, 0, height - depth], "operation": "cut"}))?;
        Ok(json!({"size": size_name, "boss_diameter_mm": boss, "hole_diameter_mm": hole, "bore_depth_mm": depth}))
    })
}

fn bamboo_culm(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.bamboo_culm";
    let length = required(cmd, p, "length", 1.0..=10000.0)?;
    let butt = required(cmd, p, "butt_diameter", 1.0..=1000.0)?;
    let tip = number(cmd, p, "tip_diameter", Some(butt), 1.0..=1000.0)?.unwrap_or(butt);
    let butt_wall = required(cmd, p, "butt_wall", 0.1..=500.0)?;
    let tip_wall = number(cmd, p, "tip_wall", Some(butt_wall), 0.1..=500.0)?.unwrap_or(butt_wall);
    let spacing = number(cmd, p, "node_spacing", Some(250.0), 0.0..=10000.0)?.unwrap_or(250.0);
    let first = number(cmd, p, "first_node", Some(spacing / 2.0), 0.0..=10000.0)?.unwrap_or(spacing / 2.0);
    let collar = number(cmd, p, "node_collar", Some(1.5), 0.0..=100.0)?.unwrap_or(1.5);
    let collar_h = number(cmd, p, "node_collar_height", Some(8.0), 0.1..=500.0)?.unwrap_or(8.0);
    let diaphragm = number(cmd, p, "diaphragm", Some(4.0), 0.0..=500.0)?.unwrap_or(4.0);
    if tip > butt + 1e-9 {
        return Err(bad(cmd, "the tip cannot be fatter than the butt: bamboo tapers one way"));
    }
    if spacing > 0.0 && spacing < 5.0 {
        return Err(bad(cmd, "`node_spacing` is 0 for no nodes, or at least 5 mm"));
    }
    if butt_wall >= butt / 2.0 || tip_wall >= tip / 2.0 {
        return Err(bad(cmd, "the wall must be thinner than the radius at both ends: a culm is a tube"));
    }
    let outer_at = |z: f64| butt / 2.0 + (tip / 2.0 - butt / 2.0) * z / length;
    let bore_at = |z: f64| outer_at(z) - (butt_wall + (tip_wall - butt_wall) * z / length);
    let mut nodes = Vec::new();
    if spacing > 0.0 {
        let mut z = first;
        while z < length - 1e-9 && nodes.len() < 1000 {
            if z > 1e-9 {
                nodes.push(z);
            }
            z += spacing;
        }
    }
    let half = collar_h / 2.0;
    let frustum = |r0: f64, r1: f64, h: f64| PI * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
    let wall_volume = frustum(butt / 2.0, tip / 2.0, length) - frustum(bore_at(0.0), bore_at(length), length);
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Bamboo culm Ø{butt} x {length}"));
    in_component(s, &name, |s| {
        // One revolved wall section: up the outside with the collars, back down the bore with the
        // diaphragms closing it at the nodes.
        let mut pts: Vec<[f64; 2]> = vec![[bore_at(0.0), 0.0], [outer_at(0.0), 0.0]];
        for &z in &nodes {
            if collar > 0.0 && half > 0.0 && z - half > 0.0 && z + half < length {
                pts.push([outer_at(z - half), z - half]);
                pts.push([outer_at(z) + collar, z]);
                pts.push([outer_at(z + half), z + half]);
            }
        }
        pts.push([outer_at(length), length]);
        pts.push([bore_at(length), length]);
        for &z in nodes.iter().rev() {
            let d = diaphragm / 2.0;
            if diaphragm > 0.0 && z - d > 0.0 && z + d < length {
                pts.push([bore_at(z + d), z + d]);
                pts.push([0.0, z + d]);
                pts.push([0.0, z - d]);
                pts.push([bore_at(z - d), z - d]);
            }
        }
        let points: Vec<[f64; 2]> = pts.iter().map(|[r, z]| [*r, -*z]).collect();
        run(s, "sketch.create", json!({"plane": "XZ"}))?;
        run(s, "sketch.line", json!({"points": points, "closed": true}))?;
        run(s, "sketch.finish", json!({}))?;
        run(s, "solid.revolve", json!({"axis": "y", "operation": "new"}))?;
        Ok(json!({"nodes": nodes.len(), "wall_volume_mm3": wall_volume, "length_mm": length}))
    })
}

/// Adds a closed mesh (mm) to the design as a mesh body.
pub(super) fn add_mesh_body(s: &mut Session, name: &str, mesh: Mesh, cmd: &str) -> Result<Value> {
    solidcraft_kernel::mesh_body(&mesh.positions, &mesh.triangles).map_err(|e| bad(cmd, format!("the built mesh is not usable: {e}")))?;
    let data = MeshData {
        name: name.to_string(),
        positions: mesh.positions.iter().flat_map(|p| [p.x, p.y, p.z]).collect(),
        triangles: mesh.triangles.iter().flatten().copied().collect(),
        color: None,
    };
    let kind = FeatureKind::MeshImport { file: "construction kit".into(), meshes: vec![data] };
    super::features::add_feature(s, &json!({"name": name}), kind)
}

fn brickify(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "construct.brickify";
    let st = s.world_state();
    let name = match p.get("body") {
        None | Some(Value::Null) => {
            let selected: Vec<String> =
                s.selection.iter().filter_map(|x| if let crate::Sel::Body { name } = x { Some(name.clone()) } else { None }).collect();
            let all: Vec<String> = st.bodies.iter().map(|b| b.name.clone()).collect();
            match (selected.as_slice(), all.as_slice()) {
                ([one], _) | (_, [one]) => one.clone(),
                _ => return Err(bad(cmd, "give `body` (or select one body)")),
            }
        }
        Some(Value::String(n)) => n.clone(),
        Some(_) => return Err(bad(cmd, "`body` must be a body name")),
    };
    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let brick = match p.get("brick") {
        None | Some(Value::Null) => construction::BrickKind::Brick,
        Some(Value::String(text)) => construction::BrickKind::from_name(text).ok_or_else(|| {
            bad(cmd, format!("`brick` is one of {}", construction::BRICK_KINDS.iter().map(|k| k.name()).collect::<Vec<_>>().join(", ")))
        })?,
        Some(_) => return Err(bad(cmd, "`brick` is a brick system's name")),
    };
    let options = KitOptions {
        use_bricks: bool_(p, "bricks").unwrap_or(true),
        use_profiles: bool_(p, "profiles").unwrap_or(true),
        use_rods: bool_(p, "rods").unwrap_or(true),
        use_bolts: bool_(p, "bolts").unwrap_or(true),
        brick,
        profile_series: hardware_size(cmd, p, "profile_series", &construction::PROFILE_SERIES, 20)?,
        min_profile_length: number(cmd, p, "min_profile_mm", Some(40.0), 20.0..=6000.0)?.unwrap_or(40.0),
        rod_size: hardware_size(cmd, p, "rod", &construction::ROD_SIZES, 8)?,
        bolt_size: hardware_size(cmd, p, "bolt", &construction::BOLT_SIZES, 4)?,
        ..Default::default()
    };
    let mesh = body.mesh();
    let plan = construction::constructify(&mesh, &options).ok_or_else(|| bad(cmd, "the body is empty or too big for the stud grid"))?;
    let mut out = json!({
        "body": name,
        "brick": plan.brick.name(),
        "profile_series": plan.profile_series,
        "rod": format!("M{}", plan.rod_size),
        "bolt": format!("M{}", plan.bolt_size),
        "bill_of_materials": plan.bill_of_materials,
        "elements": plan.elements.iter().map(|e| json!({"kind": format!("{:?}", e.kind).to_ascii_lowercase(), "name": e.name, "position_mm": e.position, "studs": [e.studs_x, e.studs_y], "length_mm": e.length})).collect::<Vec<_>>(),
        "cells_total": plan.cells_total,
        "cells_covered": plan.cells_covered,
        "coverage": plan.coverage,
        "summary": plan.summary(),
        "warnings": plan.warnings,
    });
    if bool_(p, "build").unwrap_or(true) && !plan.elements.is_empty() {
        let kit = construction::plan_mesh(&plan);
        let kit_name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{name} kit"));
        let added = add_mesh_body(s, &kit_name, kit, cmd)?;
        out["built"] = added;
        if bool_(p, "hide").unwrap_or(false) {
            let _ = run(s, "browser.visibility", json!({"items": [name], "visible": false}));
        }
    }
    Ok(out)
}

/// A catalogue size from `key`: a number (20, 8), or a string with the digits in it (`"2020"`
/// for the 20 series, `"M8"`, `"m10"`); `default` when absent. Anything not in `allowed` is an
/// error naming the choices.
fn hardware_size(cmd: &str, p: &Value, key: &str, allowed: &[u32], default: u32) -> Result<u32> {
    let choices = || allowed.iter().map(|a| a.to_string()).collect::<Vec<_>>().join(", ");
    let size = match p.get(key) {
        None | Some(Value::Null) => return Ok(default),
        Some(Value::Number(n)) => n.as_f64().filter(|v| v.fract() == 0.0 && *v >= 0.0 && *v < 1e6).map(|v| v as u32),
        Some(Value::String(text)) => {
            let digits: String = text.chars().filter(char::is_ascii_digit).collect();
            match digits.parse::<u32>() {
                // "2020", "3030", "4040": a series written as the profile's name.
                Ok(n) if digits.len() == 4 && n % 101 == 0 => Some(n / 101),
                Ok(n) => Some(n),
                Err(_) => None,
            }
        }
        Some(_) => None,
    };
    match size {
        Some(size) if allowed.contains(&size) => Ok(size),
        _ => Err(bad(cmd, format!("`{key}` is one of {}", choices()))),
    }
}

fn capital(s: &str) -> String {
    let mut c = s.chars();
    c.next().map(|f| f.to_uppercase().collect::<String>() + c.as_str()).unwrap_or_default()
}

#[cfg(test)]
#[path = "design_tests.rs"]
mod tests;
