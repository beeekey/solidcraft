//! More of the machine's parts: worm drives and crown gears (as mesh bodies, the way the helical
//! gears are), frames of structural profiles along lines, and pipe routes along waypoints. Each
//! one goes into its own component, drawn with ordinary sketches and features where the kernel
//! carries them and as a mesh body where a helicoid would take it minutes.

use std::f64::consts::PI;

use serde_json::{Value, json};
use solidcraft_analysis::construction;
use solidcraft_geom::{Mesh, Vec3};

use super::CommandSpec;
use super::design::{add_mesh_body, helical_gear_mesh_tip, in_component, number, required, run};
use crate::params::{bad, bool_, str_, vec3};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("design.worm", "Worm", worm).at("3DEEE", "GEARS").icon("worm").params(
        "module: mm (0.2 to 50); pitch_diameter: mm; length: mm; starts?: threads (default 1, up to 6); pressure_angle_deg? (default 20); \
         left_hand?: bool; bore_mm?; name? → a worm (the driving screw of a worm drive) as a mesh body in its own component along +Z from z = 0: \
         axial_pitch_mm (π m), lead_mm (starts × pitch), lead_angle_deg, tip_diameter_mm, root_diameter_mm. Mates with design.worm_wheel",
    ),
    CommandSpec::new("design.worm_wheel", "Worm Wheel", worm_wheel).at("3DEEE", "GEARS").icon("worm_wheel").params(
        "module: mm; teeth (6 to 400); worm_pitch_diameter: mm (the worm it runs with); face_width: mm; worm_starts? (default 1); \
         pressure_angle_deg? (default 20); left_hand?: bool; bore_mm?; name? → the wheel a worm drives, as a mesh body: helical teeth at the \
         worm's lead angle and the rim throated so the tips follow the worm (the throat arc's radius is the worm's pitch radius less a module): \
         lead_angle_deg, centre_distance_mm, pitch_diameter_mm, throat_diameter_mm (tip at the mid-face), ratio (teeth / starts). The face width \
         is capped at twice the throat arc's radius",
    ),
    CommandSpec::new("design.crown_gear", "Crown Gear", crown_gear).at("3DEEE", "GEARS").icon("crown_gear").params(
        "module: mm; teeth (8 to 400); face_width: mm (the toothed band, centred on the pitch circle); thickness: mm (the disc, more than 2.25 m); \
         pressure_angle_deg? (default 20); bore_mm?; name? → a crown (face) gear as a mesh body: teeth on the flat face of a disc, radial grooves \
         of constant width narrowing with depth at the pressure angle, driven by a spur pinion of the same module at right angles: \
         pitch_diameter_mm, inner_diameter_mm, outer_diameter_mm, tooth_depth_mm",
    ),
    CommandSpec::new("design.frame", "Frame", frame).at("3DEEE", "STRUCTURE").icon("frame").params(
        "members: [{from: [x,y,z], to: [x,y,z], rotation_deg?}] mm (1 to 200); profile: {family: square_tube | rect_tube | round_tube | round_rod | angle | \
         channel | i_beam | tslot, size?: \"40x40x3\" (tubes, angles: height x width x wall; round: diameter x wall; rod: diameter), \"UNP 50\" | \"UNP 80\" \
         (channel), \"IPE 80\" | \"IPE 100\" | \"IPE 120\" (I-beam), \"2020\" | \"3030\" | \"4040\" (T-slot), or a?: height, b?: width, t?: wall or web, \
         t2?: flange (mm) outright}; ends?: mitre (default: where exactly two members meet at an end they are cut on the bisecting plane) | butt \
         (every member ends square at its points); trim?: bool (default true: a member ending on another member's length is cut back to its face); \
         rotation_deg?: every profile turned about its own line; offset?: [u, v] mm, the profile shifted off its line; name? → the members as separate \
         bodies in one component, each profile centred on its line: members, total_length_mm, profile (label, area_mm2), joints: {mitred, butt, trimmed}",
    ),
    CommandSpec::new("design.fillet_weld", "Fillet Weld", fillet_weld).at("3DEEE", "STRUCTURE").icon("fillet_weld").params(
        "body: name; edge: [x, y, z] (a point on an inside corner's edge of that body) | {index}; leg_mm: the weld's leg (0.2 to 200); name? → a fillet \
         weld bead along the edge in its own component: a right triangle with its legs on the two faces that meet there, swept along the edge (straight \
         or curved, open or closed): length_mm, leg_mm, throat_mm, volume_mm3, faces. A convex edge (an outside corner) has no inside to fill and is refused",
    ),
    CommandSpec::new("design.pipe_route", "Pipe Route", pipe_route).at("3DEEE", "STRUCTURE").icon("pipe").params(
        "points: [[x,y,z]…] mm waypoints (2 to 100); outer_diameter: mm; wall?: mm (hollow; default solid); bend_radius?: mm to the centreline at \
         every corner (default twice the outer diameter, at least half of it); name? → a tube swept along the route, straight runs with tangent bends, \
         as a Pipe feature on a 3D sketch in its own component: centreline_mm, straight_mm, bends, bend_radius_mm, outer_diameter_mm, wall_mm. A \
         bend that does not fit between its neighbours is refused by its corner",
    ),
];

// ---------------------------------------------------------------------------------------------
// Lathe meshes: a ring of points at every height, quads between the rings, caps at the ends.

/// Quads between ring `a` and ring `b` (each `n` points round, counter-clockwise seen from +Z).
/// The face normal is the tangent crossed with (b − a); `flip` turns it over.
fn strip(mesh: &mut Mesh, a: u32, b: u32, n: u32, flip: bool, face: u32) {
    for i in 0..n {
        let j = (i + 1) % n;
        let (p, q, r, s) = if flip { (a + j, a + i, b + i, b + j) } else { (a + i, a + j, b + j, b + i) };
        mesh.triangles.push([p, q, r]);
        mesh.triangles.push([p, r, s]);
        mesh.tri_face.push(face);
        mesh.tri_face.push(face);
    }
}

/// A fan from the point `centre` to ring `a`: `up` makes it face +Z.
fn fan(mesh: &mut Mesh, centre: u32, a: u32, n: u32, up: bool, face: u32) {
    for i in 0..n {
        let j = (i + 1) % n;
        mesh.triangles.push(if up { [centre, a + i, a + j] } else { [centre, a + j, a + i] });
        mesh.tri_face.push(face);
    }
}

/// Pushes a ring of `n` points at height `z` with the radius `radius(angle)`; returns its start.
fn ring(mesh: &mut Mesh, n: u32, z: f64, radius: &dyn Fn(f64) -> f64) -> Option<u32> {
    let start = mesh.positions.len() as u32;
    for i in 0..n {
        let a = 2.0 * PI * f64::from(i) / f64::from(n);
        let r = radius(a);
        if !r.is_finite() || r <= 0.0 {
            return None;
        }
        mesh.positions.push(Vec3::new(r * a.cos(), r * a.sin(), z));
    }
    Some(start)
}

/// A turned body as one closed mesh: at each height in `zs` a ring of `samples` points at
/// `radius(z, angle)`, quads between neighbouring rings, and the ends closed to a bore of
/// diameter `bore` (or to the axis without one).
fn lathe_mesh(zs: &[f64], samples: u32, bore: f64, radius: &dyn Fn(f64, f64) -> f64) -> Option<Mesh> {
    if zs.len() < 2 || zs.len() > 4096 || !(3..=4096).contains(&samples) || !(bore >= 0.0) {
        return None;
    }
    let mut mesh = Mesh::default();
    let mut rings: Vec<(u32, Option<u32>)> = Vec::with_capacity(zs.len());
    for &z in zs {
        let outer = ring(&mut mesh, samples, z, &|a| radius(z, a))?;
        let inner = if bore > 0.0 {
            for i in 0..samples {
                if mesh.positions.get((outer + i) as usize).map(|p| p.x.hypot(p.y))? <= bore / 2.0 {
                    return None;
                }
            }
            Some(ring(&mut mesh, samples, z, &|_| bore / 2.0)?)
        } else {
            None
        };
        rings.push((outer, inner));
    }
    for w in 0..rings.len() - 1 {
        strip(&mut mesh, rings[w].0, rings[w + 1].0, samples, false, 0);
        if let (Some(lo), Some(hi)) = (rings[w].1, rings[w + 1].1) {
            strip(&mut mesh, lo, hi, samples, true, 1);
        }
    }
    let (first, last) = (rings[0], rings[rings.len() - 1]);
    match (first.1, last.1) {
        (Some(b0), Some(b1)) => {
            strip(&mut mesh, b0, first.0, samples, false, 2);
            strip(&mut mesh, b1, last.0, samples, true, 3);
        }
        _ => {
            let c = mesh.positions.len() as u32;
            mesh.positions.push(Vec3::new(0.0, 0.0, zs[0]));
            mesh.positions.push(Vec3::new(0.0, 0.0, zs[zs.len() - 1]));
            fan(&mut mesh, c, first.0, samples, false, 2);
            fan(&mut mesh, c + 1, last.0, samples, true, 3);
        }
    }
    mesh.normals.resize(mesh.positions.len(), Vec3::ZERO);
    Some(mesh)
}

/// The thread profile of a worm as its radius at height `z` and angle `a` (mm, radians): a
/// trapezoid between the tip and the root cylinders, the flanks at the pressure angle, the thread
/// as thick as the gap at the pitch cylinder.
struct WormProfile {
    axial_pitch: f64,
    lead: f64,
    pitch_r: f64,
    tip: f64,
    root: f64,
    slope: f64,
    /// Half the thread's thickness at the tip and at the root (mm).
    w_tip: f64,
    w_root: f64,
    hand: f64,
}

impl WormProfile {
    fn radius(&self, z: f64, a: f64) -> f64 {
        // Thread centres sit where z − (a / 2π)·lead is a multiple of the axial pitch.
        let along = z - self.hand * a / (2.0 * PI) * self.lead;
        let frac = along / self.axial_pitch;
        let d = (frac - frac.round()).abs() * self.axial_pitch;
        if d <= self.w_tip {
            self.tip
        } else if d >= self.w_root {
            self.root
        } else {
            self.pitch_r + (self.axial_pitch / 4.0 - d) / self.slope
        }
    }
}

fn worm(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.worm";
    let module = required(cmd, p, "module", 0.2..=50.0)?;
    let pitch_d = required(cmd, p, "pitch_diameter", 1.0..=2000.0)?;
    let length = required(cmd, p, "length", 1.0..=5000.0)?;
    let starts = number(cmd, p, "starts", Some(1.0), 1.0..=6.0)?.unwrap_or(1.0) as usize;
    let pressure = number(cmd, p, "pressure_angle_deg", Some(20.0), 10.0..=30.0)?.unwrap_or(20.0).to_radians();
    let bore = number(cmd, p, "bore_mm", Some(0.0), 0.0..=2000.0)?.unwrap_or(0.0);
    let left = bool_(p, "left_hand").unwrap_or(false);
    let axial_pitch = PI * module;
    let lead = starts as f64 * axial_pitch;
    let (pitch_r, tip, root) = (pitch_d / 2.0, pitch_d / 2.0 + module, pitch_d / 2.0 - 1.25 * module);
    if root <= 0.5 {
        return Err(bad(cmd, "the thread is deeper than the worm is thick: a bigger pitch diameter or a smaller module"));
    }
    if bore > 0.0 && bore / 2.0 >= root - 0.5 {
        return Err(bad(cmd, "the bore reaches the thread's root"));
    }
    let slope = pressure.tan();
    let profile = WormProfile {
        axial_pitch,
        lead,
        pitch_r,
        tip,
        root,
        slope,
        w_tip: axial_pitch / 4.0 - module * slope,
        w_root: axial_pitch / 4.0 + 1.25 * module * slope,
        hand: if left { -1.0 } else { 1.0 },
    };
    if profile.w_tip <= 0.02 * module {
        return Err(bad(cmd, "the thread comes to a point at the tip: a smaller pressure angle"));
    }
    let lead_angle = (lead / (PI * pitch_d)).atan();
    let stations = (length / (axial_pitch / 16.0)).ceil().clamp(2.0, 2000.0) as usize;
    let zs: Vec<f64> = (0..=stations).map(|k| length * k as f64 / stations as f64).collect();
    let samples = (96 * starts as u32).min(512);
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Worm m{module} {starts} start"));
    let mesh = lathe_mesh(&zs, samples, bore, &|z, a| profile.radius(z, a)).ok_or_else(|| bad(cmd, "the worm could not be meshed"))?;
    in_component(s, &name, |s| {
        add_mesh_body(s, &name, mesh, cmd)?;
        Ok(json!({
            "axial_pitch_mm": axial_pitch,
            "lead_mm": lead,
            "lead_angle_deg": lead_angle.to_degrees(),
            "starts": starts,
            "pitch_diameter_mm": pitch_d,
            "tip_diameter_mm": 2.0 * tip,
            "root_diameter_mm": 2.0 * root,
            "length_mm": length,
            "hand": if left { "left" } else { "right" },
            "mesh_body": true,
        }))
    })
}

/// The tooth's half angle at radius `r` of an involute gear: positive while the flanks are apart.
fn tooth_half_angle(module: f64, teeth: usize, pressure: f64, r: f64) -> f64 {
    let pitch = module * teeth as f64 / 2.0;
    let base = pitch * pressure.cos();
    let param = |r: f64| if r <= base { 0.0 } else { (r * r / (base * base) - 1.0).sqrt() };
    let inv = |t: f64| t - t.atan();
    PI / (2.0 * teeth as f64) + inv(param(pitch)) - inv(param(r))
}

fn worm_wheel(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.worm_wheel";
    let module = required(cmd, p, "module", 0.2..=50.0)?;
    let teeth = required(cmd, p, "teeth", 6.0..=400.0)? as usize;
    let worm_d = required(cmd, p, "worm_pitch_diameter", 1.0..=2000.0)?;
    let width = required(cmd, p, "face_width", 0.1..=2000.0)?;
    let starts = number(cmd, p, "worm_starts", Some(1.0), 1.0..=6.0)?.unwrap_or(1.0) as usize;
    let pressure = number(cmd, p, "pressure_angle_deg", Some(20.0), 10.0..=30.0)?.unwrap_or(20.0).to_radians();
    let bore = number(cmd, p, "bore_mm", Some(0.0), 0.0..=2000.0)?.unwrap_or(0.0);
    let left = bool_(p, "left_hand").unwrap_or(false);
    let pitch_r = module * teeth as f64 / 2.0;
    let lead = starts as f64 * PI * module;
    let lead_angle = (lead / (PI * worm_d)).atan();
    let centres = worm_d / 2.0 + pitch_r;
    let throat_r = worm_d / 2.0 - module;
    if throat_r <= 0.5 {
        return Err(bad(cmd, "the worm is too thin for this module: its pitch radius must exceed a module"));
    }
    if width > 2.0 * throat_r - 0.2 {
        return Err(bad(cmd, format!("the face is wider than the worm it wraps: face_width at most {:.2} mm", 2.0 * throat_r - 0.2)));
    }
    if bore > 0.0 && bore / 2.0 >= pitch_r - 1.25 * module - 0.5 {
        return Err(bad(cmd, "the bore is wider than the wheel's root"));
    }
    // The tips may follow the throat up to where the flanks would meet.
    let (mut lo, mut hi) = (pitch_r + module, pitch_r + 1.5 * module);
    for _ in 0..24 {
        let mid = 0.5 * (lo + hi);
        if tooth_half_angle(module, teeth, pressure, mid) > 3e-3 { lo = mid } else { hi = mid }
    }
    let tip_cap = lo;
    let tip_at = |u: f64| {
        let dz = (u - 0.5) * width;
        let on_throat = if dz.abs() >= throat_r { centres } else { centres - (throat_r * throat_r - dz * dz).sqrt() };
        on_throat.clamp(pitch_r + module, tip_cap)
    };
    let twist = width * lead_angle.tan() / pitch_r * if left { -1.0 } else { 1.0 };
    let twist_at = |u: f64| twist * u;
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Worm wheel m{module} z{teeth}"));
    let (mesh, outline) = helical_gear_mesh_tip(module, teeth, pressure, width, bore, &twist_at, Some(&tip_at))
        .ok_or_else(|| bad(cmd, "the wheel's teeth could not be built: too few teeth for this module"))?;
    in_component(s, &name, |s| {
        add_mesh_body(s, &name, mesh, cmd)?;
        Ok(json!({
            "lead_angle_deg": lead_angle.to_degrees(),
            "helix_angle_deg": lead_angle.to_degrees(),
            "centre_distance_mm": centres,
            "pitch_diameter_mm": 2.0 * outline.pitch,
            "throat_diameter_mm": 2.0 * (pitch_r + module),
            "rim_diameter_mm": 2.0 * tip_at(0.0),
            "root_diameter_mm": 2.0 * outline.root,
            "ratio": teeth as f64 / starts as f64,
            "worm_starts": starts,
            "hand": if left { "left" } else { "right" },
            "mesh_body": true,
        }))
    })
}

/// A crown gear as a closed mesh: the toothed band as a polar height field between the inner
/// and outer radii, the plain plateau inside it, the outer wall, the back and the bore.
fn crown_gear_mesh(module: f64, teeth: usize, pressure: f64, face_width: f64, thickness: f64, bore: f64) -> Option<Mesh> {
    let pitch_r = module * teeth as f64 / 2.0;
    let (inner, outer) = (pitch_r - face_width / 2.0, pitch_r + face_width / 2.0);
    let depth = 2.25 * module;
    if !(inner > bore / 2.0 + 0.5) || !(depth < thickness) || !(8..=400).contains(&teeth) {
        return None;
    }
    let slope = pressure.tan();
    let half_pitch = PI * module / 4.0;
    let (pitch_z, root_z) = (thickness - module, thickness - depth);
    if half_pitch + (root_z - pitch_z) * slope <= 1e-6 {
        return None;
    }
    let per_tooth = (6000 / teeth).clamp(8, 24) as u32;
    let n = per_tooth * teeth as u32;
    let height = |rho: f64, a: f64| -> f64 {
        let turns = a * teeth as f64 / (2.0 * PI);
        let delta = (turns - turns.round()) * 2.0 * PI / teeth as f64;
        let t = rho * delta.sin().abs();
        (pitch_z + (t - half_pitch) / slope).clamp(root_z, thickness)
    };
    const RINGS: usize = 6;
    let mut mesh = Mesh::default();
    let mut band: Vec<u32> = Vec::with_capacity(RINGS + 1);
    for i in 0..=RINGS {
        let rho = inner + (outer - inner) * i as f64 / RINGS as f64;
        let start = mesh.positions.len() as u32;
        for k in 0..n {
            let a = 2.0 * PI * f64::from(k) / f64::from(n);
            mesh.positions.push(Vec3::new(rho * a.cos(), rho * a.sin(), height(rho, a)));
        }
        band.push(start);
    }
    let plateau = ring(&mut mesh, n, thickness, &|_| inner - 0.02)?;
    let foot = ring(&mut mesh, n, 0.0, &|_| outer)?;
    for i in 0..RINGS {
        strip(&mut mesh, band[i], band[i + 1], n, true, 0);
    }
    strip(&mut mesh, band[0], plateau, n, false, 1);
    strip(&mut mesh, foot, band[RINGS], n, false, 2);
    if bore > 0.0 {
        let bore_top = ring(&mut mesh, n, thickness, &|_| bore / 2.0)?;
        let bore_foot = ring(&mut mesh, n, 0.0, &|_| bore / 2.0)?;
        strip(&mut mesh, bore_top, plateau, n, true, 3);
        strip(&mut mesh, bore_foot, foot, n, false, 4);
        strip(&mut mesh, bore_foot, bore_top, n, true, 5);
    } else {
        let c = mesh.positions.len() as u32;
        mesh.positions.push(Vec3::new(0.0, 0.0, thickness));
        mesh.positions.push(Vec3::new(0.0, 0.0, 0.0));
        fan(&mut mesh, c, plateau, n, true, 3);
        fan(&mut mesh, c + 1, foot, n, false, 4);
    }
    mesh.normals.resize(mesh.positions.len(), Vec3::ZERO);
    Some(mesh)
}

fn crown_gear(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.crown_gear";
    let module = required(cmd, p, "module", 0.2..=50.0)?;
    let teeth = required(cmd, p, "teeth", 8.0..=400.0)? as usize;
    let face_width = required(cmd, p, "face_width", 0.1..=2000.0)?;
    let thickness = required(cmd, p, "thickness", 0.1..=2000.0)?;
    let pressure = number(cmd, p, "pressure_angle_deg", Some(20.0), 10.0..=30.0)?.unwrap_or(20.0).to_radians();
    let bore = number(cmd, p, "bore_mm", Some(0.0), 0.0..=2000.0)?.unwrap_or(0.0);
    let pitch_r = module * teeth as f64 / 2.0;
    let (inner, outer) = (pitch_r - face_width / 2.0, pitch_r + face_width / 2.0);
    if inner <= bore / 2.0 + 0.5 {
        return Err(bad(cmd, "the toothed band reaches the centre (or the bore): too few teeth for this face width"));
    }
    if 2.25 * module >= thickness {
        return Err(bad(cmd, format!("the teeth are deeper than the disc is thick: thickness more than {:.2} mm", 2.25 * module)));
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Crown gear m{module} z{teeth}"));
    let mesh = crown_gear_mesh(module, teeth, pressure, face_width, thickness, bore)
        .ok_or_else(|| bad(cmd, "the flanks meet before the root: pressure angle too steep for this module"))?;
    in_component(s, &name, |s| {
        add_mesh_body(s, &name, mesh, cmd)?;
        Ok(json!({
            "pitch_diameter_mm": 2.0 * pitch_r,
            "inner_diameter_mm": 2.0 * inner,
            "outer_diameter_mm": 2.0 * outer,
            "tooth_depth_mm": 2.25 * module,
            "thickness_mm": thickness,
            "pinion": format!("a spur pinion of module {module} at right angles, its axis {pitch_r:.2} mm from the crown's"),
            "mesh_body": true,
        }))
    })
}

// ---------------------------------------------------------------------------------------------
// Frames

/// A structural section, centred on the member's line: closed polylines and circles (mm) in the
/// profile plane, a point inside the material, and its area.
struct Profile {
    label: String,
    /// The outer boundary: closed polylines and circles (radii).
    polygons: Vec<Vec<[f64; 2]>>,
    circles: Vec<f64>,
    /// The hollow inside it (a tube's bore), cut after the members are trimmed so that a trim
    /// by another member sees that member solid.
    hole_polygons: Vec<Vec<[f64; 2]>>,
    hole_circles: Vec<f64>,
    pick: [f64; 2],
    area: f64,
}

fn shoelace(points: &[[f64; 2]]) -> f64 {
    let n = points.len();
    (0..n)
        .map(|i| {
            let (a, b) = (points[i], points[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        .abs()
        / 2.0
}

fn centred(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let (lo_x, hi_x) = points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p[0]), hi.max(p[0])));
    let (lo_y, hi_y) = points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p[1]), hi.max(p[1])));
    let (cx, cy) = ((lo_x + hi_x) / 2.0, (lo_y + hi_y) / 2.0);
    points.iter().map(|p| [p[0] - cx, p[1] - cy]).collect()
}

const CHANNELS: [(&str, f64, f64, f64, f64); 2] = [("UNP 50", 50.0, 38.0, 5.0, 7.0), ("UNP 80", 80.0, 45.0, 6.0, 8.0)];
const I_BEAMS: [(&str, f64, f64, f64, f64); 3] =
    [("IPE 80", 80.0, 46.0, 3.8, 5.2), ("IPE 100", 100.0, 55.0, 4.1, 5.7), ("IPE 120", 120.0, 64.0, 4.4, 6.3)];

/// The numbers of a size such as "40x40x3" (also with × or spaces).
fn size_numbers(size: &str) -> Vec<f64> {
    size.split(|c: char| c == 'x' || c == 'X' || c == '×' || c.is_whitespace())
        .filter(|t| !t.is_empty())
        .filter_map(|t| t.parse::<f64>().ok())
        .collect()
}

fn profile_of(cmd: &str, p: &Value) -> Result<Profile> {
    let spec = p.get("profile").ok_or_else(|| bad(cmd, "give `profile`: {family, size} or {family, a, b, t, t2}"))?;
    let family = spec
        .get("family")
        .and_then(Value::as_str)
        .map(|f| f.trim().to_ascii_lowercase().replace([' ', '-'], "_"))
        .ok_or_else(|| bad(cmd, "`profile.family` is square_tube, rect_tube, round_tube, round_rod, angle, channel, i_beam or tslot"))?;
    let size = spec.get("size").and_then(Value::as_str).map(str::trim).unwrap_or("");
    let nums = size_numbers(size);
    let dim = |key: &str, from_size: Option<f64>| -> Result<Option<f64>> {
        match spec.get(key) {
            None | Some(Value::Null) => Ok(from_size),
            Some(v) => v
                .as_f64()
                .filter(|x| x.is_finite() && (0.1..=5000.0).contains(x))
                .map(Some)
                .ok_or_else(|| bad(cmd, format!("`profile.{key}` must be a size in mm"))),
        }
    };
    let need = |x: Option<f64>, what: &str| x.ok_or_else(|| bad(cmd, format!("`profile.size` or `profile.{what}` is needed for a {family}")));
    let wall_ok = |a: f64, t: f64| if t * 2.0 >= a - 0.2 { Err(bad(cmd, "the wall is thicker than half the section")) } else { Ok(()) };
    match family.as_str() {
        "square_tube" | "rect_tube" => {
            let square = family == "square_tube";
            let a = need(dim("a", nums.first().copied())?, "a")?;
            let b = if square { a } else { need(dim("b", nums.get(1).copied())?, "b")? };
            // A square tube is "40x40x3" or "40x3".
            let wall_index = if square && nums.len() == 2 { 1 } else { 2 };
            let t = need(dim("t", nums.get(wall_index).copied())?, "t")?;
            wall_ok(a.min(b), t)?;
            let outer = vec![[-b / 2.0, -a / 2.0], [b / 2.0, -a / 2.0], [b / 2.0, a / 2.0], [-b / 2.0, a / 2.0]];
            let inner = vec![[-b / 2.0 + t, -a / 2.0 + t], [b / 2.0 - t, -a / 2.0 + t], [b / 2.0 - t, a / 2.0 - t], [-b / 2.0 + t, a / 2.0 - t]];
            let area = shoelace(&outer) - shoelace(&inner);
            Ok(Profile {
                label: format!("{} {a}x{b}x{t}", if square { "Square tube" } else { "Rectangular tube" }),
                polygons: vec![outer],
                circles: Vec::new(),
                hole_polygons: vec![inner],
                hole_circles: Vec::new(),
                pick: [0.0, 0.0],
                area,
            })
        }
        "round_tube" => {
            let d = need(dim("a", nums.first().copied())?, "a")?;
            let t = need(dim("t", nums.get(1).copied())?, "t")?;
            wall_ok(d, t)?;
            Ok(Profile {
                label: format!("Round tube {d}x{t}"),
                polygons: Vec::new(),
                circles: vec![d / 2.0],
                hole_polygons: Vec::new(),
                hole_circles: vec![d / 2.0 - t],
                pick: [0.0, 0.0],
                area: PI * (d * d / 4.0 - (d / 2.0 - t) * (d / 2.0 - t)),
            })
        }
        "round_rod" | "rod" => {
            let d = need(dim("a", nums.first().copied())?, "a")?;
            Ok(Profile {
                label: format!("Round rod {d}"),
                polygons: Vec::new(),
                circles: vec![d / 2.0],
                hole_polygons: Vec::new(),
                hole_circles: Vec::new(),
                pick: [0.0, 0.0],
                area: PI * d * d / 4.0,
            })
        }
        "angle" => {
            let a = need(dim("a", nums.first().copied())?, "a")?;
            let b = dim("b", nums.get(1).copied())?.unwrap_or(a);
            let t = need(dim("t", nums.get(2).copied().or(nums.get(1).copied().filter(|_| nums.len() == 2)))?, "t")?;
            wall_ok(a.min(b), t)?;
            let points = centred(&[[0.0, 0.0], [b, 0.0], [b, t], [t, t], [t, a], [0.0, a]]);
            let pick = [points[0][0] + t / 2.0, points[0][1] + t / 2.0];
            Ok(Profile {
                label: format!("Angle {a}x{b}x{t}"),
                area: shoelace(&points),
                polygons: vec![points],
                circles: Vec::new(),
                hole_polygons: Vec::new(),
                hole_circles: Vec::new(),
                pick,
            })
        }
        "channel" | "u_channel" | "unp" => {
            let table = CHANNELS.iter().find(|c| c.0.eq_ignore_ascii_case(size) || size_numbers(size).first().is_some_and(|n| *n == c.1));
            let (h, w, tw, tf) = (
                need(dim("a", table.map(|c| c.1))?, "a")?,
                need(dim("b", table.map(|c| c.2))?, "b")?,
                need(dim("t", table.map(|c| c.3))?, "t")?,
                need(dim("t2", table.map(|c| c.4))?, "t2")?,
            );
            if tw >= w - 0.2 || 2.0 * tf >= h - 0.2 {
                return Err(bad(cmd, "the channel's web or flanges are thicker than the section"));
            }
            let points = centred(&[[0.0, 0.0], [w, 0.0], [w, tf], [tw, tf], [tw, h - tf], [w, h - tf], [w, h], [0.0, h]]);
            let pick = [points[0][0] + tw / 2.0, 0.0];
            Ok(Profile {
                label: table.map_or_else(|| format!("Channel {h}x{w}x{tw}"), |c| c.0.to_string()),
                area: shoelace(&points),
                polygons: vec![points],
                circles: Vec::new(),
                hole_polygons: Vec::new(),
                hole_circles: Vec::new(),
                pick,
            })
        }
        "i_beam" | "ipe" | "h_beam" => {
            let table = I_BEAMS.iter().find(|c| c.0.eq_ignore_ascii_case(size) || size_numbers(size).first().is_some_and(|n| *n == c.1));
            let (h, w, tw, tf) = (
                need(dim("a", table.map(|c| c.1))?, "a")?,
                need(dim("b", table.map(|c| c.2))?, "b")?,
                need(dim("t", table.map(|c| c.3))?, "t")?,
                need(dim("t2", table.map(|c| c.4))?, "t2")?,
            );
            if tw >= w - 0.2 || 2.0 * tf >= h - 0.2 {
                return Err(bad(cmd, "the beam's web or flanges are thicker than the section"));
            }
            let (x0, x1, x2, x3) = (-w / 2.0, -tw / 2.0, tw / 2.0, w / 2.0);
            let points = centred(&[
                [x0, 0.0],
                [x3, 0.0],
                [x3, tf],
                [x2, tf],
                [x2, h - tf],
                [x3, h - tf],
                [x3, h],
                [x0, h],
                [x0, h - tf],
                [x1, h - tf],
                [x1, tf],
                [x0, tf],
            ]);
            Ok(Profile {
                label: table.map_or_else(|| format!("I-beam {h}x{w}x{tw}"), |c| c.0.to_string()),
                area: shoelace(&points),
                polygons: vec![points],
                circles: Vec::new(),
                hole_polygons: Vec::new(),
                hole_circles: Vec::new(),
                pick: [0.0, 0.0],
            })
        }
        "tslot" | "t_slot" => {
            let series = dim("a", nums.first().map(|n| if *n >= 100.0 { (*n / 100.0).round() } else { *n }))?.unwrap_or(20.0) as u32;
            let (outline, bore) = construction::tslot_outline(series).ok_or_else(|| bad(cmd, "a T-slot profile is 2020, 3030 or 4040"))?;
            let f = series as f64;
            let area = shoelace(&outline) - PI * bore * bore / 4.0;
            Ok(Profile {
                label: format!("T-slot {series}{series}"),
                polygons: vec![outline],
                circles: Vec::new(),
                hole_polygons: Vec::new(),
                hole_circles: vec![bore / 2.0],
                pick: [f / 4.0 - 1.0, f / 4.0 - 1.0],
                area,
            })
        }
        _ => Err(bad(cmd, "`profile.family` is square_tube, rect_tube, round_tube, round_rod, angle, channel, i_beam or tslot")),
    }
}

/// A unit vector perpendicular to `d` that leans upward (or along +Y for a vertical member).
fn up_perpendicular(d: Vec3) -> Vec3 {
    let seed = if d.z.abs() < 0.9 { Vec3::Z } else { Vec3::Y };
    (seed - d * seed.dot(d)).normalized().unwrap_or(Vec3::X)
}

fn rotated_about(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    v * angle.cos() + axis.cross(v) * angle.sin() + axis * axis.dot(v) * (1.0 - angle.cos())
}

struct Member {
    from: Vec3,
    to: Vec3,
    d: Vec3,
    length: f64,
    x_dir: Vec3,
    y_dir: Vec3,
}

/// How much wider (mm) than the member itself the cutter is that trims a brace back to it.
const TRIM_CLEARANCE: f64 = 0.05;

/// One end of a member: how far past its point the prism must reach and the plane that cuts it.
#[derive(Clone, Copy)]
struct EndCut {
    overshoot: f64,
    /// The mitre plane's normal pointing into the waste (away from the member), if mitred.
    waste: Option<Vec3>,
}

fn frame(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.frame";
    let profile = profile_of(cmd, p)?;
    let list = p.get("members").and_then(Value::as_array).ok_or_else(|| bad(cmd, "`members` must list {from: [x,y,z], to: [x,y,z]}"))?;
    if list.is_empty() || list.len() > 200 {
        return Err(bad(cmd, "`members` holds 1 to 200 members"));
    }
    let rotation = number(cmd, p, "rotation_deg", Some(0.0), -360.0..=360.0)?.unwrap_or(0.0).to_radians();
    let offset = match p.get("offset") {
        None | Some(Value::Null) => [0.0, 0.0],
        Some(v) => {
            let o: Option<Vec<f64>> = v.as_array().filter(|a| a.len() == 2).map(|a| a.iter().filter_map(Value::as_f64).collect());
            match o {
                Some(o) if o.len() == 2 && o.iter().all(|x| x.is_finite() && x.abs() <= 1000.0) => [o[0], o[1]],
                _ => return Err(bad(cmd, "`offset` is [u, v] in mm (within 1000)")),
            }
        }
    };
    let mitre = match str_(p, "ends").map(str::to_ascii_lowercase).as_deref() {
        None | Some("mitre") | Some("miter") => true,
        Some("butt") => false,
        Some(_) => return Err(bad(cmd, "`ends` is mitre or butt")),
    };
    let trim = bool_(p, "trim").unwrap_or(true);
    let mut members = Vec::with_capacity(list.len());
    for (i, m) in list.iter().enumerate() {
        let from =
            m.get("from").and_then(vec3).filter(|v| v.len() <= 1e5).ok_or_else(|| bad(cmd, format!("member {} needs `from`: [x, y, z]", i + 1)))?;
        let to = m.get("to").and_then(vec3).filter(|v| v.len() <= 1e5).ok_or_else(|| bad(cmd, format!("member {} needs `to`: [x, y, z]", i + 1)))?;
        let length = (to - from).len();
        if !(1.0..=20000.0).contains(&length) {
            return Err(bad(cmd, format!("member {} must be 1 mm to 20 m long", i + 1)));
        }
        let turn = rotation + number(cmd, m, "rotation_deg", Some(0.0), -360.0..=360.0)?.unwrap_or(0.0).to_radians();
        let d = (to - from) / length;
        let y0 = up_perpendicular(d);
        let y_dir = rotated_about(y0, d, turn);
        let x_dir = y_dir.cross(d);
        members.push(Member { from, to, d, length, x_dir, y_dir });
    }
    // The outline points of the section, in the profile plane (offset applied).
    let mut outline: Vec<[f64; 2]> = profile.polygons.iter().flatten().copied().collect();
    for r in &profile.circles {
        outline.extend((0..24).map(|k| {
            let a = 2.0 * PI * k as f64 / 24.0;
            [r * a.cos(), r * a.sin()]
        }));
    }
    for q in &mut outline {
        q[0] += offset[0];
        q[1] += offset[1];
    }
    let reach = outline.iter().map(|q| q[0].hypot(q[1])).fold(0.0, f64::max);

    // Joints: member ends sharing a point.
    let mut ends: Vec<EndCut> = vec![EndCut { overshoot: 0.0, waste: None }; 2 * members.len()];
    let mut groups: Vec<Vec<(usize, usize)>> = Vec::new();
    for (i, m) in members.iter().enumerate() {
        for (e, pt) in [(0usize, m.from), (1usize, m.to)] {
            let found = groups.iter_mut().find(|g| {
                g.first().is_some_and(|&(j, f)| {
                    let q = if f == 0 { members[j].from } else { members[j].to };
                    (q - pt).len() <= 0.01
                })
            });
            match found {
                Some(g) => g.push((i, e)),
                None => groups.push(vec![(i, e)]),
            }
        }
    }
    let away = |i: usize, e: usize| if e == 0 { members[i].d } else { -members[i].d };
    let (mut mitred, mut butt) = (0usize, 0usize);
    for g in &groups {
        if g.len() == 1 {
            continue;
        }
        if g.len() != 2 || !mitre {
            butt += 1;
            continue;
        }
        let ((i, e), (j, f)) = (g[0], g[1]);
        let (a, b) = (away(i, e), away(j, f));
        let c = a.dot(b);
        if c.abs() > (20.0_f64).to_radians().cos() {
            butt += 1; // nearly straight on, or doubled up: nothing to mitre
            continue;
        }
        for &((k, end), x, y) in &[((i, e), a, b), ((j, f), b, a)] {
            let Some(n) = (x - y).normalized() else { continue };
            let m = &members[k];
            let xn = x.dot(n);
            // The end section's points, carried along the member to the mitre plane.
            let mut t_min = 0.0_f64;
            for q in &outline {
                let rel = m.x_dir * q[0] + m.y_dir * q[1];
                let t = -(rel.dot(n)) / xn;
                t_min = t_min.min(t);
            }
            ends[2 * k + end] = EndCut { overshoot: -t_min + 1.0, waste: Some(-n) };
        }
        mitred += 1;
    }

    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| "Frame".to_string());
    let body_names: Vec<String> = (0..members.len()).map(|i| format!("{name} member {}", i + 1)).collect();
    let total: f64 = members.iter().map(|m| m.length).sum();
    let label = profile.label.clone();
    let area = profile.area;
    in_component(s, &name, |s| {
        for (i, m) in members.iter().enumerate() {
            let (start, end) = (ends[2 * i], ends[2 * i + 1]);
            let origin = m.from - m.d * start.overshoot;
            run(
                s,
                "sketch.create",
                json!({"plane": {"origin": [origin.x, origin.y, origin.z], "x_dir": [m.x_dir.x, m.x_dir.y, m.x_dir.z], "y_dir": [m.y_dir.x, m.y_dir.y, m.y_dir.z]}}),
            )?;
            for poly in &profile.polygons {
                let pts: Vec<[f64; 2]> = poly.iter().map(|q| [q[0] + offset[0], q[1] + offset[1]]).collect();
                run(s, "sketch.line", json!({"points": pts, "closed": true}))?;
            }
            for r in &profile.circles {
                run(s, "sketch.circle.center", json!({"center": [offset[0], offset[1]], "diameter": 2.0 * r}))?;
            }
            run(s, "sketch.finish", json!({}))?;
            run(
                s,
                "solid.extrude",
                json!({"distance": m.length + start.overshoot + end.overshoot, "profiles": [{"point": [profile.pick[0] + offset[0], profile.pick[1] + offset[1]]}], "operation": "new", "body_name": body_names[i]}),
            )?;
            for (cut, joint) in [(start, m.from), (end, m.to)] {
                let Some(w) = cut.waste else { continue };
                let u = up_perpendicular(w);
                let v = w.cross(u);
                let half = (reach + 1.0) / w.dot(m.d).abs().max(0.17) + 1.0;
                run(
                    s,
                    "sketch.create",
                    json!({"plane": {"origin": [joint.x, joint.y, joint.z], "x_dir": [u.x, u.y, u.z], "y_dir": [v.x, v.y, v.z]}}),
                )?;
                run(s, "sketch.rectangle.two_point", json!({"p0": [-half, -half], "p1": [half, half]}))?;
                run(s, "sketch.finish", json!({}))?;
                run(s, "solid.extrude", json!({"distance": cut.overshoot + reach + 2.0, "operation": "cut", "targets": [body_names[i]]}))?;
            }
        }
        // A member ending on another member's length is cut back to that member's face (the
        // members are still solid here, so the cut leaves nothing inside a bore).
        let mut trimmed = 0usize;
        if trim {
            for (i, m) in members.iter().enumerate() {
                for pt in [m.from, m.to] {
                    for (j, other) in members.iter().enumerate() {
                        if i == j {
                            continue;
                        }
                        let along = (pt - other.from).dot(other.d);
                        let off = ((pt - other.from) - other.d * along).len();
                        if off <= 0.01 && along > 1.0 && along < other.length - 1.0 {
                            // The cutter is the other member grown by a hair: a brace as wide as
                            // the member it lands on has its sides tangent to that member, and a
                            // boolean between tangent surfaces has no clean answer.
                            let grow = 1.0 + TRIM_CLEARANCE / reach.max(1.0);
                            let (start, end) = (ends[2 * j], ends[2 * j + 1]);
                            let origin = other.from - other.d * (start.overshoot + 1.0);
                            run(
                                s,
                                "sketch.create",
                                json!({"plane": {"origin": [origin.x, origin.y, origin.z], "x_dir": [other.x_dir.x, other.x_dir.y, other.x_dir.z], "y_dir": [other.y_dir.x, other.y_dir.y, other.y_dir.z]}}),
                            )?;
                            for poly in &profile.polygons {
                                let pts: Vec<[f64; 2]> = poly.iter().map(|q| [q[0] * grow + offset[0], q[1] * grow + offset[1]]).collect();
                                run(s, "sketch.line", json!({"points": pts, "closed": true}))?;
                            }
                            for r in &profile.circles {
                                run(s, "sketch.circle.center", json!({"center": [offset[0], offset[1]], "diameter": 2.0 * (r + TRIM_CLEARANCE)}))?;
                            }
                            run(s, "sketch.finish", json!({}))?;
                            let tool = format!("{name} trim tool");
                            run(
                                s,
                                "solid.extrude",
                                json!({"distance": other.length + start.overshoot + end.overshoot + 2.0, "profiles": [{"point": [profile.pick[0] * grow + offset[0], profile.pick[1] * grow + offset[1]]}], "operation": "new", "body_name": tool}),
                            )?;
                            run(s, "solid.combine", json!({"target": body_names[i], "tools": [tool], "operation": "cut", "keep_tools": false}))?;
                            trimmed += 1;
                        }
                    }
                }
            }
        }
        // The bores, last.
        if !profile.hole_polygons.is_empty() || !profile.hole_circles.is_empty() {
            for (i, m) in members.iter().enumerate() {
                let (start, end) = (ends[2 * i], ends[2 * i + 1]);
                let origin = m.from - m.d * (start.overshoot + 1.0);
                run(
                    s,
                    "sketch.create",
                    json!({"plane": {"origin": [origin.x, origin.y, origin.z], "x_dir": [m.x_dir.x, m.x_dir.y, m.x_dir.z], "y_dir": [m.y_dir.x, m.y_dir.y, m.y_dir.z]}}),
                )?;
                for poly in &profile.hole_polygons {
                    let pts: Vec<[f64; 2]> = poly.iter().map(|q| [q[0] + offset[0], q[1] + offset[1]]).collect();
                    run(s, "sketch.line", json!({"points": pts, "closed": true}))?;
                }
                for r in &profile.hole_circles {
                    run(s, "sketch.circle.center", json!({"center": [offset[0], offset[1]], "diameter": 2.0 * r}))?;
                }
                run(s, "sketch.finish", json!({}))?;
                run(
                    s,
                    "solid.extrude",
                    json!({"distance": m.length + start.overshoot + end.overshoot + 2.0, "operation": "cut", "targets": [body_names[i]]}),
                )?;
            }
        }
        Ok(json!({
            "members": members.len(),
            "total_length_mm": total,
            "profile": {"label": label, "area_mm2": area},
            "joints": {"mitred": mitred, "butt": butt, "trimmed": trimmed},
            "bodies": body_names,
        }))
    })
}

// ---------------------------------------------------------------------------------------------
// Pipe routes

fn pipe_route(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.pipe_route";
    let od = required(cmd, p, "outer_diameter", 0.5..=2000.0)?;
    let wall = number(cmd, p, "wall", None, 0.05..=1000.0)?;
    if wall.is_some_and(|w| 2.0 * w >= od - 0.05) {
        return Err(bad(cmd, "the wall is thicker than half the outer diameter"));
    }
    let bend_r = number(cmd, p, "bend_radius", Some(2.0 * od), 0.1..=1e5)?.unwrap_or(2.0 * od);
    if bend_r < od / 2.0 {
        return Err(bad(cmd, format!("`bend_radius` is to the centreline and must be at least half the outer diameter ({:.2} mm)", od / 2.0)));
    }
    let raw = p.get("points").and_then(Value::as_array).ok_or_else(|| bad(cmd, "`points` must list [x, y, z] waypoints"))?;
    if raw.len() < 2 || raw.len() > 100 {
        return Err(bad(cmd, "`points` holds 2 to 100 waypoints"));
    }
    let mut points: Vec<Vec3> = Vec::with_capacity(raw.len());
    for v in raw {
        let q = vec3(v).filter(|q| q.len() <= 1e5).ok_or_else(|| bad(cmd, "a waypoint is [x, y, z] in mm"))?;
        if points.last().is_some_and(|last: &Vec3| (*last - q).len() < 0.01) {
            continue; // a repeated point
        }
        points.push(q);
    }
    if points.len() < 2 {
        return Err(bad(cmd, "the route needs two different points"));
    }
    // Corners: the tangent length each bend takes from its two legs.
    let mut tangents = vec![0.0; points.len()];
    let mut bends = 0usize;
    for i in 1..points.len() - 1 {
        let (Some(u), Some(v)) = ((points[i] - points[i - 1]).normalized(), (points[i + 1] - points[i]).normalized()) else { continue };
        let c = u.dot(v).clamp(-1.0, 1.0);
        let turn = c.acos();
        if turn < 1e-3 {
            continue; // straight through
        }
        if turn > PI - 0.05 {
            return Err(bad(cmd, format!("the route turns back on itself at point {}", i + 1)));
        }
        tangents[i] = bend_r * (turn / 2.0).tan();
        bends += 1;
    }
    for i in 0..points.len() - 1 {
        let leg = (points[i + 1] - points[i]).len();
        if tangents[i] + tangents[i + 1] > leg + 1e-9 {
            return Err(bad(
                cmd,
                format!(
                    "the bend radius does not fit: the leg from point {} to {} is {leg:.2} mm, the bends need {:.2} mm",
                    i + 1,
                    i + 2,
                    tangents[i] + tangents[i + 1]
                ),
            ));
        }
    }
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Pipe route Ø{od}"));
    let mut straight = 0.0;
    let mut arc_length = 0.0;
    in_component(s, &name, |s| {
        run(s, "sketch.create", json!({"plane": "XY"}))?;
        run(s, "sketch.options", json!({"sketch_3d": true}))?;
        let mut path: Vec<Value> = Vec::new();
        let mut cursor = points[0];
        for i in 0..points.len() - 1 {
            let (a, b) = (points[i], points[i + 1]);
            let dir = (b - a).normalized().unwrap_or(Vec3::X);
            let leg_end = b - dir * tangents[i + 1];
            if (leg_end - cursor).len() > 1e-6 {
                let r = run(s, "sketch.line3d", json!({"points": [[cursor.x, cursor.y, cursor.z], [leg_end.x, leg_end.y, leg_end.z]]}))?;
                path.extend(r["curves"].as_array().cloned().unwrap_or_default());
                straight += (leg_end - cursor).len();
            }
            cursor = leg_end;
            if i + 1 < points.len() - 1 && tangents[i + 1] > 0.0 {
                // The bend: from the end of this leg round to the start of the next.
                let c = points[i + 1];
                let out = (points[i + 2] - c).normalized().unwrap_or(dir);
                let start = c - dir * tangents[i + 1];
                let finish = c + out * tangents[i + 1];
                let turn = dir.dot(out).clamp(-1.0, 1.0).acos();
                let (Some(bisector), Some(axis)) = ((out - dir).normalized(), dir.cross(out).normalized()) else { continue };
                let centre = c + bisector * (bend_r / (turn / 2.0).cos());
                let r0 = start - centre;
                const ARC: usize = 8;
                let pts: Vec<[f64; 3]> = (0..=ARC)
                    .map(|k| {
                        let q = centre + rotated_about(r0, axis, turn * k as f64 / ARC as f64);
                        [q.x, q.y, q.z]
                    })
                    .collect();
                let r = run(s, "sketch.spline3d", json!({"points": pts}))?;
                path.extend(r["curves"].as_array().cloned().unwrap_or_default());
                arc_length += bend_r * turn;
                cursor = finish;
            }
        }
        let sk = run(s, "sketch.finish", json!({}))?;
        let mut pipe = json!({"path_sketch": sk["sketch"], "path": path, "diameter": od, "operation": "new"});
        if let Some(w) = wall {
            pipe["wall"] = json!(w);
        }
        run(s, "solid.pipe", pipe)?;
        Ok(json!({
            "centreline_mm": straight + arc_length,
            "straight_mm": straight,
            "bends": bends,
            "bend_radius_mm": bend_r,
            "outer_diameter_mm": od,
            "wall_mm": wall,
        }))
    })
}

// ---------------------------------------------------------------------------------------------
// Welds

/// Distance from `q` to the polyline `points`.
fn polyline_distance(points: &[Vec3], q: Vec3) -> f64 {
    let mut best = f64::INFINITY;
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let ab = b - a;
        let len2 = ab.len2();
        let t = if len2 > 1e-18 { ((q - a).dot(ab) / len2).clamp(0.0, 1.0) } else { 0.0 };
        best = best.min((q - (a + ab * t)).len());
    }
    if let Some(only) = points.first().filter(|_| points.len() == 1) {
        best = best.min((q - *only).len());
    }
    best
}

/// A face meeting an edge, from the display mesh.
struct FaceAtEdge {
    face: u32,
    /// Triangles along the edge.
    weight: usize,
    /// How near the reference point the triangle whose normal is kept sits.
    nearest: f64,
    normal: Vec3,
    /// Direction from the edge into the face, at that same triangle.
    inward: Vec3,
}

/// The faces whose triangles have two corners on the polyline, heaviest first; each face's
/// normal is the one of its triangle nearest `near`.
fn faces_at_edge(mesh: &Mesh, points: &[Vec3], tol: f64, near: Vec3) -> Vec<FaceAtEdge> {
    let start = near;
    let mut faces: Vec<FaceAtEdge> = Vec::new();
    for (t, face) in mesh.triangles.iter().zip(&mesh.tri_face) {
        let (Some(a), Some(b), Some(c)) = (mesh.positions.get(t[0] as usize), mesh.positions.get(t[1] as usize), mesh.positions.get(t[2] as usize))
        else {
            continue;
        };
        let on_edge = [a, b, c].iter().filter(|q| polyline_distance(points, ***q) <= tol * 2.0).count();
        if on_edge < 2 {
            continue;
        }
        let Some(normal) = (*b - *a).cross(*c - *a).normalized() else { continue };
        let centre = (*a + *b + *c) / 3.0;
        let nearest = (centre - start).len();
        let inward = centre - nearest_on_polyline(points, centre);
        match faces.iter_mut().find(|f| f.face == *face) {
            Some(f) => {
                f.weight += 1;
                if nearest < f.nearest {
                    f.nearest = nearest;
                    f.normal = normal;
                    f.inward = inward;
                }
            }
            None => faces.push(FaceAtEdge { face: *face, weight: 1, nearest, normal, inward }),
        }
    }
    faces.sort_by_key(|f| std::cmp::Reverse(f.weight));
    faces
}

/// The point of the polyline nearest `q`.
fn nearest_on_polyline(points: &[Vec3], q: Vec3) -> Vec3 {
    let mut best = (f64::INFINITY, q);
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let ab = b - a;
        let len2 = ab.len2();
        let t = if len2 > 1e-18 { ((q - a).dot(ab) / len2).clamp(0.0, 1.0) } else { 0.0 };
        let on = a + ab * t;
        let d = (q - on).len();
        if d < best.0 {
            best = (d, on);
        }
    }
    best.1
}

fn fillet_weld(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "design.fillet_weld";
    let leg = required(cmd, p, "leg_mm", 0.2..=200.0)?;
    let body_name = str_(p, "body").ok_or_else(|| bad(cmd, "give `body`: the body whose edge is welded"))?.to_string();
    let edge_spec = p.get("edge").ok_or_else(|| bad(cmd, "give `edge`: a point [x, y, z] on the edge, or {index}"))?;
    let st = s.world_state();
    let b = st.body(&body_name).ok_or_else(|| bad(cmd, format!("no body `{body_name}`")))?;
    let size = b.body.size();
    let tol = (size * 2e-3).max(1e-3);
    let edges = b.body.edges(tol)?;
    let edge = match (edge_spec.get("index").and_then(Value::as_u64), vec3(edge_spec).or_else(|| edge_spec.get("point").and_then(vec3))) {
        (Some(i), _) => edges.iter().find(|e| e.index == i as usize).ok_or_else(|| bad(cmd, format!("`{body_name}` has no edge {i}")))?,
        (None, Some(q)) => {
            let near = (size * 0.02).max(0.5);
            edges
                .iter()
                .filter(|e| e.points.len() >= 2)
                .map(|e| (polyline_distance(&e.points, q), e))
                .filter(|(d, _)| *d <= near)
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, e)| e)
                .ok_or_else(|| bad(cmd, "no edge of the body passes near that point"))?
        }
        _ => return Err(bad(cmd, "`edge` is a point [x, y, z] on the edge or {index}")),
    };
    if edge.points.len() < 2 || edge.length <= tol {
        return Err(bad(cmd, "the edge has no length to weld along"));
    }
    let mesh = b.mesh();
    let picked = faces_at_edge(&mesh, &edge.points, tol, edge.points[0]);
    if picked.len() < 2 {
        return Err(bad(cmd, "the edge needs two faces meeting at it (a corner)"));
    }
    // The kernel splits a round edge (and the faces either side) in two: chain the edges that
    // run on tangentially from either end between faces whose normals carry on, so the weld goes
    // right round.
    let mut points: Vec<Vec3> = edge.points.clone();
    let mut used = vec![edge.index];
    let mut extended = true;
    let join = tol * 4.0;
    let same_way = |a: Vec3, b: Vec3| a.normalized().zip(b.normalized()).is_some_and(|(a, b)| a.dot(b) > (15.0_f64).to_radians().cos());
    // The polyline's chords either side of a junction may differ by a facet's turn; a real
    // corner turns by far more.
    let runs_on = |a: Vec3, b: Vec3| a.normalized().zip(b.normalized()).is_some_and(|(a, b)| a.dot(b) > (45.0_f64).to_radians().cos());
    let carries_on = |ours: &[FaceAtEdge], theirs: &[FaceAtEdge]| -> bool {
        let (Some(a), Some(b), Some(c), Some(d)) = (ours.first(), ours.get(1), theirs.first(), theirs.get(1)) else { return false };
        (same_way(a.normal, c.normal) && same_way(b.normal, d.normal)) || (same_way(a.normal, d.normal) && same_way(b.normal, c.normal))
    };
    while extended && used.len() < 64 {
        extended = false;
        let (Some(first), Some(last)) = (points.first().copied(), points.last().copied()) else { break };
        let (Some(second), Some(penultimate)) = (points.get(1).copied(), points.get(points.len().wrapping_sub(2)).copied()) else { break };
        if (first - last).len() <= join {
            break; // closed
        }
        for other in edges.iter().filter(|e| !used.contains(&e.index) && e.points.len() >= 2) {
            let (Some(o_first), Some(o_last)) = (other.points.first().copied(), other.points.last().copied()) else { continue };
            let (Some(o_second), Some(o_penultimate)) = (other.points.get(1).copied(), other.points.get(other.points.len() - 2).copied()) else {
                continue;
            };
            let at_end =
                |junction: Vec3| carries_on(&faces_at_edge(&mesh, &points, tol, junction), &faces_at_edge(&mesh, &other.points, tol, junction));
            if (o_first - last).len() <= join && runs_on(last - penultimate, o_second - o_first) && at_end(last) {
                points.extend(other.points.iter().skip(1).copied());
            } else if (o_last - last).len() <= join && runs_on(last - penultimate, o_penultimate - o_last) && at_end(last) {
                points.extend(other.points.iter().rev().skip(1).copied());
            } else if (o_last - first).len() <= join && runs_on(second - first, o_last - o_penultimate) && at_end(first) {
                let mut head = other.points.clone();
                head.pop();
                head.extend(points.iter().copied());
                points = head;
            } else if (o_first - first).len() <= join && runs_on(second - first, o_first - o_second) && at_end(first) {
                let mut head: Vec<Vec3> = other.points.iter().rev().copied().collect();
                head.pop();
                head.extend(points.iter().copied());
                points = head;
            } else {
                continue;
            }
            used.push(other.index);
            extended = true;
            break;
        }
    }
    let faces = faces_at_edge(&mesh, &points, tol, points[0]);
    let (Some(a), Some(bq)) = (faces.first(), faces.get(1)) else { return Err(bad(cmd, "the edge needs two faces meeting at it (a corner)")) };
    let (n1, n2, f1, f2) = (a.normal, bq.normal, a.face, bq.face);
    if n1.dot(n2).abs() > (10.0_f64).to_radians().cos() {
        return Err(bad(cmd, "the two faces are nearly flush: no corner to fill"));
    }
    let start = points[0];
    let Some(tangent) = (points[1] - points[0]).normalized() else { return Err(bad(cmd, "the edge has no direction")) };
    // Each leg lies on its face, pointing from the edge into the face. At an inside corner each
    // leg heads toward the other face's outside; at an outside corner it heads away from it.
    let into = |inward: Vec3| (inward - tangent * inward.dot(tangent)).normalized();
    let (Some(leg1), Some(leg2)) = (into(a.inward), into(bq.inward)) else { return Err(bad(cmd, "the corner's faces are not oriented")) };
    if leg1.dot(n2) <= 0.05 || leg2.dot(n1) <= 0.05 {
        return Err(bad(cmd, "that is an outside corner: a fillet weld fills an inside corner"));
    }
    let length: f64 = points.windows(2).map(|w| (w[1] - w[0]).len()).sum();
    let edge_points = points;
    let y_dir = tangent.cross(leg1);
    let c = [leg * leg2.dot(leg1), leg * leg2.dot(y_dir)];
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Fillet weld {leg} mm"));
    let path_points: Vec<[f64; 3]> = {
        let n = edge_points.len();
        let step = n.div_ceil(400).max(1);
        let mut pts: Vec<[f64; 3]> = edge_points.iter().step_by(step).map(|q| [q.x, q.y, q.z]).collect();
        if let Some(last) = edge_points.last() {
            let l = [last.x, last.y, last.z];
            if pts.last() != Some(&l) {
                pts.push(l);
            }
        }
        pts
    };
    in_component(s, &name, |s| {
        run(
            s,
            "sketch.create",
            json!({"plane": {"origin": [start.x, start.y, start.z], "x_dir": [leg1.x, leg1.y, leg1.z], "y_dir": [y_dir.x, y_dir.y, y_dir.z]}}),
        )?;
        run(s, "sketch.line", json!({"points": [[0.0, 0.0], [leg, 0.0], c], "closed": true}))?;
        let profile = run(s, "sketch.finish", json!({}))?;
        run(s, "sketch.create", json!({"plane": "XY"}))?;
        run(s, "sketch.options", json!({"sketch_3d": true}))?;
        let curve = if path_points.len() == 2 {
            run(s, "sketch.line3d", json!({"points": path_points}))?
        } else {
            run(s, "sketch.spline3d", json!({"points": path_points}))?
        };
        let path = run(s, "sketch.finish", json!({}))?;
        run(s, "solid.sweep", json!({"sketch": profile["sketch"], "path_sketch": path["sketch"], "path": curve["curves"], "operation": "new"}))?;
        Ok(json!({
            "length_mm": length,
            "leg_mm": leg,
            "throat_mm": leg / 2f64.sqrt(),
            "volume_mm3": leg * leg / 2.0 * length,
            "faces": [f1, f2],
            "edges": used,
        }))
    })
}

#[cfg(test)]
#[path = "design_frames_tests.rs"]
mod tests;
