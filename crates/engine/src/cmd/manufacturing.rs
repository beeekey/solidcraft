//! Can this part be made: 3D printing (`inspect.print`, `inspect.print_orient`) and machining
//! (`inspect.machining`), measured from the body's mesh and faces by [`solidcraft_analysis`].

use serde_json::{Value, json};
use solidcraft_analysis::machining::{
    self, AccessGrid, Finding, Hole, InternalCorner, MachiningSetup, Material, Process, RevolutionReport, Severity, ShoulderFillet,
};
use solidcraft_analysis::print_analysis::{self, InfillPattern, PrintSettings};
use solidcraft_analysis::voxelise;
use solidcraft_geom::{Mesh, UnitVec3, Vec3};
use solidcraft_kernel::FaceSurface;

use super::CommandSpec;
use crate::params::{bad, str_};
use crate::{Result, Sel, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("inspect.print", "Print Analysis", print).at("3DEEE", "MAKE").icon("print_analysis").noundo().params(
        "body?: name (default: the selected body, or the design's only one); up?: [x, y, z] the build direction (default +Z); critical_angle_deg?: \
         overhang threshold from the horizontal (10 to 85, default 45); build_volume_mm?: [x, y, z] the printer (default 220 × 220 × 250); \
         layer_height_mm? (0.05 to 1, default 0.2); material?: the filament (PLA, PLA+, PETG, ABS, ASA, TPU 95A; default PLA); flow_rate_mm3_s? \
         (default 8); layer_overhead_s? (default 1); max_bridge_mm? (default 20); infill?: 0 to 1 (default 0.2); pattern?: Grid | Lines | \
         Rectilinear | Triangles | Tri-hexagon | Cubic | Gyroid | Honeycomb | Concentric | Cross-hatch | Lightning; walls?: perimeters (default 2) \
         → watertight, fits_build_volume, height_mm, layers, volume_mm3, overhang_area_mm2, support_contact_area_mm2, support_volume_mm3 (an upper \
         bound), material_grams, time_s, thinnest_wall_mm, thinnest_wall_at, bridged_area_mm2, longest_bridge_mm, printed_material: {young_modulus_in_plane_pa, \
         young_modulus_across_layers_pa, tensile_in_plane_mpa, tensile_interlayer_mpa, solid_fraction, mass_grams, note}, verdict, warnings",
    ),
    CommandSpec::new("inspect.print_orient", "Best Print Orientation", print_orient).at("3DEEE", "MAKE").icon("print_orient").noundo().params(
        "body?: name; critical_angle_deg?, build_volume_mm?, layer_height_mm? as inspect.print → options: the six axis directions best first, each \
         {direction, up, fits_build_volume, overhang_area_mm2, support_contact_area_mm2, height_mm, layers}; best; verdict",
    ),
    CommandSpec::new("inspect.machining", "Machining Analysis", machining_cmd).at("3DEEE", "MAKE").icon("machining").noundo().params(
        "body?: name; process?: \"auto\" (default: turning when the part is a body of revolution, else milling) | \"milling\" | \"turning\"; \
         material?: Aluminium | Steel | Plastic (default: from the body's material, else Aluminium); cutter_mm? (1 to 50, default 6); \
         min_corner_radius_mm? (default 1.5); corner_clearance? (default 1.2); blind_drill_ratio? (5), through_drill_ratio? (10); setup_warn_at? (3); \
         resolution?: cells along the longest side for the tool-access scan (8 to 120, default 40); turning_axis?: [x, y, z] (default: the \
         longest side) → assumed: {process, material, cutter_mm, …}, access: {setups, setup_axes, machined_faces, surface_faces, \
         unreachable_fraction, unreachable_regions: [{faces, area_mm2, at}], trapped_volume_mm3, trapped_at}, thinnest_wall_mm, thinnest_wall_at, \
         holes: [{diameter_mm, depth_mm, through, axis, at}], corners: [{radius_mm, depth_mm, at}], revolution: {is_body_of_revolution, length_mm, \
         max_diameter_mm, slenderness, off_axis_volume_mm3, flats, cross_holes}, findings: [{severity, rule, detail, measured, limit, unit, at}] \
         worst first, refusals, warnings_count, notes, caveats (what was not recognised, said out loud), text, verdict, warnings. Holes and corner \
         radii come from the cylindrical faces; sharp inside corners and pocket floors are not recognised yet and the caveats say so",
    ),
];

fn number(cmd: &str, p: &Value, key: &str, default: f64, range: std::ops::RangeInclusive<f64>) -> Result<f64> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v
            .as_f64()
            .filter(|x| x.is_finite() && range.contains(x))
            .ok_or_else(|| bad(cmd, format!("`{key}` must be a number from {} to {}", range.start(), range.end()))),
    }
}

fn body_name(s: &Session, st: &solidcraft_doc::ModelState, p: &Value, cmd: &str) -> Result<String> {
    let name = match p.get("body") {
        None | Some(Value::Null) => {
            let selected: Vec<String> =
                s.selection.iter().filter_map(|x| if let Sel::Body { name } = x { Some(name.clone()) } else { None }).collect();
            let all: Vec<String> = st.bodies.iter().map(|b| b.name.clone()).collect();
            match (selected.as_slice(), all.as_slice()) {
                ([one], _) | (_, [one]) => one.clone(),
                _ => return Err(bad(cmd, "give `body` (or select one body)")),
            }
        }
        Some(Value::String(n)) => n.clone(),
        Some(_) => return Err(bad(cmd, "`body` must be a body name")),
    };
    if st.body(&name).is_none() {
        return Err(bad(cmd, format!("no body `{name}`")));
    }
    Ok(name)
}

struct PrintParams {
    up: Vec3,
    critical: f64,
    build_volume: Vec3,
    layer_height: f64,
}

fn print_params(cmd: &str, p: &Value) -> Result<PrintParams> {
    let up = match p.get("up") {
        None | Some(Value::Null) => Vec3::Z,
        Some(v) => crate::params::vec3(v).and_then(|u| u.normalized()).ok_or_else(|| bad(cmd, "`up` is a direction [x, y, z]"))?,
    };
    let critical = number(cmd, p, "critical_angle_deg", 45.0, 10.0..=85.0)?.to_radians();
    let build_volume = match p.get("build_volume_mm") {
        None | Some(Value::Null) => Vec3::new(220.0, 220.0, 250.0),
        Some(v) => crate::params::vec3(v)
            .filter(|b| b.x > 0.0 && b.y > 0.0 && b.z > 0.0)
            .ok_or_else(|| bad(cmd, "`build_volume_mm` is [x, y, z] in mm, all positive"))?,
    };
    let layer_height = number(cmd, p, "layer_height_mm", 0.2, 0.05..=1.0)?;
    Ok(PrintParams { up, critical, build_volume, layer_height })
}

fn print(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.print";
    let started = web_time::Instant::now();
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let params = print_params(cmd, p)?;
    let material = str_(p, "material").unwrap_or("PLA");
    if material.len() > 64 {
        return Err(bad(cmd, "`material` is a filament name"));
    }
    let filament = print_analysis::filament_named(material);
    let flow_rate = number(cmd, p, "flow_rate_mm3_s", 8.0, 0.1..=1000.0)?;
    let overhead = number(cmd, p, "layer_overhead_s", 1.0, 0.0..=600.0)?;
    let max_bridge = number(cmd, p, "max_bridge_mm", 20.0, 0.0..=1000.0)?;
    let infill = number(cmd, p, "infill", 0.2, 0.0..=1.0)?;
    let walls = number(cmd, p, "walls", 2.0, 0.0..=20.0)? as u32;
    let pattern = match str_(p, "pattern") {
        None => InfillPattern::Grid,
        Some(name) => {
            InfillPattern::from_name(name).ok_or_else(|| bad(cmd, "`pattern` is one of the slicer's infill patterns (Grid, Gyroid, Cubic, …)"))?
        }
    };
    if p.get("mode").and_then(Value::as_str).is_some_and(|m| !m.is_empty()) {
        return Err(bad(cmd, "`mode` is not an option of this command"));
    }
    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let mesh = body.mesh();
    let report = print_analysis::analyze(&mesh, params.up, params.critical, params.build_volume, params.layer_height);
    let cost = print_analysis::estimate(&mesh, params.up, params.critical, params.layer_height, filament.density, flow_rate, overhead);
    let wall = print_analysis::thinnest_wall(&mesh);
    let bridged = print_analysis::bridges(&mesh, params.up, max_bridge);
    let settings = PrintSettings {
        material: filament.name.to_string(),
        pattern,
        infill_density: infill,
        wall_loops: walls,
        layer_height: params.layer_height,
        ..Default::default()
    };
    let printed = print_analysis::effective_material_of(&settings, &mesh);
    let mut warnings: Vec<String> = Vec::new();
    if !report.watertight {
        warnings.push("the mesh is not watertight: a slicer will see open edges".into());
    }
    if !report.fits_build_volume {
        warnings.push(format!(
            "does not fit the {:.0} × {:.0} × {:.0} mm build volume in any orientation",
            params.build_volume.x, params.build_volume.y, params.build_volume.z
        ));
    }
    if let Some((t, _)) = wall
        && t < 2.0 * settings.line_width
    {
        warnings.push(format!("the thinnest wall is {t:.2} mm, under two line widths: it will not print as a wall"));
    }
    let verdict = format!(
        "{}{:.0} mm² needs support ({:.0} mm³ at most), {} layers, about {:.0} g and {:.0} min",
        if report.overhang_area_mm2 > 0.0 { "" } else { "prints without support: " },
        report.overhang_area_mm2,
        cost.support_volume_mm3,
        report.layer_count,
        cost.material_grams,
        cost.time_seconds / 60.0
    );
    Ok(json!({
        "body": name,
        "up": params.up,
        "watertight": report.watertight,
        "fits_build_volume": report.fits_build_volume,
        "height_mm": report.height_mm,
        "layers": report.layer_count,
        "volume_mm3": report.volume_mm3,
        "overhang_area_mm2": report.overhang_area_mm2,
        "support_contact_area_mm2": report.support_contact_area_mm2,
        "support_volume_mm3": cost.support_volume_mm3,
        "material_grams": cost.material_grams,
        "time_s": cost.time_seconds,
        "thinnest_wall_mm": wall.map(|w| w.0),
        "thinnest_wall_at": wall.map(|w| w.1),
        "bridged_area_mm2": bridged.bridged_area_mm2,
        "longest_bridge_mm": bridged.longest_span_mm,
        "printed_material": {
            "filament": filament.name,
            "pattern": pattern.name(),
            "infill": infill,
            "walls": walls,
            "young_modulus_in_plane_pa": printed.material.map(|m| m.ex),
            "young_modulus_across_layers_pa": printed.material.map(|m| m.ez),
            "tensile_in_plane_mpa": printed.tensile_in_plane / 1e6,
            "tensile_interlayer_mpa": printed.tensile_interlayer / 1e6,
            "solid_fraction": printed.solid_fraction,
            "shell_fraction": printed.shell_fraction,
            "mass_grams": printed.mass_grams,
            "note": printed.note,
        },
        "verdict": verdict,
        "warnings": warnings,
        "ms": started.elapsed().as_millis() as u64,
    }))
}

fn print_orient(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.print_orient";
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let params = print_params(cmd, p)?;
    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let options = print_analysis::advise_orientation(&body.mesh(), params.critical, params.build_volume, params.layer_height);
    let best = options.first().map(|o| o.name).unwrap_or("+Z");
    let verdict = match options.first() {
        Some(o) if o.report.overhang_area_mm2 <= 0.0 => format!("print {best} up: no support needed"),
        Some(o) => format!("print {best} up: {:.0} mm² of overhang, the least of the six", o.report.overhang_area_mm2),
        None => "no mesh to orient".into(),
    };
    Ok(json!({
        "body": name,
        "options": options.iter().map(|o| json!({
            "direction": o.name, "up": o.build_direction, "fits_build_volume": o.report.fits_build_volume,
            "overhang_area_mm2": o.report.overhang_area_mm2, "support_contact_area_mm2": o.report.support_contact_area_mm2,
            "height_mm": o.report.height_mm, "layers": o.report.layer_count,
        })).collect::<Vec<_>>(),
        "best": best,
        "verdict": verdict,
    }))
}

/// The angular coverage of a cylindrical face's triangles about its axis, in 36 sectors: every
/// sector a triangle's edge sweeps across counts, not only the ones its corners land in, so a
/// coarse tessellation (a dozen facets round a small bore) still reads as a full cylinder.
fn coverage(mesh: &Mesh, triangles: &[usize], origin: Vec3, axis: Vec3) -> usize {
    let u = axis.any_perp();
    let v = axis.cross(u);
    let mut sectors = [false; 36];
    let angle_of = |p: Vec3| {
        let r = p - origin;
        let r = r - axis * r.dot(axis);
        r.dot(v).atan2(r.dot(u))
    };
    let sector_of = |angle: f64| (((angle + std::f64::consts::PI) / (2.0 * std::f64::consts::PI) * 36.0) as usize).min(35);
    for &t in triangles {
        let Some(ps) = mesh.triangles.get(t).and_then(|t| mesh.tri(t)) else { continue };
        let angles = [angle_of(ps[0]), angle_of(ps[1]), angle_of(ps[2])];
        for (a, b) in [(0, 1), (1, 2), (2, 0)] {
            let (from, to) = (angles[a], angles[b]);
            let mut sweep = to - from;
            if sweep > std::f64::consts::PI {
                sweep -= 2.0 * std::f64::consts::PI;
            } else if sweep < -std::f64::consts::PI {
                sweep += 2.0 * std::f64::consts::PI;
            }
            // An edge that spans half a turn is a seam or a cap, not a sweep round the wall.
            if sweep.abs() >= std::f64::consts::PI - 1e-9 {
                continue;
            }
            let steps = ((sweep.abs() / (2.0 * std::f64::consts::PI) * 36.0).ceil() as usize).clamp(1, 36);
            for k in 0..=steps {
                let angle = from + sweep * k as f64 / steps as f64;
                let wrapped = (angle + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI;
                sectors[sector_of(wrapped)] = true;
            }
        }
    }
    sectors.iter().filter(|s| **s).count()
}

/// Holes and inside corners from the body's cylindrical faces, judged with the mesh.
fn cylinders(
    mesh: &Mesh,
    spots: &[Option<solidcraft_doc::naming::FaceSpot>],
    tool_axis: Vec3,
) -> (Vec<Hole>, Vec<InternalCorner>, Vec<ShoulderFillet>, usize) {
    let index = mesh.inside_index();
    let inside = |p: Vec3| index.contains(p);
    let mut holes = Vec::new();
    let mut corners = Vec::new();
    let mut fillets = Vec::new();
    let mut cross = 0usize;
    // The kernel splits a cylinder into faces (two halves, more after cuts): judge the whole
    // cylinder, so the faces of one bore are gathered by axis, radius and position first.
    let mut groups: Vec<(Vec3, UnitVec3, f64, Vec<usize>)> = Vec::new();
    let mut others: Vec<(usize, Vec<usize>)> = Vec::new();
    for (face, spot) in spots.iter().enumerate() {
        let Some(spot) = spot else { continue };
        let tris: Vec<usize> = mesh.tri_face.iter().enumerate().filter(|(_, f)| **f as usize == face).map(|(t, _)| t).collect();
        if tris.is_empty() {
            continue;
        }
        if let Some(FaceSurface::Cylinder { origin, axis, radius }) = spot.surface
            && radius > 0.0
            && radius.is_finite()
        {
            let foot = origin - axis * origin.dot(axis);
            let tol = (radius * 1e-3).max(1e-6);
            if let Some(g) = groups.iter_mut().find(|(o, a, r, _)| {
                (a.dot(axis).abs() - 1.0).abs() < 1e-6 && (r - radius).abs() < tol && ((*o - axis * o.dot(axis)) - foot).len() < tol
            }) {
                g.3.extend(tris);
            } else {
                groups.push((origin, axis, radius, tris));
            }
        } else {
            others.push((face, tris));
        }
    }
    let cylinder_faces: Vec<(Option<&solidcraft_doc::naming::FaceSpot>, Vec<usize>, Option<FaceSurface>)> = groups
        .into_iter()
        .map(|(origin, axis, radius, tris)| (None, tris, Some(FaceSurface::Cylinder { origin, axis, radius })))
        .chain(others.into_iter().map(|(face, tris)| (spots[face].as_ref(), tris, spots[face].as_ref().and_then(|s| s.surface))))
        .collect();
    for (_spot, tris, surface) in cylinder_faces {
        match surface {
            Some(FaceSurface::Cylinder { origin, axis, radius }) => {
                let Some(axis) = axis.normalized() else { continue };
                if !(radius > 0.0) || !radius.is_finite() {
                    continue;
                }
                let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
                for &t in &tris {
                    if let Some(ps) = mesh.triangles.get(t).and_then(|t| mesh.tri(t)) {
                        for p in ps {
                            let s = (p - origin).dot(axis);
                            lo = lo.min(s);
                            hi = hi.max(s);
                        }
                    }
                }
                if !(hi > lo) {
                    continue;
                }
                let on_axis = |s: f64| origin + axis * s;
                let mid = on_axis(0.5 * (lo + hi));
                let hole_like = !inside(mid);
                if !hole_like {
                    continue; // a boss, not a bore
                }
                let full = coverage(mesh, &tris, origin, axis) >= 34;
                let depth = hi - lo;
                if full {
                    let probe = (0.5 * radius).max(0.01);
                    let open_low = !inside(on_axis(lo - probe));
                    let open_high = !inside(on_axis(hi + probe));
                    let through = open_low && open_high;
                    let entry_at = if open_high || !open_low { on_axis(hi) } else { on_axis(lo) };
                    if axis.dot(tool_axis).abs() < 0.1 {
                        cross += 1;
                    }
                    holes.push(Hole {
                        axis_point: mid,
                        axis,
                        diameter: 2.0 * radius,
                        depth,
                        through,
                        entry_at,
                        entry_face_curved: false,
                        entry_deviation_degrees: 0.0,
                        flat_bottom: false,
                        thinnest_surround: 0.0,
                        broken_into: false,
                    });
                } else if axis.dot(tool_axis).abs() > 0.99 {
                    let at =
                        tris.first().and_then(|t| mesh.triangles.get(*t)).and_then(|t| mesh.tri(t)).map(|[a, b, c]| (a + b + c) / 3.0).unwrap_or(mid);
                    corners.push(InternalCorner { at, radius, depth, sharp: false });
                }
            }
            Some(FaceSurface::Torus { center, minor, .. }) => {
                let at =
                    tris.first().and_then(|t| mesh.triangles.get(*t)).and_then(|t| mesh.tri(t)).map(|[a, b, c]| (a + b + c) / 3.0).unwrap_or(center);
                // A concave round sits with material behind its surface on the far side of its
                // tube: the tube's centre line is in the air.
                if !inside(center) {
                    fillets.push(ShoulderFillet { radius: minor, at });
                }
            }
            _ => {}
        }
    }
    (holes, corners, fillets, cross)
}

/// How round the part is about `axis` through the box's centre, from the mesh vertices.
fn revolution(mesh: &Mesh, axis: Vec3, flats: usize, cross_holes: usize) -> RevolutionReport {
    let bounds = mesh.bounds();
    let centre = bounds.center();
    let mut report = RevolutionReport { axis_point: centre, axis, flats, cross_holes, ..Default::default() };
    let volume = print_analysis::volume_of(mesh);
    if !(volume > 0.0) {
        return report;
    }
    report.actual_volume = volume;
    let stations: Vec<(f64, f64)> = mesh
        .positions
        .iter()
        .map(|p| {
            let r = *p - centre;
            let s = r.dot(axis);
            ((r - axis * s).len(), s)
        })
        .collect();
    let (lo, hi) = stations.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), (_, s)| (lo.min(*s), hi.max(*s)));
    if !(hi > lo) {
        return report;
    }
    let bins = 400usize;
    let step = (hi - lo) / bins as f64;
    let mut rmax = vec![0.0_f64; bins];
    for (r, s) in &stations {
        let b = (((s - lo) / step) as usize).min(bins - 1);
        rmax[b] = rmax[b].max(*r);
        // A vertex also bounds the neighbouring bin, so a shoulder is not read as a gap.
        if b + 1 < bins {
            rmax[b + 1] = rmax[b + 1].max(*r);
        }
    }
    // A tessellated cylinder has vertices only at its ends: carry the radius across bins no
    // vertex lands in by interpolating between the bounded bins either side.
    let mut filled: Vec<usize> = rmax.iter().enumerate().filter(|(_, r)| **r > 0.0).map(|(b, _)| b).collect();
    filled.dedup();
    for pair in filled.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if b > a + 1 {
            let (ra, rb) = (rmax[a], rmax[b]);
            for (k, bin) in (a + 1..b).enumerate() {
                let t = (k + 1) as f64 / (b - a) as f64;
                rmax[bin] = ra + (rb - ra) * t;
            }
        }
    }
    report.revolved_volume = rmax.iter().map(|r| std::f64::consts::PI * r * r * step).sum();
    report.off_axis_volume = (report.revolved_volume - volume).abs();
    report.length = hi - lo;
    report.max_diameter = 2.0 * rmax.iter().cloned().fold(0.0, f64::max);
    report.slenderness = if report.max_diameter > 0.0 { report.length / report.max_diameter } else { 0.0 };
    report.is_body_of_revolution = volume > 0.0 && report.off_axis_volume / volume < 0.02 && flats == 0;
    report
}

fn machining_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.machining";
    let started = web_time::Instant::now();
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let material_name = s.doc.materials.get(body.definition_name()).cloned().unwrap_or_else(|| "Default".to_string());
    let process = match str_(p, "process").unwrap_or("auto") {
        "auto" => Process::Auto,
        "milling" => Process::Milling3Axis,
        "turning" => Process::Turning,
        _ => return Err(bad(cmd, "`process` is \"auto\", \"milling\" or \"turning\"")),
    };
    let mut setup = MachiningSetup { process, ..MachiningSetup::default() };
    setup.material = match str_(p, "material") {
        Some(m) => Material::parse(m).ok_or_else(|| bad(cmd, "`material` is Aluminium, Steel or Plastic"))?,
        None => {
            let lower = material_name.to_ascii_lowercase();
            if lower.contains("steel") || lower.contains("iron") || lower.contains("titanium") {
                Material::Steel
            } else if lower.contains("plastic")
                || lower.contains("abs")
                || lower.contains("pla")
                || lower.contains("nylon")
                || lower.contains("polycarbonate")
            {
                Material::Plastic
            } else {
                Material::Aluminium
            }
        }
    };
    setup.cutter_diameter = number(cmd, p, "cutter_mm", 6.0, 1.0..=50.0)?;
    setup.min_corner_radius = number(cmd, p, "min_corner_radius_mm", 1.5, 0.1..=25.0)?;
    setup.corner_clearance = number(cmd, p, "corner_clearance", 1.2, 1.0..=3.0)?;
    setup.blind_drill_ratio = number(cmd, p, "blind_drill_ratio", 5.0, 1.0..=50.0)?;
    setup.through_drill_ratio = number(cmd, p, "through_drill_ratio", 10.0, 1.0..=100.0)?;
    setup.setup_warn_at = number(cmd, p, "setup_warn_at", 3.0, 1.0..=6.0)? as usize;
    let resolution = number(cmd, p, "resolution", 40.0, 8.0..=120.0)? as usize;
    let turning_axis = match p.get("turning_axis") {
        None | Some(Value::Null) => None,
        Some(v) => Some(crate::params::vec3(v).and_then(|a| a.normalized()).ok_or_else(|| bad(cmd, "`turning_axis` is a direction [x, y, z]"))?),
    };
    if p.get("mode").and_then(Value::as_str).is_some_and(|m| !m.is_empty()) {
        return Err(bad(cmd, "`mode` is not an option of this command"));
    }

    let mesh = body.mesh();
    if mesh.triangles.is_empty() {
        return Err(bad(cmd, "there is no solid here to check"));
    }
    let spots = solidcraft_doc::naming::face_spots(body);
    let tol = (body.body.size() * 2e-3).max(1e-3);
    let faces = body.body.faces(tol).unwrap_or_default();
    let bounds = mesh.bounds();
    let size = bounds.size();
    let longest_axis = if size.x >= size.y && size.x >= size.z {
        Vec3::X
    } else if size.y >= size.z {
        Vec3::Y
    } else {
        Vec3::Z
    };
    let axis = turning_axis.unwrap_or(longest_axis);
    // Flats: planar faces parallel to the turning axis that are not the ends.
    let flats = faces.iter().filter(|f| f.plane_normal.is_some_and(|n| n.dot(axis).abs() < 0.05)).count();
    let (holes, corners, fillets, cross_holes) = cylinders(&mesh, &spots, Vec3::Z);
    let rev = revolution(&mesh, axis, flats, cross_holes);
    if setup.process == Process::Auto {
        setup.process = if rev.is_body_of_revolution { Process::Turning } else { Process::Milling3Axis };
    }
    let mut findings: Vec<Finding> = Vec::new();
    let mut caveats: Vec<String> = Vec::new();
    let mut scan = None;
    if let Some(grid) = voxelise(&mesh, resolution, 2_000_000) {
        // Recognised holes are drilled, not milled: fill them in before asking what the end mill
        // reaches, or every hole narrower than the cutter reads as an undercut.
        let mut stock = grid.solid.clone();
        for hole in &holes {
            // `axis_point` may sit at either end or the middle of the hole: fill a hole's length
            // either way from it; beyond the part there is no grid to fill.
            let (axis, radius, reach) = (hole.axis, hole.diameter / 2.0 + grid.h * 1000.0 * 0.5, hole.depth + grid.h * 1000.0);
            for (v, cell) in stock.iter_mut().enumerate() {
                if *cell {
                    continue;
                }
                let c = grid.centre(v) * 1000.0 - hole.axis_point;
                let along = c.dot(axis);
                if along.abs() <= reach && (c - axis * along).len() <= radius {
                    *cell = true;
                }
            }
        }
        let access = machining::scan_access(
            &AccessGrid { nx: grid.nx, ny: grid.ny, nz: grid.nz, h: grid.h * 1000.0, origin: grid.origin * 1000.0, solid: &stock },
            0.5 * setup.cutter_diameter,
        );
        for region in &access.unreachable_regions {
            findings.push(Finding { severity: Severity::Refuse, rule: "undercut".into(), detail: "no straight approach from any of the six directions reaches this surface. Name the cutter that would -- a T-slot, dovetail or woodruff -- or redesign it.".into(), measured: region.area, limit: 0.0, unit: "mm²".into(), at: Some(region.sample) });
        }
        if access.has_trapped {
            findings.push(Finding {
                severity: Severity::Refuse,
                rule: "enclosed cavity".into(),
                detail: "this void is sealed inside the material. No cutter of any shape reaches it.".into(),
                measured: access.trapped_volume,
                limit: 0.0,
                unit: "mm³".into(),
                at: access.trapped_sample,
            });
        }
        if access.setup_count >= setup.setup_warn_at {
            let axes: Vec<&str> = (0..6).filter(|d| access.setup_axes[*d]).map(|d| machining::AXIS_NAMES[d]).collect();
            findings.push(Finding { severity: Severity::Warning, rule: "setups".into(), detail: format!("each setup is money, and features cut in different ones carry a stack-up between them. The approaches needed are {}. Dimension what has to be true to itself so it is cut in one.", axes.join(" ")), measured: access.setup_count as f64, limit: setup.setup_warn_at as f64, unit: "count".into(), at: None });
        }
        scan = Some(access);
    } else {
        caveats.push("the tool-access scan did not run: the grid would be too fine".into());
    }
    let wall = print_analysis::thinnest_wall(&mesh);
    match wall {
        Some((t, at)) => findings.extend(machining::check_wall(t, at, &setup)),
        None => caveats.push("the wall thickness could not be measured on this shape".into()),
    }
    if setup.process == Process::Turning {
        findings.extend(machining::check_turning(&rev, &fillets));
    } else {
        findings.extend(machining::check_corners(&corners, &setup));
        caveats.push("sharp inside corners and pocket floors are not recognised from the faces yet: only radiused corners (concave cylinders along the tool axis) were judged, and the depth-to-width rule was not applied".into());
    }
    findings.extend(machining::check_holes(&holes, &setup));
    machining::sort_findings(&mut findings);
    let refusals = findings.iter().filter(|f| f.severity == Severity::Refuse).count();
    let warnings_count = findings.iter().filter(|f| f.severity == Severity::Warning).count();
    let notes = findings.len() - refusals - warnings_count;
    let text = machining::describe(&setup, scan.as_ref(), Some(&rev), &findings, &caveats);
    let verdict = if refusals > 0 {
        format!("cannot be made as drawn: {refusals} refusal(s), the first is {}", findings[0].rule)
    } else if warnings_count > 0 {
        format!("makeable with {warnings_count} warning(s)")
    } else {
        "nothing to report against the rules this checks".to_string()
    };
    let finding_json = |f: &Finding| json!({"severity": f.severity.name(), "rule": f.rule, "detail": f.detail, "measured": f.measured, "limit": f.limit, "unit": f.unit, "at": f.at});
    Ok(json!({
        "body": name,
        "assumed": {
            "process": match setup.process { Process::Turning => "turning", _ => "milling" },
            "material": setup.material.name(),
            "cutter_mm": setup.cutter_diameter,
            "min_corner_radius_mm": setup.min_corner_radius,
            "blind_drill_ratio": setup.blind_drill_ratio,
            "through_drill_ratio": setup.through_drill_ratio,
            "turning_axis": axis,
        },
        "access": scan.as_ref().map(|a| json!({
            "setups": a.setup_count,
            "setup_axes": (0..6).filter(|d| a.setup_axes[*d]).map(|d| machining::AXIS_NAMES[d]).collect::<Vec<_>>(),
            "machined_faces": a.machined_faces,
            "surface_faces": a.surface_faces,
            "unreachable_fraction": a.unreachable_fraction,
            "unreachable_regions": a.unreachable_regions.iter().map(|r| json!({"faces": r.faces, "area_mm2": r.area, "at": r.sample})).collect::<Vec<_>>(),
            "trapped_volume_mm3": a.trapped_volume,
            "trapped_at": a.trapped_sample,
        })),
        "thinnest_wall_mm": wall.map(|w| w.0),
        "thinnest_wall_at": wall.map(|w| w.1),
        "holes": holes.iter().map(|h| json!({"diameter_mm": h.diameter, "depth_mm": h.depth, "through": h.through, "axis": h.axis, "at": h.entry_at})).collect::<Vec<_>>(),
        "corners": corners.iter().map(|c| json!({"radius_mm": c.radius, "depth_mm": c.depth, "at": c.at})).collect::<Vec<_>>(),
        "revolution": {
            "is_body_of_revolution": rev.is_body_of_revolution,
            "length_mm": rev.length,
            "max_diameter_mm": rev.max_diameter,
            "slenderness": rev.slenderness,
            "off_axis_volume_mm3": rev.off_axis_volume,
            "flats": rev.flats,
            "cross_holes": rev.cross_holes,
        },
        "findings": findings.iter().map(finding_json).collect::<Vec<_>>(),
        "refusals": refusals,
        "warnings_count": warnings_count,
        "notes": notes,
        "caveats": caveats,
        "text": text,
        "verdict": verdict,
        "warnings": Vec::<String>::new(),
        "ms": started.elapsed().as_millis() as u64,
    }))
}

#[cfg(test)]
#[path = "manufacturing_tests.rs"]
mod tests;
