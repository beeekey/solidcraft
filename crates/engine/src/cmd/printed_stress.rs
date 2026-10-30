//! Stress of a part as it will be printed (`inspect.printed_stress`) and the same load through
//! every infill pattern (`inspect.compare_infill`). The slicer's settings become an orthotropic
//! material ([`solidcraft_analysis::print_analysis`]), the layers stack along the build direction,
//! and the answer is judged twice: within a layer and between layers, where printed parts come
//! apart. Neither command changes the design.

use serde_json::{Value, json};
use solidcraft_analysis::print_analysis::{FILAMENTS, INFILL_PATTERNS, InfillPattern, PrintSettings};
use solidcraft_analysis::{Governs, MeshFemError, MeshFemOptions, Region, compare_infill, printed_stress};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use super::stress::{body_name, region};
use crate::params::{bad, bool_, str_};
use crate::{Result, Session};

const PRINT_PARAMS: &str = "body?: name (default: the selected body, or the design's only one); fixed: the face the part is held at, load: the face \
     the force acts on (a point, a face name or {face: index}); force_n: [fx, fy, fz] newtons; up?: [x, y, z] the build direction, the layers \
     stack along it (default +Z); material?: the filament (PLA, PLA+, PETG, ABS, ASA, TPU 95A; default PLA); infill?: 0 to 1 (default 0.2); \
     walls?: perimeters (0 to 20, default 2); line_width_mm? (0.1 to 2, default 0.42); layer_height_mm? (0.05 to 1, default 0.2); \
     top_bottom_layers? (0 to 20, default 4); resolution?: elements along the longest side (6 to 60, default 20)";

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("inspect.printed_stress", "Printed Stress", printed).at("3DEEE", "MAKE").icon("printed_stress").noundo().params(
        "pattern?: Grid | Lines | Rectilinear | Triangles | Tri-hexagon | Cubic | Gyroid | Honeycomb | Concentric | Cross-hatch | Lightning \
         (default Grid); field?: true also returns every cell's von Mises stress and its stress normal to the layers (MPa); and body?, fixed, load, \
         force_n, up?, material?, infill?, walls?, line_width_mm?, layer_height_mm?, top_bottom_layers?, resolution? as inspect.compare_infill → \
         body, up, settings, printed_material: {young_in_plane_gpa, young_across_layers_gpa, shear_in_plane_gpa, shear_interlayer_gpa, \
         tensile_in_plane_mpa, tensile_interlayer_mpa, solid_fraction, shell_fraction, density_kg_m3, mass_grams}, grid, voxels, element_mm, \
         converged, max_displacement_mm, max_displacement_at, load_displacement_mm, max_von_mises_mpa, max_von_mises_at, max_layer_normal_mpa \
         (tension pulling the layers apart), max_layer_normal_at, max_layer_shear_mpa, safety_in_plane, safety_interlayer (null when nothing is \
         stressed that way), governs: \"within a layer\" | \"between layers\", elements_through_thinnest, mesh_advice, verdict, advice, warnings, \
         field?, layer_field?. The model is homogenised: the shell and the infill are one material, so it will not find a stress concentration \
         at one strand, and the interlayer strength is the published band (about half of in-plane), not a measurement of your printer",
    ),
    CommandSpec::new("inspect.compare_infill", "Compare Infill", compare).at("3DEEE", "MAKE").icon("compare_infill").noundo().params(PRINT_PARAMS),
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

struct Setup {
    name: String,
    fixed: Region,
    load: Region,
    force: Vec3,
    up: Vec3,
    settings: PrintSettings,
    resolution: usize,
}

fn setup(s: &Session, p: &Value, cmd: &str) -> Result<Setup> {
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let fixed = p.get("fixed").ok_or_else(|| bad(cmd, "give `fixed`: the face the part is held at"))?;
    let load = p.get("load").ok_or_else(|| bad(cmd, "give `load`: the face the force acts on"))?;
    let fixed = region(s, fixed, &name, cmd)?;
    let load = region(s, load, &name, cmd)?;
    let force = p
        .get("force_n")
        .and_then(crate::params::vec3)
        .filter(|f| f.len() > 0.0 && f.len() <= 1e9)
        .ok_or_else(|| bad(cmd, "`force_n` must be [fx, fy, fz] newtons, not zero and at most 1e9 N in all"))?;
    let up = match p.get("up") {
        None | Some(Value::Null) => Vec3::Z,
        Some(v) => crate::params::vec3(v).and_then(|u| u.normalized()).ok_or_else(|| bad(cmd, "`up` is the build direction [x, y, z], not zero"))?,
    };
    let material = match p.get("material") {
        None | Some(Value::Null) => "PLA",
        Some(Value::String(m)) => m.as_str(),
        Some(_) => return Err(bad(cmd, "`material` is a filament name")),
    };
    let wanted = material.trim().trim_end_matches("(printed)").trim();
    let filament = FILAMENTS
        .iter()
        .find(|f| f.name.eq_ignore_ascii_case(wanted))
        .ok_or_else(|| bad(cmd, format!("`material` is a filament: {}", FILAMENTS.iter().map(|f| f.name).collect::<Vec<_>>().join(", "))))?;
    let pattern = match str_(p, "pattern") {
        None => InfillPattern::Grid,
        Some(name) => InfillPattern::from_name(name)
            .ok_or_else(|| bad(cmd, format!("`pattern` is one of {}", INFILL_PATTERNS.iter().map(|(_, n)| *n).collect::<Vec<_>>().join(", "))))?,
    };
    if str_(p, "mode").is_some_and(|m| !m.is_empty()) {
        return Err(bad(cmd, "`mode` is not an option of this command"));
    }
    let settings = PrintSettings {
        material: filament.name.to_string(),
        pattern,
        infill_density: number(cmd, p, "infill", 0.2, 0.0..=1.0)?,
        wall_loops: number(cmd, p, "walls", 2.0, 0.0..=20.0)?.round() as u32,
        line_width: number(cmd, p, "line_width_mm", 0.42, 0.1..=2.0)?,
        layer_height: number(cmd, p, "layer_height_mm", 0.2, 0.05..=1.0)?,
        top_bottom_layers: number(cmd, p, "top_bottom_layers", 4.0, 0.0..=20.0)?.round() as u32,
    };
    let resolution = number(cmd, p, "resolution", 20.0, 6.0..=60.0)? as usize;
    Ok(Setup { name, fixed, load, force, up, settings, resolution })
}

fn solve_error(cmd: &str, e: MeshFemError) -> crate::EngineError {
    match e {
        MeshFemError::Input => bad(cmd, "the body or the printed material cannot be analysed"),
        other => bad(cmd, other.to_string()),
    }
}

/// A safety factor as JSON: `null` when nothing is stressed that way.
fn finite(x: f64) -> Value {
    if x.is_finite() { json!(x) } else { Value::Null }
}

fn settings_json(settings: &PrintSettings) -> Value {
    json!({
        "material": settings.material,
        "pattern": settings.pattern.name(),
        "infill": settings.infill_density,
        "walls": settings.wall_loops,
        "line_width_mm": settings.line_width,
        "layer_height_mm": settings.layer_height,
        "top_bottom_layers": settings.top_bottom_layers,
    })
}

fn printed(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.printed_stress";
    let started = web_time::Instant::now();
    let setup = setup(s, p, cmd)?;
    let st = s.world_state();
    let body = st.body(&setup.name).ok_or_else(|| bad(cmd, format!("no body `{}`", setup.name)))?;
    let mesh = body.mesh();
    let r = printed_stress(&mesh, &setup.fixed, &setup.load, setup.force, &setup.settings, setup.up, setup.resolution, &MeshFemOptions::default())
        .map_err(|e| solve_error(cmd, e))?;
    let mut warnings: Vec<String> = Vec::new();
    if !r.solve.converged {
        warnings.push("the solve did not converge: the numbers are approximate".into());
    }
    if !r.solve.mesh_advice.is_empty() {
        warnings.push(r.solve.mesh_advice.clone());
    }
    if setup.settings.pattern == InfillPattern::Lightning {
        warnings.push("lightning infill holds up the top skin; it is not meant to carry load".into());
    }
    let mm = |v: Vec3| v * 1000.0;
    let peak_mpa = r.solve.max_von_mises / 1e6;
    let layer_mpa = r.layer.max_normal / 1e6;
    let safety = r.safety_in_plane.min(r.safety_interlayer);
    let verdict = match r.governs {
        _ if safety >= 2.0 => format!("holds as printed: {} governs, safety factor {safety:.1}", r.governs.name()),
        _ if safety >= 1.0 => format!("holds, barely: {} governs, safety factor {safety:.2}", r.governs.name()),
        Governs::BetweenLayers => format!(
            "delaminates: {layer_mpa:.1} MPa pulls the layers apart, {:.1}× the {:.1} MPa bond between them",
            1.0 / safety.max(1e-12),
            r.effective.tensile_interlayer / 1e6
        ),
        Governs::WithinLayer => {
            format!(
                "yields within a layer: peak {peak_mpa:.1} MPa is {:.1}× the {:.1} MPa in plane",
                1.0 / safety.max(1e-12),
                r.effective.tensile_in_plane / 1e6
            )
        }
    };
    let fields = bool_(p, "field").unwrap_or(false).then(|| {
        let cells = |values: &[f64]| -> Vec<(usize, f64)> {
            r.solve.solid.iter().enumerate().filter(|(_, solid)| **solid).map(|(i, _)| (i, values.get(i).copied().unwrap_or(0.0) / 1e6)).collect()
        };
        let frame = |cells: Vec<(usize, f64)>| json!({ "origin_mm": r.solve.origin * 1000.0, "element_mm": r.solve.h * 1000.0, "grid": r.solve.grid, "cells": cells });
        (frame(cells(&r.solve.voxel_stress)), frame(cells(&r.layer.normal)))
    });
    let (field, layer_field) = match fields {
        Some((f, l)) => (Some(f), Some(l)),
        None => (None, None),
    };
    Ok(json!({
        "body": setup.name,
        "up": setup.up,
        "settings": settings_json(&setup.settings),
        "printed_material": {
            "young_in_plane_gpa": r.material.ey / 1e9,
            "young_across_layers_gpa": r.material.ex / 1e9,
            "shear_in_plane_gpa": r.material.gyz / 1e9,
            "shear_interlayer_gpa": r.material.gxy / 1e9,
            "tensile_in_plane_mpa": r.effective.tensile_in_plane / 1e6,
            "tensile_interlayer_mpa": r.effective.tensile_interlayer / 1e6,
            "solid_fraction": r.effective.solid_fraction,
            "shell_fraction": r.effective.shell_fraction,
            "density_kg_m3": r.effective.density,
            "mass_grams": r.effective.mass_grams,
        },
        "grid": r.solve.grid,
        "voxels": r.solve.voxels,
        "element_mm": r.solve.h * 1000.0,
        "converged": r.solve.converged,
        "max_displacement_mm": r.solve.max_displacement * 1000.0,
        "max_displacement_at": mm(r.solve.max_displacement_at),
        "load_displacement_mm": r.solve.load_displacement * 1000.0,
        "max_von_mises_mpa": peak_mpa,
        "max_von_mises_at": mm(r.solve.max_von_mises_at),
        "max_layer_normal_mpa": layer_mpa,
        "max_layer_normal_at": r.max_layer_normal_at.map(mm),
        "max_layer_shear_mpa": r.layer.max_shear / 1e6,
        "safety_in_plane": finite(r.safety_in_plane),
        "safety_interlayer": finite(r.safety_interlayer),
        "governs": r.governs.name(),
        "elements_through_thinnest": r.solve.elements_through_thinnest,
        "mesh_advice": r.solve.mesh_advice,
        "verdict": verdict,
        "advice": r.effective.note,
        "warnings": warnings,
        "field": field,
        "layer_field": layer_field,
        "ms": started.elapsed().as_millis() as u64,
    }))
}

fn compare(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.compare_infill";
    let started = web_time::Instant::now();
    let setup = setup(s, p, cmd)?;
    let st = s.world_state();
    let body = st.body(&setup.name).ok_or_else(|| bad(cmd, format!("no body `{}`", setup.name)))?;
    let mesh = body.mesh();
    let rows = compare_infill(&mesh, &setup.fixed, &setup.load, setup.force, &setup.settings, setup.up, setup.resolution, &MeshFemOptions::default())
        .map_err(|e| solve_error(cmd, e))?;
    let sf = |x: f64| if x.is_finite() { format!("{x:.2}") } else { "—".to_string() };
    let mut text = format!("{:<14}{:>9}{:>9}{:>10}{:>10}{:>10}\n", "pattern", "mass g", "Ex GPa", "defl mm", "SF layer", "SF plane");
    for r in &rows {
        text.push_str(&format!(
            "{:<14}{:>9.1}{:>9.2}{:>10.3}{:>10}{:>10}\n",
            r.pattern.name(),
            r.mass_grams,
            r.young_in_plane / 1e9,
            r.max_displacement * 1000.0,
            sf(r.safety_interlayer),
            sf(r.safety_in_plane)
        ));
    }
    let stiffest = rows.iter().min_by(|a, b| a.max_displacement.total_cmp(&b.max_displacement));
    let safest = rows.iter().max_by(|a, b| a.safety_interlayer.total_cmp(&b.safety_interlayer));
    let mut warnings: Vec<String> = Vec::new();
    if rows.iter().any(|r| !r.safety_in_plane.is_finite() || !r.safety_interlayer.is_finite()) {
        warnings.push("a safety factor shown as — means nothing is stressed that way".into());
    }
    let printed_once =
        printed_stress(&mesh, &setup.fixed, &setup.load, setup.force, &setup.settings, setup.up, setup.resolution, &MeshFemOptions::default());
    if let Ok(r) = &printed_once
        && !r.solve.mesh_advice.is_empty()
    {
        warnings.push(r.solve.mesh_advice.clone());
    }
    let verdict = match (stiffest, safest) {
        (Some(a), Some(b)) => format!(
            "same {:.1} g every way; stiffest {} ({:.3} mm), safest between layers {} (safety {})",
            a.mass_grams,
            a.pattern.name(),
            a.max_displacement * 1000.0,
            b.pattern.name(),
            sf(b.safety_interlayer)
        ),
        _ => "no pattern could be solved".to_string(),
    };
    Ok(json!({
        "body": setup.name,
        "up": setup.up,
        "settings": settings_json(&setup.settings),
        "patterns": rows.iter().map(|r| json!({
            "pattern": r.pattern.name(),
            "mass_g": r.mass_grams,
            "young_in_plane_gpa": r.young_in_plane / 1e9,
            "young_across_layers_gpa": r.young_across_layers / 1e9,
            "deflection_mm": r.max_displacement * 1000.0,
            "max_von_mises_mpa": r.max_von_mises / 1e6,
            "max_layer_normal_mpa": r.max_layer_normal / 1e6,
            "safety_interlayer": finite(r.safety_interlayer),
            "safety_in_plane": finite(r.safety_in_plane),
            "governs": r.governs.name(),
        })).collect::<Vec<_>>(),
        "stiffest": stiffest.map(|r| r.pattern.name()),
        "safest_between_layers": safest.map(|r| r.pattern.name()),
        "text": text,
        "verdict": verdict,
        "advice": "mass is set by the density alone, so the pattern is a free choice of where the stiffness goes; the layer safety factor is \
                   against the bond between layers, which is what printed parts usually fail at",
        "warnings": warnings,
        "ms": started.elapsed().as_millis() as u64,
    }))
}

#[cfg(test)]
#[path = "printed_stress_tests.rs"]
mod tests;
