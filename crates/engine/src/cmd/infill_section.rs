//! `inspect.infill_section`: the infill a slicer would lay down where a plane cuts a printed part,
//! drawn as a 3D sketch of strands. The printed-part solve homogenises the lattice, so this is the
//! only way to see the lattice its numbers describe.

use serde_json::{Value, json};
use solidcraft_analysis::infill_section::infill_on_plane;
use solidcraft_analysis::print_analysis::{FILAMENTS, INFILL_PATTERNS, InfillPattern, PrintSettings};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use super::design::{number, run};
use super::stress::body_name;
use crate::params::{bad, str_};
use crate::{Result, Session};

/// The most strands one section may draw.
const MAX_STRANDS: usize = 6000;

pub static COMMANDS: &[CommandSpec] =
    &[CommandSpec::new("inspect.infill_section", "Infill on a Section", infill_section).at("3DEEE", "MAKE").icon("infill_section").params(
        "body?: name (default: the selected body, or the design's only one); axis?: X | Y | Z the cut plane's normal (default Z, a layer); \
         station_mm?: where along it (default: the body's middle); material?, pattern?, infill?, walls?, line_width_mm?, layer_height_mm? as \
         inspect.printed_stress; name? → body, axis, station_mm, pattern, pitch_mm, strands, walls, length_mm, sketch, truncated. The pattern at \
         its pitch, the wall loops around the outside and gaps wherever the part has a hole, as a 3D sketch of lines on the cut plane: a drawing \
         of what the stress solve's homogenised material stands for, not a slicer's toolpath",
    )];

fn infill_section(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.infill_section";
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let (axis, axis_name) = match str_(p, "axis").map(|a| a.trim().to_ascii_uppercase()).as_deref() {
        None | Some("") | Some("Z") => (2usize, "Z"),
        Some("X") => (0, "X"),
        Some("Y") => (1, "Y"),
        Some(_) => return Err(bad(cmd, "`axis` is X, Y or Z")),
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
        Some(n) => InfillPattern::from_name(n)
            .ok_or_else(|| bad(cmd, format!("`pattern` is one of {}", INFILL_PATTERNS.iter().map(|(_, n)| *n).collect::<Vec<_>>().join(", "))))?,
    };
    let settings = PrintSettings {
        material: filament.name.to_string(),
        pattern,
        infill_density: number(cmd, p, "infill", Some(0.2), 0.0..=1.0)?.unwrap_or(0.2),
        wall_loops: number(cmd, p, "walls", Some(2.0), 0.0..=20.0)?.unwrap_or(2.0).round() as u32,
        line_width: number(cmd, p, "line_width_mm", Some(0.42), 0.1..=2.0)?.unwrap_or(0.42),
        layer_height: number(cmd, p, "layer_height_mm", Some(0.2), 0.05..=1.0)?.unwrap_or(0.2),
        top_bottom_layers: 4,
    };
    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let mesh = body.mesh();
    let bounds = mesh.bounds();
    let (lo, hi) = match axis {
        0 => (bounds.min.x, bounds.max.x),
        1 => (bounds.min.y, bounds.max.y),
        _ => (bounds.min.z, bounds.max.z),
    };
    if !(hi > lo) {
        return Err(bad(cmd, "the body has no extent along that axis"));
    }
    let middle = (lo + hi) / 2.0;
    let station = number(cmd, p, "station_mm", Some(middle), -1e6..=1e6)?.unwrap_or(middle);
    if station <= lo || station >= hi {
        return Err(bad(cmd, format!("the plane misses the body: `station_mm` lies between {lo:.2} and {hi:.2}")));
    }
    let index = mesh.inside_index();
    let strands = infill_on_plane(&settings, bounds.min, bounds.max, axis, station, &|q: Vec3| index.contains(q), MAX_STRANDS);
    if strands.is_empty() {
        return Err(bad(cmd, "the plane meets no material"));
    }
    let walls = strands.iter().filter(|s| s.wall).count();
    let length: f64 = strands.iter().map(|s| (s.to - s.from).len()).sum();
    let directions = match pattern {
        InfillPattern::Grid | InfillPattern::Rectilinear | InfillPattern::CrossHatch => 2.0,
        InfillPattern::Triangles | InfillPattern::TriHexagon | InfillPattern::Honeycomb | InfillPattern::Cubic => 3.0,
        _ => 1.0,
    };
    let pitch = if settings.infill_density > 0.0 { settings.line_width / settings.infill_density * directions } else { 0.0 };
    let sketch_name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{name} infill {axis_name} {station:.1}"));
    let sketch = run(s, "sketch.create", json!({"plane": "XY", "name": sketch_name}))?;
    run(s, "sketch.options", json!({"sketch_3d": true}))?;
    for strand in &strands {
        run(s, "sketch.line3d", json!({"points": [[strand.from.x, strand.from.y, strand.from.z], [strand.to.x, strand.to.y, strand.to.z]]}))?;
    }
    run(s, "sketch.finish", json!({}))?;
    Ok(json!({
        "body": name,
        "axis": axis_name,
        "station_mm": station,
        "pattern": pattern.name(),
        "infill": settings.infill_density,
        "pitch_mm": pitch,
        "strands": strands.len() - walls,
        "walls": walls,
        "length_mm": length,
        "sketch": sketch.get("sketch").cloned().unwrap_or(Value::Null),
        "truncated": strands.len() >= MAX_STRANDS,
    }))
}

#[cfg(test)]
#[path = "infill_section_tests.rs"]
mod tests;
