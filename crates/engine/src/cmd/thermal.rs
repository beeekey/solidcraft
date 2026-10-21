//! Thermal Analysis (`inspect.thermal`): a part sitting in still air, with watts put in over a
//! face and, optionally, a face held at a temperature. Steady conduction through the voxelised
//! part, free convection by the correlation for each face's orientation (channels between fins
//! included), and radiation, solved together by [`solidcraft_analysis::thermal_solve`].

use serde_json::{Value, json};
use solidcraft_analysis::{AmbientAir, HeatInput, HeldTemperature, Placement, Region, ThermalProblem, handbook, thermal_solve};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use crate::params::{bad, bool_};
use crate::{Result, Sel, Session};

pub static COMMANDS: &[CommandSpec] =
    &[CommandSpec::new("inspect.thermal", "Thermal Analysis", thermal).at("SOLID", "INSPECT").icon("measure").noundo().params(
        "body?: name (default: the selected body, or the design's only one); heat: a face (a point [x, y, z] on it, its name from model.faces, or \
     {face: index}) the power goes in over; watts: W (0.001 to 1e6); held?: a face held at held_c? °C (a cold plate, a massive mount); \
     ambient_c?: the room (default 20); emissivity?: 0 to 1 (default: the material's plain surface from the handbook); up?: [x, y, z] (default +Z); \
     conductivity_w_mk?: W/m·K (default: the body's material through the handbook, else a plastic); resolution?: cells along the longest side \
     (8 to 120, default 40); enclosure_area_m2?: the outside area of a sealed box round the part (its air then runs above the room); \
     field?: true also returns every cell's temperature (origin_mm, element_mm, grid, cells: [[index, celsius]…]) → body, material, \
     conductivity_w_mk, emissivity, peak_c, mean_surface_c, air_c, enclosure_rise_k, convected_w, radiated_w, held_w, film_w_m2k, exposed_area_cm2, \
     channelled_fraction (how finned the part is), narrowest_gap_mm, rayleigh, grid, rounds, verdict, warnings. Not a flow solve: it assumes still, \
     unbounded air unless a box is given",
    )];

const MAX_WATTS: f64 = 1e6;

fn number(cmd: &str, p: &Value, key: &str, default: Option<f64>, range: std::ops::RangeInclusive<f64>) -> Result<Option<f64>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v
            .as_f64()
            .filter(|x| x.is_finite() && range.contains(x))
            .map(Some)
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

/// A face as a placement: `{face: index}`, or a point or face name resolved to the face under it,
/// or, when no face is near enough, the box side nearest the point.
fn placement(s: &Session, v: &Value, body: &str, cmd: &str) -> Result<Placement> {
    let st = s.world_state();
    let b = st.body(body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
    let mesh = b.mesh();
    let triangles_of =
        |face: usize| -> Vec<usize> { mesh.tri_face.iter().enumerate().filter(|(_, f)| **f as usize == face).map(|(t, _)| t).collect() };
    if let Some(face) = v.get("face").and_then(Value::as_u64) {
        let triangles = triangles_of(usize::try_from(face).unwrap_or(usize::MAX));
        if triangles.is_empty() {
            return Err(bad(cmd, format!("`{body}` has no face {face}")));
        }
        return Ok(Placement::Face(Region::Face(triangles)));
    }
    let point = super::face::face_ref(s, v, Some(body), cmd)?;
    match solidcraft_doc::appearance::face_index_at(b, point).map(triangles_of).filter(|t| !t.is_empty()) {
        Some(triangles) => Ok(Placement::Face(Region::Face(triangles))),
        None => Ok(Placement::Face(Region::NearestWall(point))),
    }
}

fn thermal(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.thermal";
    let started = web_time::Instant::now();
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let heat = p.get("heat").ok_or_else(|| bad(cmd, "give `heat`: the face the power goes in over"))?;
    let watts = number(cmd, p, "watts", None, 0.001..=MAX_WATTS)?.ok_or_else(|| bad(cmd, "give `watts`"))?;
    let heat = placement(s, heat, &name, cmd)?;
    let held = match p.get("held") {
        None | Some(Value::Null) => None,
        Some(v) => Some((placement(s, v, &name, cmd)?, number(cmd, p, "held_c", Some(20.0), -200.0..=2000.0)?.unwrap_or(20.0))),
    };
    let ambient_c = number(cmd, p, "ambient_c", Some(20.0), -100.0..=500.0)?.unwrap_or(20.0);
    let resolution = number(cmd, p, "resolution", Some(40.0), 8.0..=120.0)?.unwrap_or(40.0) as usize;
    let enclosure = number(cmd, p, "enclosure_area_m2", Some(0.0), 0.0..=1e4)?.unwrap_or(0.0);
    let up = match p.get("up") {
        None | Some(Value::Null) => Vec3::Z,
        Some(v) => crate::params::vec3(v).and_then(|u| u.normalized()).ok_or_else(|| bad(cmd, "`up` is a direction [x, y, z]"))?,
    };
    if p.get("mode").and_then(Value::as_str).is_some_and(|m| !m.is_empty()) {
        return Err(bad(cmd, "`mode` is not an option of this command"));
    }

    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let material_name = s.doc.materials.get(body.definition_name()).cloned().unwrap_or_else(|| "Default".to_string());
    let book = handbook::thermal(&material_name);
    let mut warnings: Vec<String> = Vec::new();
    if book.is_none() && p.get("conductivity_w_mk").is_none() {
        warnings.push(format!("no thermal constants are known for `{material_name}`: a plastic was assumed; give conductivity_w_mk"));
    }
    let fallback = handbook::thermal("Default");
    let conductivity = number(cmd, p, "conductivity_w_mk", book.or(fallback).map(|t| t.conductivity), 0.01..=5000.0)?.unwrap_or(0.25);
    let emissivity = number(cmd, p, "emissivity", book.or(fallback).map(|t| t.emissivity), 0.0..=1.0)?.unwrap_or(0.9);

    let problem = ThermalProblem {
        conductivity,
        resolution,
        sources: vec![HeatInput { region: heat, watts, name: "heat".into() }],
        held: held.map(|(region, c)| HeldTemperature { region, temperature: c + 273.15, name: "held".into() }).into_iter().collect(),
        air: AmbientAir { temperature: ambient_c + 273.15, emissivity, up, enclosure_area: enclosure, ..Default::default() },
        max_rounds: 40,
    };
    let mesh = body.mesh();
    let result = thermal_solve(&mesh, &problem);
    if !result.solved {
        return Err(bad(cmd, result.advice));
    }
    for sentence in result.advice.split(". ").map(str::trim).filter(|a| !a.is_empty()) {
        warnings.push(sentence.trim_end_matches('.').to_string());
    }
    let celsius = |k: f64| k - 273.15;
    let field = bool_(p, "field").unwrap_or(false).then(|| {
        let cells: Vec<(usize, f64)> = result
            .solid
            .iter()
            .enumerate()
            .filter(|(_, solid)| **solid)
            .map(|(i, _)| (i, celsius(result.temperature.get(i).copied().unwrap_or(0.0))))
            .collect();
        json!({ "origin_mm": result.origin * 1000.0, "element_mm": result.h * 1000.0, "grid": result.grid, "cells": cells })
    });
    let peak_c = celsius(result.peak_temperature);
    let verdict = format!(
        "peaks at {peak_c:.1} °C, {:.1} K above the air; {:.0}% leaves by convection, {:.0}% by radiation{}",
        result.peak_temperature - result.air_temperature,
        100.0 * result.convected / watts,
        100.0 * result.radiated / watts,
        if result.held_flux > 0.01 * watts { format!(", {:.0}% into the held face", 100.0 * result.held_flux / watts) } else { String::new() }
    );
    Ok(json!({
        "body": name,
        "material": material_name,
        "conductivity_w_mk": conductivity,
        "emissivity": emissivity,
        "peak_c": peak_c,
        "mean_surface_c": celsius(result.mean_surface_temperature),
        "air_c": celsius(result.air_temperature),
        "enclosure_rise_k": result.enclosure_rise,
        "convected_w": result.convected,
        "radiated_w": result.radiated,
        "held_w": result.held_flux,
        "film_w_m2k": result.film_coefficient,
        "exposed_area_cm2": result.exposed_area * 1e4,
        "channelled_fraction": result.channelled_fraction,
        "narrowest_gap_mm": result.narrowest_gap * 1000.0,
        "rayleigh": result.rayleigh,
        "grid": result.grid,
        "element_mm": result.h * 1000.0,
        "rounds": result.rounds,
        "verdict": verdict,
        "warnings": warnings,
        "field": field,
        "ms": started.elapsed().as_millis() as u64,
    }))
}

#[cfg(test)]
#[path = "thermal_tests.rs"]
mod tests;
