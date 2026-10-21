//! Flow Analysis (`inspect.flow`): what the air or coolant does around or through a part, by the
//! lattice-Boltzmann solver in [`solidcraft_analysis::fluid_solve`]: pressure drop through a
//! passage, drag on a shape, and, with heat on, how much heat the flow carries away from the part
//! (conjugate: conduction through the metal and convection into the fluid as one system).

use serde_json::{Value, json};
use solidcraft_analysis::{FlowBody, FlowRegion, FluidError, FluidProblem, fluid_named, fluid_solve, handbook};
use solidcraft_geom::{Aabb3, Vec3};

use super::CommandSpec;
use crate::params::{bad, bool_, str_};
use crate::{Result, Sel, Session};

pub static COMMANDS: &[CommandSpec] =
    &[CommandSpec::new("inspect.flow", "Flow Analysis", flow).at("SOLID", "INSPECT").icon("measure").noundo().params(
        "body?: name (default: the selected body, or the design's only one); region: \"internal\" (through the passages the inlet reaches) | \
     \"external\" (around the part, in a tunnel); inlet, outlet: points [x, y, z] (mm) near opposite sides of the flow domain, each picks the \
     nearest side as the inlet or outlet plane; velocity_m_s: at the inlet (1e-4 to 100); fluid?: \"Water\" (default) | \"Air\" | \
     \"Oil (ISO VG 46)\" | \"Ethylene glycol 50%\", or density_kg_m3? and viscosity_pa_s?; domain?: {min: [x,y,z], max: [x,y,z]} (mm), the box \
     the fluid occupies for external flow (default: the part's box padded); resolution?: cells along the longest side (8 to 160, default 40); \
     max_steps?: (100 to 100000, default 20000); heat?: true also solves the temperature: watts? dissipated in the part (its temperature is then \
     solved), or wall_c? the part is held at; inlet_c? (default 20) → converged, steps, reynolds, cells_across_passage, pressure_drop_pa, \
     volume_flow_l_min, mean_inlet_velocity_m_s, max_velocity_m_s, drag_force_n: [x, y, z], and with heat: outlet_c, max_c, heat_removed_w; \
     verdict, warnings. Laminar, with a subgrid eddy viscosity past what the grid resolves; the warnings say when the number is indicative",
    )];

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

fn flow(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.flow";
    let started = web_time::Instant::now();
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let region = match str_(p, "region").unwrap_or("internal") {
        "internal" => FlowRegion::Internal,
        "external" => FlowRegion::External,
        _ => return Err(bad(cmd, "`region` is \"internal\" or \"external\"")),
    };
    let point = |key: &str| -> Result<Vec3> {
        p.get(key)
            .and_then(crate::params::vec3)
            .map(|v| v / 1000.0)
            .ok_or_else(|| bad(cmd, format!("give `{key}`: a point [x, y, z] in mm near a side of the flow domain")))
    };
    let (inlet_point, outlet_point) = (point("inlet")?, point("outlet")?);
    let velocity = number(cmd, p, "velocity_m_s", None, 1e-4..=100.0)?.ok_or_else(|| bad(cmd, "give `velocity_m_s`"))?;
    let base = fluid_named(str_(p, "fluid").unwrap_or("Water"));
    if let Some(given) = str_(p, "fluid")
        && !base.name.eq_ignore_ascii_case(given.trim())
    {
        return Err(bad(cmd, "`fluid` is Water, Air, Oil (ISO VG 46) or Ethylene glycol 50%"));
    }
    let density = number(cmd, p, "density_kg_m3", Some(base.density), 1e-3..=1e5)?.unwrap_or(base.density);
    let viscosity = number(cmd, p, "viscosity_pa_s", Some(base.viscosity), 1e-7..=1e3)?.unwrap_or(base.viscosity);
    let domain = match p.get("domain") {
        None | Some(Value::Null) => None,
        Some(d) => {
            let (min, max) = (d.get("min").and_then(crate::params::vec3), d.get("max").and_then(crate::params::vec3));
            match (min, max) {
                (Some(min), Some(max)) if max.x > min.x && max.y > min.y && max.z > min.z => Some(Aabb3 { min: min / 1000.0, max: max / 1000.0 }),
                _ => return Err(bad(cmd, "`domain` is {min: [x, y, z], max: [x, y, z]} in mm with a positive size")),
            }
        }
    };
    let resolution = number(cmd, p, "resolution", Some(40.0), 8.0..=160.0)?.unwrap_or(40.0) as usize;
    let max_steps = number(cmd, p, "max_steps", Some(20_000.0), 100.0..=100_000.0)?.unwrap_or(20_000.0) as usize;
    let heat = bool_(p, "heat").unwrap_or(false);
    let watts = number(cmd, p, "watts", Some(0.0), 0.0..=1e6)?.unwrap_or(0.0);
    let wall_c = number(cmd, p, "wall_c", Some(80.0), -200.0..=2000.0)?.unwrap_or(80.0);
    let inlet_c = number(cmd, p, "inlet_c", Some(20.0), -200.0..=2000.0)?.unwrap_or(20.0);
    if p.get("mode").and_then(Value::as_str).is_some_and(|m| !m.is_empty()) {
        return Err(bad(cmd, "`mode` is not an option of this command"));
    }

    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let material_name = s.doc.materials.get(body.definition_name()).cloned().unwrap_or_else(|| "Default".to_string());
    let solid_conductivity = handbook::thermal(&material_name).map(|t| t.conductivity).unwrap_or(0.25);
    let solid_density = s.doc.density(body.definition_name()) * 1000.0;
    let problem = FluidProblem {
        region,
        inlet_point,
        outlet_point,
        inlet_velocity: velocity,
        density,
        viscosity,
        domain,
        resolution,
        max_steps,
        solve_heat: heat,
        inlet_temperature: inlet_c + 273.15,
        wall_temperature: wall_c + 273.15,
        heat_power: watts,
        fluid_conductivity: base.conductivity,
        fluid_heat_capacity: base.heat_capacity,
        solid_conductivity,
        solid_density,
        solid_heat_capacity: 900.0,
        ..Default::default()
    };
    let mesh = body.mesh();
    let result = fluid_solve(FlowBody::Mesh(&mesh), &problem).map_err(|e: FluidError| bad(cmd, e.to_string()))?;
    let warnings: Vec<String> =
        result.advice.split(". ").map(str::trim).filter(|a| !a.is_empty()).map(|a| a.trim_end_matches('.').to_string()).collect();
    let celsius = |k: f64| k - 273.15;
    let verdict = if !result.converged && result.steps == 0 {
        "the inlet reaches no open space".to_string()
    } else if !result.converged {
        format!("did not settle in {} steps", result.steps)
    } else if heat && result.heat_solved {
        format!(
            "Re {:.0}: {:.1} Pa drop, {:.2} L/min; the fluid leaves at {:.1} °C carrying {:.1} W, the part peaks at {:.1} °C",
            result.reynolds,
            result.pressure_drop,
            result.volume_flow_rate * 60_000.0,
            celsius(result.outlet_temperature),
            result.heat_removed,
            celsius(result.max_temperature)
        )
    } else {
        format!(
            "Re {:.0}: {:.1} Pa drop, {:.2} L/min, drag {:.3} N",
            result.reynolds,
            result.pressure_drop,
            result.volume_flow_rate * 60_000.0,
            result.drag_force.len()
        )
    };
    Ok(json!({
        "body": name,
        "fluid": base.name,
        "region": if region == FlowRegion::Internal { "internal" } else { "external" },
        "converged": result.converged,
        "steps": result.steps,
        "residual": result.residual,
        "grid": result.grid,
        "element_mm": result.h * 1000.0,
        "reynolds": result.reynolds,
        "cells_across_passage": result.cells_across_passage,
        "pressure_drop_pa": result.pressure_drop,
        "volume_flow_l_min": result.volume_flow_rate * 60_000.0,
        "mean_inlet_velocity_m_s": result.mean_inlet_velocity,
        "max_velocity_m_s": result.max_velocity,
        "drag_force_n": result.drag_force,
        "heat_solved": result.heat_solved,
        "outlet_c": result.heat_solved.then(|| celsius(result.outlet_temperature)),
        "max_c": result.heat_solved.then(|| celsius(result.max_temperature)),
        "heat_removed_w": result.heat_solved.then_some(result.heat_removed),
        "verdict": verdict,
        "warnings": warnings,
        "ms": started.elapsed().as_millis() as u64,
    }))
}

#[cfg(test)]
#[path = "flow_tests.rs"]
mod tests;
