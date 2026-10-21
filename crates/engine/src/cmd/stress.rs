//! Stress Analysis (`inspect.stress`): hold a body at one face, push on another, and ask how far
//! it moves, how hard it is stressed and whether it holds. The body is voxelised and solved by
//! the native finite-element solver ([`solidcraft_analysis`]); the material's elastic constants
//! come from the design's material through a small handbook, or from the call itself.

use serde_json::{Value, json};
use solidcraft_analysis::{Assembly, ContactOptions, MeshFemError, MeshFemOptions, OrthotropicMaterial, Region, handbook, solve_assembly};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use crate::params::{bad, bool_, str_};
use crate::{Result, Sel, Session};

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new("inspect.stress", "Stress Analysis", stress).at("SOLID", "INSPECT").icon("measure").noundo().params(
    "body?: name (default: the selected body, or the design's only one), in world placement, or bodies?: [names] (2 to 16) solved together on one grid, \
     bonded where they touch unless contact is set; fixed: a face (a point [x, y, z] on it, its name from model.faces, or \
     {face: index, body?: name (default: the first body)}): the body is clamped over that face (a point on no face: on the side of its box nearest the point); load: a face the same way \
     (body? default: the last body): the force is spread over it; force_n: [fx, fy, fz] newtons (total, up to 1e9 N); contact?: true | {friction?: coefficient (default 0.15), \
     preload_n?: a clamp holding the interfaces shut, what a bolt does}: the interfaces between the bodies push but do not pull (open where they lift, slide \
     within the friction); young_modulus_pa?, poisson_ratio?, yield_pa?: the material (default: each body's \
     physical material through the built-in handbook, else a generic plastic; the other bodies' stiffness relative to the first); resolution?: elements along the longest side (6 to 60, default 24); \
     gravity?: true (9.81 m/s² along −Z) | [x, y, z] m/s²; resting?: true when the mount only pushes back (a foot on a table rather than a bolt); \
     buckling?: true also finds the load factor at which it buckles (the eigenvalue of a perfect member), how much a real member's bow of L/500 \
     grows at this load and the bending stress that adds; collapse?: true then bows it into that mode and walks the load on until no equilibrium is \
     left, the load a real member actually folds at (slow: a nonlinear solve per step); \
     field?: true also returns the stress of every material cell (origin_mm, element_mm, grid, cells: [[index, von_mises_mpa]…], x fastest) → \
     body, material, young_modulus_pa, poisson_ratio, yield_mpa?, grid, voxels, converged, max_displacement_mm, max_displacement_at, \
     load_displacement_mm (mean of the loaded side along the force), max_von_mises_mpa, max_von_mises_at, safety_factor? (yield / peak), \
     lifted_nodes, contact? (links, open_links, stuck_links, sliding_links, rounds, slid_free), mesh_advice (empty when the mesh is fine enough), verdict, warnings, ms. Linear elasticity on trilinear hexahedra, so \
     a clamp and a point load are stress singularities: read the peak with the mesh advice",
)];

/// Largest force (N) and modulus (Pa) a call may give.
const MAX_FORCE: f64 = 1e9;
const MAX_MODULUS: f64 = 1e13;
const MIN_MODULUS: f64 = 1e3;
/// Default elements along the longest side.
const DEFAULT_RESOLUTION: usize = 24;

/// A positive number in `range`, `default` when absent.
fn positive(cmd: &str, p: &Value, key: &str, default: Option<f64>, range: std::ops::RangeInclusive<f64>) -> Result<Option<f64>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v
            .as_f64()
            .filter(|x| x.is_finite() && range.contains(x))
            .map(Some)
            .ok_or_else(|| bad(cmd, format!("`{key}` must be a number from {} to {}", range.start(), range.end()))),
    }
}

/// The body: `body`, else the one selected body, else the design's only one.
pub(super) fn body_name(s: &Session, st: &solidcraft_doc::ModelState, p: &Value, cmd: &str) -> Result<String> {
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

fn gravity(cmd: &str, p: &Value) -> Result<Vec3> {
    match p.get("gravity") {
        None | Some(Value::Null) | Some(Value::Bool(false)) => Ok(Vec3::ZERO),
        Some(Value::Bool(true)) => Ok(Vec3::new(0.0, 0.0, -9.81)),
        Some(v) => {
            crate::params::vec3(v).filter(|g| g.len() <= 1000.0).ok_or_else(|| bad(cmd, "`gravity` is true or [x, y, z] in m/s² (up to 1000)"))
        }
    }
}

/// A held or loaded region: `{face: index}` names a face of the body outright; a point or a face
/// name becomes the face under it, or, when no face is near enough, the box side nearest it.
pub(super) fn region(s: &Session, v: &Value, body: &str, cmd: &str) -> Result<Region> {
    let st = s.world_state();
    let b = st.body(body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
    let mesh = b.mesh();
    let triangles_of =
        |face: usize| -> Vec<usize> { mesh.tri_face.iter().enumerate().filter(|(_, f)| **f as usize == face).map(|(t, _)| t).collect() };
    if let Some(face) = v.get("face").and_then(Value::as_u64) {
        let face = usize::try_from(face).unwrap_or(usize::MAX);
        let triangles = triangles_of(face);
        if triangles.is_empty() {
            return Err(bad(cmd, format!("`{body}` has no face {face}")));
        }
        return Ok(Region::Face(triangles));
    }
    let point = super::face::face_ref(s, v, Some(body), cmd)?;
    match solidcraft_doc::appearance::face_index_at(b, point).map(triangles_of).filter(|t| !t.is_empty()) {
        Some(triangles) => Ok(Region::Face(triangles)),
        None => Ok(Region::NearestWall(point)),
    }
}

/// The bodies: `bodies` (2 to 16 names), else the one body `body` names.
fn body_names(s: &Session, st: &solidcraft_doc::ModelState, p: &Value, cmd: &str) -> Result<Vec<String>> {
    match p.get("bodies") {
        None | Some(Value::Null) => Ok(vec![body_name(s, st, p, cmd)?]),
        Some(Value::Array(a)) => {
            if a.len() < 2 || a.len() > 16 {
                return Err(bad(cmd, "`bodies` lists 2 to 16 bodies (one body: `body`)"));
            }
            let mut names = Vec::with_capacity(a.len());
            for v in a {
                let n = v.as_str().ok_or_else(|| bad(cmd, "`bodies` must be body names"))?;
                if st.body(n).is_none() {
                    return Err(bad(cmd, format!("no body `{n}`")));
                }
                if names.iter().any(|x: &String| x == n) {
                    return Err(bad(cmd, format!("`{n}` is listed twice")));
                }
                names.push(n.to_string());
            }
            Ok(names)
        }
        Some(_) => Err(bad(cmd, "`bodies` must be a list of body names")),
    }
}

/// A region on one of the bodies: `body` inside it picks which, else the `default` one.
fn region_on(s: &Session, v: &Value, names: &[String], default: usize, cmd: &str) -> Result<(usize, Region)> {
    let which = match v.get("body").and_then(Value::as_str) {
        Some(b) => names.iter().position(|n| n == b).ok_or_else(|| bad(cmd, format!("`{b}` is not one of the bodies")))?,
        None => default,
    };
    let name = names.get(which).ok_or_else(|| bad(cmd, "no body"))?;
    Ok((which, region(s, v, name, cmd)?))
}

fn contact_options(cmd: &str, p: &Value) -> Result<Option<ContactOptions>> {
    match p.get("contact") {
        None | Some(Value::Null) | Some(Value::Bool(false)) => Ok(None),
        Some(Value::Bool(true)) => Ok(Some(ContactOptions { friction: 0.15, preload_n: 0.0 })),
        Some(v @ Value::Object(_)) => Ok(Some(ContactOptions {
            friction: positive(cmd, v, "friction", Some(0.15), 0.0..=2.0)?.unwrap_or(0.15),
            preload_n: positive(cmd, v, "preload_n", Some(0.0), 0.0..=MAX_FORCE)?.unwrap_or(0.0),
        })),
        Some(_) => Err(bad(cmd, "`contact` is true or {friction?, preload_n?}")),
    }
}

fn stress(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.stress";
    let started = web_time::Instant::now();
    let st = s.world_state();
    let names = body_names(s, &st, p, cmd)?;
    let name = names.first().cloned().ok_or_else(|| bad(cmd, "no body"))?;
    let fixed = p.get("fixed").ok_or_else(|| bad(cmd, "give `fixed`: the face the body is held at"))?;
    let load = p.get("load").ok_or_else(|| bad(cmd, "give `load`: the face the force acts on"))?;
    let fixed = region_on(s, fixed, &names, 0, cmd)?;
    let load = region_on(s, load, &names, names.len() - 1, cmd)?;
    let contact = contact_options(cmd, p)?;
    if contact.is_some() && names.len() < 2 {
        return Err(bad(cmd, "`contact` needs `bodies`: two or more bodies that touch"));
    }
    let force = p
        .get("force_n")
        .and_then(crate::params::vec3)
        .filter(|f| f.len() > 0.0 && f.len() <= MAX_FORCE)
        .ok_or_else(|| bad(cmd, format!("`force_n` must be [fx, fy, fz] newtons, not zero and at most {MAX_FORCE:e} N in all")))?;
    let resolution = positive(cmd, p, "resolution", Some(DEFAULT_RESOLUTION as f64), 6.0..=60.0)?.unwrap_or(DEFAULT_RESOLUTION as f64) as usize;
    let options = MeshFemOptions {
        gravity: gravity(cmd, p)?,
        resting_support: bool_(p, "resting").unwrap_or(false),
        lift_direction: None,
        solve_buckling: bool_(p, "buckling").unwrap_or(false),
        follow_to_collapse: bool_(p, "collapse").unwrap_or(false),
        contact,
    };
    if str_(p, "mode").is_some_and(|m| !m.is_empty()) {
        return Err(bad(cmd, "`mode` is not an option of this command"));
    }

    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let material_name = s.doc.materials.get(body.definition_name()).cloned().unwrap_or_else(|| "Default".to_string());
    let book = handbook::elastic(&material_name);
    let mut warnings: Vec<String> = Vec::new();
    if book.is_none() && p.get("young_modulus_pa").is_none() {
        warnings.push(format!("no elastic constants are known for `{material_name}`: a generic plastic was assumed; give young_modulus_pa"));
    }
    let fallback = handbook::elastic("Default");
    let young = positive(cmd, p, "young_modulus_pa", book.or(fallback).map(|e| e.young_modulus_pa), MIN_MODULUS..=MAX_MODULUS)?
        .ok_or_else(|| bad(cmd, "give `young_modulus_pa`"))?;
    let poisson = positive(cmd, p, "poisson_ratio", book.or(fallback).map(|e| e.poisson_ratio), 0.0..=0.49)?.unwrap_or(0.3);
    let yield_pa = positive(cmd, p, "yield_pa", book.map(|e| e.yield_pa), MIN_MODULUS..=MAX_MODULUS)?;
    let density_kg_m3 = s.doc.density(body.definition_name()) * 1000.0;
    let material = OrthotropicMaterial::isotropic(young, poisson);

    // The other bodies' stiffness relative to the first, from their own materials (an override
    // given outright applies to them all).
    let mut meshes = Vec::with_capacity(names.len());
    let mut modulus_scale = Vec::with_capacity(names.len());
    let mut material_names: Vec<String> = Vec::with_capacity(names.len());
    for n in &names {
        let b = st.body(n).ok_or_else(|| bad(cmd, format!("no body `{n}`")))?;
        let m = s.doc.materials.get(b.definition_name()).cloned().unwrap_or_else(|| "Default".to_string());
        let e = if p.get("young_modulus_pa").is_some() { young } else { handbook::elastic(&m).or(fallback).map_or(young, |e| e.young_modulus_pa) };
        if names.len() > 1 && handbook::elastic(&m).is_none() && p.get("young_modulus_pa").is_none() {
            warnings.push(format!("no elastic constants are known for `{m}` ({n}): a generic plastic was assumed"));
        }
        modulus_scale.push(e / young);
        material_names.push(m);
        meshes.push(b.mesh());
    }
    let assembly = Assembly { meshes: meshes.iter().map(|m| m.as_ref()).collect(), modulus_scale, fixed, load };
    let result = solve_assembly(&assembly, force, &material, density_kg_m3, resolution, &options).map_err(|e| match e {
        MeshFemError::Input => bad(cmd, "the body or the material cannot be analysed"),
        other => bad(cmd, other.to_string()),
    })?;
    if let Some(c) = &result.contact {
        if c.slid_free {
            warnings.push("the joint slid: the friction could not hold a body against the load, so its answer is a sliding one".into());
        }
        if c.open_links > 0 {
            warnings.push(format!("{} of {} contact links opened: the parts lift apart there", c.open_links, c.links));
        }
    }
    if !result.converged {
        warnings.push("the solve did not converge: the numbers are approximate".into());
    }
    if !result.mesh_advice.is_empty() {
        warnings.push(result.mesh_advice.clone());
    }
    let field = bool_(p, "field").unwrap_or(false).then(|| {
        let cells: Vec<(usize, f64)> = result
            .solid
            .iter()
            .enumerate()
            .filter(|(_, solid)| **solid)
            .map(|(i, _)| (i, result.voxel_stress.get(i).copied().unwrap_or(0.0) / 1e6))
            .collect();
        json!({ "origin_mm": result.origin * 1000.0, "element_mm": result.h * 1000.0, "grid": result.grid, "cells": cells })
    });
    let mm = |v: Vec3| v * 1000.0;
    let peak_mpa = result.max_von_mises / 1e6;
    let safety = yield_pa.filter(|_| result.max_von_mises > 0.0).map(|y| y / result.max_von_mises);
    let buckles = options.solve_buckling && result.buckling_factor > 0.0 && result.buckling_factor < 1.0;
    let collapses = options.follow_to_collapse && result.collapse_factor > 0.0 && result.collapse_factor < 1.0;
    let verdict = match safety {
        _ if collapses => format!("collapses: a real member with an L/500 bow folds at {:.2}× this load", result.collapse_factor),
        _ if buckles => format!("buckles: the load is {:.1}× what it takes to fold it", 1.0 / result.buckling_factor),
        Some(sf) if sf >= 2.0 => format!("holds: peak {peak_mpa:.1} MPa, safety factor {sf:.1}"),
        Some(sf) if sf >= 1.0 => format!("holds, barely: peak {peak_mpa:.1} MPa, safety factor {sf:.2}"),
        Some(sf) => format!("yields: peak {peak_mpa:.1} MPa is {:.1}× the material's {:.0} MPa", 1.0 / sf, yield_pa.unwrap_or(0.0) / 1e6),
        None => format!("peak {peak_mpa:.1} MPa; no yield stress to judge it against"),
    };
    Ok(json!({
        "body": name,
        "bodies": (names.len() > 1).then_some(&names),
        "materials": (names.len() > 1).then_some(&material_names),
        "contact": result.contact.as_ref().map(|c| json!({
            "friction": contact.map(|o| o.friction),
            "preload_n": contact.map(|o| o.preload_n),
            "links": c.links,
            "open_links": c.open_links,
            "stuck_links": c.stuck_links,
            "sliding_links": c.sliding_links,
            "rounds": c.rounds,
            "slid_free": c.slid_free,
        })),
        "material": material_name,
        "material_note": book.map(|e| e.note),
        "young_modulus_pa": young,
        "poisson_ratio": poisson,
        "yield_mpa": yield_pa.map(|y| y / 1e6),
        "density_kg_m3": density_kg_m3,
        "grid": result.grid,
        "voxels": result.voxels,
        "element_mm": result.h * 1000.0,
        "converged": result.converged,
        "residual_ratio": result.residual_ratio,
        "max_displacement_mm": result.max_displacement * 1000.0,
        "max_displacement_at": mm(result.max_displacement_at),
        "load_displacement_mm": result.load_displacement * 1000.0,
        "max_von_mises_mpa": peak_mpa,
        "max_von_mises_at": mm(result.max_von_mises_at),
        "safety_factor": safety,
        "lifted_nodes": result.lifted_nodes,
        "buckling": options.solve_buckling.then(|| json!({
            "load_factor": result.buckling_factor,
            "amplification": result.buckling_amplification,
            "second_order_stress_mpa": result.second_order_stress / 1e6,
            "advice": result.buckling_advice,
            "collapse_factor": result.collapse_factor,
            "collapse_deflection_mm": result.collapse_deflection * 1000.0,
            "collapse_advice": result.collapse_advice,
        })),
        "elements_through_thinnest": result.elements_through_thinnest,
        "mesh_advice": result.mesh_advice,
        "verdict": verdict,
        "warnings": warnings,
        "field": field,
        "ms": started.elapsed().as_millis() as u64,
    }))
}

#[cfg(test)]
#[path = "stress_tests.rs"]
mod tests;
