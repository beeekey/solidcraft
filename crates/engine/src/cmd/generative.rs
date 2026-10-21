//! Generative design: inventing the shape instead of analysing one.
//!
//! `generative.study` searches skeletons of struts and plates between the mounts and the loads,
//! scored by the voxel finite elements (or by conduction and the flow solver for a heat sink or
//! a part in a stream), and offers several genuinely different designs; `build: n` adds one as a
//! mesh body. `generative.free_form` grows a density-field body straight from the boundary
//! conditions. `generative.check` verifies a body on a finer mesh with buckling, fatigue and
//! makeability. `inspect.fibre` checks a stress solve in the frame of a material's grain.

use serde_json::{Value, json};
use solidcraft_analysis::fibre_failure::{BAMBOO, FibreState, FibreStrength, bamboo_strength, check_fibre_failure};
use solidcraft_analysis::generative::{
    CheckOptions, Design, DesignGoal, DesignLoad, DesignRegion, FieldOptions, GenerativeProblem, RegionShape, SurfaceFinish, check_design,
    design_mesh, generative_search, mesh_from_mask, refine_to_field,
};
use solidcraft_analysis::makeability::MakeabilityRules;
use solidcraft_analysis::{LoadCase, OrthotropicMaterial, Region, VoxelFemProblem, handbook, solve_voxels, voxelise};
use solidcraft_doc::{FeatureKind, MeshData};
use solidcraft_geom::{Aabb3, Mesh, Vec3};

use super::CommandSpec;
use crate::params::{bad, bool_, str_};
use crate::{Result, Sel, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("generative.study", "Generative Design", study).at("3DEEE", "GENERATIVE").icon("generative_design").params(
        "mounts: [region…] where the part is held; loads: [{…region, force_n?: [x, y, z], watts?, cycles?, stress_ratio?}] where it is pushed or \
         heated; a region is {face: index, point: [x, y, z], body?: name, pad_mm?: thickness (default 3)} (a face of an existing body, taken as a \
         slab that thick) or {box: {min: [x, y, z], max: [x, y, z]}} (mm), with name?, bonded? (false for a foot that only rests, with lift?: [x, y, \
         z]); design_space?: a body name (its volume) or {box} (default: the box round the regions, grown by half); material?: a handbook name \
         (default: the design-space body's material, else steel), or young_modulus_pa?, yield_pa?, density_kg_m3?; safety_factor? (default 2); \
         goal?: carry_load (default) | remove_heat | low_drag; stream?: {axis?: x|y|z, reversed?, velocity_m_s?, fluid?: Water|Air|…, \
         inlet_c?, checks?, steps?}; temperature_limit_c? (default 85); gravity?: true|false (default true); buckling_factor?, fatigue_damage? \
         (constraints during the search); printable?, mouldable?, machinable? (+ tool_mm?, pull_axis?, build_axis?); resolution? (8 to 48, default \
         18), population? (6 to 200, default 20), generations? (1 to 200, default 10), solutions? (1 to 10, default 3), seed?, free_nodes? (0 to 8), \
         min_radius_mm?, max_radius_mm?; build?: n adds design n (0 = the best) as a mesh body named name? → designs: [{index, mass_g, \
         safety_factor, max_stress_mpa, max_displacement_mm, feasible, struts, plates, nodes_mm, peak_c?, drag_n?, pressure_drop_pa?, \
         buckling_factor?, fatigue_damage?, verdict, summary}], evaluations, fem_evaluations, stream_checks, mass_history_g, advice, warnings, \
         built?: {feature, body}, ms",
    ),
    CommandSpec::new("generative.free_form", "Free-Form Body", free_form).at("3DEEE", "GENERATIVE").icon("free_form").params(
        "the same mounts, loads, design_space and material as generative.study; volume_fraction? (0.02 to 0.9, default 0.3); resolution? (12 \
         to 60, default 30); iterations? (4 to 200, default 30); minimum_feature_mm?; meet_target? (default true: tune the volume to the \
         lightest body that reaches the safety factor); build? (default true) adds the body as a mesh body named name? → mass_g, \
         volume_fraction, safety_factor, max_stress_mpa, max_displacement_mm, iterations, compliance_history, advice, built?, ms",
    ),
    CommandSpec::new("generative.check", "Check Design", check).at("3DEEE", "GENERATIVE").icon("generative_check").noundo().params(
        "body: name, with the same mounts, loads and material as generative.study; resolution? (16 to 96, default 40); cyclic?: true when the \
         loads repeat; finish?: Polished|Ground|Machined|Hot-rolled|As forged|As printed; endurance_pa?, ultimate_pa?; overhang_degrees? → \
         safety_factor, max_stress_mpa, max_displacement_mm, mass_g, elements_through_thinnest, thinnest_mm, buckling: {assessed, factor, \
         note}, fatigue: {assessed, cycles_to_failure, damage, lifetimes, note}, makeability: {overhang_fraction, undercut_fraction, \
         draft_fraction, unreachable_fraction, setup_count}, manufacturing_note, verdict, warnings",
    ),
    CommandSpec::new("inspect.fibre", "Grain Failure", fibre).at("3DEEE", "GENERATIVE").icon("grain_failure").noundo().params(
        "body?: name; fixed, load: faces as inspect.stress takes them; force_n: [x, y, z]; fibre: [x, y, z] the grain direction; species?: \
         Tonkin|Moso|Guadua|Generic bamboo (its strengths and stiffnesses), or along_tension_pa, along_compression_pa, across_tension_pa, \
         across_compression_pa, shear_pa with young_modulus_pa?; resolution? (6 to 60, default 24) → worst_ratio (1 = at strength), mode, \
         safety, ratios per mode, von_mises_ratio, advice, verdict, warnings. Von Mises is the wrong question for a material with a grain; this \
         is the right one",
    ),
];

const MM: f64 = 1000.0;

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

fn body_mesh(s: &Session, name: &str, cmd: &str) -> Result<std::sync::Arc<Mesh>> {
    let st = s.world_state();
    let b = st.body(name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    Ok(b.mesh())
}

/// A region as the study takes it: a face of a body as a slab, or a box (mm in, metres out).
fn region(s: &Session, v: &Value, cmd: &str, what: &str) -> Result<DesignRegion> {
    let obj = v.as_object().ok_or_else(|| bad(cmd, format!("a {what} region is {{face, point, body?}} or {{box: {{min, max}}}}")))?;
    let name = obj.get("name").and_then(Value::as_str).unwrap_or(what).chars().take(64).collect::<String>();
    let shape = if let Some(b) = obj.get("box") {
        let (min, max) = (b.get("min").and_then(crate::params::vec3), b.get("max").and_then(crate::params::vec3));
        match (min, max) {
            (Some(min), Some(max)) if max.x > min.x && max.y > min.y && max.z > min.z => RegionShape::Box(Aabb3 { min: min / MM, max: max / MM }),
            _ => return Err(bad(cmd, format!("the {what} box needs min and max [x, y, z] in mm with a positive size"))),
        }
    } else {
        let body = obj.get("body").and_then(Value::as_str).map(str::to_string).or_else(|| {
            let st = s.world_state();
            match st.bodies.as_slice() {
                [one] => Some(one.name.clone()),
                _ => None,
            }
        });
        let body = body.ok_or_else(|| bad(cmd, format!("the {what} face needs `body` when the design has more than one")))?;
        let mesh = body_mesh(s, &body, cmd)?;
        let face = match obj.get("face").and_then(Value::as_u64) {
            Some(f) => usize::try_from(f).unwrap_or(usize::MAX),
            None => {
                let point = obj
                    .get("point")
                    .and_then(crate::params::vec3)
                    .ok_or_else(|| bad(cmd, format!("the {what} face needs `face` (an index) or `point` on it")))?;
                let st = s.world_state();
                let b = st.body(&body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
                solidcraft_doc::appearance::face_index_at(b, point)
                    .ok_or_else(|| bad(cmd, format!("no face of `{body}` is near the {what} point")))?
            }
        };
        let mut bounds = Aabb3::EMPTY;
        let mut any = false;
        for (t, f) in mesh.triangles.iter().zip(&mesh.tri_face) {
            if *f as usize != face {
                continue;
            }
            if let Some(corners) = mesh.tri(t) {
                for c in corners {
                    bounds.add(c);
                    any = true;
                }
            }
        }
        if !any {
            return Err(bad(cmd, format!("`{body}` has no face {face}")));
        }
        let pad = number(cmd, v, "pad_mm", Some(3.0), 0.1..=100.0)?.unwrap_or(3.0);
        let half = Vec3::new(pad, pad, pad) * 0.5;
        RegionShape::Box(Aabb3 { min: (bounds.min - half) / MM, max: (bounds.max + half) / MM })
    };
    let mut region = DesignRegion::new(shape, &name);
    region.bonded = obj.get("bonded").and_then(Value::as_bool).unwrap_or(true);
    if let Some(lift) = obj.get("lift").and_then(crate::params::vec3).and_then(|l| l.normalized()) {
        region.lift_direction = lift;
    }
    Ok(region)
}

/// The problem common to the three design commands.
fn problem_of(s: &Session, p: &Value, cmd: &str) -> Result<(GenerativeProblem, Vec<String>)> {
    let mut warnings = Vec::new();
    let mounts = p.get("mounts").and_then(Value::as_array).ok_or_else(|| bad(cmd, "give `mounts`: a list of regions the part is held at"))?;
    let loads =
        p.get("loads").and_then(Value::as_array).ok_or_else(|| bad(cmd, "give `loads`: a list of regions the part is pushed or heated at"))?;
    if mounts.is_empty() || mounts.len() > 16 || loads.is_empty() || loads.len() > 16 {
        return Err(bad(cmd, "1 to 16 mounts and 1 to 16 loads"));
    }
    let mut problem = GenerativeProblem::default();
    for m in mounts {
        problem.mounts.push(region(s, m, cmd, "mount")?);
    }
    for l in loads {
        let force = l.get("force_n").and_then(crate::params::vec3).unwrap_or(Vec3::ZERO);
        if force.len() > 1e9 {
            return Err(bad(cmd, "`force_n` is at most 1e9 N"));
        }
        problem.loads.push(DesignLoad {
            region: region(s, l, cmd, "load")?,
            force,
            watts: number(cmd, l, "watts", Some(0.0), 0.0..=1e6)?.unwrap_or(0.0),
            cycles: number(cmd, l, "cycles", Some(0.0), 0.0..=1e15)?.unwrap_or(0.0),
            stress_ratio: number(cmd, l, "stress_ratio", Some(0.0), -1.0..=0.99)?.unwrap_or(0.0),
        });
    }
    let mut material_name: Option<String> = None;
    match p.get("design_space") {
        None | Some(Value::Null) => {}
        Some(Value::String(name)) => {
            let mesh = body_mesh(s, name, cmd)?;
            let st = s.world_state();
            if let Some(b) = st.body(name) {
                material_name = s.doc.materials.get(b.definition_name()).cloned();
            }
            problem.design_space = Some(RegionShape::Mesh((*mesh).clone()));
        }
        Some(v) => match v.get("box") {
            Some(b) => {
                let (min, max) = (b.get("min").and_then(crate::params::vec3), b.get("max").and_then(crate::params::vec3));
                match (min, max) {
                    (Some(min), Some(max)) if max.x > min.x && max.y > min.y && max.z > min.z => {
                        problem.design_space = Some(RegionShape::Box(Aabb3 { min: min / MM, max: max / MM }))
                    }
                    _ => return Err(bad(cmd, "`design_space.box` needs min and max [x, y, z] in mm with a positive size")),
                }
            }
            None => return Err(bad(cmd, "`design_space` is a body name or {box: {min, max}}")),
        },
    }
    let name = str_(p, "material").map(str::to_string).or(material_name).unwrap_or_else(|| "Steel".into());
    let book = handbook::elastic(&name);
    if book.is_none() && p.get("young_modulus_pa").is_none() {
        warnings.push(format!("no elastic constants are known for `{name}`: steel was assumed"));
    }
    let fallback = handbook::elastic("Steel");
    problem.youngs_modulus = number(cmd, p, "young_modulus_pa", book.or(fallback).map(|e| e.young_modulus_pa), 1e3..=1e13)?.unwrap_or(200e9);
    problem.poisson_ratio = number(cmd, p, "poisson_ratio", book.or(fallback).map(|e| e.poisson_ratio), 0.0..=0.49)?.unwrap_or(0.3);
    problem.yield_strength = number(cmd, p, "yield_pa", book.or(fallback).map(|e| e.yield_pa), 1e3..=1e12)?.unwrap_or(250e6);
    problem.density = number(
        cmd,
        p,
        "density_kg_m3",
        Some(if name.eq_ignore_ascii_case("steel") { 7850.0 } else { s.doc.material_density(&name).map(|d| d * 1000.0).unwrap_or(7850.0) }),
        1.0..=30_000.0,
    )?
    .unwrap_or(7850.0);
    problem.target_safety_factor = number(cmd, p, "safety_factor", Some(2.0), 1.0..=20.0)?.unwrap_or(2.0);
    problem.goal = match str_(p, "goal").unwrap_or("carry_load") {
        "carry_load" => DesignGoal::CarryLoad,
        "remove_heat" => DesignGoal::RemoveHeat,
        "low_drag" => DesignGoal::LowDrag,
        _ => return Err(bad(cmd, "`goal` is carry_load, remove_heat or low_drag")),
    };
    if let Some(stream) = p.get("stream").filter(|v| v.is_object()) {
        problem.stream.axis = match stream.get("axis").and_then(Value::as_str).unwrap_or("x") {
            "x" | "X" => 0,
            "y" | "Y" => 1,
            "z" | "Z" => 2,
            _ => return Err(bad(cmd, "`stream.axis` is x, y or z")),
        };
        problem.stream.reversed = stream.get("reversed").and_then(Value::as_bool).unwrap_or(false);
        problem.stream.velocity = number(cmd, stream, "velocity_m_s", Some(0.15), 1e-4..=100.0)?.unwrap_or(0.15);
        let fluid = solidcraft_analysis::fluid_named(stream.get("fluid").and_then(Value::as_str).unwrap_or("Air"));
        problem.stream.density = fluid.density;
        problem.stream.viscosity = fluid.viscosity;
        problem.stream.conductivity = fluid.conductivity;
        problem.stream.heat_capacity = fluid.heat_capacity;
        problem.stream.inlet_temperature = number(cmd, stream, "inlet_c", Some(20.0), -100.0..=500.0)?.unwrap_or(20.0) + 273.15;
        problem.stream.solver_checks = number(cmd, stream, "checks", Some(2.0), 0.0..=10.0)?.unwrap_or(2.0) as usize;
        problem.stream.solver_steps = number(cmd, stream, "steps", Some(4000.0), 200.0..=20_000.0)?.unwrap_or(4000.0) as usize;
    } else {
        problem.stream.solver_checks = 2;
        problem.stream.solver_steps = 4000;
    }
    problem.temperature_limit = number(cmd, p, "temperature_limit_c", Some(85.0), -100.0..=2000.0)?.unwrap_or(85.0) + 273.15;
    problem.solid_conductivity = handbook::thermal(&name).map(|t| t.conductivity).unwrap_or(50.0);
    if !bool_(p, "gravity").unwrap_or(true) {
        problem.gravity = Vec3::ZERO;
    }
    problem.target_buckling_factor = number(cmd, p, "buckling_factor", Some(0.0), 0.0..=100.0)?.unwrap_or(0.0);
    problem.target_fatigue_damage = number(cmd, p, "fatigue_damage", Some(0.0), 0.0..=1.0)?.unwrap_or(0.0);
    let axis = |key: &str| -> Result<usize> {
        match p.get(key) {
            None | Some(Value::Null) => Ok(2),
            Some(Value::String(a)) => match a.as_str() {
                "x" | "X" => Ok(0),
                "y" | "Y" => Ok(1),
                "z" | "Z" => Ok(2),
                _ => Err(bad(cmd, format!("`{key}` is x, y or z"))),
            },
            _ => Err(bad(cmd, format!("`{key}` is x, y or z"))),
        }
    };
    problem.manufacturing = MakeabilityRules {
        printable_without_support: bool_(p, "printable").unwrap_or(false),
        build_axis: axis("build_axis")?,
        mouldable: bool_(p, "mouldable").unwrap_or(false),
        pull_axis: axis("pull_axis")?,
        machinable: bool_(p, "machinable").unwrap_or(false),
        tool_radius: number(cmd, p, "tool_mm", Some(6.0), 0.1..=100.0)?.unwrap_or(6.0) / 2.0 / MM,
        weight: 1.0,
        ..Default::default()
    };
    problem.resolution = number(cmd, p, "resolution", Some(18.0), 8.0..=48.0)?.unwrap_or(18.0) as usize;
    problem.population = number(cmd, p, "population", Some(20.0), 6.0..=200.0)?.unwrap_or(20.0) as usize;
    problem.generations = number(cmd, p, "generations", Some(10.0), 1.0..=200.0)?.unwrap_or(10.0) as usize;
    problem.solutions = number(cmd, p, "solutions", Some(3.0), 1.0..=10.0)?.unwrap_or(3.0) as usize;
    problem.seed = number(cmd, p, "seed", Some(1.0), 0.0..=1e15)?.unwrap_or(1.0) as u64;
    problem.free_nodes = number(cmd, p, "free_nodes", Some(3.0), 0.0..=8.0)?.unwrap_or(3.0) as usize;
    problem.min_radius = number(cmd, p, "min_radius_mm", Some(2.5), 0.1..=1000.0)?.unwrap_or(2.5) / MM;
    problem.max_radius = number(cmd, p, "max_radius_mm", Some(25.0), 0.1..=1000.0)?.unwrap_or(25.0) / MM;
    if problem.max_radius < problem.min_radius {
        return Err(bad(cmd, "`max_radius_mm` must be at least `min_radius_mm`"));
    }
    Ok((problem, warnings))
}

/// Adds a closed mesh (mm) to the design as a mesh body.
fn add_mesh_body(s: &mut Session, p: &Value, mesh: Mesh, default_name: &str, cmd: &str) -> Result<Value> {
    solidcraft_engine_kernel_check(&mesh).map_err(|e| bad(cmd, format!("the built mesh is not usable: {e}")))?;
    let name = str_(p, "name").filter(|n| !n.trim().is_empty()).unwrap_or(default_name).chars().take(64).collect::<String>();
    let data = MeshData {
        name: name.clone(),
        positions: mesh.positions.iter().flat_map(|p| [p.x, p.y, p.z]).collect(),
        triangles: mesh.triangles.iter().flatten().copied().collect(),
        color: None,
    };
    let kind = FeatureKind::MeshImport { file: "generative design".into(), meshes: vec![data] };
    let mut params = json!({});
    params["name"] = json!(name);
    super::features::add_feature(s, &params, kind)
}

fn solidcraft_engine_kernel_check(mesh: &Mesh) -> std::result::Result<(), String> {
    solidcraft_kernel::mesh_body(&mesh.positions, &mesh.triangles).map(|_| ()).map_err(|e| e.to_string())
}

fn design_json(index: usize, d: &Design) -> Value {
    json!({
        "index": index,
        "mass_g": d.mass * 1000.0,
        "safety_factor": d.safety_factor,
        "max_stress_mpa": d.max_stress / 1e6,
        "max_displacement_mm": d.max_displacement * 1000.0,
        "stiffness_n_mm": d.stiffness / 1000.0,
        "feasible": d.feasible,
        "connected": d.connected,
        "struts": d.members.iter().map(|m| json!({"a": m.a, "b": m.b, "radius_mm": m.radius * 1000.0})).collect::<Vec<_>>(),
        "plates": d.panels.iter().map(|p| json!({"a": p.a, "b": p.b, "c": p.c, "thickness_mm": p.thickness * 1000.0})).collect::<Vec<_>>(),
        "nodes_mm": d.nodes.iter().map(|n| *n * 1000.0).collect::<Vec<_>>(),
        "peak_c": (d.peak_temperature > 0.0).then_some(d.peak_temperature - 273.15),
        "heat_removed_w": (d.heat_removed > 0.0).then_some(d.heat_removed),
        "drag_n": (d.drag > 0.0).then_some(d.drag),
        "pressure_drop_pa": (d.pressure_drop > 0.0).then_some(d.pressure_drop),
        "checked_by_flow_solver": d.checked_by_flow_solver,
        "buckling_factor": (d.buckling_factor > 0.0).then_some(d.buckling_factor),
        "fatigue_damage": (d.fatigue_damage > 0.0).then_some(d.fatigue_damage),
        "unmakeable_fraction": d.unmakeable_fraction,
        "generation": d.generation,
        "verdict": d.verdict,
        "summary": solidcraft_analysis::generative::describe_design(d),
    })
}

fn study(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "generative.study";
    let started = web_time::Instant::now();
    let (problem, mut warnings) = problem_of(s, p, cmd)?;
    let build = match p.get("build") {
        None | Some(Value::Null) | Some(Value::Bool(false)) => None,
        Some(Value::Bool(true)) => Some(0usize),
        Some(v) => Some(v.as_u64().filter(|n| *n < 10).ok_or_else(|| bad(cmd, "`build` is true or a design index 0 to 9"))? as usize),
    };
    let result = generative_search(&problem, None);
    if !result.solved {
        return Err(bad(cmd, result.advice));
    }
    for sentence in result.advice.split(". ").map(str::trim).filter(|a| !a.is_empty()) {
        warnings.push(sentence.trim_end_matches('.').to_string());
    }
    let mut out = json!({
        "designs": result.designs.iter().enumerate().map(|(i, d)| design_json(i, d)).collect::<Vec<_>>(),
        "evaluations": result.evaluations,
        "fem_evaluations": result.fem_evaluations,
        "stream_checks": result.stream_checks,
        "mass_history_g": result.mass_history.iter().map(|m| m * 1000.0).collect::<Vec<_>>(),
        "grid": [result.grid.nx, result.grid.ny, result.grid.nz],
        "element_mm": result.grid.h * 1000.0,
        "advice": result.advice,
        "warnings": warnings,
    });
    if let Some(n) = build {
        let design = result.designs.get(n).ok_or_else(|| bad(cmd, format!("there is no design {n}: {} were found", result.designs.len())))?;
        let mesh = design_mesh(&result.grid, design, 4).ok_or_else(|| bad(cmd, "this design has no struts to build"))?;
        let built = add_mesh_body(s, p, mesh, &format!("Design {}", n + 1), cmd)?;
        out["built"] = built;
    }
    out["ms"] = json!(started.elapsed().as_millis() as u64);
    Ok(out)
}

fn free_form(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "generative.free_form";
    let started = web_time::Instant::now();
    let (problem, warnings) = problem_of(s, p, cmd)?;
    let options = FieldOptions {
        resolution: number(cmd, p, "resolution", Some(30.0), 12.0..=60.0)?.unwrap_or(30.0) as usize,
        volume_fraction: number(cmd, p, "volume_fraction", Some(0.3), 0.02..=0.9)?.unwrap_or(0.3),
        minimum_feature: number(cmd, p, "minimum_feature_mm", Some(0.0), 0.0..=1000.0)?.unwrap_or(0.0) / MM,
        iterations: number(cmd, p, "iterations", Some(30.0), 4.0..=200.0)?.unwrap_or(30.0) as usize,
        meet_target: bool_(p, "meet_target").unwrap_or(true),
        meet_target_tries: 2,
        ..Default::default()
    };
    let body = refine_to_field(&problem, &Design::default(), &options);
    if !body.solved {
        return Err(bad(cmd, body.advice));
    }
    let mut out = json!({
        "mass_g": body.mass * 1000.0,
        "volume_fraction": body.volume_fraction,
        "safety_factor": body.safety_factor,
        "max_stress_mpa": body.max_stress / 1e6,
        "max_displacement_mm": body.max_displacement * 1000.0,
        "iterations": body.iterations,
        "compliance_history": body.compliance_history,
        "grid": [body.nx, body.ny, body.nz],
        "advice": body.advice,
        "warnings": warnings,
    });
    if bool_(p, "build").unwrap_or(true) {
        let mesh = mesh_from_mask(&body.mask, body.nx, body.ny, body.nz, body.h, body.origin, 4)
            .ok_or_else(|| bad(cmd, "the refined body has no surface: it may have been optimised away entirely"))?;
        out["built"] = add_mesh_body(s, p, mesh, "Free-Form Body", cmd)?;
    }
    out["ms"] = json!(started.elapsed().as_millis() as u64);
    Ok(out)
}

fn check(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "generative.check";
    let started = web_time::Instant::now();
    let name = str_(p, "body").ok_or_else(|| bad(cmd, "give `body`: the name of the body to check"))?;
    let mesh = body_mesh(s, name, cmd)?;
    let (problem, warnings) = problem_of(s, p, cmd)?;
    let options = CheckOptions {
        resolution: number(cmd, p, "resolution", Some(40.0), 16.0..=96.0)?.unwrap_or(40.0) as usize,
        cyclic_loading: bool_(p, "cyclic").unwrap_or(false),
        endurance_limit: number(cmd, p, "endurance_pa", Some(0.0), 0.0..=1e12)?.unwrap_or(0.0),
        ultimate_strength: number(cmd, p, "ultimate_pa", Some(0.0), 0.0..=1e12)?.unwrap_or(0.0),
        finish: SurfaceFinish::from_name(str_(p, "finish").unwrap_or("Machined")),
        print_axis: 2,
        overhang_degrees: number(cmd, p, "overhang_degrees", Some(45.0), 0.0..=90.0)?.unwrap_or(45.0),
    };
    let check = check_design(&problem, &mesh, &options);
    if !check.solved {
        return Err(bad(cmd, check.advice));
    }
    Ok(json!({
        "body": name,
        "resolution": check.resolution,
        "safety_factor": check.safety_factor,
        "max_stress_mpa": check.max_stress / 1e6,
        "max_displacement_mm": check.max_displacement * 1000.0,
        "mass_g": check.mass * 1000.0,
        "surface_area_cm2": check.surface_area * 1e4,
        "elements_through_thinnest": check.elements_through_thinnest,
        "thinnest_mm": check.thinnest_feature * 1000.0,
        "buckling": {"assessed": check.buckling_assessed, "factor": check.buckling_factor, "note": check.buckling_note},
        "fatigue": {"assessed": check.fatigue_assessed, "cycles_to_failure": check.cycles_to_failure, "damage": check.fatigue_damage, "lifetimes": check.blocks_to_failure, "note": check.fatigue_note},
        "makeability": {
            "overhang_fraction": check.makeability.overhang_fraction,
            "undercut_fraction": check.makeability.undercut_fraction,
            "draft_fraction": check.makeability.draft_fraction,
            "unreachable_fraction": check.makeability.unreachable_fraction,
            "setup_count": check.makeability.setup_count,
        },
        "overhang_area_cm2": check.overhang_area * 1e4,
        "manufacturing_note": check.manufacturing_note,
        "verdict": check.verdict,
        "warnings": warnings,
        "ms": started.elapsed().as_millis() as u64,
    }))
}

fn fibre(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.fibre";
    let started = web_time::Instant::now();
    let st = s.world_state();
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
    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let mesh = body.mesh();
    let triangles_of =
        |face: usize| -> Vec<usize> { mesh.tri_face.iter().enumerate().filter(|(_, f)| **f as usize == face).map(|(t, _)| t).collect() };
    let region_of = |key: &str| -> Result<Region> {
        let v = p.get(key).ok_or_else(|| bad(cmd, format!("give `{key}`: a face")))?;
        if let Some(face) = v.get("face").and_then(Value::as_u64) {
            let triangles = triangles_of(usize::try_from(face).unwrap_or(usize::MAX));
            if triangles.is_empty() {
                return Err(bad(cmd, format!("`{name}` has no face {face}")));
            }
            return Ok(Region::Face(triangles));
        }
        let point = super::face::face_ref(s, v, Some(&name), cmd)?;
        Ok(match solidcraft_doc::appearance::face_index_at(body, point).map(triangles_of).filter(|t| !t.is_empty()) {
            Some(triangles) => Region::Face(triangles),
            None => Region::NearestWall(point),
        })
    };
    let fixed = region_of("fixed")?;
    let load = region_of("load")?;
    let force = p
        .get("force_n")
        .and_then(crate::params::vec3)
        .filter(|f| f.len() > 0.0 && f.len() <= 1e9)
        .ok_or_else(|| bad(cmd, "`force_n` must be [fx, fy, fz] newtons, not zero"))?;
    let fibre = p
        .get("fibre")
        .and_then(crate::params::vec3)
        .and_then(|f| f.normalized())
        .ok_or_else(|| bad(cmd, "`fibre` is the grain direction [x, y, z]"))?;
    let species = str_(p, "species");
    let (mut strength, species_name) = match species {
        Some(sp) => (bamboo_strength(sp).ok_or_else(|| bad(cmd, "`species` is Tonkin, Moso, Guadua or Generic bamboo"))?, Some(sp.to_string())),
        None => (FibreStrength::default(), None),
    };
    let given = |key: &str, current: f64| number(cmd, p, key, Some(current), 0.0..=1e12).map(|v| v.unwrap_or(current));
    strength.along_tension = given("along_tension_pa", strength.along_tension)?;
    strength.along_compression = given("along_compression_pa", strength.along_compression)?;
    strength.across_tension = given("across_tension_pa", strength.across_tension)?;
    strength.across_compression = given("across_compression_pa", strength.across_compression)?;
    strength.shear = given("shear_pa", strength.shear)?;
    if !(strength.along_tension > 0.0 && strength.across_tension > 0.0 && strength.shear > 0.0) {
        return Err(bad(cmd, "give `species` or the strengths along_tension_pa, across_tension_pa and shear_pa"));
    }
    let sp = species_name.as_deref().and_then(|n| BAMBOO.iter().find(|b| b.name.eq_ignore_ascii_case(n.trim())));
    let material = match sp {
        Some(b) => OrthotropicMaterial::along_fibre(b.modulus_along, b.modulus_across, b.shear_modulus, 0.3, fibre),
        None => {
            let e = number(cmd, p, "young_modulus_pa", Some(15e9), 1e3..=1e13)?.unwrap_or(15e9);
            OrthotropicMaterial::along_fibre(e, e / 20.0, e / 25.0, 0.3, fibre)
        }
    };
    let resolution = number(cmd, p, "resolution", Some(24.0), 6.0..=60.0)?.unwrap_or(24.0) as usize;
    let grid = voxelise(&mesh, resolution, 400_000).ok_or_else(|| bad(cmd, "the body cannot be voxelised at this resolution"))?;
    let held = grid.region(&mesh, &fixed);
    let loaded = grid.region(&mesh, &load);
    if held.is_empty() || loaded.is_empty() {
        return Err(bad(cmd, "the fixed or the load region hit no material"));
    }
    let mut problem = VoxelFemProblem::new(grid.nx, grid.ny, grid.nz, grid.h, material);
    problem.origin = grid.origin;
    problem.solid = grid.solid.clone();
    for &v in &held {
        problem.held[v] = true;
    }
    problem.max_iterations = 1600;
    problem.cases.push(LoadCase { voxels: loaded, force, name: "applied".into() });
    let solved = solve_voxels(&problem);
    if !solved.solved {
        return Err(bad(cmd, solved.advice));
    }
    let state = FibreState {
        stress_tensor: solved.voxel_stress_tensor.clone(),
        solid: problem.solid.clone(),
        exclude_from_peak: Vec::new(),
        fibre_direction: fibre,
        worst_von_mises: solved.worst_von_mises,
    };
    let report = check_fibre_failure(&state, &strength);
    if !report.valid {
        return Err(bad(cmd, report.advice));
    }
    let worst_at = report.worst_voxel.map(|v| grid.centre(v) * MM);
    let verdict = if report.worst_ratio >= 1.0 {
        format!("fails by {} at {:.2}× strength", report.mode, report.worst_ratio)
    } else {
        format!("holds: {} governs at {:.2} of strength (safety {:.2})", report.mode, report.worst_ratio, 1.0 / report.worst_ratio.max(1e-12))
    };
    Ok(json!({
        "body": name,
        "species": species_name,
        "fibre": fibre,
        "grid": [grid.nx, grid.ny, grid.nz],
        "converged": solved.cases.first().is_some_and(|c| c.converged),
        "worst_ratio": report.worst_ratio,
        "safety": if report.worst_ratio > 0.0 { 1.0 / report.worst_ratio } else { f64::INFINITY },
        "mode": report.mode,
        "worst_at": worst_at,
        "along_tension_ratio": report.along_tension_ratio,
        "along_compression_ratio": report.along_compression_ratio,
        "across_tension_ratio": report.across_tension_ratio,
        "across_compression_ratio": report.across_compression_ratio,
        "shear_ratio": report.shear_ratio,
        "von_mises_ratio": report.von_mises_ratio_against_along_strength,
        "max_von_mises_mpa": solved.worst_von_mises / 1e6,
        "advice": report.advice,
        "verdict": verdict,
        "warnings": Vec::<String>::new(),
        "ms": started.elapsed().as_millis() as u64,
    }))
}

#[cfg(test)]
#[path = "generative_tests.rs"]
mod tests;
