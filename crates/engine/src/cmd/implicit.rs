//! Implicit modelling and the stress tools around it: `construct.variable_shell` (hollow or
//! offset a body by a thickness that varies along it), `construct.blend_boolean` (join, cut or
//! intersect two bodies with a rounded seam), `inspect.stress_zone` (the hot spot of a stress
//! solve as its own body), `inspect.probe` (the stress at a point) and `inspect.export_fem`
//! (the voxel mesh and load case as a solver deck). The implicit results are mesh bodies.

use std::fmt::Write as _;

use serde_json::{Value, json};
use solidcraft_analysis::implicit::{self, Boolean, Field, Grading};
use solidcraft_analysis::{Region, handbook, voxelise};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use super::design::{add_mesh_body, number, run};
use super::stress::{body_name, region};
use crate::params::{bad, bool_, str_, vec3};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("construct.variable_shell", "Variable Shell", variable_shell).at("3DEEE", "STRUCTURE").icon("variable_shell").params(
        "body?: name (default: the selected body, or the design's only one); operation?: shell (default: hollow the body, keeping a skin) | \
         offset (grow it, shrink with a negative thickness); thickness_mm? (default 2); grading?: uniform (default) | along_axis | from_point; \
         grade_from?, grade_to?: [x, y, z] the axis (default: the body's extent along x), or grade_from? the centre and grade_radius_mm?; \
         thickness_min_mm?, thickness_max_mm?: the thickness at the two ends of the grading (default half and twice `thickness_mm`); sample_mm?: \
         the field's spacing (default: about 60 samples along the longest side); name?; hide?: true hides the source → body, operation, grading, \
         sample_mm, volume_mm3, body_volume_mm3, triangles, advice, built. The result is a mesh body",
    ),
    CommandSpec::new("construct.blend_boolean", "Blend Boolean", blend_boolean).at("3DEEE", "STRUCTURE").icon("blend_boolean").params(
        "body: the target; tool: the other body; operation?: join (default) | cut | intersect; blend_mm?: how far the seam is rounded (default \
         2; 0 is a plain boolean); sample_mm?: the field's spacing (default: about 80 samples along the longest side of the two); name?; hide?: true \
         hides both sources → body, tool, operation, blend_mm, sample_mm, volume_mm3, triangles, built. Two bodies joined on their distance \
         fields: a fillet-like seam where they meet, as a mesh body",
    ),
    CommandSpec::new("inspect.stress_zone", "Stress Zone", stress_zone).at("3DEEE", "GENERATIVE").icon("stress_zone").params(
        "body?, fixed, load, force_n, resolution?, young_modulus_pa?, poisson_ratio? as inspect.stress; threshold?: the fraction of the peak von \
         Mises stress a cell must reach (0.05 to 1, default 0.5), or threshold_mpa?; name? → body, peak_mpa, threshold_mpa, cells, cells_total, \
         fraction, built. The cells above the threshold become a mesh body of their own: the hot spot, where a fillet or a rib belongs",
    ),
    CommandSpec::new("inspect.probe", "Probe Result", probe).at("3DEEE", "GENERATIVE").icon("probe").noundo().params(
        "body?, fixed, load, force_n, resolution?, young_modulus_pa?, poisson_ratio? as inspect.stress; at: [x, y, z] the point to read → \
         body, at, cell, von_mises_mpa, peak_mpa, fraction_of_peak, element_mm, verdict. The stress solve's field read at one point: the cell the \
         point falls in, or the nearest material cell within one element",
    ),
    CommandSpec::new("inspect.export_fem", "Export FEM Case", export_fem).at("3DEEE", "GENERATIVE").icon("export_fem").noundo().params(
        "body?: name; fixed, load, force_n as inspect.stress; resolution?: cells along the longest side (6 to 60, default 24); young_modulus_pa?, \
         poisson_ratio?: the material (default: the body's); path: the directory to write into → body, path, files, nodes, elements, fixed_nodes, \
         loaded_nodes. Writes `case.inp`, a complete CalculiX deck of the voxel hexahedra (C3D8), the held nodes, the force spread over the loaded \
         nodes and the material, plus `solve.sh` to run it with `ccx` and `README.txt`",
    ),
];

fn implicit_body(s: &Session, p: &Value, cmd: &str, key: &str) -> Result<String> {
    match p.get(key) {
        Some(Value::String(n)) => {
            if s.world_state().body(n).is_none() {
                return Err(bad(cmd, format!("no body `{n}`")));
            }
            Ok(n.clone())
        }
        Some(Value::Null) | None if key == "body" => body_name(s, &s.world_state(), p, cmd),
        _ => Err(bad(cmd, format!("`{key}` must be a body name"))),
    }
}

fn grading_of(cmd: &str, p: &Value, thickness: f64, size_min: Vec3, size_max: Vec3) -> Result<(Grading, &'static str)> {
    let key = str_(p, "grading").map(|g| g.trim().to_ascii_lowercase().replace(['-', ' '], "_")).unwrap_or_else(|| "uniform".into());
    let min = number(cmd, p, "thickness_min_mm", Some(thickness / 2.0), -500.0..=500.0)?.unwrap_or(thickness / 2.0);
    let max = number(cmd, p, "thickness_max_mm", Some(thickness * 2.0), -500.0..=500.0)?.unwrap_or(thickness * 2.0);
    let point = |k: &str| -> Result<Option<Vec3>> {
        match p.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(v) => vec3(v).filter(|q| q.len() <= 1e6).map(Some).ok_or_else(|| bad(cmd, format!("`{k}` is a point [x, y, z]"))),
        }
    };
    Ok(match key.as_str() {
        "uniform" => (Grading::Uniform(thickness), "uniform"),
        "along_axis" | "axis" => {
            let from = point("grade_from")?.unwrap_or(size_min);
            let to = point("grade_to")?.unwrap_or(Vec3::new(size_max.x, size_min.y, size_min.z));
            if (to - from).len() < 1e-9 {
                return Err(bad(cmd, "`grade_from` and `grade_to` must be two different points"));
            }
            (Grading::AlongAxis { from, to, min, max }, "along_axis")
        }
        "from_point" | "radial" | "point" => {
            let centre = point("grade_from")?.unwrap_or((size_min + size_max) * 0.5);
            let radius =
                number(cmd, p, "grade_radius_mm", Some((size_max - size_min).len() / 2.0), 0.01..=1e6)?.unwrap_or((size_max - size_min).len() / 2.0);
            (Grading::FromPoint { centre, radius, min, max }, "from_point")
        }
        _ => return Err(bad(cmd, "`grading` is uniform, along_axis or from_point")),
    })
}

fn variable_shell(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "construct.variable_shell";
    let name = implicit_body(s, p, cmd, "body")?;
    let st = s.world_state();
    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let mesh = body.mesh();
    let bounds = mesh.bounds();
    let size = bounds.max - bounds.min;
    if mesh.triangles.is_empty() || !(size.x > 0.0 && size.y > 0.0 && size.z > 0.0) {
        return Err(bad(cmd, format!("`{name}` has no volume to shell")));
    }
    let shell_op = match str_(p, "operation").map(|o| o.trim().to_ascii_lowercase()).as_deref() {
        None | Some("shell") | Some("hollow") => true,
        Some("offset") | Some("grow") => false,
        Some(_) => return Err(bad(cmd, "`operation` is shell or offset")),
    };
    let thickness = number(cmd, p, "thickness_mm", Some(2.0), -500.0..=500.0)?.unwrap_or(2.0);
    if shell_op && !(thickness > 0.0) {
        return Err(bad(cmd, "a shell needs a positive `thickness_mm`"));
    }
    if thickness == 0.0 {
        return Err(bad(cmd, "`thickness_mm` must not be zero"));
    }
    let (grading, grading_name) = grading_of(cmd, p, thickness, bounds.min, bounds.max)?;
    if shell_op && grading.reach() <= 0.0 {
        return Err(bad(cmd, "the graded thickness must be positive"));
    }
    let suggested = implicit::spacing_for(size, 60.0, 0.1);
    let h = number(cmd, p, "sample_mm", Some(suggested), 0.05..=1000.0)?.unwrap_or(suggested);
    let reach = grading.reach();
    if h > reach / 1.5 && shell_op {
        return Err(bad(
            cmd,
            format!("the sample spacing ({h:.2} mm) is too coarse for a {reach:.2} mm wall: use sample_mm {:.2} or finer", reach / 2.0),
        ));
    }
    let field = implicit::sample_mesh(&mesh, h, reach + 2.0 * h, implicit::MAX_SAMPLES)
        .ok_or_else(|| bad(cmd, "the body cannot be sampled at this spacing: it needs more samples than one field may hold, or it is empty"))?;
    let result: Field = if shell_op { implicit::shell(&field, &grading) } else { implicit::offset(&field, &grading) };
    let out = result.mesh().filter(|m| !m.triangles.is_empty()).ok_or_else(|| bad(cmd, "nothing is left: the thickness swallows the body"))?;
    let volume = out.measure().volume.abs();
    let triangles = out.triangles.len();
    let mut advice = String::new();
    if h > reach / 3.0 && shell_op {
        advice = format!("walls of {reach:.2} mm are sampled {:.1} times: a finer sample_mm makes them truer", reach / h);
    }
    let kit = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{name} {}", if shell_op { "shell" } else { "offset" }));
    let built = add_mesh_body(s, &kit, out, cmd)?;
    if bool_(p, "hide").unwrap_or(false) {
        let _ = s.execute("browser.visibility", &json!({"items": [name], "visible": false}));
    }
    Ok(json!({
        "body": name,
        "operation": if shell_op { "shell" } else { "offset" },
        "grading": grading_name,
        "thickness_mm": thickness,
        "sample_mm": h,
        "volume_mm3": volume,
        "body_volume_mm3": mesh.measure().volume.abs(),
        "triangles": triangles,
        "advice": advice,
        "built": built,
    }))
}

fn blend_boolean(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "construct.blend_boolean";
    let a = implicit_body(s, p, cmd, "body")?;
    let b = match p.get("tool") {
        Some(Value::String(n)) if s.world_state().body(n).is_some() => n.clone(),
        Some(Value::String(n)) => return Err(bad(cmd, format!("no body `{n}`"))),
        _ => return Err(bad(cmd, "give `tool`: the other body")),
    };
    if a == b {
        return Err(bad(cmd, "`body` and `tool` must be two different bodies"));
    }
    let op = match str_(p, "operation") {
        None => Boolean::Join,
        Some(o) => Boolean::from_name(o).ok_or_else(|| bad(cmd, "`operation` is join, cut or intersect"))?,
    };
    let blend_mm = number(cmd, p, "blend_mm", Some(2.0), 0.0..=500.0)?.unwrap_or(2.0);
    let st = s.world_state();
    let (ma, mb) = (st.body(&a).map(|x| x.mesh()), st.body(&b).map(|x| x.mesh()));
    let (Some(ma), Some(mb)) = (ma, mb) else { return Err(bad(cmd, "no body")) };
    if ma.triangles.is_empty() || mb.triangles.is_empty() {
        return Err(bad(cmd, "both bodies need a surface"));
    }
    let (ba, bb) = (ma.bounds(), mb.bounds());
    let size = Vec3::new(
        ba.max.x.max(bb.max.x) - ba.min.x.min(bb.min.x),
        ba.max.y.max(bb.max.y) - ba.min.y.min(bb.min.y),
        ba.max.z.max(bb.max.z) - ba.min.z.min(bb.min.z),
    );
    let suggested = implicit::spacing_for(size, 80.0, 0.1);
    let h = number(cmd, p, "sample_mm", Some(suggested), 0.05..=1000.0)?.unwrap_or(suggested);
    let fields = implicit::sample_meshes(&[&ma, &mb], h, blend_mm.max(2.0 * h) + 2.0 * h, implicit::MAX_SAMPLES)
        .ok_or_else(|| bad(cmd, "the bodies cannot be sampled at this spacing: they need more samples than one field may hold"))?;
    let result = implicit::blend(&fields[0], &fields[1], op, blend_mm).ok_or_else(|| bad(cmd, "the fields do not match"))?;
    let out = result.mesh().filter(|m| !m.triangles.is_empty()).ok_or_else(|| bad(cmd, "nothing is left of the result"))?;
    let volume = out.measure().volume.abs();
    let triangles = out.triangles.len();
    let kit = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{a} {} {b}", op.name()));
    let built = add_mesh_body(s, &kit, out, cmd)?;
    if bool_(p, "hide").unwrap_or(false) {
        let _ = s.execute("browser.visibility", &json!({"items": [a, b], "visible": false}));
    }
    Ok(json!({
        "body": a,
        "tool": b,
        "operation": op.name(),
        "blend_mm": blend_mm,
        "sample_mm": h,
        "volume_mm3": volume,
        "triangles": triangles,
        "built": built,
    }))
}

/// `inspect.stress` with the field, for the commands that read it.
fn stress_field(s: &mut Session, p: &Value, cmd: &str) -> Result<(String, Value)> {
    let name = implicit_body(s, p, cmd, "body")?;
    let mut q = json!({"body": name, "field": true});
    for k in ["fixed", "load", "force_n", "resolution", "young_modulus_pa", "poisson_ratio", "yield_pa"] {
        if let Some(v) = p.get(k) {
            q[k] = v.clone();
        }
    }
    if q.get("fixed").is_none() {
        return Err(bad(cmd, "give `fixed`: the face the body is held at"));
    }
    if q.get("load").is_none() {
        return Err(bad(cmd, "give `load`: the face the force acts on"));
    }
    let answer = run(s, "inspect.stress", q)?;
    Ok((name, answer))
}

struct Cells {
    grid: [usize; 3],
    origin: Vec3,
    h: f64,
    /// (cell, MPa)
    cells: Vec<(usize, f64)>,
}

fn cells_of(answer: &Value, cmd: &str) -> Result<Cells> {
    let f = answer.get("field").ok_or_else(|| bad(cmd, "the stress solve returned no field"))?;
    let grid: Vec<usize> =
        f.get("grid").and_then(Value::as_array).map(|a| a.iter().filter_map(|v| v.as_u64()).map(|v| v as usize).collect()).unwrap_or_default();
    let origin = f.get("origin_mm").and_then(vec3).ok_or_else(|| bad(cmd, "the field has no origin"))?;
    let h = f.get("element_mm").and_then(Value::as_f64).filter(|h| *h > 0.0).ok_or_else(|| bad(cmd, "the field has no element size"))?;
    let cells: Vec<(usize, f64)> = f
        .get("cells")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(|c| Some((c.get(0)?.as_u64()? as usize, c.get(1)?.as_f64()?))).collect())
        .unwrap_or_default();
    if grid.len() != 3 || cells.is_empty() {
        return Err(bad(cmd, "the field is empty"));
    }
    Ok(Cells { grid: [grid[0], grid[1], grid[2]], origin, h, cells })
}

fn stress_zone(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.stress_zone";
    let fraction = number(cmd, p, "threshold", Some(0.5), 0.05..=1.0)?.unwrap_or(0.5);
    let threshold_mpa = number(cmd, p, "threshold_mpa", None, 0.0..=1e9)?;
    let (name, answer) = stress_field(s, p, cmd)?;
    let cells = cells_of(&answer, cmd)?;
    let peak = cells.cells.iter().map(|c| c.1).fold(0.0, f64::max);
    if !(peak > 0.0) {
        return Err(bad(cmd, "nothing is stressed"));
    }
    let threshold = threshold_mpa.unwrap_or(peak * fraction);
    let hot: std::collections::HashSet<usize> = cells.cells.iter().filter(|c| c.1 >= threshold).map(|c| c.0).collect();
    if hot.is_empty() {
        return Err(bad(cmd, format!("no cell reaches {threshold:.2} MPa; the peak is {peak:.2} MPa")));
    }
    let mesh = implicit::cells_mesh(cells.grid, cells.origin, cells.h, |v| hot.contains(&v))
        .filter(|m| !m.triangles.is_empty())
        .ok_or_else(|| bad(cmd, "the hot cells are too few to make a body: lower the threshold or raise the resolution"))?;
    let kit = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{name} stress zone"));
    let built = add_mesh_body(s, &kit, mesh, cmd)?;
    Ok(json!({
        "body": name,
        "peak_mpa": peak,
        "threshold_mpa": threshold,
        "cells": hot.len(),
        "cells_total": cells.cells.len(),
        "fraction": hot.len() as f64 / cells.cells.len() as f64,
        "max_von_mises_at": answer.get("max_von_mises_at").cloned().unwrap_or(Value::Null),
        "verdict": answer.get("verdict").cloned().unwrap_or(Value::Null),
        "built": built,
    }))
}

fn probe(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.probe";
    let at = p.get("at").and_then(vec3).filter(|q| q.len() <= 1e6).ok_or_else(|| bad(cmd, "`at` must be a point [x, y, z] on the body"))?;
    let (name, answer) = stress_field(s, p, cmd)?;
    let cells = cells_of(&answer, cmd)?;
    let [nx, ny, nz] = cells.grid;
    let rel = (at - cells.origin) / cells.h;
    let ijk = |q: Vec3| -> Option<usize> {
        let (i, j, k) = (q.x.floor(), q.y.floor(), q.z.floor());
        if i < 0.0 || j < 0.0 || k < 0.0 || i >= nx as f64 || j >= ny as f64 || k >= nz as f64 {
            return None;
        }
        Some((k as usize * ny + j as usize) * nx + i as usize)
    };
    let value = |v: usize| cells.cells.iter().find(|c| c.0 == v).map(|c| c.1);
    let mut found = ijk(rel).and_then(|v| value(v).map(|m| (v, m)));
    if found.is_none() {
        // The nearest material cell within one element.
        let mut best: Option<(f64, usize, f64)> = None;
        for (v, mpa) in &cells.cells {
            let (i, j, k) = (*v % nx, (*v / nx) % ny, *v / (nx * ny));
            let centre = cells.origin + Vec3::new(i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5) * cells.h;
            let d = (centre - at).len();
            if d <= 1.5 * cells.h && best.is_none_or(|b| d < b.0) {
                best = Some((d, *v, *mpa));
            }
        }
        found = best.map(|(_, v, m)| (v, m));
    }
    let (cell, mpa) = found.ok_or_else(|| bad(cmd, "the point lies outside the material"))?;
    let peak = cells.cells.iter().map(|c| c.1).fold(0.0, f64::max);
    let (i, j, k) = (cell % nx, (cell / nx) % ny, cell / (nx * ny));
    Ok(json!({
        "body": name,
        "at": [at.x, at.y, at.z],
        "cell": [i, j, k],
        "von_mises_mpa": mpa,
        "peak_mpa": peak,
        "fraction_of_peak": if peak > 0.0 { mpa / peak } else { 0.0 },
        "element_mm": cells.h,
        "verdict": format!("{mpa:.2} MPa here, {:.0}% of the peak {peak:.2} MPa", if peak > 0.0 { 100.0 * mpa / peak } else { 0.0 }),
    }))
}

fn export_fem(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.export_fem";
    let dir = str_(p, "path")
        .filter(|d| !d.trim().is_empty() && d.len() < 4096)
        .ok_or_else(|| bad(cmd, "give `path`: the directory to write the case into"))?
        .to_string();
    // A destination outside an agent's folders is refused before anything is solved.
    solidcraft_io::vfs::check(&dir).map_err(|e| bad(cmd, format!("{dir}: {e}")))?;
    let st = s.world_state();
    let name = body_name(s, &st, p, cmd)?;
    let fixed = p.get("fixed").ok_or_else(|| bad(cmd, "give `fixed`: the face the body is held at"))?;
    let load = p.get("load").ok_or_else(|| bad(cmd, "give `load`: the face the force acts on"))?;
    let fixed: Region = region(s, fixed, &name, cmd)?;
    let load: Region = region(s, load, &name, cmd)?;
    let force = p
        .get("force_n")
        .and_then(vec3)
        .filter(|f| f.len() > 0.0 && f.len() <= 1e9)
        .ok_or_else(|| bad(cmd, "`force_n` must be [fx, fy, fz] newtons, not zero and at most 1e9 N in all"))?;
    let resolution = number(cmd, p, "resolution", Some(24.0), 6.0..=60.0)?.unwrap_or(24.0) as usize;
    let body = st.body(&name).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
    let material_name = s.doc.materials.get(body.definition_name()).cloned().unwrap_or_else(|| "Default".to_string());
    let book = handbook::elastic(&material_name).or_else(|| handbook::elastic("Default"));
    let young =
        number(cmd, p, "young_modulus_pa", book.map(|e| e.young_modulus_pa), 1e3..=1e13)?.ok_or_else(|| bad(cmd, "give `young_modulus_pa`"))?;
    let poisson = number(cmd, p, "poisson_ratio", book.map(|e| e.poisson_ratio), 0.0..=0.49)?.unwrap_or(0.3);
    let mesh = body.mesh();
    let grid = voxelise(&mesh, resolution, 400_000).ok_or_else(|| bad(cmd, "the body cannot be voxelised at this resolution"))?;
    let held = grid.region(&mesh, &fixed);
    let loaded = grid.region(&mesh, &load);
    if held.is_empty() || loaded.is_empty() {
        return Err(bad(cmd, "the fixed or the load region hit no material"));
    }
    // Nodes of the solid cells, numbered from 1 in the order they are met.
    let (nx, ny) = (grid.nx, grid.ny);
    let node_id = |i: usize, j: usize, k: usize| (k * (ny + 1) + j) * (nx + 1) + i;
    let mut number_of: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    let mut nodes: Vec<(usize, Vec3)> = Vec::new();
    let mut elements: Vec<[usize; 8]> = Vec::new();
    let corners = [(0usize, 0usize, 0usize), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)];
    for (v, solid) in grid.solid.iter().enumerate() {
        if !solid {
            continue;
        }
        let (i, j, k) = grid.ijk(v);
        let mut element = [0usize; 8];
        for (c, (di, dj, dk)) in corners.iter().enumerate() {
            let id = node_id(i + di, j + dj, k + dk);
            let n = *number_of.entry(id).or_insert_with(|| {
                nodes.push((nodes.len() + 1, grid.origin + Vec3::new((i + di) as f64, (j + dj) as f64, (k + dk) as f64) * grid.h));
                nodes.len()
            });
            element[c] = n;
        }
        elements.push(element);
    }
    let set_of = |cells: &[usize]| -> Vec<usize> {
        let mut set: Vec<usize> = Vec::new();
        for &v in cells {
            let (i, j, k) = grid.ijk(v);
            for (di, dj, dk) in corners {
                if let Some(n) = number_of.get(&node_id(i + di, j + dj, k + dk))
                    && !set.contains(n)
                {
                    set.push(*n);
                }
            }
        }
        set.sort_unstable();
        set
    };
    let fixed_nodes = set_of(&held);
    let loaded_nodes = set_of(&loaded);
    if loaded_nodes.is_empty() {
        return Err(bad(cmd, "the load region has no nodes"));
    }
    let per_node = force / loaded_nodes.len() as f64;
    let mut deck = String::new();
    let _ = writeln!(deck, "** SolidCraft FEM case: `{name}` voxelised at {} x {} x {} cells of {:.4} m", grid.nx, grid.ny, grid.nz, grid.h);
    let _ = writeln!(deck, "** Units: m, N, Pa. {} nodes, {} C3D8 elements.", nodes.len(), elements.len());
    deck.push_str("*NODE, NSET=NALL\n");
    for (n, q) in &nodes {
        let _ = writeln!(deck, "{n}, {:.9}, {:.9}, {:.9}", q.x, q.y, q.z);
    }
    deck.push_str("*ELEMENT, TYPE=C3D8, ELSET=EALL\n");
    for (e, el) in elements.iter().enumerate() {
        let _ = writeln!(deck, "{}, {}", e + 1, el.iter().map(|n| n.to_string()).collect::<Vec<_>>().join(", "));
    }
    let write_set = |deck: &mut String, title: &str, set: &[usize]| {
        deck.push_str(title);
        for chunk in set.chunks(8) {
            deck.push_str(&chunk.iter().map(|n| n.to_string()).collect::<Vec<_>>().join(", "));
            deck.push('\n');
        }
    };
    write_set(&mut deck, "*NSET, NSET=NFIX\n", &fixed_nodes);
    write_set(&mut deck, "*NSET, NSET=NLOAD\n", &loaded_nodes);
    let _ = writeln!(deck, "*MATERIAL, NAME={}\n*ELASTIC\n{young:.6e}, {poisson:.4}", material_name.replace([' ', ','], "_"));
    let _ = writeln!(deck, "*SOLID SECTION, ELSET=EALL, MATERIAL={}", material_name.replace([' ', ','], "_"));
    deck.push_str("*STEP\n*STATIC\n*BOUNDARY\nNFIX, 1, 3, 0\n*CLOAD\n");
    for (axis, component) in [(1, per_node.x), (2, per_node.y), (3, per_node.z)] {
        if component != 0.0 {
            let _ = writeln!(deck, "NLOAD, {axis}, {component:.9e}");
        }
    }
    deck.push_str("*NODE FILE\nU\n*EL FILE\nS\n*END STEP\n");
    let readme = format!(
        "SolidCraft FEM case for `{name}`\n\nThe body is voxelised into {} hexahedra ({:.3} mm cells); the nodes of the cells at the held face are \
         fixed (NFIX) and {:.3} N is spread over the {} nodes of the loaded face (NLOAD) as {:.4}, {:.4}, {:.4} N each.\nMaterial `{}`: E = {young:.3e} \
         Pa, nu = {poisson}.\n\nRun: sh solve.sh (needs ccx, CalculiX, on PATH); results land in case.frd.\n",
        elements.len(),
        grid.h * 1000.0,
        force.len(),
        loaded_nodes.len(),
        per_node.x,
        per_node.y,
        per_node.z,
        material_name
    );
    let solve = "#!/bin/sh\n# SolidCraft FEM case: needs ccx (CalculiX) on PATH.\nset -e\nccx -i case\necho 'Results: case.frd (open with cgx or ParaView after conversion)'\n";
    solidcraft_io::vfs::create_dir_all(&dir).map_err(|e| bad(cmd, format!("cannot create `{dir}`: {e}")))?;
    let files = [("case.inp", deck), ("solve.sh", solve.to_string()), ("README.txt", readme)];
    for (file, text) in &files {
        let path = std::path::Path::new(&dir).join(file).to_string_lossy().to_string();
        solidcraft_io::vfs::write(&path, text.as_bytes()).map_err(|e| bad(cmd, format!("cannot write `{file}` into `{dir}`: {e}")))?;
    }
    Ok(json!({
        "body": name,
        "path": dir,
        "files": files.iter().map(|(f, _)| *f).collect::<Vec<_>>(),
        "nodes": nodes.len(),
        "elements": elements.len(),
        "fixed_nodes": fixed_nodes.len(),
        "loaded_nodes": loaded_nodes.len(),
        "material": material_name,
        "young_modulus_pa": young,
        "poisson_ratio": poisson,
        "force_per_node_n": [per_node.x, per_node.y, per_node.z],
    }))
}

#[cfg(test)]
#[path = "implicit_tests.rs"]
mod tests;
