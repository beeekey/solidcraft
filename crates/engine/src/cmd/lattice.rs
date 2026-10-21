//! Lattice (`construct.lattice`): a body filled with a triply periodic minimal surface, a strut
//! lattice or a honeycomb under a solid skin, the fill built on the body's distance field and
//! meshed by marching tetrahedra ([`solidcraft_analysis::lattice`]). The kit comes back as a
//! mesh body next to the source, the way Brickify's and the generative studies' bodies do.

use serde_json::{Value, json};
use solidcraft_analysis::lattice::{self, BeamCell, Family, Grading, LatticeOptions, TpmsType};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use crate::params::{bad, bool_, str_, vec3};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] =
    &[CommandSpec::new("construct.lattice", "Lattice", lattice_cmd).at("3DEEE", "STRUCTURE").icon("lattice").params(
        "body?: name (default: the selected body, or the design's only one); family: walled_tpms (a sheet TPMS thickened to thickness_mm) | \
     solid_tpms (the region below level) | beam (struts of thickness_mm along a cell's edges) | honeycomb (hexagonal cells along axis); \
     cell: gyroid (default) | schwarz_p | diamond | neovius | lidinoid | split_p | iwp for a TPMS, simple_cubic (default) | body_centred | \
     face_centred | octet | diamond | kelvin for beams; cell_mm: the period (1 to 500); thickness_mm: wall or strut diameter (0.05 to 100; \
     not for solid_tpms); level?: solid_tpms surface level, −1.4 to 1.4 (default 0, half full); skin_mm?: a solid skin inside the surface \
     (default 0); axis?: X | Y | Z the honeycomb's cells run along (default Z); grading?: uniform (default) | along_axis (thickness_min_mm \
     at grade_from to thickness_max_mm at grade_to, [x, y, z] mm) | from_point (thickness_min_mm at grade_from, thickness_max_mm \
     grade_radius_mm away) | by_stress (a stress solve with fixed, load, force_n and resolution? as inspect.stress takes them; the \
     thickness follows the von Mises stress from thickness_min_mm at nothing to thickness_max_mm at the peak); samples_per_cell?: 4 to 16 \
     (default 8; the detail, and the cost: sixteen is eight times eight's work); name?: the mesh body's name; hide?: true hides the \
     source body → body, built: the new body, family, cell, cell_mm, thickness_mm, volume_fraction, body_volume_mm3, fill_volume_mm3, \
     triangles, samples, sample_mm, fraction_near_fixed, fraction_near_load (by_stress), peak_von_mises_mpa (by_stress), advice",
    )];

const CMD: &str = "construct.lattice";

fn number(p: &Value, key: &str, default: Option<f64>, range: std::ops::RangeInclusive<f64>) -> Result<Option<f64>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v
            .as_f64()
            .filter(|x| x.is_finite() && range.contains(x))
            .map(Some)
            .ok_or_else(|| bad(CMD, format!("`{key}` must be a number from {} to {}", range.start(), range.end()))),
    }
}

fn required(p: &Value, key: &str, range: std::ops::RangeInclusive<f64>) -> Result<f64> {
    let (lo, hi) = (*range.start(), *range.end());
    number(p, key, None, range)?.ok_or_else(|| bad(CMD, format!("give `{key}` ({lo} to {hi})")))
}

fn point(p: &Value, key: &str) -> Result<Option<Vec3>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => vec3(v).filter(|v| v.is_finite() && v.len() < 1e7).map(Some).ok_or_else(|| bad(CMD, format!("`{key}` must be [x, y, z] in mm"))),
    }
}

/// The body: `body`, else the one selected body, else the design's only one.
fn body_name(s: &Session, st: &solidcraft_doc::ModelState, p: &Value) -> Result<String> {
    let name = match p.get("body") {
        None | Some(Value::Null) => {
            let selected: Vec<String> =
                s.selection.iter().filter_map(|x| if let crate::Sel::Body { name } = x { Some(name.clone()) } else { None }).collect();
            let all: Vec<String> = st.bodies.iter().map(|b| b.name.clone()).collect();
            match (selected.as_slice(), all.as_slice()) {
                ([one], _) | (_, [one]) => one.clone(),
                _ => return Err(bad(CMD, "give `body` (or select one body)")),
            }
        }
        Some(Value::String(n)) => n.clone(),
        Some(_) => return Err(bad(CMD, "`body` must be a body name")),
    };
    if st.body(&name).is_none() {
        return Err(bad(CMD, format!("no body `{name}`")));
    }
    Ok(name)
}

/// The stress field of `inspect.stress` with `field: true`, as a lookup of the nearest cell's
/// von Mises stress (MPa) at a point (mm).
struct StressLookup {
    origin: Vec3,
    h: f64,
    grid: [usize; 3],
    cells: Vec<f64>,
    peak: f64,
}

impl StressLookup {
    fn from_json(v: &Value) -> Option<StressLookup> {
        let field = v.get("field")?;
        let origin = vec3(field.get("origin_mm")?)?;
        let h = field.get("element_mm")?.as_f64().filter(|h| *h > 0.0 && h.is_finite())?;
        let g = field.get("grid")?.as_array()?;
        let dim = |i: usize| g.get(i)?.as_u64().and_then(|n| usize::try_from(n).ok());
        let grid = [dim(0)?, dim(1)?, dim(2)?];
        let count = grid[0].checked_mul(grid[1])?.checked_mul(grid[2])?;
        if count == 0 || count > 1 << 24 {
            return None;
        }
        let mut cells = vec![f64::NAN; count];
        let mut peak = 0.0_f64;
        for cell in field.get("cells")?.as_array()? {
            let index = cell.get(0)?.as_u64().and_then(|n| usize::try_from(n).ok())?;
            let mpa = cell.get(1)?.as_f64()?;
            if let Some(slot) = cells.get_mut(index) {
                *slot = mpa;
                peak = peak.max(mpa);
            }
        }
        Some(StressLookup { origin, h, grid, cells, peak })
    }

    /// The stress at the material cell nearest `p`, normalised by the peak (0 to 1).
    fn at(&self, p: Vec3) -> f64 {
        if self.peak <= 0.0 {
            return 0.0;
        }
        let local = (p - self.origin) / self.h;
        let clamp = |x: f64, n: usize| (x.floor().max(0.0) as usize).min(n.saturating_sub(1));
        let (i0, j0, k0) = (clamp(local.x, self.grid[0]), clamp(local.y, self.grid[1]), clamp(local.z, self.grid[2]));
        // The cell itself, else the nearest material cell within a short search.
        let at = |i: usize, j: usize, k: usize| self.cells.get((k * self.grid[1] + j) * self.grid[0] + i).copied().filter(|v| v.is_finite());
        if let Some(v) = at(i0, j0, k0) {
            return v / self.peak;
        }
        for reach in 1..=3usize {
            let mut best: Option<(usize, f64)> = None;
            for dk in 0..=2 * reach {
                for dj in 0..=2 * reach {
                    for di in 0..=2 * reach {
                        let (Some(i), Some(j), Some(k)) = ((i0 + di).checked_sub(reach), (j0 + dj).checked_sub(reach), (k0 + dk).checked_sub(reach))
                        else {
                            continue;
                        };
                        if i >= self.grid[0] || j >= self.grid[1] || k >= self.grid[2] {
                            continue;
                        }
                        let Some(v) = at(i, j, k) else { continue };
                        let d2 = (di.abs_diff(reach)).pow(2) + (dj.abs_diff(reach)).pow(2) + (dk.abs_diff(reach)).pow(2);
                        if best.is_none_or(|(bd, _)| d2 < bd) {
                            best = Some((d2, v));
                        }
                    }
                }
            }
            if let Some((_, v)) = best {
                return v / self.peak;
            }
        }
        0.0
    }
}

fn lattice_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let family_key =
        str_(p, "family").ok_or_else(|| bad(CMD, "give `family`: walled_tpms, solid_tpms, beam or honeycomb"))?.trim().to_ascii_lowercase();
    let cell_name = str_(p, "cell").map(str::trim).unwrap_or("");
    let family = match family_key.replace(['-', ' '], "_").as_str() {
        "walled_tpms" | "walled" | "sheet" | "sheet_tpms" => Family::WalledTpms(if cell_name.is_empty() {
            TpmsType::Gyroid
        } else {
            TpmsType::from_name(cell_name).ok_or_else(|| bad(CMD, tpms_help()))?
        }),
        "solid_tpms" | "solid" | "network" => Family::SolidTpms(if cell_name.is_empty() {
            TpmsType::Gyroid
        } else {
            TpmsType::from_name(cell_name).ok_or_else(|| bad(CMD, tpms_help()))?
        }),
        "beam" | "strut" | "struts" => Family::Beam(if cell_name.is_empty() {
            BeamCell::SimpleCubic
        } else {
            BeamCell::from_name(cell_name).ok_or_else(|| {
                bad(CMD, format!("`cell` for beams is one of {}", BeamCell::ALL.iter().map(|c| c.key()).collect::<Vec<_>>().join(", ")))
            })?
        }),
        "honeycomb" | "hex" | "hexagonal" => {
            let axis = match str_(p, "axis").map(|a| a.trim().to_ascii_uppercase()).as_deref() {
                None | Some("") | Some("Z") => 2,
                Some("X") => 0,
                Some("Y") => 1,
                Some(_) => return Err(bad(CMD, "`axis` is X, Y or Z")),
            };
            Family::Honeycomb { axis }
        }
        _ => return Err(bad(CMD, "`family` is walled_tpms, solid_tpms, beam or honeycomb")),
    };
    let cell_mm = required(p, "cell_mm", 1.0..=500.0)?;
    let solid = matches!(family, Family::SolidTpms(_));
    let thickness =
        if solid { number(p, "thickness_mm", Some(0.0), 0.0..=100.0)?.unwrap_or(0.0) } else { required(p, "thickness_mm", 0.05..=100.0)? };
    let level = number(p, "level", Some(0.0), -1.4..=1.4)?.unwrap_or(0.0);
    let skin = number(p, "skin_mm", Some(0.0), 0.0..=100.0)?.unwrap_or(0.0);
    let samples = number(p, "samples_per_cell", Some(8.0), 4.0..=16.0)?.unwrap_or(8.0) as usize;
    let grading_key = str_(p, "grading").map(|g| g.trim().to_ascii_lowercase().replace(['-', ' '], "_")).unwrap_or_else(|| "uniform".into());
    let (t_min, t_max) = (
        number(p, "thickness_min_mm", Some(thickness / 2.0), 0.05..=100.0)?.unwrap_or(thickness / 2.0),
        number(p, "thickness_max_mm", Some(thickness * 2.0), 0.05..=100.0)?.unwrap_or(thickness * 2.0),
    );
    if grading_key != "uniform" {
        if solid {
            return Err(bad(CMD, "a solid TPMS has no thickness to grade; grade a walled_tpms, beam or honeycomb"));
        }
        if t_max < t_min {
            return Err(bad(CMD, "`thickness_max_mm` must be at least `thickness_min_mm`"));
        }
    }

    let st = s.world_state();
    let name = body_name(s, &st, p)?;
    let body = st.body(&name).ok_or_else(|| bad(CMD, format!("no body `{name}`")))?;
    let mesh = body.mesh();
    let bounds = mesh.bounds();
    if bounds.is_empty() || mesh.triangles.is_empty() {
        return Err(bad(CMD, format!("`{name}` has no surface to fill")));
    }

    let mut stress: Option<(StressLookup, Vec3, Vec3)> = None;
    let grading = match grading_key.as_str() {
        "uniform" => Grading::Uniform,
        "along_axis" | "axis" => {
            let from = point(p, "grade_from")?.unwrap_or(bounds.min);
            let to = point(p, "grade_to")?.unwrap_or(bounds.max);
            if from.dist(to) < 1e-9 {
                return Err(bad(CMD, "`grade_from` and `grade_to` must be two different points"));
            }
            Grading::AlongAxis { from, to }
        }
        "from_point" | "radial" | "point" => {
            let centre = point(p, "grade_from")?.unwrap_or_else(|| bounds.center());
            let radius = number(p, "grade_radius_mm", Some(bounds.diagonal() / 2.0), 0.01..=1e6)?.unwrap_or(bounds.diagonal() / 2.0);
            Grading::FromPoint { centre, radius }
        }
        "by_stress" | "stress" => {
            let fixed = p.get("fixed").cloned().ok_or_else(|| bad(CMD, "by_stress needs `fixed`: the face the body is held at"))?;
            let load = p.get("load").cloned().ok_or_else(|| bad(CMD, "by_stress needs `load`: the face the force acts on"))?;
            let force = p.get("force_n").cloned().ok_or_else(|| bad(CMD, "by_stress needs `force_n`: [fx, fy, fz] newtons"))?;
            let mut q = json!({"body": name, "fixed": fixed, "load": load, "force_n": force, "field": true});
            if let Some(r) = p.get("resolution") {
                q["resolution"] = r.clone();
            }
            let answer = s.execute("inspect.stress", &q)?;
            let lookup = StressLookup::from_json(&answer).ok_or_else(|| bad(CMD, "the stress solve returned no field to grade by"))?;
            let picked = |v: &Value| vec3(v).or_else(|| v.get("point").and_then(vec3));
            let fixed_at = picked(&fixed).unwrap_or(bounds.min);
            let load_at = picked(&load).or_else(|| answer.get("max_displacement_at").and_then(vec3)).unwrap_or(bounds.max);
            stress = Some((lookup, fixed_at, load_at));
            Grading::Uniform
        }
        _ => return Err(bad(CMD, "`grading` is uniform, along_axis, from_point or by_stress")),
    };
    let stress_field = stress.as_ref().map(|(lookup, _, _)| move |q: Vec3| lookup.at(q));
    let grading = match (&stress_field, grading) {
        (Some(f), _) => Grading::Field(f),
        (None, g) => g,
    };
    let options = LatticeOptions {
        family,
        cell_mm,
        thickness_mm: thickness,
        thickness_min_mm: t_min,
        thickness_max_mm: t_max,
        level,
        skin_mm: skin,
        grading,
        samples_per_cell: samples,
    };
    let result = lattice::fill(&mesh, &options)
        .ok_or_else(|| bad(CMD, "the body could not be filled: it is empty, flat, or needs more samples than one fill may take"))?;
    if result.triangles == 0 {
        return Err(bad(
            CMD,
            format!("the fill left no material{}", if result.advice.is_empty() { String::new() } else { format!(": {}", result.advice) }),
        ));
    }
    let body_volume = mesh.measure().volume.abs();
    let mut out = json!({
        "body": name,
        "family": family.key(),
        "cell": family.cell_name(),
        "cell_mm": cell_mm,
        "thickness_mm": if solid { Value::Null } else { json!(thickness) },
        "level": if solid { json!(level) } else { Value::Null },
        "skin_mm": skin,
        "grading": grading_key,
        "volume_fraction": result.volume_fraction,
        "body_volume_mm3": body_volume,
        "fill_volume_mm3": body_volume * result.volume_fraction,
        "triangles": result.triangles,
        "samples": result.grid[0] * result.grid[1] * result.grid[2],
        "sample_mm": result.h_mm,
        "advice": result.advice,
    });
    if let Some((lookup, fixed_at, load_at)) = &stress {
        // The density of the fill in the quarter of the body nearest the clamp and nearest the
        // free end, so a reader can see the grading did something.
        let axis = *load_at - *fixed_at;
        let span = axis.len().max(1e-9);
        let along = |q: Vec3| (q - *fixed_at).dot(axis) / (span * span);
        out["fraction_near_fixed"] = json!(result.fraction_where(|q| along(q) < 0.25));
        out["fraction_near_load"] = json!(result.fraction_where(|q| along(q) > 0.75));
        out["peak_von_mises_mpa"] = json!(lookup.peak);
    }
    let kit_name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("{name} lattice"));
    let built = super::design::add_mesh_body(s, &kit_name, result.mesh, CMD)?;
    out["built"] = built;
    if bool_(p, "hide").unwrap_or(false) {
        let _ = s.execute("browser.visibility", &json!({"items": [name], "visible": false}));
    }
    Ok(out)
}

fn tpms_help() -> String {
    format!("`cell` for a TPMS is one of {}", TpmsType::ALL.iter().map(|t| t.key()).collect::<Vec<_>>().join(", "))
}

#[cfg(test)]
#[path = "lattice_tests.rs"]
mod tests;
