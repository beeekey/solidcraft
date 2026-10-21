//! Stress Analysis: pick a body, the face it is held at and the face the load acts on, type the
//! force, and Compute runs `inspect.stress`. The answer is the dialog's rows (displacement, peak
//! stress, safety factor, the solver's own advice about its mesh) and, over the model, the
//! material cells coloured by their von Mises stress, blue for nothing to red for the peak.

use egui::{Color32, Pos2, RichText};
use serde_json::{Value, json};
use solidcraft_engine::Sel;
use solidcraft_engine::geom::Vec3;

use crate::SolveApp;
use crate::dialogs::{field, form_row};
use crate::selection::SelInput;
use crate::theme::Tokens;
use crate::viewport::Proj;

/// The dialog's state.
#[derive(Clone, Debug)]
pub struct StressForm {
    /// The force's components (N), as typed.
    pub force: [String; 3],
    /// Elements along the longest side, as typed.
    pub resolution: String,
    /// Count the part's own weight.
    pub gravity: bool,
    /// The mount only pushes back.
    pub resting: bool,
    /// Also find the buckling load factor, and follow a bowed member to its collapse.
    pub buckling: bool,
    pub collapse: bool,
    /// With several bodies: 0 bonded, 1 sliding freely, 2 in contact with friction (and a preload).
    pub contact: usize,
    pub friction: String,
    pub preload: String,
    /// The last answer, and the picks and values it was for.
    pub result: Option<Value>,
    pub of: Option<(Vec<Sel>, [String; 3], String, bool, bool, bool, bool, (usize, String, String))>,
    /// The stress field of the last answer, as cells to paint.
    pub field: Option<StressField>,
}

impl Default for StressForm {
    fn default() -> Self {
        StressForm {
            force: ["0".into(), "0".into(), "-100".into()],
            resolution: "24".into(),
            gravity: false,
            resting: false,
            buckling: false,
            collapse: false,
            contact: 0,
            friction: "0.15".into(),
            preload: "0".into(),
            result: None,
            of: None,
            field: None,
        }
    }
}

/// Material cells and a value per cell (a stress, a temperature), ready to paint.
#[derive(Clone, Debug, Default)]
pub struct StressField {
    pub origin_mm: Vec3,
    pub element_mm: f64,
    pub grid: [usize; 3],
    /// Dense value per cell; `None` where there is no material.
    pub cells: Vec<Option<f64>>,
    /// The largest and smallest values among the cells.
    pub max_mpa: f64,
    pub min: f64,
}

impl StressField {
    pub(crate) fn from_json(v: &Value) -> Option<StressField> {
        let origin = v.get("origin_mm").and_then(|o| {
            let a = o.as_array()?;
            Some(Vec3::new(a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2)?.as_f64()?))
        })?;
        let element_mm = v.get("element_mm")?.as_f64().filter(|h| *h > 0.0)?;
        let g = v.get("grid")?.as_array()?;
        let grid = [g.first()?.as_u64()? as usize, g.get(1)?.as_u64()? as usize, g.get(2)?.as_u64()? as usize];
        let count = grid[0].checked_mul(grid[1])?.checked_mul(grid[2])?;
        if count > 4_000_000 {
            return None;
        }
        let mut cells = vec![None; count];
        let mut max_mpa = f64::NEG_INFINITY;
        let mut min = f64::INFINITY;
        for cell in v.get("cells")?.as_array()? {
            let a = cell.as_array()?;
            let index = a.first()?.as_u64()? as usize;
            let stress = a.get(1)?.as_f64()?;
            if let Some(slot) = cells.get_mut(index) {
                *slot = Some(stress);
                max_mpa = max_mpa.max(stress);
                min = min.min(stress);
            }
        }
        if !max_mpa.is_finite() || !min.is_finite() {
            return None;
        }
        Some(StressField { origin_mm: origin, element_mm, grid, cells, max_mpa, min })
    }

    fn index(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.grid[1] + j) * self.grid[0] + i
    }

    fn filled(&self, i: isize, j: isize, k: isize) -> bool {
        if i < 0 || j < 0 || k < 0 {
            return false;
        }
        let (i, j, k) = (i as usize, j as usize, k as usize);
        if i >= self.grid[0] || j >= self.grid[1] || k >= self.grid[2] {
            return false;
        }
        self.cells.get(self.index(i, j, k)).copied().flatten().is_some()
    }
}

/// Blue through green and yellow to red, for 0 to 1.
pub fn ramp(t: f64) -> Color32 {
    // The Curvature Map's three stops (flat, middle, tight), so the ramp follows the theme and
    // its colour-vision modes: nothing to little, the middle of the scale, the peak.
    let stops = Tokens::get().analysis.curvature;
    let t = t.clamp(0.0, 1.0) as f32;
    let (from, to, u) = if t < 0.5 { (stops[0], stops[1], t * 2.0) } else { (stops[1], stops[2], (t - 0.5) * 2.0) };
    // Exact at the stops: a rounding hair off at the ends would make the legend's end colours
    // differ from the theme's.
    let mix = |a: f32, b: f32| if u >= 1.0 { b } else { a + (b - a) * u };
    crate::theme::srgb([mix(from[0], to[0]), mix(from[1], to[1]), mix(from[2], to[2])])
}

/// A picked face as the command takes it: its index, and the point for a body that lost it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FacePick {
    pub index: usize,
    pub point: Vec3,
}

/// A picked face as the command takes it, with the body it is on.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceOn {
    pub pick: FacePick,
    pub body: String,
}

/// The bodies picked (the faces' bodies count too, the fixed face's first), and the two faces.
fn picks(inputs: &[SelInput]) -> (Vec<String>, Option<FaceOn>, Option<FaceOn>) {
    let mut bodies: Vec<String> = Vec::new();
    let mut add = |name: &str| {
        if !bodies.iter().any(|b| b == name) {
            bodies.push(name.to_string());
        }
    };
    for s in inputs.first().map(|i| i.items.as_slice()).unwrap_or_default() {
        match s {
            Sel::Body { name } => add(name),
            Sel::Face { body, .. } => add(body),
            _ => {}
        }
    }
    let face = |n: usize| {
        inputs.get(n).and_then(|i| i.items.first()).and_then(|s| match s {
            Sel::Face { index, point, body, .. } => Some(FaceOn { pick: FacePick { index: *index, point: *point }, body: body.clone() }),
            _ => None,
        })
    };
    let (fixed, load) = (face(1), face(2));
    // Without an explicit body, the faces' bodies are the ones analysed.
    for f in [&fixed, &load].into_iter().flatten() {
        add(&f.body);
    }
    (bodies, fixed, load)
}

fn all_picks(inputs: &[SelInput]) -> Vec<Sel> {
    inputs.iter().flat_map(|i| i.items.iter().cloned()).collect()
}

/// The dialog's rows. Returns false: Enter never applies here, Compute does.
pub fn rows(app: &mut SolveApp, ui: &mut egui::Ui, f: &mut StressForm, inputs: &[SelInput]) -> bool {
    let t = Tokens::get();
    form_row(ui, "Force X (N)", |ui| {
        field(ui, &mut f.force[0]);
    });
    form_row(ui, "Force Y (N)", |ui| {
        field(ui, &mut f.force[1]);
    });
    form_row(ui, "Force Z (N)", |ui| {
        field(ui, &mut f.force[2]);
    });
    form_row(ui, "Resolution", |ui| {
        field(ui, &mut f.resolution);
    });
    form_row(ui, "", |ui| {
        crate::ui_kit::checkbox(ui, "Own weight", &mut f.gravity);
    });
    form_row(ui, "", |ui| {
        crate::ui_kit::checkbox(ui, "Resting support", &mut f.resting);
    });
    form_row(ui, "", |ui| {
        crate::ui_kit::checkbox(ui, "Buckling", &mut f.buckling);
    });
    if f.buckling {
        form_row(ui, "", |ui| {
            crate::ui_kit::checkbox(ui, "Follow to collapse (slow)", &mut f.collapse);
        });
    }

    let (bodies, fixed, load) = picks(inputs);
    if bodies.len() > 1 {
        form_row(ui, "Between bodies", |ui| {
            crate::dialogs::combo(ui, "stress_contact", &["Bonded", "Sliding contact", "Contact with friction"], &mut f.contact);
        });
        if f.contact == 2 {
            form_row(ui, "Friction", |ui| {
                field(ui, &mut f.friction);
            });
            form_row(ui, "Preload (N)", |ui| {
                field(ui, &mut f.preload);
            });
        }
    }
    let ready = !bodies.is_empty() && fixed.is_some() && load.is_some();
    let contact_now = if bodies.len() > 1 { (f.contact, f.friction.clone(), f.preload.clone()) } else { (0, String::new(), String::new()) };
    let now = (all_picks(inputs), f.force.clone(), f.resolution.clone(), f.gravity, f.resting, f.buckling, f.buckling && f.collapse, contact_now);
    let stale = f.result.is_some() && f.of.as_ref() != Some(&now);
    form_row(ui, "", |ui| {
        let label = if stale { "Compute again" } else { "Compute" };
        let button = crate::ui_kit::button(ui, label, crate::ui_kit::Button::Enabled(ready));
        crate::scenario::publish_handle("stress:compute", button.rect.center());
        if button.clicked() {
            let (r, fld) = compute_on(app, f, &bodies, fixed, load);
            f.result = Some(r);
            f.field = fld;
            f.of = Some(now);
        }
    });
    if !ready {
        form_row(ui, "", |ui| {
            ui.label(
                RichText::new("pick the body (or the bodies that touch), the face it is held at and the face the load acts on").color(t.text_dim),
            );
        });
    }
    if let Some(r) = f.result.as_ref() {
        result_rows(app, ui, r, f.field.as_ref());
    }
    false
}

#[cfg(test)]
pub(crate) fn compute(
    app: &mut SolveApp,
    f: &StressForm,
    body: Option<&str>,
    fixed: Option<FacePick>,
    load: Option<FacePick>,
) -> (Value, Option<StressField>) {
    let Some(body) = body else { return (json!({ "error": "pick a body and two faces" }), None) };
    let on = |pick: Option<FacePick>| pick.map(|pick| FaceOn { pick, body: body.to_string() });
    compute_on(app, f, &[body.to_string()], on(fixed), on(load))
}

/// Runs `inspect.stress` on the bodies picked (several: on one grid, bonded or in contact as the
/// form says), held at `fixed`, loaded at `load`.
pub(crate) fn compute_on(
    app: &mut SolveApp,
    f: &StressForm,
    bodies: &[String],
    fixed: Option<FaceOn>,
    load: Option<FaceOn>,
) -> (Value, Option<StressField>) {
    let num = |s: &str| s.trim().replace(',', ".").parse::<f64>().ok().filter(|x| x.is_finite());
    let force: Option<Vec<f64>> = f.force.iter().map(|s| num(s)).collect();
    let Some(force) = force else { return (json!({ "error": "the force needs three numbers in newtons" }), None) };
    let Ok(resolution) = f.resolution.trim().parse::<u32>() else {
        return (json!({ "error": "the resolution is a whole number of elements, 6 to 60" }), None);
    };
    let (Some(first), Some(fixed), Some(load)) = (bodies.first(), fixed, load) else {
        return (json!({ "error": "pick a body and two faces" }), None);
    };
    let mut p = json!({
        "body": first,
        "fixed": {"face": fixed.pick.index, "point": fixed.pick.point, "body": fixed.body},
        "load": {"face": load.pick.index, "point": load.pick.point, "body": load.body},
        "force_n": force,
        "resolution": resolution,
        "gravity": f.gravity,
        "resting": f.resting,
        "buckling": f.buckling,
        "collapse": f.buckling && f.collapse,
        "field": true,
    });
    if bodies.len() > 1 {
        p["bodies"] = json!(bodies);
        match f.contact {
            1 => p["contact"] = json!({"friction": 0.0}),
            2 => {
                let (Some(friction), Some(preload)) = (num(&f.friction), num(&f.preload)) else {
                    return (json!({ "error": "the friction coefficient and the preload need numbers" }), None);
                };
                p["contact"] = json!({"friction": friction, "preload_n": preload});
            }
            _ => {}
        }
    }
    match app.run("inspect.stress", p) {
        Ok(v) => {
            let field = v.get("field").and_then(StressField::from_json);
            (v, field)
        }
        Err(e) => (json!({ "error": e }), None),
    }
}

/// A displacement (mm): in the design's units when it is big enough to show there, else in µm.
fn small_length(app: &SolveApp, mm: f64) -> String {
    if mm.abs() >= 0.01 { crate::prefs::show_mm(app, mm, 1) } else { format!("{:.1} µm", mm * 1000.0) }
}

fn result_rows(app: &SolveApp, ui: &mut egui::Ui, r: &Value, field: Option<&StressField>) {
    let t = Tokens::get();
    if let Some(e) = r.get("error").and_then(Value::as_str) {
        form_row(ui, "", |ui| {
            ui.label(RichText::new(e).color(t.error));
        });
        return;
    }
    let num = |k: &str| r.get(k).and_then(Value::as_f64);
    form_row(ui, "Material", |ui| {
        ui.label(format!("{} · E {:.1} GPa", r["material"].as_str().unwrap_or("?"), num("young_modulus_pa").unwrap_or(0.0) / 1e9));
    });
    form_row(ui, "Mesh", |ui| {
        let g = &r["grid"];
        ui.label(format!("{} × {} × {} cells of {}", g[0], g[1], g[2], crate::prefs::show_mm(app, num("element_mm").unwrap_or(0.0), 1)));
    });
    form_row(ui, "Displacement", |ui| {
        ui.label(format!("{} at most", small_length(app, num("max_displacement_mm").unwrap_or(0.0))));
    });
    form_row(ui, "Peak stress", |ui| {
        ui.label(RichText::new(format!("{:.1} MPa", num("max_von_mises_mpa").unwrap_or(0.0))).strong());
    });
    if let Some(c) = r.get("contact").filter(|c| c.is_object()) {
        form_row(ui, "Contact", |ui| {
            let (links, open, sliding) =
                (c["links"].as_u64().unwrap_or(0), c["open_links"].as_u64().unwrap_or(0), c["sliding_links"].as_u64().unwrap_or(0));
            ui.label(format!("{links} links, {open} opened, {sliding} sliding"));
        });
    }
    if let Some(sf) = num("safety_factor") {
        form_row(ui, "Safety factor", |ui| {
            let colour = if sf >= 2.0 {
                t.text
            } else if sf >= 1.0 {
                t.warning
            } else {
                t.error
            };
            ui.label(RichText::new(format!("{sf:.2} (yield {:.0} MPa)", num("yield_mpa").unwrap_or(0.0))).color(colour));
        });
    }
    if let Some(b) = r.get("buckling").filter(|b| b.is_object()) {
        let bf = b["load_factor"].as_f64().unwrap_or(0.0);
        form_row(ui, "Buckling", |ui| {
            if bf > 0.0 {
                let colour = if bf >= 2.0 {
                    t.text
                } else if bf >= 1.0 {
                    t.warning
                } else {
                    t.error
                };
                ui.label(
                    RichText::new(format!(
                        "folds at {bf:.2}× this load; bow grows ×{:.1}, +{:.1} MPa bending",
                        b["amplification"].as_f64().unwrap_or(0.0),
                        b["second_order_stress_mpa"].as_f64().unwrap_or(0.0)
                    ))
                    .color(colour),
                );
            } else {
                ui.label(RichText::new(b["advice"].as_str().unwrap_or("no buckling mode")).color(t.text_dim));
            }
        });
        let cf = b["collapse_factor"].as_f64().unwrap_or(0.0);
        if cf > 0.0 {
            form_row(ui, "Collapse", |ui| {
                let colour = if cf >= 1.5 { t.text } else { t.error };
                ui.label(
                    RichText::new(format!(
                        "a real member folds at {cf:.2}× this load ({} sideways)",
                        small_length(app, b["collapse_deflection_mm"].as_f64().unwrap_or(0.0))
                    ))
                    .color(colour),
                );
            });
        } else if let Some(advice) = b["collapse_advice"].as_str().filter(|a| !a.is_empty()) {
            form_row(ui, "Collapse", |ui| {
                ui.label(RichText::new(advice).color(t.text_dim));
            });
        }
    }
    if let Some(v) = r["verdict"].as_str() {
        form_row(ui, "Verdict", |ui| {
            ui.label(v);
        });
    }
    for w in r["warnings"].as_array().into_iter().flatten().filter_map(Value::as_str) {
        form_row(ui, "", |ui| {
            ui.label(RichText::new(w).color(t.warning));
        });
    }
    if let Some(fld) = field {
        form_row(ui, "Colours", |ui| {
            ui.horizontal(|ui| {
                let (rect, _) = ui.allocate_exact_size(egui::vec2(120.0, 10.0), egui::Sense::hover());
                let n = 24;
                for s in 0..n {
                    let x0 = rect.left() + rect.width() * s as f32 / n as f32;
                    let x1 = rect.left() + rect.width() * (s + 1) as f32 / n as f32;
                    ui.painter().rect_filled(
                        egui::Rect::from_min_max(Pos2::new(x0, rect.top()), Pos2::new(x1, rect.bottom())),
                        0.0,
                        ramp(s as f64 / (n - 1) as f64),
                    );
                }
                ui.label(format!("0 to {:.1} MPa", fld.max_mpa));
            });
        });
    }
}

/// The material cells over the model, each exposed cell face coloured by the cell's stress.
/// Faces turned towards the camera only, so the near surface of a convex part reads right.
pub fn overlay(painter: &egui::Painter, proj: &Proj, f: &StressForm) {
    let Some(fld) = f.field.as_ref() else { return };
    paint_cells(painter, proj, fld, 0.0, fld.max_mpa);
}

/// Paints every exposed cell face, coloured by where the cell's value sits between `lo` and `hi`.
pub fn paint_cells(painter: &egui::Painter, proj: &Proj, fld: &StressField, lo: f64, hi: f64) {
    if !(hi > lo) {
        return;
    }
    let back = proj.cam.back();
    let h = fld.element_mm;
    let [nx, ny, nz] = fld.grid;
    let mut em = egui::Mesh::default();
    // The six faces of a cell: the outward normal and the four corners (unit cube, by axis).
    let faces: [(Vec3, [Vec3; 4]); 6] = [
        (Vec3::new(-1.0, 0.0, 0.0), [Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 1.0, 0.0)]),
        (Vec3::new(1.0, 0.0, 0.0), [Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 0.0, 1.0)]),
        (Vec3::new(0.0, -1.0, 0.0), [Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 1.0)]),
        (Vec3::new(0.0, 1.0, 0.0), [Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 0.0)]),
        (Vec3::new(0.0, 0.0, -1.0), [Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)]),
        (Vec3::new(0.0, 0.0, 1.0), [Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 1.0), Vec3::new(1.0, 1.0, 1.0), Vec3::new(0.0, 1.0, 1.0)]),
    ];
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let Some(Some(stress)) = fld.cells.get(fld.index(i, j, k)).copied() else { continue };
                let colour = ramp((stress - lo) / (hi - lo));
                let corner = fld.origin_mm + Vec3::new(i as f64, j as f64, k as f64) * h;
                for (normal, quad) in &faces {
                    let facing = normal.dot(back);
                    if facing <= 0.0 {
                        continue;
                    }
                    let (ni, nj, nk) = (i as isize + normal.x as isize, j as isize + normal.y as isize, k as isize + normal.z as isize);
                    if fld.filled(ni, nj, nk) {
                        continue;
                    }
                    let shade = 0.6 + 0.4 * facing;
                    let mut fill = colour.gamma_multiply(shade as f32);
                    fill[3] = 230;
                    let sp: Vec<Pos2> = quad.iter().filter_map(|c| proj.to_screen(corner + *c * h)).collect();
                    if sp.len() == 4 {
                        let base = em.vertices.len() as u32;
                        for p in sp {
                            em.colored_vertex(p, fill);
                        }
                        em.add_triangle(base, base + 1, base + 2);
                        em.add_triangle(base, base + 2, base + 3);
                    }
                }
            }
        }
    }
    painter.add(egui::Shape::mesh(em));
}
