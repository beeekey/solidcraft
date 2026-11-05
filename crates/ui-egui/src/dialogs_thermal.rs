//! Thermal Analysis: pick a body, the face the heat goes in over and, if there is one, the face
//! held cold, type the watts, and Compute runs `inspect.thermal`. The answer is the dialog's
//! rows and, over the model, the cells coloured from the air's temperature to the peak.

use egui::{Pos2, RichText};
use serde_json::{Value, json};
use solidcraft_engine::Sel;

use crate::SolveApp;
use crate::dialogs::{field, form_row};
use crate::dialogs_stress::{FacePick, StressField, paint_cells, ramp};
use crate::selection::SelInput;
use crate::theme::Tokens;
use crate::viewport::Proj;

#[derive(Clone, Debug)]
pub struct ThermalForm {
    pub watts: String,
    pub held_c: String,
    pub ambient_c: String,
    pub emissivity: String,
    pub resolution: String,
    pub result: Option<Value>,
    pub of: Option<(Vec<Sel>, [String; 5])>,
    pub field: Option<StressField>,
}

impl Default for ThermalForm {
    fn default() -> Self {
        ThermalForm {
            watts: "5".into(),
            held_c: "20".into(),
            ambient_c: "20".into(),
            emissivity: String::new(),
            resolution: "32".into(),
            result: None,
            of: None,
            field: None,
        }
    }
}

fn pick(inputs: &[SelInput], n: usize) -> Option<FacePick> {
    inputs.get(n).and_then(|i| i.items.first()).and_then(|s| match s {
        Sel::Face { index, point, .. } => Some(FacePick { index: *index, point: *point }),
        _ => None,
    })
}

fn body_of(inputs: &[SelInput]) -> Option<String> {
    let named = inputs.first().and_then(|i| i.items.first()).and_then(|s| match s {
        Sel::Body { name } => Some(name.clone()),
        Sel::Face { body, .. } => Some(body.clone()),
        _ => None,
    });
    named.or_else(|| {
        inputs.get(1).and_then(|i| i.items.first()).and_then(|s| match s {
            Sel::Face { body, .. } => Some(body.clone()),
            _ => None,
        })
    })
}

/// The dialog's rows. Returns false: Compute applies, not Enter.
pub fn rows(app: &mut SolveApp, ui: &mut egui::Ui, f: &mut ThermalForm, inputs: &[SelInput]) -> bool {
    let t = Tokens::get();
    form_row(ui, "Power (W)", |ui| {
        field(ui, &mut f.watts);
    });
    form_row(ui, "Held at (°C)", |ui| {
        field(ui, &mut f.held_c);
    });
    form_row(ui, "Room (°C)", |ui| {
        field(ui, &mut f.ambient_c);
    });
    form_row(ui, "Emissivity", |ui| {
        field(ui, &mut f.emissivity);
    });
    form_row(ui, "Resolution", |ui| {
        field(ui, &mut f.resolution);
    });
    let body = body_of(inputs);
    let (heat, held) = (pick(inputs, 1), pick(inputs, 2));
    let ready = body.is_some() && heat.is_some();
    let picks: Vec<Sel> = inputs.iter().flat_map(|i| i.items.iter().cloned()).collect();
    let now = (picks, [f.watts.clone(), f.held_c.clone(), f.ambient_c.clone(), f.emissivity.clone(), f.resolution.clone()]);
    let stale = f.result.is_some() && f.of.as_ref() != Some(&now);
    form_row(ui, "", |ui| {
        let button = crate::ui_kit::button(ui, if stale { "Compute again" } else { "Compute" }, crate::ui_kit::Button::Enabled(ready));
        crate::scenario::publish_handle("thermal:compute", button.rect.center());
        if button.clicked() {
            let (r, fld) = compute(app, f, body.as_deref(), heat, held);
            f.result = Some(r);
            f.field = fld;
            f.of = Some(now);
        }
    });
    if !ready {
        form_row(ui, "", |ui| {
            ui.label(RichText::new("pick the body and the face the heat goes in over; a held face is optional").color(t.text_dim));
        });
    }
    if let Some(r) = f.result.as_ref() {
        result_rows(ui, r, f.field.as_ref());
    }
    false
}

pub(crate) fn compute(
    app: &mut SolveApp,
    f: &ThermalForm,
    body: Option<&str>,
    heat: Option<FacePick>,
    held: Option<FacePick>,
) -> (Value, Option<StressField>) {
    let num = |s: &str| s.trim().replace(',', ".").parse::<f64>().ok().filter(|x| x.is_finite());
    let (Some(body), Some(heat)) = (body, heat) else { return (json!({ "error": "pick a body and the heated face" }), None) };
    let Some(watts) = num(&f.watts) else { return (json!({ "error": "the power needs a number of watts" }), None) };
    let Ok(resolution) = f.resolution.trim().parse::<u32>() else {
        return (json!({ "error": "the resolution is a whole number of cells, 8 to 120" }), None);
    };
    let mut p = json!({
        "body": body,
        "heat": {"face": heat.index, "point": heat.point},
        "watts": watts,
        "ambient_c": num(&f.ambient_c).unwrap_or(20.0),
        "resolution": resolution,
        "field": true,
    });
    if let Some(held) = held {
        p["held"] = json!({"face": held.index, "point": held.point});
        p["held_c"] = json!(num(&f.held_c).unwrap_or(20.0));
    }
    if let Some(e) = num(&f.emissivity) {
        p["emissivity"] = json!(e);
    }
    match app.run("inspect.thermal", p) {
        Ok(v) => {
            let field = v.get("field").and_then(StressField::from_json);
            (v, field)
        }
        Err(e) => (json!({ "error": e }), None),
    }
}

fn result_rows(ui: &mut egui::Ui, r: &Value, field: Option<&StressField>) {
    let t = Tokens::get();
    if let Some(e) = r.get("error").and_then(Value::as_str) {
        form_row(ui, "", |ui| {
            ui.label(RichText::new(e).color(t.error));
        });
        return;
    }
    let num = |k: &str| r.get(k).and_then(Value::as_f64).unwrap_or(0.0);
    form_row(ui, "Material", |ui| {
        ui.label(format!("{} · {:.0} W/m·K · ε {:.2}", r["material"].as_str().unwrap_or("?"), num("conductivity_w_mk"), num("emissivity")));
    });
    form_row(ui, "Peak", |ui| {
        let rise = num("peak_c") - num("air_c");
        let colour = if rise < 40.0 {
            t.text
        } else if rise < 80.0 {
            t.warning
        } else {
            t.error
        };
        ui.label(RichText::new(format!("{:.1} °C ({rise:.1} K above the air)", num("peak_c"))).strong().color(colour));
    });
    form_row(ui, "Surface", |ui| {
        ui.label(format!("{:.1} °C mean, film {:.1} W/m²K over {:.0} cm²", num("mean_surface_c"), num("film_w_m2k"), num("exposed_area_cm2")));
    });
    form_row(ui, "Heat path", |ui| {
        ui.label(format!(
            "{:.1} W convected, {:.1} W radiated, {:.1} W into the held face",
            num("convected_w"),
            num("radiated_w"),
            num("held_w").max(0.0)
        ));
    });
    if num("channelled_fraction") > 0.0 {
        form_row(ui, "Fins", |ui| {
            ui.label(format!(
                "{:.0}% of the surface sits in channels, the narrowest {:.1} mm",
                100.0 * num("channelled_fraction"),
                num("narrowest_gap_mm")
            ));
        });
    }
    if num("enclosure_rise_k") > 0.0 {
        form_row(ui, "Box", |ui| {
            ui.label(format!("its air runs {:.1} K above the room", num("enclosure_rise_k")));
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
                ui.label(format!("{:.1} to {:.1} °C", fld.min, fld.max_mpa));
            });
        });
    }
}

pub fn overlay(painter: &egui::Painter, proj: &Proj, f: &ThermalForm) {
    let Some(fld) = f.field.as_ref() else { return };
    let air = f.result.as_ref().and_then(|r| r["air_c"].as_f64()).unwrap_or(fld.min);
    paint_cells(painter, proj, fld, air.min(fld.min), fld.max_mpa);
}
