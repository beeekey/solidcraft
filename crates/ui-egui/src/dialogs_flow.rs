//! Flow Analysis: pick a body and a face at each end of the flow, choose the fluid and its speed,
//! and Compute runs `inspect.flow`: the pressure drop, the flow rate, the drag and, with heat on,
//! what the fluid carries away.

use egui::RichText;
use serde_json::{Value, json};
use solidcraft_engine::Sel;
use solidcraft_engine::geom::Vec3;

use crate::SolveApp;
use crate::dialogs::{combo, field, form_row};
use crate::selection::SelInput;
use crate::theme::Tokens;

pub const FLUIDS: [&str; 4] = ["Water", "Air", "Oil (ISO VG 46)", "Ethylene glycol 50%"];

#[derive(Clone, Debug)]
pub struct FlowForm {
    /// 0 = through the part (internal), 1 = around it (external).
    pub region: usize,
    pub fluid: usize,
    pub velocity: String,
    pub resolution: String,
    pub heat: bool,
    pub watts: String,
    pub result: Option<Value>,
    pub of: Option<(Vec<Sel>, usize, usize, String, String, bool, String)>,
}

impl Default for FlowForm {
    fn default() -> Self {
        FlowForm { region: 0, fluid: 0, velocity: "0.5".into(), resolution: "40".into(), heat: false, watts: "10".into(), result: None, of: None }
    }
}

fn point(inputs: &[SelInput], n: usize) -> Option<Vec3> {
    inputs.get(n).and_then(|i| i.items.first()).and_then(|s| match s {
        Sel::Face { point, .. } | Sel::Vertex { point, .. } => Some(*point),
        _ => None,
    })
}

fn body_of(inputs: &[SelInput]) -> Option<String> {
    inputs.iter().flat_map(|i| i.items.iter()).find_map(|s| match s {
        Sel::Body { name } => Some(name.clone()),
        Sel::Face { body, .. } | Sel::Vertex { body, .. } => Some(body.clone()),
        _ => None,
    })
}

/// The dialog's rows. Returns false: Compute applies, not Enter.
pub fn rows(app: &mut SolveApp, ui: &mut egui::Ui, f: &mut FlowForm, inputs: &[SelInput]) -> bool {
    let t = Tokens::get();
    form_row(ui, "Flow", |ui| {
        combo(ui, "flow_region", &["Through the part", "Around the part"], &mut f.region);
    });
    form_row(ui, "Fluid", |ui| {
        combo(ui, "flow_fluid", &FLUIDS, &mut f.fluid);
    });
    form_row(ui, "Speed (m/s)", |ui| {
        field(ui, &mut f.velocity);
    });
    form_row(ui, "Resolution", |ui| {
        field(ui, &mut f.resolution);
    });
    form_row(ui, "", |ui| {
        crate::ui_kit::checkbox(ui, "Heat from the part", &mut f.heat);
    });
    if f.heat {
        form_row(ui, "Power (W)", |ui| {
            field(ui, &mut f.watts);
        });
    }
    let body = body_of(inputs);
    let (inlet, outlet) = (point(inputs, 1), point(inputs, 2));
    let ready = body.is_some() && inlet.is_some() && outlet.is_some();
    let picks: Vec<Sel> = inputs.iter().flat_map(|i| i.items.iter().cloned()).collect();
    let now = (picks, f.region, f.fluid, f.velocity.clone(), f.resolution.clone(), f.heat, f.watts.clone());
    let stale = f.result.is_some() && f.of.as_ref() != Some(&now);
    form_row(ui, "", |ui| {
        let button = crate::ui_kit::button(ui, if stale { "Compute again" } else { "Compute" }, crate::ui_kit::Button::Enabled(ready));
        crate::scenario::publish_handle("flow:compute", button.rect.center());
        if button.clicked() {
            f.result = Some(compute(app, f, body.as_deref(), inlet, outlet));
            f.of = Some(now);
        }
    });
    if !ready {
        form_row(ui, "", |ui| {
            ui.label(RichText::new("pick the body, then a face where the fluid comes in and one where it leaves").color(t.text_dim));
        });
    }
    if let Some(r) = f.result.as_ref() {
        result_rows(ui, r);
    }
    false
}

pub(crate) fn compute(app: &mut SolveApp, f: &FlowForm, body: Option<&str>, inlet: Option<Vec3>, outlet: Option<Vec3>) -> Value {
    let num = |s: &str| s.trim().replace(',', ".").parse::<f64>().ok().filter(|x| x.is_finite());
    let (Some(body), Some(inlet), Some(outlet)) = (body, inlet, outlet) else { return json!({ "error": "pick a body and the two faces" }) };
    let Some(velocity) = num(&f.velocity) else { return json!({ "error": "the speed needs a number of m/s" }) };
    let Ok(resolution) = f.resolution.trim().parse::<u32>() else { return json!({ "error": "the resolution is a whole number of cells, 8 to 160" }) };
    let mut p = json!({
        "body": body,
        "region": if f.region == 0 { "internal" } else { "external" },
        "inlet": inlet,
        "outlet": outlet,
        "velocity_m_s": velocity,
        "fluid": FLUIDS.get(f.fluid).copied().unwrap_or("Water"),
        "resolution": resolution,
    });
    if f.heat {
        p["heat"] = json!(true);
        if let Some(w) = num(&f.watts) {
            p["watts"] = json!(w);
        }
    }
    match app.run("inspect.flow", p) {
        Ok(v) => v,
        Err(e) => json!({ "error": e }),
    }
}

fn result_rows(ui: &mut egui::Ui, r: &Value) {
    let t = Tokens::get();
    if let Some(e) = r.get("error").and_then(Value::as_str) {
        form_row(ui, "", |ui| {
            ui.label(RichText::new(e).color(t.error));
        });
        return;
    }
    let num = |k: &str| r.get(k).and_then(Value::as_f64).unwrap_or(0.0);
    form_row(ui, "Reynolds", |ui| {
        let re = num("reynolds");
        ui.label(RichText::new(format!("{re:.0}{}", if re > 2300.0 { " (past laminar)" } else { "" })).color(if re > 2300.0 {
            t.warning
        } else {
            t.text
        }));
    });
    form_row(ui, "Pressure drop", |ui| {
        ui.label(RichText::new(format!("{:.2} Pa", num("pressure_drop_pa"))).strong());
    });
    form_row(ui, "Flow rate", |ui| {
        ui.label(format!(
            "{:.3} L/min at {:.3} m/s, peak {:.3} m/s",
            num("volume_flow_l_min"),
            num("mean_inlet_velocity_m_s"),
            num("max_velocity_m_s")
        ));
    });
    if let Some(d) = r.get("drag_force_n").and_then(|d| d.as_array()) {
        let f: Vec<f64> = d.iter().filter_map(Value::as_f64).collect();
        if f.len() == 3 {
            form_row(ui, "Drag", |ui| {
                ui.label(format!("{:.4} N ({:.4}, {:.4}, {:.4})", (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt(), f[0], f[1], f[2]));
            });
        }
    }
    if r["heat_solved"].as_bool().unwrap_or(false) {
        form_row(ui, "Heat", |ui| {
            ui.label(format!(
                "fluid leaves at {:.1} °C carrying {:.1} W; the part peaks at {:.1} °C",
                num("outlet_c"),
                num("heat_removed_w"),
                num("max_c")
            ));
        });
    }
    form_row(ui, "Solve", |ui| {
        ui.label(format!(
            "{} steps, {}{} cells across the passage",
            num("steps") as u64,
            if r["converged"].as_bool().unwrap_or(false) { "settled, " } else { "not settled, " },
            num("cells_across_passage") as u64
        ));
    });
    for w in r["warnings"].as_array().into_iter().flatten().filter_map(Value::as_str) {
        form_row(ui, "", |ui| {
            ui.label(RichText::new(w).color(t.warning));
        });
    }
}
