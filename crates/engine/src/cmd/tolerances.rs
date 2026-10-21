//! Fits, general tolerances, tolerance stack-up and the fastener for a hole: the numbers a
//! drawing calls up, from the standards rather than from memory ([`solidcraft_analysis::fits`],
//! [`solidcraft_analysis::general_tolerances`], [`solidcraft_analysis::stack_up`],
//! [`solidcraft_analysis::hole_stack`]). These commands take numbers, not geometry, and change
//! nothing in the design.

use serde_json::{Value, json};
use solidcraft_analysis::fits::{FitDuty, check_fit, fit_limits, known_fit_classes};
use solidcraft_analysis::general_tolerances::{GeneralClass, general_angular, general_linear, general_radius_or_chamfer};
use solidcraft_analysis::hole_stack::{choose_fastener, find_borehole, tapping_drill};
use solidcraft_analysis::stack_up::{Contributor, StackRequirement, stack_up};

use super::CommandSpec;
use crate::params::{bad, bool_, str_};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("inspect.fit", "Fit", fit).at("3DEEE", "CALCULATE").icon("shaft_fit").noundo().params(
        "nominal_mm: the size (0 to 500); hole: a hole class (\"H7\"); shaft: a shaft class (\"g6\"), or instead hole_min_mm, hole_max_mm, shaft_min_mm, \
         shaft_max_mm: measured sizes; engagement_length_mm?, hub_outer_diameter_mm?, shaft_bore_diameter_mm? (0 = solid), hub_material?, shaft_material? \
         (Steel, Stainless Steel, Aluminum, Brass, Bronze, Copper, Titanium, Cast Iron; default Steel), friction? (default 0.15), smoothing_mm? (roughness lost \
         on first assembly) → kind (clearance | transition | interference), hole, shaft: {nominal_mm, lower_mm, upper_mm, min_mm, max_mm, source}, \
         min_clearance_mm, max_clearance_mm (negative = interference), min_interference_mm, max_interference_mm; for an interference fit with the geometry \
         given: contact_pressure_mpa, assembly_force_n (at the tightest), min_retained_torque_nm (at the loosest), max_retained_torque_nm, hub_hoop_stress_mpa, \
         hub_safety, shrink_temperature_rise_k; verdict, advice, warnings. ISO 286 limits; the force and torque are different fits and are reported apart",
    ),
    CommandSpec::new("inspect.general_tolerance", "General Tolerance", general_tolerance).at("3DEEE", "CALCULATE").icon("general_tolerance").noundo().params(
        "class: \"f\" | \"m\" | \"c\" | \"v\" or a drawing note (\"ISO 2768-mK\"); nominal_mm: the dimension, or for an angle the length of its shorter side; \
         kind?: \"linear\" (default) | \"radius\" (external radii and chamfer heights) | \"angle\" → plus_minus_mm (or plus_minus_deg), lower_mm, upper_mm, \
         source, verdict, warnings. ISO 2768-1 tables 1, 2 and 3; where the standard prints a dash the dimension must be toleranced individually and the \
         command says so",
    ),
    CommandSpec::new("inspect.tolerance_stack", "Tolerance Stack", tolerance_stack).at("3DEEE", "CALCULATE").icon("tolerance_stack").noundo().params(
        "chain: [{name?, nominal_mm, plus_minus_mm | lower_mm and upper_mm, sensitivity?: +1 adds, -1 subtracts (default 1)}] (1 to 10000); minimum_mm?, \
         maximum_mm?: what the gap has to be → nominal_mm, mean_mm, worst_case: {min_mm, max_mm, half_width_mm}, rss: {min_mm, max_mm, half_width_mm}, \
         ranked: [{name, half_width_mm, worst_case_share, variance_share}] largest variance first, worst_case_passes?, rss_passes?, sigma_at_limit?, \
         verdict, advice, warnings. Worst case promises; RSS assumes independent, normally distributed parts, and the advice says so",
    ),
    CommandSpec::new("inspect.hole_stack", "Hole Stack", hole_stack).at("3DEEE", "CALCULATE").icon("hole_stack").noundo().params(
        "hole_diameter_mm: the clearance hole as drilled; grip_mm: the material under the head; washer?: true (default) | false; nut?: true (default) | \
         false → thread (\"M6\"), series (Fine | Medium | Coarse), series_diameter_mm, screw: {size, length_mm (the next listed ISO 4762 length), needed_mm, \
         washer_thickness_mm, nut_height_mm, pitch_mm, description}, drill: {tapping_mm, clearance_fine_mm, clearance_medium_mm, clearance_coarse_mm, \
         counterbore_diameter_mm, counterbore_depth_mm, countersink_diameter_mm?, socket_head_diameter_mm, socket_head_height_mm}, verdict, warnings. \
         The hole is matched back to its thread through ISO 273 within 0.03 mm; a hole drilled for nothing is refused by name",
    ),
];

/// Largest length (mm) any of these commands takes.
const MAX_MM: f64 = 1e6;

/// A finite number in `range`, or `default` when absent.
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

fn required(cmd: &str, p: &Value, key: &str, range: std::ops::RangeInclusive<f64>) -> Result<f64> {
    number(cmd, p, key, None, range)?.ok_or_else(|| bad(cmd, format!("give `{key}`")))
}

fn text(cmd: &str, p: &Value, key: &str) -> Result<Option<String>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) if s.len() <= 64 => Ok(Some(s.clone())),
        Some(_) => Err(bad(cmd, format!("`{key}` must be a short string"))),
    }
}

fn limits_json(l: &solidcraft_analysis::fits::Limits, source: &str) -> Value {
    json!({
        "nominal_mm": l.nominal_mm,
        "lower_mm": l.lower_mm,
        "upper_mm": l.upper_mm,
        "min_mm": l.min_size(),
        "max_mm": l.max_size(),
        "source": source,
    })
}

fn fit(_s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.fit";
    let hole_class = text(cmd, p, "hole")?.unwrap_or_default();
    let shaft_class = text(cmd, p, "shaft")?.unwrap_or_default();
    let size = |key: &str| number(cmd, p, key, Some(0.0), 0.0..=MAX_MM).map(|v| v.unwrap_or(0.0));
    let duty = FitDuty {
        nominal_mm: size("nominal_mm")?,
        hole_class: hole_class.clone(),
        shaft_class: shaft_class.clone(),
        hole_min_mm: size("hole_min_mm")?,
        hole_max_mm: size("hole_max_mm")?,
        shaft_min_mm: size("shaft_min_mm")?,
        shaft_max_mm: size("shaft_max_mm")?,
        engagement_length_mm: size("engagement_length_mm")?,
        hub_outer_diameter_mm: size("hub_outer_diameter_mm")?,
        shaft_bore_diameter_mm: size("shaft_bore_diameter_mm")?,
        hub_material: text(cmd, p, "hub_material")?.unwrap_or_else(|| "Steel".into()),
        shaft_material: text(cmd, p, "shaft_material")?.unwrap_or_else(|| "Steel".into()),
        friction: number(cmd, p, "friction", Some(0.15), 0.0..=2.0)?.unwrap_or(0.15),
        smoothing_mm: size("smoothing_mm")?,
    };
    let r = check_fit(&duty).map_err(|e| bad(cmd, e))?;
    let source = |class: &str| {
        if class.is_empty() { "measured".to_string() } else { fit_limits(duty.nominal_mm, class).map(|l| l.source).unwrap_or_default() }
    };
    let mut warnings: Vec<String> = Vec::new();
    if r.kind != solidcraft_analysis::fits::FitKind::Clearance && r.assembly_force_n == 0.0 {
        warnings.push(r.advice.clone());
    }
    if r.hub_safety > 0.0 && r.hub_safety < 1.0 {
        warnings.push(format!("the hub yields: hoop stress {:.0} MPa, safety factor {:.2}", r.hub_hoop_stress_pa * 1e-6, r.hub_safety));
    }
    Ok(json!({
        "kind": r.kind,
        "hole": limits_json(&r.hole, &source(&hole_class)),
        "shaft": limits_json(&r.shaft, &source(&shaft_class)),
        "min_clearance_mm": r.min_clearance_mm,
        "max_clearance_mm": r.max_clearance_mm,
        "min_interference_mm": r.min_interference_mm,
        "max_interference_mm": r.max_interference_mm,
        "contact_pressure_mpa": r.contact_pressure_pa * 1e-6,
        "assembly_force_n": r.assembly_force_n,
        "min_retained_torque_nm": r.min_retained_torque_nm,
        "max_retained_torque_nm": r.max_retained_torque_nm,
        "hub_hoop_stress_mpa": r.hub_hoop_stress_pa * 1e-6,
        "hub_safety": r.hub_safety,
        "shrink_temperature_rise_k": r.shrink_temperature_rise_k,
        "verdict": r.verdict,
        "advice": r.advice,
        "known_classes": known_fit_classes().len(),
        "warnings": warnings,
    }))
}

fn general_tolerance(_s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.general_tolerance";
    let class_text = text(cmd, p, "class")?.ok_or_else(|| bad(cmd, "give `class`: f, m, c, v or a drawing note such as \"ISO 2768-m\""))?;
    let class = GeneralClass::parse(&class_text).ok_or_else(|| bad(cmd, format!("`{class_text}` is not an ISO 2768 class: f, m, c or v")))?;
    let nominal = required(cmd, p, "nominal_mm", 0.0..=MAX_MM)?;
    let kind = str_(p, "kind").unwrap_or("linear");
    let (found, unit) = match kind {
        "linear" => (general_linear(nominal, class), "mm"),
        "radius" | "chamfer" => (general_radius_or_chamfer(nominal, class), "mm"),
        "angle" | "angular" => (general_angular(nominal, class), "deg"),
        other => return Err(bad(cmd, format!("`kind` is linear, radius or angle, not `{other}`"))),
    };
    let t = found.map_err(|e| bad(cmd, e))?;
    let mut out = json!({
        "class": class.letter(),
        "kind": kind,
        "nominal_mm": nominal,
        "source": t.source,
        "verdict": format!("{nominal} {} ± {} {unit} ({})", if unit == "deg" { "mm shorter side" } else { "mm" }, t.plus_minus, class.letter()),
        "warnings": [],
    });
    if unit == "deg" {
        out["plus_minus_deg"] = json!(t.plus_minus);
    } else {
        out["plus_minus_mm"] = json!(t.plus_minus);
        out["lower_mm"] = json!(nominal - t.plus_minus);
        out["upper_mm"] = json!(nominal + t.plus_minus);
    }
    Ok(out)
}

fn tolerance_stack(_s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.tolerance_stack";
    let items = p
        .get("chain")
        .and_then(Value::as_array)
        .ok_or_else(|| bad(cmd, "give `chain`: a list of {name, nominal_mm, plus_minus_mm | lower_mm, upper_mm, sensitivity}"))?;
    if items.is_empty() || items.len() > solidcraft_analysis::stack_up::MAX_CHAIN {
        return Err(bad(cmd, format!("`chain` holds 1 to {} contributors", solidcraft_analysis::stack_up::MAX_CHAIN)));
    }
    let mut chain = Vec::with_capacity(items.len());
    for (i, item) in items.iter().enumerate() {
        if !item.is_object() {
            return Err(bad(cmd, format!("chain[{i}] must be an object")));
        }
        let name = text(cmd, item, "name")?.unwrap_or_else(|| format!("item {}", i + 1));
        let nominal = required(cmd, item, "nominal_mm", -MAX_MM..=MAX_MM)?;
        let (lower, upper) = match number(cmd, item, "plus_minus_mm", None, 0.0..=MAX_MM)? {
            Some(pm) => (-pm, pm),
            None => (required(cmd, item, "lower_mm", -MAX_MM..=MAX_MM)?, required(cmd, item, "upper_mm", -MAX_MM..=MAX_MM)?),
        };
        let sensitivity = number(cmd, item, "sensitivity", Some(1.0), -100.0..=100.0)?.unwrap_or(1.0);
        chain.push(Contributor { name, nominal_mm: nominal, lower_mm: lower, upper_mm: upper, sensitivity });
    }
    let want = match (number(cmd, p, "minimum_mm", None, -MAX_MM..=MAX_MM)?, number(cmd, p, "maximum_mm", None, -MAX_MM..=MAX_MM)?) {
        (Some(minimum_mm), Some(maximum_mm)) => Some(StackRequirement { minimum_mm, maximum_mm }),
        (None, None) => None,
        _ => return Err(bad(cmd, "give both `minimum_mm` and `maximum_mm`, or neither")),
    };
    let r = stack_up(&chain, want).map_err(|e| bad(cmd, e))?;
    let mut warnings: Vec<String> = Vec::new();
    if r.worst_case_passes == Some(false) {
        warnings.push("the worst case does not fit the requirement".into());
    }
    Ok(json!({
        "nominal_mm": r.nominal_mm,
        "mean_mm": r.mean_mm,
        "worst_case": {"min_mm": r.worst_case_min_mm, "max_mm": r.worst_case_max_mm, "half_width_mm": r.worst_case_half_width_mm},
        "rss": {"min_mm": r.rss_min_mm, "max_mm": r.rss_max_mm, "half_width_mm": r.rss_half_width_mm},
        "ranked": r.ranked,
        "worst_case_passes": r.worst_case_passes,
        "rss_passes": r.rss_passes,
        "sigma_at_limit": r.sigma_at_limit,
        "verdict": r.verdict,
        "advice": r.advice,
        "warnings": warnings,
    }))
}

fn hole_stack(_s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.hole_stack";
    let hole = required(cmd, p, "hole_diameter_mm", 0.0..=1000.0)?;
    let grip = required(cmd, p, "grip_mm", 0.0..=10_000.0)?;
    let washer = bool_(p, "washer").unwrap_or(true);
    let nut = bool_(p, "nut").unwrap_or(true);
    let (matched, screw) = choose_fastener(hole, grip, washer, nut).map_err(|e| bad(cmd, e))?;
    let drill = find_borehole(&matched.thread).map(|b| {
        json!({
            "tapping_mm": tapping_drill(b),
            "clearance_fine_mm": b.clearance_fine,
            "clearance_medium_mm": b.clearance_medium,
            "clearance_coarse_mm": b.clearance_coarse,
            "counterbore_diameter_mm": b.counterbore_diameter,
            "counterbore_depth_mm": b.counterbore_depth,
            "countersink_diameter_mm": b.countersink_diameter,
            "socket_head_diameter_mm": b.socket_head_diameter,
            "socket_head_height_mm": b.socket_head_height,
        })
    });
    let mut warnings: Vec<String> = Vec::new();
    if !nut {
        warnings.push("no nut: the screw ends flush with the far face, which only holds if that face is tapped".into());
    }
    Ok(json!({
        "thread": matched.thread,
        "series": matched.series,
        "series_diameter_mm": matched.series_diameter_mm,
        "screw": {
            "size": screw.size,
            "length_mm": screw.length_mm,
            "needed_mm": screw.needed_mm,
            "washer_thickness_mm": screw.washer_thickness_mm,
            "nut_height_mm": screw.nut_height_mm,
            "pitch_mm": screw.pitch_mm,
            "washer": screw.washer,
            "nut": screw.nut,
            "description": screw.description,
        },
        "drill": drill,
        "verdict": format!("ISO 4762 {} x {:.0} through a {} {} clearance hole", screw.size, screw.length_mm, matched.series.to_lowercase(), matched.thread),
        "warnings": warnings,
    }))
}

#[cfg(test)]
#[path = "tolerances_tests.rs"]
mod tests;
