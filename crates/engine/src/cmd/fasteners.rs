//! Bolted joints, thread engagement, bearing life and damping: the checks that take numbers
//! rather than geometry and answer with a verdict ([`solidcraft_analysis`]).

use serde_json::{Value, json};
use solidcraft_analysis::bamboo_joint::{BambooJoint, check_bamboo_joint, engagement_for};
use solidcraft_analysis::bearing::{BearingDuty, bearing_life, find_bearing};
use solidcraft_analysis::bolt::{BoltedJoint, check_bolted_joint};
use solidcraft_analysis::damping::{SuspensionDuty, check_suspension, loss_factor_of};
use solidcraft_analysis::thread_engagement::{Material, Severity, check_engagement};

use super::CommandSpec;
use crate::params::{bad, bool_, str_};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("inspect.bolted_joint", "Bolted Joint", bolted_joint).at("3DEEE", "CALCULATE").icon("bolted_joint").noundo().params(
        "size: \"M3\"…\"M36\" (ISO coarse); grade?: \"4.6\" | \"5.8\" | \"8.8\" (default) | \"10.9\" | \"12.9\" | \"A2-70\"; clamped_mm: the grip, the \
         thickness of the parts clamped (0.1 to 10000); preload_n?: N (default 75 % of proof); external_n?: N pulling the joint apart, per bolt; \
         shear_n?: N sideways, carried by friction; friction?: between the clamped faces (default 0.15); nut_factor?: K in T = K F d (default 0.20 dry, \
         0.15 oiled); cycles?: how often the external load is applied, for the fatigue check; bolt_modulus_pa?, clamped_modulus_pa? (default 210e9) → \
         stress_area_mm2, bolt_stiffness_n_m, clamped_stiffness_n_m, load_factor, preload_n, bolt_load_n, residual_clamp_n, bolt_stress_mpa, \
         safety_against_proof, separation_load_n, separates, slip_load_n, slips, tightening_torque_nm, alternating_stress_mpa, endurance_limit_mpa, \
         fatigue_safety, verdict, advice, warnings. Two springs in parallel (VDI 2230); not modelled: embedding, gasket creep, eccentric loading",
    ),
    CommandSpec::new("inspect.thread_engagement", "Thread Engagement", thread_engagement).at("3DEEE", "CALCULATE").icon("thread_engagement").noundo().params(
        "size: \"M<n>\"; material: \"steel\" | \"aluminium\" | \"plastic\"; engagement_mm: the tapped depth (0 to 10000); through?: true when the \
         hole goes through the part → diameter_mm, needed_mm (1.5 D steel, 2 D aluminium, 2.5 D plastic), engagement_mm, factor, severity (ok | note \
         | warning), verdict, warnings",
    ),
    CommandSpec::new("inspect.bearing_life", "Bearing Life", bearing_life_cmd).at("3DEEE", "CALCULATE").icon("bearing_life").noundo().params(
        "designation?: \"6001\", \"608\", \"6805\", \"Headset 41.8\"… (ratings from the table) or dynamic_n, static_n?: C and C0 from the catalogue; \
         radial_n, axial_n?: the load (N, up to 1e9); rpm?: speed, for hours; wheel_mm?: wheel diameter, for kilometres; roller?: true for a roller \
         bearing (exponent 10/3) → equivalent_load_n, load_ratio, million_revolutions (L10), hours, kilometres, static_safety, verdict, advice, \
         warnings. ISO 281; a bearing sold without ratings gets no life and says why",
    ),
    CommandSpec::new("inspect.damping", "Damping and Isolation", damping).at("3DEEE", "CALCULATE").icon("damping").noundo().params(
        "mass_kg: sprung mass (0 to 1e6); rate_n_mm: spring rate in N/mm (or rate_n_m); damping_ns_m?: a damper's coefficient; loss_factor?: a \
         material's (or material?: \"Aluminium\" | \"Steel\" | \"Titanium\" | \"Carbon fibre\" | \"Bamboo\" | \"Wood (ash, hickory)\" | \"Rubber \
         (natural)\"); excitation_hz?: the frequency to report at (default 10) → natural_hz, damping_ratio, static_deflection_mm, \
         logarithmic_decrement, cycles_to_half_amplitude, transmissibility, isolation_above_hz, verdict, advice, materials (the loss-factor table), \
         warnings",
    ),
    CommandSpec::new("inspect.bamboo_joint", "Bamboo Joint", bamboo_joint).at("3DEEE", "CALCULATE").icon("bamboo_joint").noundo().params(
        "culm_diameter_mm: the culm's outside diameter where it enters the socket (5 to 500); wall_mm: its wall (thinner than the radius); \
         engagement_mm: how far it goes into the socket (0.1 to 5000); glue_gap_mm?: radial, per side (default 0.3); adhesive_shear_mpa? (default 8, \
         epoxy on prepared bamboo); bamboo_shear_mpa? (default 5, interlaminar: the one that usually decides); bamboo_crush_mpa? (default 11, across \
         the fibre); node_inside_socket?: true when a node lies in the engaged length (doubles what the culm takes across the fibre); axial_n?: the \
         pull-out force; moment_nm?: the bending moment at the socket mouth; safety?: the factor to size the engagement for (default 3) → bond_area_mm2, \
         engagement_ratio, governing_shear_mpa, bamboo_governs, pull_out_capacity_n, shear_stress_mpa, axial_safety, bearing_stress_mpa, \
         bearing_capacity_mpa, bearing_safety, moment_capacity_nm, engagement_needed_mm (for axial_n at `safety`), verdict, advice, warnings. The three \
         ways a glued bamboo joint fails: the culm shears out a plug along its fibres, the glue line lets go, the sleeve crushes the culm oval",
    ),
];

fn bamboo_joint(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.bamboo_joint";
    let joint = BambooJoint {
        culm_diameter: required(cmd, p, "culm_diameter_mm", 5.0..=500.0)? * 1e-3,
        wall_thickness: required(cmd, p, "wall_mm", 0.1..=250.0)? * 1e-3,
        engagement: required(cmd, p, "engagement_mm", 0.1..=5000.0)? * 1e-3,
        glue_gap: number(cmd, p, "glue_gap_mm", 0.3, 0.0..=10.0)? * 1e-3,
        adhesive_shear_strength: number(cmd, p, "adhesive_shear_mpa", 8.0, 0.01..=1000.0)? * 1e6,
        bamboo_shear_strength: number(cmd, p, "bamboo_shear_mpa", 5.0, 0.01..=1000.0)? * 1e6,
        bamboo_crush_strength: number(cmd, p, "bamboo_crush_mpa", 11.0, 0.01..=1000.0)? * 1e6,
        node_inside_socket: bool_(p, "node_inside_socket").unwrap_or(false),
        axial_force: number(cmd, p, "axial_n", 0.0, 0.0..=1e9)?,
        bending_moment: number(cmd, p, "moment_nm", 0.0, 0.0..=1e9)?,
    };
    let safety = number(cmd, p, "safety", 3.0, 0.1..=100.0)?;
    let r = check_bamboo_joint(&joint);
    if !r.valid {
        return Err(bad(cmd, r.advice));
    }
    let mut warnings: Vec<String> = Vec::new();
    if r.axial_safety > 0.0 && r.axial_safety < safety {
        warnings.push(format!("the pull-out safety {:.2} is below {safety:.1}", r.axial_safety));
    }
    if r.bearing_safety > 0.0 && r.bearing_safety < safety {
        warnings.push(format!("the culm crushes under the moment: bearing safety {:.2}", r.bearing_safety));
    }
    if r.engagement_ratio < 1.5 {
        warnings.push("the engagement is under 1.5 diameters".into());
    }
    let needed = engagement_for(&joint, joint.axial_force, safety);
    let verdict = match (r.axial_safety > 0.0, r.bearing_safety > 0.0) {
        (true, true) => format!(
            "pull-out safety {:.2} ({}), bearing safety {:.2}",
            r.axial_safety,
            if r.bamboo_governs { "the bamboo shears" } else { "the glue lets go" },
            r.bearing_safety
        ),
        (true, false) => {
            format!("pull-out safety {:.2}: {}", r.axial_safety, if r.bamboo_governs { "the bamboo shears first" } else { "the glue lets go first" })
        }
        (false, true) => format!("bearing safety {:.2} under the moment", r.bearing_safety),
        (false, false) => format!("holds {:.0} N in pull-out and {:.1} N·m before the culm crushes", r.pull_out_capacity, r.moment_capacity),
    };
    Ok(json!({
        "bond_area_mm2": r.bond_area * 1e6,
        "engagement_ratio": r.engagement_ratio,
        "governing_shear_mpa": r.governing_shear / 1e6,
        "bamboo_governs": r.bamboo_governs,
        "pull_out_capacity_n": r.pull_out_capacity,
        "shear_stress_mpa": r.shear_stress / 1e6,
        "axial_safety": r.axial_safety,
        "bearing_stress_mpa": r.bearing_stress / 1e6,
        "bearing_capacity_mpa": r.bearing_capacity / 1e6,
        "bearing_safety": r.bearing_safety,
        "moment_capacity_nm": r.moment_capacity,
        "engagement_needed_mm": needed * 1e3,
        "safety": safety,
        "verdict": verdict,
        "advice": r.advice,
        "warnings": warnings,
    }))
}

/// A finite number in `range`, `default` when absent.
fn number(cmd: &str, p: &Value, key: &str, default: f64, range: std::ops::RangeInclusive<f64>) -> Result<f64> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v
            .as_f64()
            .filter(|x| x.is_finite() && range.contains(x))
            .ok_or_else(|| bad(cmd, format!("`{key}` must be a number from {} to {}", range.start(), range.end()))),
    }
}

/// A finite number in `range` that must be given.
fn required(cmd: &str, p: &Value, key: &str, range: std::ops::RangeInclusive<f64>) -> Result<f64> {
    match p.get(key) {
        None | Some(Value::Null) => Err(bad(cmd, format!("give `{key}`"))),
        Some(_) => number(cmd, p, key, 0.0, range),
    }
}

/// A short string parameter, `default` when absent.
fn text<'a>(cmd: &str, p: &'a Value, key: &str, default: &'a str) -> Result<&'a str> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(Value::String(s)) if !s.is_empty() && s.len() <= 64 => Ok(s.as_str()),
        Some(_) => Err(bad(cmd, format!("`{key}` must be a short name"))),
    }
}

fn bolted_joint(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.bolted_joint";
    let joint = BoltedJoint {
        size: text(cmd, p, "size", "M8")?.to_string(),
        grade: text(cmd, p, "grade", "8.8")?.to_string(),
        clamped_length: required(cmd, p, "clamped_mm", 0.1..=10000.0)? * 1e-3,
        clamped_modulus: number(cmd, p, "clamped_modulus_pa", 210e9, 1e6..=1e13)?,
        bolt_modulus: number(cmd, p, "bolt_modulus_pa", 210e9, 1e6..=1e13)?,
        preload: number(cmd, p, "preload_n", 0.0, 0.0..=1e9)?,
        external_load: number(cmd, p, "external_n", 0.0, 0.0..=1e9)?,
        shear_load: number(cmd, p, "shear_n", 0.0, 0.0..=1e9)?,
        face_friction: number(cmd, p, "friction", 0.15, 0.0..=2.0)?,
        nut_factor: number(cmd, p, "nut_factor", 0.20, 0.01..=1.0)?,
        cycles: number(cmd, p, "cycles", 0.0, 0.0..=1e12)?,
    };
    let r = check_bolted_joint(&joint);
    if !r.solved {
        return Err(bad(cmd, r.verdict));
    }
    let mut warnings: Vec<String> = Vec::new();
    if r.separates {
        warnings.push("the joint separates under the external load".into());
    }
    if r.slips {
        warnings.push("the joint slips under the sideways load".into());
    }
    if r.safety_against_proof > 0.0 && r.safety_against_proof < 1.0 {
        warnings.push("the bolt is past its proof stress".into());
    }
    if r.fatigue_safety > 0.0 && r.fatigue_safety < 1.5 {
        warnings.push("the fatigue margin is below 1.5".into());
    }
    Ok(json!({
        "size": joint.size,
        "grade": joint.grade,
        "stress_area_mm2": r.stress_area * 1e6,
        "bolt_stiffness_n_m": r.bolt_stiffness,
        "clamped_stiffness_n_m": r.clamped_stiffness,
        "load_factor": r.load_factor,
        "preload_n": r.preload,
        "bolt_load_n": r.bolt_load,
        "residual_clamp_n": r.residual_clamp,
        "bolt_stress_mpa": r.bolt_stress / 1e6,
        "safety_against_proof": r.safety_against_proof,
        "separation_load_n": r.separation_load,
        "separates": r.separates,
        "slip_load_n": r.slip_load,
        "slips": r.slips,
        "tightening_torque_nm": r.tightening_torque,
        "alternating_stress_mpa": r.alternating_stress / 1e6,
        "endurance_limit_mpa": r.endurance_limit / 1e6,
        "fatigue_safety": r.fatigue_safety,
        "verdict": r.verdict,
        "advice": r.advice,
        "warnings": warnings,
    }))
}

fn thread_engagement(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.thread_engagement";
    let size = text(cmd, p, "size", "M6")?;
    let material_name = text(cmd, p, "material", "steel")?;
    let material = Material::parse(material_name).ok_or_else(|| bad(cmd, "`material` is \"steel\", \"aluminium\" or \"plastic\""))?;
    let engagement = required(cmd, p, "engagement_mm", 0.0..=10000.0)? * 1e-3;
    let through = bool_(p, "through").unwrap_or(false);
    let e = check_engagement(size, material, engagement, through).map_err(|why| bad(cmd, why))?;
    let (severity, warnings) = match e.severity {
        Severity::Ok => ("ok", Vec::new()),
        Severity::Note => ("note", vec![e.detail.clone()]),
        Severity::Warning => ("warning", vec![e.detail.clone()]),
    };
    Ok(json!({
        "size": size,
        "material": material.name(),
        "diameter_mm": e.diameter * 1e3,
        "needed_mm": e.needed * 1e3,
        "engagement_mm": e.measured * 1e3,
        "factor": e.factor,
        "through": through,
        "severity": severity,
        "verdict": e.detail,
        "warnings": warnings,
    }))
}

fn bearing_life_cmd(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.bearing_life";
    let mut duty = BearingDuty::default();
    let mut designation: Option<String> = None;
    let mut used_for: Option<&str> = None;
    if let Some(name) = p.get("designation").filter(|v| !v.is_null()) {
        let name = name.as_str().filter(|s| !s.is_empty() && s.len() <= 64).ok_or_else(|| bad(cmd, "`designation` must be a short name"))?;
        let b = find_bearing(name).ok_or_else(|| bad(cmd, format!("no bearing called `{name}` in the table")))?;
        duty.dynamic_rating = b.dynamic_rating;
        duty.static_rating = b.static_rating;
        designation = Some(b.designation.to_string());
        used_for = Some(b.used_for);
    }
    duty.dynamic_rating = number(cmd, p, "dynamic_n", duty.dynamic_rating, 0.0..=1e9)?;
    duty.static_rating = number(cmd, p, "static_n", duty.static_rating, 0.0..=1e9)?;
    duty.radial_load = required(cmd, p, "radial_n", 0.0..=1e9)?;
    duty.axial_load = number(cmd, p, "axial_n", 0.0, 0.0..=1e9)?;
    duty.radial_factor = number(cmd, p, "radial_factor", 0.56, 0.0..=10.0)?;
    duty.axial_factor = number(cmd, p, "axial_factor", 1.5, 0.0..=10.0)?;
    duty.roller_not_ball = bool_(p, "roller").unwrap_or(false);
    duty.revolutions_per_minute = number(cmd, p, "rpm", 0.0, 0.0..=1e6)?;
    duty.metres_per_revolution = number(cmd, p, "wheel_mm", 0.0, 0.0..=1e5)? * 1e-3 * std::f64::consts::PI;
    if designation.is_none() && !(duty.dynamic_rating > 0.0) {
        return Err(bad(cmd, "give `designation` or `dynamic_n` (the bearing's C rating)"));
    }
    let life = bearing_life(&duty);
    if !life.valid {
        return Err(bad(cmd, life.advice));
    }
    let mut warnings: Vec<String> = Vec::new();
    if life.load_ratio < 4.0 {
        warnings.push("the load is more than a quarter of the dynamic rating".into());
    }
    if life.static_safety > 0.0 && life.static_safety < 2.0 {
        warnings.push("the static safety is below 2: brinelling when parked loaded".into());
    }
    let verdict = if life.hours > 0.0 {
        format!("L10 {:.0} million revolutions, {:.0} hours at {:.0} rpm", life.million_revolutions, life.hours, duty.revolutions_per_minute)
    } else if life.kilometres > 0.0 {
        format!("L10 {:.0} million revolutions, {:.0} km", life.million_revolutions, life.kilometres)
    } else {
        format!("L10 {:.0} million revolutions", life.million_revolutions)
    };
    Ok(json!({
        "designation": designation,
        "used_for": used_for,
        "dynamic_n": duty.dynamic_rating,
        "static_n": duty.static_rating,
        "equivalent_load_n": life.equivalent_load,
        "load_ratio": life.load_ratio,
        "million_revolutions": life.million_revolutions,
        "hours": life.hours,
        "kilometres": life.kilometres,
        "static_safety": life.static_safety,
        "verdict": verdict,
        "advice": life.advice,
        "warnings": warnings,
    }))
}

fn damping(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.damping";
    let mut duty = SuspensionDuty { sprung_mass: required(cmd, p, "mass_kg", 0.0..=1e6)?, ..Default::default() };
    duty.spring_rate = match (p.get("rate_n_mm").filter(|v| !v.is_null()), p.get("rate_n_m").filter(|v| !v.is_null())) {
        (Some(_), _) => number(cmd, p, "rate_n_mm", 0.0, 0.0..=1e9)? * 1e3,
        (None, Some(_)) => number(cmd, p, "rate_n_m", 0.0, 0.0..=1e12)?,
        (None, None) => return Err(bad(cmd, "give `rate_n_mm` (or `rate_n_m`)")),
    };
    duty.damping_coefficient = number(cmd, p, "damping_ns_m", 0.0, 0.0..=1e9)?;
    duty.loss_factor = number(cmd, p, "loss_factor", 0.0, 0.0..=2.0)?;
    let mut material_used: Option<&str> = None;
    if let Some(name) = str_(p, "material") {
        let m = loss_factor_of(name).ok_or_else(|| {
            bad(cmd, format!("no loss factor for `{name}`: the table knows aluminium, steel, titanium, carbon fibre, bamboo, wood and rubber"))
        })?;
        if duty.loss_factor <= 0.0 {
            duty.loss_factor = m.loss_factor;
        }
        material_used = Some(m.name);
    }
    duty.excitation_hertz = number(cmd, p, "excitation_hz", 10.0, 0.0..=1e6)?;
    let r = check_suspension(&duty);
    if !r.valid {
        return Err(bad(cmd, r.advice));
    }
    let mut warnings: Vec<String> = Vec::new();
    if r.transmissibility > 1.0 {
        warnings.push(format!("amplifies at {:.1} Hz rather than isolating", duty.excitation_hertz));
    }
    if r.damping_ratio <= 0.0 {
        warnings.push("no damping was given: it rings for ever".into());
    }
    let verdict = if r.transmissibility > 1.0 {
        format!(
            "natural {:.1} Hz, zeta {:.3}: amplifies {:.2}x at {:.1} Hz",
            r.natural_hertz, r.damping_ratio, r.transmissibility, duty.excitation_hertz
        )
    } else {
        format!(
            "natural {:.1} Hz, zeta {:.3}: {:.0}% reaches the load at {:.1} Hz",
            r.natural_hertz,
            r.damping_ratio,
            r.transmissibility * 100.0,
            duty.excitation_hertz
        )
    };
    let materials: Vec<Value> = solidcraft_analysis::damping::MATERIAL_DAMPING
        .iter()
        .map(|m| json!({"name": m.name, "loss_factor": m.loss_factor, "note": m.note}))
        .collect();
    Ok(json!({
        "material": material_used,
        "natural_hz": r.natural_hertz,
        "damping_ratio": r.damping_ratio,
        "static_deflection_mm": r.static_deflection * 1e3,
        "logarithmic_decrement": r.logarithmic_decrement,
        "cycles_to_half_amplitude": r.cycles_to_half_amplitude,
        "transmissibility": r.transmissibility,
        "isolation_above_hz": r.isolation_above_hertz,
        "excitation_hz": duty.excitation_hertz,
        "verdict": verdict,
        "advice": r.advice,
        "materials": materials,
        "warnings": warnings,
    }))
}

#[cfg(test)]
#[path = "fasteners_tests.rs"]
mod tests;
