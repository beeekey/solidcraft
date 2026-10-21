//! What a bolted joint actually does.
//!
//! Placing a screw, a washer and a nut says what a joint looks like. The question an engineer
//! asks is whether it holds: does the preload survive the external load or do the parts come
//! apart, does the bolt yield, will it slip sideways, what torque is it tightened to? None of
//! that is a finite-element problem. A bolted joint is a pair of springs in parallel, the bolt
//! stretching and the clamped parts relaxing, and everything falls out of that, which is why
//! VDI 2230 and every textbook do it this way rather than by meshing the thread.
//!
//! What this is: the standard calculation on the real fastener table. What it is not: thread-root
//! stress concentration, embedding relaxation over time, gasket creep or eccentric loading. Those
//! are stated where the result is reported rather than left for the reader to assume.
//!
//! Lengths are metres, forces newtons, stresses pascals.

use std::f64::consts::PI;

/// A property class as stamped on the head: 8.8, 10.9, 12.9, plus an austenitic grade.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoltGrade {
    pub name: &'static str,
    /// Sp, Pa: the stress the bolt must not pass.
    pub proof_strength: f64,
    pub yield_strength: f64,
    pub ultimate_strength: f64,
}

/// Proof, yield and ultimate per ISO 898-1. Proof is the one that matters: a preload spec is
/// written as a fraction of it.
pub const GRADES: [BoltGrade; 6] = [
    BoltGrade { name: "4.6", proof_strength: 225e6, yield_strength: 240e6, ultimate_strength: 400e6 },
    BoltGrade { name: "5.8", proof_strength: 380e6, yield_strength: 420e6, ultimate_strength: 520e6 },
    BoltGrade { name: "8.8", proof_strength: 600e6, yield_strength: 640e6, ultimate_strength: 800e6 },
    BoltGrade { name: "10.9", proof_strength: 830e6, yield_strength: 940e6, ultimate_strength: 1040e6 },
    BoltGrade { name: "12.9", proof_strength: 970e6, yield_strength: 1100e6, ultimate_strength: 1220e6 },
    BoltGrade { name: "A2-70", proof_strength: 450e6, yield_strength: 450e6, ultimate_strength: 700e6 },
];

/// The grade with this name, if the table has it.
pub fn grade_named(name: &str) -> Option<BoltGrade> {
    GRADES.iter().copied().find(|g| g.name == name.trim())
}

/// ISO 261 coarse pitches (m): what "M8" means when nobody says otherwise. A size outside this
/// list is refused rather than interpolated: a guessed pitch is a wrong stress area is a wrong
/// answer.
const PITCHES: &[(&str, f64)] = &[
    ("M3", 0.00050),
    ("M4", 0.00070),
    ("M5", 0.00080),
    ("M6", 0.00100),
    ("M8", 0.00125),
    ("M10", 0.00150),
    ("M12", 0.00175),
    ("M14", 0.00200),
    ("M16", 0.00200),
    ("M20", 0.00250),
    ("M24", 0.00300),
    ("M30", 0.00350),
    ("M36", 0.00400),
];

/// Coarse pitch (m) of a standard size, `None` for any other.
pub fn coarse_pitch(size: &str) -> Option<f64> {
    let size = size.trim();
    PITCHES.iter().find(|(s, _)| s.eq_ignore_ascii_case(size)).map(|(_, p)| *p)
}

/// Nominal diameter (m) of an "M<number>" size, `None` when it is not one.
pub fn nominal_diameter(size: &str) -> Option<f64> {
    let rest = size.trim().strip_prefix(['M', 'm'])?;
    let d: f64 = rest.trim().parse().ok()?;
    (d.is_finite() && d > 0.0).then_some(d * 1e-3)
}

/// The thread's tensile stress area (m²): As = π/4 (d − 0.9382 p)². `None` for a size whose
/// coarse pitch is not in the table.
pub fn thread_stress_area(size: &str) -> Option<f64> {
    let pitch = coarse_pitch(size)?;
    let diameter = nominal_diameter(size)?;
    let effective = diameter - 0.9382 * pitch;
    (effective > 0.0).then(|| PI / 4.0 * effective * effective)
}

/// The joint to check.
#[derive(Debug, Clone, PartialEq)]
pub struct BoltedJoint {
    /// An ISO metric size from the pitch table.
    pub size: String,
    pub grade: String,
    /// Grip: the total thickness of the parts being clamped (m).
    pub clamped_length: f64,
    /// What the clamped parts are made of, and the bolt (Pa).
    pub clamped_modulus: f64,
    pub bolt_modulus: f64,
    /// Preload (N). 0 asks for the usual 75 % of proof, which is what a torque spec is normally
    /// written to.
    pub preload: f64,
    /// The external load pulling the joint apart, per bolt, along the axis (N).
    pub external_load: f64,
    /// A sideways load the joint must carry by friction, per bolt (N).
    pub shear_load: f64,
    /// Friction between the clamped faces, for the slip check.
    pub face_friction: f64,
    /// Nut factor K in T = K F d: 0.20 dry, 0.15 lightly oiled.
    pub nut_factor: f64,
    /// How many times the external load is applied, for the fatigue check.
    pub cycles: f64,
}

impl Default for BoltedJoint {
    fn default() -> Self {
        BoltedJoint {
            size: "M8".into(),
            grade: "8.8".into(),
            clamped_length: 0.020,
            clamped_modulus: 210e9,
            bolt_modulus: 210e9,
            preload: 0.0,
            external_load: 0.0,
            shear_load: 0.0,
            face_friction: 0.15,
            nut_factor: 0.20,
            cycles: 0.0,
        }
    }
}

/// The answer.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BoltedJointResult {
    pub solved: bool,
    /// As (m²).
    pub stress_area: f64,
    /// kb (N/m).
    pub bolt_stiffness: f64,
    /// km (N/m).
    pub clamped_stiffness: f64,
    /// C = kb / (kb + km): the share of the external load the bolt takes. The rest comes off the
    /// clamp, which is the point of preloading.
    pub load_factor: f64,
    /// N actually used.
    pub preload: f64,
    /// N in the bolt at full external load.
    pub bolt_load: f64,
    /// N still holding the parts together.
    pub residual_clamp: f64,
    /// Pa, direct tension.
    pub bolt_stress: f64,
    pub safety_against_proof: f64,
    /// N at which the parts lift apart.
    pub separation_load: f64,
    pub separates: bool,
    /// N sideways the friction can carry.
    pub slip_load: f64,
    pub slips: bool,
    /// N·m.
    pub tightening_torque: f64,
    /// Alternating stress in the bolt and the endurance limit it is held to (rolled thread);
    /// 0 when no cycles were given.
    pub alternating_stress: f64,
    pub endurance_limit: f64,
    pub fatigue_safety: f64,
    pub verdict: String,
    pub advice: String,
}

/// The standard check. `solved` is false, with the reason in `verdict`, for a size or grade the
/// tables do not know or a grip of nothing.
pub fn check_bolted_joint(joint: &BoltedJoint) -> BoltedJointResult {
    let mut result = BoltedJointResult::default();
    let (Some(diameter), Some(area)) = (nominal_diameter(&joint.size), thread_stress_area(&joint.size)) else {
        result.verdict = format!("'{}' is not a metric size this knows the pitch of (M3 to M36 coarse)", joint.size);
        return result;
    };
    result.stress_area = area;
    let Some(grade) = grade_named(&joint.grade) else {
        result.verdict = format!("'{}' is not a property class in the table", joint.grade);
        return result;
    };
    if !(joint.clamped_length > 0.0) || !joint.clamped_length.is_finite() {
        result.verdict = "the clamped length (the grip) has to be more than nothing".into();
        return result;
    }
    if !(joint.bolt_modulus > 0.0) || !(joint.clamped_modulus > 0.0) || !joint.bolt_modulus.is_finite() || !joint.clamped_modulus.is_finite() {
        result.verdict = "the bolt and the clamped parts need positive moduli".into();
        return result;
    }

    // The bolt is a bar in tension over the grip.
    result.bolt_stiffness = area * joint.bolt_modulus / joint.clamped_length;

    // The clamped parts are the frustum of material the head squeezes, not the whole plate: the
    // standard closed form for two similar members with a head bearing on about 1.5 d.
    {
        let bearing = 1.5 * diameter;
        let top = 0.5774 * joint.clamped_length + 0.5 * bearing;
        let bottom = 0.5774 * joint.clamped_length + 2.5 * bearing;
        let ratio = 5.0 * top / bottom;
        result.clamped_stiffness = if ratio > 1.0 { 0.5774 * PI * joint.clamped_modulus * diameter / (2.0 * ratio.ln()) } else { 0.0 };
    }
    if !(result.clamped_stiffness > 0.0) {
        result.verdict = "the clamped members have no stiffness at this grip length".into();
        return result;
    }
    result.load_factor = result.bolt_stiffness / (result.bolt_stiffness + result.clamped_stiffness);

    // 75 % of proof is the usual target for a reusable joint, and what a published torque figure
    // is normally written to.
    result.preload = if joint.preload > 0.0 { joint.preload } else { 0.75 * area * grade.proof_strength };

    let external = joint.external_load.max(0.0);
    result.bolt_load = result.preload + result.load_factor * external;
    result.residual_clamp = result.preload - (1.0 - result.load_factor) * external;
    result.bolt_stress = result.bolt_load / area;
    result.safety_against_proof = if result.bolt_stress > 0.0 { grade.proof_strength / result.bolt_stress } else { 0.0 };

    // Separation: where the clamp runs out. Past it the bolt takes the whole external load
    // directly and the joint is no longer a joint.
    result.separation_load = if result.load_factor < 1.0 { result.preload / (1.0 - result.load_factor) } else { 0.0 };
    result.separates = external >= result.separation_load;

    // Slip: a bolt in a clearance hole carries sideways load by friction between the faces, not
    // in shear.
    result.slip_load = joint.face_friction * result.residual_clamp.max(0.0);
    result.slips = joint.shear_load > result.slip_load;

    result.tightening_torque = joint.nut_factor * result.preload * diameter;

    if joint.cycles > 0.0 && external > 0.0 {
        // The bolt only feels its share of the external load, and only half of that as an
        // alternating component about the mean.
        result.alternating_stress = 0.5 * result.load_factor * external / area;
        // Rolled threads, per the usual figures for the class: an endurance limit for the thread,
        // not for a plain bar, because that is where a bolt fails.
        result.endurance_limit = if grade.name == "12.9" || grade.name == "10.9" { 140e6 } else { 129e6 };
        result.fatigue_safety = if result.alternating_stress > 0.0 { result.endurance_limit / result.alternating_stress } else { 0.0 };
    }

    let mut verdict = format!(
        "{} {}: preload {:.1} kN at {:.1} N.m, bolt sees {:.1} kN ({:.1} MPa, {:.2} x proof)",
        joint.size,
        grade.name,
        result.preload * 1e-3,
        result.tightening_torque,
        result.bolt_load * 1e-3,
        result.bolt_stress * 1e-6,
        result.safety_against_proof
    );
    if external > 0.0 {
        verdict.push_str(&format!(", clamp left {:.1} kN", result.residual_clamp * 1e-3));
    }
    result.verdict = verdict;

    let mut advice = String::new();
    if result.separates {
        advice.push_str(&format!(
            "THE JOINT SEPARATES: {:.1} kN is past the {:.1} kN this preload can hold. The bolt then carries the whole external load directly and fatigues quickly. Preload harder, use a bigger bolt, or add bolts. ",
            external * 1e-3,
            result.separation_load * 1e-3
        ));
    }
    if result.safety_against_proof < 1.0 && result.safety_against_proof > 0.0 {
        advice.push_str(&format!("The bolt is past its proof stress ({:.2} x): it is yielding. ", result.safety_against_proof));
    }
    if joint.shear_load > 0.0 {
        if result.slips {
            advice.push_str(&format!(
                "IT SLIPS: friction can hold {:.3} kN sideways and {:.3} kN is applied. A bolt in a clearance hole is not a shear pin -- add a dowel, or clamp harder. ",
                result.slip_load * 1e-3,
                joint.shear_load * 1e-3
            ));
        } else {
            advice.push_str(&format!(
                "Friction holds {:.1} kN sideways against the {:.1} kN applied. ",
                result.slip_load * 1e-3,
                joint.shear_load * 1e-3
            ));
        }
    }
    if result.fatigue_safety > 0.0 && result.fatigue_safety < 1.5 {
        advice.push_str(&format!("Fatigue margin is only {:.2} on the thread's endurance limit. ", result.fatigue_safety));
    }
    if joint.preload <= 0.0 {
        advice.push_str("Preload was taken as 75% of proof, which is the usual target for a joint that comes apart again; say what you actually torque to if it differs. ");
    }
    advice.push_str(&format!(
        "The torque figure assumes a nut factor of {:.1}, which is a lubrication guess and the largest uncertainty here: torque control alone scatters preload by about a quarter either way. ",
        joint.nut_factor
    ));
    advice.push_str("Not modelled: embedding relaxation, gasket creep, eccentric loading, or stress concentration at the thread root.");
    result.advice = advice;
    result.solved = true;
    result
}

#[cfg(test)]
#[path = "bolt_tests.rs"]
mod tests;
