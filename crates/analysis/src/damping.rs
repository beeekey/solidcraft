//! Damping: what a frame does with the road, and what a damper adds to it.
//!
//! Every material turns some of the energy of a vibration into heat each cycle, and the fraction
//! is its loss factor. Aluminium's is about 0.0005, steel's 0.002, carbon fibre's 0.003, bamboo's
//! 0.02 to 0.04: a natural fibre composite with a soft lignin matrix between stiff cellulose
//! fibres, and the matrix does the damping. For light damping the loss factor gives the damping
//! ratio directly (ζ = η / 2), and from there the logarithmic decrement, how many cycles a
//! ring-down takes to fade, and how much of a bump gets through.
//!
//! The same arithmetic covers a real damper on a trailer: a sprung mass, a spring rate and a
//! damping coefficient give the natural frequency, the damping ratio and the transmissibility at
//! any frequency.

use std::f64::consts::PI;

/// A sprung thing: a mass on a spring, with something taking energy out.
#[derive(Debug, Clone, PartialEq)]
pub struct SuspensionDuty {
    /// kg carried by the spring.
    pub sprung_mass: f64,
    /// N/m.
    pub spring_rate: f64,
    /// N·s/m. Either give this, or a material's loss factor and let it be worked out, which is how
    /// a bamboo member with no damper on it still has damping.
    pub damping_coefficient: f64,
    /// Dimensionless material damping.
    pub loss_factor: f64,
    /// The frequency to report transmissibility at. Road roughness under a bicycle is broad, but
    /// the part that shakes cargo apart is 5–20 Hz.
    pub excitation_hertz: f64,
}

impl Default for SuspensionDuty {
    fn default() -> Self {
        SuspensionDuty { sprung_mass: 0.0, spring_rate: 0.0, damping_coefficient: 0.0, loss_factor: 0.0, excitation_hertz: 10.0 }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct SuspensionResult {
    pub valid: bool,
    pub advice: String,
    pub natural_hertz: f64,
    /// ζ.
    pub damping_ratio: f64,
    /// m: how far it sinks under its own load.
    pub static_deflection: f64,
    pub logarithmic_decrement: f64,
    /// How many cycles a ring-down takes to fall to half amplitude: a metal frame rings for
    /// hundreds, a bamboo one for a handful.
    pub cycles_to_half_amplitude: f64,
    /// How much of the excitation reaches the load: below 1 it is isolated, above 1 amplified.
    pub transmissibility: f64,
    /// Above this frequency the suspension isolates rather than amplifies.
    pub isolation_above_hertz: f64,
}

/// The loss factors the application quotes, for comparison. Representative measured values; the
/// point of the table is the ratio between the rows rather than any one number.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaterialDamping {
    pub name: &'static str,
    pub loss_factor: f64,
    pub note: &'static str,
}

pub const MATERIAL_DAMPING: &[MaterialDamping] = &[
    MaterialDamping {
        name: "Aluminium",
        loss_factor: 0.0005,
        note: "almost none: an aluminium frame passes the road straight through, which is why it is the material that gets called harsh",
    },
    MaterialDamping { name: "Steel", loss_factor: 0.002, note: "four times aluminium, and riders can feel the difference" },
    MaterialDamping { name: "Titanium", loss_factor: 0.002, note: "about steel" },
    MaterialDamping { name: "Carbon fibre", loss_factor: 0.003, note: "the matrix does some damping; the layup can be tuned to do more" },
    MaterialDamping {
        name: "Bamboo",
        loss_factor: 0.030,
        note: "ten to eighty times the metals. The lignin between the cellulose fibres is a soft matrix working against stiff fibres, and that is what turns vibration into heat. It is the reason a bamboo frame rides the way it does, and it is not a small effect",
    },
    MaterialDamping {
        name: "Wood (ash, hickory)",
        loss_factor: 0.015,
        note: "about half bamboo, and for the same reason -- which is why tool handles are wood and not aluminium",
    },
    MaterialDamping {
        name: "Rubber (natural)",
        loss_factor: 0.100,
        note: "what an elastomer bush is for: a hundred times a metal, in a part the size of a thumb",
    },
];

/// The loss factor of a material in the table (case-insensitive).
pub fn loss_factor_of(name: &str) -> Option<MaterialDamping> {
    let wanted = name.trim();
    MATERIAL_DAMPING.iter().copied().find(|m| m.name.eq_ignore_ascii_case(wanted))
}

/// Natural frequency, damping ratio, ring-down and transmissibility of a sprung mass.
pub fn check_suspension(duty: &SuspensionDuty) -> SuspensionResult {
    let mut result = SuspensionResult::default();
    if !(duty.sprung_mass > 0.0) || !(duty.spring_rate > 0.0) || !duty.sprung_mass.is_finite() || !duty.spring_rate.is_finite() {
        result.advice = "a suspension needs a mass and a spring rate".into();
        return result;
    }
    result.valid = true;
    let omega = (duty.spring_rate / duty.sprung_mass).sqrt();
    result.natural_hertz = omega / (2.0 * PI);
    result.static_deflection = duty.sprung_mass * 9.80665 / duty.spring_rate;
    let critical = 2.0 * (duty.spring_rate * duty.sprung_mass).sqrt();

    // Either a real damper, or the material's own loss factor, which for light damping is exactly
    // twice the damping ratio.
    result.damping_ratio = if duty.damping_coefficient > 0.0 { duty.damping_coefficient / critical } else { duty.loss_factor.max(0.0) / 2.0 };
    if result.damping_ratio > 0.0 && result.damping_ratio < 1.0 {
        result.logarithmic_decrement = 2.0 * PI * result.damping_ratio / (1.0 - result.damping_ratio * result.damping_ratio).sqrt();
        result.cycles_to_half_amplitude = 2.0_f64.ln() / result.logarithmic_decrement;
    }

    let ratio = duty.excitation_hertz.max(0.0) / result.natural_hertz;
    let zeta = result.damping_ratio;
    let top = 1.0 + (2.0 * zeta * ratio).powi(2);
    let bottom = (1.0 - ratio * ratio).powi(2) + (2.0 * zeta * ratio).powi(2);
    result.transmissibility = if bottom > 0.0 { (top / bottom).sqrt() } else { 0.0 };
    // Below √2 times the natural frequency a suspension makes things worse, not better: geometry
    // of the response curve, not a rule of thumb, and why a soft spring isolates and a stiff one
    // does not.
    result.isolation_above_hertz = 2.0_f64.sqrt() * result.natural_hertz;

    let mut advice = String::new();
    if result.transmissibility > 1.0 {
        advice.push_str(&format!(
            "At {:.2} Hz this AMPLIFIES by {:.2} rather than isolating: the excitation is below {:.2} Hz, where a suspension always makes things worse. Soften the spring or accept it. ",
            duty.excitation_hertz, result.transmissibility, result.isolation_above_hertz
        ));
    } else {
        advice.push_str(&format!(
            "It isolates: {:.0}% of the excitation at {:.0} Hz reaches the load. ",
            result.transmissibility * 100.0,
            duty.excitation_hertz
        ));
    }
    if zeta > 0.0 && zeta < 0.05 {
        advice.push_str(&format!(
            "Damping is light (zeta {zeta:.2}), so it isolates well but rings: it takes {:.2} cycles to fall to half amplitude after a bump. ",
            result.cycles_to_half_amplitude
        ));
    } else if zeta > 0.7 {
        advice.push_str(&format!("Damping is heavy (zeta {zeta:.2}): the bump is over quickly but more of it gets through. "));
    }
    if result.static_deflection > 0.0 {
        advice.push_str(&format!(
            "It sits {:.0} mm down under its own load, which is the sag to allow for in the travel.",
            result.static_deflection * 1000.0
        ));
    }
    result.advice = advice;
    result
}

#[cfg(test)]
#[path = "damping_tests.rs"]
mod tests;
