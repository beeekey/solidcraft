//! Thread engagement: how deep a tapped hole has to be for the thread to hold.
//!
//! The rule is the design-for-manufacture one: 1.5 × D in steel, 2 × D in aluminium, 2.5 × D in
//! plastic (the low end of 2.5–3). Shorter and the thread strips before the bolt breaks; a full
//! diameter deeper than needed in a blind hole is tapping that costs money and breaks taps for
//! nothing. Lengths are metres.

use crate::bolt::nominal_diameter;

/// What the tapped part is made of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Material {
    Steel,
    Aluminium,
    Plastic,
}

impl Material {
    /// Parses "steel", "aluminium"/"aluminum", "plastic" (case-insensitive).
    pub fn parse(name: &str) -> Option<Material> {
        match name.trim().to_ascii_lowercase().as_str() {
            "steel" | "stainless" | "stainless steel" => Some(Material::Steel),
            "aluminium" | "aluminum" | "alu" => Some(Material::Aluminium),
            "plastic" | "abs" | "pla" | "nylon" | "polycarbonate" => Some(Material::Plastic),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Material::Steel => "steel",
            Material::Aluminium => "aluminium",
            Material::Plastic => "plastic",
        }
    }
}

/// Engagement the material's rule asks for, as a multiple of D.
pub fn engagement_factor(material: Material) -> f64 {
    match material {
        Material::Steel => 1.5,
        Material::Aluminium => 2.0,
        Material::Plastic => 2.5,
    }
}

/// How serious a finding is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// Enough engagement, nothing to say.
    Ok,
    /// Deeper than it needs to be in a blind hole: expensive tapping.
    Note,
    /// Short of the rule: the thread strips first.
    Warning,
}

/// The check's answer.
#[derive(Debug, Clone, PartialEq)]
pub struct Engagement {
    pub severity: Severity,
    /// The thread's nominal diameter (m).
    pub diameter: f64,
    /// Engagement the rule asks for (m).
    pub needed: f64,
    /// Engagement given (m).
    pub measured: f64,
    pub factor: f64,
    pub detail: String,
}

/// Checks `engagement` metres of an `M<n>` thread in `material`; `through` says the hole goes
/// through the part, so deeper is not an option there and a nut or an insert is the fix.
pub fn check_engagement(size: &str, material: Material, engagement: f64, through: bool) -> Result<Engagement, String> {
    let diameter = nominal_diameter(size).ok_or_else(|| format!("'{size}' is not an M-size"))?;
    if !(engagement >= 0.0) || !engagement.is_finite() {
        return Err("the engagement must be a finite length".into());
    }
    let factor = engagement_factor(material);
    let needed = factor * diameter;
    let mm = |m: f64| format!("{:.2}", m * 1e3);
    let name = material.name();
    let (severity, detail) = if engagement + 1e-9 < needed {
        (
            Severity::Warning,
            format!(
                "{size} tapped {} mm {} in {name}; {factor:.1} x D = {} mm of engagement is the rule -- {}",
                mm(engagement),
                if through { "through" } else { "deep" },
                mm(needed),
                if through {
                    "a nut or a heat-set insert holds where the plate cannot"
                } else {
                    "deepen it, or move the thread into thicker material"
                }
            ),
        )
    } else if engagement > needed + diameter && !through {
        (
            Severity::Note,
            format!(
                "{size} tapped {} mm deep where {} mm holds in {name}; tapped depth is expensive and tap breakage is worse",
                mm(engagement),
                mm(needed)
            ),
        )
    } else {
        (Severity::Ok, format!("{size} tapped {} mm in {name} meets the {factor:.1} x D rule ({} mm)", mm(engagement), mm(needed)))
    };
    Ok(Engagement { severity, diameter, needed, measured: engagement, factor, detail })
}

#[cfg(test)]
#[path = "thread_engagement_tests.rs"]
mod tests;
