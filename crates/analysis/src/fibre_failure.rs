//! Whether a material with a grain in it has broken.
//!
//! Von Mises is a criterion for ductile metals: it rolls the stress state into one number on the
//! assumption that the material is equally strong in every direction. Bamboo, wood and a
//! unidirectional laminate are forty times stronger along the fibre than across it, so the stress
//! is taken back into the fibre's own frame and each component checked against the strength in
//! that direction: along the fibre in tension and in compression, across it in tension (the low
//! number that splits it) and compression, and in shear between the layers. This is the
//! max-stress criterion, chosen because it says which way the part fails.

use serde::{Deserialize, Serialize};
use solidcraft_geom::Vec3;

/// What a fibre material can take, in its own frame (Pa, positive numbers).
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct FibreStrength {
    pub along_tension: f64,
    pub along_compression: f64,
    /// The one that splits it.
    pub across_tension: f64,
    pub across_compression: f64,
    /// Interlaminar.
    pub shear: f64,
}

/// A bamboo species' strengths and stiffnesses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BambooSpecies {
    pub name: &'static str,
    pub latin: &'static str,
    /// kg/m³, air dry
    pub density: f64,
    /// Pa, along the fibre, across it, and in shear.
    pub modulus_along: f64,
    pub modulus_across: f64,
    pub shear_modulus: f64,
    pub tension_along: f64,
    pub compression_along: f64,
    pub shear_strength: f64,
    pub crush_across: f64,
}

/// The bamboo species in the content library: middling published numbers; bamboo varies by a
/// third either way between plants.
pub const BAMBOO: &[BambooSpecies] = &[
    BambooSpecies {
        name: "Tonkin",
        latin: "Pseudosasa amabilis",
        density: 780.0,
        modulus_along: 20e9,
        modulus_across: 1.0e9,
        shear_modulus: 0.7e9,
        tension_along: 200e6,
        compression_along: 65e6,
        shear_strength: 6.0e6,
        crush_across: 14e6,
    },
    BambooSpecies {
        name: "Moso",
        latin: "Phyllostachys edulis",
        density: 670.0,
        modulus_along: 15e9,
        modulus_across: 0.8e9,
        shear_modulus: 0.6e9,
        tension_along: 150e6,
        compression_along: 55e6,
        shear_strength: 5.0e6,
        crush_across: 11e6,
    },
    BambooSpecies {
        name: "Guadua",
        latin: "Guadua angustifolia",
        density: 700.0,
        modulus_along: 17e9,
        modulus_across: 0.9e9,
        shear_modulus: 0.65e9,
        tension_along: 150e6,
        compression_along: 50e6,
        shear_strength: 5.5e6,
        crush_across: 12e6,
    },
    BambooSpecies {
        name: "Generic bamboo",
        latin: "",
        density: 700.0,
        modulus_along: 15e9,
        modulus_across: 0.9e9,
        shear_modulus: 0.6e9,
        tension_along: 160e6,
        compression_along: 55e6,
        shear_strength: 5.0e6,
        crush_across: 11e6,
    },
];

/// The strengths of a bamboo species by name (case-insensitive), or `None` rather than a
/// plausible default. Across the fibre in tension runs about the interlaminar shear strength,
/// because both are the matrix between the fibres letting go.
pub fn bamboo_strength(species: &str) -> Option<FibreStrength> {
    let found = BAMBOO.iter().find(|s| s.name.eq_ignore_ascii_case(species.trim()))?;
    Some(FibreStrength {
        along_tension: found.tension_along,
        along_compression: found.compression_along,
        across_tension: found.shear_strength,
        across_compression: found.crush_across,
        shear: found.shear_strength,
    })
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FibreFailureReport {
    pub valid: bool,
    pub advice: String,
    /// The worst utilisation anywhere: 1 means it is exactly at strength.
    pub worst_ratio: f64,
    /// Which way it goes there.
    pub mode: String,
    pub worst_voxel: Option<usize>,
    /// Each mode's utilisation at its own worst voxel.
    pub along_tension_ratio: f64,
    pub along_compression_ratio: f64,
    pub across_tension_ratio: f64,
    pub across_compression_ratio: f64,
    pub shear_ratio: f64,
    /// What von Mises would have said, for comparison.
    pub von_mises_ratio_against_along_strength: f64,
}

/// The solved state this check needs, and nothing else.
#[derive(Debug, Clone, Default)]
pub struct FibreState {
    /// Six per voxel: xx yy zz xy yz zx.
    pub stress_tensor: Vec<f64>,
    pub solid: Vec<bool>,
    /// May be empty.
    pub exclude_from_peak: Vec<bool>,
    pub fibre_direction: Vec3,
    /// For the comparison; may be 0.
    pub worst_von_mises: f64,
}

/// The fibre frame: the fibre and two directions across it, perpendicular to it and to each
/// other.
fn fibre_frame(fibre: Vec3) -> Option<[[f64; 3]; 3]> {
    let axis = fibre.normalized()?;
    let helper = if axis.z.abs() > 0.9 { Vec3::X } else { Vec3::Z };
    let across = axis.cross(helper).normalized()?;
    let third = axis.cross(across);
    Some([[axis.x, axis.y, axis.z], [across.x, across.y, across.z], [third.x, third.y, third.z]])
}

/// Checks a solved stress state against `strength`, in the frame of the material's own fibre.
pub fn check_fibre_failure(state: &FibreState, strength: &FibreStrength) -> FibreFailureReport {
    let mut report = FibreFailureReport::default();
    let voxels = state.solid.len();
    if voxels == 0 || state.stress_tensor.len() != voxels * 6 {
        report.advice = "there is no solved stress state to check".into();
        return report;
    }
    let Some(axes) = fibre_frame(state.fibre_direction) else {
        report.advice = "this material has no fibre direction, so there is no frame to take the stress back into. Set it on the material: a check that does not know which way the grain runs cannot tell splitting from stretching".into();
        return report;
    };
    if !(strength.along_tension > 0.0) || !(strength.across_tension > 0.0) || !(strength.shear > 0.0) {
        report.advice = "every strength has to be a positive number".into();
        return report;
    }
    report.valid = true;
    let ratio_of = |stress: f64, limit: f64| if limit > 0.0 { stress.abs() / limit } else { 0.0 };
    let exclude = state.exclude_from_peak.len() == voxels;
    for v in 0..voxels {
        if !state.solid[v] || (exclude && state.exclude_from_peak[v]) {
            continue;
        }
        let s = &state.stress_tensor[v * 6..v * 6 + 6];
        let global = [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]];
        let mut local = [[0.0; 3]; 3];
        for (a, row) in local.iter_mut().enumerate() {
            for (b, value) in row.iter_mut().enumerate() {
                let mut sum = 0.0;
                for i in 0..3 {
                    for j in 0..3 {
                        sum += axes[a][i] * global[i][j] * axes[b][j];
                    }
                }
                *value = sum;
            }
        }
        let along = local[0][0];
        let along_tension = if along > 0.0 { ratio_of(along, strength.along_tension) } else { 0.0 };
        let along_compression = if along < 0.0 { ratio_of(along, strength.along_compression) } else { 0.0 };
        let (mut across_tension, mut across_compression) = (0.0_f64, 0.0_f64);
        for (axis, row) in local.iter().enumerate().skip(1) {
            let across = row[axis];
            if across > 0.0 {
                across_tension = across_tension.max(ratio_of(across, strength.across_tension));
            } else {
                across_compression = across_compression.max(ratio_of(across, strength.across_compression));
            }
        }
        let shear = ratio_of(local[0][1], strength.shear).max(ratio_of(local[0][2], strength.shear));
        report.along_tension_ratio = report.along_tension_ratio.max(along_tension);
        report.along_compression_ratio = report.along_compression_ratio.max(along_compression);
        report.across_tension_ratio = report.across_tension_ratio.max(across_tension);
        report.across_compression_ratio = report.across_compression_ratio.max(across_compression);
        report.shear_ratio = report.shear_ratio.max(shear);
        let modes = [
            (along_tension, "tension along the fibre"),
            (along_compression, "compression along the fibre"),
            (across_tension, "splitting across the fibre"),
            (across_compression, "crushing across the fibre"),
            (shear, "interlaminar shear"),
        ];
        let (worst, mode) = modes.iter().fold((0.0_f64, ""), |best, m| if m.0 > best.0 { (m.0, m.1) } else { best });
        if worst > report.worst_ratio {
            report.worst_ratio = worst;
            report.worst_voxel = Some(v);
            report.mode = mode.into();
        }
    }
    if state.worst_von_mises > 0.0 {
        report.von_mises_ratio_against_along_strength = state.worst_von_mises / strength.along_tension;
    }
    let mut advice = if report.worst_ratio >= 1.0 {
        format!("It fails, by {}, at {:.2} times what it can take there. ", report.mode, report.worst_ratio)
    } else if report.worst_ratio > 0.0 {
        format!("The governing mode is {}, at {:.2} of strength (safety {:.2}). ", report.mode, report.worst_ratio, 1.0 / report.worst_ratio)
    } else {
        "Nothing in it is stressed. ".to_string()
    };
    if report.von_mises_ratio_against_along_strength > 0.0 && report.worst_ratio > 1.5 * report.von_mises_ratio_against_along_strength {
        advice.push_str(&format!(
            "Von Mises against the along-fibre strength would have called this {:.2} -- comfortable. It is not: a metal criterion cannot see the difference between pulling along the fibre and splitting across it, and this part is doing the second. ",
            report.von_mises_ratio_against_along_strength
        ));
    }
    if report.across_tension_ratio >= 0.5 || report.shear_ratio >= 0.5 {
        advice.push_str("What is loading it across the fibre is worth looking at directly: for bamboo that usually means a joint, a clamp or a hole, and the cure is geometry rather than a bigger tube. ");
    }
    report.advice = advice.trim_end().to_string();
    report
}

#[cfg(test)]
#[path = "fibre_failure_tests.rs"]
mod tests;
