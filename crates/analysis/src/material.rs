//! A material that may answer differently in each direction.
//!
//! A 3D-printed part is not isotropic: extruded strands bond far better along a layer than
//! between layers. Wood and bamboo are stiff along the grain and soft across it. Treating either
//! as a solid block of one modulus overstates it exactly where it breaks.

use serde::{Deserialize, Serialize};
use solidcraft_geom::Vec3;

/// Why a material cannot be used.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum MaterialError {
    /// A modulus is zero, negative or not a number.
    #[error("every modulus must be a positive, finite number")]
    Modulus,
    /// A ratio or the fibre direction is not a number, or the compliance matrix the constants
    /// describe cannot be inverted.
    #[error("the material's stiffnesses and Poisson ratios are not physically consistent")]
    Inconsistent,
}

/// Linear elastic constants, written in the material's own frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OrthotropicMaterial {
    /// Young's moduli along the material's x, y and z (Pa).
    pub ex: f64,
    pub ey: f64,
    pub ez: f64,
    /// Major Poisson ratios.
    pub nu_xy: f64,
    pub nu_yz: f64,
    pub nu_zx: f64,
    /// Shear moduli (Pa).
    pub gxy: f64,
    pub gyz: f64,
    pub gzx: f64,
    /// Where the material's own x axis points in the model. Zero means the material frame
    /// coincides with the model's. Only one direction is needed: bamboo, wood and a unidirectional
    /// laminate are transversely isotropic, so how they are rolled about the fibre does not
    /// matter.
    pub fibre_direction: Vec3,
}

impl OrthotropicMaterial {
    /// The same modulus and ratio in every direction.
    pub fn isotropic(youngs_modulus: f64, poisson_ratio: f64) -> Self {
        let g = youngs_modulus / (2.0 * (1.0 + poisson_ratio));
        Self {
            ex: youngs_modulus,
            ey: youngs_modulus,
            ez: youngs_modulus,
            nu_xy: poisson_ratio,
            nu_yz: poisson_ratio,
            nu_zx: poisson_ratio,
            gxy: g,
            gyz: g,
            gzx: g,
            fibre_direction: Vec3::ZERO,
        }
    }

    /// Stiff along one direction and soft across it: bamboo, wood, a unidirectional laminate.
    /// `along` and `across` are the two Young's moduli, `shear` the modulus between them.
    pub fn along_fibre(along: f64, across: f64, shear: f64, poisson_ratio: f64, direction: Vec3) -> Self {
        Self {
            ex: along,
            ey: across,
            ez: across,
            nu_xy: poisson_ratio,
            nu_yz: poisson_ratio,
            nu_zx: poisson_ratio * across / along.max(f64::MIN_POSITIVE),
            gxy: shear,
            gyz: across / (2.0 * (1.0 + poisson_ratio)),
            gzx: shear,
            fibre_direction: direction,
        }
    }

    /// The modulus everything else is scaled against: the stiffest direction.
    pub fn reference(&self) -> f64 {
        self.ex.max(self.ey).max(self.ez)
    }

    /// Every modulus positive and finite, every ratio and the fibre direction finite.
    pub fn validate(&self) -> Result<(), MaterialError> {
        let moduli = [self.ex, self.ey, self.ez, self.gxy, self.gyz, self.gzx];
        if moduli.iter().any(|m| !(*m > 0.0) || !m.is_finite()) {
            return Err(MaterialError::Modulus);
        }
        if ![self.nu_xy, self.nu_yz, self.nu_zx].iter().all(|n| n.is_finite()) || !self.fibre_direction.is_finite() {
            return Err(MaterialError::Inconsistent);
        }
        Ok(())
    }

    pub fn is_valid(&self) -> bool {
        self.validate().is_ok()
    }
}
