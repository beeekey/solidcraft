//! Elastic constants for the built-in physical materials.
//!
//! Textbook values for a representative grade of each material (an aluminium alloy of the 6061-T6
//! kind, a grey cast iron, a mild steel), good enough to size a part and say so. A design that
//! knows better passes its own numbers.

/// Linear elastic constants and the stress the material yields at.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Elastic {
    /// Pa
    pub young_modulus_pa: f64,
    pub poisson_ratio: f64,
    /// Pa; tensile strength for a brittle material, which has no yield of its own.
    pub yield_pa: f64,
    /// What the numbers stand for.
    pub note: &'static str,
}

/// The constants for a material by its built-in name (case-insensitive), `None` for a material
/// the handbook does not know.
pub fn elastic(material: &str) -> Option<Elastic> {
    let name = material.trim();
    let entry = |young_modulus_pa, poisson_ratio, yield_pa, note| Some(Elastic { young_modulus_pa, poisson_ratio, yield_pa, note });
    match name.to_ascii_lowercase().as_str() {
        "default" => entry(2.5e9, 0.35, 50e6, "a generic engineering plastic"),
        "steel" => entry(200e9, 0.29, 250e6, "mild structural steel"),
        "stainless steel" => entry(193e9, 0.27, 215e6, "austenitic stainless steel, annealed"),
        "aluminum" | "aluminium" => entry(69e9, 0.33, 275e6, "a 6061-T6 class aluminium alloy"),
        "brass" => entry(100e9, 0.34, 200e6, "cartridge brass, half hard"),
        "copper" => entry(117e9, 0.34, 70e6, "annealed copper"),
        "titanium" => entry(110e9, 0.34, 880e6, "a Ti-6Al-4V class alloy"),
        "cast iron" => entry(110e9, 0.26, 150e6, "grey cast iron (tensile strength)"),
        "abs plastic" | "abs" => entry(2.0e9, 0.35, 40e6, "injection-moulded ABS"),
        "pla" => entry(3.5e9, 0.36, 60e6, "PLA as printed, along the layers"),
        "nylon" => entry(2.0e9, 0.39, 60e6, "PA6 nylon, dry"),
        "polycarbonate" => entry(2.3e9, 0.37, 62e6, "polycarbonate"),
        "oak" => entry(11e9, 0.35, 50e6, "oak along the grain, as if isotropic"),
        "glass" => entry(70e9, 0.22, 50e6, "soda-lime glass (tensile strength)"),
        _ => None,
    }
}

/// How a material conducts and radiates heat.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thermal {
    /// W/m·K
    pub conductivity: f64,
    /// A typical emissivity of the material's plain surface, 0 to 1.
    pub emissivity: f64,
}

/// The thermal constants for a material by its built-in name (case-insensitive).
pub fn thermal(material: &str) -> Option<Thermal> {
    let entry = |conductivity, emissivity| Some(Thermal { conductivity, emissivity });
    match material.trim().to_ascii_lowercase().as_str() {
        "default" => entry(0.25, 0.90),
        "steel" => entry(50.0, 0.80),
        "stainless steel" => entry(16.0, 0.35),
        "aluminum" | "aluminium" => entry(205.0, 0.10),
        "brass" => entry(110.0, 0.20),
        "copper" => entry(400.0, 0.15),
        "titanium" => entry(7.0, 0.30),
        "cast iron" => entry(55.0, 0.85),
        "abs plastic" | "abs" => entry(0.25, 0.90),
        "pla" => entry(0.13, 0.90),
        "nylon" => entry(0.25, 0.90),
        "polycarbonate" => entry(0.20, 0.90),
        "oak" => entry(0.17, 0.90),
        "glass" => entry(1.0, 0.92),
        _ => None,
    }
}
