//! What an untoleranced dimension means: ISO 2768-1's general tolerances.
//!
//! A drawing note, "General tolerances ISO 2768-m", says what every dimension without a
//! tolerance of its own is allowed to be. There are three tables, not one: linear dimensions,
//! external radii and chamfer heights, and angles, where the angular table is entered by the
//! length of the shorter side. Where the standard prints a dash (below 0.5 mm in every class,
//! and 0.5 to 3 mm in the very coarse class) the lookup refuses and says the dimension must be
//! toleranced individually, rather than returning a zero somebody would read as "exact".

use serde::{Deserialize, Serialize};

/// The four classes of ISO 2768-1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GeneralClass {
    Fine,
    Medium,
    Coarse,
    VeryCoarse,
}

impl GeneralClass {
    /// "f" / "m" / "c" / "v", and the forms a drawing note uses: "ISO 2768-m", "ISO 2768-mK".
    /// `None` when it is none of them, so a class nobody recognises does not silently become
    /// the middle one.
    pub fn parse(text: &str) -> Option<GeneralClass> {
        let text = text.trim();
        let letter = match text.rfind('-') {
            Some(dash) => text.get(dash + 1..).and_then(|rest| rest.chars().next()),
            None => text.chars().next(),
        }?;
        match letter {
            'f' | 'F' => Some(GeneralClass::Fine),
            'm' | 'M' => Some(GeneralClass::Medium),
            'c' | 'C' => Some(GeneralClass::Coarse),
            'v' | 'V' => Some(GeneralClass::VeryCoarse),
            _ => None,
        }
    }

    pub fn letter(self) -> &'static str {
        match self {
            GeneralClass::Fine => "f",
            GeneralClass::Medium => "m",
            GeneralClass::Coarse => "c",
            GeneralClass::VeryCoarse => "v",
        }
    }

    fn column(self) -> usize {
        match self {
            GeneralClass::Fine => 0,
            GeneralClass::Medium => 1,
            GeneralClass::Coarse => 2,
            GeneralClass::VeryCoarse => 3,
        }
    }
}

/// A general tolerance: ± `plus_minus` (mm, or degrees for the angular table), and where it
/// came from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeneralTolerance {
    pub plus_minus: f64,
    pub source: String,
}

/// A size band of a table: f, m, c, v. A negative entry is the standard's dash.
struct Band {
    over_mm: f64,
    upto_mm: f64,
    plus_minus: [f64; 4],
}

/// ISO 2768-1 table 1, linear dimensions, ± mm.
const LINEAR: [Band; 7] = [
    Band { over_mm: 0.5, upto_mm: 3.0, plus_minus: [0.05, 0.1, 0.2, -1.0] },
    Band { over_mm: 3.0, upto_mm: 6.0, plus_minus: [0.05, 0.1, 0.3, 0.5] },
    Band { over_mm: 6.0, upto_mm: 30.0, plus_minus: [0.1, 0.2, 0.5, 1.0] },
    Band { over_mm: 30.0, upto_mm: 120.0, plus_minus: [0.15, 0.3, 0.8, 1.5] },
    Band { over_mm: 120.0, upto_mm: 400.0, plus_minus: [0.2, 0.5, 1.2, 2.5] },
    Band { over_mm: 400.0, upto_mm: 1000.0, plus_minus: [0.3, 0.8, 2.0, 4.0] },
    Band { over_mm: 1000.0, upto_mm: 2000.0, plus_minus: [0.5, 1.2, 3.0, 6.0] },
];

/// ISO 2768-1 table 2, external radii and chamfer heights, ± mm. Fine and medium share a column,
/// as do coarse and very coarse.
const RADIUS: [Band; 3] = [
    Band { over_mm: 0.5, upto_mm: 3.0, plus_minus: [0.2, 0.2, 0.4, 0.4] },
    Band { over_mm: 3.0, upto_mm: 6.0, plus_minus: [0.5, 0.5, 1.0, 1.0] },
    Band { over_mm: 6.0, upto_mm: 1e9, plus_minus: [1.0, 1.0, 2.0, 2.0] },
];

/// ISO 2768-1 table 3, angles by the length of the shorter side, ± degrees.
const ANGULAR: [Band; 5] = [
    Band { over_mm: 0.0, upto_mm: 10.0, plus_minus: [1.0, 1.0, 1.5, 3.0] },
    Band { over_mm: 10.0, upto_mm: 50.0, plus_minus: [0.5, 0.5, 1.0, 2.0] },
    Band { over_mm: 50.0, upto_mm: 120.0, plus_minus: [20.0 / 60.0, 20.0 / 60.0, 0.5, 1.0] },
    Band { over_mm: 120.0, upto_mm: 400.0, plus_minus: [10.0 / 60.0, 10.0 / 60.0, 15.0 / 60.0, 0.5] },
    Band { over_mm: 400.0, upto_mm: 1e9, plus_minus: [5.0 / 60.0, 5.0 / 60.0, 10.0 / 60.0, 20.0 / 60.0] },
];

fn lookup(bands: &[Band], nominal_mm: f64, class: GeneralClass, table: &str, what: &str) -> Result<GeneralTolerance, String> {
    if !nominal_mm.is_finite() {
        return Err(format!("ISO 2768-1 {table} needs a size that is a number; {what} was not given one."));
    }
    for band in bands {
        if nominal_mm <= band.over_mm || nominal_mm > band.upto_mm {
            continue;
        }
        let value = band.plus_minus[class.column()];
        if value < 0.0 {
            return Err(format!(
                "ISO 2768-1 {table} prints a dash for {what} from {:.1} to {:.1} mm in class {}: there is no general tolerance there, and a dimension that size must be toleranced individually.",
                band.over_mm,
                band.upto_mm,
                class.letter()
            ));
        }
        return Ok(GeneralTolerance {
            plus_minus: value,
            source: format!("ISO 2768-1 {table}, {:.1} to {:.1} mm, class {}", band.over_mm, band.upto_mm, class.letter()),
        });
    }
    let first = bands.first().map(|b| b.over_mm).unwrap_or(0.0);
    let last = bands.last().map(|b| b.upto_mm).unwrap_or(0.0);
    let mut refusal = format!("ISO 2768-1 {table} covers {what} from {first:.1} to ");
    if last > 1e8 {
        refusal.push_str("any size");
    } else {
        refusal.push_str(&format!("{last:.1} mm"));
    }
    refusal.push_str(&format!("; {nominal_mm:.3} mm was asked for"));
    if nominal_mm <= first {
        refusal.push_str(". Below that the standard gives no general tolerance at all, and the dimension must be toleranced individually");
    }
    refusal.push('.');
    Err(refusal)
}

/// ISO 2768-1 table 1: linear dimensions (± mm).
pub fn general_linear(nominal_mm: f64, class: GeneralClass) -> Result<GeneralTolerance, String> {
    lookup(&LINEAR, nominal_mm, class, "table 1", "linear dimensions")
}

/// ISO 2768-1 table 2: external radii and chamfer heights (± mm). Not the same numbers as
/// table 1: a 1 mm chamfer is ±0.2 in class m where a 1 mm length is ±0.1.
pub fn general_radius_or_chamfer(nominal_mm: f64, class: GeneralClass) -> Result<GeneralTolerance, String> {
    lookup(&RADIUS, nominal_mm, class, "table 2", "external radii and chamfer heights")
}

/// ISO 2768-1 table 3: angles, entered by the length of the shorter side (± degrees).
pub fn general_angular(shorter_side_mm: f64, class: GeneralClass) -> Result<GeneralTolerance, String> {
    lookup(&ANGULAR, shorter_side_mm, class, "table 3", "the shorter side of an angle")
}

#[cfg(test)]
#[path = "general_tolerances_tests.rs"]
mod tests;
