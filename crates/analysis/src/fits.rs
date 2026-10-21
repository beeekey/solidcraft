//! Limits and fits: what size to make a hole and its shaft so they go together the way you
//! meant, from ISO 286 rather than from memory, and what it then takes to put them together.
//!
//! A fit class is a letter and a grade, `H7` for a hole or `g6` for a shaft, and what it means
//! is two numbers each: how much bigger and smaller than nominal the part may be. The IT grades
//! are ISO 286-1's standard tolerances in 13 size steps to 500 mm (IT14 to IT18 are IT9 to IT13
//! times ten, which is a property of the standard); the fundamental deviations are tabulated per
//! shaft letter on that letter's own size steps, and the hole letters are derived by the
//! standard's own rule (EI = −es for A to H, ES = −ei for J onward, plus the delta term for K, M,
//! N at IT8 and finer and P onward at IT7 and finer). Where a size, grade or letter is outside
//! what the table can stand behind, the lookup refuses and says so rather than extrapolating a
//! number somebody would cut metal to.
//!
//! The press-fit check keeps one discipline: the force to assemble is set by the maximum
//! interference the tolerances allow, the torque it holds afterwards by the minimum. They are
//! different fits, and one number for both answers a question nobody asked.
//!
//! Lengths here are millimetres; stresses are pascals, forces newtons, torques newton-metres.

use serde::{Deserialize, Serialize};

/// What a toleranced size may actually be (mm).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Limits {
    pub nominal_mm: f64,
    /// Signed deviation from nominal (mm).
    pub lower_mm: f64,
    pub upper_mm: f64,
}

impl Limits {
    pub fn min_size(&self) -> f64 {
        self.nominal_mm + self.lower_mm
    }
    pub fn max_size(&self) -> f64 {
        self.nominal_mm + self.upper_mm
    }
    pub fn width(&self) -> f64 {
        self.upper_mm - self.lower_mm
    }
}

/// The limits a class puts on a size, with where they came from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LimitLookup {
    pub limits: Limits,
    /// "ISO 286: IT7 = 21 um over 18 to 30 mm; H gives EI = 0"
    pub source: String,
}

/// One size step and its standard tolerances IT1..IT18 (µm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GradeRow {
    pub over_mm: f64,
    pub upto_mm: f64,
    pub it_um: [f64; 18],
}

/// IT1..IT13 per size step (µm); IT14..IT18 follow as IT9..IT13 × 10.
const RAW_GRADES: [(f64, f64, [f64; 13]); 13] = [
    (0.0, 3.0, [0.8, 1.2, 2.0, 3.0, 4.0, 6.0, 10.0, 14.0, 25.0, 40.0, 60.0, 100.0, 140.0]),
    (3.0, 6.0, [1.0, 1.5, 2.5, 4.0, 5.0, 8.0, 12.0, 18.0, 30.0, 48.0, 75.0, 120.0, 180.0]),
    (6.0, 10.0, [1.0, 1.5, 2.5, 4.0, 6.0, 9.0, 15.0, 22.0, 36.0, 58.0, 90.0, 150.0, 220.0]),
    (10.0, 18.0, [1.2, 2.0, 3.0, 5.0, 8.0, 11.0, 18.0, 27.0, 43.0, 70.0, 110.0, 180.0, 270.0]),
    (18.0, 30.0, [1.5, 2.5, 4.0, 6.0, 9.0, 13.0, 21.0, 33.0, 52.0, 84.0, 130.0, 210.0, 330.0]),
    (30.0, 50.0, [1.5, 2.5, 4.0, 7.0, 11.0, 16.0, 25.0, 39.0, 62.0, 100.0, 160.0, 250.0, 390.0]),
    (50.0, 80.0, [2.0, 3.0, 5.0, 8.0, 13.0, 19.0, 30.0, 46.0, 74.0, 120.0, 190.0, 300.0, 460.0]),
    (80.0, 120.0, [2.5, 4.0, 6.0, 10.0, 15.0, 22.0, 35.0, 54.0, 87.0, 140.0, 220.0, 350.0, 540.0]),
    (120.0, 180.0, [3.5, 5.0, 8.0, 12.0, 18.0, 25.0, 40.0, 63.0, 100.0, 160.0, 250.0, 400.0, 630.0]),
    (180.0, 250.0, [4.5, 7.0, 10.0, 14.0, 20.0, 29.0, 46.0, 72.0, 115.0, 185.0, 290.0, 460.0, 720.0]),
    (250.0, 315.0, [6.0, 8.0, 12.0, 16.0, 23.0, 32.0, 52.0, 81.0, 130.0, 210.0, 320.0, 520.0, 810.0]),
    (315.0, 400.0, [7.0, 9.0, 13.0, 18.0, 25.0, 36.0, 57.0, 89.0, 140.0, 230.0, 360.0, 570.0, 890.0]),
    (400.0, 500.0, [8.0, 10.0, 15.0, 20.0, 27.0, 40.0, 63.0, 97.0, 155.0, 250.0, 400.0, 630.0, 970.0]),
];

/// The standard tolerance table, smallest size step first.
pub fn grade_rows() -> Vec<GradeRow> {
    RAW_GRADES
        .iter()
        .map(|(over, upto, it13)| {
            let mut it_um = [0.0; 18];
            it_um[..13].copy_from_slice(it13);
            for g in 13..18 {
                it_um[g] = it13[g - 5] * 10.0;
            }
            GradeRow { over_mm: *over, upto_mm: *upto, it_um }
        })
        .collect()
}

/// The size steps most letters follow (the grade table's).
const PLAIN_STEPS: [(f64, f64); 13] = [
    (0.0, 3.0),
    (3.0, 6.0),
    (6.0, 10.0),
    (10.0, 18.0),
    (18.0, 30.0),
    (30.0, 50.0),
    (50.0, 80.0),
    (80.0, 120.0),
    (120.0, 180.0),
    (180.0, 250.0),
    (250.0, 315.0),
    (315.0, 400.0),
    (400.0, 500.0),
];

/// r and s split finer above 50 mm.
const FINE_STEPS: [(f64, f64); 22] = [
    (0.0, 3.0),
    (3.0, 6.0),
    (6.0, 10.0),
    (10.0, 18.0),
    (18.0, 30.0),
    (30.0, 50.0),
    (50.0, 65.0),
    (65.0, 80.0),
    (80.0, 100.0),
    (100.0, 120.0),
    (120.0, 140.0),
    (140.0, 160.0),
    (160.0, 180.0),
    (180.0, 200.0),
    (200.0, 225.0),
    (225.0, 250.0),
    (250.0, 280.0),
    (280.0, 315.0),
    (315.0, 355.0),
    (355.0, 400.0),
    (400.0, 450.0),
    (450.0, 500.0),
];

/// One shaft letter's fundamental deviations (µm) on its own size steps. `upper` letters (a..h)
/// give es, the upper deviation, ≤ 0; the others give ei, the lower deviation, ≥ 0.
struct Letter {
    letter: &'static str,
    upper: bool,
    steps: &'static [(f64, f64)],
    um: &'static [f64],
}

const LETTERS: [Letter; 11] = [
    Letter {
        letter: "d",
        upper: true,
        steps: &PLAIN_STEPS,
        um: &[-20.0, -30.0, -40.0, -50.0, -65.0, -80.0, -100.0, -120.0, -145.0, -170.0, -190.0, -210.0, -230.0],
    },
    Letter {
        letter: "e",
        upper: true,
        steps: &PLAIN_STEPS,
        um: &[-14.0, -20.0, -25.0, -32.0, -40.0, -50.0, -60.0, -72.0, -85.0, -100.0, -110.0, -125.0, -135.0],
    },
    Letter {
        letter: "f",
        upper: true,
        steps: &PLAIN_STEPS,
        um: &[-6.0, -10.0, -13.0, -16.0, -20.0, -25.0, -30.0, -36.0, -43.0, -50.0, -56.0, -62.0, -68.0],
    },
    Letter {
        letter: "g",
        upper: true,
        steps: &PLAIN_STEPS,
        um: &[-2.0, -4.0, -5.0, -6.0, -7.0, -9.0, -10.0, -12.0, -14.0, -15.0, -17.0, -18.0, -20.0],
    },
    Letter { letter: "h", upper: true, steps: &PLAIN_STEPS, um: &[0.0; 13] },
    Letter { letter: "k", upper: false, steps: &PLAIN_STEPS, um: &[0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0, 5.0] },
    Letter { letter: "m", upper: false, steps: &PLAIN_STEPS, um: &[2.0, 4.0, 6.0, 7.0, 8.0, 9.0, 11.0, 13.0, 15.0, 17.0, 20.0, 21.0, 23.0] },
    Letter { letter: "n", upper: false, steps: &PLAIN_STEPS, um: &[4.0, 8.0, 10.0, 12.0, 15.0, 17.0, 20.0, 23.0, 27.0, 31.0, 34.0, 37.0, 40.0] },
    Letter { letter: "p", upper: false, steps: &PLAIN_STEPS, um: &[6.0, 12.0, 15.0, 18.0, 22.0, 26.0, 32.0, 37.0, 43.0, 50.0, 56.0, 62.0, 68.0] },
    Letter {
        letter: "r",
        upper: false,
        steps: &FINE_STEPS,
        um: &[10.0, 15.0, 19.0, 23.0, 28.0, 34.0, 41.0, 43.0, 51.0, 54.0, 63.0, 65.0, 68.0, 77.0, 80.0, 84.0, 94.0, 98.0, 108.0, 114.0, 126.0, 132.0],
    },
    Letter {
        letter: "s",
        upper: false,
        steps: &FINE_STEPS,
        um: &[
            14.0, 19.0, 23.0, 28.0, 35.0, 43.0, 53.0, 59.0, 71.0, 79.0, 92.0, 100.0, 108.0, 122.0, 130.0, 140.0, 158.0, 170.0, 190.0, 208.0, 232.0,
            252.0,
        ],
    },
];

fn letters_covered() -> String {
    LETTERS.iter().map(|l| l.letter).collect::<Vec<_>>().join(" ")
}

/// "H7" / "g6" / "js9" → (letter as written, grade). Case says hole from shaft, which is the
/// standard's own convention.
pub fn parse_fit_class(text: &str) -> Option<(String, u32)> {
    let text = text.trim();
    let letters: String = text.chars().take_while(|c| c.is_ascii_alphabetic()).collect();
    let digits = text.get(letters.len()..)?;
    if letters.is_empty() || digits.is_empty() || letters.len() > 3 || digits.len() > 3 || !digits.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    let grade = digits.parse::<u32>().ok()?;
    Some((letters, grade))
}

/// The standard tolerance IT`grade` at this size (mm), or why there is none.
pub fn standard_tolerance(nominal_mm: f64, grade: u32) -> Result<f64, String> {
    if !nominal_mm.is_finite() {
        return Err("the nominal size must be a number".into());
    }
    let row = grade_rows()
        .into_iter()
        .find(|r| nominal_mm > r.over_mm && nominal_mm <= r.upto_mm)
        .ok_or_else(|| {
            format!(
                "ISO 286 as held here covers 0 to 500 mm; {nominal_mm:.3} mm was asked for. Above 500 mm the deviations come from a different set of formulae and are not in this table."
            )
        })?;
    if !(1..=18).contains(&grade) {
        return Err(format!("the standard tolerance grades are IT1 to IT18; IT{grade} was asked for."));
    }
    row.it_um.get(grade as usize - 1).map(|um| um * 1e-3).ok_or_else(|| "grade out of table".to_string())
}

/// The limits a class puts on a size (mm).
pub fn fit_limits(nominal_mm: f64, fit_class: &str) -> Result<LimitLookup, String> {
    let (letter, grade) = parse_fit_class(fit_class)
        .ok_or_else(|| format!("'{fit_class}' is not a fit class: it needs a letter and a grade, like H7 for a hole or g6 for a shaft."))?;
    let it = standard_tolerance(nominal_mm, grade)?;
    let it_um = it * 1e3;
    let hole = letter.chars().next().is_some_and(|c| c.is_ascii_uppercase());
    let lower = letter.to_ascii_lowercase();
    let mut limits = Limits { nominal_mm, ..Default::default() };

    // js and JS are symmetric by definition rather than by table.
    if lower == "js" {
        limits.upper_mm = 0.5 * it;
        limits.lower_mm = -0.5 * it;
        return Ok(LimitLookup { limits, source: format!("ISO 286: IT{grade} = {it_um:.1} um, and js is +/- IT/2 by definition") });
    }
    if lower == "j" {
        return Err(format!(
            "j and J have their own irregular table and are rarely used; this table does not carry them. js (symmetric, +/- IT/2) is almost always what is wanted, and the available letters are: {} js",
            letters_covered()
        ));
    }
    let table = LETTERS.iter().find(|l| l.letter == lower).ok_or_else(|| {
        format!("'{letter}' is not a letter this table carries. It has: {} js (and their upper-case hole forms).", letters_covered())
    })?;
    let deviation_um = table
        .steps
        .iter()
        .zip(table.um)
        .find(|((over, upto), _)| nominal_mm > *over && nominal_mm <= *upto)
        .map(|(_, um)| *um)
        .ok_or_else(|| format!("'{letter}' has no entry at {nominal_mm:.3} mm; this table runs from 0 to 500 mm."))?;
    let fundamental = deviation_um * 1e-3;

    if !hole {
        if table.upper {
            limits.upper_mm = fundamental;
            limits.lower_mm = fundamental - it;
        } else {
            limits.lower_mm = fundamental;
            limits.upper_mm = fundamental + it;
        }
        let which = if table.upper { "es" } else { "ei" };
        return Ok(LimitLookup { limits, source: format!("ISO 286: IT{grade} = {it_um:.1} um, {lower} gives {which} = {deviation_um:.1} um") });
    }
    if table.upper {
        // A..H: EI = -es.
        limits.lower_mm = -fundamental;
        limits.upper_mm = -fundamental + it;
        return Ok(LimitLookup { limits, source: format!("ISO 286: IT{grade} = {it_um:.1} um, EI = -es({lower}) = {:.1} um", -deviation_um) });
    }
    // J..ZC: ES = -ei, plus delta where the standard says so.
    let delta_applies = if matches!(lower.as_str(), "k" | "m" | "n") { grade <= 8 } else { grade <= 7 };
    let delta = if delta_applies && grade >= 2 { it - standard_tolerance(nominal_mm, grade - 1)? } else { 0.0 };
    if lower == "n" && grade > 8 {
        return Err(format!(
            "ISO 286 gives N no fundamental deviation above IT8, so N{grade} is not a fit. That is the standard's own carve-out, not a gap in this table."
        ));
    }
    limits.upper_mm = -fundamental + delta;
    limits.lower_mm = limits.upper_mm - it;
    Ok(LimitLookup {
        limits,
        source: format!("ISO 286: IT{grade} = {it_um:.1} um, ES = -ei({lower}) + delta = {:.1} + {:.1} um", -deviation_um, delta * 1e3),
    })
}

/// Every class this table can answer for, for choice lists.
pub fn known_fit_classes() -> Vec<String> {
    let mut out = Vec::new();
    for letter in &LETTERS {
        for grade in 5..=11 {
            out.push(format!("{}{grade}", letter.letter));
            out.push(format!("{}{grade}", letter.letter.to_ascii_uppercase()));
        }
    }
    for grade in 5..=11 {
        out.push(format!("js{grade}"));
        out.push(format!("JS{grade}"));
    }
    out.sort();
    out
}

/// What a fit is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FitKind {
    Clearance,
    Transition,
    Interference,
}

impl FitKind {
    pub fn name(self) -> &'static str {
        match self {
            FitKind::Clearance => "clearance",
            FitKind::Transition => "transition",
            FitKind::Interference => "interference",
        }
    }
}

/// A material as the press-fit pressure needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressMaterial {
    pub young_modulus_pa: f64,
    pub poisson_ratio: f64,
    pub yield_pa: f64,
    /// 1/K
    pub thermal_expansion: f64,
}

/// Textbook constants for the common hub and shaft materials, by name (case-insensitive).
pub fn press_material(name: &str) -> Option<PressMaterial> {
    let m = |e, nu, y, a| Some(PressMaterial { young_modulus_pa: e, poisson_ratio: nu, yield_pa: y, thermal_expansion: a });
    match name.trim().to_ascii_lowercase().as_str() {
        "steel" => m(210e9, 0.3, 250e6, 12e-6),
        "stainless steel" | "stainless" => m(193e9, 0.29, 215e6, 17e-6),
        "aluminum" | "aluminium" => m(69e9, 0.33, 95e6, 23e-6),
        "aluminium 6061-t6" | "aluminum 6061-t6" => m(68.9e9, 0.33, 276e6, 23.6e-6),
        "brass" => m(100e9, 0.34, 200e6, 20e-6),
        "bronze" => m(110e9, 0.34, 250e6, 18e-6),
        "copper" => m(117e9, 0.34, 70e6, 16.5e-6),
        "titanium" => m(114e9, 0.34, 880e6, 8.6e-6),
        "cast iron" => m(110e9, 0.26, 250e6, 11e-6),
        _ => None,
    }
}

/// What a fit check is asked.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FitDuty {
    pub nominal_mm: f64,
    /// "H7", "g6"; or give measured size ranges instead.
    pub hole_class: String,
    pub shaft_class: String,
    pub hole_min_mm: f64,
    pub hole_max_mm: f64,
    pub shaft_min_mm: f64,
    pub shaft_max_mm: f64,
    /// What an interference fit needs before anything can be said about force. Left at zero,
    /// the pressure results are zero and the advice says which input was missing.
    pub engagement_length_mm: f64,
    pub hub_outer_diameter_mm: f64,
    /// 0 = solid shaft.
    pub shaft_bore_diameter_mm: f64,
    pub hub_material: String,
    pub shaft_material: String,
    pub friction: f64,
    /// Roughness smoothed off on first assembly, diametral (mm).
    pub smoothing_mm: f64,
}

impl Default for FitDuty {
    fn default() -> Self {
        FitDuty {
            nominal_mm: 0.0,
            hole_class: String::new(),
            shaft_class: String::new(),
            hole_min_mm: 0.0,
            hole_max_mm: 0.0,
            shaft_min_mm: 0.0,
            shaft_max_mm: 0.0,
            engagement_length_mm: 0.0,
            hub_outer_diameter_mm: 0.0,
            shaft_bore_diameter_mm: 0.0,
            hub_material: "Steel".into(),
            shaft_material: "Steel".into(),
            friction: 0.15,
            smoothing_mm: 0.0,
        }
    }
}

/// What the check found.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FitResult {
    pub verdict: String,
    pub advice: String,
    pub kind: FitKind,
    pub hole: Limits,
    pub shaft: Limits,
    /// mm, negative = interference.
    pub min_clearance_mm: f64,
    pub max_clearance_mm: f64,
    /// mm, after smoothing.
    pub min_interference_mm: f64,
    pub max_interference_mm: f64,
    /// Interference only, and zero when the geometry to compute them was not given.
    pub contact_pressure_pa: f64,
    pub assembly_force_n: f64,
    pub min_retained_torque_nm: f64,
    pub max_retained_torque_nm: f64,
    pub hub_hoop_stress_pa: f64,
    pub hub_safety: f64,
    pub shrink_temperature_rise_k: f64,
}

/// Whether a hole and a shaft go together the way you meant, and what it takes.
pub fn check_fit(duty: &FitDuty) -> Result<FitResult, String> {
    let sizes = [
        duty.nominal_mm,
        duty.hole_min_mm,
        duty.hole_max_mm,
        duty.shaft_min_mm,
        duty.shaft_max_mm,
        duty.engagement_length_mm,
        duty.hub_outer_diameter_mm,
        duty.shaft_bore_diameter_mm,
        duty.friction,
        duty.smoothing_mm,
    ];
    if sizes.iter().any(|x| !x.is_finite()) {
        return Err("every size of a fit must be a number".into());
    }
    let by_class = !duty.hole_class.is_empty() && !duty.shaft_class.is_empty();
    let by_size = duty.hole_max_mm > duty.hole_min_mm && duty.shaft_max_mm > duty.shaft_min_mm;
    if !by_class && !by_size {
        return Err("a fit needs either a pair of classes (hole \"H7\", shaft \"g6\") or a pair of measured size ranges. Neither was given.".into());
    }
    let (hole, shaft) = if by_class {
        if !(duty.nominal_mm > 0.0) {
            return Err("a fit class means nothing without a nominal size to apply it to; none was given.".into());
        }
        let hole = fit_limits(duty.nominal_mm, &duty.hole_class).map_err(|e| format!("the hole class was refused: {e}"))?;
        let shaft = fit_limits(duty.nominal_mm, &duty.shaft_class).map_err(|e| format!("the shaft class was refused: {e}"))?;
        (hole.limits, shaft.limits)
    } else {
        let mid = |a: f64, b: f64| 0.5 * (a + b);
        (
            Limits {
                nominal_mm: mid(duty.hole_min_mm, duty.hole_max_mm),
                lower_mm: duty.hole_min_mm - mid(duty.hole_min_mm, duty.hole_max_mm),
                upper_mm: duty.hole_max_mm - mid(duty.hole_min_mm, duty.hole_max_mm),
            },
            Limits {
                nominal_mm: mid(duty.shaft_min_mm, duty.shaft_max_mm),
                lower_mm: duty.shaft_min_mm - mid(duty.shaft_min_mm, duty.shaft_max_mm),
                upper_mm: duty.shaft_max_mm - mid(duty.shaft_min_mm, duty.shaft_max_mm),
            },
        )
    };
    let min_clearance = hole.min_size() - shaft.max_size();
    let max_clearance = hole.max_size() - shaft.min_size();
    let kind = if min_clearance >= 0.0 {
        FitKind::Clearance
    } else if max_clearance <= 0.0 {
        FitKind::Interference
    } else {
        FitKind::Transition
    };
    let smoothing = duty.smoothing_mm.max(0.0);
    let max_interference = (-min_clearance - smoothing).max(0.0);
    let min_interference = (-max_clearance - smoothing).max(0.0);
    let mut result = FitResult {
        verdict: format!("{} fit: from {min_clearance:.3} to {max_clearance:.3} mm of clearance", kind.name()),
        advice: String::new(),
        kind,
        hole,
        shaft,
        min_clearance_mm: min_clearance,
        max_clearance_mm: max_clearance,
        min_interference_mm: min_interference,
        max_interference_mm: max_interference,
        contact_pressure_pa: 0.0,
        assembly_force_n: 0.0,
        min_retained_torque_nm: 0.0,
        max_retained_torque_nm: 0.0,
        hub_hoop_stress_pa: 0.0,
        hub_safety: 0.0,
        shrink_temperature_rise_k: 0.0,
    };
    if kind == FitKind::Clearance {
        result.advice = "this fit does not grip: it slides, and the assembly force and retained torque are zero rather than small. Locate it some other way -- a shoulder, a key, a pin or a screw.".into();
        return Ok(result);
    }
    // Everything past here needs the geometry the pressure depends on. In metres.
    let d = (if hole.nominal_mm > 0.0 { hole.nominal_mm } else { 0.5 * (hole.min_size() + shaft.max_size()) }) * 1e-3;
    let length = duty.engagement_length_mm * 1e-3;
    let outer = duty.hub_outer_diameter_mm * 1e-3;
    let bore = duty.shaft_bore_diameter_mm.max(0.0) * 1e-3;
    if !(length > 0.0) || !(outer > d) {
        let missing = if !(length > 0.0) && !(outer > d) {
            "neither the engagement length nor the hub's outer diameter was given."
        } else if !(length > 0.0) {
            "the engagement length was not given."
        } else {
            "the hub's outer diameter was not given, or is not bigger than the bore."
        };
        result.advice = format!(
            "the pressure this fit develops depends on how far the hub reaches round the shaft and how long the joint is, and {missing} The limits above stand; the force and torque are not computed."
        );
        return Ok(result);
    }
    let (Some(hub), Some(pin)) = (press_material(&duty.hub_material), press_material(&duty.shaft_material)) else {
        let unknown = if press_material(&duty.hub_material).is_none() { &duty.hub_material } else { &duty.shaft_material };
        result.advice = format!("the pressure needs both materials' stiffness, and {unknown} is not in the material table. The limits above stand.");
        return Ok(result);
    };
    // Lamé, for a hub of outer diameter Do round a shaft of bore di:
    //   p = δ / [ d ( (1/Eo)((Do²+d²)/(Do²-d²) + νo) + (1/Ei)((d²+di²)/(d²-di²) - νi) ) ]
    let hub_term = (1.0 / hub.young_modulus_pa) * ((outer * outer + d * d) / (outer * outer - d * d) + hub.poisson_ratio);
    let shaft_term = if d * d - bore * bore > 1e-18 {
        (1.0 / pin.young_modulus_pa) * ((d * d + bore * bore) / (d * d - bore * bore) - pin.poisson_ratio)
    } else {
        0.0
    };
    let compliance = d * (hub_term + shaft_term);
    let pressure_for = |interference_mm: f64| if compliance > 1e-30 { interference_mm * 1e-3 / compliance } else { 0.0 };
    let friction = duty.friction.max(0.0);
    result.contact_pressure_pa = pressure_for(max_interference);
    let contact_area = core::f64::consts::PI * d * length;
    result.assembly_force_n = friction * result.contact_pressure_pa * contact_area;
    result.max_retained_torque_nm = result.assembly_force_n * d / 2.0;
    result.min_retained_torque_nm = friction * pressure_for(min_interference) * contact_area * d / 2.0;
    result.hub_hoop_stress_pa = result.contact_pressure_pa * (outer * outer + d * d) / (outer * outer - d * d);
    result.hub_safety = if result.hub_hoop_stress_pa > 1e-9 { hub.yield_pa / result.hub_hoop_stress_pa } else { 0.0 };
    if hub.thermal_expansion > 1e-12 {
        result.shrink_temperature_rise_k = (max_interference * 1e-3 + 0.0001 * d) / (hub.thermal_expansion * d);
    }
    let mut advice = format!(
        "pressed at the tightest the tolerances allow ({max_interference:.3} mm) it takes {:.0} N; at the loosest ({min_interference:.3} mm) it holds {:.0} N.m. Those are different fits and the two numbers must not be swapped.",
        result.assembly_force_n, result.min_retained_torque_nm
    );
    if result.hub_safety > 0.0 && result.hub_safety < 1.0 {
        advice.push_str(&format!(
            " THE HUB YIELDS: hoop stress {:.0} MPa against {:.0} MPa. Make the hub thicker, or loosen the fit.",
            result.hub_hoop_stress_pa * 1e-6,
            hub.yield_pa * 1e-6
        ));
    }
    if result.shrink_temperature_rise_k > 0.0 {
        advice.push_str(&format!(" Heating the hub {:.0} K would let it drop on instead of being pressed.", result.shrink_temperature_rise_k));
    }
    result.advice = advice;
    Ok(result)
}

#[cfg(test)]
#[path = "fits_tests.rs"]
mod tests;
