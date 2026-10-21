//! A fastener chosen from the hole it goes through.
//!
//! The hole's diameter is matched back to a thread size through the same ISO 273 clearance table
//! a hole is drilled from, so the round trip closes on itself; the screw's length is washer +
//! grip + nut + two pitches, rounded up to the next listed ISO 4762 length, never interpolated.
//! Measuring the grip under the head belongs to the geometry side; this module takes the
//! measured hole and grip and answers the rest, with the drill sizes for the thread it found.
//!
//! Sources: ISO 261 (coarse pitch), ISO 273 (clearance holes, the fine/medium/coarse series as
//! H12/H13/H14), ISO 4762 (socket head cap screws: dk, k, listed lengths), DIN 974-1 series 1
//! (counterbores), ISO 15065 (90° countersinks), ISO 4032 (hex nuts), ISO 7089 (plain washers).
//! The tapping drill is D − P, the shop rule for metric coarse. A value the table cannot stand
//! behind is `None`, never a guess. Lengths in millimetres.

use serde::{Deserialize, Serialize};

/// Everything about drilling for one thread size (mm).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoreholeSize {
    pub thread: &'static str,
    pub diameter: f64,
    pub coarse_pitch: f64,
    pub clearance_fine: f64,
    pub clearance_medium: f64,
    pub clearance_coarse: f64,
    pub socket_head_diameter: f64,
    pub socket_head_height: f64,
    pub counterbore_diameter: f64,
    pub counterbore_depth: f64,
    /// `None` where the standard lists nothing.
    pub countersink_diameter: Option<f64>,
}

const fn size(
    thread: &'static str,
    d: f64,
    p: f64,
    fine: f64,
    medium: f64,
    coarse: f64,
    dk: f64,
    k: f64,
    cb_d: f64,
    cb_depth: f64,
    csk: f64,
) -> BoreholeSize {
    BoreholeSize {
        thread,
        diameter: d,
        coarse_pitch: p,
        clearance_fine: fine,
        clearance_medium: medium,
        clearance_coarse: coarse,
        socket_head_diameter: dk,
        socket_head_height: k,
        counterbore_diameter: cb_d,
        counterbore_depth: cb_depth,
        countersink_diameter: if csk > 0.0 { Some(csk) } else { None },
    }
}

/// The table, smallest first.
pub const BOREHOLES: &[BoreholeSize] = &[
    size("M1.6", 1.6, 0.35, 1.7, 1.8, 2.0, 3.0, 1.6, 3.5, 1.8, 3.8),
    size("M2", 2.0, 0.40, 2.2, 2.4, 2.6, 3.8, 2.0, 4.4, 2.2, 4.7),
    size("M2.5", 2.5, 0.45, 2.7, 2.9, 3.1, 4.5, 2.5, 5.0, 2.7, 5.8),
    size("M3", 3.0, 0.50, 3.2, 3.4, 3.6, 5.5, 3.0, 6.5, 3.4, 6.8),
    size("M4", 4.0, 0.70, 4.3, 4.5, 4.8, 7.0, 4.0, 8.0, 4.6, 9.1),
    size("M5", 5.0, 0.80, 5.3, 5.5, 5.8, 8.5, 5.0, 10.0, 5.7, 11.3),
    size("M6", 6.0, 1.00, 6.4, 6.6, 7.0, 10.0, 6.0, 11.0, 6.8, 13.5),
    size("M8", 8.0, 1.25, 8.4, 9.0, 10.0, 13.0, 8.0, 15.0, 9.0, 18.0),
    size("M10", 10.0, 1.50, 10.5, 11.0, 12.0, 16.0, 10.0, 18.0, 11.0, 22.5),
    size("M12", 12.0, 1.75, 13.0, 13.5, 14.5, 18.0, 12.0, 20.0, 13.0, 27.0),
    size("M14", 14.0, 2.00, 15.0, 15.5, 16.5, 21.0, 14.0, 24.0, 15.5, 0.0),
    size("M16", 16.0, 2.00, 17.0, 17.5, 18.5, 24.0, 16.0, 26.0, 17.5, 33.5),
    size("M20", 20.0, 2.50, 21.0, 22.0, 24.0, 30.0, 20.0, 33.0, 21.5, 40.0),
    size("M24", 24.0, 3.00, 25.0, 26.0, 28.0, 36.0, 24.0, 40.0, 25.5, 0.0),
    size("M30", 30.0, 3.50, 31.0, 33.0, 35.0, 45.0, 30.0, 48.0, 31.5, 0.0),
    size("M36", 36.0, 4.00, 37.0, 39.0, 42.0, 54.0, 36.0, 57.0, 37.5, 0.0),
];

/// By designation ("M6", case-insensitive).
pub fn find_borehole(thread: &str) -> Option<&'static BoreholeSize> {
    let wanted = thread.trim();
    BOREHOLES.iter().find(|s| s.thread.eq_ignore_ascii_case(wanted))
}

/// The tapping drill for metric coarse: D − P, about 77 % of full thread depth.
pub fn tapping_drill(size: &BoreholeSize) -> f64 {
    size.diameter - size.coarse_pitch
}

/// ISO 4032 hex nut: (size, across flats, height), mm.
const NUTS: [(&str, f64, f64); 7] =
    [("M3", 5.5, 2.4), ("M4", 7.0, 3.2), ("M5", 8.0, 4.7), ("M6", 10.0, 5.2), ("M8", 13.0, 6.8), ("M10", 16.0, 8.4), ("M12", 18.0, 10.8)];
/// ISO 7089 plain washer: (size, inner, outer, thickness), mm.
const WASHERS: [(&str, f64, f64, f64); 7] = [
    ("M3", 3.2, 7.0, 0.5),
    ("M4", 4.3, 9.0, 0.8),
    ("M5", 5.3, 10.0, 1.0),
    ("M6", 6.4, 12.0, 1.6),
    ("M8", 8.4, 16.0, 1.6),
    ("M10", 10.5, 20.0, 2.0),
    ("M12", 13.0, 24.0, 2.5),
];
/// ISO 4762's listed lengths (mm), smallest first.
pub const ISO_4762_LENGTHS: [f64; 30] = [
    3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0,
    130.0, 140.0, 150.0, 160.0, 180.0, 200.0,
];

/// ISO 4032 nut height (mm) for a size the catalogue carries.
pub fn nut_height(thread: &str) -> Option<f64> {
    NUTS.iter().find(|(s, _, _)| s.eq_ignore_ascii_case(thread.trim())).map(|(_, _, h)| *h)
}

/// ISO 7089 washer (outer diameter, thickness) (mm) for a size the catalogue carries.
pub fn washer(thread: &str) -> Option<(f64, f64)> {
    WASHERS.iter().find(|(s, _, _, _)| s.eq_ignore_ascii_case(thread.trim())).map(|(_, _, outer, t)| (*outer, *t))
}

/// Which thread a hole was drilled for.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThreadMatch {
    pub thread: String,
    /// "Fine" | "Medium" | "Coarse"
    pub series: String,
    /// The table's clearance it matched (mm).
    pub series_diameter_mm: f64,
}

/// Matches a hole diameter (mm) back to a thread through ISO 273, within 0.03 mm (the series are
/// never closer than 0.2). The error names the nearest entry.
pub fn match_clearance(hole_diameter_mm: f64) -> Result<ThreadMatch, String> {
    if !hole_diameter_mm.is_finite() || hole_diameter_mm <= 0.0 {
        return Err("a hole diameter must be a positive number of millimetres".into());
    }
    let mut nearest: Option<(f64, String)> = None;
    for size in BOREHOLES {
        for (name, value) in [("Fine", size.clearance_fine), ("Medium", size.clearance_medium), ("Coarse", size.clearance_coarse)] {
            let gap = (value - hole_diameter_mm).abs();
            if gap <= 0.03 {
                return Ok(ThreadMatch { thread: size.thread.to_string(), series: name.to_string(), series_diameter_mm: value });
            }
            if nearest.as_ref().is_none_or(|(g, _)| gap < *g) {
                nearest = Some((gap, format!("{} {name} {value:.2} mm", size.thread)));
            }
        }
    }
    let nearest = nearest.map(|(_, n)| n).unwrap_or_default();
    Err(format!(
        "a {hole_diameter_mm:.2} mm hole is no ISO 273 clearance for any metric size in the table (the nearest is {nearest}); the fastener is not chosen from a hole that was not drilled for one"
    ))
}

/// The screw for a measured stack.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FastenerChoice {
    /// "M6"
    pub size: String,
    /// The listed ISO 4762 length chosen (mm).
    pub length_mm: f64,
    /// washer + grip + nut + 2 pitches (mm).
    pub needed_mm: f64,
    pub washer_thickness_mm: f64,
    pub nut_height_mm: f64,
    pub pitch_mm: f64,
    pub washer: bool,
    pub nut: bool,
    pub description: String,
}

/// The screw for a hole of `hole_diameter_mm` through `grip_mm` of material. A through hole
/// without a nut gets the grip plus the washer and nothing else, which the description says.
pub fn choose_fastener(hole_diameter_mm: f64, grip_mm: f64, washer_wanted: bool, nut_wanted: bool) -> Result<(ThreadMatch, FastenerChoice), String> {
    if !grip_mm.is_finite() || grip_mm <= 0.0 || grip_mm > 10_000.0 {
        return Err("the grip (material under the head) must be a positive length in millimetres".into());
    }
    let matched = match_clearance(hole_diameter_mm)?;
    let bore = find_borehole(&matched.thread).ok_or_else(|| format!("no drill data for {}", matched.thread))?;
    // The screw catalogue (ISO 4762 M3..M12) is the washer catalogue's range.
    if washer(&matched.thread).is_none() {
        return Err(format!("no ISO 4762 screw in the catalogue for {}", matched.thread));
    }
    let washer_thickness = if washer_wanted { washer(&matched.thread).map(|(_, t)| t).unwrap_or(0.0) } else { 0.0 };
    let (nut_h, pitch) = if nut_wanted {
        (nut_height(&matched.thread).ok_or_else(|| format!("no ISO 4032 nut in the catalogue for {}", matched.thread))?, bore.coarse_pitch)
    } else {
        (0.0, 0.0)
    };
    let needed = washer_thickness + grip_mm + nut_h + 2.0 * pitch;
    let longest = ISO_4762_LENGTHS.last().copied().unwrap_or(0.0);
    let length = ISO_4762_LENGTHS
        .iter()
        .copied()
        .find(|listed| listed + 1e-9 >= needed)
        .ok_or_else(|| format!("the stack needs a {needed:.2} mm screw and ISO 4762 lists nothing past {longest:.0} mm"))?;
    let mut description = format!("ISO 4762 {} x {length:.0}: ", matched.thread);
    if washer_wanted {
        description.push_str(&format!("washer {washer_thickness:.2} + "));
    }
    description.push_str(&format!("grip {grip_mm:.2}"));
    if nut_wanted {
        description.push_str(&format!(" + nut {nut_h:.2} + 2 pitches {:.2}", 2.0 * pitch));
    }
    description.push_str(&format!(" = {needed:.2} mm, next listed length {length:.0}"));
    if !nut_wanted {
        description.push_str("; no nut: the screw ends flush with the far face, which only holds if that face is tapped");
    }
    Ok((
        matched.clone(),
        FastenerChoice {
            size: matched.thread,
            length_mm: length,
            needed_mm: needed,
            washer_thickness_mm: washer_thickness,
            nut_height_mm: nut_h,
            pitch_mm: pitch,
            washer: washer_wanted,
            nut: nut_wanted,
            description,
        },
    ))
}

#[cfg(test)]
#[path = "hole_stack_tests.rs"]
mod tests;
