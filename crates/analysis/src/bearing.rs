//! How long a rolling bearing lasts, and whether it survives standing still.
//!
//! A rolling bearing does not wear out: it fails by fatigue of the raceway, and fatigue is
//! statistical, so the answer is never "it lasts X". It is L10, the life nine bearings in ten
//! exceed. ISO 281 gives it as (C/P)^p million revolutions, p = 3 for balls and 10/3 for rollers,
//! C the basic dynamic rating and P the equivalent load.
//!
//! Standing still is a different failure with a different rating: a stationary bearing under load
//! brinells its raceway, and what governs is the basic static rating C0. Both numbers come off the
//! manufacturer's page. A bearing sold without them, as most cartridge headset bearings are, gets
//! no life figure and says why.

/// A bearing by designation: ISO 15 boundary dimensions (m) and catalogue ratings (N) for an
/// ordinary open steel bearing of that size.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BearingSize {
    pub designation: &'static str,
    pub bore: f64,
    pub outer: f64,
    pub width: f64,
    /// C; 0 when nobody publishes one.
    pub dynamic_rating: f64,
    /// C0; 0 likewise.
    pub static_rating: f64,
    /// Contact angle (degrees); 0 for a deep-groove bearing.
    pub contact_angle: f64,
    pub used_for: &'static str,
}

const fn size(designation: &'static str, bore: f64, outer: f64, width: f64, c: f64, c0: f64, angle: f64, used_for: &'static str) -> BearingSize {
    BearingSize { designation, bore, outer, width, dynamic_rating: c, static_rating: c0, contact_angle: angle, used_for }
}

/// The bearings the application knows.
pub const SIZES: [BearingSize; 21] = [
    size("625", 0.005, 0.016, 0.005, 1140.0, 380.0, 0.0, "small idlers and jockey wheels"),
    size("608", 0.008, 0.022, 0.007, 3450.0, 1370.0, 0.0, "skate and scooter wheels, small trailer wheels"),
    size("6000", 0.010, 0.026, 0.008, 4550.0, 1960.0, 0.0, "small pivots"),
    size("6001", 0.012, 0.028, 0.008, 5100.0, 2360.0, 0.0, "bicycle hubs, trailer wheels, suspension pivots"),
    size("6002", 0.015, 0.032, 0.009, 5850.0, 2850.0, 0.0, "hubs and pivots"),
    size("6003", 0.017, 0.035, 0.010, 6370.0, 3250.0, 0.0, "trailer wheels, loaded pivots"),
    size("6200", 0.010, 0.030, 0.009, 5070.0, 2240.0, 0.0, "general"),
    size("6201", 0.012, 0.032, 0.010, 6890.0, 3100.0, 0.0, "general"),
    size("6202", 0.015, 0.035, 0.011, 7800.0, 3750.0, 0.0, "general"),
    size("6204", 0.020, 0.047, 0.014, 12700.0, 6550.0, 0.0, "general"),
    // Thin section: what a bicycle hub or a bottom bracket is built from.
    size("6802", 0.015, 0.024, 0.005, 1730.0, 1080.0, 0.0, "rear hubs and freehub bodies"),
    size("6803", 0.017, 0.026, 0.005, 1900.0, 1270.0, 0.0, "thin-section hubs"),
    size("6805", 0.025, 0.037, 0.007, 4360.0, 2900.0, 0.0, "bottom brackets for a 24 mm axle"),
    size("6806", 0.030, 0.042, 0.007, 4620.0, 3350.0, 0.0, "bottom brackets for a 30 mm axle"),
    size("6900", 0.010, 0.022, 0.006, 2700.0, 1270.0, 0.0, "small pivots"),
    size("6901", 0.012, 0.024, 0.006, 2890.0, 1460.0, 0.0, "pivots and idlers"),
    size("6902", 0.015, 0.028, 0.007, 4360.0, 2240.0, 0.0, "front hubs, suspension pivots"),
    size("6903", 0.017, 0.030, 0.007, 4620.0, 2550.0, 0.0, "hubs and pivots"),
    // Cartridge headset bearings are a bicycle-industry standard by dimension only: nobody
    // publishes C and C0 for them, so neither does this.
    size("Headset 41.8", 0.03015, 0.0418, 0.008, 0.0, 0.0, 45.0, "the ordinary 1-1/8 inch headset, upper and lower"),
    size("Headset 41.0", 0.03015, 0.041, 0.0065, 0.0, 0.0, 45.0, "the shallower 1-1/8 inch cartridge"),
    size("Headset 51.8", 0.040, 0.0518, 0.008, 0.0, 0.0, 45.0, "the 1.5 inch lower bearing of a tapered steerer"),
];

/// The bearing with this designation (case-insensitive), if the table has it.
pub fn find_bearing(designation: &str) -> Option<BearingSize> {
    let wanted = designation.trim();
    SIZES.iter().copied().find(|s| s.designation.eq_ignore_ascii_case(wanted))
}

/// What the bearing is asked to do.
#[derive(Debug, Clone, PartialEq)]
pub struct BearingDuty {
    /// C (N), from the catalogue.
    pub dynamic_rating: f64,
    /// C0 (N).
    pub static_rating: f64,
    pub radial_load: f64,
    pub axial_load: f64,
    /// Deep-groove ball bearings take axial load through the same raceway, so the two combine into
    /// one equivalent load with the ISO factors for the ordinary case (X = 0.56, Y about 1.5). A
    /// heavily thrust-loaded bearing needs the catalogue's own e and Y.
    pub radial_factor: f64,
    pub axial_factor: f64,
    /// Changes the exponent to 10/3.
    pub roller_not_ball: bool,
    /// 0 reports revolutions, not hours.
    pub revolutions_per_minute: f64,
    /// For a wheel bearing: how far one revolution carries the vehicle, so the life can be
    /// reported in kilometres. 0 switches it off.
    pub metres_per_revolution: f64,
}

impl Default for BearingDuty {
    fn default() -> Self {
        BearingDuty {
            dynamic_rating: 0.0,
            static_rating: 0.0,
            radial_load: 0.0,
            axial_load: 0.0,
            radial_factor: 0.56,
            axial_factor: 1.5,
            roller_not_ball: false,
            revolutions_per_minute: 0.0,
            metres_per_revolution: 0.0,
        }
    }
}

/// L10 life and the static check.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BearingLife {
    pub valid: bool,
    pub advice: String,
    /// P (N).
    pub equivalent_load: f64,
    /// C / P.
    pub load_ratio: f64,
    /// L10.
    pub million_revolutions: f64,
    /// 0 when no speed was given.
    pub hours: f64,
    /// 0 when no wheel size was given.
    pub kilometres: f64,
    /// C0 / P0; 0 when no static rating was given.
    pub static_safety: f64,
}

/// L10 life and the static check. `valid` is false with a reason when the ratings are missing,
/// which is the honest answer for a bearing sold without them.
pub fn bearing_life(duty: &BearingDuty) -> BearingLife {
    let mut result = BearingLife::default();
    if !(duty.dynamic_rating > 0.0) || !duty.dynamic_rating.is_finite() {
        result.advice = "this bearing has no published dynamic rating, so it has no L10 life. Cartridge headset bearings are usually sold without one: they are chosen by size and by seal quality, and they fail from water and grit rather than from fatigue.".into();
        return result;
    }
    let radial = duty.radial_load.max(0.0);
    let axial = duty.axial_load.max(0.0);
    if !(radial > 0.0) && !(axial > 0.0) {
        result.advice = "a bearing with no load on it has no life to calculate".into();
        return result;
    }
    result.valid = true;

    // Pure radial when the thrust is small, the combination when it is not: the larger of the two
    // is what ISO 281 amounts to for a deep-groove ball bearing.
    let combined = duty.radial_factor * radial + duty.axial_factor * axial;
    result.equivalent_load = radial.max(combined);
    result.load_ratio = duty.dynamic_rating / result.equivalent_load;
    let exponent = if duty.roller_not_ball { 10.0 / 3.0 } else { 3.0 };
    result.million_revolutions = result.load_ratio.powf(exponent);
    if duty.revolutions_per_minute > 0.0 {
        result.hours = result.million_revolutions * 1e6 / (60.0 * duty.revolutions_per_minute);
    }
    if duty.metres_per_revolution > 0.0 {
        result.kilometres = result.million_revolutions * 1e6 * duty.metres_per_revolution / 1000.0;
    }
    if duty.static_rating > 0.0 {
        // The static equivalent load: radial plus about half the thrust, never less than the
        // radial alone.
        let static_load = radial.max(0.6 * radial + 0.5 * axial);
        result.static_safety = duty.static_rating / static_load;
    }

    let mut advice = String::new();
    if result.load_ratio < 4.0 {
        advice.push_str("The load is more than a quarter of the bearing's dynamic rating, which is a hard life for a small bearing: L10 falls with the CUBE of the load, so a bearing one size up lasts several times longer for a few grams. ");
    }
    if result.static_safety > 0.0 && result.static_safety < 2.0 {
        advice.push_str(&format!(
            "The static safety is only {:.1}. A bearing standing still under this load is brinelling its raceway -- the damage is done parked, not riding, and it shows up as roughness afterwards. ",
            result.static_safety
        ));
    }
    if result.million_revolutions > 10000.0 {
        advice.push_str("L10 comes out enormous, which means fatigue is not what will finish this bearing off. Lightly loaded bearings die of water, grit and brinelling from being parked loaded, and none of those are in this number: choose it for its seal, not for this figure. ");
    }
    if axial > 0.5 * radial {
        advice.push_str("The thrust is over half the radial load. A deep-groove ball bearing will take that, but the X and Y factors above are the ordinary-case ones: for a joint that is mostly thrust -- a headset -- use an angular-contact bearing and its own factors. ");
    }
    result.advice = advice.trim_end().to_string();
    result
}

#[cfg(test)]
#[path = "bearing_tests.rs"]
mod tests;
