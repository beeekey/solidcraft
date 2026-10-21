//! Belt and chain drives: pulley and sprocket geometry from the belt and chain standards, the
//! taut-belt path around any number of wheels, a tensioner solve, and the standard belts that
//! fit. Pure geometry and arithmetic in metres, so the feature layer only turns it into shapes.

use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

/// Tooth form of a belt standard; the pulley profile follows the belt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ToothForm {
    /// GT2/GT3: rounded, deep, the modern positioning belt.
    Curvilinear,
    /// HTD: circular arc.
    Circular,
    /// T / AT / MXL / XL / L / H: straight flanks.
    Trapezoidal,
    /// A friction V-belt: no teeth at all.
    Vee,
}

/// One belt standard. Lengths are metres. `pitch_line_differential` (PLD) is the distance from
/// the bottom of the belt tooth to the tensile cord: the pulley's outside diameter is the pitch
/// diameter minus twice that.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeltStandard {
    pub name: &'static str,
    pub pitch: f64,
    pub pitch_line_differential: f64,
    /// Pulley groove depth.
    pub tooth_depth: f64,
    pub belt_thickness: f64,
    pub tooth: ToothForm,
}

/// Every belt standard this build knows, in a stable order. Pitches are exact by definition;
/// the pitch line differentials come from the manufacturers' tables.
pub const BELT_STANDARDS: &[BeltStandard] = &[
    BeltStandard {
        name: "GT2-2M",
        pitch: 0.002,
        pitch_line_differential: 0.000254,
        tooth_depth: 0.00076,
        belt_thickness: 0.00152,
        tooth: ToothForm::Curvilinear,
    },
    BeltStandard {
        name: "GT2-3M",
        pitch: 0.003,
        pitch_line_differential: 0.000381,
        tooth_depth: 0.00114,
        belt_thickness: 0.00241,
        tooth: ToothForm::Curvilinear,
    },
    BeltStandard {
        name: "GT2-5M",
        pitch: 0.005,
        pitch_line_differential: 0.0005715,
        tooth_depth: 0.00191,
        belt_thickness: 0.00381,
        tooth: ToothForm::Curvilinear,
    },
    BeltStandard {
        name: "HTD-3M",
        pitch: 0.003,
        pitch_line_differential: 0.000381,
        tooth_depth: 0.00117,
        belt_thickness: 0.0024,
        tooth: ToothForm::Circular,
    },
    BeltStandard {
        name: "HTD-5M",
        pitch: 0.005,
        pitch_line_differential: 0.0005715,
        tooth_depth: 0.00191,
        belt_thickness: 0.0036,
        tooth: ToothForm::Circular,
    },
    BeltStandard {
        name: "HTD-8M",
        pitch: 0.008,
        pitch_line_differential: 0.0006858,
        tooth_depth: 0.00320,
        belt_thickness: 0.0054,
        tooth: ToothForm::Circular,
    },
    BeltStandard {
        name: "HTD-14M",
        pitch: 0.014,
        pitch_line_differential: 0.0012700,
        tooth_depth: 0.00560,
        belt_thickness: 0.0100,
        tooth: ToothForm::Circular,
    },
    BeltStandard {
        name: "T2.5",
        pitch: 0.0025,
        pitch_line_differential: 0.00025,
        tooth_depth: 0.00070,
        belt_thickness: 0.0013,
        tooth: ToothForm::Trapezoidal,
    },
    BeltStandard {
        name: "T5",
        pitch: 0.005,
        pitch_line_differential: 0.0005,
        tooth_depth: 0.00120,
        belt_thickness: 0.0022,
        tooth: ToothForm::Trapezoidal,
    },
    BeltStandard {
        name: "T10",
        pitch: 0.010,
        pitch_line_differential: 0.0010,
        tooth_depth: 0.00250,
        belt_thickness: 0.0045,
        tooth: ToothForm::Trapezoidal,
    },
    BeltStandard {
        name: "AT5",
        pitch: 0.005,
        pitch_line_differential: 0.0005,
        tooth_depth: 0.00120,
        belt_thickness: 0.0027,
        tooth: ToothForm::Trapezoidal,
    },
    BeltStandard {
        name: "AT10",
        pitch: 0.010,
        pitch_line_differential: 0.0010,
        tooth_depth: 0.00250,
        belt_thickness: 0.0050,
        tooth: ToothForm::Trapezoidal,
    },
    BeltStandard {
        name: "MXL",
        pitch: 0.0020320,
        pitch_line_differential: 0.000254,
        tooth_depth: 0.00051,
        belt_thickness: 0.00114,
        tooth: ToothForm::Trapezoidal,
    },
    BeltStandard {
        name: "XL",
        pitch: 0.0050800,
        pitch_line_differential: 0.000254,
        tooth_depth: 0.00127,
        belt_thickness: 0.00229,
        tooth: ToothForm::Trapezoidal,
    },
    BeltStandard {
        name: "L",
        pitch: 0.0095250,
        pitch_line_differential: 0.000381,
        tooth_depth: 0.00191,
        belt_thickness: 0.00381,
        tooth: ToothForm::Trapezoidal,
    },
    BeltStandard {
        name: "H",
        pitch: 0.0127000,
        pitch_line_differential: 0.000686,
        tooth_depth: 0.00229,
        belt_thickness: 0.00411,
        tooth: ToothForm::Trapezoidal,
    },
];

/// Lookup by name ("GT2-3M", "HTD-5M", "T5", …), case-insensitive.
pub fn belt_standard(name: &str) -> Option<&'static BeltStandard> {
    BELT_STANDARDS.iter().find(|s| s.name.eq_ignore_ascii_case(name.trim()))
}

/// Pitch diameter of a toothed pulley: n·p / π.
pub fn pitch_diameter(standard: &BeltStandard, teeth: usize) -> f64 {
    teeth as f64 * standard.pitch / PI
}

/// Outside (tip) diameter: the pitch diameter less twice the PLD.
pub fn outside_diameter(standard: &BeltStandard, teeth: usize) -> f64 {
    pitch_diameter(standard, teeth) - 2.0 * standard.pitch_line_differential
}

/// Root diameter of the groove: the tip diameter less twice the tooth depth.
pub fn root_diameter(standard: &BeltStandard, teeth: usize) -> f64 {
    outside_diameter(standard, teeth) - 2.0 * standard.tooth_depth
}

/// Teeth engaged over a wrap angle (radians), rounded down.
pub fn teeth_in_mesh(pulley_teeth: usize, wrap: f64) -> usize {
    if pulley_teeth == 0 || !(wrap > 0.0) {
        return 0;
    }
    (pulley_teeth as f64 * wrap / (2.0 * PI)).floor().max(0.0) as usize
}

/// The cord radius of a belt bent around a plain roller: the belt lies on it with its back, so
/// the cord sits a belt thickness out from the roller surface, less the PLD.
pub fn cord_radius_on_back_idler(standard: &BeltStandard, roller_radius: f64) -> f64 {
    roller_radius + standard.belt_thickness - standard.pitch_line_differential
}

/// The cord radius of a belt on a toothed pulley: its pitch radius.
pub fn cord_radius_on_pulley(standard: &BeltStandard, teeth: usize) -> f64 {
    pitch_diameter(standard, teeth) / 2.0
}

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Vec2 {
        Vec2 { x, y }
    }
}

fn distance(a: Vec2, b: Vec2) -> f64 {
    (b.x - a.x).hypot(b.y - a.y)
}

fn distance_to_segment(point: Vec2, a: Vec2, b: Vec2) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let length2 = dx * dx + dy * dy;
    if length2 < 1e-18 {
        return distance(point, a);
    }
    let t = (((point.x - a.x) * dx + (point.y - a.y) * dy) / length2).clamp(0.0, 1.0);
    distance(point, Vec2::new(a.x + dx * t, a.y + dy * t))
}

/// Do the two segments properly cross (sharing an endpoint does not count)?
fn segments_cross(a1: Vec2, a2: Vec2, b1: Vec2, b2: Vec2) -> bool {
    let orientation = |p: Vec2, q: Vec2, r: Vec2| -> i32 {
        let value = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
        if value.abs() < 1e-12 {
            0
        } else if value > 0.0 {
            1
        } else {
            -1
        }
    };
    orientation(a1, a2, b1) * orientation(a1, a2, b2) < 0 && orientation(b1, b2, a1) * orientation(b1, b2, a2) < 0
}

/// A wheel the belt runs over. `side` is +1 when the belt turns counter-clockwise around it and
/// −1 when clockwise (an idler pressed into the loop from outside). `radius` is the radius the
/// belt's cord follows.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Wheel {
    pub centre: Vec2,
    pub radius: f64,
    pub side: i32,
}

/// One straight run of belt between two wheels.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Span {
    pub from: Vec2,
    pub to: Vec2,
    pub length: f64,
}

/// The computed loop. `valid` is false when the numbers describe something that is not a belt;
/// `problem` then says what, and the length must not be used.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BeltPath {
    /// `spans[i]` leaves wheel i and arrives at wheel i+1.
    pub spans: Vec<Span>,
    /// Radians of contact on each wheel, unsigned.
    pub wraps: Vec<f64>,
    pub length: f64,
    /// Smallest gap from a span to a wheel it misses.
    pub clearance: f64,
    /// Times the belt runs through itself.
    pub crossings: usize,
    pub valid: bool,
    pub problem: String,
}

/// The taut path over the wheels in the given order. A belt cannot pass through itself: a wheel
/// the taut belt would not reach makes the loop take the long way round it, and that always shows
/// as the incoming and outgoing runs crossing, so the crossing count is the gate. `allow_crossing`
/// permits the classic crossed two-pulley flat-belt drive.
pub fn belt_path(wheels: &[Wheel], allow_crossing: bool) -> BeltPath {
    let mut path = BeltPath::default();
    let count = wheels.len();
    if count < 2 {
        path.problem = "a belt needs at least two wheels".into();
        return path;
    }
    for wheel in wheels {
        if !(wheel.radius > 0.0) || !wheel.radius.is_finite() || !wheel.centre.x.is_finite() || !wheel.centre.y.is_finite() {
            path.problem = "every wheel needs a positive pitch radius".into();
            return path;
        }
        if wheel.side != 1 && wheel.side != -1 {
            path.problem = "wrap side must be +1 or -1".into();
            return path;
        }
    }
    for (i, a) in wheels.iter().enumerate() {
        for b in &wheels[i + 1..] {
            if distance(a.centre, b.centre) < a.radius + b.radius - 1e-9 {
                path.problem = "two wheels overlap: no belt runs between them".into();
                return path;
            }
        }
    }
    // Tangents: signed radii make the four cases one formula. The outward radius at a contact is
    // the travel direction turned by −side·90°; closing the span perpendicular to travel gives
    // sin(θ − φ) = (R₂ − R₁) / d with R = side · radius.
    path.spans = vec![Span::default(); count];
    for i in 0..count {
        let (a, b) = (wheels[i], wheels[(i + 1) % count]);
        let (dx, dy) = (b.centre.x - a.centre.x, b.centre.y - a.centre.y);
        let d = dx.hypot(dy);
        if d < 1e-12 {
            path.problem = "two wheels share a centre".into();
            return path;
        }
        let ratio = (b.side as f64 * b.radius - a.side as f64 * a.radius) / d;
        if ratio.abs() > 1.0 {
            path.problem = "no common tangent: the wheels overlap for this wrap".into();
            return path;
        }
        let theta = dy.atan2(dx);
        let phi = theta - ratio.asin();
        let normal_a = phi - a.side as f64 * PI / 2.0;
        let normal_b = phi - b.side as f64 * PI / 2.0;
        let from = Vec2::new(a.centre.x + a.radius * normal_a.cos(), a.centre.y + a.radius * normal_a.sin());
        let to = Vec2::new(b.centre.x + b.radius * normal_b.cos(), b.centre.y + b.radius * normal_b.sin());
        path.spans[i] = Span { from, to, length: distance(from, to) };
    }
    // Wraps: from where the belt arrives to where it leaves, in the wheel's own direction.
    path.wraps = vec![0.0; count];
    let mut total = 0.0;
    for (i, wheel) in wheels.iter().enumerate() {
        let arrive = path.spans[(i + count - 1) % count].to;
        let depart = path.spans[i].from;
        let angle_in = (arrive.y - wheel.centre.y).atan2(arrive.x - wheel.centre.x);
        let angle_out = (depart.y - wheel.centre.y).atan2(depart.x - wheel.centre.x);
        let mut sweep = (angle_out - angle_in) * wheel.side as f64;
        while sweep < 0.0 {
            sweep += 2.0 * PI;
        }
        while sweep >= 2.0 * PI {
            sweep -= 2.0 * PI;
        }
        path.wraps[i] = sweep;
        total += wheel.radius * sweep;
    }
    total += path.spans.iter().map(|s| s.length).sum::<f64>();
    path.length = total;
    for i in 0..count {
        for j in i + 1..count {
            if segments_cross(path.spans[i].from, path.spans[i].to, path.spans[j].from, path.spans[j].to) {
                path.crossings += 1;
            }
        }
    }
    if path.crossings > 0 && !allow_crossing {
        path.problem = "the belt would pass through itself: a wheel is placed where the taut belt cannot reach it (or the drive is crossed, which only a flat belt can do)".into();
        return path;
    }
    // No span may pass through a wheel it does not touch.
    path.clearance = f64::MAX;
    for i in 0..count {
        for (j, wheel) in wheels.iter().enumerate() {
            if j == i || j == (i + 1) % count {
                continue;
            }
            let gap = distance_to_segment(wheel.centre, path.spans[i].from, path.spans[i].to) - wheel.radius;
            path.clearance = path.clearance.min(gap);
        }
    }
    if count == 2 {
        path.clearance = 0.0;
    }
    if path.clearance < 0.0 {
        path.problem = "the belt would cut through a wheel it does not wrap".into();
        return path;
    }
    path.valid = true;
    path
}

/// Result of sliding one wheel to make the belt a chosen length.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TensionerSolution {
    pub solved: bool,
    /// Metres along `direction` from the start pose.
    pub travel: f64,
    pub length: f64,
    /// The shortest and longest valid belts on this travel.
    pub reachable_min: f64,
    pub reachable_max: f64,
    pub problem: String,
}

/// Slides wheel `index` along `direction` until the loop measures `target_length`, staying in
/// the physically valid band; out of reach, the reachable range is reported instead.
pub fn solve_tensioner(wheels: &[Wheel], index: usize, direction: Vec2, target_length: f64, travel_limit: f64) -> TensionerSolution {
    let mut solution = TensionerSolution::default();
    let mut wheels = wheels.to_vec();
    let Some(start) = wheels.get(index).map(|w| w.centre) else {
        solution.problem = "no such wheel".into();
        return solution;
    };
    let norm = direction.x.hypot(direction.y);
    if norm < 1e-12 || !norm.is_finite() {
        solution.problem = "the tensioner needs a travel direction".into();
        return solution;
    }
    let unit = Vec2::new(direction.x / norm, direction.y / norm);
    let mut length_at = |travel: f64| -> Option<f64> {
        wheels[index].centre = Vec2::new(start.x + unit.x * travel, start.y + unit.y * travel);
        let path = belt_path(&wheels, false);
        path.valid.then_some(path.length)
    };
    const SAMPLES: usize = 400;
    let mut valid: Vec<(f64, f64)> = Vec::new();
    for step in 0..=SAMPLES {
        let travel = -travel_limit + 2.0 * travel_limit * step as f64 / SAMPLES as f64;
        if let Some(length) = length_at(travel) {
            valid.push((travel, length));
        }
    }
    let Some(first) = valid.first() else {
        solution.problem = "no valid tensioner position on this travel".into();
        return solution;
    };
    solution.reachable_min = valid.iter().map(|v| v.1).fold(first.1, f64::min);
    solution.reachable_max = valid.iter().map(|v| v.1).fold(first.1, f64::max);
    // Bracket the target between neighbouring valid samples, then bisect inside that band.
    let mut bracket = None;
    for pair in valid.windows(2) {
        let (a, b) = (pair[0].1 - target_length, pair[1].1 - target_length);
        if a * b <= 0.0 {
            bracket = Some((pair[0].0, pair[1].0));
            break;
        }
    }
    let Some((mut low, mut high)) = bracket else {
        solution.problem = "that belt is not reachable on this travel".into();
        return solution;
    };
    let mut low_length = length_at(low).unwrap_or(target_length);
    let mut middle = low;
    for _ in 0..200 {
        middle = (low + high) / 2.0;
        let Some(length) = length_at(middle) else {
            high = middle;
            continue;
        };
        if (length - target_length).abs() < 1e-12 {
            break;
        }
        if (low_length - target_length) * (length - target_length) <= 0.0 {
            high = middle;
        } else {
            low = middle;
            low_length = length;
        }
    }
    let Some(achieved) = length_at(middle) else {
        solution.problem = "the solved position is not a valid belt path".into();
        return solution;
    };
    solution.solved = true;
    solution.travel = middle;
    solution.length = achieved;
    solution
}

/// A standard belt that fits a drive: tooth count and its exact length.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeltChoice {
    pub teeth: usize,
    pub length: f64,
}

/// Standard belts (integer tooth counts) whose length lies within the reachable band, shortest
/// first (at most 10 000 of them).
pub fn reachable_belts(standard: &BeltStandard, reachable_min: f64, reachable_max: f64) -> Vec<BeltChoice> {
    let mut choices = Vec::new();
    if !(standard.pitch > 0.0) || reachable_max < reachable_min || !reachable_min.is_finite() || !reachable_max.is_finite() {
        return choices;
    }
    let first = (reachable_min / standard.pitch - 1e-9).ceil().max(1.0) as usize;
    let last = (reachable_max / standard.pitch + 1e-9).floor().max(0.0) as usize;
    for teeth in first..=last.min(first + 10_000) {
        choices.push(BeltChoice { teeth, length: teeth as f64 * standard.pitch });
    }
    choices
}

// --- roller chains (ISO 606) --------------------------------------------------------------------

/// One roller chain. Lengths in metres.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ChainStandard {
    pub name: &'static str,
    pub pitch: f64,
    pub roller_diameter: f64,
    /// Between the inner plates: the sprocket tooth fits here.
    pub inner_width: f64,
    /// The link plates' depth.
    pub plate_height: f64,
}

const fn chain(name: &'static str, p: f64, d1: f64, b1: f64, h: f64) -> ChainStandard {
    ChainStandard { name, pitch: p / 1000.0, roller_diameter: d1 / 1000.0, inner_width: b1 / 1000.0, plate_height: h / 1000.0 }
}

/// ISO 606 B series and the ANSI chains, from the standard's dimension tables.
pub const CHAIN_STANDARDS: [ChainStandard; 10] = [
    chain("05B", 8.0, 5.0, 3.0, 7.1),
    chain("06B", 9.525, 6.35, 5.72, 8.2),
    chain("08B", 12.7, 8.51, 7.75, 11.8),
    chain("10B", 15.875, 10.16, 9.65, 14.7),
    chain("12B", 19.05, 12.07, 11.68, 16.0),
    chain("ANSI 25", 6.35, 3.30, 3.18, 6.0),
    chain("ANSI 35", 9.525, 5.08, 4.77, 9.0),
    chain("ANSI 40", 12.7, 7.95, 7.85, 12.0),
    chain("ANSI 50", 15.875, 10.16, 9.40, 15.0),
    chain("ANSI 60", 19.05, 11.91, 12.57, 18.0),
];

pub fn chain_standard(name: &str) -> Option<&'static ChainStandard> {
    CHAIN_STANDARDS.iter().find(|c| c.name.eq_ignore_ascii_case(name.trim()))
}

/// The circle the roller centres sit on: p / sin(180° / z).
pub fn sprocket_pitch_diameter(chain: &ChainStandard, teeth: usize) -> f64 {
    chain.pitch / (PI / teeth.max(3) as f64).sin()
}

/// ISO 606's root diameter: the pitch diameter less a roller.
pub fn sprocket_root_diameter(chain: &ChainStandard, teeth: usize) -> f64 {
    sprocket_pitch_diameter(chain, teeth) - chain.roller_diameter
}

/// The tip diameter, between ISO 606's minimum and maximum.
pub fn sprocket_tip_diameter(chain: &ChainStandard, teeth: usize) -> f64 {
    let d = sprocket_pitch_diameter(chain, teeth);
    let least = d + chain.pitch * (1.0 - 1.6 / teeth.max(3) as f64) - chain.roller_diameter;
    let most = d + 1.25 * chain.pitch - chain.roller_diameter;
    (least + most) / 2.0
}

/// The tooth's width across the sprocket (ISO 606: 0.95 of the inner width up to 12.7 mm pitch,
/// 0.93 above).
pub fn sprocket_tooth_width(chain: &ChainStandard) -> f64 {
    (if chain.pitch <= 0.0127 + 1e-9 { 0.95 } else { 0.93 }) * chain.inner_width
}

/// Links a chain needs over two sprockets `centre_distance` apart, rounded up to an even number.
pub fn chain_links(chain: &ChainStandard, teeth1: usize, teeth2: usize, centre_distance: f64) -> usize {
    if !(centre_distance > 0.0) || !centre_distance.is_finite() {
        return 0;
    }
    let (p, c) = (chain.pitch, centre_distance);
    let d = (teeth2 as f64 - teeth1 as f64) / (2.0 * PI);
    let links = 2.0 * c / p + (teeth1 + teeth2) as f64 / 2.0 + d * d * p / c;
    let mut whole = (links - 1e-9).ceil().max(0.0) as usize;
    if !whole.is_multiple_of(2) {
        whole += 1;
    }
    whole
}

/// The centre distance at which `links` links fit exactly; 0 when no distance does.
pub fn chain_centre_distance(chain: &ChainStandard, teeth1: usize, teeth2: usize, links: usize) -> f64 {
    let s = (teeth1 + teeth2) as f64 / 2.0;
    let d = (teeth2 as f64 - teeth1 as f64) / (2.0 * PI);
    let b = links as f64 - s;
    let discriminant = b * b - 8.0 * d * d;
    if b <= 0.0 || discriminant < 0.0 {
        return 0.0;
    }
    chain.pitch * (b + discriminant.sqrt()) / 4.0
}

#[cfg(test)]
#[path = "drives_tests.rs"]
mod tests;
