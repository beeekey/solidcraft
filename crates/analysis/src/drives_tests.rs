//! The two-pulley cases have exact closed forms, so the solver is checked against them; the
//! multi-wheel cases through the invariants a real belt must satisfy.

use std::f64::consts::PI;

use super::*;

fn open_belt_exact(centres: f64, r1: f64, r2: f64) -> f64 {
    let alpha = ((r2 - r1) / centres).asin();
    2.0 * (centres * centres - (r2 - r1) * (r2 - r1)).sqrt() + r1 * (PI - 2.0 * alpha) + r2 * (PI + 2.0 * alpha)
}

fn crossed_belt_exact(centres: f64, r1: f64, r2: f64) -> f64 {
    let beta = ((r1 + r2) / centres).asin();
    2.0 * (centres * centres - (r1 + r2) * (r1 + r2)).sqrt() + (r1 + r2) * (PI + 2.0 * beta)
}

fn wheel(x: f64, y: f64, radius: f64, side: i32) -> Wheel {
    Wheel { centre: Vec2::new(x, y), radius, side }
}

/// Every tangent point lies on its circle and the span meets the radius at a right angle.
fn expect_tangency(wheels: &[Wheel], path: &BeltPath) {
    assert_eq!(path.spans.len(), wheels.len());
    for i in 0..wheels.len() {
        let (a, b) = (wheels[i], wheels[(i + 1) % wheels.len()]);
        let span = path.spans[i];
        assert!(((span.from.x - a.centre.x).hypot(span.from.y - a.centre.y) - a.radius).abs() < 1e-12);
        assert!(((span.to.x - b.centre.x).hypot(span.to.y - b.centre.y) - b.radius).abs() < 1e-12);
        let (dx, dy) = (span.to.x - span.from.x, span.to.y - span.from.y);
        let length = dx.hypot(dy);
        assert!(length > 1e-9);
        for (w, point) in [(a, span.from), (b, span.to)] {
            let (rx, ry) = (point.x - w.centre.x, point.y - w.centre.y);
            assert!(((dx * rx + dy * ry) / length).abs() < 1e-9);
        }
    }
}

fn turning(wheels: &[Wheel], path: &BeltPath) -> f64 {
    wheels.iter().zip(&path.wraps).map(|(w, wrap)| w.side as f64 * wrap).sum()
}

#[test]
fn open_belt_matches_the_exact_closed_form() {
    let (centres, r1, r2) = (0.120, 0.020, 0.035);
    let wheels = [wheel(0.0, 0.0, r1, 1), wheel(centres, 0.0, r2, 1)];
    let path = belt_path(&wheels, false);
    assert!(path.valid, "{}", path.problem);
    assert!((path.length - open_belt_exact(centres, r1, r2)).abs() < 1e-12);
    expect_tangency(&wheels, &path);
    let alpha = ((r2 - r1) / centres).asin();
    assert!((path.wraps[0] - (PI - 2.0 * alpha)).abs() < 1e-12);
    assert!((path.wraps[1] - (PI + 2.0 * alpha)).abs() < 1e-12);
    assert!((turning(&wheels, &path) - 2.0 * PI).abs() < 1e-12);
}

#[test]
fn crossed_belt_matches_the_exact_closed_form() {
    let (centres, r1, r2) = (0.120, 0.020, 0.035);
    let wheels = [wheel(0.0, 0.0, r1, 1), wheel(centres, 0.0, r2, -1)];
    let refused = belt_path(&wheels, false);
    assert!(!refused.valid && refused.crossings == 1);
    let path = belt_path(&wheels, true);
    assert!(path.valid, "{}", path.problem);
    assert!((path.length - crossed_belt_exact(centres, r1, r2)).abs() < 1e-12);
    expect_tangency(&wheels, &path);
}

#[test]
fn equal_pulleys_give_parallel_spans_and_half_turn_wraps() {
    let (centres, radius) = (0.100, 0.025);
    let wheels = [wheel(0.0, 0.0, radius, 1), wheel(centres, 0.0, radius, 1)];
    let path = belt_path(&wheels, false);
    assert!(path.valid);
    assert!((path.length - (2.0 * centres + 2.0 * PI * radius)).abs() < 1e-12);
    assert!((path.wraps[0] - PI).abs() < 1e-12 && (path.wraps[1] - PI).abs() < 1e-12);
    assert!((path.spans[0].length - centres).abs() < 1e-12);
}

#[test]
fn four_wheels_with_an_inside_idler_stay_tangent_and_closed() {
    let wheels = [wheel(0.0, 0.0, 0.020, 1), wheel(0.150, 0.0, 0.035, 1), wheel(0.110, 0.060, 0.012, -1), wheel(0.040, 0.070, 0.015, 1)];
    let path = belt_path(&wheels, false);
    assert!(path.valid, "{}", path.problem);
    expect_tangency(&wheels, &path);
    assert!((turning(&wheels, &path) - 2.0 * PI).abs() < 1e-9);
    assert!(path.clearance > 0.0);
    let plain = belt_path(&[wheels[0], wheels[1], wheels[3]], false);
    assert!(plain.valid && path.length > plain.length);
}

#[test]
fn a_flipped_wrap_and_a_cutting_span_and_overlapping_wheels_are_rejected() {
    let flipped =
        belt_path(&[wheel(0.0, 0.0, 0.020, 1), wheel(0.150, 0.0, 0.035, 1), wheel(0.110, 0.072, 0.012, -1), wheel(0.040, 0.070, 0.015, 1)], false);
    assert!(!flipped.valid && flipped.crossings > 0 && flipped.problem.contains("through itself"));
    assert!(flipped.wraps[2] > 5.0);
    let cutting = belt_path(&[wheel(0.0, 0.0, 0.010, 1), wheel(0.100, 0.0, 0.010, 1), wheel(0.050, 0.015, 0.030, 1)], false);
    assert!(!cutting.valid && (cutting.clearance * 1e3 + 5.0).abs() < 0.01 && cutting.problem.contains("cut through"));
    let clear = belt_path(&[wheel(0.0, 0.0, 0.010, 1), wheel(0.100, 0.0, 0.010, 1), wheel(0.050, 0.040, 0.030, 1)], false);
    assert!(clear.valid && (clear.clearance * 1e3 - 20.0).abs() < 0.01);
    let overlapping = belt_path(&[wheel(0.0, 0.0, 0.050, 1), wheel(0.020, 0.0, 0.040, -1)], false);
    assert!(!overlapping.valid && overlapping.problem.contains("overlap"));
    assert!(!belt_path(&[wheel(0.0, 0.0, f64::NAN, 1), wheel(1.0, 0.0, 0.1, 1)], false).valid);
    assert!(!belt_path(&[wheel(0.0, 0.0, 0.1, 1)], false).valid);
}

#[test]
fn pulley_diameters_follow_the_standard_and_teeth_in_mesh_the_wrap() {
    let gt2 = belt_standard("GT2-2M").unwrap_or(&BELT_STANDARDS[0]);
    assert!((pitch_diameter(gt2, 20) * 1e3 - 12.7324).abs() < 1e-3);
    assert!((outside_diameter(gt2, 20) * 1e3 - (12.7324 - 0.508)).abs() < 1e-3);
    assert!(root_diameter(gt2, 20) < outside_diameter(gt2, 20));
    let htd = belt_standard("htd-5m").unwrap_or(&BELT_STANDARDS[0]);
    assert!((pitch_diameter(htd, 30) * 1e3 - 30.0 * 5.0 / PI).abs() < 1e-9);
    assert!(belt_standard("no such belt").is_none());
    assert_eq!(teeth_in_mesh(20, PI), 10);
    assert_eq!(teeth_in_mesh(20, PI / 2.0), 5);
    assert_eq!(teeth_in_mesh(20, 0.0), 0);
    assert!(cord_radius_on_back_idler(gt2, 0.010) > 0.010);
}

#[test]
fn the_tensioner_solves_for_a_real_standard_belt_and_reports_the_range_otherwise() {
    let mut wheels = vec![wheel(0.0, 0.0, 0.020, 1), wheel(0.150, 0.0, 0.035, 1), wheel(0.110, 0.060, 0.012, -1), wheel(0.040, 0.070, 0.015, 1)];
    let standard = belt_standard("HTD-5M").unwrap_or(&BELT_STANDARDS[0]);
    let probe = solve_tensioner(&wheels, 2, Vec2::new(0.0, -1.0), 0.0, 0.020);
    assert!(!probe.solved && probe.reachable_max > probe.reachable_min);
    let belts = reachable_belts(standard, probe.reachable_min, probe.reachable_max);
    assert!(!belts.is_empty());
    for choice in &belts {
        assert!((choice.length - choice.teeth as f64 * standard.pitch).abs() < 1e-12);
        assert!(choice.length >= probe.reachable_min - 1e-9 && choice.length <= probe.reachable_max + 1e-9);
    }
    let pick = belts[0];
    let solution = solve_tensioner(&wheels, 2, Vec2::new(0.0, -1.0), pick.length, 0.020);
    assert!(solution.solved, "{}", solution.problem);
    assert!((solution.length - pick.length).abs() < 1e-9);
    wheels[2].centre.y -= solution.travel;
    let path = belt_path(&wheels, false);
    assert!(path.valid && (path.length - pick.length).abs() < 1e-9);
    assert!((turning(&wheels, &path) - 2.0 * PI).abs() < 1e-9);
    let unreachable = solve_tensioner(&[wheel(0.0, 0.0, 0.020, 1), wheel(0.150, 0.0, 0.035, 1)], 1, Vec2::new(1.0, 0.0), 10.0, 0.020);
    assert!(!unreachable.solved && !unreachable.problem.is_empty() && unreachable.reachable_max > 0.0);
    assert!(!solve_tensioner(&wheels, 9, Vec2::new(1.0, 0.0), 1.0, 0.02).solved);
}

#[test]
fn moving_a_pulley_apart_lengthens_the_belt_monotonically() {
    let mut previous = 0.0;
    let mut centres = 0.080;
    while centres <= 0.200 {
        let path = belt_path(&[wheel(0.0, 0.0, 0.020, 1), wheel(centres, 0.0, 0.035, 1)], false);
        assert!(path.valid && path.length > previous);
        previous = path.length;
        centres += 0.010;
    }
}

#[test]
fn sprocket_diameters_and_chain_links_follow_iso_606() {
    let chain = chain_standard("08B").unwrap_or(&CHAIN_STANDARDS[0]);
    assert!((sprocket_pitch_diameter(chain, 17) - 0.0691158).abs() < 1e-6);
    assert!((sprocket_root_diameter(chain, 17) - (0.0691158 - 0.00851)).abs() < 1e-6);
    let tip = sprocket_tip_diameter(chain, 17);
    assert!(tip > 0.0691158 + 0.0127 * (1.0 - 1.6 / 17.0) - 0.00851 - 1e-9);
    assert!(tip < 0.0691158 + 1.25 * 0.0127 - 0.00851 + 1e-9);
    assert!((sprocket_tooth_width(chain) - 0.95 * 0.00775).abs() < 1e-12);
    assert!(chain_standard("no such chain").is_none());
    assert_eq!(chain_links(chain, 17, 34, 0.400), 90);
    let centre = chain_centre_distance(chain, 17, 34, 90);
    assert!(centre > 0.400);
    let (p, d) = (chain.pitch, 17.0 / (2.0 * PI));
    assert!((2.0 * centre / p + 25.5 + d * d * p / centre - 90.0).abs() < 1e-9);
    assert_eq!(chain_centre_distance(chain, 17, 34, 20), 0.0);
    assert_eq!(chain_links(chain, 17, 34, f64::NAN), 0);
}
