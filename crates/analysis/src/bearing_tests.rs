use std::f64::consts::PI;

use super::{BearingDuty, bearing_life, find_bearing};

fn near(got: f64, want: f64, tol: f64) -> bool {
    (got - want).abs() <= tol
}

#[test]
fn l10_follows_iso_281_and_the_static_check_its_rating() {
    // A 6001 (C 5100 N, C0 2360 N) under 500 N radial.
    let b = find_bearing("6001").unwrap_or_else(|| panic!("6001 in the table"));
    let mut duty = BearingDuty { dynamic_rating: b.dynamic_rating, static_rating: b.static_rating, radial_load: 500.0, ..Default::default() };
    let life = bearing_life(&duty);
    assert!(life.valid, "{}", life.advice);
    assert!(near(life.equivalent_load, 500.0, 1e-12));
    assert!(near(life.million_revolutions, (5100.0_f64 / 500.0).powi(3), 1e-9));
    assert!(near(life.static_safety, 2360.0 / 500.0, 1e-12));
    // Twice the load, an eighth of the life.
    duty.radial_load = 1000.0;
    assert!(near(bearing_life(&duty).million_revolutions, life.million_revolutions / 8.0, life.million_revolutions * 1e-12));
    // A roller bearing wears the load more slowly: the exponent is 10/3.
    duty.radial_load = 500.0;
    duty.roller_not_ball = true;
    assert!(near(bearing_life(&duty).million_revolutions, (5100.0_f64 / 500.0).powf(10.0 / 3.0), 1e-6));
    // In kilometres, the unit a wheel bearing is thought about in, and hours at a speed.
    duty.roller_not_ball = false;
    duty.metres_per_revolution = PI * 0.700;
    duty.revolutions_per_minute = 200.0;
    let rolling = bearing_life(&duty);
    assert!(near(rolling.kilometres, rolling.million_revolutions * 1e6 * PI * 0.700 / 1000.0, 1e-6));
    assert!(rolling.kilometres > 1000.0);
    assert!(near(rolling.hours, rolling.million_revolutions * 1e6 / (60.0 * 200.0), 1e-6));
}

#[test]
fn a_bearing_sold_without_ratings_gets_no_life_and_says_why() {
    let headset = find_bearing("Headset 41.8").unwrap_or_else(|| panic!("the headset is in the table"));
    let duty = BearingDuty { dynamic_rating: headset.dynamic_rating, radial_load: 500.0, ..Default::default() };
    let none = bearing_life(&duty);
    assert!(!none.valid);
    assert!(none.advice.contains("no published dynamic rating"));
    // And no load is no life either.
    let idle = BearingDuty { dynamic_rating: 5100.0, ..Default::default() };
    assert!(!bearing_life(&idle).valid);
    assert!(find_bearing("no such").is_none());
}

#[test]
fn the_advice_names_a_hard_life_and_a_thrust_heavy_duty() {
    let duty = BearingDuty { dynamic_rating: 1140.0, static_rating: 380.0, radial_load: 400.0, axial_load: 300.0, ..Default::default() };
    let life = bearing_life(&duty);
    assert!(life.valid);
    // X R + Y A = 0.56 · 400 + 1.5 · 300 = 674 N, more than the radial alone.
    assert!(near(life.equivalent_load, 674.0, 1e-9), "{}", life.equivalent_load);
    assert!(life.advice.contains("CUBE"), "{}", life.advice);
    assert!(life.advice.contains("thrust"), "{}", life.advice);
    assert!(life.advice.contains("brinelling"), "{}", life.advice);
}
