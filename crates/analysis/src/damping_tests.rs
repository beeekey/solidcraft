use std::f64::consts::PI;

use super::{SuspensionDuty, check_suspension, loss_factor_of};

fn near(got: f64, want: f64, tol: f64) -> bool {
    (got - want).abs() <= tol
}

#[test]
fn bamboo_damps_what_aluminium_passes_straight_through() {
    let bamboo = loss_factor_of("Bamboo").map(|m| m.loss_factor).unwrap_or(0.0);
    let aluminium = loss_factor_of("Aluminium").map(|m| m.loss_factor).unwrap_or(0.0);
    assert!(bamboo > 10.0 * aluminium);
    assert!(loss_factor_of("Rubber (natural)").map(|m| m.loss_factor).unwrap_or(0.0) > bamboo);
    // The same frame member in the two materials: same mass and stiffness, so the same natural
    // frequency, and a completely different ring-down.
    let mut duty = SuspensionDuty {
        sprung_mass: 8.0,
        spring_rate: 8.0 * (2.0 * PI * 30.0_f64).powi(2),
        excitation_hertz: 60.0,
        loss_factor: bamboo,
        ..Default::default()
    };
    let in_bamboo = check_suspension(&duty);
    assert!(in_bamboo.valid, "{}", in_bamboo.advice);
    assert!(near(in_bamboo.natural_hertz, 30.0, 1e-9));
    assert!(near(in_bamboo.damping_ratio, bamboo / 2.0, 1e-12));
    duty.loss_factor = aluminium;
    let in_aluminium = check_suspension(&duty);
    assert!(near(in_aluminium.natural_hertz, in_bamboo.natural_hertz, 1e-9));
    assert!(in_bamboo.cycles_to_half_amplitude < 10.0);
    assert!(in_aluminium.cycles_to_half_amplitude > 100.0);
    assert!(near(in_aluminium.cycles_to_half_amplitude / in_bamboo.cycles_to_half_amplitude, bamboo / aluminium, bamboo / aluminium * 0.01));
}

#[test]
fn a_suspension_only_isolates_above_root_two_times_its_own_frequency() {
    let mut duty = SuspensionDuty { sprung_mass: 25.0, spring_rate: 12000.0, damping_coefficient: 400.0, ..Default::default() };
    let tuned = check_suspension(&duty);
    assert!(tuned.valid, "{}", tuned.advice);
    assert!(near(tuned.natural_hertz, (12000.0_f64 / 25.0).sqrt() / (2.0 * PI), 1e-12));
    assert!(near(tuned.damping_ratio, 400.0 / (2.0 * (12000.0_f64 * 25.0).sqrt()), 1e-12));
    assert!(near(tuned.isolation_above_hertz, 2.0_f64.sqrt() * tuned.natural_hertz, 1e-12));
    assert!(near(tuned.static_deflection, 25.0 * 9.80665 / 12000.0, 1e-12));
    // At the crossing, transmissibility is one however it is damped.
    for damping in [50.0, 400.0, 2000.0] {
        let at = SuspensionDuty { damping_coefficient: damping, excitation_hertz: tuned.isolation_above_hertz, ..duty.clone() };
        assert!(near(check_suspension(&at).transmissibility, 1.0, 1e-9));
    }
    // Below it, more gets through than went in; above it, less.
    duty.excitation_hertz = tuned.natural_hertz;
    assert!(check_suspension(&duty).transmissibility > 1.0);
    duty.excitation_hertz = 5.0 * tuned.natural_hertz;
    let isolated = check_suspension(&duty);
    assert!(isolated.transmissibility < 0.2);
    assert!(isolated.advice.contains("isolates"));
    // A trailer with no spring at all is not a suspension and is told so.
    let rigid = SuspensionDuty { sprung_mass: 25.0, ..Default::default() };
    assert!(!check_suspension(&rigid).valid);
    let nan = SuspensionDuty { sprung_mass: f64::NAN, spring_rate: 1.0, ..Default::default() };
    assert!(!check_suspension(&nan).valid);
}
