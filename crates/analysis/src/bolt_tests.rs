//! A bolted joint: two springs in parallel, and everything follows from that. Every number is
//! checked against a published one (ISO 898-1 proof strengths, the stress-area formula, the
//! textbook stiffness ratio for a steel joint) or against an identity the physics cannot break.

use super::{BoltedJoint, check_bolted_joint, grade_named, thread_stress_area};

fn near(got: f64, want: f64, tol: f64) -> bool {
    (got - want).abs() <= tol
}

fn area_mm2(size: &str) -> f64 {
    thread_stress_area(size).unwrap_or(0.0) * 1e6
}

#[test]
fn the_stress_area_matches_the_published_table() {
    // ISO metric coarse, mm²: M6 20.1, M8 36.6, M10 58.0, M12 84.3, M16 157, M20 245.
    assert!(near(area_mm2("M6"), 20.1, 0.2), "{}", area_mm2("M6"));
    assert!(near(area_mm2("M8"), 36.6, 0.2));
    assert!(near(area_mm2("M10"), 58.0, 0.3));
    assert!(near(area_mm2("M12"), 84.3, 0.4));
    assert!(near(area_mm2("M16"), 157.0, 1.0));
    assert!(near(area_mm2("M20"), 245.0, 1.5));
    // A size with no pitch in the table is refused, not interpolated.
    assert_eq!(thread_stress_area("M7.5"), None);
    assert_eq!(thread_stress_area("nonsense"), None);
}

#[test]
fn the_property_classes_carry_their_iso_strengths() {
    let g = |n: &str| grade_named(n).map(|g| g.proof_strength).unwrap_or(0.0);
    assert!(near(g("8.8"), 600e6, 1e6));
    assert!(near(g("10.9"), 830e6, 1e6));
    assert!(near(g("12.9"), 970e6, 1e6));
    // The class name says it: 8.8 is 800 MPa ultimate, 0.8 of that as yield.
    assert!(near(grade_named("8.8").map(|g| g.ultimate_strength).unwrap_or(0.0), 800e6, 1e6));
    assert!(near(grade_named("10.9").map(|g| g.ultimate_strength).unwrap_or(0.0), 1040e6, 1e6));
    assert!(grade_named("no such class").is_none());
}

#[test]
fn an_m10_joint_lands_where_the_handbook_puts_it() {
    let joint = BoltedJoint { size: "M10".into(), grade: "8.8".into(), clamped_length: 0.030, ..Default::default() };
    let r = check_bolted_joint(&joint);
    assert!(r.solved, "{}", r.verdict);
    // 0.75 · 58 mm² · 600 MPa = 26.1 kN, the standard preload target.
    assert!(near(r.preload, 26100.0, 400.0), "{}", r.preload);
    // T = 0.2 F d = 0.2 · 26100 · 0.010 = 52 N·m, as a torque chart says for an M10 8.8.
    assert!(near(r.tightening_torque, 52.0, 2.0), "{}", r.tightening_torque);
    // The clamped parts are several times stiffer than the bolt in a normal steel joint; the
    // textbook range for C is about 0.15 to 0.30.
    assert!(r.load_factor > 0.10 && r.load_factor < 0.35, "{}", r.load_factor);
    assert!(r.clamped_stiffness > 2.0 * r.bolt_stiffness);
    // Untouched by an external load, the bolt sits at 75 % of proof.
    assert!(near(r.safety_against_proof, 1.0 / 0.75, 0.02));
}

#[test]
fn what_the_bolt_gains_and_the_clamp_loses_add_up_to_the_load() {
    for external in [0.0, 2000.0, 8000.0, 20000.0] {
        let joint = BoltedJoint { size: "M12".into(), grade: "10.9".into(), clamped_length: 0.025, external_load: external, ..Default::default() };
        let r = check_bolted_joint(&joint);
        assert!(r.solved, "{}", r.verdict);
        let gained = r.bolt_load - r.preload;
        let lost = r.preload - r.residual_clamp;
        assert!(near(gained + lost, external, 1e-6 * external.max(1.0)), "bolt +{gained} N, clamp -{lost} N, load {external} N");
        if external > 0.0 {
            assert!(near(gained / external, r.load_factor, 1e-9));
        }
    }
}

#[test]
fn the_joint_separates_exactly_where_the_clamp_runs_out() {
    let mut joint = BoltedJoint { size: "M8".into(), grade: "8.8".into(), clamped_length: 0.020, ..Default::default() };
    let idle = check_bolted_joint(&joint);
    assert!(idle.solved);
    joint.external_load = idle.separation_load * 0.99;
    let held = check_bolted_joint(&joint);
    assert!(!held.separates, "{}", held.advice);
    assert!(held.residual_clamp > 0.0);
    joint.external_load = idle.separation_load * 1.01;
    let gone = check_bolted_joint(&joint);
    assert!(gone.separates);
    assert!(gone.residual_clamp < 0.0);
    assert!(gone.advice.contains("SEPARATES"), "{}", gone.advice);
    // More preload holds more load, in direct proportion.
    joint.preload = 2.0 * idle.preload;
    joint.external_load = 0.0;
    let harder = check_bolted_joint(&joint);
    assert!(near(harder.separation_load, 2.0 * idle.separation_load, 0.01 * idle.separation_load));
}

#[test]
fn sideways_load_is_carried_by_friction_and_says_so_when_it_is_not() {
    let mut joint =
        BoltedJoint { size: "M10".into(), grade: "8.8".into(), clamped_length: 0.030, face_friction: 0.15, shear_load: 1000.0, ..Default::default() };
    let holds = check_bolted_joint(&joint);
    assert!(holds.solved);
    assert!(near(holds.slip_load, 0.15 * holds.residual_clamp, 1.0));
    assert!(!holds.slips, "{}", holds.advice);
    joint.shear_load = 8000.0; // more than 0.15 · 26 kN
    let slips = check_bolted_joint(&joint);
    assert!(slips.slips);
    assert!(slips.advice.contains("SLIPS"), "{}", slips.advice);
    // Friction goes with the clamp: nearly separated, almost nothing resists sliding.
    joint.shear_load = 1000.0;
    joint.external_load = holds.separation_load * 0.98;
    let loaded = check_bolted_joint(&joint);
    assert!(loaded.slip_load < 0.05 * holds.slip_load, "{} N against {} N", loaded.slip_load, holds.slip_load);
}

#[test]
fn preload_protects_the_bolt_from_the_cycling_load() {
    let joint =
        BoltedJoint { size: "M10".into(), grade: "8.8".into(), clamped_length: 0.030, external_load: 10000.0, cycles: 1e7, ..Default::default() };
    let r = check_bolted_joint(&joint);
    assert!(r.solved);
    // The alternating stress is half the bolt's share, not half the load.
    let share = r.load_factor * joint.external_load;
    assert!(near(r.alternating_stress, 0.5 * share / r.stress_area, 1.0));
    let whole = 0.5 * joint.external_load / r.stress_area;
    assert!(r.alternating_stress < 0.4 * whole);
    assert!(r.endurance_limit > 100e6);
    assert!(r.fatigue_safety > 0.0);
}

#[test]
fn nonsense_is_refused_rather_than_answered() {
    let mut joint = BoltedJoint { size: "M4.5".into(), ..Default::default() };
    let r = check_bolted_joint(&joint);
    assert!(!r.solved);
    assert!(r.verdict.contains("not a metric size"));
    joint.size = "M8".into();
    joint.grade = "9.9".into();
    assert!(!check_bolted_joint(&joint).solved);
    joint.grade = "8.8".into();
    joint.clamped_length = 0.0;
    let r = check_bolted_joint(&joint);
    assert!(!r.solved);
    assert!(r.verdict.contains("grip"));
    joint.clamped_length = f64::NAN;
    assert!(!check_bolted_joint(&joint).solved);
    // And the honest caveats are always said.
    joint.clamped_length = 0.020;
    let good = check_bolted_joint(&joint);
    assert!(good.solved);
    assert!(good.advice.contains("nut factor"), "{}", good.advice);
    assert!(good.advice.contains("Not modelled"), "{}", good.advice);
}
