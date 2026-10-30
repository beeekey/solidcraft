use solidcraft_geom::Vec3;

use crate::fibre_failure::{FibreState, FibreStrength, bamboo_strength, check_fibre_failure};

fn tonkin() -> FibreStrength {
    bamboo_strength("tonkin").unwrap_or_default()
}

fn state(tensor: [f64; 6], fibre: Vec3) -> FibreState {
    FibreState { stress_tensor: tensor.to_vec(), solid: vec![true], exclude_from_peak: Vec::new(), fibre_direction: fibre, worst_von_mises: 0.0 }
}

#[test]
fn the_species_table_answers_by_name_and_refuses_the_unknown() {
    let t = tonkin();
    assert_eq!(t.along_tension, 200e6);
    assert_eq!(t.shear, 6.0e6);
    assert_eq!(t.across_tension, t.shear, "splitting runs at the interlaminar strength");
    assert!(bamboo_strength("Moso").is_some());
    assert!(bamboo_strength("oak").is_none());
}

#[test]
fn pulling_along_the_fibre_is_the_easy_direction() {
    // 100 MPa along X with the fibre along X: half the tensile strength, safe.
    let along = check_fibre_failure(&state([100e6, 0.0, 0.0, 0.0, 0.0, 0.0], Vec3::X), &tonkin());
    assert!(along.valid);
    assert!((along.worst_ratio - 0.5).abs() < 1e-9, "{}", along.worst_ratio);
    assert_eq!(along.mode, "tension along the fibre");
    // The same 100 MPa across the fibre splits it many times over.
    let across = check_fibre_failure(&state([100e6, 0.0, 0.0, 0.0, 0.0, 0.0], Vec3::Y), &tonkin());
    assert!(across.worst_ratio > 10.0, "{}", across.worst_ratio);
    assert_eq!(across.mode, "splitting across the fibre");
    assert!(across.advice.contains("fails"));
}

#[test]
fn shear_between_the_layers_is_the_other_low_number() {
    let mut s = state([0.0, 0.0, 0.0, 3e6, 0.0, 0.0], Vec3::X);
    s.worst_von_mises = 3e6 * 3f64.sqrt();
    let report = check_fibre_failure(&s, &tonkin());
    assert_eq!(report.mode, "interlaminar shear");
    assert!((report.worst_ratio - 0.5).abs() < 1e-9);
    assert!(report.advice.contains("Von Mises"), "{}", report.advice);
    assert!(report.advice.contains("joint"), "{}", report.advice);
}

#[test]
fn a_missing_frame_or_state_is_refused_with_a_reason() {
    let no_fibre = check_fibre_failure(&state([1e6; 6], Vec3::ZERO), &tonkin());
    assert!(!no_fibre.valid && no_fibre.advice.contains("fibre direction"));
    let empty = check_fibre_failure(&FibreState::default(), &tonkin());
    assert!(!empty.valid);
    let weak = check_fibre_failure(&state([1e6; 6], Vec3::X), &FibreStrength::default());
    assert!(!weak.valid && weak.advice.contains("positive"));
}
