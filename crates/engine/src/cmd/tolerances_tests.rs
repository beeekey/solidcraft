use serde_json::{Value, json};

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or(f64::NAN)
}

#[test]
fn a_fit_reports_iso_286_limits_and_the_press_numbers() {
    let mut s = Session::default();
    let r =
        run(&mut s, "inspect.fit", json!({"nominal_mm": 25, "hole": "H7", "shaft": "s6", "engagement_length_mm": 20, "hub_outer_diameter_mm": 50}));
    assert_eq!(r["kind"], "interference", "{r}");
    assert!((f(&r["hole"]["upper_mm"]) - 0.021).abs() < 1e-9);
    assert!((f(&r["shaft"]["lower_mm"]) - 0.035).abs() < 1e-9);
    assert!((f(&r["max_interference_mm"]) - 0.048).abs() < 1e-9);
    assert!((f(&r["contact_pressure_mpa"]) - 151.2).abs() < 0.1, "{r}");
    assert!((f(&r["assembly_force_n"]) - 35630.0).abs() < 20.0);
    assert!(r["hole"]["source"].as_str().unwrap_or_default().contains("ISO 286"));
    assert!(r["warnings"].as_array().is_some_and(|w| w.iter().any(|x| x.as_str().unwrap_or_default().contains("yields"))), "{r}");
    let g6 = run(&mut s, "inspect.fit", json!({"nominal_mm": 25, "hole": "H7", "shaft": "g6"}));
    assert_eq!(g6["kind"], "clearance");
    assert!((f(&g6["min_clearance_mm"]) - 0.007).abs() < 1e-9);
    assert!(g6["verdict"].as_str().unwrap_or_default().starts_with("clearance fit"));
}

#[test]
fn a_fit_refuses_what_the_table_cannot_stand_behind() {
    let mut s = Session::default();
    let err = |s: &mut Session, p: Value| s.execute("inspect.fit", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(&mut s, json!({"nominal_mm": 25, "hole": "H20", "shaft": "g6"})).contains("IT1 to IT18"));
    assert!(err(&mut s, json!({"nominal_mm": 620, "hole": "H7", "shaft": "g6"})).contains("500 mm"));
    assert!(err(&mut s, json!({"hole": "H7", "shaft": "g6"})).contains("nominal"));
    assert!(err(&mut s, json!({"nominal_mm": "x", "hole": "H7", "shaft": "g6"})).contains("nominal_mm"));
    assert!(err(&mut s, json!({})).contains("classes"));
    // Measured sizes work without classes.
    let r = run(&mut s, "inspect.fit", json!({"hole_min_mm": 25.0, "hole_max_mm": 25.021, "shaft_min_mm": 24.98, "shaft_max_mm": 24.993}));
    assert_eq!(r["kind"], "clearance");
    assert_eq!(r["hole"]["source"], "measured");
}

#[test]
fn a_general_tolerance_reads_the_right_table() {
    let mut s = Session::default();
    let linear = run(&mut s, "inspect.general_tolerance", json!({"class": "ISO 2768-mK", "nominal_mm": 25}));
    assert!((f(&linear["plus_minus_mm"]) - 0.2).abs() < 1e-9, "{linear}");
    assert!((f(&linear["upper_mm"]) - 25.2).abs() < 1e-9);
    let chamfer = run(&mut s, "inspect.general_tolerance", json!({"class": "m", "nominal_mm": 1, "kind": "radius"}));
    assert!((f(&chamfer["plus_minus_mm"]) - 0.2).abs() < 1e-9);
    let angle = run(&mut s, "inspect.general_tolerance", json!({"class": "m", "nominal_mm": 25, "kind": "angle"}));
    assert!((f(&angle["plus_minus_deg"]) - 0.5).abs() < 1e-9);
    let err = |s: &mut Session, p: Value| s.execute("inspect.general_tolerance", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(&mut s, json!({"class": "k", "nominal_mm": 25})).contains("not an ISO 2768 class"));
    assert!(err(&mut s, json!({"class": "v", "nominal_mm": 2})).contains("individually"));
    assert!(err(&mut s, json!({"class": "m", "nominal_mm": 25, "kind": "weight"})).contains("kind"));
}

#[test]
fn a_tolerance_stack_adds_up_both_ways() {
    let mut s = Session::default();
    let r = run(
        &mut s,
        "inspect.tolerance_stack",
        json!({"chain": [
            {"name": "housing", "nominal_mm": 50, "plus_minus_mm": 0.10},
            {"name": "shaft", "nominal_mm": 30, "plus_minus_mm": 0.05, "sensitivity": -1},
            {"name": "bearing", "nominal_mm": 12, "lower_mm": -0.02, "upper_mm": 0.02, "sensitivity": -1}
        ], "minimum_mm": 7.9, "maximum_mm": 8.1}),
    );
    assert!((f(&r["nominal_mm"]) - 8.0).abs() < 1e-9, "{r}");
    assert!((f(&r["worst_case"]["half_width_mm"]) - 0.17).abs() < 1e-12);
    assert_eq!(r["worst_case_passes"], false);
    assert_eq!(r["ranked"][0]["name"], "housing");
    assert!(r["warnings"].as_array().is_some_and(|w| !w.is_empty()));
    let err = |s: &mut Session, p: Value| s.execute("inspect.tolerance_stack", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(&mut s, json!({})).contains("chain"));
    assert!(err(&mut s, json!({"chain": []})).contains("1 to"));
    assert!(err(&mut s, json!({"chain": [5]})).contains("object"));
    assert!(err(&mut s, json!({"chain": [{"nominal_mm": 1}]})).contains("lower_mm"));
    assert!(err(&mut s, json!({"chain": [{"nominal_mm": 1, "plus_minus_mm": 0.1}], "minimum_mm": 1})).contains("maximum_mm"));
}

#[test]
fn a_hole_names_its_screw() {
    let mut s = Session::default();
    let r = run(&mut s, "inspect.hole_stack", json!({"hole_diameter_mm": 6.6, "grip_mm": 16}));
    assert_eq!(r["thread"], "M6", "{r}");
    assert_eq!(r["series"], "Medium");
    assert!((f(&r["screw"]["length_mm"]) - 25.0).abs() < 1e-9);
    assert!((f(&r["screw"]["needed_mm"]) - 24.8).abs() < 1e-9);
    assert!((f(&r["drill"]["tapping_mm"]) - 5.0).abs() < 1e-9);
    assert!(r["verdict"].as_str().unwrap_or_default().contains("M6 x 25"));
    let no_nut = run(&mut s, "inspect.hole_stack", json!({"hole_diameter_mm": 6.6, "grip_mm": 16, "nut": false}));
    assert!((f(&no_nut["screw"]["length_mm"]) - 20.0).abs() < 1e-9);
    assert!(no_nut["warnings"].as_array().is_some_and(|w| w.len() == 1));
    let err = |s: &mut Session, p: Value| s.execute("inspect.hole_stack", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(&mut s, json!({"hole_diameter_mm": 7.3, "grip_mm": 16})).contains("ISO 273"));
    assert!(err(&mut s, json!({"grip_mm": 16})).contains("hole_diameter_mm"));
    assert!(err(&mut s, json!({"hole_diameter_mm": 6.6, "grip_mm": -1})).contains("grip_mm"));
}
