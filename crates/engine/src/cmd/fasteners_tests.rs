use serde_json::{Value, json};

use crate::Session;

fn run(id: &str, p: Value) -> Value {
    let mut s = Session::default();
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn fails(id: &str, p: Value) -> String {
    let mut s = Session::default();
    s.execute(id, &p).map(|_| ()).unwrap_err().to_string()
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or(f64::NAN)
}

#[test]
fn an_m10_bolted_joint_matches_the_torque_chart() {
    let r = run("inspect.bolted_joint", json!({"size": "M10", "grade": "8.8", "clamped_mm": 30}));
    assert!((f(&r["preload_n"]) - 26100.0).abs() < 400.0, "{r}");
    assert!((f(&r["tightening_torque_nm"]) - 52.0).abs() < 2.0, "{r}");
    assert_eq!(r["separates"], false);
    assert_eq!(r["warnings"], json!([]));
    assert!(r["verdict"].as_str().unwrap_or_default().starts_with("M10 8.8"), "{r}");
    // Pulled past its clamp, it says so.
    let gone = run("inspect.bolted_joint", json!({"size": "M10", "grade": "8.8", "clamped_mm": 30, "external_n": 60000}));
    assert_eq!(gone["separates"], true, "{gone}");
    assert!(gone["warnings"].as_array().is_some_and(|w| !w.is_empty()));
    assert!(fails("inspect.bolted_joint", json!({"size": "M7", "clamped_mm": 10})).contains("metric size"));
    assert!(fails("inspect.bolted_joint", json!({"size": "M8"})).contains("clamped_mm"));
    assert!(fails("inspect.bolted_joint", json!({"size": "M8", "clamped_mm": "ten"})).contains("clamped_mm"));
}

#[test]
fn thread_engagement_applies_the_rule_per_material() {
    let short = run("inspect.thread_engagement", json!({"size": "M6", "material": "aluminium", "engagement_mm": 8, "through": true}));
    assert_eq!(short["severity"], "warning", "{short}");
    assert!((f(&short["needed_mm"]) - 12.0).abs() < 1e-9);
    let fine = run("inspect.thread_engagement", json!({"size": "M6", "material": "steel", "engagement_mm": 10}));
    assert_eq!(fine["severity"], "ok", "{fine}");
    assert_eq!(fine["warnings"], json!([]));
    assert!(fails("inspect.thread_engagement", json!({"size": "M6", "material": "wood", "engagement_mm": 10})).contains("material"));
    assert!(fails("inspect.thread_engagement", json!({"size": "M6", "material": "steel"})).contains("engagement_mm"));
}

#[test]
fn bearing_life_comes_from_the_table_or_the_ratings() {
    let r = run("inspect.bearing_life", json!({"designation": "6001", "radial_n": 500, "rpm": 200, "wheel_mm": 700}));
    assert!((f(&r["million_revolutions"]) - (5100.0_f64 / 500.0).powi(3)).abs() < 1e-6, "{r}");
    assert!(f(&r["hours"]) > 0.0 && f(&r["kilometres"]) > 1000.0, "{r}");
    assert_eq!(r["designation"], "6001");
    let by_numbers = run("inspect.bearing_life", json!({"dynamic_n": 5100, "static_n": 2360, "radial_n": 500}));
    assert_eq!(f(&by_numbers["million_revolutions"]), f(&r["million_revolutions"]));
    assert!(fails("inspect.bearing_life", json!({"designation": "Headset 41.8", "radial_n": 500})).contains("no published dynamic rating"));
    assert!(fails("inspect.bearing_life", json!({"radial_n": 500})).contains("designation"));
    assert!(fails("inspect.bearing_life", json!({"designation": "9999", "radial_n": 500})).contains("9999"));
}

#[test]
fn damping_reports_isolation_and_the_material_table() {
    let trailer = run("inspect.damping", json!({"mass_kg": 25, "rate_n_mm": 12, "damping_ns_m": 400, "excitation_hz": 10}));
    let natural = (12000.0_f64 / 25.0).sqrt() / (2.0 * std::f64::consts::PI);
    assert!((f(&trailer["natural_hz"]) - natural).abs() < 1e-9, "{trailer}");
    assert!(f(&trailer["transmissibility"]) < 1.0, "{trailer}");
    assert!(trailer["materials"].as_array().is_some_and(|m| m.len() == 7));
    let bamboo = run(
        "inspect.damping",
        json!({"mass_kg": 8, "rate_n_m": 8.0 * (2.0 * std::f64::consts::PI * 30.0_f64).powi(2), "material": "Bamboo", "excitation_hz": 60}),
    );
    assert!((f(&bamboo["damping_ratio"]) - 0.015).abs() < 1e-12, "{bamboo}");
    assert!(f(&bamboo["cycles_to_half_amplitude"]) < 10.0);
    assert!(fails("inspect.damping", json!({"mass_kg": 8, "rate_n_mm": 1, "material": "cheese"})).contains("loss factor"));
    assert!(fails("inspect.damping", json!({"mass_kg": 8})).contains("rate_n_mm"));
}

#[test]
fn a_bamboo_joint_is_governed_by_the_bamboo_and_sized_by_its_engagement() {
    // Ø32 culm, 60 mm deep: bond area π·32·60 = 6032 mm², pull-out 5 MPa × that = 30.2 kN.
    let r = run("inspect.bamboo_joint", json!({"culm_diameter_mm": 32, "wall_mm": 8, "engagement_mm": 60, "axial_n": 4000, "moment_nm": 100}));
    let area = std::f64::consts::PI * 32.0 * 60.0;
    assert!((f(&r["bond_area_mm2"]) - area).abs() < 1e-6, "{r}");
    assert_eq!(r["bamboo_governs"], true);
    assert!((f(&r["pull_out_capacity_n"]) - 5.0 * area).abs() < 1e-6, "{r}");
    assert!((f(&r["axial_safety"]) - 5.0 * area / 4000.0).abs() < 1e-6, "{r}");
    // 100 N·m as a couple: 1.5·100/0.06 N over 32·60/3 mm² = 3.9 MPa against 11 MPa.
    assert!((f(&r["bearing_stress_mpa"]) - 2500.0 / 640.0).abs() < 1e-6, "{r}");
    assert!((f(&r["bearing_capacity_mpa"]) - 11.0).abs() < 1e-9);
    // The engagement for 4 kN at safety 3: 12000 / (π·32·5) = 23.9 mm.
    assert!((f(&r["engagement_needed_mm"]) - 12000.0 / (std::f64::consts::PI * 32.0 * 5.0)).abs() < 1e-6, "{r}");
    assert!(r["advice"].as_str().unwrap_or_default().contains("node"), "{r}");
    assert!(r["verdict"].as_str().unwrap_or_default().contains("pull-out safety"), "{r}");
    let noded = run("inspect.bamboo_joint", json!({"culm_diameter_mm": 32, "wall_mm": 8, "engagement_mm": 60, "node_inside_socket": true}));
    assert!((f(&noded["bearing_capacity_mpa"]) - 22.0).abs() < 1e-9, "{noded}");
    assert!(fails("inspect.bamboo_joint", json!({"culm_diameter_mm": 32, "wall_mm": 20, "engagement_mm": 60})).contains("tube"));
    assert!(fails("inspect.bamboo_joint", json!({"culm_diameter_mm": 32, "wall_mm": 8})).contains("engagement_mm"));
    assert!(fails("inspect.bamboo_joint", json!({"culm_diameter_mm": "thick", "wall_mm": 8, "engagement_mm": 60})).contains("culm_diameter_mm"));
}
