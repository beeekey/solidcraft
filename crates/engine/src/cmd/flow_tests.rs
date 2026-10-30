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

/// A 60 × 28 × 12 mm block with a 20 × 4 mm passage along X.
fn duct() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 28, "height": 12, "body_name": "Duct"}));
    run(&mut s, "solid.box", json!({"length": 60, "width": 20, "height": 4, "corner": [0, 4, 4], "operation": "cut", "targets": ["Duct"]}));
    s
}

#[test]
fn water_through_the_duct_loses_pressure_and_carries_heat() {
    let mut s = duct();
    let r = run(
        &mut s,
        "inspect.flow",
        json!({"region": "internal", "inlet": [0, 14, 6], "outlet": [60, 14, 6], "velocity_m_s": 0.005, "resolution": 60, "heat": true, "watts": 5}),
    );
    assert_eq!(r["converged"], true, "{r}");
    assert!(f(&r["pressure_drop_pa"]) > 0.0, "{r}");
    assert!(f(&r["reynolds"]) > 0.0 && f(&r["reynolds"]) < 100.0, "{r}");
    assert!(f(&r["volume_flow_l_min"]) > 0.0, "{r}");
    assert!((f(&r["heat_removed_w"]) - 5.0).abs() < 0.5, "{r}");
    assert!(f(&r["max_c"]) > f(&r["outlet_c"]), "{r}");
    assert!(r["verdict"].as_str().unwrap_or_default().starts_with("Re "), "{r}");
}

#[test]
fn bad_input_is_refused_with_a_reason() {
    let mut s = duct();
    let mut err = |p: Value| s.execute("inspect.flow", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"outlet": [60, 14, 6], "velocity_m_s": 1})).contains("inlet"));
    assert!(err(json!({"inlet": [0, 14, 6], "outlet": [60, 14, 6]})).contains("velocity"));
    assert!(err(json!({"inlet": [0, 14, 6], "outlet": [0, 14, 6], "velocity_m_s": 1, "resolution": 20})).contains("same face"));
    assert!(err(json!({"inlet": [0, 14, 6], "outlet": [60, 14, 6], "velocity_m_s": 1, "fluid": "mercury"})).contains("fluid"));
    assert!(err(json!({"inlet": [0, 14, 6], "outlet": [60, 14, 6], "velocity_m_s": 1, "region": "sideways"})).contains("region"));
    assert!(
        err(json!({"inlet": [0, 14, 6], "outlet": [60, 14, 6], "velocity_m_s": 1, "domain": {"min": [0, 0, 0], "max": [0, 0, 0]}}))
            .contains("domain")
    );
}
