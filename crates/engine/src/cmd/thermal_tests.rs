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

/// A 60 × 10 × 60 mm aluminium plate standing on its edge.
fn plate() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 10, "height": 60, "body_name": "Plate"}));
    run(&mut s, "material.assign", json!({"bodies": ["Plate"], "material": "Aluminum"}));
    s
}

#[test]
fn every_watt_leaves_the_plate_and_the_answer_names_the_material() {
    let mut s = plate();
    let r = run(&mut s, "inspect.thermal", json!({"heat": [30, 5, 0], "watts": 5, "emissivity": 0.85, "resolution": 24}));
    assert_eq!(r["material"], "Aluminum", "{r}");
    assert_eq!(f(&r["conductivity_w_mk"]), 205.0);
    let leaving = f(&r["convected_w"]) + f(&r["radiated_w"]);
    assert!((leaving - 5.0).abs() < 0.1, "{r}");
    assert!(f(&r["peak_c"]) > 20.0 && f(&r["peak_c"]) < 200.0, "{r}");
    assert!(r["verdict"].as_str().unwrap_or_default().starts_with("peaks at"), "{r}");
    assert!(r["field"].is_null());
    // A cold plate under it takes most of the heat.
    let held =
        run(&mut s, "inspect.thermal", json!({"heat": [30, 5, 60], "watts": 10, "held": [30, 5, 0], "held_c": 20, "resolution": 24, "field": true}));
    assert!(f(&held["held_w"]) > 5.0, "{held}");
    assert!(f(&held["peak_c"]) < 60.0, "{held}");
    assert!(held["field"]["cells"].as_array().is_some_and(|c| !c.is_empty()));
}

#[test]
fn bad_input_is_refused_with_a_reason() {
    let mut s = plate();
    let mut err = |p: Value| s.execute("inspect.thermal", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"watts": 5})).contains("heat"));
    assert!(err(json!({"heat": [30, 5, 0]})).contains("watts"));
    assert!(err(json!({"heat": [30, 5, 0], "watts": 1e9})).contains("watts"));
    assert!(err(json!({"heat": [30, 5, 0], "watts": 5, "resolution": 1000})).contains("resolution"));
    assert!(err(json!({"heat": [30, 5, 0], "watts": 5, "up": [0, 0, 0]})).contains("up"));
}
