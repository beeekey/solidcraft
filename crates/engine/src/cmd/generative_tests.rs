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

/// The bracket problem from the solver's tests, as the command takes it: boxes in mm.
fn bracket(extra: Value) -> Value {
    let mut p = json!({
        "mounts": [{"box": {"min": [0, 0, 0], "max": [12, 40, 60]}, "name": "wall"}],
        "loads": [{"box": {"min": [108, 0, 0], "max": [120, 40, 12]}, "name": "tip", "force_n": [0, 0, -1000]}],
        "design_space": {"box": {"min": [0, 0, 0], "max": [120, 40, 60]}},
        "material": "Steel",
        "resolution": 14,
        "population": 10,
        "generations": 3,
        "solutions": 2,
        "seed": 7,
        "free_nodes": 2,
        "min_radius_mm": 3,
        "max_radius_mm": 18,
        "gravity": false,
    });
    if let Some(o) = extra.as_object() {
        for (k, v) in o {
            p[k] = v.clone();
        }
    }
    p
}

#[test]
fn a_study_offers_designs_and_builds_one_as_a_mesh_body() {
    let mut s = Session::default();
    let r = run(&mut s, "generative.study", bracket(json!({"build": 0, "name": "Bracket"})));
    let designs = r["designs"].as_array().cloned().unwrap_or_default();
    assert!(!designs.is_empty() && designs.len() <= 2, "{r}");
    for d in &designs {
        assert!(f(&d["mass_g"]) > 0.0 && f(&d["safety_factor"]) > 0.0, "{d}");
        assert!(d["summary"].as_str().is_some_and(|t| t.contains("struts")), "{d}");
    }
    assert!(r["built"]["feature"].is_number(), "{r}");
    let bodies: Vec<String> = s.world_state().bodies.iter().map(|b| b.name.clone()).collect();
    assert!(bodies.iter().any(|b| b == "Bracket"), "{bodies:?}");
    assert!(r["fem_evaluations"].as_u64().unwrap_or(0) > 0);
    assert_eq!(r["mass_history_g"].as_array().map(Vec::len), Some(3));
}

#[test]
fn a_free_form_body_is_built_and_checked() {
    let mut s = Session::default();
    let r = run(
        &mut s,
        "generative.free_form",
        bracket(json!({"resolution": 16, "iterations": 8, "volume_fraction": 0.25, "meet_target": false, "name": "Field"})),
    );
    assert!(f(&r["mass_g"]) > 0.0, "{r}");
    assert!(r["built"]["feature"].is_number(), "{r}");
    let r = run(&mut s, "generative.check", bracket(json!({"body": "Field", "resolution": 20, "cyclic": true, "finish": "As forged"})));
    assert!(f(&r["safety_factor"]) > 0.0, "{r}");
    assert!(r["buckling"]["note"].as_str().is_some_and(|n| !n.is_empty()));
    assert_eq!(r["fatigue"]["assessed"], true, "{r}");
    assert!(r["fatigue"]["note"].as_str().is_some_and(|n| n.contains("As forged")), "{r}");
    assert!(r["manufacturing_note"].as_str().is_some_and(|n| n.contains("thinnest")), "{r}");
}

#[test]
fn mounts_and_loads_can_be_faces_of_a_body() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 120, "width": 40, "height": 60, "body_name": "Space"}));
    let r = run(
        &mut s,
        "generative.study",
        json!({
            "mounts": [{"point": [0, 20, 30], "body": "Space", "pad_mm": 12}],
            "loads": [{"point": [120, 20, 30], "body": "Space", "pad_mm": 12, "force_n": [0, 0, -500]}],
            "design_space": "Space",
            "resolution": 12, "population": 8, "generations": 2, "solutions": 1, "gravity": false,
        }),
    );
    assert!(!r["designs"].as_array().cloned().unwrap_or_default().is_empty(), "{r}");
}

#[test]
fn the_fibre_check_tells_splitting_from_stretching() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 100, "width": 10, "height": 10, "body_name": "Culm"}));
    let along = run(
        &mut s,
        "inspect.fibre",
        json!({"fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [2000, 0, 0], "fibre": [1, 0, 0], "species": "Tonkin", "resolution": 20}),
    );
    let across = run(
        &mut s,
        "inspect.fibre",
        json!({"fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [2000, 0, 0], "fibre": [0, 1, 0], "species": "Tonkin", "resolution": 20}),
    );
    assert!(f(&across["worst_ratio"]) > f(&along["worst_ratio"]) * 5.0, "{along}\n{across}");
    assert!(across["mode"].as_str().is_some_and(|m| m.contains("across")), "{across}");
    assert!(along["verdict"].as_str().is_some_and(|v| v.starts_with("holds")), "{along}");
}

#[test]
fn bad_input_is_refused_with_a_reason() {
    let mut s = Session::default();
    let mut err = |id: &str, p: Value| s.execute(id, &p).map(|_| ()).unwrap_err().to_string();
    assert!(err("generative.study", json!({"loads": []})).contains("mounts"));
    assert!(err("generative.study", bracket(json!({"goal": "fly"}))).contains("goal"));
    assert!(err("generative.study", bracket(json!({"mounts": [{"box": {"min": [0, 0, 0], "max": [0, 0, 0]}}]}))).contains("box"));
    assert!(err("generative.study", bracket(json!({"min_radius_mm": 20, "max_radius_mm": 5}))).contains("radius"));
    assert!(err("generative.check", bracket(json!({}))).contains("body"));
    assert!(err("inspect.fibre", json!({"fixed": [0, 0, 0], "load": [1, 0, 0], "force_n": [1, 0, 0]})).contains("body"));
}
