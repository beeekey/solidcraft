//! The parametric check sweeps every driver on a scratch copy and leaves the design alone.

use serde_json::{Value, json};

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn driver<'a>(r: &'a Value, name: &str) -> &'a Value {
    r["drivers"].as_array().and_then(|a| a.iter().find(|d| d["name"] == name)).unwrap_or(&Value::Null)
}

fn volume(s: &mut Session) -> f64 {
    run(s, "inspect.measure", json!({}))["total"]["volume_mm3"].as_f64().unwrap_or(f64::NAN)
}

#[test]
fn a_parameter_the_model_follows_and_one_it_ignores() {
    let mut s = Session::default();
    run(&mut s, "parameters.add", json!({"name": "len", "expression": "60 mm"}));
    run(&mut s, "parameters.add", json!({"name": "spare", "expression": "5 mm"}));
    run(&mut s, "solid.box", json!({"length": "len", "width": 20, "height": 10, "body_name": "Bar"}));
    let before = volume(&mut s);
    let undo_depth = s.undo.len();
    let r = run(&mut s, "inspect.parametric_check", json!({}));
    assert_eq!(r["checked"], 2, "{r}");
    let len = driver(&r, "len");
    assert_eq!(len["down"]["verdict"], "follows", "{r}");
    assert_eq!(len["up"]["verdict"], "follows", "{r}");
    assert_eq!(len["worst"], "follows");
    assert!((len["down"]["asked"].as_f64().unwrap_or(0.0) - 54.0).abs() < 1e-9, "{r}");
    assert!((len["up"]["asked"].as_f64().unwrap_or(0.0) - 66.0).abs() < 1e-9, "{r}");
    assert!(len["up"]["note"].as_str().unwrap_or_default().contains("followed"), "{r}");
    let spare = driver(&r, "spare");
    assert_eq!(spare["worst"], "inert", "{r}");
    assert_eq!(r["survived"], 1);
    assert_eq!(r["inert"], 1);
    assert_eq!(r["broke"], 0);
    assert!(r["advice"].as_str().unwrap_or_default().contains("inert parameter"), "{r}");
    // The design is as it was: same volume, same parameter, nothing on the undo stack.
    assert_eq!(volume(&mut s), before);
    assert_eq!(s.undo.len(), undo_depth);
    let list = run(&mut s, "parameters.list", json!({}));
    let len_row = list["parameters"].as_array().and_then(|a| a.iter().find(|p| p["name"] == "len")).cloned().unwrap_or_default();
    assert_eq!(len_row["expression"], "60 mm", "{list}");
    assert_eq!(len_row["value"], 60.0, "{list}");
}

#[test]
fn a_hole_that_outgrows_its_plate_breaks_the_model_one_way() {
    let mut s = Session::default();
    run(&mut s, "parameters.add", json!({"name": "d", "expression": "30 mm"}));
    run(&mut s, "solid.box", json!({"length": 40, "width": 40, "height": 10, "corner": [-20, -20, 0], "body_name": "Plate"}));
    run(&mut s, "solid.cylinder", json!({"diameter": "d", "height": 10, "operation": "cut", "targets": ["Plate"]}));
    // ±50 %: Ø15 still a hole, Ø45 wider than the plate, which leaves four corners (or nothing).
    let r = run(&mut s, "inspect.parametric_check", json!({"percent": 50}));
    let d = driver(&r, "d");
    assert_eq!(d["down"]["verdict"], "follows", "{r}");
    let up = d["up"]["verdict"].as_str().unwrap_or_default();
    assert!(matches!(up, "DETACHES" | "EMPTIES" | "FAILS"), "{r}");
    assert_eq!(d["worst"], up);
    assert_eq!(r["broke"], 1, "{r}");
    assert!(r["advice"].as_str().unwrap_or_default().contains("breaks the model"), "{r}");
    assert!(r["verdict"].as_str().unwrap_or_default().contains("break"), "{r}");
}

#[test]
fn model_dimensions_join_the_sweep_only_when_asked_and_the_cap_holds() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 30, "width": 20, "height": 10}));
    let r = run(&mut s, "inspect.parametric_check", json!({}));
    assert_eq!(r["checked"], 0, "{r}");
    assert_eq!(r["verdict"], "no parameters to check");
    assert!(r["advice"].as_str().unwrap_or_default().contains("include_model"), "{r}");
    let all = run(&mut s, "inspect.parametric_check", json!({"include_model": true, "max_drivers": 2}));
    assert_eq!(all["checked"], 2, "{all}");
    assert!(all["warnings"].as_array().is_some_and(|w| w.iter().any(|x| x.as_str().unwrap_or_default().contains("max_drivers"))), "{all}");
    assert_eq!(all["drivers"][0]["source"], "feature", "{all}");
    assert_eq!(all["drivers"][0]["worst"], "follows", "{all}");
    assert!(s.execute("inspect.parametric_check", &json!({"percent": 200})).is_err());
    assert!(s.execute("inspect.parametric_check", &json!({"max_drivers": "many"})).is_err());
}
