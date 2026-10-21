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

/// A 40 × 40 × 5 top on a 10 × 10 × 15 leg: a table.
fn table() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 10, "width": 10, "height": 15, "corner": [15, 15, 0], "body_name": "Table"}));
    run(&mut s, "solid.box", json!({"length": 40, "width": 40, "height": 5, "corner": [0, 0, 15], "operation": "join", "targets": ["Table"]}));
    s
}

#[test]
fn the_table_needs_support_leg_down_and_none_upside_down() {
    let mut s = table();
    let r = run(&mut s, "inspect.print", json!({"body": "Table"}));
    assert_eq!(r["watertight"], true, "{r}");
    assert!((f(&r["overhang_area_mm2"]) - 1500.0).abs() < 1e-3, "{r}");
    assert!((f(&r["support_volume_mm3"]) - 1500.0 * 15.0).abs() < 1e-3, "{r}");
    assert!((f(&r["volume_mm3"]) - 9500.0).abs() < 1e-3, "{r}");
    assert_eq!(r["layers"], 100, "{r}");
    assert!(f(&r["material_grams"]) > 0.0 && f(&r["time_s"]) > 0.0);
    assert!((f(&r["thinnest_wall_mm"]) - 5.0).abs() < 0.01, "{r}");
    assert!(f(&r["printed_material"]["young_modulus_in_plane_pa"]) > 0.0, "{r}");
    assert!(r["verdict"].as_str().unwrap_or_default().contains("needs support"), "{r}");
    let flipped = run(&mut s, "inspect.print", json!({"body": "Table", "up": [0, 0, -1]}));
    assert!(f(&flipped["overhang_area_mm2"]).abs() < 1e-6, "{flipped}");
    let orient = run(&mut s, "inspect.print_orient", json!({}));
    assert_eq!(orient["best"], "-Z", "{orient}");
    assert_eq!(orient["options"].as_array().map(Vec::len), Some(6));
    let mut err = |p: Value| s.execute("inspect.print", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"critical_angle_deg": 95})).contains("critical_angle_deg"));
    assert!(err(json!({"pattern": "spiral"})).contains("pattern"));
    assert!(err(json!({"up": [0, 0, 0]})).contains("up"));
}

#[test]
fn a_pocketed_block_is_milled_from_the_top_and_its_bore_is_a_hole() {
    // 40 × 40 × 20 with a 20 × 20 × 10 pocket from the top and a 6 mm through hole beside it.
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 40, "height": 20, "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 20, "width": 20, "height": 11, "corner": [10, 10, 10], "operation": "cut", "targets": ["Block"]}));
    run(&mut s, "solid.cylinder", json!({"radius": 3, "height": 30, "base": [5, 5, -5], "axis": "Z", "operation": "cut", "targets": ["Block"]}));
    let r = run(&mut s, "inspect.machining", json!({"body": "Block", "cutter_mm": 6}));
    assert_eq!(r["assumed"]["process"], "milling", "{r}");
    assert_eq!(r["assumed"]["material"], "Aluminium");
    assert_eq!(r["access"]["setups"], 1, "{}", r["access"]);
    assert_eq!(r["access"]["setup_axes"], json!(["+Z"]), "{}", r["access"]);
    assert_eq!(f(&r["access"]["unreachable_fraction"]), 0.0, "{}", r["access"]);
    let holes = r["holes"].as_array().cloned().unwrap_or_default();
    assert_eq!(holes.len(), 1, "{r}");
    assert!((f(&holes[0]["diameter_mm"]) - 6.0).abs() < 1e-6 && (f(&holes[0]["depth_mm"]) - 20.0).abs() < 1e-3, "{}", holes[0]);
    assert_eq!(holes[0]["through"], true);
    assert!(r["text"].as_str().unwrap_or_default().starts_with("assumed:"), "{}", r["text"]);
    assert!(r["caveats"].as_array().is_some_and(|c| !c.is_empty()));
    // A 1 mm cutter for a 20 mm pocket and no sharp-corner recognition: nothing refused.
    assert_eq!(r["refusals"], 0, "{r}");
    // A sealed void is refused.
    run(
        &mut s,
        "solid.box",
        json!({"length": 10, "width": 10, "height": 5, "corner": [25, 25, 2], "operation": "cut", "targets": ["Block"], "body_name": "Block"}),
    );
    let r = run(&mut s, "inspect.machining", json!({"body": "Block"}));
    assert!(r["findings"].as_array().is_some_and(|fs| fs.iter().any(|f| f["rule"] == "enclosed cavity")), "{r}");
    assert!(r["verdict"].as_str().unwrap_or_default().starts_with("cannot be made"), "{r}");
}

#[test]
fn a_bar_is_a_turned_part_and_bad_input_is_refused() {
    let mut s = Session::default();
    run(&mut s, "solid.cylinder", json!({"radius": 10, "height": 100, "base": [0, 0, 0], "axis": "X", "body_name": "Bar"}));
    let r = run(&mut s, "inspect.machining", json!({"body": "Bar"}));
    assert_eq!(r["assumed"]["process"], "turning", "{}", r["revolution"]);
    assert_eq!(r["revolution"]["is_body_of_revolution"], true, "{}", r["revolution"]);
    assert!((f(&r["revolution"]["slenderness"]) - 5.0).abs() < 0.1, "{}", r["revolution"]);
    assert!(r["findings"].as_array().is_some_and(|fs| fs.iter().any(|f| f["rule"] == "slenderness")), "{r}");
    let mut err = |p: Value| s.execute("inspect.machining", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"process": "grinding"})).contains("process"));
    assert!(err(json!({"material": "cheese"})).contains("material"));
    assert!(err(json!({"cutter_mm": 500})).contains("cutter_mm"));
}
