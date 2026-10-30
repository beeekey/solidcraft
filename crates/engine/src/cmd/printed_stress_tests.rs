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

/// An 80 × 10 × 10 bar along x.
fn bar() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 80, "width": 10, "height": 10, "body_name": "Bar"}));
    s
}

#[test]
fn a_bar_printed_flat_holds_within_a_layer_and_standing_on_end_fails_between_them() {
    let mut s = bar();
    let load = json!({"fixed": [0, 5, 5], "load": [80, 5, 5], "force_n": [0, 0, -20], "resolution": 12});
    let flat = run(&mut s, "inspect.printed_stress", load.clone());
    assert_eq!(flat["body"], "Bar");
    assert_eq!(flat["governs"], "within a layer", "{flat}");
    assert_eq!(flat["settings"]["pattern"], "Grid");
    assert_eq!(flat["settings"]["material"], "PLA");
    assert!(f(&flat["printed_material"]["young_in_plane_gpa"]) > f(&flat["printed_material"]["young_across_layers_gpa"]), "{flat}");
    assert!(f(&flat["printed_material"]["tensile_interlayer_mpa"]) < f(&flat["printed_material"]["tensile_in_plane_mpa"]));
    assert!(f(&flat["printed_material"]["mass_grams"]) > 0.0);
    assert!(f(&flat["max_von_mises_mpa"]) > 0.0 && f(&flat["max_displacement_mm"]) > 0.0);
    assert!(f(&flat["safety_in_plane"]) > 0.0 && f(&flat["safety_interlayer"]) > f(&flat["safety_in_plane"]), "{flat}");
    assert!(flat["verdict"].as_str().unwrap_or("").contains("within a layer"), "{flat}");
    assert!(flat["advice"].as_str().unwrap_or("").contains("interlayer"));
    assert!(flat.get("field").is_none_or(Value::is_null));

    let mut standing = load.clone();
    standing["up"] = json!([1, 0, 0]);
    standing["field"] = json!(true);
    let standing = run(&mut s, "inspect.printed_stress", standing);
    assert_eq!(standing["governs"], "between layers", "{standing}");
    assert!(f(&standing["safety_interlayer"]) < f(&flat["safety_interlayer"]));
    assert!(f(&standing["max_layer_normal_mpa"]) > f(&flat["max_layer_normal_mpa"]));
    assert!(f(&standing["max_layer_normal_at"][0]) < 20.0, "the layers part at the root: {}", standing["max_layer_normal_at"]);
    assert!(f(&standing["max_displacement_mm"]) > f(&flat["max_displacement_mm"]), "softer across the layers");
    let cells = standing["field"]["cells"].as_array().map(Vec::len).unwrap_or(0);
    assert!(cells > 0 && standing["layer_field"]["cells"].as_array().map(Vec::len) == Some(cells), "{}", standing["grid"]);

    // A weaker print: one wall, lightning infill, the part gives.
    let mut weak = load.clone();
    weak["pattern"] = json!("Lightning");
    weak["walls"] = json!(1);
    weak["infill"] = json!(0.05);
    weak["force_n"] = json!([0, 0, -200]);
    let weak = run(&mut s, "inspect.printed_stress", weak);
    assert!(f(&weak["safety_in_plane"]) < f(&flat["safety_in_plane"]), "{weak}");
    assert!(weak["warnings"].as_array().is_some_and(|w| w.iter().any(|x| x.as_str().unwrap_or("").contains("lightning"))), "{weak}");
    assert_eq!(weak["settings"]["walls"], 1);
}

#[test]
fn compare_infill_lists_every_pattern_at_one_mass() {
    let mut s = bar();
    let r =
        run(&mut s, "inspect.compare_infill", json!({"fixed": [0, 5, 5], "load": [80, 5, 5], "force_n": [0, 0, -20], "resolution": 8, "walls": 1}));
    let rows = r["patterns"].as_array().cloned().unwrap_or_default();
    assert_eq!(rows.len(), 11, "{r}");
    assert_eq!(rows[0]["pattern"], "Grid");
    assert_eq!(rows[6]["pattern"], "Gyroid");
    let mass = f(&rows[0]["mass_g"]);
    assert!(mass > 0.0 && rows.iter().all(|row| (f(&row["mass_g"]) - mass).abs() < 1e-9 * mass), "{r}");
    assert!(rows.iter().all(|row| f(&row["deflection_mm"]) > 0.0 && f(&row["safety_in_plane"]) > 0.0), "{r}");
    let stiffest = r["stiffest"].as_str().unwrap_or("");
    assert!(rows.iter().any(|row| row["pattern"] == stiffest), "{r}");
    assert!(r["safest_between_layers"].is_string());
    let text = r["text"].as_str().unwrap_or("");
    assert!(text.starts_with("pattern") && text.contains("Gyroid") && text.lines().count() == 12, "{text}");
    assert!(r["verdict"].as_str().unwrap_or("").starts_with("same "), "{r}");
    assert!(r["advice"].as_str().unwrap_or("").contains("free choice"));
    // Lightning bends more than grid.
    let by = |name: &str| rows.iter().find(|row| row["pattern"] == name).cloned().unwrap_or(Value::Null);
    assert!(f(&by("Lightning")["deflection_mm"]) > f(&by("Grid")["deflection_mm"]), "{r}");
    assert!(f(&by("Concentric")["young_in_plane_gpa"]) > f(&by("Grid")["young_in_plane_gpa"]), "{r}");
}

#[test]
fn bad_print_settings_are_refused_before_any_solve() {
    let mut s = bar();
    let mut err = |id: &str, p: Value| s.execute(id, &p).map(|_| ()).unwrap_err().to_string();
    let ok = json!({"fixed": [0, 5, 5], "load": [80, 5, 5], "force_n": [0, 0, -20]});
    let with = |k: &str, v: Value| {
        let mut p = ok.clone();
        p[k] = v;
        p
    };
    assert!(err("inspect.printed_stress", json!({"load": [80, 5, 5], "force_n": [0, 0, -20]})).contains("fixed"));
    assert!(err("inspect.printed_stress", json!({"fixed": [0, 5, 5], "force_n": [0, 0, -20]})).contains("load"));
    assert!(err("inspect.printed_stress", with("force_n", json!([0, 0, 0]))).contains("force_n"));
    assert!(err("inspect.printed_stress", with("up", json!([0, 0, 0]))).contains("up"));
    assert!(err("inspect.printed_stress", with("pattern", json!("Spiral"))).contains("Gyroid"));
    assert!(err("inspect.printed_stress", with("material", json!("Unobtainium"))).contains("PETG"));
    assert!(err("inspect.printed_stress", with("infill", json!(1.5))).contains("infill"));
    assert!(err("inspect.printed_stress", with("walls", json!(-1))).contains("walls"));
    assert!(err("inspect.printed_stress", with("layer_height_mm", json!(0.0))).contains("layer_height_mm"));
    assert!(err("inspect.printed_stress", with("resolution", json!(1000))).contains("resolution"));
    assert!(err("inspect.printed_stress", with("body", json!("Nothing"))).contains("Nothing"));
    assert!(err("inspect.compare_infill", with("force_n", json!("lots"))).contains("force_n"));
    assert!(err("inspect.compare_infill", with("material", json!(7))).contains("filament"));
    // Nothing changed.
    assert_eq!(s.world_state().bodies.len(), 1);
}
