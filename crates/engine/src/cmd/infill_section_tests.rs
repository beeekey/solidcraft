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
fn a_section_shows_the_walls_and_the_grid_with_a_gap_at_the_bore() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10, "body_name": "Plate"}));
    let r = run(&mut s, "inspect.infill_section", json!({"body": "Plate", "infill": 0.2, "walls": 2}));
    assert_eq!(r["axis"], "Z");
    assert!((f(&r["station_mm"]) - 5.0).abs() < 1e-9);
    assert!((f(&r["pitch_mm"]) - 4.2).abs() < 1e-9, "0.42 / 0.2 × 2 directions: {r}");
    assert_eq!(r["walls"], 8, "two loops of four sides: {r}");
    let strands = r["strands"].as_u64().unwrap_or(0);
    // 40 / 4.2 ≈ 9 lines one way and 30 / 4.2 ≈ 7 the other, within the box.
    assert!((12..=22).contains(&strands), "{r}");
    assert!(f(&r["length_mm"]) > 300.0, "{r}");
    assert_eq!(r["truncated"], false);
    // A bore cuts the strands that cross it in two.
    let mut s2 = Session::default();
    run(&mut s2, "solid.box", json!({"length": 40, "width": 30, "height": 10, "body_name": "Plate"}));
    run(&mut s2, "solid.cylinder", json!({"radius": 6, "height": 10, "base": [20, 15, 0], "operation": "cut", "targets": ["Plate"]}));
    let r2 = run(&mut s2, "inspect.infill_section", json!({"body": "Plate", "infill": 0.2, "walls": 2}));
    assert!(r2["strands"].as_u64().unwrap_or(0) > strands, "{r2}");
    assert!(f(&r2["length_mm"]) < f(&r["length_mm"]), "less lattice where the bore is");
    // Other patterns draw too; gyroid is a contour, concentric rings.
    for pattern in ["Gyroid", "Concentric", "Honeycomb", "Lightning"] {
        let r = run(&mut s, "inspect.infill_section", json!({"body": "Plate", "pattern": pattern, "axis": "x", "station_mm": 10}));
        assert_eq!(r["pattern"], pattern);
        assert!(r["strands"].as_u64().unwrap_or(0) > 0, "{r}");
    }
    let mut err = |p: Value| s.execute("inspect.infill_section", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"body": "Plate", "station_mm": 50})).contains("misses"));
    assert!(err(json!({"body": "Plate", "axis": "Q"})).contains("axis"));
    assert!(err(json!({"body": "Plate", "pattern": "Spiral"})).contains("Gyroid"));
}
