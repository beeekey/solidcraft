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

fn volume(s: &Session, body: &str) -> f64 {
    s.world_state().body(body).map(|b| b.mesh().measure().volume.abs()).unwrap_or(f64::NAN)
}

#[test]
fn a_variable_shell_keeps_the_skin_and_an_offset_grows_the_body() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 30, "width": 30, "height": 30, "body_name": "Cube"}));
    let r = run(&mut s, "construct.variable_shell", json!({"body": "Cube", "thickness_mm": 3, "sample_mm": 0.75}));
    assert_eq!(r["operation"], "shell");
    assert_eq!(r["grading"], "uniform");
    let v = f(&r["volume_mm3"]);
    let expected = 27_000.0 - 24.0_f64.powi(3);
    assert!((v - expected).abs() < 0.12 * expected, "{r}");
    assert_eq!(s.world_state().bodies.len(), 2, "{r}");
    let built = r["built"]["name"].as_str().unwrap_or("Cube shell");
    assert!((volume(&s, built) - v).abs() < 1e-6 * v.max(1.0));
    // Graded along x: between the two uniform shells.
    let g = run(
        &mut s,
        "construct.variable_shell",
        json!({"body": "Cube", "thickness_mm": 3, "grading": "along_axis", "thickness_min_mm": 1.5, "thickness_max_mm": 4.5, "sample_mm": 0.75, "name": "Graded", "hide": true}),
    );
    let vg = f(&g["volume_mm3"]);
    assert!(vg > 27_000.0 - 27.0_f64.powi(3) && vg < 27_000.0 - 21.0_f64.powi(3), "{g}");
    assert!(s.world_state().body("Graded").is_some());
    let o = run(&mut s, "construct.variable_shell", json!({"body": "Cube", "operation": "offset", "thickness_mm": 2, "sample_mm": 1.0}));
    assert!(f(&o["volume_mm3"]) > 30_000.0, "{o}");
    let mut err = |p: Value| s.execute("construct.variable_shell", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"body": "Cube", "thickness_mm": 0})).contains("thickness_mm"));
    assert!(err(json!({"body": "Cube", "operation": "melt"})).contains("operation"));
    assert!(err(json!({"body": "Cube", "thickness_mm": 1, "sample_mm": 5})).contains("sample_mm"));
    assert!(err(json!({"body": "Cube", "grading": "spiral"})).contains("grading"));
    assert!(err(json!({"body": "Nothing"})).contains("Nothing"));
}

#[test]
fn a_blend_boolean_rounds_the_seam() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 30, "width": 10, "height": 10, "body_name": "A"}));
    run(&mut s, "solid.box", json!({"length": 10, "width": 10, "height": 30, "corner": [10, 0, 0], "body_name": "B"}));
    let plain = run(&mut s, "construct.blend_boolean", json!({"body": "A", "tool": "B", "blend_mm": 0, "sample_mm": 0.5, "name": "Plain"}));
    let rounded = run(&mut s, "construct.blend_boolean", json!({"body": "A", "tool": "B", "blend_mm": 3, "sample_mm": 0.5, "name": "Rounded"}));
    let (vp, vr) = (f(&plain["volume_mm3"]), f(&rounded["volume_mm3"]));
    assert!((vp - 5000.0).abs() < 0.1 * 5000.0, "{plain}");
    assert!(vr > vp + 20.0, "{vp} → {vr}");
    assert_eq!(rounded["operation"], "join");
    let cut = run(&mut s, "construct.blend_boolean", json!({"body": "A", "tool": "B", "operation": "cut", "blend_mm": 0, "sample_mm": 0.5}));
    assert!((f(&cut["volume_mm3"]) - 2000.0).abs() < 0.12 * 2000.0, "{cut}");
    assert_eq!(s.world_state().bodies.len(), 5);
    let mut err = |p: Value| s.execute("construct.blend_boolean", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"body": "A"})).contains("tool"));
    assert!(err(json!({"body": "A", "tool": "A"})).contains("different"));
    assert!(err(json!({"body": "A", "tool": "Nothing"})).contains("Nothing"));
    assert!(err(json!({"body": "A", "tool": "B", "operation": "weld"})).contains("operation"));
}

#[test]
fn the_stress_zone_is_the_hot_end_and_the_probe_reads_it() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 80, "width": 10, "height": 10, "body_name": "Bar"}));
    let load = json!({"body": "Bar", "fixed": [0, 5, 5], "load": [80, 5, 5], "force_n": [0, 0, -50], "resolution": 16});
    let z = run(&mut s, "inspect.stress_zone", load.clone());
    let (cells, total) = (z["cells"].as_u64().unwrap_or(0), z["cells_total"].as_u64().unwrap_or(0));
    assert!(cells > 0 && cells < total, "{z}");
    assert!(f(&z["threshold_mpa"]) > 0.0 && (f(&z["threshold_mpa"]) - 0.5 * f(&z["peak_mpa"])).abs() < 1e-9, "{z}");
    assert_eq!(s.world_state().bodies.len(), 2);
    let zone = s.world_state().body("Bar stress zone").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!(zone.min.x < 20.0, "the hot spot is at the root: {zone:?}");
    let mut strict = load.clone();
    strict["threshold_mpa"] = json!(1e6);
    assert!(s.execute("inspect.stress_zone", &strict).unwrap_err().to_string().contains("no cell"));

    let mut at_root = load.clone();
    at_root["at"] = json!([8, 5, 9.5]);
    let root = run(&mut s, "inspect.probe", at_root);
    let mut at_tip = load.clone();
    at_tip["at"] = json!([75, 5, 5]);
    let tip = run(&mut s, "inspect.probe", at_tip);
    assert!(f(&root["von_mises_mpa"]) > f(&tip["von_mises_mpa"]), "{root} vs {tip}");
    assert!(f(&root["fraction_of_peak"]) > 0.3, "{root}");
    assert!(root["verdict"].as_str().unwrap_or("").contains("MPa"));
    let mut outside = load.clone();
    outside["at"] = json!([40, 50, 50]);
    assert!(s.execute("inspect.probe", &outside).unwrap_err().to_string().contains("outside"));
    assert!(s.execute("inspect.probe", &load).unwrap_err().to_string().contains("at"));
    assert_eq!(s.world_state().bodies.len(), 2, "a probe changes nothing");
}

#[test]
fn the_fem_export_writes_a_complete_deck() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 10, "height": 10, "body_name": "Bar"}));
    let dir = std::env::temp_dir().join(format!("solidcraft_fem_{}", std::process::id()));
    let r = run(
        &mut s,
        "inspect.export_fem",
        json!({"body": "Bar", "fixed": [0, 5, 5], "load": [40, 5, 5], "force_n": [0, 0, -100], "resolution": 8, "path": dir.to_string_lossy()}),
    );
    assert!(r["nodes"].as_u64().unwrap_or(0) >= 81 && r["elements"].as_u64().unwrap_or(0) >= 32, "{r}");
    assert!(r["fixed_nodes"].as_u64().unwrap_or(0) > 0 && r["loaded_nodes"].as_u64().unwrap_or(0) > 0);
    let deck = std::fs::read_to_string(dir.join("case.inp")).unwrap_or_default();
    assert!(
        deck.contains("*ELEMENT, TYPE=C3D8") && deck.contains("*NSET, NSET=NFIX") && deck.contains("NLOAD, 3, -"),
        "{}",
        &deck[..deck.len().min(400)]
    );
    assert!(deck.contains("*ELASTIC"));
    assert!(dir.join("solve.sh").exists() && dir.join("README.txt").exists());
    let _ = std::fs::remove_dir_all(&dir);
    let mut err = |p: Value| s.execute("inspect.export_fem", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"body": "Bar", "fixed": [0, 5, 5], "load": [40, 5, 5], "force_n": [0, 0, -100]})).contains("path"));
    assert!(err(json!({"body": "Bar", "load": [40, 5, 5], "force_n": [0, 0, -100], "path": "x"})).contains("fixed"));
}
