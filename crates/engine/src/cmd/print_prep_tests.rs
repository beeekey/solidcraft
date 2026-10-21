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
fn the_elephant_foot_chamfers_only_the_edges_on_the_plate() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 30, "width": 20, "height": 10, "body_name": "Block"}));
    let before = volume(&s, "Block");
    let r = run(&mut s, "print.elephant_foot", json!({"body": "Block", "distance_mm": 0.5}));
    assert_eq!(r["edges"], 4, "{r}");
    let after = volume(&s, "Block");
    // Four edges, each a 0.5 × 0.5 right triangle times its length: (30 + 20) × 2 × 0.125 = 12.5 mm³.
    assert!((before - after - 12.5).abs() < 0.5, "{before} → {after}");
    let st = s.world_state();
    let bb = st.body("Block").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!((bb.min.z).abs() < 1e-6 && (bb.max.z - 10.0).abs() < 1e-6, "the height is kept: {bb:?}");
    // Printed upside down the top edges are the first layer.
    let r = run(&mut s, "print.elephant_foot", json!({"body": "Block", "up": [0, 0, -1], "distance_mm": 0.5}));
    assert_eq!(r["edges"], 4);
    assert!((volume(&s, "Block") - (before - 25.0)).abs() < 1.0);
    let mut err = |p: Value| s.execute("print.elephant_foot", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"body": "Block", "distance_mm": 50})).contains("distance_mm"));
    assert!(err(json!({"body": "Block", "up": [0, 0, 0]})).contains("up"));
    assert!(err(json!({"body": "Nothing"})).contains("Nothing"));
}

#[test]
fn hole_compensation_widens_the_bores_and_leaves_the_bosses_alone() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 8, "body_name": "Plate"}));
    run(&mut s, "solid.cylinder", json!({"radius": 2.0, "height": 8, "base": [10, 15, 0], "operation": "cut", "targets": ["Plate"]}));
    run(&mut s, "solid.cylinder", json!({"radius": 4.0, "height": 8, "base": [25, 15, 0], "operation": "cut", "targets": ["Plate"]}));
    // A boss on top: a cylindrical face with material inside it.
    run(&mut s, "solid.cylinder", json!({"radius": 3.0, "height": 5, "base": [35, 25, 8], "operation": "join", "targets": ["Plate"]}));
    let before = volume(&s, "Plate");
    let r = run(&mut s, "print.hole_compensation", json!({"body": "Plate", "delta_mm": 0.4}));
    assert_eq!(r["bores"], 2, "{r}");
    let mut d: Vec<f64> = r["diameters_mm"].as_array().map(|a| a.iter().map(f).collect()).unwrap_or_default();
    d.sort_by(f64::total_cmp);
    assert!((d[0] - 4.0).abs() < 1e-6 && (d[1] - 8.0).abs() < 1e-6, "{d:?}");
    // Each bore grows by π (r' ² − r²) × 8 : (2.2² − 2²) and (4.2² − 4²).
    let expected = std::f64::consts::PI * 8.0 * ((2.2f64.powi(2) - 4.0) + (4.2f64.powi(2) - 16.0));
    let removed = before - volume(&s, "Plate");
    assert!((removed - expected).abs() < 0.03 * expected, "removed {removed}, expected {expected}");
    // Only the small one.
    let mut s2 = Session::default();
    run(&mut s2, "solid.box", json!({"length": 40, "width": 30, "height": 8, "body_name": "Plate"}));
    run(&mut s2, "solid.cylinder", json!({"radius": 2.0, "height": 8, "base": [10, 15, 0], "operation": "cut", "targets": ["Plate"]}));
    run(&mut s2, "solid.cylinder", json!({"radius": 4.0, "height": 8, "base": [25, 15, 0], "operation": "cut", "targets": ["Plate"]}));
    let r = run(&mut s2, "print.hole_compensation", json!({"body": "Plate", "max_diameter_mm": 5}));
    assert_eq!(r["bores"], 1, "{r}");
    // Nothing to do on a plain block.
    let mut s3 = Session::default();
    run(&mut s3, "solid.box", json!({"length": 10, "width": 10, "height": 10}));
    assert!(s3.execute("print.hole_compensation", &json!({})).unwrap_err().to_string().contains("no round bores"));
}

#[test]
fn split_for_print_leaves_two_halves_with_pins_in_one_and_sockets_in_the_other() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 50, "body_name": "Tall"}));
    let whole = volume(&s, "Tall");
    let r = run(&mut s, "print.split", json!({"body": "Tall", "pin_diameter_mm": 4, "pin_length_mm": 6, "clearance_mm": 0.3}));
    assert_eq!(r["pins"], 4, "{r}");
    assert_eq!(r["axis"], "Z");
    assert!((f(&r["station_mm"]) - 25.0).abs() < 1e-9);
    let (lower, upper) = (r["lower"].as_str().unwrap_or(""), r["upper"].as_str().unwrap_or(""));
    assert_eq!(s.world_state().bodies.len(), 2);
    let st = s.world_state();
    let lb = st.body(lower).map(|b| b.mesh().bounds()).unwrap_or_default();
    let ub = st.body(upper).map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!((lb.min.z).abs() < 1e-6 && (lb.max.z - 31.0).abs() < 1e-6, "the pins stand 6 mm above the cut: {lb:?}");
    assert!((ub.min.z - 25.0).abs() < 1e-6 && (ub.max.z - 50.0).abs() < 1e-6, "{ub:?}");
    let pin = std::f64::consts::PI * 4.0 * 6.0;
    let socket = std::f64::consts::PI * (2.15f64).powi(2) * 6.3;
    assert!((volume(&s, lower) - (whole / 2.0 + 4.0 * pin)).abs() < 5.0, "{}", volume(&s, lower));
    assert!((volume(&s, upper) - (whole / 2.0 - 4.0 * socket)).abs() < 5.0, "{}", volume(&s, upper));
    // Along x, plain, at a chosen station.
    let mut s2 = Session::default();
    run(&mut s2, "solid.box", json!({"length": 60, "width": 40, "height": 50, "body_name": "Tall"}));
    let r = run(&mut s2, "print.split", json!({"body": "Tall", "axis": "x", "station_mm": 20, "pins": false}));
    assert_eq!(r["pins"], 0);
    assert!((volume(&s2, r["lower"].as_str().unwrap_or("")) - 40.0 * 50.0 * 20.0).abs() < 1e-3);
    let mut err = |p: Value| s2.execute("print.split", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"body": "Tall", "station_mm": 100})).contains("misses"));
    assert!(err(json!({"body": "Tall", "axis": "W"})).contains("axis"));
}

#[test]
fn topology_optimisation_keeps_a_connected_lighter_body() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 10, "height": 20, "body_name": "Bar"}));
    let r = run(
        &mut s,
        "construct.topology_optimize",
        json!({"body": "Bar", "fixed": [0, 5, 10], "load": [60, 5, 10], "force_n": [0, 0, -100], "target_fraction": 0.5, "resolution": 18}),
    );
    let kept = f(&r["kept_fraction"]);
    assert!(kept > 0.3 && kept < 0.8, "{r}");
    assert_eq!(s.world_state().bodies.len(), 2, "{r}");
    let built = r["built"]["name"].as_str().unwrap_or("Bar optimised");
    let v = volume(&s, built);
    assert!(v > 0.2 * 12_000.0 && v < 0.9 * 12_000.0, "optimised volume {v}");
    let mut err = |p: Value| s.execute("construct.topology_optimize", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"body": "Bar", "load": [60, 5, 10], "force_n": [0, 0, -100]})).contains("fixed"));
    assert!(err(json!({"body": "Bar", "fixed": [0, 5, 10], "load": [60, 5, 10], "force_n": [0, 0, 0]})).contains("force_n"));
    assert!(
        err(json!({"body": "Bar", "fixed": [0, 5, 10], "load": [60, 5, 10], "force_n": [0, 0, -1], "target_fraction": 1.5}))
            .contains("target_fraction")
    );
}

#[test]
fn a_grill_cuts_slots_through_the_wall_and_can_raise_a_rim() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 3, "body_name": "Wall"}));
    let before = volume(&s, "Wall");
    let r = run(
        &mut s,
        "plastic.grill",
        json!({"face": [30, 20, 3], "width_mm": 20, "length_mm": 12, "slot_width_mm": 2, "rib_width_mm": 1.5, "rim_width_mm": 1.5}),
    );
    assert_eq!(r["body"], "Wall");
    assert!((f(&r["wall_mm"]) - 3.0).abs() < 1e-3, "{r}");
    // Inside the rim: 17 × 9; slots 2 wide at a pitch of 3.5 centred: 3 whole ones.
    assert_eq!(r["slots"], 3, "{r}");
    assert_eq!(r["rim"], false);
    let removed = before - volume(&s, "Wall");
    assert!((removed - 3.0 * 17.0 * 2.0 * 3.0).abs() < 0.5, "removed {removed}");
    // With spars and a raised rim.
    let mut s2 = Session::default();
    run(&mut s2, "solid.box", json!({"length": 60, "width": 40, "height": 3, "body_name": "Wall"}));
    let r = run(
        &mut s2,
        "plastic.grill",
        json!({"face": [30, 20, 3], "width_mm": 20, "length_mm": 12, "rim_height_mm": 1.0, "spar_width_mm": 1.0, "spar_pitch_mm": 6.0, "direction": [0, 1, 0]}),
    );
    assert_eq!(r["rim"], true, "{r}");
    assert!(r["slots"].as_u64().unwrap_or(0) > 3, "{r}");
    let bb = s2.world_state().body("Wall").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!((bb.max.z - 4.0).abs() < 1e-6, "the rim stands 1 mm proud: {bb:?}");
    assert!((f(&r["direction"][1]) - 1.0).abs() < 1e-9, "{r}");
    let mut err = |p: Value| s2.execute("plastic.grill", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"face": [500, 500, 500]})).contains("face"));
    assert!(err(json!({"face": [30, 20, 3], "width_mm": 2})).contains("too small"));
    assert!(err(json!({})).contains("face"));
}
