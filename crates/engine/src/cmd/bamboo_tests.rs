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

fn bodies(s: &Session) -> Vec<String> {
    s.world_state().bodies.iter().map(|b| b.name.clone()).collect()
}

#[test]
fn a_connector_has_a_sleeve_and_a_bore_per_culm() {
    let mut s = Session::default();
    let r = run(
        &mut s,
        "design.bamboo_connector",
        json!({"arms": [{"direction": [1, 0, 0], "diameter_mm": 32}, {"direction": [0, 1, 0], "diameter_mm": 32, "engagement_mm": 50}, {"direction": [0, 0, 1], "diameter_mm": 24}], "name": "Muffe"}),
    );
    assert_eq!(r["arms"], 3, "{r}");
    assert!((f(&r["hub_diameter_mm"]) - 2.0 * 1.3 * 19.3).abs() < 1e-9, "{r}");
    let expected = std::f64::consts::PI * (32.0 * 64.0 + 32.0 * 50.0 + 24.0 * 48.0);
    assert!((f(&r["bond_area_mm2"]) - expected).abs() < 1e-6, "{r}");
    assert_eq!(bodies(&s), ["Muffe", "Muffe socket 1", "Muffe socket 2", "Muffe socket 3"]);
    let st = s.world_state();
    let bb = st.body("Muffe socket 1").map(|b| b.mesh().bounds()).unwrap_or_default();
    // The x sleeve reaches hub + engagement = 25.09 + 64; the bore is open at its mouth.
    assert!((bb.max.x - (25.09 + 64.0)).abs() < 0.2, "{bb:?}");
    let bb = st.body("Muffe socket 2").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!((bb.max.y - (25.09 + 50.0)).abs() < 0.2, "{bb:?}");
    let mesh = st.body("Muffe socket 1").map(|b| b.mesh()).unwrap_or_default();
    let inside = mesh.inside_index();
    assert!(!inside.contains(solidcraft_geom::Vec3::new(60.0, 0.0, 0.0)), "the bore is open");
    assert!(inside.contains(solidcraft_geom::Vec3::new(60.0, 17.0, 0.0)), "the sleeve wall is there");
    assert!(inside.contains(solidcraft_geom::Vec3::new(24.0, 0.0, 0.0)), "the bore bottoms just outside the hub");
    let mut err = |p: Value| s.execute("design.bamboo_connector", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({})).contains("arms"));
    assert!(err(json!({"arms": [{"direction": [0, 0, 0], "diameter_mm": 32}]})).contains("direction"));
    assert!(err(json!({"arms": [{"direction": [1, 0, 0]}]})).contains("diameter_mm"));
}

#[test]
fn a_bicycle_frame_has_eight_members_and_six_joints_and_a_cut_list() {
    let mut s = Session::default();
    let r = run(&mut s, "design.bamboo_frame", json!({"kind": "bicycle", "node_spacing_mm": 0, "name": "Bike"}));
    let members = r["members"].as_array().cloned().unwrap_or_default();
    assert_eq!(members.len(), 8, "{r}");
    assert_eq!(r["junctions"], 6);
    assert_eq!(bodies(&s).len(), 8 + 6 + 16, "members, hubs and sockets: {:?}", bodies(&s));
    let stay = members.iter().find(|m| m["name"] == "chain stay (left)").cloned().unwrap_or(Value::Null);
    // Centre-plane 430 with 135 dropouts: the stay itself runs to the dropout.
    assert!((f(&stay["axis_mm"]) - (430.0_f64.powi(2) - 68.0_f64.powi(2) + 67.5_f64.powi(2) + 68.0_f64.powi(2)).sqrt()).abs() < 1e-6, "{stay}");
    assert!(members.iter().all(|m| f(&m["cut_mm"]) > 0.0 && f(&m["cut_mm"]) < f(&m["axis_mm"])));
    assert_eq!(members.iter().filter(|m| m["material"] == "tube").count(), 1);
    assert!(f(&r["bond_area_mm2"]) > 0.0 && f(&r["total_cut_mm"]) > 2000.0, "{r}");
    let st = s.world_state();
    let bb = st.body("Bike head tube").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!(bb.min.x > 300.0, "the head tube is forward of the bottom bracket: {bb:?}");
    let mut err = |p: Value| s.execute("design.bamboo_frame", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({})).contains("kind"));
    assert!(err(json!({"kind": "unicycle"})).contains("kind"));
    assert!(err(json!({"kind": "bicycle", "seat_angle_deg": 30})).contains("seat_angle_deg"));
}

#[test]
fn a_trailer_frame_has_rails_cross_members_and_a_drawbar() {
    let mut s = Session::default();
    let r = run(&mut s, "design.bamboo_frame", json!({"kind": "trailer", "cross_members": 2, "node_spacing_mm": 0, "name": "Trailer"}));
    assert_eq!(r["members"].as_array().map(Vec::len), Some(8), "{r}");
    assert_eq!(r["junctions"], 9);
    assert_eq!(bodies(&s).len(), 8 + 9 + 16, "{:?}", bodies(&s));
    let st = s.world_state();
    let bb = st.body("Trailer drawbar").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!(bb.max.x > 1000.0 && bb.max.z > 100.0, "the drawbar reaches forward and up: {bb:?}");
}

#[test]
fn a_bearing_is_two_rings_and_the_balls_to_the_catalogue() {
    let mut s = Session::default();
    let r = run(&mut s, "design.bearing", json!({"designation": "6805", "center": [10, 0, 0]}));
    assert_eq!(r["bore_mm"], 25.0);
    assert_eq!(r["outer_mm"], 37.0);
    assert_eq!(r["width_mm"], 7.0);
    assert_eq!(r["dynamic_rating_n"], 4360.0);
    assert_eq!(bodies(&s).len(), 3, "{:?}", bodies(&s));
    let st = s.world_state();
    let outer = st.body("Bearing 6805 outer ring").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!((outer.max.x - 28.5).abs() < 0.05 && (outer.min.x + 8.5).abs() < 0.05, "{outer:?}");
    assert!((outer.max.z - 7.0).abs() < 1e-6);
    let inner = st.body("Bearing 6805 inner ring").map(|b| b.mesh()).unwrap_or_default();
    assert!(!inner.inside_index().contains(solidcraft_geom::Vec3::new(10.0, 0.0, 3.5)), "the bore is open");
    assert!(s.execute("design.bearing", &json!({"designation": "9999"})).unwrap_err().to_string().contains("6805"));
    assert!(s.execute("design.bearing", &json!({})).unwrap_err().to_string().contains("designation"));
}

#[test]
fn a_damper_is_drawn_to_its_mounting_dimensions() {
    let mut s = Session::default();
    let r = run(&mut s, "design.damper", json!({"eye_to_eye_mm": 165, "stroke_mm": 40, "coil_over": true, "name": "Shock"}));
    assert_eq!(r["compressed_mm"], 125.0);
    assert_eq!(bodies(&s).len(), 4, "{:?}", bodies(&s));
    let st = s.world_state();
    let body = st.body("Shock body").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!(body.min.z.abs() < 1e-6 && (body.max.z - 165.0).abs() < 1e-6, "neck to neck: {body:?}");
    assert!((body.max.x - 16.0).abs() < 0.1, "body radius 16: {body:?}");
    let eye1 = st.body("Shock eye 1").map(|b| b.mesh().bounds()).unwrap_or_default();
    let eye2 = st.body("Shock eye 2").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!((eye1.min.z + 8.0).abs() < 0.1 && (eye2.max.z - 173.0).abs() < 0.1, "eye to eye 165 plus an eye's radius each end: {eye1:?} {eye2:?}");
    let eye = st.body("Shock eye 1").map(|b| b.mesh()).unwrap_or_default();
    assert!(!eye.inside_index().contains(solidcraft_geom::Vec3::ZERO), "the eye bore is open");
    let coil = st.body("Shock coil").map(|b| b.mesh().bounds()).unwrap_or_default();
    assert!((coil.max.x - 25.6).abs() < 0.3, "coil 1.6 × 32: {coil:?}");
    let mut err = |p: Value| s.execute("design.damper", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"rod_diameter_mm": 40})).contains("rod"));
    assert!(err(json!({"stroke_mm": 200})).contains("stroke"));
    assert!(err(json!({"eye_to_eye_mm": 20, "stroke_mm": 5, "eye_bore_mm": 20})).contains("eyes"));
}

/// How long each generator takes; run with `--ignored --nocapture`.
#[test]
#[ignore]
fn bamboo_generators_timing() {
    for (id, p) in [
        ("design.bamboo_connector", json!({"arms": [{"direction": [1, 0, 0], "diameter_mm": 32}, {"direction": [0, 0, 1], "diameter_mm": 24}]})),
        ("design.bamboo_frame", json!({"kind": "bicycle", "node_spacing_mm": 0})),
        ("design.bamboo_frame", json!({"kind": "bicycle"})),
        ("design.bamboo_frame", json!({"kind": "trailer", "node_spacing_mm": 0})),
        ("design.bearing", json!({"designation": "6805"})),
        ("design.damper", json!({"coil_over": true})),
        ("design.damper", json!({})),
    ] {
        let mut s = Session::default();
        let start = std::time::Instant::now();
        let r = s.execute(id, &p);
        eprintln!("TIMING {id} {p}: {:.2} s ({})", start.elapsed().as_secs_f64(), if r.is_ok() { "ok" } else { "err" });
    }
}
