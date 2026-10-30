use serde_json::{Value, json};

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn refuse(s: &mut Session, id: &str, p: Value) -> String {
    match s.execute(id, &p) {
        Ok(v) => panic!("{id} {p} was accepted: {v}"),
        Err(e) => e.to_string(),
    }
}

/// A 40 mm cube named Box.
fn with_box() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 40, "height": 40, "operation": "new", "body_name": "Box"}));
    s
}

/// Signed volume (mm³) and bounds of the body named `name`, from its mesh (a mesh body's mesh is
/// unwelded, so `mass_properties` says nothing; the signed-volume sum does).
fn measure(s: &Session, name: &str) -> (f64, solidcraft_geom::Vec3, solidcraft_geom::Vec3) {
    let st = s.world_state();
    let b = st.bodies.iter().find(|b| b.name == name).unwrap_or_else(|| panic!("no body {name}"));
    let m = b.mesh();
    let bounds = m.bounds();
    (m.measure().volume, bounds.min, bounds.max)
}

#[test]
fn a_gyroid_fills_a_box_with_a_mesh_body_of_the_box_size() {
    let mut s = with_box();
    let r = run(
        &mut s,
        "construct.lattice",
        json!({"body": "Box", "family": "walled_tpms", "cell": "gyroid", "cell_mm": 10, "thickness_mm": 1.5, "samples_per_cell": 10}),
    );
    assert_eq!(r["family"], "walled_tpms");
    assert_eq!(r["cell"], "Gyroid");
    let fraction = r["volume_fraction"].as_f64().unwrap_or(0.0);
    assert!(fraction > 0.1 && fraction < 0.4, "{r}");
    assert!(r["triangles"].as_u64().unwrap_or(0) > 1000, "{r}");
    let (volume, lo, hi) = measure(&s, "Box lattice");
    assert!(volume > 0.1 * 64_000.0 && volume < 0.4 * 64_000.0, "{volume}");
    assert!((volume / 64_000.0 - fraction).abs() < 0.02, "{volume} vs {fraction}");
    for (a, b) in [(lo.x, 0.0), (lo.y, 0.0), (lo.z, 0.0), (hi.x, 40.0), (hi.y, 40.0), (hi.z, 40.0)] {
        assert!((a - b).abs() < 0.6, "{lo:?} {hi:?}");
    }
    // The source is still there and visible unless asked otherwise.
    assert!(s.world_state().bodies.iter().any(|b| b.name == "Box"));
}

#[test]
fn a_skin_raises_the_fraction_and_hide_hides_the_source() {
    let mut s = with_box();
    let bare = run(&mut s, "construct.lattice", json!({"body": "Box", "family": "walled_tpms", "cell_mm": 10, "thickness_mm": 1.5, "name": "Bare"}));
    let skinned = run(
        &mut s,
        "construct.lattice",
        json!({"body": "Box", "family": "walled_tpms", "cell_mm": 10, "thickness_mm": 1.5, "skin_mm": 2, "name": "Skinned", "hide": true}),
    );
    let (a, b) = (bare["volume_fraction"].as_f64().unwrap_or(0.0), skinned["volume_fraction"].as_f64().unwrap_or(0.0));
    assert!(b > a + 0.15, "{a} {b}");
    assert_eq!(skinned["skin_mm"], 2.0);
    assert!(s.visibility.hidden_bodies.iter().any(|b| b == "Box"), "{:?}", s.visibility.hidden_bodies);
    assert!(!s.visibility.hidden_bodies.iter().any(|b| b == "Skinned"));
}

#[test]
fn every_family_builds_and_reports_its_cell() {
    let mut s = with_box();
    let cases = [
        (json!({"family": "solid_tpms", "cell": "schwarz_p", "cell_mm": 10, "level": 0.3}), "Schwarz P"),
        (json!({"family": "beam", "cell": "octet", "cell_mm": 10, "thickness_mm": 2}), "Octet"),
        (json!({"family": "honeycomb", "cell_mm": 8, "thickness_mm": 1, "axis": "X"}), "Hexagonal"),
    ];
    for (i, (mut p, cell)) in cases.into_iter().enumerate() {
        p["body"] = json!("Box");
        p["name"] = json!(format!("Fill {i}"));
        let r = run(&mut s, "construct.lattice", p);
        assert_eq!(r["cell"], cell, "{r}");
        let f = r["volume_fraction"].as_f64().unwrap_or(0.0);
        assert!(f > 0.05 && f < 0.95, "{r}");
        let (volume, _, _) = measure(&s, &format!("Fill {i}"));
        assert!(volume > 0.0, "{volume}");
    }
}

#[test]
fn refusals_name_what_is_missing() {
    let mut s = with_box();
    assert!(refuse(&mut s, "construct.lattice", json!({"body": "Box", "cell_mm": 10, "thickness_mm": 1})).contains("`family`"));
    assert!(refuse(&mut s, "construct.lattice", json!({"body": "Box", "family": "walled_tpms", "thickness_mm": 1})).contains("`cell_mm`"));
    assert!(refuse(&mut s, "construct.lattice", json!({"body": "Box", "family": "walled_tpms", "cell_mm": 10})).contains("`thickness_mm`"));
    assert!(
        refuse(&mut s, "construct.lattice", json!({"body": "Box", "family": "walled_tpms", "cell": "swirl", "cell_mm": 10, "thickness_mm": 1}))
            .contains("gyroid")
    );
    assert!(
        refuse(&mut s, "construct.lattice", json!({"body": "Box", "family": "beam", "cell": "swirl", "cell_mm": 10, "thickness_mm": 1}))
            .contains("octet")
    );
    assert!(refuse(&mut s, "construct.lattice", json!({"body": "Nope", "family": "beam", "cell_mm": 10, "thickness_mm": 1})).contains("Nope"));
    assert!(
        refuse(&mut s, "construct.lattice", json!({"body": "Box", "family": "honeycomb", "cell_mm": 10, "thickness_mm": 1, "axis": "W"}))
            .contains("axis")
    );
    assert!(
        refuse(
            &mut s,
            "construct.lattice",
            json!({"body": "Box", "family": "walled_tpms", "cell_mm": 10, "thickness_mm": 1, "grading": "by_stress"})
        )
        .contains("`fixed`")
    );
    assert!(
        refuse(&mut s, "construct.lattice", json!({"body": "Box", "family": "solid_tpms", "cell_mm": 10, "grading": "along_axis"}))
            .contains("solid TPMS")
    );
    // Nothing was built by any of them.
    assert_eq!(s.world_state().bodies.len(), 1);
}

#[test]
fn grading_by_stress_thickens_the_walls_at_the_clamp() {
    // A cantilever 80 long, held at x = 0, pushed down at x = 80: bending stress is highest at
    // the clamp and nothing at the free end, so the lattice should be densest at the clamp.
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 80, "width": 20, "height": 20, "operation": "new", "body_name": "Beam"}));
    let r = run(
        &mut s,
        "construct.lattice",
        json!({
            "body": "Beam", "family": "walled_tpms", "cell_mm": 10, "thickness_mm": 1.5,
            "thickness_min_mm": 0.8, "thickness_max_mm": 3.5, "samples_per_cell": 10,
            "grading": "by_stress", "fixed": [0, 10, 10], "load": [80, 10, 10], "force_n": [0, 0, -200], "resolution": 16
        }),
    );
    let near_fixed = r["fraction_near_fixed"].as_f64().unwrap_or(0.0);
    let near_load = r["fraction_near_load"].as_f64().unwrap_or(1.0);
    assert!(near_fixed > near_load * 1.4, "{r}");
    assert!(r["peak_von_mises_mpa"].as_f64().unwrap_or(0.0) > 0.0, "{r}");
    let (volume, _, _) = measure(&s, "Beam lattice");
    assert!(volume > 0.0);
}
