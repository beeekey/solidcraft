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

fn bounds(s: &Session, body: &str) -> solidcraft_geom::Aabb3 {
    s.world_state().body(body).map(|b| b.mesh().bounds()).unwrap_or_default()
}

#[test]
fn an_end_cap_covers_the_section_at_the_chosen_end() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 100, "width": 20, "height": 30, "body_name": "Beam"}));
    let r = run(&mut s, "design.frame_end_cap", json!({"member": "Beam", "thickness_mm": 5}));
    assert_eq!(r["end"], "end", "{r}");
    assert_eq!(r["caps"], 1);
    let section: Vec<f64> = r["section_mm"].as_array().map(|a| a.iter().map(f).collect()).unwrap_or_default();
    assert!((section[0] * section[1] - 600.0).abs() < 1e-6, "{r}");
    assert!((volume(&s, "Beam") - 63_000.0).abs() < 1e-3, "{}", volume(&s, "Beam"));
    let bb = bounds(&s, "Beam");
    assert!((bb.max.x - 105.0).abs() < 1e-6 && bb.min.x.abs() < 1e-6, "{bb:?}");
    let r = run(&mut s, "design.frame_end_cap", json!({"member": "Beam", "thickness_mm": 5, "at": [0, 10, 15]}));
    assert_eq!(r["end"], "start");
    assert!((bounds(&s, "Beam").min.x + 5.0).abs() < 1e-6);
    let mut err = |p: Value| s.execute("design.frame_end_cap", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"member": "Beam", "end": "middle"})).contains("end"));
    assert!(err(json!({"member": "Nothing"})).contains("Nothing"));
}

#[test]
fn a_notch_cuts_one_member_around_another_and_keeps_the_tool() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 100, "width": 20, "height": 20, "body_name": "Rail"}));
    run(&mut s, "solid.box", json!({"length": 20, "width": 100, "height": 20, "corner": [40, -40, 10], "body_name": "Post"}));
    let r = run(&mut s, "design.frame_notch", json!({"member": "Rail", "tool": "Post"}));
    assert_eq!(r["tool"], "Post");
    assert!((volume(&s, "Rail") - (40_000.0 - 20.0 * 20.0 * 10.0)).abs() < 1e-3, "{}", volume(&s, "Rail"));
    assert!((volume(&s, "Post") - 40_000.0).abs() < 1e-3, "the tool is kept whole");
    assert!(s.execute("design.frame_notch", &json!({"member": "Rail", "tool": "Rail"})).unwrap_err().to_string().contains("different"));
    assert!(s.execute("design.frame_notch", &json!({"member": "Rail"})).unwrap_err().to_string().contains("tool"));
}

#[test]
fn a_member_is_rebuilt_on_a_new_section_and_grows_or_shrinks_at_one_end() {
    let mut s = Session::default();
    run(
        &mut s,
        "design.frame",
        json!({"profile": {"family": "square_tube", "size": "40x40x3"}, "members": [{"from": [0, 0, 0], "to": [300, 0, 0]}], "ends": "butt", "name": "F"}),
    );
    let member = "F member 1";
    let before = volume(&s, member);
    let r = run(&mut s, "design.frame_change_section", json!({"member": member, "profile": {"family": "rect_tube", "a": 60, "b": 30, "t": 3}}));
    assert!((f(&r["length_mm"]) - 300.0).abs() < 1e-6, "{r}");
    let new = r["replaced_by"].as_str().unwrap_or("");
    assert!(new.contains("resection"), "{r}");
    let after = volume(&s, new);
    // 60 × 30 × 3 wall: (60·30 − 54·24) × 300.
    assert!((after - (1800.0 - 1296.0) * 300.0).abs() < 0.02 * after, "{after} (was {before})");
    let bb = bounds(&s, new);
    assert!(bb.min.x.abs() < 1e-6 && (bb.max.x - 300.0).abs() < 1e-6, "{bb:?}");

    let mut s2 = Session::default();
    run(&mut s2, "solid.box", json!({"length": 100, "width": 20, "height": 30, "body_name": "Beam"}));
    let r = run(&mut s2, "design.frame_extend", json!({"member": "Beam", "amount_mm": 10}));
    assert!((f(&r["length_mm"]) - 110.0).abs() < 1e-9, "{r}");
    assert!((bounds(&s2, "Beam").max.x - 110.0).abs() < 1e-6);
    let r = run(&mut s2, "design.frame_extend", json!({"member": "Beam", "amount_mm": -15, "end": "start"}));
    assert!((f(&r["length_mm"]) - 95.0).abs() < 1e-9, "{r}");
    assert!((bounds(&s2, "Beam").min.x - 15.0).abs() < 1e-6, "{:?}", bounds(&s2, "Beam"));
    assert!((volume(&s2, "Beam") - 95.0 * 600.0).abs() < 1e-3);
    let mut err = |p: Value| s2.execute("design.frame_extend", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"member": "Beam"})).contains("amount_mm"));
    assert!(err(json!({"member": "Beam", "amount_mm": -500})).contains("whole"));
}

#[test]
fn a_groove_weld_fills_the_gap_between_two_facing_plates() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 30, "width": 20, "height": 10, "body_name": "A"}));
    run(&mut s, "solid.box", json!({"length": 30, "width": 20, "height": 10, "corner": [32, 0, 0], "body_name": "B"}));
    let r = run(&mut s, "design.groove_weld", json!({"face_a": [30, 10, 5], "face_b": [32, 10, 5]}));
    assert!((f(&r["gap_mm"]) - 2.0).abs() < 1e-6, "{r}");
    assert!((f(&r["area_mm2"]) - 200.0).abs() < 1e-6, "{r}");
    let bead = r["body"].as_str().unwrap_or("");
    assert_eq!(s.world_state().bodies.len(), 3);
    assert!((volume(&s, bead) - 400.0).abs() < 1e-3, "{}", volume(&s, bead));
    let bb = bounds(&s, bead);
    assert!((bb.min.x - 30.0).abs() < 1e-6 && (bb.max.x - 32.0).abs() < 1e-6, "{bb:?}");
    let mut err = |p: Value| s.execute("design.groove_weld", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"face_a": [30, 10, 5]})).contains("face_b"));
    assert!(err(json!({"face_a": [30, 10, 5], "face_b": [47, 20, 5]})).contains("parallel"));
    assert!(err(json!({"face_a": [30, 10, 5], "face_b": [15, 10, 10]})).contains("parallel"));
    assert!(err(json!({"face_a": [30, 10, 5], "face_b": [0, 10, 5]})).contains("behind"));
}

#[test]
fn a_mitre_closes_two_members_on_the_plane_halving_their_corner() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 100, "width": 20, "height": 20, "body_name": "A"}));
    run(&mut s, "solid.box", json!({"length": 20, "width": 100, "height": 20, "body_name": "B"}));
    let r = run(&mut s, "design.frame_mitre", json!({"member_a": "A", "member_b": "B"}));
    let corner: Vec<f64> = r["corner"].as_array().map(|a| a.iter().map(f).collect()).unwrap_or_default();
    assert!((corner[0] - 10.0).abs() < 1e-6 && (corner[1] - 10.0).abs() < 1e-6, "{r}");
    // Each loses the half of the 20 × 20 overlap beyond the diagonal.
    assert!((volume(&s, "A") - 36_000.0).abs() < 1.0, "{}", volume(&s, "A"));
    assert!((volume(&s, "B") - 36_000.0).abs() < 1.0, "{}", volume(&s, "B"));
    let a = s.world_state().body("A").map(|b| b.mesh()).unwrap_or_default();
    assert!(a.inside_index().contains(solidcraft_geom::Vec3::new(15.0, 5.0, 10.0)), "A keeps x > y");
    assert!(!a.inside_index().contains(solidcraft_geom::Vec3::new(5.0, 15.0, 10.0)), "A gives up x < y");
    let mut err = |p: Value| s.execute("design.frame_mitre", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"member_a": "A", "member_b": "A"})).contains("different"));
    run(&mut s, "solid.box", json!({"length": 100, "width": 20, "height": 20, "corner": [0, 200, 0], "body_name": "C"}));
    assert!(s.execute("design.frame_mitre", &json!({"member_a": "A", "member_b": "C"})).unwrap_err().to_string().contains("parallel"));
}
