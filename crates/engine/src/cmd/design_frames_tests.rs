use serde_json::{Value, json};
use solidcraft_geom::Vec3;

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

/// Volume (mm³) and bounding box of every body of the component whose name starts with
/// `prefix`: the kernel's measure for a B-rep, the mesh's signed volume for a mesh body (whose
/// B-rep has no mass properties).
fn measure(s: &Session, prefix: &str) -> (f64, Vec3, Vec3, usize) {
    let st = s.world_state();
    let comp = s.doc.components.iter().find(|c| c.name.starts_with(prefix)).map(|c| c.id).unwrap_or(0);
    let (mut volume, mut lo, mut hi, mut count) = (0.0, Vec3::new(f64::MAX, f64::MAX, f64::MAX), Vec3::new(f64::MIN, f64::MIN, f64::MIN), 0);
    for b in st.bodies.iter().filter(|b| s.doc.model_body_component(b) == comp) {
        let m = b.mesh();
        match solidcraft_kernel::measure(&b.body) {
            Ok(exact) => volume += exact.volume,
            Err(_) => {
                for t in &m.triangles {
                    let (a, b2, c) = (m.positions[t[0] as usize], m.positions[t[1] as usize], m.positions[t[2] as usize]);
                    volume += a.dot(b2.cross(c)) / 6.0;
                }
            }
        }
        let bb = m.bounds();
        lo = lo.min(bb.min);
        hi = hi.max(bb.max);
        count += 1;
    }
    (volume, lo, hi, count)
}

#[test]
fn a_worm_is_a_threaded_cylinder_between_its_tip_and_root() {
    // m2, pitch Ø20, 40 long: tip Ø24, root Ø15. The thread fills about half the band between
    // root and tip, so the volume sits between the root and the tip cylinders.
    let mut s = Session::default();
    let r = run(&mut s, "design.worm", json!({"module": 2, "pitch_diameter": 20, "length": 40, "starts": 2}));
    assert_eq!(r["tip_diameter_mm"], 24.0, "{r}");
    assert_eq!(r["root_diameter_mm"], 15.0);
    assert!((r["lead_mm"].as_f64().unwrap_or(0.0) - 4.0 * std::f64::consts::PI).abs() < 1e-9);
    assert_eq!(r["mesh_body"], true);
    let (volume, lo, hi, _) = measure(&s, "Worm");
    assert!((hi.x - lo.x - 24.0).abs() < 0.3 && (hi.y - lo.y - 24.0).abs() < 0.3, "{lo:?} {hi:?}");
    assert!((hi.z - lo.z - 40.0).abs() < 1e-6);
    let (root, tip) = (std::f64::consts::PI * 7.5 * 7.5 * 40.0, std::f64::consts::PI * 144.0 * 40.0);
    assert!(volume > root && volume < tip, "{volume} between {root} and {tip}");
}

#[test]
fn a_worm_wheel_is_throated_and_turns_at_the_lead_angle() {
    // Worm m2 Ø20 single start: lead 2π, lead angle atan(2π / 20π) = 5.71°. Wheel z30: pitch Ø60,
    // centre distance 10 + 30 = 40.
    let mut s = Session::default();
    let r = run(&mut s, "design.worm_wheel", json!({"module": 2, "teeth": 30, "worm_pitch_diameter": 20, "face_width": 12, "bore_mm": 10}));
    assert!((r["lead_angle_deg"].as_f64().unwrap_or(0.0) - 5.7106).abs() < 1e-3, "{r}");
    assert_eq!(r["centre_distance_mm"], 40.0);
    assert_eq!(r["throat_diameter_mm"], 64.0);
    assert_eq!(r["ratio"], 30.0);
    let rim = r["rim_diameter_mm"].as_f64().unwrap_or(0.0);
    assert!(rim > 64.0 && rim < 68.0, "the rim at the face edges follows the worm: {rim}");
    let (volume, lo, hi, _) = measure(&s, "Worm wheel");
    assert!((hi.x - lo.x - rim).abs() < 0.3, "{lo:?} {hi:?} vs {rim}");
    assert!((hi.z - lo.z - 12.0).abs() < 1e-6);
    let pitch_cylinder = std::f64::consts::PI * (900.0 - 25.0) * 12.0;
    assert!((volume - pitch_cylinder).abs() < 0.15 * pitch_cylinder, "{volume} vs {pitch_cylinder}");
}

#[test]
fn a_worm_wheel_refuses_a_face_wider_than_the_worm() {
    let mut s = Session::default();
    let e = s
        .execute("design.worm_wheel", &json!({"module": 2, "teeth": 30, "worm_pitch_diameter": 20, "face_width": 30}))
        .err()
        .map(|e| e.to_string())
        .unwrap_or_default();
    assert!(e.contains("wider than the worm"), "{e}");
}

#[test]
fn a_crown_gear_has_teeth_in_a_band_on_its_face() {
    // m2 z30: pitch Ø60; band 10 wide: Ø50 to Ø70; disc 8 thick, teeth 4.5 deep. The band's teeth
    // take about half the band's top 4.5 mm, so the volume is the disc less about half the band's
    // toothed layer.
    let mut s = Session::default();
    let r = run(&mut s, "design.crown_gear", json!({"module": 2, "teeth": 30, "face_width": 10, "thickness": 8, "bore_mm": 8}));
    assert_eq!(r["pitch_diameter_mm"], 60.0, "{r}");
    assert_eq!(r["outer_diameter_mm"], 70.0);
    assert_eq!(r["tooth_depth_mm"], 4.5);
    let (volume, lo, hi, _) = measure(&s, "Crown gear");
    assert!((hi.x - lo.x - 70.0).abs() < 0.3 && (hi.z - lo.z - 8.0).abs() < 1e-6, "{lo:?} {hi:?}");
    let disc = std::f64::consts::PI * (35.0 * 35.0 - 16.0) * 8.0;
    let band_layer = std::f64::consts::PI * (35.0 * 35.0 - 25.0 * 25.0) * 4.5;
    let expected = disc - 0.5 * band_layer;
    assert!((volume - expected).abs() < 0.1 * disc, "{volume} vs {expected}");
}

#[test]
fn a_crown_gear_refuses_a_disc_thinner_than_its_teeth() {
    let mut s = Session::default();
    let e = s
        .execute("design.crown_gear", &json!({"module": 2, "teeth": 30, "face_width": 10, "thickness": 4}))
        .err()
        .map(|e| e.to_string())
        .unwrap_or_default();
    assert!(e.contains("deeper than the disc"), "{e}");
}

#[test]
fn a_mitred_square_tube_frame_fills_its_rectangle() {
    // Four 40x40x3 members round a 400 x 300 rectangle, mitred: the outside of the frame is
    // exactly 440 x 340 (the profile is centred on the lines), and the steel is the rectangle's
    // perimeter times the section, less the four corners counted twice.
    let mut s = Session::default();
    let r = run(
        &mut s,
        "design.frame",
        json!({
            "profile": {"family": "square_tube", "size": "40x40x3"},
            "members": [
                {"from": [0, 0, 0], "to": [400, 0, 0]},
                {"from": [400, 0, 0], "to": [400, 300, 0]},
                {"from": [400, 300, 0], "to": [0, 300, 0]},
                {"from": [0, 300, 0], "to": [0, 0, 0]}
            ]
        }),
    );
    assert_eq!(r["members"], 4, "{r}");
    assert_eq!(r["joints"]["mitred"], 4);
    assert_eq!(r["total_length_mm"], 1400.0);
    let area = r["profile"]["area_mm2"].as_f64().unwrap_or(0.0);
    assert!((area - (1600.0 - 34.0 * 34.0)).abs() < 1e-9, "{area}");
    let (volume, lo, hi, count) = measure(&s, "Frame");
    assert_eq!(count, 4);
    assert!((lo.x + 20.0).abs() < 1e-6 && (hi.x - 420.0).abs() < 1e-6, "{lo:?} {hi:?}");
    assert!((lo.y + 20.0).abs() < 1e-6 && (hi.y - 320.0).abs() < 1e-6, "{lo:?} {hi:?}");
    assert!((lo.z + 20.0).abs() < 1e-6 && (hi.z - 20.0).abs() < 1e-6, "{lo:?} {hi:?}");
    // Mitred members do not overlap: the volume is the centreline length times the area.
    let expected = 1400.0 * area;
    assert!((volume - expected).abs() < 0.01 * expected, "{volume} vs {expected}");
}

#[test]
fn a_brace_ending_on_a_member_is_trimmed_to_its_face_and_round_members_mitre_too() {
    let mut s = Session::default();
    let r = run(
        &mut s,
        "design.frame",
        json!({
            "profile": {"family": "round_tube", "size": "20x2"},
            "members": [
                {"from": [0, 0, 0], "to": [200, 0, 0]},
                {"from": [200, 0, 0], "to": [200, 0, 150]},
                {"from": [100, 0, 0], "to": [200, 0, 100]}
            ]
        }),
    );
    assert_eq!(r["joints"]["mitred"], 1, "{r}");
    assert_eq!(r["joints"]["trimmed"], 2);
    let (volume, lo, hi, count) = measure(&s, "Frame");
    assert_eq!(count, 3);
    // The first member starts square at x = 0; the upright's tube reaches x = 210.
    assert!(lo.x.abs() < 0.05 && (hi.x - 210.0).abs() < 0.05, "{lo:?} {hi:?}");
    assert!((hi.z - 150.0).abs() < 1e-6 && (lo.y + 10.0).abs() < 0.05, "{lo:?} {hi:?}");
    let area = std::f64::consts::PI * (100.0 - 64.0);
    let untrimmed = (200.0 + 150.0 + 200f64.sqrt() * 10.0) * area;
    assert!(volume < untrimmed && volume > 0.9 * untrimmed, "{volume} vs {untrimmed}");
}

#[test]
fn frame_profiles_from_the_tables_and_outright() {
    for (profile, area) in [
        (json!({"family": "angle", "size": "40x40x4"}), 40.0 * 4.0 + 36.0 * 4.0),
        (json!({"family": "channel", "size": "UNP 50"}), 38.0 * 7.0 * 2.0 + (50.0 - 14.0) * 5.0),
        (json!({"family": "i_beam", "size": "IPE 80"}), 46.0 * 5.2 * 2.0 + (80.0 - 10.4) * 3.8),
        (json!({"family": "rect_tube", "a": 60, "b": 40, "t": 3}), 2400.0 - 54.0 * 34.0),
        (json!({"family": "round_rod", "size": "10"}), std::f64::consts::PI * 25.0),
    ] {
        let mut s = Session::default();
        let r = run(&mut s, "design.frame", json!({"profile": profile, "members": [{"from": [0, 0, 0], "to": [0, 0, 100]}]}));
        let got = r["profile"]["area_mm2"].as_f64().unwrap_or(0.0);
        assert!((got - area).abs() < 1e-6, "{profile}: {got} vs {area}");
        let (volume, _, _, _) = measure(&s, "Frame");
        assert!((volume - 100.0 * area).abs() < 0.02 * 100.0 * area, "{profile}: {volume} vs {}", 100.0 * area);
    }
    let mut s = Session::default();
    let e = s
        .execute("design.frame", &json!({"profile": {"family": "angle"}, "members": [{"from": [0, 0, 0], "to": [0, 0, 100]}]}))
        .err()
        .map(|e| e.to_string())
        .unwrap_or_default();
    assert!(e.contains("profile.size"), "{e}");
}

#[test]
fn a_pipe_route_bends_round_its_corners() {
    // Two 100 mm legs at a right angle, Ø12 x 1, bend radius 24: the bend takes 24 mm from each
    // leg and adds a quarter circle of radius 24.
    let mut s = Session::default();
    let r = run(
        &mut s,
        "design.pipe_route",
        json!({"points": [[0, 0, 0], [100, 0, 0], [100, 100, 0]], "outer_diameter": 12, "wall": 1, "bend_radius": 24}),
    );
    assert_eq!(r["bends"], 1, "{r}");
    let centreline = r["centreline_mm"].as_f64().unwrap_or(0.0);
    assert!((centreline - (152.0 + 12.0 * std::f64::consts::PI)).abs() < 1e-6, "{centreline}");
    let (volume, lo, hi, count) = measure(&s, "Pipe route");
    assert_eq!(count, 1);
    // The tube starts square at the first point and ends square at the last.
    assert!(lo.x.abs() < 0.2 && (hi.x - 106.0).abs() < 0.2, "{lo:?} {hi:?}");
    assert!((hi.y - 100.0).abs() < 0.2 && (lo.y + 6.0).abs() < 0.2 && (lo.z + 6.0).abs() < 0.2, "{lo:?} {hi:?}");
    let area = std::f64::consts::PI * (36.0 - 25.0);
    assert!((volume - centreline * area).abs() < 0.05 * centreline * area, "{volume} vs {}", centreline * area);
}

#[test]
fn a_pipe_route_refuses_a_bend_that_does_not_fit() {
    let mut s = Session::default();
    let e = s
        .execute("design.pipe_route", &json!({"points": [[0, 0, 0], [20, 0, 0], [20, 100, 0]], "outer_diameter": 12, "bend_radius": 30}))
        .err()
        .map(|e| e.to_string())
        .unwrap_or_default();
    assert!(e.contains("does not fit"), "{e}");
}

#[test]
fn a_fillet_weld_fills_the_inside_corner_of_an_l() {
    // A 40 × 40 × 5 base with a 40 × 5 × 20 wall standing on it along y = 0..5: the inside
    // corner runs along x at (y, z) = (5, 5). A 4 mm fillet there is a right triangle of 8 mm²
    // swept 40 mm: 320 mm³, lying in y = 5..9, z = 5..9.
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 40, "height": 5, "body_name": "Bracket"}));
    run(&mut s, "solid.box", json!({"length": 40, "width": 5, "height": 20, "corner": [0, 0, 5], "operation": "join", "targets": ["Bracket"]}));
    let r = run(&mut s, "design.fillet_weld", json!({"body": "Bracket", "edge": [20, 5, 5], "leg_mm": 4}));
    assert_eq!(r["length_mm"], 40.0, "{r}");
    assert_eq!(r["volume_mm3"], 320.0);
    let (volume, lo, hi, count) = measure(&s, "Fillet weld");
    assert_eq!(count, 1);
    assert!((volume - 320.0).abs() < 1.0, "{volume}");
    assert!((lo.y - 5.0).abs() < 1e-6 && (hi.y - 9.0).abs() < 1e-6, "{lo:?} {hi:?}");
    assert!((lo.z - 5.0).abs() < 1e-6 && (hi.z - 9.0).abs() < 1e-6, "{lo:?} {hi:?}");
    assert!(lo.x.abs() < 1e-6 && (hi.x - 40.0).abs() < 1e-6, "{lo:?} {hi:?}");
    // An outside corner has nothing to fill.
    let e = s
        .execute("design.fillet_weld", &json!({"body": "Bracket", "edge": [20, 40, 5], "leg_mm": 4}))
        .err()
        .map(|e| e.to_string())
        .unwrap_or_default();
    assert!(e.contains("outside corner"), "{e}");
}

#[test]
fn a_fillet_weld_runs_round_a_pipe_standing_on_a_plate() {
    // A Ø20 tube on a 60 × 60 × 5 plate: the weld runs round the foot, 3 mm legs, about
    // 4.5 mm² × π·20 = 283 mm³ (a little more outside the circle than inside).
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 60, "height": 5, "body_name": "Plate"}));
    run(&mut s, "solid.cylinder", json!({"diameter": 20, "height": 30, "base": [30, 30, 5], "operation": "join", "targets": ["Plate"]}));
    let r = run(&mut s, "design.fillet_weld", json!({"body": "Plate", "edge": [40, 30, 5], "leg_mm": 3}));
    let length = r["length_mm"].as_f64().unwrap_or(0.0);
    assert!((length - 20.0 * std::f64::consts::PI).abs() < 0.5, "{r}");
    let (volume, lo, hi, _) = measure(&s, "Fillet weld");
    assert!(volume > 250.0 && volume < 330.0, "{volume}");
    assert!((lo.z - 5.0).abs() < 1e-6 && (hi.z - 8.0).abs() < 0.05, "{lo:?} {hi:?}");
    assert!((lo.x - 17.0).abs() < 0.1 && (hi.x - 43.0).abs() < 0.1, "{lo:?} {hi:?}");
}
