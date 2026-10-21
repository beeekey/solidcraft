use serde_json::{Value, json};

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

/// Volume (mm³) and bounding box of the bodies of the component named `prefix`.
fn measure(s: &Session, prefix: &str) -> (f64, solidcraft_geom::Vec3, solidcraft_geom::Vec3) {
    let st = s.world_state();
    let comp = s.doc.components.iter().find(|c| c.name.starts_with(prefix)).map(|c| c.id).unwrap_or(0);
    let (mut volume, mut lo, mut hi) =
        (0.0, solidcraft_geom::Vec3::new(f64::MAX, f64::MAX, f64::MAX), solidcraft_geom::Vec3::new(f64::MIN, f64::MIN, f64::MIN));
    for b in st.bodies.iter().filter(|b| s.doc.model_body_component(b) == comp) {
        volume += solidcraft_kernel::measure(&b.body).map(|m| m.volume).unwrap_or(0.0);
        let m = b.mesh().bounds();
        lo = lo.min(m.min);
        hi = hi.max(m.max);
    }
    (volume, lo, hi)
}

#[test]
fn a_spur_gear_has_the_right_size_and_about_the_pitch_cylinders_volume() {
    // m2 z20: pitch Ø40, tip Ø44, root Ø35. Teeth take about as much as they leave, so the
    // volume sits near the pitch cylinder's: π·20²·10 = 12 566 mm³.
    let mut s = Session::default();
    let r = run(&mut s, "design.spur_gear", json!({"module": 2, "teeth": 20, "width": 10, "bore_mm": 8}));
    assert_eq!(r["tip_diameter_mm"], 44.0, "{r}");
    assert_eq!(r["pitch_diameter_mm"], 40.0);
    let (volume, lo, hi) = measure(&s, "Spur gear");
    assert!((hi.x - lo.x - 44.0).abs() < 0.2 && (hi.y - lo.y - 44.0).abs() < 0.2, "{lo:?} {hi:?}");
    assert!((hi.z - lo.z - 10.0).abs() < 1e-6);
    let pitch_cylinder = std::f64::consts::PI * 400.0 * 10.0 - std::f64::consts::PI * 16.0 * 10.0;
    assert!((volume - pitch_cylinder).abs() < 0.12 * pitch_cylinder, "{volume} vs {pitch_cylinder}");
}

#[test]
fn a_helical_gear_keeps_the_spur_gears_volume_and_a_herringbone_too() {
    let mut s = Session::default();
    run(&mut s, "design.spur_gear", json!({"module": 2, "teeth": 20, "width": 10, "name": "Spur"}));
    run(&mut s, "design.spur_gear", json!({"module": 2, "teeth": 20, "width": 10, "helix_deg": 20, "name": "Helical"}));
    run(&mut s, "design.spur_gear", json!({"module": 2, "teeth": 20, "width": 10, "helix_deg": 20, "herringbone": true, "name": "Herring"}));
    let (spur, ..) = measure(&s, "Spur");
    let (helical, _, hi) = measure(&s, "Helical");
    let (herring, ..) = measure(&s, "Herring");
    assert!((helical - spur).abs() < 0.03 * spur, "helical {helical} vs spur {spur}");
    assert!((herring - spur).abs() < 0.03 * spur, "herringbone {herring} vs spur {spur}");
    assert!((hi.z - 10.0).abs() < 1e-6);
}

#[test]
fn a_ring_gear_is_a_rim_with_the_teeth_inside() {
    let mut s = Session::default();
    let r = run(&mut s, "design.spur_gear", json!({"module": 2, "teeth": 30, "width": 8, "internal": true, "rim_mm": 5}));
    assert_eq!(r["kind"], "Ring gear", "{r}");
    let (volume, lo, hi) = measure(&s, "Ring gear");
    // Outer Ø = root Ø 65 + 2·5 = 75; the opening is about the pitch circle Ø60.
    assert!((hi.x - lo.x - 75.0).abs() < 0.2, "{lo:?} {hi:?}");
    let annulus = std::f64::consts::PI * (37.5_f64.powi(2) - 30.0_f64.powi(2)) * 8.0;
    assert!((volume - annulus).abs() < 0.15 * annulus, "{volume} vs {annulus}");
}

#[test]
fn a_rack_and_a_bevel_gear_build() {
    let mut s = Session::default();
    let r = run(&mut s, "design.gear_rack", json!({"module": 2, "teeth": 10, "width": 8}));
    assert!((r["length_mm"].as_f64().unwrap_or(0.0) - 20.0 * std::f64::consts::PI).abs() < 1e-9, "{r}");
    let (volume, lo, hi) = measure(&s, "Rack");
    // The back (2 × module = 4) solid under the roots, teeth half the 2 + 2.5 band above them.
    let length = 20.0 * std::f64::consts::PI;
    let expected = length * 8.0 * (4.0 + 0.5 * 4.5);
    assert!((volume - expected).abs() < 0.1 * expected, "{volume} vs {expected}");
    assert!((hi.x - lo.x - length).abs() < 1e-6 && (hi.y - lo.y - 8.0).abs() < 1e-6);
    let b = run(&mut s, "design.bevel_gear", json!({"module": 2, "teeth": 20, "pitch_angle_deg": 45, "face_width": 8}));
    assert!(b["outer_cone_distance_mm"].as_f64().unwrap_or(0.0) > 28.0, "{b}");
    let (bevel, _, _) = measure(&s, "Bevel gear");
    assert!(bevel > 0.0 && bevel < std::f64::consts::PI * 22.0 * 22.0 * 8.0, "{bevel}");
}

#[test]
fn a_shaft_stacks_its_sections_with_a_keyway_and_a_groove() {
    let mut s = Session::default();
    let r = run(
        &mut s,
        "design.shaft",
        json!({"sections": [{"diameter": 20, "length": 30, "keyway": {"width": 6, "depth": 3.5}}, {"diameter": 25, "length": 40, "groove": {"width": 1.3, "depth": 1}}, {"diameter": 20, "length": 30}], "chamfer_mm": 1}),
    );
    assert_eq!(r["length_mm"], 100.0, "{r}");
    let (volume, lo, hi) = measure(&s, "Shaft");
    assert!((hi.z - lo.z - 100.0).abs() < 1e-6, "{lo:?} {hi:?}");
    let cylinders = std::f64::consts::PI * (100.0 * 30.0 + 156.25 * 40.0 + 100.0 * 30.0);
    assert!(volume < cylinders && volume > 0.95 * cylinders, "{volume} vs {cylinders}");
}

#[test]
fn springs_build_and_a_compression_spring_reports_its_rate() {
    let mut s = Session::default();
    let r = run(&mut s, "design.spring", json!({"kind": "compression", "mean_diameter": 20, "wire_diameter": 2, "coils": 6, "free_length": 50}));
    // k = G d⁴ / (8 D³ n) = 79e9 · (2e-3)⁴ / (8 · (20e-3)³ · 6) = 3.29 N/mm.
    let rate = r["rate_n_mm"].as_f64().unwrap_or(0.0);
    assert!((rate - 3.29).abs() < 0.05, "{r}");
    let (volume, lo, hi) = measure(&s, "Compression");
    assert!(volume > 0.0 && (hi.z - lo.z) > 40.0, "{volume} {lo:?} {hi:?}");
    run(&mut s, "design.spring", json!({"kind": "torsion", "mean_diameter": 15, "wire_diameter": 1.5, "coils": 4, "leg_mm": 15}));
    let (torsion, ..) = measure(&s, "Torsion");
    assert!(torsion > 0.0);
    assert!(
        s.execute("design.spring", &json!({"kind": "compression", "mean_diameter": 20, "wire_diameter": 2, "coils": 20, "free_length": 20})).is_err(),
        "coils that touch are refused"
    );
}

#[test]
fn a_gt2_pulley_has_its_catalogue_diameters_and_a_bore() {
    // GT2 20T: pitch Ø 12.73, outside Ø 12.22 (pitch line differential 0.254 each side).
    let mut s = Session::default();
    let r = run(&mut s, "design.pulley", json!({"standard": "GT2-2M", "teeth": 20, "width": 7, "bore_mm": 5, "flanges": true}));
    assert!((r["pitch_diameter_mm"].as_f64().unwrap_or(0.0) - 12.732).abs() < 0.01, "{r}");
    assert!((r["outside_diameter_mm"].as_f64().unwrap_or(0.0) - 12.224).abs() < 0.01, "{r}");
    let (volume, lo, hi) = measure(&s, "Pulley GT2-2M 20T");
    // The flanges set the width: Ø 12.22 + 2 × 1.5 flange, 7 + 2 × 1.5 tall.
    assert!((hi.x - lo.x - 15.22).abs() < 0.1 && (hi.z - lo.z - 10.0).abs() < 1e-6, "{lo:?} {hi:?}");
    let solid_cylinder = std::f64::consts::PI * 6.11 * 6.11 * 7.0;
    assert!(volume > 0.4 * solid_cylinder && volume < 1.6 * solid_cylinder, "{volume} vs {solid_cylinder}");
}

#[test]
fn a_two_pulley_belt_measures_like_the_handbook_and_builds_a_loop() {
    // Two GT2 20T pulleys 100 mm apart: L = 2C + πD = 200 + 40.0 → 240 mm = 120 teeth.
    let mut s = Session::default();
    let wheels = json!([{"centre": [0, 0], "teeth": 20}, {"centre": [100, 0], "teeth": 20}]);
    let r = run(&mut s, "design.belt", json!({"standard": "GT2-2M", "width": 6, "wheels": wheels, "build": false}));
    assert!((r["length_mm"].as_f64().unwrap_or(0.0) - 240.0).abs() < 0.05, "{r}");
    assert_eq!(r["teeth"], 120, "{r}");
    assert_eq!(r["teeth_in_mesh"], json!([10, 10]), "{r}");
    assert!(r.get("component").is_none());
    let built = run(&mut s, "design.belt", json!({"standard": "GT2-2M", "width": 6, "wheels": wheels}));
    assert!(built["component"].as_u64().unwrap_or(0) > 0, "{built}");
    let (volume, lo, hi) = measure(&s, "Belt GT2-2M");
    // The loop spans the centres plus the cord radius (6.37) and half the belt (0.76) each side,
    // 6 mm wide in Z.
    assert!((hi.x - lo.x - 114.26).abs() < 0.5 && (hi.z - lo.z - 6.0).abs() < 1e-6, "{lo:?} {hi:?}");
    let ring = 240.0 * 1.52 * 6.0;
    assert!((volume - ring).abs() < 0.15 * ring, "{volume} vs {ring}");
}

#[test]
fn a_belt_that_cannot_exist_is_refused() {
    let mut s = Session::default();
    let wheels = json!([{"centre": [0, 0], "teeth": 60}, {"centre": [10, 0], "teeth": 60}]);
    let err = s.execute("design.belt", &json!({"standard": "GT2-2M", "width": 6, "wheels": wheels})).err().map(|e| e.to_string()).unwrap_or_default();
    assert!(!err.is_empty(), "overlapping pulleys must be refused");
    assert!(s.execute("design.belt", &json!({"standard": "no such", "width": 6, "wheels": wheels})).is_err());
}

#[test]
fn a_sprocket_and_a_chain_follow_the_chain_standard() {
    // 08B 17T: pitch Ø = 12.7 / sin(π/17) = 69.11 mm; the chain for two 17T sprockets 200 mm apart
    // is 2·200/12.7 + 17 = 48.5 → 50 links (even).
    let mut s = Session::default();
    let r = run(&mut s, "design.sprocket", json!({"standard": "08B", "teeth": 17, "bore_mm": 10}));
    assert!((r["pitch_diameter_mm"].as_f64().unwrap_or(0.0) - 69.11).abs() < 0.05, "{r}");
    let (volume, lo, hi) = measure(&s, "Sprocket 08B 17T");
    // The tooth tips lose a little to the opening flanks' cut.
    assert!((hi.x - lo.x - r["tip_diameter_mm"].as_f64().unwrap_or(0.0)).abs() < 1.0, "{lo:?} {hi:?} {r}");
    assert!(volume > 0.0);
    let c = run(
        &mut s,
        "design.chain",
        json!({"standard": "08B", "sprockets": [{"centre": [0, 0], "teeth": 17}, {"centre": [200, 0], "teeth": 17}], "build": false}),
    );
    assert_eq!(c["links"], 50, "{c}");
    assert!((c["length_mm"].as_f64().unwrap_or(0.0) - 635.0).abs() < 1e-6);
    let built = run(
        &mut s,
        "design.chain",
        json!({"standard": "08B", "sprockets": [{"centre": [0, 0], "teeth": 17}, {"centre": [200, 0], "teeth": 17}], "links": 50}),
    );
    assert!(built["component"].as_u64().unwrap_or(0) > 0, "{built}");
    let st = s.world_state();
    let chain_body = st.bodies.iter().find(|b| b.name.starts_with("Chain 08B")).map(|b| b.mesh().bounds());
    let bounds = chain_body.unwrap_or_default();
    assert!((bounds.max.x - bounds.min.x) > 260.0 && (bounds.max.x - bounds.min.x) < 290.0, "{bounds:?}");
}

#[test]
fn a_tslot_profile_is_its_series_square_with_a_bore() {
    let mut s = Session::default();
    let r = run(&mut s, "design.tslot", json!({"series": 20, "length": 100}));
    assert_eq!(r["bore_mm"], 4.2, "{r}");
    let (volume, lo, hi) = measure(&s, "T-slot 2020");
    assert!((hi.x - lo.x - 20.0).abs() < 1e-6 && (hi.y - lo.y - 20.0).abs() < 1e-6 && (hi.z - lo.z - 100.0).abs() < 1e-6, "{lo:?} {hi:?}");
    // Slots and bore take a good part of the square: an extruded 2020 is 160 to 200 mm² of 400.
    assert!(volume > 120.0 * 100.0 && volume < 300.0 * 100.0, "{volume} {r}");
    assert!(s.execute("design.tslot", &json!({"series": 25, "length": 100})).is_err());
}

#[test]
fn a_stepper_motor_is_a_nema_frame_with_boss_and_shaft() {
    let mut s = Session::default();
    let r = run(&mut s, "design.stepper_motor", json!({"frame": "NEMA 17"}));
    assert_eq!(r["body_mm"], 42.3, "{r}");
    assert_eq!(r["bolt_square_mm"], 31.0);
    let (volume, lo, hi) = measure(&s, "NEMA 17");
    assert!((hi.x - lo.x - 42.3).abs() < 1e-6 && (lo.z + 40.0).abs() < 1e-6 && (hi.z - 26.0).abs() < 1e-6, "{lo:?} {hi:?}");
    // The body, plus the Ø22 × 2 boss and the Ø5 × 24 shaft less its flat: about 1230 mm³ more.
    assert!(volume > 42.3 * 42.3 * 40.0 + 1000.0 && volume < 42.3 * 42.3 * 40.0 + 1300.0, "{volume}");
}

#[test]
fn a_snap_hook_and_an_insert_boss_build_to_their_sizes() {
    let mut s = Session::default();
    run(&mut s, "design.snap_hook", json!({"width": 6, "arm_length": 20, "thickness": 2, "hook_depth": 1.5, "hook_length": 3}));
    let (volume, lo, hi) = measure(&s, "Snap hook");
    assert!((hi.x - lo.x - 23.0).abs() < 1e-6 && (hi.y - lo.y - 3.5).abs() < 1e-6 && (hi.z - lo.z - 6.0).abs() < 1e-6, "{lo:?} {hi:?}");
    // Arm 20 × 2 plus the hook: a 3 × 2 block and a 3 × 1.5 / 2 triangle, all 6 deep.
    assert!((volume - (40.0 + 6.0 + 2.25) * 6.0).abs() < 1e-3, "{volume}");
    let r = run(&mut s, "design.insert_boss", json!({"size": "M3", "height": 10}));
    assert_eq!(r["hole_diameter_mm"], 4.0, "{r}");
    let (boss, lo, hi) = measure(&s, "Insert boss M3");
    assert!((hi.x - lo.x - 8.0).abs() < 0.05 && (hi.z - lo.z - 10.0).abs() < 1e-6, "{lo:?} {hi:?}");
    let expected = std::f64::consts::PI * (16.0 * 10.0 - 4.0 * 7.2);
    assert!((boss - expected).abs() < 0.01 * expected, "{boss} vs {expected}");
    assert!(s.execute("design.insert_boss", &json!({"size": "M3", "height": 5})).is_err(), "too short for the insert");
}

#[test]
fn a_bamboo_culm_is_a_tapered_tube_with_nodes() {
    let mut s = Session::default();
    let r = run(
        &mut s,
        "design.bamboo_culm",
        json!({"length": 500, "butt_diameter": 32, "tip_diameter": 28, "butt_wall": 8, "tip_wall": 7, "node_spacing": 250, "first_node": 125}),
    );
    assert_eq!(r["nodes"], 2, "{r}");
    let (volume, lo, hi) = measure(&s, "Bamboo culm");
    // Butt Ø32 plus the collars 1.5 proud at the first node (Ø31 + 3 there), 500 long.
    assert!((hi.z - lo.z - 500.0).abs() < 1e-6 && (hi.x - lo.x - 34.0).abs() < 0.1, "{lo:?} {hi:?}");
    let wall = r["wall_volume_mm3"].as_f64().unwrap_or(0.0);
    assert!(volume > wall && volume < wall * 1.25, "{volume} vs the bare wall {wall}");
    assert!(s.execute("design.bamboo_culm", &json!({"length": 500, "butt_diameter": 28, "tip_diameter": 32, "butt_wall": 8})).is_err());
}

#[test]
fn brickify_covers_a_box_with_bricks_and_reports_the_bill() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 32, "width": 16, "height": 19.2, "body_name": "Block"}));
    let r = run(&mut s, "construct.brickify", json!({"body": "Block", "profiles": false, "rods": false}));
    assert!(r["coverage"].as_f64().unwrap_or(0.0) > 0.9, "{r}");
    let bom = r["bill_of_materials"].as_object().cloned().unwrap_or_default();
    assert!(!bom.is_empty(), "{r}");
    assert!(r["built"]["name"].as_str().unwrap_or("").contains("kit") || r["built"].is_object(), "{r}");
    let st = s.world_state();
    assert!(st.bodies.iter().any(|b| b.name.contains("kit")), "{:?}", st.bodies.iter().map(|b| b.name.clone()).collect::<Vec<_>>());
    assert!(s.execute("construct.brickify", &json!({"body": "Nothing"})).is_err());
    // The systems are chosen by name, as the dialog words them or as a script would.
    let r = run(
        &mut s,
        "construct.brickify",
        json!({"body": "Block", "brick": "Plates (8 mm studs, 3.2 mm thin)", "profiles": false, "rods": false, "bolts": false, "build": false}),
    );
    assert_eq!(r["brick"], "plate");
    // Six plate layers: the even ones take a 2x4 lengthways, the odd ones two 2x2 crossways.
    assert_eq!(r["bill_of_materials"]["Plate 2x4"], 3, "{r}");
    assert_eq!(r["bill_of_materials"]["Plate 2x2"], 6, "{r}");
    let r = run(
        &mut s,
        "construct.brickify",
        json!({"body": "Block", "brick": "large", "profile_series": "3030", "rod": "M10", "bolt": 5, "build": false}),
    );
    assert_eq!(r["brick"], "large");
    assert_eq!(r["profile_series"], 30);
    assert_eq!(r["rod"], "M10");
    assert_eq!(r["bolt"], "M5");
    assert_eq!(r["bill_of_materials"]["Large brick 1x2"], 1, "{r}");
    // Too thin for a 3030 run and too short for a rod or a bolt: the reply says so.
    let warnings = r["warnings"].as_array().cloned().unwrap_or_default();
    assert!(warnings.iter().any(|w| w.as_str().unwrap_or("").contains("T-slot 3030")), "{r}");
    assert!(warnings.iter().any(|w| w.as_str().unwrap_or("").contains("M10")), "{r}");
    let mut err = |p: Value| s.execute("construct.brickify", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"body": "Block", "brick": "duplo"})).contains("brick, plate, large"));
    assert!(err(json!({"body": "Block", "profile_series": 25})).contains("20, 30, 40"));
    assert!(err(json!({"body": "Block", "rod": "M7"})).contains("6, 8, 10, 12"));
    assert!(err(json!({"body": "Block", "bolt": 2.5})).contains("3, 4, 5, 6, 8"));
}

/// How long each command that runs on the design's only body takes on a small box: what the
/// hostile-input fuzz pays thousands of times over. Run with `--ignored --nocapture`.
#[test]
#[ignore]
fn body_defaulting_commands_are_cheap_on_a_small_box() {
    for id in [
        "inspect.machining",
        "inspect.print",
        "inspect.print_orient",
        "construct.brickify",
        "inspect.stress",
        "inspect.thermal",
        "inspect.flow",
        "generative.check",
        "inspect.fibre",
        "inspect.printed_stress",
        "inspect.compare_infill",
        "print.elephant_foot",
        "print.hole_compensation",
        "print.split",
        "plastic.grill",
    ] {
        let mut s = Session::default();
        let _ = s.execute("solid.box", &json!({"length": 10, "width": 10, "height": 10}));
        let start = std::time::Instant::now();
        let r = s.execute(id, &json!({"plane": "XY"}));
        eprintln!("{id}: {:.3} s ({})", start.elapsed().as_secs_f64(), if r.is_ok() { "ok" } else { "err" });
    }
}
