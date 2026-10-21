//! Exact counts on exactly sized fixtures.

use solidcraft_geom::{Mesh, Vec3};

use super::*;

fn box_mesh(min: Vec3, size: Vec3) -> Mesh {
    let mut mesh = Mesh::default();
    push_box(&mut mesh, min, size);
    mesh
}

fn bricks_only() -> Options {
    Options { use_profiles: false, use_rods: false, ..Default::default() }
}

#[test]
fn one_brick_body_becomes_exactly_that_brick() {
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(16.0, 16.0, 9.6)), &bricks_only()).unwrap_or_default();
    assert_eq!(plan.elements.len(), 1, "{plan:?}");
    assert_eq!(plan.elements[0].name, "Brick 2x2");
    assert_eq!(plan.bill_of_materials.get("Brick 2x2"), Some(&1));
    assert_eq!(plan.cells_total, 4);
    assert!((plan.coverage - 1.0).abs() < 1e-12);
    let mesh = plan_mesh(&plan);
    assert_eq!(mesh.triangles.len(), 12 + 4 * (2 * 14 + 2 * 16));
    assert!(mesh.mass_properties().map(|m| m.volume).unwrap_or(0.0) > 15.8 * 15.8 * 9.6);
}

#[test]
fn layers_interlock_by_alternating_orientation() {
    // 64 × 16 mm, two layers: one 2x8 on the even layer, 2x2 blocks on the odd one.
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(64.0, 16.0, 19.2)), &bricks_only()).unwrap_or_default();
    assert_eq!(plan.cells_total, 32);
    assert!((plan.coverage - 1.0).abs() < 1e-12);
    assert_eq!(plan.bill_of_materials.get("Brick 2x8"), Some(&1));
    assert_eq!(plan.bill_of_materials.get("Brick 2x2"), Some(&4));
}

#[test]
fn a_long_bar_becomes_a_tslot_profile() {
    let options = Options { use_rods: false, ..Default::default() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(100.0, 20.0, 20.0)), &options).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("T-slot 2020 x 100 mm"), Some(&1), "{:?}", plan.bill_of_materials);
    assert!(plan.bill_of_materials.keys().all(|k| !k.starts_with("Brick")));
    assert!((plan.coverage - 1.0).abs() < 1e-12);
    let mesh = plan_mesh(&plan);
    let b = mesh.bounds();
    assert!((b.max.x - b.min.x - 100.0).abs() < 1e-9, "{b:?}");
}

#[test]
fn a_tall_tower_gets_a_clamping_rod() {
    // 48 × 48 × 57.6 mm: one M8 rod through the centre column, two nuts and washers.
    let options = Options { use_profiles: false, ..Default::default() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(48.0, 48.0, 57.6)), &options).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("Threaded rod M8 x 74 mm"), Some(&1), "{:?}", plan.bill_of_materials);
    assert_eq!(plan.bill_of_materials.get("Nut M8"), Some(&2));
    assert_eq!(plan.bill_of_materials.get("Washer M8"), Some(&2));
    assert!(plan.bill_of_materials.get("Brick 2x6").copied().unwrap_or(0) > 0);
    assert!((plan.coverage - 1.0).abs() < 1e-12);
    assert!(!plan.summary().is_empty());
}

#[test]
fn round_bodies_keep_only_the_cells_really_inside() {
    // A Ø40 puck approximated by a 32-gon prism: corner cells fall outside.
    let mut mesh = Mesh::default();
    push_prism(&mut mesh, &circle(20.0, 32), Vec3::new(20.0, 20.0, 0.0), 19.2);
    let plan = constructify(&mesh, &bricks_only()).unwrap_or_default();
    assert!(plan.cells_total < 50 && plan.cells_total > 30, "{}", plan.cells_total);
    assert!((plan.coverage - 1.0).abs() < 1e-12);
    assert!(constructify(&Mesh::default(), &bricks_only()).is_none());
}

#[test]
fn the_tslot_outline_is_a_closed_square_with_four_slots() {
    let (outline, bore) = tslot_outline(20).unwrap_or_default();
    assert_eq!(outline.len(), 36);
    assert!((bore - 4.2).abs() < 1e-12);
    assert!(outline.iter().all(|p| p[0].abs() <= 10.0 + 1e-9 && p[1].abs() <= 10.0 + 1e-9));
    assert!(tslot_outline(25).is_none());
}

#[test]
fn the_brick_system_sets_the_grid_and_the_names() {
    // Plates: three per brick height, same footprint.
    let plates = Options { brick: BrickKind::Plate, ..bricks_only() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(16.0, 16.0, 9.6)), &plates).unwrap_or_default();
    assert_eq!(plan.grid, [2, 2, 3]);
    assert_eq!(plan.bill_of_materials.get("Plate 2x2"), Some(&3), "{:?}", plan.bill_of_materials);
    assert!((plan.coverage - 1.0).abs() < 1e-12);
    // Large bricks: 16 mm studs, 19.2 mm high, big studs on top.
    let large = Options { brick: BrickKind::Large, ..bricks_only() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(32.0, 32.0, 19.2)), &large).unwrap_or_default();
    assert_eq!(plan.grid, [2, 2, 1]);
    assert_eq!(plan.bill_of_materials.get("Large brick 2x2"), Some(&1), "{:?}", plan.bill_of_materials);
    let b = plan_mesh(&plan).bounds();
    assert!((b.max.z - 19.2 - 4.5).abs() < 1e-9, "stud height: {b:?}");
    // Names resolve however they are written; the labels list every kind.
    assert_eq!(BrickKind::from_name("Plates (8 mm studs, 3.2 mm thin)"), Some(BrickKind::Plate));
    assert_eq!(BrickKind::from_name(" LARGE bricks"), Some(BrickKind::Large));
    assert_eq!(BrickKind::from_name("classic"), Some(BrickKind::Brick));
    assert_eq!(BrickKind::from_name("duplo"), None);
    assert_eq!(BRICK_KINDS.iter().map(|k| k.label()).collect::<Vec<_>>().len(), 3);
}

#[test]
fn the_profile_series_and_the_rod_size_are_chosen() {
    let options = Options { use_rods: false, use_bolts: false, profile_series: 30, ..Default::default() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(120.0, 30.0, 30.0)), &options).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("T-slot 3030 x 120 mm"), Some(&1), "{:?}", plan.bill_of_materials);
    let b = plan_mesh(&plan).bounds();
    assert!((b.max.y - b.min.y - 30.0).abs() < 1e-9, "{b:?}");
    let options = Options { use_profiles: false, use_bolts: false, rod_size: 12, ..Default::default() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(48.0, 48.0, 57.6)), &options).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("Threaded rod M12 x 82 mm"), Some(&1), "{:?}", plan.bill_of_materials);
    assert_eq!(plan.bill_of_materials.get("Nut M12"), Some(&2));
    // A series, rod or bolt off the catalogue is refused.
    assert!(constructify(&box_mesh(Vec3::ZERO, Vec3::new(16.0, 16.0, 9.6)), &Options { profile_series: 25, ..Default::default() }).is_none());
    assert!(constructify(&box_mesh(Vec3::ZERO, Vec3::new(16.0, 16.0, 9.6)), &Options { rod_size: 7, ..Default::default() }).is_none());
    assert!(constructify(&box_mesh(Vec3::ZERO, Vec3::new(16.0, 16.0, 9.6)), &Options { bolt_size: 2, ..Default::default() }).is_none());
}

#[test]
fn short_columns_get_a_bolt_and_tall_ones_a_rod() {
    // 48 × 48 × 19.2 mm: two layers, too short for a rod, bolted through the centre column with
    // the next stock length above 19.2 + 1.5 × 4 = 25.2 mm.
    let options = Options { use_profiles: false, ..Default::default() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(48.0, 48.0, 19.2)), &options).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("Hex bolt M4 x 30 mm"), Some(&1), "{:?}", plan.bill_of_materials);
    assert_eq!(plan.bill_of_materials.get("Nut M4"), Some(&1));
    assert_eq!(plan.bill_of_materials.get("Washer M4"), Some(&2));
    assert!(plan.bill_of_materials.keys().all(|k| !k.starts_with("Threaded rod")));
    let bolt = plan.elements.iter().find(|e| e.kind == ElementKind::Bolt).cloned().unwrap_or_else(|| plan.elements[0].clone());
    assert!((bolt.position.z + bolt.length - 19.2).abs() < 1e-9, "the head sits on the column: {bolt:?}");
    let b = plan_mesh(&plan).bounds();
    assert!((b.max.z - 19.2 - 0.7 * 4.0).abs() < 1e-9, "the head stands 0.7 d proud: {b:?}");
    // Bolts off: nothing clamps a short column. A tall column still takes the rod, not a bolt.
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(48.0, 48.0, 19.2)), &Options { use_bolts: false, ..options.clone() }).unwrap_or_default();
    assert!(plan.bill_of_materials.keys().all(|k| !k.starts_with("Hex bolt")));
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(48.0, 48.0, 57.6)), &options).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("Threaded rod M8 x 74 mm"), Some(&1));
    assert!(plan.bill_of_materials.keys().all(|k| !k.starts_with("Hex bolt")));
    // A bigger bolt on a one-layer plate stack: M8 wants 9.6 + 12 = 21.6, so 25 mm.
    let options = Options { use_profiles: false, use_rods: false, bolt_size: 8, bolt_min_layers: 1, ..Default::default() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(48.0, 48.0, 9.6)), &options).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("Hex bolt M8 x 25 mm"), Some(&1), "{:?}", plan.bill_of_materials);
}

#[test]
fn a_small_block_still_gets_a_profile_and_a_centred_bolt() {
    // 50 × 30 × 20 mm: two 2020 cells in a row make the 40 mm run the default asks for.
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(50.0, 30.0, 20.0)), &Options::default()).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("T-slot 2020 x 40 mm"), Some(&1), "{:?}", plan.bill_of_materials);
    // 24 × 24 × 19.2 mm: three studs wide, so the bolt column sits on the middle stud instead
    // of an absolute 48 mm lattice the block never reaches.
    let options = Options { use_profiles: false, ..Default::default() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(24.0, 24.0, 19.2)), &options).unwrap_or_default();
    assert_eq!(plan.bill_of_materials.get("Hex bolt M4 x 30 mm"), Some(&1), "{:?}", plan.bill_of_materials);
    let bolt = plan.elements.iter().find(|e| e.kind == ElementKind::Bolt).cloned().unwrap_or_else(|| plan.elements[0].clone());
    assert!((bolt.position.x - 12.0).abs() < 1e-9 && (bolt.position.y - 12.0).abs() < 1e-9, "{bolt:?}");
    assert!(plan.warnings.is_empty(), "{:?}", plan.warnings);
    // A 15 mm thin plate holds no 2020 profile and no two-layer column: the plan says why.
    let options = Options { use_bricks: false, ..Default::default() };
    let plan = constructify(&box_mesh(Vec3::ZERO, Vec3::new(50.0, 30.0, 15.0)), &options).unwrap_or_default();
    assert!(plan.elements.is_empty(), "{:?}", plan.elements);
    assert!(plan.warnings.iter().any(|w| w.contains("T-slot 2020")), "{:?}", plan.warnings);
    assert!(plan.warnings.iter().any(|w| w.contains("rod M8") && w.contains("bolt M4")), "{:?}", plan.warnings);
}
