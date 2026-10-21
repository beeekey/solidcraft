//! The machining rules on features whose numbers can be derived on paper, and the access scan
//! on voxel blocks with pockets carved out of them.

use solidcraft_geom::Vec3;

use super::{
    AccessGrid, END_MILL_DIAMETERS, Hole, InternalCorner, MachiningSetup, Material, Pocket, Process, RevolutionReport, Severity, ShoulderFillet,
    check_corners, check_holes, check_pockets, check_turning, check_wall, describe, is_in_table, iso235_drill_sizes, nearest_from_table,
    preferred_drill_sizes, scan_access, sort_findings, wall_limits,
};

fn count(findings: &[super::Finding], rule: &str, severity: Severity) -> usize {
    findings.iter().filter(|f| f.rule == rule && f.severity == severity).count()
}

#[test]
fn the_tables_hold_what_a_shop_stocks() {
    assert!(is_in_table(&END_MILL_DIAMETERS, 6.0, 0.005));
    assert!(!is_in_table(&END_MILL_DIAMETERS, 7.0, 0.005));
    assert_eq!(nearest_from_table(&END_MILL_DIAMETERS, 9.4), 10.0);
    let iso = iso235_drill_sizes();
    assert!(is_in_table(&iso, 1.05, 0.005) && is_in_table(&iso, 6.8, 0.005) && is_in_table(&iso, 10.5, 0.005));
    assert!(!is_in_table(&iso, 6.83, 0.005));
    assert!(is_in_table(&preferred_drill_sizes(), 6.5, 0.005) && !is_in_table(&preferred_drill_sizes(), 6.8, 0.005));
    assert_eq!(nearest_from_table(&[], 3.0), 0.0);
    assert_eq!(Material::parse("aluminum"), Some(Material::Aluminium));
    assert_eq!(Material::parse("unobtainium"), None);
}

#[test]
fn a_plastic_wall_limit_says_it_borrowed_the_aluminium_numbers() {
    let plastic = wall_limits(Material::Plastic);
    let aluminium = wall_limits(Material::Aluminium);
    assert!(plastic.borrowed && !aluminium.borrowed && !wall_limits(Material::Steel).borrowed);
    assert_eq!(plastic.absolute, aluminium.absolute);
    let setup = MachiningSetup { material: Material::Plastic, ..Default::default() };
    let found = check_wall(0.7, Vec3::ZERO, &setup);
    assert_eq!(found.len(), 1);
    assert!(found[0].detail.contains("aluminium limits are used"), "{}", found[0].detail);
    assert_eq!(check_wall(0.3, Vec3::ZERO, &setup)[0].severity, Severity::Refuse);
    assert!(check_wall(2.0, Vec3::ZERO, &setup).is_empty());
}

#[test]
fn a_square_pocket_has_four_corners_no_cutter_can_cut() {
    let corners: Vec<InternalCorner> =
        (0..4).map(|n| InternalCorner { at: Vec3::new(n as f64, 0.0, 0.0), radius: 0.0, depth: 10.0, sharp: true }).collect();
    let setup = MachiningSetup { process: Process::Milling3Axis, cutter_diameter: 6.0, ..Default::default() };
    assert_eq!(count(&check_corners(&corners, &setup), "internal corner radius", Severity::Refuse), 4);
}

#[test]
fn a_radiused_corner_passes_for_one_cutter_and_not_the_next_size_up() {
    // R6 corners take a 6 mm cutter and refuse a 16 mm one: measured 6 against 8 needed.
    let corners: Vec<InternalCorner> =
        (0..2).map(|n| InternalCorner { at: Vec3::new(n as f64, 0.0, 0.0), radius: 6.0, depth: 11.0, sharp: false }).collect();
    let small = MachiningSetup { process: Process::Milling3Axis, cutter_diameter: 6.0, ..Default::default() };
    assert_eq!(count(&check_corners(&corners, &small), "internal corner radius", Severity::Refuse), 0);
    let large = MachiningSetup { process: Process::Milling3Axis, cutter_diameter: 16.0, ..Default::default() };
    let found = check_corners(&corners, &large);
    assert_eq!(count(&found, "internal corner radius", Severity::Refuse), 2);
    for f in found.iter().filter(|f| f.rule == "internal corner radius") {
        assert!((f.measured - 6.0).abs() < 1e-9 && (f.limit - 8.0).abs() < 1e-9);
    }
    // A radius nobody stocks is noted with the nearest standard cutter.
    let odd = [InternalCorner { at: Vec3::ZERO, radius: 4.7, depth: 5.0, sharp: false }];
    let found = check_corners(&odd, &small);
    assert_eq!(count(&found, "non-standard corner radius", Severity::Note), 1);
    assert!(found.iter().any(|f| f.rule == "non-standard corner radius" && (f.limit - 5.0).abs() < 1e-9));
}

#[test]
fn a_deep_narrow_pocket_is_refused_and_a_shallow_one_is_not() {
    let setup = MachiningSetup { process: Process::Milling3Axis, cutter_diameter: 6.0, ..Default::default() };
    for (depth, worst) in [(20.0, None), (35.0, Some(Severity::Note)), (50.0, Some(Severity::Warning)), (70.0, Some(Severity::Refuse))] {
        let pocket = [Pocket { floor_at: Vec3::ZERO, depth, narrowest_width: 10.0, floor_corner_sharp: false }];
        let found = check_pockets(&pocket, &setup);
        match worst {
            None => assert!(found.is_empty(), "depth {depth}: {found:?}"),
            Some(s) => assert_eq!(count(&found, "pocket depth vs narrowest width", s), 1, "depth {depth}"),
        }
    }
    let narrow = [Pocket { floor_at: Vec3::ZERO, depth: 5.0, narrowest_width: 4.0, floor_corner_sharp: true }];
    let found = check_pockets(&narrow, &setup);
    assert_eq!(count(&found, "pocket narrower than the cutter", Severity::Refuse), 1);
    assert_eq!(count(&found, "floor-to-wall corner", Severity::Note), 1);
}

#[test]
fn holes_are_judged_by_depth_drill_size_wall_and_neighbours() {
    let setup = MachiningSetup::default();
    let hole = |x: f64, d: f64, depth: f64, through: bool| Hole {
        axis_point: Vec3::new(x, 0.0, 0.0),
        axis: Vec3::Z,
        diameter: d,
        depth,
        through,
        entry_at: Vec3::new(x, 0.0, 10.0),
        entry_face_curved: false,
        entry_deviation_degrees: 0.0,
        flat_bottom: false,
        thinnest_surround: 0.0,
        broken_into: false,
    };
    let deep_blind = check_holes(&[hole(0.0, 5.0, 30.0, false)], &setup);
    assert_eq!(count(&deep_blind, "hole depth vs diameter", Severity::Refuse), 1);
    assert!(check_holes(&[hole(0.0, 5.0, 30.0, true)], &setup).iter().all(|f| f.rule != "hole depth vs diameter"), "through, drilled from both ends");
    let odd = check_holes(&[hole(0.0, 6.83, 5.0, true)], &setup);
    assert_eq!(count(&odd, "non-standard drill size", Severity::Warning), 1);
    let half = check_holes(&[hole(0.0, 6.8, 5.0, true)], &setup);
    assert_eq!(count(&half, "drill size off the whole and half millimetres", Severity::Note), 1);
    let mut thin = hole(0.0, 6.0, 5.0, true);
    thin.thinnest_surround = 2.0;
    assert_eq!(count(&check_holes(&[thin], &setup), "material around a hole", Severity::Refuse), 1);
    let mut tilted = hole(0.0, 6.0, 5.0, true);
    tilted.entry_deviation_degrees = 3.0;
    tilted.flat_bottom = true;
    let found = check_holes(&[tilted], &setup);
    assert_eq!(count(&found, "drill entry face", Severity::Warning), 1);
    assert_eq!(count(&found, "flat-bottomed blind hole", Severity::Note), 1);
    let crossing = check_holes(&[hole(0.0, 6.0, 5.0, true), hole(4.0, 6.0, 5.0, true), hole(20.0, 6.0, 5.0, true)], &setup);
    assert_eq!(count(&crossing, "intersecting holes", Severity::Warning), 1);
}

#[test]
fn a_turned_part_is_judged_by_slenderness_grooves_and_shoulders() {
    let bar = RevolutionReport {
        is_body_of_revolution: true,
        actual_volume: 1.0,
        length: 100.0,
        max_diameter: 20.0,
        slenderness: 5.0,
        ..Default::default()
    };
    let found = check_turning(&bar, &[]);
    assert_eq!(count(&found, "slenderness", Severity::Note), 1);
    let slender = RevolutionReport { slenderness: 12.0, ..bar.clone() };
    assert_eq!(count(&check_turning(&slender, &[]), "slenderness", Severity::Warning), 1);
    let flat = RevolutionReport { is_body_of_revolution: false, off_axis_volume: 1635.0, flats: 1, ..bar.clone() };
    let found = check_turning(&flat, &[]);
    assert_eq!(count(&found, "off-axis material", Severity::Note), 1);
    assert!(found.iter().any(|f| f.detail.contains("1 flat(s)")));
    let fillets = [
        ShoulderFillet { radius: 0.1, at: Vec3::ZERO },
        ShoulderFillet { radius: 0.8, at: Vec3::ZERO },
        ShoulderFillet { radius: 1.0, at: Vec3::ZERO },
    ];
    let found = check_turning(&bar, &fillets);
    assert_eq!(count(&found, "shoulder fillet radius", Severity::Warning), 1);
    assert_eq!(count(&found, "shoulder fillet radius", Severity::Note), 1);
    assert!(check_turning(&RevolutionReport::default(), &[]).is_empty());
}

#[test]
fn the_report_says_what_it_assumed_before_anything_else_and_sorts_worst_first() {
    let setup = MachiningSetup { process: Process::Milling3Axis, ..Default::default() };
    let mut findings = check_pockets(&[Pocket { floor_at: Vec3::ZERO, depth: 35.0, narrowest_width: 10.0, floor_corner_sharp: true }], &setup);
    findings.extend(check_corners(&[InternalCorner { at: Vec3::ZERO, radius: 0.0, depth: 10.0, sharp: true }], &setup));
    sort_findings(&mut findings);
    assert_eq!(findings[0].severity, Severity::Refuse);
    for pair in findings.windows(2) {
        assert!(pair[0].severity >= pair[1].severity);
    }
    let text = describe(&setup, None, None, &findings, &["a caveat".into()]);
    assert!(text.starts_with("assumed:"), "{text}");
    assert!(text.contains("D6.0 end mill") && text.contains("Aluminium") && text.contains("NOT CHECKED  a caveat"), "{text}");
    for f in &findings {
        assert!(!f.rule.is_empty() && !f.detail.is_empty());
    }
}

/// A block filling the grid at one millimetre cells, ready to have pockets cut out of it.
struct Block {
    nx: usize,
    ny: usize,
    nz: usize,
    solid: Vec<bool>,
}

impl Block {
    fn new(nx: usize, ny: usize, nz: usize) -> Block {
        // One cell of air round the block so the outside is outside.
        let mut solid = vec![false; (nx + 2) * (ny + 2) * (nz + 2)];
        for k in 1..=nz {
            for j in 1..=ny {
                for i in 1..=nx {
                    solid[(k * (ny + 2) + j) * (nx + 2) + i] = true;
                }
            }
        }
        Block { nx: nx + 2, ny: ny + 2, nz: nz + 2, solid }
    }
    fn carve(&mut self, i0: usize, i1: usize, j0: usize, j1: usize, k0: usize, k1: usize) {
        for k in k0 + 1..k1 + 1 {
            for j in j0 + 1..j1 + 1 {
                for i in i0 + 1..i1 + 1 {
                    self.solid[(k * self.ny + j) * self.nx + i] = false;
                }
            }
        }
    }
    fn scan(&self, tool_radius: f64) -> super::AccessScan {
        scan_access(
            &AccessGrid { nx: self.nx, ny: self.ny, nz: self.nz, h: 1.0, origin: Vec3::new(-1.0, -1.0, -1.0), solid: &self.solid },
            tool_radius,
        )
    }
}

#[test]
fn a_plain_block_is_stock_and_has_nothing_to_machine() {
    let scan = Block::new(40, 40, 20).scan(3.0);
    assert_eq!(scan.machined_faces, 0, "every face of a block is the billet's own outside");
    assert_eq!(scan.setup_count, 0);
    assert!(scan.unreachable_regions.is_empty() && !scan.has_trapped);
    assert_eq!(scan.trapped_volume, 0.0);
}

#[test]
fn one_pocket_is_one_setup_and_an_exact_number_of_faces() {
    let mut block = Block::new(40, 40, 20);
    block.carve(10, 30, 10, 30, 10, 20);
    let scan = block.scan(3.0);
    let (floor, walls) = (20 * 20, 4 * 20 * 10);
    assert_eq!(scan.machined_faces, floor + walls);
    assert_eq!(scan.setup_count, 1, "{:?}", scan.setup_axes);
    assert!(scan.setup_axes[4], "+Z, where the pocket opens");
    assert_eq!(scan.unreachable_fraction, 0.0, "an end mill cuts a pocket wall with its flank");
    assert_eq!(scan.reached_by_axis[4], floor + walls);
}

#[test]
fn a_pocket_each_end_is_two_setups() {
    let mut block = Block::new(40, 40, 20);
    block.carve(10, 30, 10, 30, 14, 20);
    block.carve(10, 30, 10, 30, 0, 6);
    let scan = block.scan(3.0);
    assert_eq!(scan.setup_count, 2);
    assert!(scan.setup_axes[4] && scan.setup_axes[5]);
    assert_eq!(scan.unreachable_fraction, 0.0);
}

#[test]
fn a_t_slot_lobe_is_an_undercut_with_a_place_on_it_and_a_sealed_void_is_measured_exactly() {
    let mut block = Block::new(40, 40, 20);
    block.carve(18, 22, 10, 30, 10, 20); // throat
    block.carve(12, 28, 10, 30, 6, 10); // lobe
    let scan = block.scan(3.0);
    assert!(!scan.unreachable_regions.is_empty(), "the lobe should be out of reach");
    assert!(scan.unreachable_regions[0].faces > 0);
    assert!(!scan.has_trapped && scan.trapped_volume == 0.0, "it opens through the throat");
    let at = scan.unreachable_regions[0].sample;
    assert!(at.z >= 5.0 && at.z <= 11.0, "{at:?}");
    let mut sealed = Block::new(40, 40, 20);
    sealed.carve(15, 25, 15, 25, 5, 15);
    let scan = sealed.scan(3.0);
    assert!(scan.has_trapped);
    assert!((scan.trapped_volume - 1000.0).abs() < 1e-6);
}

#[test]
fn a_slot_narrower_than_the_cutter_opens_to_nothing() {
    let mut block = Block::new(40, 40, 20);
    block.carve(19, 22, 10, 30, 10, 20);
    assert_eq!(block.scan(1.0).unreachable_fraction, 0.0, "a 2 mm cutter goes down a 3 mm slot");
    assert!(block.scan(3.0).unreachable_fraction > 0.0, "a 6 mm one does not");
}
