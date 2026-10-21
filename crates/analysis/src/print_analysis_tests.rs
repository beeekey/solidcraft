//! Printability measured on meshes whose numbers can be derived on paper.

use solidcraft_geom::{Mesh, Vec3};

use super::{
    InfillPattern, PrintSettings, advise_orientation, analyze, bridges, effective_material, estimate, filament_named, thinnest_wall,
    topology_optimize, watertight,
};
use crate::voxelise::Region;

const DEG: f64 = std::f64::consts::PI / 180.0;
const PRINTER: Vec3 = Vec3 { x: 220.0, y: 220.0, z: 250.0 };

/// A closed box mesh from `min` (mm), triangles counter-clockwise from outside.
fn box_at(min: Vec3, size: Vec3) -> Mesh {
    let p = |x: f64, y: f64, z: f64| min + Vec3::new(x, y, z);
    let (a, b, c) = (size.x, size.y, size.z);
    let positions = vec![p(0.0, 0.0, 0.0), p(a, 0.0, 0.0), p(a, b, 0.0), p(0.0, b, 0.0), p(0.0, 0.0, c), p(a, 0.0, c), p(a, b, c), p(0.0, b, c)];
    let triangles =
        vec![[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7]];
    let mut mesh = Mesh { positions, triangles, ..Default::default() };
    mesh.normals = vec![Vec3::ZERO; 8];
    mesh.tri_face = (0..12).map(|t| (t / 2) as u32).collect();
    mesh
}

/// A mushroom as one closed skin: a 10 × 10 column 15 tall under a 40 × 40 × 5 top, the top's
/// underside an annulus round the column. Overhang = 40·40 − 10·10 = 1500 mm².
fn mushroom() -> Mesh {
    let mut m = Mesh::default();
    let mut add = |pts: [Vec3; 3]| {
        let base = m.positions.len() as u32;
        m.positions.extend(pts);
        m.triangles.push([base, base + 1, base + 2]);
    };
    let quad = |a: Vec3, b: Vec3, c: Vec3, d: Vec3, add: &mut dyn FnMut([Vec3; 3])| {
        add([a, b, c]);
        add([a, c, d]);
    };
    let v = Vec3::new;
    // Column: bottom (down), four walls.
    quad(v(15.0, 15.0, 0.0), v(15.0, 25.0, 0.0), v(25.0, 25.0, 0.0), v(25.0, 15.0, 0.0), &mut add);
    quad(v(15.0, 15.0, 0.0), v(25.0, 15.0, 0.0), v(25.0, 15.0, 15.0), v(15.0, 15.0, 15.0), &mut add); // y = 15, normal −y
    quad(v(25.0, 25.0, 0.0), v(15.0, 25.0, 0.0), v(15.0, 25.0, 15.0), v(25.0, 25.0, 15.0), &mut add); // y = 25
    quad(v(25.0, 15.0, 0.0), v(25.0, 25.0, 0.0), v(25.0, 25.0, 15.0), v(25.0, 15.0, 15.0), &mut add); // x = 25
    quad(v(15.0, 25.0, 0.0), v(15.0, 15.0, 0.0), v(15.0, 15.0, 15.0), v(15.0, 25.0, 15.0), &mut add); // x = 15
    // Top: its top face (up), four walls split where the underside's cells meet them (no
    // T-junctions), and the underside as the eight cells of a 3 × 3 grid round the column.
    let stations = [0.0, 15.0, 25.0, 40.0];
    for xw in stations.windows(2) {
        for yw in stations.windows(2) {
            quad(v(xw[0], yw[0], 20.0), v(xw[1], yw[0], 20.0), v(xw[1], yw[1], 20.0), v(xw[0], yw[1], 20.0), &mut add);
        }
    }
    for w in stations.windows(2) {
        let (s0, s1) = (w[0], w[1]);
        quad(v(s0, 0.0, 15.0), v(s1, 0.0, 15.0), v(s1, 0.0, 20.0), v(s0, 0.0, 20.0), &mut add); // y = 0
        quad(v(s1, 40.0, 15.0), v(s0, 40.0, 15.0), v(s0, 40.0, 20.0), v(s1, 40.0, 20.0), &mut add); // y = 40
        quad(v(40.0, s0, 15.0), v(40.0, s1, 15.0), v(40.0, s1, 20.0), v(40.0, s0, 20.0), &mut add); // x = 40
        quad(v(0.0, s1, 15.0), v(0.0, s0, 15.0), v(0.0, s0, 20.0), v(0.0, s1, 20.0), &mut add); // x = 0
    }
    let down = |a: Vec3, b: Vec3, c: Vec3, d: Vec3, add: &mut dyn FnMut([Vec3; 3])| {
        add([a, c, b]);
        add([a, d, c]);
    };
    for (xi, xw) in stations.windows(2).enumerate() {
        for (yi, yw) in stations.windows(2).enumerate() {
            if xi == 1 && yi == 1 {
                continue; // the column
            }
            down(v(xw[0], yw[0], 15.0), v(xw[1], yw[0], 15.0), v(xw[1], yw[1], 15.0), v(xw[0], yw[1], 15.0), &mut add);
        }
    }
    m.normals = vec![Vec3::ZERO; m.positions.len()];
    m.tri_face = (0..m.triangles.len()).map(|t| (t / 2) as u32).collect();
    m
}

/// A frustum whose walls lean out at exactly 45°: 40 square at z = 0, 60 square at z = 10.
fn frustum() -> Mesh {
    let v = Vec3::new;
    let positions = vec![
        v(0.0, 0.0, 0.0),
        v(40.0, 0.0, 0.0),
        v(40.0, 40.0, 0.0),
        v(0.0, 40.0, 0.0),
        v(-10.0, -10.0, 10.0),
        v(50.0, -10.0, 10.0),
        v(50.0, 50.0, 10.0),
        v(-10.0, 50.0, 10.0),
    ];
    let triangles =
        vec![[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7]];
    let mut mesh = Mesh { positions, triangles, ..Default::default() };
    mesh.normals = vec![Vec3::ZERO; 8];
    mesh.tri_face = (0..12).map(|t| (t / 2) as u32).collect();
    mesh
}

#[test]
fn a_box_on_the_plate_needs_no_support() {
    let report = analyze(&box_at(Vec3::ZERO, Vec3::new(30.0, 20.0, 10.0)), Vec3::Z, 45.0 * DEG, PRINTER, 0.2);
    assert!(report.watertight);
    assert!(report.fits_build_volume);
    assert!(report.overhang_area_mm2.abs() < 1e-9, "the seated bottom is excluded");
    assert_eq!(report.layer_count, 50);
    assert!((report.height_mm - 10.0).abs() < 1e-9);
    assert!((report.volume_mm3 - 6000.0).abs() < 1e-6);
    // Too big for the printer in every axis order.
    let big = analyze(&box_at(Vec3::ZERO, Vec3::new(300.0, 20.0, 10.0)), Vec3::Z, 45.0 * DEG, PRINTER, 0.2);
    assert!(!big.fits_build_volume);
    // Nothing: not watertight, no numbers.
    assert!(!analyze(&Mesh::default(), Vec3::Z, 45.0 * DEG, PRINTER, 0.2).watertight);
}

#[test]
fn a_table_tops_overhang_is_measured_exactly() {
    let m = mushroom();
    assert!(watertight(&m), "the mushroom is one closed skin");
    let report = analyze(&m, Vec3::Z, 45.0 * DEG, PRINTER, 0.2);
    let expected = 40.0 * 40.0 - 10.0 * 10.0;
    assert!((report.overhang_area_mm2 - expected).abs() < 1e-6, "{}", report.overhang_area_mm2);
    assert!((report.support_contact_area_mm2 - expected).abs() < 1e-6);
    assert!((report.height_mm - 20.0).abs() < 1e-9);
    // Support is the prism under the overhang: 1500 mm² × 15 mm.
    let cost = estimate(&m, Vec3::Z, 45.0 * DEG, 0.2, 1240.0, 8.0, 1.0);
    assert!((cost.support_volume_mm3 - expected * 15.0).abs() < 1e-6, "{}", cost.support_volume_mm3);
    let part = 10.0 * 10.0 * 15.0 + 40.0 * 40.0 * 5.0;
    assert!((cost.material_volume_mm3 - part).abs() < 1e-6, "{} vs {part}", cost.material_volume_mm3);
    assert!((cost.material_grams - (part + expected * 15.0) * 1e-9 * 1240.0 * 1000.0).abs() < 1e-9);
    assert_eq!(cost.layer_count, 100);
    assert!((cost.time_seconds - ((part + expected * 15.0) / 8.0 + 100.0)).abs() < 1e-6);
}

#[test]
fn forty_five_degree_walls_print_without_support() {
    let m = frustum();
    let at45 = analyze(&m, Vec3::Z, 45.0 * DEG, PRINTER, 0.2);
    assert!(at45.overhang_area_mm2.abs() < 1e-9, "{}", at45.overhang_area_mm2);
    let at40 = analyze(&m, Vec3::Z, 40.0 * DEG, PRINTER, 0.2);
    let wall_area = 4.0 * (40.0 + 60.0) / 2.0 * 10.0 * 2f64.sqrt();
    assert!((at40.overhang_area_mm2 - wall_area).abs() < wall_area * 1e-6, "{} vs {wall_area}", at40.overhang_area_mm2);
}

#[test]
fn the_orientation_advisor_flips_the_table() {
    let options = advise_orientation(&mushroom(), 45.0 * DEG, PRINTER, 0.2);
    assert_eq!(options.len(), 6);
    assert_eq!(options[0].name, "-Z", "the top face becomes the plate");
    assert!(options[0].report.overhang_area_mm2.abs() < 1e-9);
    let leg_down = options.iter().find(|o| o.name == "+Z").map(|o| o.report.overhang_area_mm2).unwrap_or(0.0);
    assert!((leg_down - 1500.0).abs() < 1e-6, "{leg_down}");
}

#[test]
fn the_thinnest_wall_is_the_box_s_thinnest_side_and_a_beam_bridges() {
    let (thickness, _) = thinnest_wall(&box_at(Vec3::ZERO, Vec3::new(20.0, 20.0, 10.0))).unwrap_or((0.0, Vec3::ZERO));
    assert!((thickness - 10.0).abs() < 1e-3, "{thickness}");
    assert!(thinnest_wall(&Mesh::default()).is_none());
    // Two 10 mm pillars 30 mm apart under a beam: the beam's underside between them bridges.
    let mut m = box_at(Vec3::ZERO, Vec3::new(10.0, 10.0, 20.0));
    m.append(&box_at(Vec3::new(40.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 20.0)), 6);
    m.append(&box_at(Vec3::new(0.0, 0.0, 20.0), Vec3::new(50.0, 10.0, 5.0)), 12);
    let bridged = bridges(&m, Vec3::Z, 40.0);
    assert!(bridged.bridged_area_mm2 > 0.0, "{bridged:?}");
    assert!((bridged.longest_span_mm - 30.0).abs() < 1e-3, "{bridged:?}");
    let too_far = bridges(&m, Vec3::Z, 20.0);
    assert!(too_far.longest_span_mm < 30.0, "{too_far:?}");
}

fn with(pattern: InfillPattern, density: f64, walls: u32) -> super::EffectiveMaterial {
    let settings = PrintSettings { pattern, infill_density: density, wall_loops: walls, ..Default::default() };
    // A 120 × 30 × 8 mm bar.
    effective_material(&settings, 120.0 * 30.0 * 8.0, 2.0 * (120.0 * 30.0 + 30.0 * 8.0 + 8.0 * 120.0))
}

#[test]
fn denser_infill_is_stiffer_and_heavier_and_walls_beat_infill() {
    let (mut previous_e, mut previous_mass) = (0.0, 0.0);
    for density in [0.0, 0.1, 0.2, 0.4, 0.6, 1.0] {
        let m = with(InfillPattern::Gyroid, density, 2);
        let e = m.material.map(|m| m.ex).unwrap_or(0.0);
        assert!(e > previous_e && m.mass_grams > previous_mass, "at {density}");
        previous_e = e;
        previous_mass = m.mass_grams;
    }
    let thin = with(InfillPattern::Grid, 0.40, 2);
    let thick = with(InfillPattern::Grid, 0.20, 4);
    assert!(thick.material.map(|m| m.ex) > thin.material.map(|m| m.ex), "four walls at 20% beat two at 40%");
    assert!(thick.mass_grams < thin.mass_grams * 1.2);
}

#[test]
fn patterns_differ_in_where_they_put_the_stiffness_and_layers_are_the_weak_direction() {
    let (grid, gyroid, concentric, lightning) = (
        with(InfillPattern::Grid, 0.25, 2),
        with(InfillPattern::Gyroid, 0.25, 2),
        with(InfillPattern::Concentric, 0.25, 2),
        with(InfillPattern::Lightning, 0.25, 2),
    );
    assert!(
        (grid.mass_grams - gyroid.mass_grams).abs() < 1e-9 && (grid.mass_grams - lightning.mass_grams).abs() < 1e-9,
        "every pattern at one density weighs the same"
    );
    let ratio = |m: &super::EffectiveMaterial| m.material.map(|m| m.ez / m.ex).unwrap_or(0.0);
    assert!(ratio(&grid) > ratio(&gyroid), "grid walls run straight up");
    let ex = |m: &super::EffectiveMaterial| m.material.map(|m| m.ex).unwrap_or(0.0);
    assert!(ex(&concentric) > ex(&grid) && ex(&concentric) > ex(&gyroid));
    assert!(ex(&lightning) < ex(&gyroid));
    assert!(lightning.note.contains("not to carry load"));
    for (pattern, name) in super::INFILL_PATTERNS {
        let m = with(pattern, 0.25, 2);
        assert!(m.tensile_interlayer < m.tensile_in_plane, "{name} must be weaker across the layers");
        assert!(m.tensile_interlayer > m.tensile_in_plane * 0.3, "{name}: a published band");
        assert_eq!(InfillPattern::from_name(name), Some(pattern));
    }
    let solid = with(InfillPattern::Grid, 1.0, 2);
    assert!((solid.solid_fraction - 1.0).abs() < 1e-9);
    assert!(solid.note.contains("pattern hardly matters"));
    assert!(solid.material.is_some_and(|m| m.ez < m.ex), "the layer bond does not go away at 100%");
    assert_eq!(filament_named("petg (printed)").name, "PETG");
    assert!(effective_material(&PrintSettings::default(), f64::NAN, 1.0).material.is_none());
}

#[test]
fn topology_optimisation_keeps_the_load_path_and_honours_the_target() {
    // A 60 × 10 × 10 cantilever held at x = 0 and loaded down at x = 60: keep 45%.
    let bar = box_at(Vec3::ZERO, Vec3::new(60.0, 10.0, 10.0));
    let mask = topology_optimize(
        &bar,
        &Region::NearestWall(Vec3::new(0.0, 5.0, 5.0)),
        &Region::NearestWall(Vec3::new(60.0, 5.0, 5.0)),
        Vec3::new(0.0, 0.0, -100.0),
        0.45,
        24,
    )
    .unwrap_or_default();
    assert!(mask.voxels_total > 0, "{mask:?}");
    assert!(mask.kept_fraction >= 0.44 && mask.kept_fraction <= 0.65, "{}", mask.kept_fraction);
    // Both end slabs stay: the fixed cells and the loaded cells are always kept.
    let (nx, ny, nz) = (mask.grid[0], mask.grid[1], mask.grid[2]);
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    for k in 0..nz {
        for j in 0..ny {
            assert!(mask.kept[at(0, j, k)] && mask.kept[at(nx - 1, j, k)]);
        }
    }
    // The core of the neutral axis goes first: fewer kept cells in the middle rows than the top
    // and bottom in a mid section.
    let mid = nx / 2;
    let count = |k: usize| (0..ny).filter(|j| mask.kept[at(mid, *j, k)]).count();
    assert!(count(0) + count(nz - 1) >= count(nz / 2), "outer fibres carry bending");
    assert!(topology_optimize(&bar, &Region::NearestWall(Vec3::ZERO), &Region::NearestWall(Vec3::X), Vec3::Z, 1.5, 24).is_none());
}
