//! The printed-part solve held to what a slicer's choices must do: the build direction decides
//! which failure governs, every pattern weighs the same, and up = +Z is the plain material.

use solidcraft_geom::{Mesh, Vec3};

use crate::element::Element;
use crate::print_analysis::{INFILL_PATTERNS, InfillPattern, PrintSettings, effective_material};
use crate::printed_stress::{Governs, compare_infill, layer_stress, printed_material, printed_stress};
use crate::{MeshFemOptions, Region};

/// A closed box mesh from the origin, in millimetres, triangles counter-clockwise from outside.
fn box_mesh(a: f64, b: f64, c: f64) -> Mesh {
    let p = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
    let positions = vec![p(0.0, 0.0, 0.0), p(a, 0.0, 0.0), p(a, b, 0.0), p(0.0, b, 0.0), p(0.0, 0.0, c), p(a, 0.0, c), p(a, b, c), p(0.0, b, c)];
    let triangles =
        vec![[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7]];
    let mut mesh = Mesh { positions, triangles, ..Default::default() };
    mesh.normals = vec![Vec3::ZERO; 8];
    mesh.tri_face = (0..12).map(|t| (t / 2) as u32).collect();
    mesh
}

#[test]
fn printed_up_z_is_the_effective_material_itself_and_other_directions_turn_it() {
    let settings = PrintSettings::default();
    let effective = effective_material(&settings, 100.0 * 10.0 * 10.0, 2.0 * (1000.0 + 100.0 + 100.0));
    let plain = effective.material.unwrap_or_else(|| panic!("a material"));
    let up = printed_material(&effective, Vec3::Z).unwrap_or_else(|| panic!("a printed material"));
    assert_eq!(up.ex, plain.ez);
    assert_eq!(up.ey, plain.ex);
    assert!(up.ex < up.ey, "the layer bond makes the part softer across the layers: {up:?}");
    let a = Element::build(&plain, 1e-3).unwrap_or_else(|e| panic!("plain element: {e}"));
    let b = Element::build(&up, 1e-3).unwrap_or_else(|e| panic!("printed element: {e}"));
    for (ra, rb) in a.c.iter().zip(b.c.iter()) {
        for (x, y) in ra.iter().zip(rb.iter()) {
            assert!((x - y).abs() <= 1e-9 * x.abs().max(1.0), "{x} vs {y}");
        }
    }
    // Printed lying on its side, the soft direction is x.
    let side = Element::build(&printed_material(&effective, Vec3::X).unwrap_or_else(|| panic!("a material")), 1e-3)
        .unwrap_or_else(|e| panic!("element: {e}"));
    assert!((side.c[0][0] - b.c[2][2]).abs() <= 1e-9 * b.c[2][2].abs(), "{} vs {}", side.c[0][0], b.c[2][2]);
    assert!((side.c[2][2] - b.c[0][0]).abs() <= 1e-9 * b.c[0][0].abs());
    assert!(printed_material(&effective, Vec3::ZERO).is_none());
    assert!(printed_material(&Default::default(), Vec3::Z).is_none());
}

#[test]
fn layer_stress_reads_the_tension_and_the_shear_on_the_layer_planes() {
    // One voxel pulled 10 MPa along z and sheared 3 MPa in zx.
    let tensor = [1e6, 2e6, 10e6, 0.0, 0.0, 3e6];
    let z = layer_stress(&tensor, &[true], Vec3::Z).unwrap_or_else(|| panic!("a field"));
    assert!((z.max_normal - 10e6).abs() < 1.0 && (z.max_shear - 3e6).abs() < 1.0, "{z:?}");
    let x = layer_stress(&tensor, &[true], Vec3::X).unwrap_or_else(|| panic!("a field"));
    assert!((x.max_normal - 1e6).abs() < 1.0 && (x.max_shear - 3e6).abs() < 1.0, "{x:?}");
    // Compression is no tension.
    let c = layer_stress(&[0.0, 0.0, -5e6, 0.0, 0.0, 0.0], &[true], Vec3::Z).unwrap_or_else(|| panic!("a field"));
    assert_eq!(c.max_normal, 0.0);
    assert!(c.max_normal_voxel.is_none() && c.normal[0] == -5e6);
    assert!(layer_stress(&tensor[..5], &[true], Vec3::Z).is_none());
    assert!(layer_stress(&tensor, &[true], Vec3::ZERO).is_none());
}

#[test]
fn the_build_direction_decides_which_failure_governs() {
    // A flat bar bent down: printed lying flat its bending stress runs within the layers; printed
    // standing on end the same stress pulls the layers apart.
    let bar = box_mesh(80.0, 10.0, 10.0);
    let fixed = Region::NearestWall(Vec3::new(0.0, 5.0, 5.0));
    let load = Region::NearestWall(Vec3::new(80.0, 5.0, 5.0));
    let settings = PrintSettings::default();
    let flat = printed_stress(&bar, &fixed, &load, Vec3::new(0.0, 0.0, -20.0), &settings, Vec3::Z, 12, &MeshFemOptions::default())
        .unwrap_or_else(|e| panic!("flat: {e}"));
    assert!(flat.solve.converged && flat.solve.max_von_mises > 0.0);
    assert_eq!(flat.governs, Governs::WithinLayer, "flat: in-plane {} interlayer {}", flat.safety_in_plane, flat.safety_interlayer);
    let standing = printed_stress(&bar, &fixed, &load, Vec3::new(0.0, 0.0, -20.0), &settings, Vec3::X, 12, &MeshFemOptions::default())
        .unwrap_or_else(|e| panic!("standing: {e}"));
    assert_eq!(standing.governs, Governs::BetweenLayers, "standing: in-plane {} interlayer {}", standing.safety_in_plane, standing.safety_interlayer);
    assert!(standing.safety_interlayer < flat.safety_interlayer);
    assert!(standing.layer.max_normal > flat.layer.max_normal);
    assert!(standing.max_layer_normal_at.is_some_and(|p| p.x < 0.02), "the layers part at the root: {:?}", standing.max_layer_normal_at);
    // Softer across the layers, the standing bar bends more.
    assert!(standing.solve.max_displacement > flat.solve.max_displacement);
    assert!(printed_stress(&bar, &fixed, &load, Vec3::Z, &settings, Vec3::ZERO, 12, &MeshFemOptions::default()).is_err());
}

#[test]
fn every_pattern_weighs_the_same_and_they_differ_in_where_the_stiffness_goes() {
    let bar = box_mesh(60.0, 12.0, 6.0);
    let fixed = Region::NearestWall(Vec3::new(0.0, 6.0, 3.0));
    let load = Region::NearestWall(Vec3::new(60.0, 6.0, 3.0));
    let settings = PrintSettings { wall_loops: 1, top_bottom_layers: 2, ..PrintSettings::default() };
    let rows = compare_infill(&bar, &fixed, &load, Vec3::new(0.0, 0.0, -5.0), &settings, Vec3::Z, 8, &MeshFemOptions::default())
        .unwrap_or_else(|e| panic!("rows: {e}"));
    assert_eq!(rows.len(), INFILL_PATTERNS.len());
    assert!(rows.iter().zip(INFILL_PATTERNS).all(|(r, (p, _))| r.pattern == p), "in the patterns' order");
    let mass = rows[0].mass_grams;
    assert!(mass > 0.0 && rows.iter().all(|r| (r.mass_grams - mass).abs() < 1e-9 * mass), "the pattern is a free choice at a given density");
    let of = |p: InfillPattern| rows.iter().find(|r| r.pattern == p).unwrap_or_else(|| panic!("a row"));
    assert!(of(InfillPattern::Concentric).young_in_plane > of(InfillPattern::Grid).young_in_plane, "concentric follows the perimeter");
    assert!(of(InfillPattern::Lightning).young_in_plane < of(InfillPattern::Grid).young_in_plane);
    assert!(of(InfillPattern::Lightning).max_displacement > of(InfillPattern::Grid).max_displacement, "a softer lattice bends more");
    assert!(rows.iter().all(|r| r.safety_in_plane.is_finite() && r.safety_in_plane > 0.0), "{rows:?}");
    assert!(of(InfillPattern::Lightning).note.contains("suspicion"));
}
