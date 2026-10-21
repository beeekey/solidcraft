//! Implicit fields held to geometry: a sampled box measures as a box, a blend adds a fillet's
//! worth of material, a shell is the skin, cells come back as a body.

use solidcraft_geom::{Mesh, Vec3};

use crate::implicit::{Boolean, Grading, blend, cells_mesh, offset, sample_mesh, sample_meshes, shell, smooth_min, spacing_for};

fn box_mesh(min: Vec3, size: Vec3) -> Mesh {
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

fn volume(m: &Mesh) -> f64 {
    m.measure().volume.abs()
}

#[test]
fn a_sampled_box_comes_back_as_a_box() {
    let cube = box_mesh(Vec3::ZERO, Vec3::new(20.0, 20.0, 20.0));
    let field = sample_mesh(&cube, 1.0, 2.0, 1_000_000).unwrap_or_else(|| panic!("a field"));
    assert!(field.inside_nodes() > 0);
    // The centre is deep inside, a far corner well outside.
    let centre = field
        .values
        .iter()
        .enumerate()
        .min_by(|a, b| (field.node(a.0) - Vec3::new(10.0, 10.0, 10.0)).len().total_cmp(&(field.node(b.0) - Vec3::new(10.0, 10.0, 10.0)).len()))
        .map(|(_, v)| *v)
        .unwrap_or(0.0);
    assert!(centre < -8.0, "{centre}");
    let back = field.mesh().unwrap_or_else(|| panic!("a mesh"));
    let v = volume(&back);
    assert!((v - 8000.0).abs() < 0.08 * 8000.0, "volume {v}");
    assert!(sample_mesh(&cube, 0.01, 1.0, 10_000).is_none(), "too fine is refused");
    assert!(sample_mesh(&Mesh::default(), 1.0, 1.0, 10_000).is_none());
}

#[test]
fn smooth_minimum_rounds_the_seam_and_a_blend_adds_material() {
    assert_eq!(smooth_min(1.0, 2.0, 0.0), 1.0);
    assert!(smooth_min(1.0, 1.0, 2.0) < 1.0, "two equal distances blend below either");
    assert!((smooth_min(-5.0, 10.0, 1.0) + 5.0).abs() < 1e-12, "far apart, the plain minimum");
    let a = box_mesh(Vec3::ZERO, Vec3::new(30.0, 10.0, 10.0));
    let b = box_mesh(Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 30.0));
    let fields = sample_meshes(&[&a, &b], 0.5, 4.0, 4_000_000).unwrap_or_else(|| panic!("fields"));
    let plain = blend(&fields[0], &fields[1], Boolean::Join, 0.0).unwrap_or_else(|| panic!("a join"));
    let rounded = blend(&fields[0], &fields[1], Boolean::Join, 3.0).unwrap_or_else(|| panic!("a join"));
    let (vp, vr) = (volume(&plain.mesh().unwrap_or_default()), volume(&rounded.mesh().unwrap_or_default()));
    let union = 3000.0 + 3000.0 - 1000.0;
    assert!((vp - union).abs() < 0.1 * union, "plain union {vp}");
    assert!(vr > vp + 20.0, "the fillet adds material: {vp} → {vr}");
    let cut = blend(&fields[0], &fields[1], Boolean::Cut, 0.0).unwrap_or_else(|| panic!("a cut"));
    let vc = volume(&cut.mesh().unwrap_or_default());
    assert!((vc - 2000.0).abs() < 0.12 * 2000.0, "cut {vc}");
    let common = blend(&fields[0], &fields[1], Boolean::Intersect, 0.0).unwrap_or_else(|| panic!("an intersection"));
    let vi = volume(&common.mesh().unwrap_or_default());
    assert!((vi - 1000.0).abs() < 0.15 * 1000.0, "intersection {vi}");
    assert!(Boolean::from_name("Union") == Some(Boolean::Join) && Boolean::from_name("weld").is_none());
}

#[test]
fn a_shell_is_the_skin_and_an_offset_grows_the_body() {
    let cube = box_mesh(Vec3::ZERO, Vec3::new(30.0, 30.0, 30.0));
    let field = sample_mesh(&cube, 0.75, 4.0, 4_000_000).unwrap_or_else(|| panic!("a field"));
    let skin = shell(&field, &Grading::Uniform(3.0));
    let v = volume(&skin.mesh().unwrap_or_default());
    let expected = 27_000.0 - 24.0_f64.powi(3);
    assert!((v - expected).abs() < 0.12 * expected, "shell {v} vs {expected}");
    let grown = offset(&field, &Grading::Uniform(2.0));
    let vg = volume(&grown.mesh().unwrap_or_default());
    assert!(vg > 30_000.0 && vg < 34.0_f64.powi(3) * 1.05, "grown {vg}");
    // Graded: thin at one end, thick at the other, so the skin's volume lies between the two
    // uniform shells.
    let graded = shell(&field, &Grading::AlongAxis { from: Vec3::ZERO, to: Vec3::new(30.0, 0.0, 0.0), min: 1.5, max: 4.5 });
    let vgr = volume(&graded.mesh().unwrap_or_default());
    let thin = 27_000.0 - 27.0_f64.powi(3);
    let thick = 27_000.0 - 21.0_f64.powi(3);
    assert!(vgr > thin && vgr < thick, "graded {vgr} between {thin} and {thick}");
    let g = Grading::FromPoint { centre: Vec3::ZERO, radius: 10.0, min: 1.0, max: 3.0 };
    assert!(
        (g.at(Vec3::ZERO) - 1.0).abs() < 1e-12
            && (g.at(Vec3::new(5.0, 0.0, 0.0)) - 2.0).abs() < 1e-12
            && (g.at(Vec3::new(50.0, 0.0, 0.0)) - 3.0).abs() < 1e-12
    );
    assert_eq!(g.reach(), 3.0);
}

#[test]
fn kept_cells_become_a_body_and_a_lone_strut_survives() {
    // A 10 × 2 × 2 block of cells, one cell high strut along x in the middle row kept.
    let grid = [10usize, 3, 3];
    let keep = |v: usize| {
        let (i, j, k) = (v % 10, (v / 10) % 3, v / 30);
        let _ = i;
        j == 1 && k == 1
    };
    let m = cells_mesh(grid, Vec3::ZERO, 2.0, keep).unwrap_or_else(|| panic!("a mesh"));
    assert!(!m.triangles.is_empty());
    let v = volume(&m);
    // A one-cell strut 20 mm long with a 2 mm section, its corners eased.
    assert!(v > 45.0 && v <= 80.0 * 1.01, "strut volume {v}");
    assert!(cells_mesh([0, 3, 3], Vec3::ZERO, 2.0, |_| true).is_none());
    assert!((spacing_for(Vec3::new(100.0, 20.0, 10.0), 50.0, 0.2) - 2.0).abs() < 1e-12);
    assert!((spacing_for(Vec3::new(1.0, 1.0, 1.0), 50.0, 0.2) - 0.2).abs() < 1e-12);
}

#[test]
fn a_full_block_of_cells_is_the_block() {
    let m = cells_mesh([18, 3, 6], Vec3::ZERO, 10.0 / 3.0, |_| true).unwrap_or_else(|| panic!("a mesh"));
    let v = volume(&m);
    assert!(v > 0.9 * 12_000.0 && v <= 12_000.0 * 1.01, "block volume {v}");
    let b = m.bounds();
    assert!(b.min.x.abs() < 1e-9 && (b.max.x - 60.0).abs() < 1e-9 && (b.max.z - 20.0).abs() < 1e-9, "the faces are exact: {b:?}");
    let half = cells_mesh([18, 3, 6], Vec3::ZERO, 10.0 / 3.0, |c| (c % 18) < 9).unwrap_or_else(|| panic!("a mesh"));
    let vh = volume(&half);
    assert!(vh > 0.9 * 6_000.0 && vh < 6_000.0 * 1.01, "half volume {vh}");
}
