use std::collections::HashMap;
use std::f64::consts::PI;

use solidcraft_geom::{Mesh, Vec3};

use crate::lattice::{BeamCell, Family, Grading, LatticeOptions, LatticeResult, TpmsType, beam_cell_struts, fill, marching_tetrahedra, tpms_value};

/// A closed box from `lo` to `hi` (mm), wound outwards.
fn box_mesh(lo: Vec3, hi: Vec3) -> Mesh {
    let p =
        |x: f64, y: f64, z: f64| Vec3::new(if x > 0.0 { hi.x } else { lo.x }, if y > 0.0 { hi.y } else { lo.y }, if z > 0.0 { hi.z } else { lo.z });
    let positions = vec![
        p(0.0, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(1.0, 1.0, 0.0),
        p(0.0, 1.0, 0.0),
        p(0.0, 0.0, 1.0),
        p(1.0, 0.0, 1.0),
        p(1.0, 1.0, 1.0),
        p(0.0, 1.0, 1.0),
    ];
    let triangles =
        vec![[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
    Mesh { positions, triangles, ..Default::default() }
}

fn cube(side: f64) -> Mesh {
    box_mesh(Vec3::ZERO, Vec3::new(side, side, side))
}

fn signed_volume(mesh: &Mesh) -> f64 {
    mesh.measure().volume
}

/// Every directed edge appears once and its reverse once: a closed, consistently wound surface.
fn is_closed(mesh: &Mesh) -> bool {
    let mut edges: HashMap<(u32, u32), i32> = HashMap::new();
    for t in &mesh.triangles {
        for n in 0..3 {
            let (a, b) = (t[n], t[(n + 1) % 3]);
            *edges.entry((a.min(b), a.max(b))).or_insert(0) += if a < b { 1 } else { -1 };
        }
    }
    !edges.is_empty() && edges.values().all(|v| *v == 0)
}

fn fill_cube(side: f64, options: &LatticeOptions<'_>) -> Option<LatticeResult> {
    fill(&cube(side), options)
}

#[test]
fn the_gyroid_at_level_zero_fills_half_the_box() {
    // The zero surface of the gyroid splits space into two congruent halves.
    let r = fill_cube(40.0, &LatticeOptions { family: Family::SolidTpms(TpmsType::Gyroid), cell_mm: 10.0, level: 0.0, ..LatticeOptions::default() })
        .unwrap_or_default();
    assert!((r.volume_fraction - 0.5).abs() < 0.04, "{}", r.volume_fraction);
    assert!(r.triangles > 1000, "{}", r.triangles);
    assert!(is_closed(&r.mesh));
    assert!(signed_volume(&r.mesh) > 0.0);
}

#[test]
fn a_higher_level_fills_more_of_the_solid_gyroid() {
    let at = |level: f64| {
        fill_cube(40.0, &LatticeOptions { family: Family::SolidTpms(TpmsType::Gyroid), cell_mm: 10.0, level, ..LatticeOptions::default() })
            .map(|r| r.volume_fraction)
            .unwrap_or(-1.0)
    };
    let (low, mid, high) = (at(-0.8), at(0.0), at(0.8));
    assert!(low < mid && mid < high, "{low} {mid} {high}");
    assert!(low > 0.1 && high < 0.9, "{low} {high}");
}

#[test]
fn a_walled_gyroid_grows_with_its_thickness() {
    let at = |thickness_mm: f64| {
        fill_cube(
            40.0,
            &LatticeOptions {
                family: Family::WalledTpms(TpmsType::Gyroid),
                cell_mm: 10.0,
                thickness_mm,
                samples_per_cell: 16,
                ..LatticeOptions::default()
            },
        )
        .map(|r| r.volume_fraction)
        .unwrap_or(-1.0)
    };
    let (thin, thick) = (at(1.0), at(2.0));
    // The gyroid's area per cell is about 3.1 a²; a sheet of thickness t fills ≈ 3.1 t / a.
    assert!((thin - 0.31).abs() < 0.08, "{thin}");
    assert!((thick - 0.62).abs() < 0.12, "{thick}");
    assert!(thick > thin * 1.6 && thick < thin * 2.4, "{thin} {thick}");
}

#[test]
fn a_schwarz_p_solid_is_a_closed_outward_surface() {
    let r =
        fill_cube(40.0, &LatticeOptions { family: Family::SolidTpms(TpmsType::SchwarzP), cell_mm: 10.0, level: 0.0, ..LatticeOptions::default() })
            .unwrap_or_default();
    assert!(is_closed(&r.mesh), "{} triangles", r.triangles);
    let volume = signed_volume(&r.mesh);
    assert!(volume > 0.0);
    // The meshed volume agrees with the sampled coverage.
    assert!((volume / 64_000.0 - r.volume_fraction).abs() < 1e-9, "{volume} {}", r.volume_fraction);
    let covered = r.fraction_where(|_| true);
    assert!((covered - r.volume_fraction).abs() < 0.05, "{covered} {}", r.volume_fraction);
    assert!(r.advice.is_empty(), "{}", r.advice);
    // Every vertex lies within the cube, as the fill is clipped to the body.
    let bounds = r.mesh.bounds();
    assert!(bounds.min.x > -0.01 && bounds.max.x < 40.01, "{bounds:?}");
}

#[test]
fn every_tpms_evaluates_and_meshes() {
    for kind in TpmsType::ALL {
        assert!(tpms_value(kind, 0.3, 1.1, 2.2).is_finite());
        assert_eq!(TpmsType::from_name(kind.key()), Some(kind));
        assert_eq!(TpmsType::from_name(kind.display()), Some(kind));
        let r = fill_cube(30.0, &LatticeOptions { family: Family::WalledTpms(kind), cell_mm: 10.0, thickness_mm: 1.5, ..LatticeOptions::default() });
        let r = r.unwrap_or_default();
        assert!(r.triangles > 0 && is_closed(&r.mesh), "{kind:?}");
        assert!(r.volume_fraction > 0.1 && r.volume_fraction < 0.85, "{kind:?} {}", r.volume_fraction);
    }
}

#[test]
fn a_simple_cubic_beam_lattice_weighs_what_its_struts_do() {
    // Per cell: three struts of length a (twelve edges, each shared by four cells), diameter d:
    // volume 3 π d²/4 a, minus the overlap at the nodes, which is small for a thin strut.
    let (a, d) = (10.0, 3.0);
    let r = fill_cube(
        40.0,
        &LatticeOptions {
            family: Family::Beam(BeamCell::SimpleCubic),
            cell_mm: a,
            thickness_mm: d,
            samples_per_cell: 16,
            ..LatticeOptions::default()
        },
    )
    .unwrap_or_default();
    // Three cylinders minus what they share at the node (three bicylinders less two tricylinders).
    let r3 = (d / 2.0).powi(3);
    let expected = (3.0 * PI * d * d / 4.0 * a - (3.0 * 16.0 / 3.0 * r3 - 2.0 * (16.0 - 8.0 * 2.0_f64.sqrt()) * r3)) / (a * a * a);
    assert!((r.volume_fraction - expected).abs() < 0.15 * expected, "{} vs {expected}", r.volume_fraction);
    assert!(is_closed(&r.mesh));
    assert!(signed_volume(&r.mesh) > 0.0);
}

#[test]
fn every_beam_cell_has_struts_and_meshes() {
    let expected = [
        (BeamCell::SimpleCubic, 12),
        (BeamCell::BodyCentred, 8),
        (BeamCell::FaceCentred, 24),
        (BeamCell::Octet, 36),
        (BeamCell::Diamond, 16),
        (BeamCell::Kelvin, 72),
    ];
    for (cell, struts) in expected {
        assert_eq!(beam_cell_struts(cell).len(), struts, "{cell:?}");
        assert_eq!(BeamCell::from_name(cell.key()), Some(cell));
        let r = fill_cube(30.0, &LatticeOptions { family: Family::Beam(cell), cell_mm: 10.0, thickness_mm: 2.0, ..LatticeOptions::default() })
            .unwrap_or_default();
        assert!(r.triangles > 0 && is_closed(&r.mesh), "{cell:?}");
        assert!(r.volume_fraction > 0.02 && r.volume_fraction < 0.9, "{cell:?} {}", r.volume_fraction);
    }
}

#[test]
fn a_thin_honeycomb_fills_two_t_over_root_three_a() {
    let (a, t) = (10.0, 0.6);
    let r = fill_cube(
        40.0,
        &LatticeOptions { family: Family::Honeycomb { axis: 2 }, cell_mm: a, thickness_mm: t, samples_per_cell: 16, ..LatticeOptions::default() },
    )
    .unwrap_or_default();
    let expected = 2.0 * t / (3.0_f64.sqrt() * a);
    assert!((r.volume_fraction - expected).abs() < 0.25 * expected, "{} vs {expected}", r.volume_fraction);
    assert!(is_closed(&r.mesh));
    // The cells run along Z: the fraction is the same in every slab along Z.
    let low = r.fraction_where(|p| p.z < 10.0);
    let high = r.fraction_where(|p| p.z > 30.0);
    assert!((low - high).abs() < 0.03, "{low} {high}");
}

#[test]
fn a_skin_keeps_the_surface_solid() {
    let bare = fill_cube(
        40.0,
        &LatticeOptions { family: Family::WalledTpms(TpmsType::Gyroid), cell_mm: 10.0, thickness_mm: 1.0, ..LatticeOptions::default() },
    )
    .unwrap_or_default();
    let skinned = fill_cube(
        40.0,
        &LatticeOptions { family: Family::WalledTpms(TpmsType::Gyroid), cell_mm: 10.0, thickness_mm: 1.0, skin_mm: 3.0, ..LatticeOptions::default() },
    )
    .unwrap_or_default();
    assert!(skinned.volume_fraction > bare.volume_fraction + 0.2, "{} vs {}", skinned.volume_fraction, bare.volume_fraction);
    // Inside the skin everything is filled.
    let rim = skinned.fraction_where(|p| p.x < 2.0 || p.x > 38.0);
    assert!(rim > 0.97, "{rim}");
    assert!(is_closed(&skinned.mesh));
}

#[test]
fn grading_along_an_axis_thickens_towards_its_end() {
    let r = fill_cube(
        40.0,
        &LatticeOptions {
            family: Family::WalledTpms(TpmsType::Gyroid),
            cell_mm: 10.0,
            thickness_min_mm: 0.8,
            thickness_max_mm: 3.0,
            grading: Grading::AlongAxis { from: Vec3::new(0.0, 20.0, 20.0), to: Vec3::new(40.0, 20.0, 20.0) },
            ..LatticeOptions::default()
        },
    )
    .unwrap_or_default();
    let (start, end) = (r.fraction_where(|p| p.x < 10.0), r.fraction_where(|p| p.x > 30.0));
    assert!(end > start * 1.8, "{start} {end}");
    let field = |p: Vec3| p.z / 40.0;
    let by_field = fill_cube(
        40.0,
        &LatticeOptions {
            family: Family::WalledTpms(TpmsType::Gyroid),
            cell_mm: 10.0,
            thickness_min_mm: 0.8,
            thickness_max_mm: 3.0,
            grading: Grading::Field(&field),
            ..LatticeOptions::default()
        },
    )
    .unwrap_or_default();
    assert!(by_field.fraction_where(|p| p.z > 30.0) > by_field.fraction_where(|p| p.z < 10.0) * 1.8);
    let radial = fill_cube(
        40.0,
        &LatticeOptions {
            family: Family::WalledTpms(TpmsType::Gyroid),
            cell_mm: 10.0,
            thickness_min_mm: 0.8,
            thickness_max_mm: 3.0,
            grading: Grading::FromPoint { centre: Vec3::new(20.0, 20.0, 20.0), radius: 20.0 },
            ..LatticeOptions::default()
        },
    )
    .unwrap_or_default();
    let centre = radial.fraction_where(|p| p.dist(Vec3::new(20.0, 20.0, 20.0)) < 8.0);
    let corners = radial.fraction_where(|p| p.dist(Vec3::new(20.0, 20.0, 20.0)) > 22.0);
    assert!(corners > centre * 1.5, "{centre} {corners}");
}

#[test]
fn the_advice_names_what_is_wrong() {
    let coarse =
        fill_cube(40.0, &LatticeOptions { cell_mm: 10.0, thickness_mm: 0.2, samples_per_cell: 4, ..LatticeOptions::default() }).unwrap_or_default();
    assert!(coarse.advice.contains("coarse") && coarse.advice.contains("thinner than two samples"), "{}", coarse.advice);
    let big = fill_cube(10.0, &LatticeOptions { cell_mm: 30.0, thickness_mm: 2.0, ..LatticeOptions::default() }).unwrap_or_default();
    assert!(big.advice.contains("thinnest side"), "{}", big.advice);
    // Walls too thin for the samples leave nothing, and the result says so instead of vanishing.
    let gone =
        fill_cube(10.0, &LatticeOptions { family: Family::SolidTpms(TpmsType::Gyroid), cell_mm: 10.0, level: -1.6, ..LatticeOptions::default() })
            .unwrap_or_default();
    assert!(gone.triangles == 0 && gone.advice.contains("no material"), "{} {}", gone.triangles, gone.advice);
    // A grid that would exceed the budget is coarsened and says so.
    let huge = fill(
        &box_mesh(Vec3::ZERO, Vec3::new(400.0, 400.0, 400.0)),
        &LatticeOptions { cell_mm: 10.0, thickness_mm: 2.0, samples_per_cell: 16, ..LatticeOptions::default() },
    )
    .unwrap_or_default();
    assert!(huge.advice.contains("resolution limited"), "{}", huge.advice);
    assert!(huge.grid[0] * huge.grid[1] * huge.grid[2] <= crate::lattice::MAX_SAMPLES);
}

#[test]
fn hostile_options_return_none_and_never_panic() {
    let body = cube(20.0);
    for cell in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(fill(&body, &LatticeOptions { cell_mm: cell, ..LatticeOptions::default() }).is_none(), "{cell}");
    }
    for thickness in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(fill(&body, &LatticeOptions { thickness_mm: thickness, ..LatticeOptions::default() }).is_none(), "{thickness}");
    }
    assert!(fill(&body, &LatticeOptions { skin_mm: -1.0, ..LatticeOptions::default() }).is_none());
    assert!(fill(&body, &LatticeOptions { level: f64::NAN, family: Family::SolidTpms(TpmsType::Gyroid), ..LatticeOptions::default() }).is_none());
    assert!(fill(&body, &LatticeOptions { family: Family::Honeycomb { axis: 7 }, ..LatticeOptions::default() }).is_none());
    assert!(
        fill(
            &body,
            &LatticeOptions {
                grading: Grading::FromPoint { centre: Vec3::ZERO, radius: 0.0 },
                thickness_min_mm: 1.0,
                thickness_max_mm: 2.0,
                ..LatticeOptions::default()
            }
        )
        .is_none()
    );
    assert!(fill(&Mesh::default(), &LatticeOptions::default()).is_none());
    // A flat (empty-volume) mesh has no interior.
    let flat = box_mesh(Vec3::ZERO, Vec3::new(10.0, 10.0, 0.0));
    assert!(fill(&flat, &LatticeOptions::default()).is_none());
    // Tiny bodies with tiny cells are capped, not run away with.
    let tiny = fill(&body, &LatticeOptions { cell_mm: 0.001, thickness_mm: 0.0001, ..LatticeOptions::default() });
    assert!(tiny.is_none_or(|r| r.grid[0] * r.grid[1] * r.grid[2] <= crate::lattice::MAX_SAMPLES));
}

#[test]
fn marching_tetrahedra_meshes_a_sphere_to_its_volume() {
    let (n, h) = (41usize, 1.0);
    let centre = Vec3::new(20.0, 20.0, 20.0);
    let radius = 15.0;
    let origin = Vec3::ZERO;
    let field: Vec<f64> = (0..n * n * n)
        .map(|idx| {
            let (i, j, k) = (idx % n, (idx / n) % n, idx / (n * n));
            (origin + Vec3::new(i as f64, j as f64, k as f64) * h).dist(centre) - radius
        })
        .collect();
    let mesh = marching_tetrahedra(&field, n, n, n, origin, h).unwrap_or_default();
    assert!(is_closed(&mesh));
    let volume = signed_volume(&mesh);
    let exact = 4.0 / 3.0 * PI * radius.powi(3);
    assert!((volume - exact).abs() < 0.02 * exact, "{volume} vs {exact}");
    // Every normal points away from the centre.
    for t in &mesh.triangles {
        let [a, b, c] = mesh.tri(t).unwrap_or([Vec3::ZERO; 3]);
        let normal = (b - a).cross(c - a);
        assert!(normal.dot((a + b + c) / 3.0 - centre) > 0.0);
    }
    assert!(marching_tetrahedra(&field, n, n, 1, origin, h).is_none());
    assert!(marching_tetrahedra(&field[..10], n, n, n, origin, h).is_none());
    assert!(marching_tetrahedra(&field, n, n, n, origin, 0.0).is_none());
}
