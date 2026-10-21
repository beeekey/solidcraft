//! The solver held to closed-form elasticity.

use solidcraft_geom::{Mesh, Vec3};

use crate::{
    Assembly, ContactOptions, LoadCase, MeshFemError, MeshFemOptions, OrthotropicMaterial, Region, VoxelFemProblem, point_triangle_distance,
    solve_assembly, solve_mesh, voxel_fem::solve_voxels, voxelise,
};

/// A closed box mesh from the origin, in millimetres, triangles counter-clockwise from outside.
fn box_mesh(a: f64, b: f64, c: f64) -> Mesh {
    let p = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
    let positions = vec![p(0.0, 0.0, 0.0), p(a, 0.0, 0.0), p(a, b, 0.0), p(0.0, b, 0.0), p(0.0, 0.0, c), p(a, 0.0, c), p(a, b, c), p(0.0, b, c)];
    let triangles = vec![
        [0, 2, 1],
        [0, 3, 2], // bottom (normal -z)
        [4, 5, 6],
        [4, 6, 7], // top
        [0, 1, 5],
        [0, 5, 4], // front (y = 0)
        [2, 3, 7],
        [2, 7, 6], // back
        [1, 2, 6],
        [1, 6, 5], // right (x = a)
        [3, 0, 4],
        [3, 4, 7], // left
    ];
    let mut mesh = Mesh { positions, triangles, ..Default::default() };
    mesh.normals = vec![Vec3::ZERO; 8];
    mesh.tri_face = (0..12).map(|t| (t / 2) as u32).collect();
    mesh
}

fn cantilever(nx: usize, ny: usize, nz: usize, h: f64, force: Vec3, e: f64) -> VoxelFemProblem {
    let mut problem = VoxelFemProblem::new(nx, ny, nz, h, OrthotropicMaterial::isotropic(e, 0.3));
    problem.max_iterations = 4000;
    let mut tip = Vec::new();
    for k in 0..nz {
        for j in 0..ny {
            let root = problem.index(0, j, k);
            problem.held[root] = true;
            tip.push(problem.index(nx - 1, j, k));
        }
    }
    problem.cases.push(LoadCase { voxels: tip, force, name: "tip".into() });
    problem
}

#[test]
fn an_axial_rod_obeys_hookes_law() {
    // 100 × 10 × 10 mm steel rod pulled with 1 kN: trilinear hexes represent the constant-strain
    // state exactly, so δ = FL/EA and von Mises equals F/A within the end-effect tolerance.
    let rod = box_mesh(100.0, 10.0, 10.0);
    let (e, f, a, l) = (210e9, 1000.0, 1e-4, 0.100);
    let result = solve_mesh(
        &rod,
        &Region::NearestWall(Vec3::new(0.0, 5.0, 5.0)),
        &Region::NearestWall(Vec3::new(100.0, 5.0, 5.0)),
        Vec3::new(f, 0.0, 0.0),
        &OrthotropicMaterial::isotropic(e, 0.3),
        7850.0,
        30,
        &MeshFemOptions::default(),
    )
    .unwrap_or_default();
    assert!(result.converged, "{}", result.mesh_advice);
    let analytic = f * l / (e * a); // 4.76 µm
    // The clamped face suppresses Poisson contraction (Saint-Venant), so the rod reads a few
    // percent stiff at L/D = 10.
    assert!((result.load_displacement - analytic).abs() < 0.08 * analytic, "{} vs {analytic}", result.load_displacement);
    assert!((result.max_von_mises - f / a).abs() < 0.15 * (f / a), "{} vs {}", result.max_von_mises, f / a);
}

#[test]
fn a_cantilever_bends_the_right_way_and_magnitude() {
    // 100 × 10 × 10 mm steel cantilever, 100 N down at the tip. Euler–Bernoulli predicts 0.19 mm;
    // coarse trilinear hexes are stiffer in bending (shear locking), so accept the order of
    // magnitude and direction, plus the stress maximum at the clamped end.
    let beam = box_mesh(100.0, 10.0, 10.0);
    let (e, f) = (210e9, 100.0);
    let result = solve_mesh(
        &beam,
        &Region::NearestWall(Vec3::new(0.0, 5.0, 5.0)),
        &Region::NearestWall(Vec3::new(100.0, 5.0, 5.0)),
        Vec3::new(0.0, 0.0, -f),
        &OrthotropicMaterial::isotropic(e, 0.3),
        7850.0,
        40,
        &MeshFemOptions::default(),
    )
    .unwrap_or_default();
    assert!(result.converged);
    let inertia = 0.010_f64.powi(4) / 12.0;
    let analytic = f * 0.100_f64.powi(3) / (3.0 * e * inertia); // 1.9e-4 m
    assert!(result.load_displacement > 0.4 * analytic && result.load_displacement < 1.3 * analytic, "{} vs {analytic}", result.load_displacement);
    assert!(result.max_displacement > 0.4 * analytic);
    // Peak stress at the clamped end, not at the tip.
    assert!(result.max_von_mises_at.x < 0.020, "{:?}", result.max_von_mises_at);
    // Bending stress at the outermost element centre: σ = F (L − h/2)(c − h/2) / I ≈ 44 MPa.
    let h = 0.100 / 40.0;
    let centre_stress = f * (0.100 - h / 2.0) * (0.005 - h / 2.0) / inertia;
    assert!(result.max_von_mises > 0.8 * centre_stress && result.max_von_mises < 1.3 * centre_stress, "{} vs {centre_stress}", result.max_von_mises);
}

#[test]
fn stiffness_scales_with_youngs_modulus() {
    let rod = box_mesh(50.0, 10.0, 10.0);
    let solve = |e: f64| {
        solve_mesh(
            &rod,
            &Region::NearestWall(Vec3::new(0.0, 5.0, 5.0)),
            &Region::NearestWall(Vec3::new(50.0, 5.0, 5.0)),
            Vec3::new(500.0, 0.0, 0.0),
            &OrthotropicMaterial::isotropic(e, 0.3),
            2700.0,
            20,
            &MeshFemOptions::default(),
        )
        .unwrap_or_default()
    };
    let (soft, stiff) = (solve(70e9), solve(140e9));
    assert!((soft.load_displacement - 2.0 * stiff.load_displacement).abs() < 0.01 * soft.load_displacement);
    assert!((soft.max_von_mises - stiff.max_von_mises).abs() < 0.01 * soft.max_von_mises);
}

#[test]
fn the_voxel_solver_agrees_with_beam_theory_on_a_cantilever() {
    // 120 × 20 × 20 mm bar held at x = 0, pushed down at x = L.
    let (length, side): (f64, f64) = (0.120, 0.020);
    let (nx, ny, nz) = (36, 6, 6);
    let problem = cantilever(nx, ny, nz, length / nx as f64, Vec3::new(0.0, 0.0, -500.0), 200e9);
    let result = solve_voxels(&problem);
    assert!(result.solved, "{}", result.advice);
    assert_eq!(result.cases.len(), 1);
    assert!(result.cases[0].converged);
    assert_eq!(result.elements, nx * ny * nz);

    // PL³/(3EI) plus the shear term that matters on a bar this stubby (L/h = 6).
    let inertia = side.powi(4) / 12.0;
    let bending = 500.0 * length.powi(3) / (3.0 * 200e9 * inertia);
    let shear = 500.0 * length / (0.85 * side * side * 200e9 / (2.0 * 1.3));
    let expected = bending + shear;
    assert!((result.worst_displacement - expected).abs() < 0.25 * expected, "{} mm vs {} mm", result.worst_displacement * 1e3, expected * 1e3);
    // Peak bending stress at the root, M c / I.
    let expected_stress = 500.0 * length * (side / 2.0) / inertia;
    assert!(result.worst_von_mises > 0.5 * expected_stress && result.worst_von_mises < 2.0 * expected_stress);
    assert!(result.worst_compliance > 0.0);
}

#[test]
fn the_voxel_solver_refuses_an_unsupported_or_unloaded_problem() {
    let mut problem = VoxelFemProblem::new(4, 4, 4, 0.01, OrthotropicMaterial::isotropic(70e9, 0.33));
    problem.cases.push(LoadCase { voxels: vec![63], force: Vec3::new(0.0, 0.0, -10.0), name: String::new() });
    // Nothing holds it: a singular system, and saying so beats a confident rigid-body displacement.
    let floating = solve_voxels(&problem);
    assert!(!floating.solved);
    assert!(floating.advice.contains("holds"), "{}", floating.advice);
    // Held, but the load lands where there is no material: reported unsolved rather than scoring
    // as infinitely stiff.
    problem.held[0] = true;
    problem.solid[63] = false;
    let missing = solve_voxels(&problem);
    assert!(missing.solved);
    assert_eq!(missing.cases.len(), 1);
    assert!(!missing.cases[0].converged);
    assert_eq!(missing.cases[0].compliance, 0.0);
    // And no cases at all is refused outright.
    problem.cases.clear();
    assert!(!solve_voxels(&problem).solved);
}

#[test]
fn each_load_case_is_solved_on_its_own() {
    // Two load cases are two questions: a bracket has to survive braking and cornering, not
    // their sum.
    let (nx, ny, nz) = (20, 5, 5);
    let mut problem = cantilever(nx, ny, nz, 0.005, Vec3::new(0.0, 0.0, -200.0), 70e9);
    problem.max_iterations = 3000;
    let tip = problem.cases[0].voxels.clone();
    problem.cases.push(LoadCase { voxels: tip, force: Vec3::new(200.0, 0.0, 0.0), name: "pull".into() });
    let result = solve_voxels(&problem);
    assert!(result.solved, "{}", result.advice);
    assert_eq!(result.cases.len(), 2);
    // Bending sideways is far more compliant than stretching along the length.
    assert!(result.cases[0].compliance > 10.0 * result.cases[1].compliance);
    assert_eq!(result.worst_case, Some(0));
    assert_eq!(result.worst_compliance, result.cases[0].compliance);
}

#[test]
fn hostile_grids_are_refused_not_panicked() {
    let material = OrthotropicMaterial::isotropic(70e9, 0.33);
    let mut zero = VoxelFemProblem::new(0, 3, 3, 0.01, material);
    zero.cases.push(LoadCase::default());
    assert!(!solve_voxels(&zero).solved);
    let mut bad_h = VoxelFemProblem::new(3, 3, 3, f64::NAN, material);
    bad_h.cases.push(LoadCase::default());
    assert!(!solve_voxels(&bad_h).solved);
    let mut short_mask = VoxelFemProblem::new(3, 3, 3, 0.01, material);
    short_mask.solid.pop();
    short_mask.cases.push(LoadCase::default());
    assert!(!solve_voxels(&short_mask).solved);
    let mut wild_voxel = VoxelFemProblem::new(3, 3, 3, 0.01, material);
    wild_voxel.held[0] = true;
    wild_voxel.cases.push(LoadCase { voxels: vec![usize::MAX, 26], force: Vec3::new(1.0, 0.0, 0.0), name: String::new() });
    assert!(solve_voxels(&wild_voxel).solved);
    let empty = Mesh::default();
    assert!(
        solve_mesh(&empty, &Region::NearestWall(Vec3::ZERO), &Region::NearestWall(Vec3::X), Vec3::X, &material, 1.0, 20, &MeshFemOptions::default())
            .is_err()
    );
    let nan_material = OrthotropicMaterial::isotropic(f64::NAN, 0.3);
    assert!(
        solve_mesh(
            &box_mesh(10.0, 10.0, 10.0),
            &Region::NearestWall(Vec3::ZERO),
            &Region::NearestWall(Vec3::X),
            Vec3::X,
            &nan_material,
            1.0,
            20,
            &MeshFemOptions::default()
        )
        .is_err()
    );
}

#[test]
fn a_picked_face_holds_exactly_the_cells_it_touches() {
    // The left end of the bar (x = 0) is triangles 10 and 11 of `box_mesh`; held there and pulled
    // at the right end (8, 9) the bar obeys Hooke's law as the wall-slab path does, and the two
    // regions are the two end slabs.
    let rod = box_mesh(100.0, 10.0, 10.0);
    let grid = voxelise(&rod, 30, 1_000_000).unwrap_or_else(|| panic!("a grid"));
    let left = grid.face_slab(&rod, &[10, 11]);
    let right = grid.face_slab(&rod, &[8, 9]);
    assert_eq!(left.len(), grid.ny * grid.nz, "{left:?}");
    assert!(left.iter().all(|v| grid.ijk(*v).0 == 0));
    assert!(right.iter().all(|v| grid.ijk(*v).0 == grid.nx - 1));
    let (e, f, a, l) = (210e9, 1000.0, 1e-4, 0.100);
    let result = solve_mesh(
        &rod,
        &Region::Face(vec![10, 11]),
        &Region::Face(vec![8, 9]),
        Vec3::new(f, 0.0, 0.0),
        &OrthotropicMaterial::isotropic(e, 0.3),
        7850.0,
        30,
        &MeshFemOptions::default(),
    )
    .unwrap_or_default();
    let analytic = f * l / (e * a);
    assert!((result.load_displacement - analytic).abs() < 0.08 * analytic, "{} vs {analytic}", result.load_displacement);
    // A face that touches nothing is refused, not solved as if the whole box were held.
    assert!(
        solve_mesh(
            &rod,
            &Region::Face(vec![999]),
            &Region::Face(vec![8, 9]),
            Vec3::X,
            &OrthotropicMaterial::isotropic(e, 0.3),
            1.0,
            20,
            &MeshFemOptions::default()
        )
        .is_err()
    );
}

#[test]
fn point_triangle_distance_matches_the_easy_cases() {
    let (a, b, c) = (Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
    assert!((point_triangle_distance(Vec3::new(2.0, 2.0, 3.0), a, b, c) - 3.0).abs() < 1e-12);
    assert!((point_triangle_distance(Vec3::new(-4.0, 0.0, 0.0), a, b, c) - 4.0).abs() < 1e-12);
    assert!((point_triangle_distance(Vec3::new(10.0, 10.0, 0.0), a, b, c) - 50f64.sqrt()).abs() < 1e-12);
}

/// A box translated by `by` (mm).
fn moved(mesh: &Mesh, by: Vec3) -> Mesh {
    let mut m = mesh.clone();
    for p in &mut m.positions {
        *p += by;
    }
    m
}

#[test]
fn two_bonded_blocks_carry_a_push_like_one_and_a_contact_opens_under_a_pull() {
    // A 40 × 20 × 20 mm block held at x = 0 with a second 40 × 20 × 20 block butted against its
    // end face. Pushed along −X at the far end, the bonded pair and the pair in contact answer
    // alike (the interface is closed either way). Pulled along +X, the bonded pair stretches as
    // one bar; the pair in contact comes apart: nothing holds the second block.
    let a = box_mesh(40.0, 20.0, 20.0);
    let b = moved(&box_mesh(40.0, 20.0, 20.0), Vec3::new(40.0, 0.0, 0.0));
    let material = OrthotropicMaterial::isotropic(210e9, 0.3);
    let held = Region::NearestWall(Vec3::new(0.0, 10.0, 10.0));
    let far = Region::NearestWall(Vec3::new(80.0, 10.0, 10.0));
    let assembly = |fixed: Region, load: Region| Assembly { meshes: vec![&a, &b], modulus_scale: vec![1.0, 1.0], fixed: (0, fixed), load: (1, load) };
    let push = Vec3::new(-1000.0, 0.0, 0.0);
    let bonded = solve_assembly(&assembly(held.clone(), far.clone()), push, &material, 7850.0, 24, &MeshFemOptions::default()).unwrap_or_default();
    let touching = MeshFemOptions { contact: Some(ContactOptions { friction: 0.0, preload_n: 0.0 }), ..MeshFemOptions::default() };
    let contact = solve_assembly(&assembly(held.clone(), far.clone()), push, &material, 7850.0, 24, &touching).unwrap_or_default();
    assert!(bonded.converged && contact.converged, "{} / {}", bonded.mesh_advice, contact.mesh_advice);
    assert!(bonded.contact.is_none() && bonded.body_of.len() == bonded.solid.len());
    let report = contact.contact.clone().unwrap_or_default();
    assert!(report.links > 0 && report.open_links == 0, "{report:?}");
    // Hooke: δ = FL/EA = 1000 × 0.08 / (210e9 × 4e-4) = 0.95 µm, the same for both.
    let analytic = 1000.0 * 0.08 / (210e9 * 4e-4);
    assert!((bonded.load_displacement.abs() - analytic).abs() < 0.1 * analytic, "{} vs {analytic}", bonded.load_displacement);
    assert!(
        (contact.load_displacement - bonded.load_displacement).abs() < 0.1 * analytic,
        "{} vs {}",
        contact.load_displacement,
        bonded.load_displacement
    );

    let pull = Vec3::new(1000.0, 0.0, 0.0);
    let stretched = solve_assembly(&assembly(held.clone(), far.clone()), pull, &material, 7850.0, 24, &MeshFemOptions::default()).unwrap_or_default();
    assert!((stretched.load_displacement - analytic).abs() < 0.1 * analytic, "{}", stretched.load_displacement);
    let apart = solve_assembly(&assembly(held, far), pull, &material, 7850.0, 24, &touching);
    assert!(matches!(apart, Err(MeshFemError::CameApart)), "{:?}", apart.map(|r| r.load_displacement));
}

#[test]
fn a_softer_second_body_stretches_more() {
    // The second block at a tenth of the stiffness: the pair in series stretches by
    // (1 + 10) / 2 times what two steel blocks do.
    let a = box_mesh(40.0, 20.0, 20.0);
    let b = moved(&box_mesh(40.0, 20.0, 20.0), Vec3::new(40.0, 0.0, 0.0));
    let material = OrthotropicMaterial::isotropic(210e9, 0.3);
    let held = Region::NearestWall(Vec3::new(0.0, 10.0, 10.0));
    let far = Region::NearestWall(Vec3::new(80.0, 10.0, 10.0));
    let pull = Vec3::new(1000.0, 0.0, 0.0);
    let steel = Assembly { meshes: vec![&a, &b], modulus_scale: vec![1.0, 1.0], fixed: (0, held.clone()), load: (1, far.clone()) };
    let soft = Assembly { meshes: vec![&a, &b], modulus_scale: vec![1.0, 0.1], fixed: (0, held), load: (1, far) };
    let same = solve_assembly(&steel, pull, &material, 7850.0, 24, &MeshFemOptions::default()).unwrap_or_default();
    let mixed = solve_assembly(&soft, pull, &material, 7850.0, 24, &MeshFemOptions::default()).unwrap_or_default();
    let ratio = mixed.load_displacement / same.load_displacement;
    assert!((ratio - 5.5).abs() < 0.6, "{ratio}");
}

#[test]
fn an_assembly_with_bad_input_is_refused() {
    let a = box_mesh(40.0, 20.0, 20.0);
    let material = OrthotropicMaterial::isotropic(210e9, 0.3);
    let held = Region::NearestWall(Vec3::new(0.0, 10.0, 10.0));
    let bad_scale = Assembly { meshes: vec![&a, &a], modulus_scale: vec![1.0], fixed: (0, held.clone()), load: (1, held.clone()) };
    assert!(matches!(solve_assembly(&bad_scale, Vec3::X, &material, 1.0, 20, &MeshFemOptions::default()), Err(MeshFemError::Input)));
    let bad_body = Assembly { meshes: vec![&a], modulus_scale: vec![1.0], fixed: (0, held.clone()), load: (3, held) };
    assert!(matches!(solve_assembly(&bad_body, Vec3::X, &material, 1.0, 20, &MeshFemOptions::default()), Err(MeshFemError::Input)));
}
