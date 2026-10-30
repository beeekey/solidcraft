//! Buckling held to Euler's column, and the co-rotational solve to what a bent beam does.

use solidcraft_geom::Vec3;

use crate::{
    BucklingResult, LargeDisplacementOptions, LoadCase, OrthotropicMaterial, VoxelFemProblem, buckling, buckling_rayleigh, large_displacement,
    limit_load, solve_voxels,
};

/// A cantilever column on the grid, held at k = 0, pushed at the top.
fn slender_column(length: f64, across: usize, along: usize, force: Vec3) -> VoxelFemProblem {
    let mut problem = VoxelFemProblem::new(across, across, along, length / along as f64, OrthotropicMaterial::isotropic(200e9, 0.3));
    problem.max_iterations = 6000;
    problem.tolerance = 1e-11;
    let mut load = LoadCase { force, name: "push".into(), ..Default::default() };
    for j in 0..across {
        for i in 0..across {
            let root = problem.index(i, j, 0);
            problem.held[root] = true;
            load.voxels.push(problem.index(i, j, along - 1));
        }
    }
    problem.cases.push(load);
    problem
}

/// A cosine bow of amplitude `bow` in the shape a fixed-free column buckles in, with the
/// cross-sections turned as well as moved: sliding them sideways upright would be a shear.
fn bowed_column(problem: &VoxelFemProblem, bow: f64) -> Vec<f64> {
    let (nnx, nny, nnz) = (problem.nx + 1, problem.ny + 1, problem.nz + 1);
    let length = problem.h * problem.nz as f64;
    let centre = problem.h * problem.nx as f64 / 2.0;
    let mut crooked = vec![0.0; nnx * nny * nnz * 3];
    for k in 0..nnz {
        let z = k as f64 * problem.h;
        let sideways = bow * (1.0 - (std::f64::consts::PI * z / (2.0 * length)).cos());
        let slope = bow * (std::f64::consts::PI / (2.0 * length)) * (std::f64::consts::PI * z / (2.0 * length)).sin();
        for j in 0..nny {
            for i in 0..nnx {
                let node = (k * nny + j) * nnx + i;
                crooked[3 * node] = sideways;
                crooked[3 * node + 2] = -(i as f64 * problem.h - centre) * slope;
            }
        }
    }
    crooked
}

#[test]
fn buckling_matches_eulers_column() {
    // Euler had the answer in 1744: a column fixed at one end and free at the other buckles at
    // π² E I / (4 L²).
    let length = 0.300;
    let (nx, ny, nz) = (3, 3, 60);
    let mut problem = slender_column(length, nx, nz, Vec3::new(0.0, 0.0, -1000.0));
    problem.max_iterations = 4000;
    problem.tolerance = 1e-10;
    let side = nx as f64 * problem.h; // the mesh is the column: 15 mm square
    let result = solve_voxels(&problem);
    assert!(result.solved, "{}", result.advice);
    let folded = buckling(&problem, &result, 30);
    assert!(folded.solved, "{}", folded.advice);
    let inertia = side.powi(4) / 12.0;
    let critical = std::f64::consts::PI.powi(2) * 200e9 * inertia / (4.0 * length * length);
    let expected = critical / 1000.0;
    // Three elements across is a coarse column, and Euler ignores shear.
    assert!((folded.load_factor - expected).abs() < 0.35 * expected, "folds at {} N, Euler says {critical} N", folded.load_factor * 1000.0);
    assert!(folded.iterations > 0);
    assert_eq!(folded.mode.len(), (nx + 1) * (ny + 1) * (nz + 1) * 3);
}

#[test]
fn a_column_in_tension_has_no_buckling_mode_and_says_so() {
    let mut problem = slender_column(0.120, 3, 24, Vec3::new(0.0, 0.0, 2000.0));
    problem.max_iterations = 2000;
    let result = solve_voxels(&problem);
    assert!(result.solved);
    let folded = buckling(&problem, &result, 20);
    assert!(!folded.solved);
    assert!(!folded.advice.is_empty(), "it has to say why");
}

#[test]
fn the_buckling_mode_found_is_the_easiest_one() {
    let (nx, ny, nz) = (3, 3, 40);
    let mut problem = slender_column(0.200, nx, nz, Vec3::new(0.0, 0.0, -800.0));
    problem.max_iterations = 4000;
    let result = solve_voxels(&problem);
    assert!(result.solved);
    let folded = buckling(&problem, &result, 40);
    assert!(folded.solved, "{}", folded.advice);
    // A shape bending the other way and higher harmonics are valid deformations, and none may
    // fold sooner than the one found.
    let (nnx, nny, nnz) = (nx + 1, ny + 1, nz + 1);
    for harmonic in 1..=3 {
        let mut mode = vec![0.0; nnx * nny * nnz * 3];
        for k in 0..nnz {
            for j in 0..nny {
                for i in 0..nnx {
                    let node = (k * nny + j) * nnx + i;
                    mode[3 * node + 1] = (harmonic as f64 * std::f64::consts::PI * k as f64 / nz as f64).sin();
                }
            }
        }
        let factor = buckling_rayleigh(&problem, &result, &mode);
        if factor <= 0.0 {
            continue;
        }
        assert!(factor >= folded.load_factor * 0.98, "shape {harmonic} folds at {factor}, easier than {}", folded.load_factor);
    }
}

#[test]
fn the_second_order_magnifier_grows_as_the_load_approaches_buckling() {
    // A slender square strut, 20 × 20 mm over 400 mm, pushed along its axis.
    let (nx, ny, nz) = (40, 2, 2);
    let solve_at = |force: f64| {
        let mut problem = VoxelFemProblem::new(nx, ny, nz, 0.010, OrthotropicMaterial::isotropic(200e9, 0.3));
        problem.max_iterations = 4000;
        problem.tolerance = 1e-9;
        let mut case = LoadCase { force: Vec3::new(-force, 0.0, 0.0), ..Default::default() };
        for k in 0..nz {
            for j in 0..ny {
                let root = problem.index(0, j, k);
                problem.held[root] = true;
                case.voxels.push(problem.index(nx - 1, j, k));
            }
        }
        problem.cases.push(case);
        let solved = solve_voxels(&problem);
        (problem, solved)
    };
    let (light, light_result) = solve_at(20_000.0);
    assert!(light_result.solved, "{}", light_result.advice);
    let at_light = buckling(&light, &light_result, 12);
    assert!(at_light.solved, "{}", at_light.advice);
    assert!(at_light.load_factor > 1.0, "this load must not already buckle it");
    assert!((at_light.amplification - 1.0 / (1.0 - 1.0 / at_light.load_factor)).abs() < 1e-9);
    assert!(at_light.amplification > 1.0);
    assert!((at_light.assumed_imperfection - 0.400 / 500.0).abs() < 1e-9, "400 mm of strut, straight to L/500");
    assert!(at_light.second_order_stress > 0.0);

    // At 80% of the critical load the factor is 1.25 and the magnifier is 5.
    let critical = 20_000.0 * at_light.load_factor;
    let (heavy, heavy_result) = solve_at(0.8 * critical);
    assert!(heavy_result.solved);
    let at_heavy = buckling(&heavy, &heavy_result, 12);
    assert!(at_heavy.solved, "{}", at_heavy.advice);
    assert!((at_heavy.load_factor - 1.25).abs() < 0.05, "{}", at_heavy.load_factor);
    assert!((at_heavy.amplification - 5.0).abs() < 0.5, "{}", at_heavy.amplification);
    assert!(at_heavy.amplification > at_light.amplification);
    let load_ratio = 0.8 * critical / 20_000.0;
    assert!(at_heavy.second_order_stress > 2.0 * load_ratio * at_light.second_order_stress, "the bending grows far faster than the load");
}

#[test]
fn a_beam_bent_through_a_large_angle_keeps_its_length() {
    // Rotation costs a co-rotational element nothing: bend a cantilever until its tip has swung
    // through a third of a radian and its centreline is still as long as it started.
    let length = 0.400;
    let (across, along) = (2, 40);
    let problem = slender_column(length, across, along, Vec3::new(12_000.0, 0.0, 0.0));
    let options = LargeDisplacementOptions { load_steps: 8, ..Default::default() };
    let result = large_displacement(&problem, &options);
    assert!(result.solved, "{}", result.advice);
    assert!(result.carried_the_whole_load, "{}", result.advice);
    assert!(result.path.last().is_some_and(|s| s.rotation_degrees > 15.0), "a large rotation, not a small one dressed up");
    let (nnx, nny) = (across + 1, across + 1);
    let node = |k: usize| (k * nny + across / 2) * nnx + across / 2;
    let u = &result.displacement;
    let mut arc = 0.0;
    for k in 0..along {
        let (a, b) = (node(k), node(k + 1));
        let dx = u[3 * b] - u[3 * a];
        let dy = u[3 * b + 1] - u[3 * a + 1];
        let dz = problem.h + (u[3 * b + 2] - u[3 * a + 2]);
        arc += (dx * dx + dy * dy + dz * dz).sqrt();
    }
    assert!((arc - length).abs() < length * 0.01, "the beam changed length while bending: {arc}");
    let tip = node(along);
    assert!(u[3 * tip].abs() > 0.05 * length);
    assert!(u[3 * tip + 2] < -1e-4, "the tip pulls back towards the root");
}

#[test]
fn at_small_loads_the_nonlinear_solve_agrees_with_the_linear_one() {
    let problem = slender_column(0.200, 3, 24, Vec3::new(60.0, 0.0, 0.0));
    let linear = solve_voxels(&problem);
    assert!(linear.solved, "{}", linear.advice);
    let options = LargeDisplacementOptions { load_steps: 2, ..Default::default() };
    let large = large_displacement(&problem, &options);
    assert!(large.solved, "{}", large.advice);
    assert!(large.carried_the_whole_load);
    let last = large.path.last().cloned().unwrap_or_default();
    assert!(
        (last.displacement - linear.worst_displacement).abs() < linear.worst_displacement * 0.02,
        "{} vs {}",
        last.displacement,
        linear.worst_displacement
    );
    assert!(last.rotation_degrees < 2.0);
}

#[test]
fn a_bowed_column_deflects_by_the_amplification_theory_asks() {
    // A real column is never straight: bowed and squashed, its sideways deflection grows as
    // 1 / (1 − P/Pcr), which the linear solver cannot produce at all.
    let length = 0.300;
    let problem = slender_column(length, 3, 40, Vec3::new(0.0, 0.0, -1000.0));
    let linear = solve_voxels(&problem);
    assert!(linear.solved, "{}", linear.advice);
    let folded = buckling(&problem, &linear, 30);
    assert!(folded.solved, "{}", folded.advice);
    let critical = folded.load_factor * 1000.0;
    let bow = length / 500.0;
    let crooked = bowed_column(&problem, bow);
    let lateral_at = |share: f64| {
        let mut loaded = problem.clone();
        loaded.cases[0].force = Vec3::new(0.0, 0.0, -share * critical);
        let options = LargeDisplacementOptions { load_steps: 6, imperfection: crooked.clone(), ..Default::default() };
        let result = large_displacement(&loaded, &options);
        assert!(result.carried_the_whole_load, "no equilibrium at {share} of critical: {}", result.advice);
        result.path.last().map(|s| s.lateral).unwrap_or(0.0)
    };
    let half = bow + lateral_at(0.5);
    let most = bow + lateral_at(0.8);
    assert!((half / bow - 2.0).abs() < 0.15, "amplification at half critical: {}", half / bow);
    assert!((most / bow - 5.0).abs() < 0.8, "amplification at 80% of critical: {}", most / bow);
    assert!(most - bow > (half - bow) * 3.0, "the growth accelerates towards the limit");
}

#[test]
fn past_the_limit_load_it_says_there_is_no_equilibrium_rather_than_a_huge_number() {
    let length = 0.300;
    let mut problem = slender_column(length, 3, 40, Vec3::new(0.0, 0.0, -1000.0));
    let linear = solve_voxels(&problem);
    assert!(linear.solved, "{}", linear.advice);
    let folded = buckling(&problem, &linear, 30);
    assert!(folded.solved, "{}", folded.advice);
    let critical = folded.load_factor * 1000.0;
    let crooked = bowed_column(&problem, length / 500.0);
    problem.cases[0].force = Vec3::new(0.0, 0.0, -1.6 * critical);
    let options = LargeDisplacementOptions { load_steps: 16, imperfection: crooked, ..Default::default() };
    let result = large_displacement(&problem, &options);
    assert!(result.solved, "{}", result.advice);
    assert!(!result.carried_the_whole_load);
    let carried = result.limit_load_fraction * 1.6 * critical;
    assert!(carried > 0.5 * critical && carried < 1.35 * critical, "gave up at {carried} N against {critical} N");
    assert!(!result.advice.is_empty());
    assert!(result.path.last().is_some_and(|s| s.displacement < length), "a real deflection, not a runaway");
}

#[test]
fn the_collapse_load_of_a_real_column_is_below_its_eigenvalue() {
    let problem = slender_column(0.300, 3, 40, Vec3::new(0.0, 0.0, -1000.0));
    let linear = solve_voxels(&problem);
    assert!(linear.solved, "{}", linear.advice);
    let folded = buckling(&problem, &linear, 30);
    assert!(folded.solved, "{}", folded.advice);
    let collapse = limit_load(&problem, &folded, 1.0 / 500.0, 1.2, 12);
    assert!(collapse.solved, "{}", collapse.advice);
    assert!(!collapse.carried_the_whole_load, "pushed past the eigenvalue and it held: {}", collapse.advice);
    assert!(collapse.limit_load_fraction < folded.load_factor, "a crooked column cannot hold what a perfect one holds");
    assert!(collapse.limit_load_fraction > 0.5 * folded.load_factor, "an L/500 bow should not halve the strength");
    let (first, last) = (collapse.path.first().cloned().unwrap_or_default(), collapse.path.last().cloned().unwrap_or_default());
    assert!(last.lateral > first.lateral && last.load_fraction > first.load_fraction);
    let nothing = limit_load(&problem, &BucklingResult::default(), 1.0 / 500.0, 1.2, 12);
    assert!(!nothing.solved);
    assert!(nothing.advice.contains("no buckling mode"));
}
