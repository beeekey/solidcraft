//! There is no reference answer to compare a generated design against, so every check is on a
//! property that has to hold whatever the search invents: the material reaches from the mounts to
//! the loads, it stays inside the space it was given, the mass it reports is the mass of the
//! voxels it filled, a longer search is not worse than a short one, and the same seed gives the
//! same run.

use solidcraft_geom::{Aabb3, Vec3};

use crate::generative::{
    CheckOptions, Design, DesignGoal, DesignLoad, DesignMember, DesignPanel, DesignRegion, FieldOptions, GenerativeProblem, RegionShape,
    SurfaceFinish, build_design_grid, check_design, connected_to_mounts, cycles_to_failure, decode_field, describe_design, design_mesh, encode_field,
    estimate_peak_temperature, generative_search, mesh_from_mask, rasterise, refine_to_field, scan_design_makeability, size_factor, stream_drag,
    surface_factor,
};
use crate::makeability::MakeabilityRules;
use crate::{LoadCase, OrthotropicMaterial, VoxelFemProblem, solve_voxels};

const SPAN: f64 = 0.120;
const DEPTH: f64 = 0.040;
const HEIGHT: f64 = 0.060;
const PAD: f64 = 0.012;

fn boxed(min: [f64; 3], size: [f64; 3]) -> RegionShape {
    RegionShape::Box(Aabb3 { min: Vec3::new(min[0], min[1], min[2]), max: Vec3::new(min[0] + size[0], min[1] + size[1], min[2] + size[2]) })
}

/// The classic bracket: held on the wall at x = 0, pushed down 1 kN at the far end, free to use
/// a 120 × 40 × 60 mm box in between.
fn bracket_problem() -> GenerativeProblem {
    GenerativeProblem {
        design_space: Some(boxed([0.0; 3], [SPAN, DEPTH, HEIGHT])),
        mounts: vec![DesignRegion::new(boxed([0.0; 3], [PAD, DEPTH, HEIGHT]), "wall")],
        loads: vec![DesignLoad {
            region: DesignRegion::new(boxed([SPAN - PAD, 0.0, 0.0], [PAD, DEPTH, PAD]), "tip"),
            force: Vec3::new(0.0, 0.0, -1000.0),
            watts: 0.0,
            cycles: 0.0,
            stress_ratio: 0.0,
        }],
        youngs_modulus: 200e9,
        yield_strength: 250e6,
        density: 7850.0,
        target_safety_factor: 2.0,
        resolution: 16,
        population: 14,
        generations: 6,
        solutions: 3,
        seed: 7,
        free_nodes: 2,
        min_radius: 0.003,
        max_radius: 0.018,
        ..Default::default()
    }
}

#[test]
fn the_grid_finds_the_mounts_the_loads_and_the_space() {
    let problem = bracket_problem();
    let grid = build_design_grid(&problem, None);
    assert!(grid.nx > 0);
    assert!(!grid.assumed_space);
    assert_eq!(grid.mount_voxels.len(), 1);
    assert_eq!(grid.load_voxels.len(), 1);
    assert!(!grid.mount_voxels[0].is_empty() && !grid.load_voxels[0].is_empty());
    assert_eq!(grid.anchors.len(), 2);
    assert!(grid.anchors[0].x < PAD);
    assert!(grid.anchors[1].x > SPAN - 2.0 * PAD);
    assert!(grid.anchors[1].z < PAD * 2.0);
    for v in 0..grid.count() {
        if grid.space[v] {
            let c = grid.centre(v);
            assert!(c.x <= SPAN + 1e-9 && c.z <= HEIGHT + 1e-9);
        }
    }
}

#[test]
fn rasterising_a_strut_fills_a_tube_and_nothing_outside_the_space() {
    let problem = bracket_problem();
    let grid = build_design_grid(&problem, None);
    let design = Design { nodes: grid.anchors.clone(), members: vec![DesignMember { a: 0, b: 1, radius: 0.010 }], ..Default::default() };
    let solid = rasterise(&grid, &design);
    let filled = solid.iter().filter(|s| **s).count();
    assert!(filled > 0);
    let volume = filled as f64 * grid.h * grid.h * grid.h;
    assert!(volume < 0.6 * SPAN * DEPTH * HEIGHT);
    for (v, cell) in solid.iter().enumerate() {
        if *cell {
            assert!(grid.space[v], "material outside the design space");
        }
    }
    for &v in grid.mount_voxels[0].iter().chain(grid.load_voxels[0].iter()) {
        assert!(solid[v]);
    }
    assert!(connected_to_mounts(&grid, &solid));
}

#[test]
fn the_search_produces_distinct_designs_that_carry_the_load() {
    let problem = bracket_problem();
    let result = generative_search(&problem, None);
    assert!(result.solved, "{}", result.advice);
    assert!(!result.designs.is_empty() && result.designs.len() <= problem.solutions);
    assert!(result.fem_evaluations > 0);
    for design in &result.designs {
        assert!(design.connected, "{}", design.verdict);
        assert!(design.scored_by_fem, "an offered design must have been solved, not just estimated");
        assert!(design.mass > 0.0 && design.mass < 0.5 * SPAN * DEPTH * HEIGHT * problem.density);
        assert!(design.safety_factor > 0.0);
        assert!(!design.verdict.is_empty());
        assert!(!describe_design(design).is_empty());
    }
    for a in 0..result.designs.len() {
        for b in a + 1..result.designs.len() {
            let (da, db) = (&result.designs[a], &result.designs[b]);
            let same_mass = (da.mass - db.mass).abs() < 1e-6 * da.mass;
            let same_shape = da.members.len() == db.members.len() && da.nodes.iter().zip(&db.nodes).all(|(p, q)| (*p - *q).len() < 1e-9);
            assert!(!(same_mass && same_shape), "two offered designs are the same");
        }
    }
}

#[test]
fn the_same_seed_gives_the_same_run_and_another_seed_does_not() {
    let mut problem = bracket_problem();
    problem.generations = 4;
    let first = generative_search(&problem, None);
    let again = generative_search(&problem, None);
    assert!(first.solved && again.solved);
    assert_eq!(first.designs.len(), again.designs.len());
    assert_eq!(first.designs[0].mass, again.designs[0].mass);
    assert_eq!(first.designs[0].max_stress, again.designs[0].max_stress);
    problem.seed = 99;
    let other = generative_search(&problem, None);
    assert!(other.solved, "{}", other.advice);
    assert_ne!(first.designs[0].mass, other.designs[0].mass, "a search whose seed did nothing is not searching");
}

#[test]
fn searching_longer_does_not_give_a_worse_answer() {
    let mut short_run = bracket_problem();
    short_run.generations = 1;
    let mut long_run = bracket_problem();
    long_run.generations = 8;
    let quick = generative_search(&short_run, None);
    let thorough = generative_search(&long_run, None);
    assert!(quick.solved && thorough.solved);
    assert!(thorough.designs[0].fitness <= quick.designs[0].fitness * 1.001, "eight generations should not be worse than one");
    assert_eq!(thorough.mass_history.len(), 8);
}

#[test]
fn the_chosen_design_builds_as_one_closed_mesh() {
    let problem = bracket_problem();
    let result = generative_search(&problem, None);
    assert!(result.solved, "{}", result.advice);
    let mesh = design_mesh(&result.grid, &result.designs[0], 4).unwrap_or_default();
    assert!(!mesh.triangles.is_empty());
    assert_eq!(mesh.tri_face.len(), mesh.triangles.len());
    // Closed: every edge is shared by exactly two triangles.
    let mut edges = std::collections::HashMap::new();
    for t in &mesh.triangles {
        for n in 0..3 {
            let (a, b) = (t[n], t[(n + 1) % 3]);
            *edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    assert!(edges.values().all(|c| *c == 2), "the skin is not closed");
    // The mass the search reported came from the same voxels, so the mesh volume is the same
    // order.
    let volume = mesh.mass_properties().map(|m| m.volume.abs()).unwrap_or(0.0) * 1e-9;
    let built = volume * problem.density;
    assert!(built > 0.5 * result.designs[0].mass && built < 1.5 * result.designs[0].mass, "{built} vs {}", result.designs[0].mass);
    let bounds = mesh.bounds();
    assert!(bounds.min.x < 0.02 * SPAN * 1000.0 + 1e-6);
    assert!(bounds.max.x > 0.95 * SPAN * 1000.0, "it reaches from the wall to the tip");
}

#[test]
fn two_load_cases_are_both_survived_rather_than_averaged() {
    let mut problem = bracket_problem();
    problem.loads.push(DesignLoad {
        region: DesignRegion::new(boxed([SPAN - PAD, 0.0, HEIGHT - PAD], [PAD, DEPTH, PAD]), "cornering"),
        force: Vec3::new(0.0, 800.0, 0.0),
        watts: 0.0,
        cycles: 0.0,
        stress_ratio: 0.0,
    });
    problem.generations = 5;
    let result = generative_search(&problem, None);
    assert!(result.solved, "{}", result.advice);
    assert!(result.designs[0].connected);
    let mut down_only = bracket_problem();
    down_only.generations = 5;
    let single = generative_search(&down_only, None);
    assert!(single.solved);
    assert!(result.designs[0].mass >= 0.5 * single.designs[0].mass, "carrying a second load case cannot be free");
}

#[test]
fn a_setup_that_cannot_be_searched_says_why_in_words() {
    let mut no_mount = bracket_problem();
    no_mount.mounts.clear();
    let r = generative_search(&no_mount, None);
    assert!(!r.solved && r.advice.contains("holds"));
    let mut no_load = bracket_problem();
    no_load.loads.clear();
    assert!(generative_search(&no_load, None).advice.contains("pushes"));
    let mut detached = bracket_problem();
    detached.mounts[0].shape = boxed([0.5, 0.5, 0.5], [PAD, DEPTH, PAD]);
    let r = generative_search(&detached, None);
    assert!(!r.solved && r.advice.contains("wall"), "{}", r.advice);
    let mut bad_material = bracket_problem();
    bad_material.youngs_modulus = f64::NAN;
    assert!(!generative_search(&bad_material, None).solved);
}

#[test]
fn without_a_design_space_the_search_says_it_guessed_one() {
    let mut problem = bracket_problem();
    problem.design_space = None;
    problem.generations = 3;
    let result = generative_search(&problem, None);
    assert!(result.solved, "{}", result.advice);
    assert!(result.grid.assumed_space);
    assert!(result.advice.contains("no design space"), "{}", result.advice);
}

#[test]
fn a_heavier_load_costs_more_material() {
    let mut light = bracket_problem();
    light.generations = 8;
    let mut heavy = light.clone();
    heavy.loads[0].force = Vec3::new(0.0, 0.0, -4000.0);
    let light_result = generative_search(&light, None);
    let heavy_result = generative_search(&heavy, None);
    assert!(light_result.solved && heavy_result.solved);
    assert!(
        heavy_result.designs[0].mass > light_result.designs[0].mass,
        "four times the load, {} g against {} g",
        heavy_result.designs[0].mass * 1000.0,
        light_result.designs[0].mass * 1000.0
    );
    // And the stress rises with the load for a given shape.
    let grid = build_design_grid(&light, None);
    let plain = Design { nodes: grid.anchors.clone(), members: vec![DesignMember { a: 0, b: 1, radius: 0.008 }], ..Default::default() };
    let solid = rasterise(&grid, &plain);
    let mut fem = VoxelFemProblem::new(grid.nx, grid.ny, grid.nz, grid.h, OrthotropicMaterial::isotropic(200e9, 0.3));
    fem.solid = solid;
    for &v in &grid.mount_voxels[0] {
        fem.held[v] = true;
    }
    fem.cases = vec![
        LoadCase { voxels: grid.load_voxels[0].clone(), force: Vec3::new(0.0, 0.0, -1000.0), name: String::new() },
        LoadCase { voxels: grid.load_voxels[0].clone(), force: Vec3::new(0.0, 0.0, -4000.0), name: String::new() },
    ];
    let result = solve_voxels(&fem);
    assert!(result.solved, "{}", result.advice);
    assert!((result.cases[1].max_von_mises - 4.0 * result.cases[0].max_von_mises).abs() < 0.01 * result.cases[1].max_von_mises);
}

#[test]
fn an_area_thinner_than_a_voxel_is_snapped_rather_than_dropped() {
    let mut problem = bracket_problem();
    problem.resolution = 10;
    problem.mounts[0].shape = boxed([0.0; 3], [0.002, DEPTH, HEIGHT]);
    problem.generations = 3;
    let grid = build_design_grid(&problem, None);
    assert!(!grid.mount_voxels[0].is_empty(), "a thin pad must still be found");
    assert!(grid.snapped_regions);
    let result = generative_search(&problem, None);
    assert!(result.solved, "{}", result.advice);
    assert!(result.advice.contains("thinner than one voxel"), "{}", result.advice);
}

/// A 25 × 25 × 16 mm cold plate over a 10 × 10 mm chip, in a slow liquid stream.
fn cold_plate_problem() -> GenerativeProblem {
    let mut problem = GenerativeProblem {
        design_space: Some(boxed([0.0; 3], [0.025, 0.025, 0.016])),
        mounts: vec![DesignRegion::new(boxed([0.0; 3], [0.025, 0.025, 0.002]), "base")],
        loads: vec![DesignLoad {
            region: DesignRegion::new(boxed([0.0075, 0.0075, 0.0], [0.010, 0.010, 0.002]), "chip"),
            force: Vec3::ZERO,
            watts: 25.0,
            cycles: 0.0,
            stress_ratio: 0.0,
        }],
        goal: DesignGoal::RemoveHeat,
        youngs_modulus: 69e9,
        poisson_ratio: 0.33,
        yield_strength: 240e6,
        density: 2700.0,
        solid_conductivity: 205.0,
        temperature_limit: 273.15 + 85.0,
        resolution: 20,
        population: 16,
        generations: 6,
        solutions: 2,
        seed: 3,
        free_nodes: 3,
        min_radius: 0.0015,
        max_radius: 0.008,
        ..Default::default()
    };
    problem.stream.axis = 0;
    problem.stream.velocity = 0.005;
    problem.stream.density = 998.0;
    problem.stream.viscosity = 1.002e-3;
    problem.stream.conductivity = 0.598;
    problem.stream.heat_capacity = 4182.0;
    problem.stream.solver_checks = 2;
    problem.stream.check_resolution = 26;
    problem
}

#[test]
fn more_metal_in_the_stream_runs_cooler() {
    let problem = cold_plate_problem();
    let grid = build_design_grid(&problem, None);
    assert!(grid.nx > 0);
    let bare = Design { nodes: grid.anchors.clone(), members: vec![DesignMember { a: 0, b: 1, radius: 0.0015 }], ..Default::default() };
    let mut finned = Design { nodes: grid.anchors.clone(), ..Default::default() };
    for n in 0..3 {
        finned.nodes.push(Vec3::new(0.005 + 0.007 * n as f64, 0.012, 0.012));
    }
    finned.members.push(DesignMember { a: 0, b: 1, radius: 0.002 });
    for n in 0..3 {
        finned.members.push(DesignMember { a: 0, b: 2 + n, radius: 0.002 });
        finned.panels.push(DesignPanel { a: 0, b: 1, c: 2 + n, thickness: 0.0015 });
    }
    let bare_t = estimate_peak_temperature(&problem, &grid, &bare);
    let finned_t = estimate_peak_temperature(&problem, &grid, &finned);
    assert!(bare_t > 0.0);
    assert!(finned_t < bare_t, "fins reaching into the stream must cool it: {} C against {} C", finned_t - 273.15, bare_t - 273.15);
    assert!(finned_t > problem.stream.inlet_temperature);
}

#[test]
fn a_hotter_chip_buys_a_bigger_sink_and_the_flow_solver_has_the_last_word() {
    let mut light = cold_plate_problem();
    light.loads[0].watts = 5.0;
    light.generations = 4;
    light.stream.solver_checks = 1;
    let mut heavy = cold_plate_problem();
    heavy.loads[0].watts = 60.0;
    heavy.generations = 4;
    heavy.stream.solver_checks = 1;
    let small = generative_search(&light, None);
    let large = generative_search(&heavy, None);
    assert!(small.solved, "{}", small.advice);
    assert!(large.solved, "{}", large.advice);
    assert!(
        large.designs[0].mass > small.designs[0].mass,
        "60 W got {} g and 5 W got {} g",
        large.designs[0].mass * 1000.0,
        small.designs[0].mass * 1000.0
    );
    for design in [&small.designs[0], &large.designs[0]] {
        assert!(design.peak_temperature > 273.15);
        if design.checked_by_flow_solver {
            assert!(design.heat_removed > 0.0);
        } else {
            assert!(design.verdict.contains("flow solver"), "{}", design.verdict);
        }
    }
    assert!(small.designs[0].checked_by_flow_solver, "the light case is well inside what the solver can do: {}", small.designs[0].verdict);
    assert!(large.stream_checks > 0);
    // The reported temperature is the solved one, not the estimate.
    let best = &small.designs[0];
    let grid = build_design_grid(&light, None);
    let estimate = estimate_peak_temperature(&light, &grid, best);
    assert!(estimate > 0.0);
    assert_ne!(best.peak_temperature, estimate);
    assert!((best.heat_removed - 5.0).abs() < 2.5, "the heat carried away has to be roughly the heat put in: {}", best.heat_removed);
}

#[test]
fn a_thermal_study_without_a_wattage_says_so() {
    let mut problem = cold_plate_problem();
    problem.loads[0].watts = 0.0;
    let result = generative_search(&problem, None);
    assert!(!result.solved && result.advice.contains("watts"), "{}", result.advice);
}

#[test]
fn minding_the_stream_gives_a_slimmer_part() {
    let mut strength_only = GenerativeProblem {
        design_space: Some(boxed([0.0; 3], [0.080, 0.040, 0.040])),
        mounts: vec![DesignRegion::new(boxed([0.0; 3], [0.006, 0.040, 0.040]), "root")],
        loads: vec![DesignLoad {
            region: DesignRegion::new(boxed([0.074, 0.0, 0.015], [0.006, 0.040, 0.010]), "tip"),
            force: Vec3::new(0.0, 0.0, -300.0),
            watts: 0.0,
            cycles: 0.0,
            stress_ratio: 0.0,
        }],
        youngs_modulus: 69e9,
        poisson_ratio: 0.33,
        yield_strength: 240e6,
        density: 2700.0,
        target_safety_factor: 2.0,
        resolution: 18,
        population: 16,
        generations: 6,
        solutions: 2,
        seed: 5,
        free_nodes: 2,
        min_radius: 0.002,
        max_radius: 0.012,
        ..Default::default()
    };
    strength_only.stream.axis = 1;
    strength_only.stream.velocity = 0.05;
    strength_only.stream.solver_checks = 2;
    let mut mind = strength_only.clone();
    mind.goal = DesignGoal::LowDrag;
    let blunt = generative_search(&strength_only, None);
    let slim = generative_search(&mind, None);
    assert!(blunt.solved, "{}", blunt.advice);
    assert!(slim.solved, "{}", slim.advice);
    assert!(slim.designs[0].safety_factor > 0.0);
    assert!(slim.designs[0].checked_by_flow_solver, "{}", slim.designs[0].verdict);
    assert!(slim.designs[0].drag > 0.0);
    let grid = build_design_grid(&mind, None);
    let frontal = |design: &Design| {
        let solid = rasterise(&grid, design);
        let mut area = 0.0;
        for k in 0..grid.nz {
            for i in 0..grid.nx {
                if (0..grid.ny).any(|j| solid[grid.at(i, j, k)]) {
                    area += grid.h * grid.h;
                }
            }
        }
        area
    };
    // The frontal area is the search's own cheap proxy; the flow solver has the last word, so the
    // comparison is made on its drag for both designs.
    let blunt_drag = stream_drag(&mind, &grid, &blunt.designs[0]).unwrap_or(0.0);
    assert!(blunt_drag > 0.0);
    assert!(
        slim.designs[0].drag <= blunt_drag * 1.05,
        "drag-aware {} N over {} cm2 against {} N over {} cm2",
        slim.designs[0].drag,
        frontal(&slim.designs[0]) * 1e4,
        blunt_drag,
        frontal(&blunt.designs[0]) * 1e4
    );
}

#[test]
fn refining_a_skeleton_finds_a_lighter_body_than_the_skeleton() {
    let mut problem = bracket_problem();
    problem.generations = 6;
    let search = generative_search(&problem, None);
    assert!(search.solved, "{}", search.advice);
    let skeleton = &search.designs[0];
    let options = FieldOptions { resolution: 24, volume_fraction: 0.15, iterations: 14, meet_target_tries: 2, ..Default::default() };
    let refined = refine_to_field(&problem, skeleton, &options);
    assert!(refined.solved, "{}", refined.advice);
    assert!(refined.safety_factor >= problem.target_safety_factor * 0.9, "{}", refined.advice);
    assert!(refined.mass < skeleton.mass * 1.05, "field {} g against skeleton {} g", refined.mass * 1000.0, skeleton.mass * 1000.0);
    assert!(refined.volume_fraction > 0.0 && refined.volume_fraction < 0.9);
    assert!(refined.iterations > 0);
    let grid = build_design_grid(&problem, Some(options.resolution));
    assert_eq!(refined.mask.len(), grid.space.len());
    for v in 0..refined.mask.len() {
        if refined.mask[v] {
            assert!(grid.space[v]);
        }
    }
    assert!(connected_to_mounts(&grid, &refined.mask), "a refined body has to be one piece joined to its mounts");
}

#[test]
fn a_refined_field_builds_as_one_closed_mesh_of_the_right_size() {
    let mut problem = bracket_problem();
    problem.generations = 4;
    let search = generative_search(&problem, None);
    assert!(search.solved);
    let options = FieldOptions { resolution: 28, volume_fraction: 0.25, iterations: 15, meet_target: false, ..Default::default() };
    let refined = refine_to_field(&problem, &search.designs[0], &options);
    assert!(refined.solved, "{}", refined.advice);
    let mesh = mesh_from_mask(&refined.mask, refined.nx, refined.ny, refined.nz, refined.h, refined.origin, 4).unwrap_or_default();
    assert!(!mesh.triangles.is_empty());
    let filled = refined.mask.iter().filter(|m| **m).count();
    let voxel_volume = filled as f64 * refined.h * refined.h * refined.h;
    let volume = mesh.mass_properties().map(|m| m.volume.abs()).unwrap_or(0.0) * 1e-9;
    assert!((volume - voxel_volume).abs() < 0.25 * voxel_volume, "{} cm3 built from {} cm3 of voxels", volume * 1e6, voxel_volume * 1e6);
    // And it survives the round trip through its text form.
    let stored = decode_field(&encode_field(&refined));
    assert_eq!(stored.nx, refined.nx);
    assert_eq!(stored.mask, refined.mask);
    assert!((stored.h - refined.h).abs() < 1e-12);
    assert!(decode_field("garbage").mask.is_empty());
    assert!(decode_field("f1 3 3 3 0.01 0 0 0 1:5").mask.is_empty(), "a short run is refused");
}

#[test]
fn the_verification_runs_a_finer_mesh_than_the_search_did() {
    let mut problem = bracket_problem();
    problem.generations = 5;
    let search = generative_search(&problem, None);
    assert!(search.solved);
    let best = &search.designs[0];
    let body = design_mesh(&search.grid, best, 4).unwrap_or_default();
    let options = CheckOptions { resolution: 40, ..Default::default() };
    let check = check_design(&problem, &body, &options);
    assert!(check.solved, "{}", check.advice);
    assert_eq!(check.resolution, 40);
    assert!(check.safety_factor > 0.0 && check.mass > 0.0 && check.elements_through_thinnest > 0);
    assert!(!check.verdict.is_empty());
    assert!(
        check.safety_factor < best.safety_factor * 10.0 && check.safety_factor > best.safety_factor * 0.1,
        "{} vs {} (check mass {} g, design mass {} g, thinnest {} elements, stress {} vs {} MPa, displacement {} vs {} mm)",
        check.safety_factor,
        best.safety_factor,
        check.mass * 1e3,
        best.mass * 1e3,
        check.elements_through_thinnest,
        check.max_stress / 1e6,
        best.max_stress / 1e6,
        check.max_displacement * 1e3,
        best.max_displacement * 1e3
    );
    assert!(check.surface_area > 0.0 && check.overhang_area >= 0.0 && check.thinnest_feature > 0.0);
    assert!(check.manufacturing_note.contains("thinnest section"), "{}", check.manufacturing_note);
}

#[test]
fn buckling_is_checked_on_whatever_was_built_and_tension_has_no_mode() {
    let mut problem = bracket_problem();
    problem.loads[0].force = Vec3::new(-4000.0, 0.0, 0.0);
    problem.gravity = Vec3::ZERO;
    problem.generations = 5;
    let search = generative_search(&problem, None);
    assert!(search.solved, "{}", search.advice);
    let body = design_mesh(&search.grid, &search.designs[0], 4).unwrap_or_default();
    let options = CheckOptions { resolution: 32, ..Default::default() };
    let check = check_design(&problem, &body, &options);
    assert!(check.solved, "{}", check.advice);
    assert!(check.buckling_assessed && check.buckling_from_eigen_solve, "{}", check.buckling_note);
    assert!(check.buckling_factor > 0.0);
    let mut harder = problem.clone();
    harder.loads[0].force = Vec3::new(-8000.0, 0.0, 0.0);
    let pushed = check_design(&harder, &body, &options);
    assert!(pushed.buckling_assessed, "{}", pushed.buckling_note);
    assert!(
        (pushed.buckling_factor - check.buckling_factor / 2.0).abs() < 0.1 * check.buckling_factor,
        "{} vs {}",
        pushed.buckling_factor,
        check.buckling_factor
    );
    let mut pulled = problem.clone();
    pulled.loads[0].force = Vec3::new(4000.0, 0.0, 0.0);
    let tension = check_design(&pulled, &body, &CheckOptions { resolution: 28, ..Default::default() });
    assert!(!tension.buckling_note.is_empty());
    if !tension.buckling_assessed {
        assert!(tension.buckling_note.contains("tension") || tension.buckling_note.contains("compression"), "{}", tension.buckling_note);
    }
}

#[test]
fn fatigue_gives_a_life_and_spends_it_by_miners_rule() {
    let mut problem = bracket_problem();
    problem.generations = 4;
    problem.gravity = Vec3::ZERO;
    let search = generative_search(&problem, None);
    assert!(search.solved);
    let body = design_mesh(&search.grid, &search.designs[0], 4).unwrap_or_default();
    let steady = CheckOptions { resolution: 30, ..Default::default() };
    let once = check_design(&problem, &body, &steady);
    assert!(!once.fatigue_assessed && once.fatigue_note.contains("not assessed"), "{}", once.fatigue_note);
    let mut cycled = problem.clone();
    cycled.loads[0].cycles = 1e6;
    cycled.loads[0].stress_ratio = 0.0;
    // The arithmetic is exercised where the S-N curve slopes: the load is scaled (elasticity is
    // linear) so the finer check's peak sits at 200 MPa, between the endurance limit and the
    // ultimate strength this material is assumed to have.
    let probe = check_design(&cycled, &body, &steady);
    assert!(probe.solved && probe.max_stress > 0.0, "{}", probe.advice);
    cycled.loads[0].force = cycled.loads[0].force * (200e6 / probe.max_stress);
    let life = check_design(&cycled, &body, &steady);
    assert!((life.max_stress - 200e6).abs() < 2e6, "{} MPa", life.max_stress / 1e6);
    assert!(life.fatigue_assessed, "{}", life.fatigue_note);
    assert!(
        life.cycles_to_failure > 0.0 && life.fatigue_damage > 0.0,
        "life {} damage {} at {} MPa (search said {} MPa), thinnest {}: {}",
        life.cycles_to_failure,
        life.fatigue_damage,
        life.max_stress / 1e6,
        search.designs[0].max_stress / 1e6,
        life.elements_through_thinnest,
        life.fatigue_note
    );
    assert!(life.fatigue_note.contains("Miner"), "{}", life.fatigue_note);
    let mut longer = cycled.clone();
    longer.loads[0].cycles = 1e7;
    let harder = check_design(&longer, &body, &steady);
    assert!((harder.fatigue_damage - life.fatigue_damage * 10.0).abs() < 0.01 * life.fatigue_damage * 10.0);
    let mut reversed = cycled.clone();
    reversed.loads[0].stress_ratio = -1.0;
    let flipped = check_design(&reversed, &body, &steady);
    assert!(flipped.fatigue_damage > life.fatigue_damage, "{}", flipped.fatigue_note);
    // The S-N arithmetic itself.
    assert!(cycles_to_failure(50e6, 0.0, 100e6, 400e6) > 1e20);
    assert!((cycles_to_failure(360e6, 0.0, 100e6, 400e6) - 1000.0).abs() < 1.0);
    assert!(cycles_to_failure(200e6, 0.0, 100e6, 400e6) < cycles_to_failure(150e6, 0.0, 100e6, 400e6));
    assert!(cycles_to_failure(150e6, 100e6, 100e6, 400e6) < cycles_to_failure(150e6, 0.0, 100e6, 400e6));
    // A rougher surface shortens the life, and the note says which.
    let machined = CheckOptions { resolution: 24, cyclic_loading: true, finish: SurfaceFinish::Machined, ..Default::default() };
    let forged = CheckOptions { finish: SurfaceFinish::AsForged, ..machined.clone() };
    let a = check_design(&cycled, &body, &machined);
    let b = check_design(&cycled, &body, &forged);
    assert!(a.fatigue_assessed && b.fatigue_assessed);
    assert!(b.cycles_to_failure <= a.cycles_to_failure, "as forged {} cycles, machined {}", b.cycles_to_failure, a.cycles_to_failure);
    assert!(a.fatigue_note.contains("Machined") && b.fatigue_note.contains("As forged") && a.fatigue_note.contains("polished endurance"));
}

#[test]
fn a_search_told_to_mind_buckling_produces_a_stockier_part() {
    let mut indifferent = bracket_problem();
    indifferent.loads[0].force = Vec3::new(-6000.0, 0.0, 0.0);
    indifferent.gravity = Vec3::ZERO;
    indifferent.generations = 6;
    let mut careful = indifferent.clone();
    careful.target_buckling_factor = 4.0;
    let loose = generative_search(&indifferent, None);
    let tight = generative_search(&careful, None);
    assert!(loose.solved, "{}", loose.advice);
    assert!(tight.solved, "{}", tight.advice);
    assert!(tight.designs[0].buckling_factor > 0.0);
    assert_eq!(loose.designs[0].buckling_factor, 0.0, "a search not asked about buckling should not spend time on it");
}

#[test]
fn the_search_does_not_settle_on_the_first_thing_that_works() {
    let mut problem = bracket_problem();
    problem.gravity = Vec3::ZERO;
    problem.generations = 20;
    let result = generative_search(&problem, None);
    assert!(result.solved, "{}", result.advice);
    let best = &result.designs[0];
    assert!(
        best.mass < 0.510,
        "the single-strut plateau was 523 g and this has to beat it: {} g with {} member(s)",
        best.mass * 1000.0,
        best.members.len()
    );
    assert!(best.safety_factor > 0.0);
    assert!(result.mass_history.len() > 1);
}

#[test]
fn a_search_given_a_life_to_last_builds_for_it() {
    let mut once = bracket_problem();
    once.gravity = Vec3::ZERO;
    once.generations = 6;
    let mut forever = once.clone();
    forever.loads[0].cycles = 1e7;
    forever.loads[0].stress_ratio = -1.0;
    forever.target_fatigue_damage = 0.5;
    let single = generative_search(&once, None);
    let cycled = generative_search(&forever, None);
    assert!(single.solved, "{}", single.advice);
    assert!(cycled.solved, "{}", cycled.advice);
    assert!(cycled.designs[0].fatigue_damage > 0.0, "a search given ten million reversals has to work out the damage");
    assert_eq!(single.designs[0].fatigue_damage, 0.0);
}

#[test]
fn a_search_told_it_must_be_mouldable_avoids_undercuts() {
    let mut free = bracket_problem();
    free.gravity = Vec3::ZERO;
    free.generations = 6;
    let mut moulded = free.clone();
    moulded.manufacturing = MakeabilityRules { mouldable: true, pull_axis: 2, weight: 2.0, ..Default::default() };
    let anything = generative_search(&free, None);
    let releasable = generative_search(&moulded, None);
    assert!(anything.solved, "{}", anything.advice);
    assert!(releasable.solved, "{}", releasable.advice);
    let grid = build_design_grid(&moulded, None);
    let trapped = |design: &Design| scan_design_makeability(&grid, &rasterise(&grid, design), &moulded.manufacturing).undercut_fraction;
    assert!(
        trapped(&releasable.designs[0]) <= trapped(&anything.designs[0]) + 1e-9,
        "asked for a mouldable part it left {}% trapped, against {}% when free",
        trapped(&releasable.designs[0]) * 100.0,
        trapped(&anything.designs[0]) * 100.0
    );
    assert!(releasable.designs[0].unmakeable_fraction >= 0.0);
}

#[test]
fn gravity_and_resting_supports_change_the_design() {
    let mut weightless = bracket_problem();
    weightless.gravity = Vec3::ZERO;
    weightless.loads[0].force = Vec3::new(0.0, 0.0, -200.0);
    weightless.generations = 5;
    let mut heavy = weightless.clone();
    heavy.gravity = Vec3::new(0.0, 0.0, -9.81);
    let ignoring = generative_search(&weightless, None);
    let carrying = generative_search(&heavy, None);
    assert!(ignoring.solved, "{}", ignoring.advice);
    assert!(carrying.solved, "{}", carrying.advice);
    assert!(carrying.designs[0].max_stress >= 0.0);
    let mut resting = weightless.clone();
    resting.mounts[0].bonded = false;
    resting.mounts[0].lift_direction = Vec3::X;
    let loose = generative_search(&resting, None);
    assert!(loose.solved, "{}", loose.advice);
    assert!(loose.designs[0].max_displacement > 0.0);
}

#[test]
fn a_free_form_body_is_generated_from_the_boundary_conditions_alone() {
    let mut problem = bracket_problem();
    problem.gravity = Vec3::ZERO;
    let options = FieldOptions { resolution: 22, volume_fraction: 0.2, iterations: 15, meet_target_tries: 2, ..Default::default() };
    let generated = refine_to_field(&problem, &Design::default(), &options);
    assert!(generated.solved, "{}", generated.advice);
    assert!(generated.mass > 0.0 && generated.volume_fraction < 0.9);
    assert!(generated.safety_factor >= problem.target_safety_factor * 0.9, "{}", generated.advice);
    let grid = build_design_grid(&problem, Some(options.resolution));
    assert_eq!(generated.mask.len(), grid.space.len());
    assert!(connected_to_mounts(&grid, &generated.mask));
    for v in 0..generated.mask.len() {
        if generated.mask[v] {
            assert!(grid.space[v]);
        }
    }
    let mesh = mesh_from_mask(&generated.mask, generated.nx, generated.ny, generated.nz, generated.h, generated.origin, 4).unwrap_or_default();
    assert!(!mesh.triangles.is_empty());
    let bounds = mesh.bounds();
    assert!(bounds.min.x < 0.02 * SPAN * 1000.0 + 1e-6);
    assert!(bounds.max.x > 0.9 * SPAN * 1000.0, "it has to reach the load");
    problem.generations = 5;
    let search = generative_search(&problem, None);
    assert!(search.solved);
    let seeded = refine_to_field(&problem, &search.designs[0], &options);
    assert!(seeded.solved, "{}", seeded.advice);
    assert!(
        seeded.mass.max(generated.mass) < 3.0 * seeded.mass.min(generated.mass),
        "seeded {} g, from nothing {} g",
        seeded.mass * 1000.0,
        generated.mass * 1000.0
    );
}

#[test]
fn the_surface_and_size_factors_match_the_published_tables() {
    assert!((surface_factor(SurfaceFinish::Machined, 600e6) - 0.83).abs() < 0.02);
    assert!((surface_factor(SurfaceFinish::Ground, 600e6) - 0.90).abs() < 0.02);
    assert!((surface_factor(SurfaceFinish::HotRolled, 600e6) - 0.58).abs() < 0.03);
    assert!((surface_factor(SurfaceFinish::AsForged, 600e6) - 0.44).abs() < 0.03);
    assert_eq!(surface_factor(SurfaceFinish::Polished, 600e6), 1.0);
    assert!(surface_factor(SurfaceFinish::Machined, 1200e6) < surface_factor(SurfaceFinish::Machined, 400e6));
    assert!((size_factor(0.050) - 0.82).abs() < 0.02);
    assert_eq!(size_factor(0.002), 1.0);
    assert!(size_factor(0.200) < size_factor(0.050) && size_factor(0.200) > 0.5);
    for name in SurfaceFinish::NAMES {
        assert_eq!(SurfaceFinish::from_name(name).name(), name);
    }
}
