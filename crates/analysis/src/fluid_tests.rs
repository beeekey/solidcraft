//! Flow and conjugate heat transfer, checked against solutions that are known exactly. The flow
//! cases use a rectangular duct whose walls lie on the grid, so any error is the solver's. The
//! heat cases lean on conservation: at steady state, with insulated outer walls, every watt
//! dissipated in the solid has to leave with the fluid.

use solidcraft_geom::{Aabb3, Mesh, Vec3};

use crate::fluid::{FLUIDS, FlowBody, FlowRegion, FluidError, FluidProblem, FluidResult, VoxelBody, fluid_named, fluid_solve};

const LENGTH: f64 = 0.060; // duct length, m
const WIDTH: f64 = 0.020;
const HEIGHT: f64 = 0.004;
const WALL: f64 = 0.004;

/// A closed box mesh in mm from `min`.
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

/// A block with a rectangular passage straight through it (mm): the outer skin and the passage
/// walls, which an inside test by ray parity reads as a tunnel.
fn duct() -> Mesh {
    let mm = 1000.0;
    let mut block = box_at(Vec3::ZERO, Vec3::new(LENGTH, WIDTH + 2.0 * WALL, HEIGHT + 2.0 * WALL) * mm);
    let passage = box_at(Vec3::new(0.0, WALL, WALL) * mm, Vec3::new(LENGTH, WIDTH, HEIGHT) * mm);
    block.append(&passage, 6);
    block
}

fn duct_problem(velocity: f64, resolution: usize) -> FluidProblem {
    FluidProblem {
        region: FlowRegion::Internal,
        inlet_point: Vec3::new(0.0, WALL + WIDTH / 2.0, WALL + HEIGHT / 2.0),
        outlet_point: Vec3::new(LENGTH, WALL + WIDTH / 2.0, WALL + HEIGHT / 2.0),
        inlet_velocity: velocity,
        resolution,
        max_steps: 40_000,
        ..Default::default()
    }
}

fn solve_duct(velocity: f64, resolution: usize) -> FluidResult {
    fluid_solve(FlowBody::Mesh(&duct()), &duct_problem(velocity, resolution)).unwrap_or_default()
}

/// Exact pressure drop for laminar flow in a rectangular duct: the parallel-plate formula with
/// the standard aspect-ratio correction.
fn exact_pressure_drop(viscosity: f64, mean_velocity: f64) -> f64 {
    let correction = 1.0 - 0.630 * HEIGHT / WIDTH;
    12.0 * viscosity * LENGTH * mean_velocity / (HEIGHT * HEIGHT * correction)
}

#[test]
fn the_duct_is_a_tunnel_the_inlet_reaches() {
    let result = solve_duct(0.005, 50);
    assert!(result.steps > 0, "{}", result.advice);
    let fluid = result.fluid.iter().filter(|f| **f).count();
    let solid = result.solid.iter().filter(|s| **s).count();
    assert!(fluid > 0 && solid > 0, "fluid {fluid}, solid {solid}");
    // The passage is about 20 × 4 of the 60 × 28 × 12 mm block: a sixth of the volume.
    let share = fluid as f64 / (fluid + solid) as f64;
    assert!((share - WIDTH * HEIGHT / ((WIDTH + 2.0 * WALL) * (HEIGHT + 2.0 * WALL))).abs() < 0.08, "{share}");
}

#[test]
fn the_pressure_drop_matches_the_exact_duct_solution() {
    let problem = duct_problem(0.005, 140);
    let result = fluid_solve(FlowBody::Mesh(&duct()), &problem).unwrap_or_default();
    assert!(result.converged, "residual {} ({})", result.residual, result.advice);
    assert!(result.cells_across_passage >= 8, "this check needs the passage resolved");
    let exact = exact_pressure_drop(problem.viscosity, result.mean_inlet_velocity);
    assert!(result.pressure_drop > 0.0, "flow must lose pressure, not gain it");
    // Within 20% at ten cells across: a stair-stepped wall and a developing entry both push the
    // measured drop above the fully developed value.
    assert!((result.pressure_drop - exact).abs() < exact * 0.20, "{} Pa vs exact {exact} Pa", result.pressure_drop);
    // No-slip at the walls makes the centre run faster than the mean: near 1.9 for a 5:1 duct.
    let ratio = result.max_velocity / result.mean_inlet_velocity;
    assert!(ratio > 1.6 && ratio < 2.3, "ratio {ratio}");
}

#[test]
fn the_flow_is_pushed_through_the_opening_not_the_wall() {
    let result = solve_duct(0.005, 100);
    assert!(result.converged, "{}", result.advice);
    for v in 0..result.solid.len() {
        if result.solid[v] {
            assert_eq!(result.velocity_x[v], 0.0, "the metal is moving");
        }
    }
    assert!((result.mean_inlet_velocity - 0.005).abs() < 0.005 * 0.05, "{}", result.mean_inlet_velocity);
    let area = WIDTH * HEIGHT;
    assert!((result.volume_flow_rate - 0.005 * area).abs() < 0.005 * area * 0.25, "{}", result.volume_flow_rate);
}

#[test]
fn every_watt_put_in_leaves_with_the_fluid() {
    let mut problem = duct_problem(0.005, 80);
    problem.solve_heat = true;
    problem.heat_power = 5.0;
    let result = fluid_solve(FlowBody::Mesh(&duct()), &problem).unwrap_or_default();
    assert!(result.converged, "{}", result.advice);
    assert!(result.heat_solved);
    assert!((result.heat_removed - 5.0).abs() < 0.25, "in 5 W, out {} W: {}", result.heat_removed, result.advice);
    let capacity_rate = problem.density * result.volume_flow_rate * problem.fluid_heat_capacity;
    assert!((result.outlet_temperature - result.inlet_temperature - 5.0 / capacity_rate).abs() < 0.25);
    assert!(result.max_temperature > result.outlet_temperature, "the metal is the hotter one");
}

#[test]
fn the_fluid_never_leaves_hotter_than_the_wall_that_heats_it() {
    let mut problem = duct_problem(0.002, 80);
    problem.solve_heat = true;
    problem.inlet_temperature = 293.15;
    problem.wall_temperature = 353.15;
    let result = fluid_solve(FlowBody::Mesh(&duct()), &problem).unwrap_or_default();
    assert!(result.heat_solved, "{}", result.advice);
    assert!(result.outlet_temperature > problem.inlet_temperature, "a heated wall must warm the flow");
    assert!(result.outlet_temperature <= problem.wall_temperature + 1e-6);
    assert!(result.max_temperature <= problem.wall_temperature + 1e-6);
}

#[test]
fn faster_flow_carries_more_heat_but_warms_up_less() {
    let heated = |velocity: f64| {
        let mut problem = duct_problem(velocity, 80);
        problem.solve_heat = true;
        problem.wall_temperature = 353.15;
        fluid_solve(FlowBody::Mesh(&duct()), &problem).unwrap_or_default()
    };
    let (slow, fast) = (heated(0.002), heated(0.010));
    assert!(fast.heat_removed > slow.heat_removed, "more coolant should carry more heat away");
    assert!(fast.outlet_temperature < slow.outlet_temperature, "but it should come out cooler");
}

#[test]
fn a_passage_too_coarse_to_resolve_says_so() {
    let result = solve_duct(0.005, 50);
    assert!(result.cells_across_passage < 8);
    assert!(result.advice.contains("cells across") && result.advice.contains("resolution"), "{}", result.advice);
}

#[test]
fn nonsense_is_refused_rather_than_solved() {
    let mut same = duct_problem(0.005, 40);
    same.outlet_point = same.inlet_point;
    assert_eq!(fluid_solve(FlowBody::Mesh(&duct()), &same).err(), Some(FluidError::SameFace));
    let mut thin = duct_problem(0.005, 40);
    thin.viscosity = 0.0;
    assert_eq!(fluid_solve(FlowBody::Mesh(&duct()), &thin).err(), Some(FluidError::BadFluid));
    assert_eq!(fluid_solve(FlowBody::Mesh(&duct()), &duct_problem(0.0, 40)).err(), Some(FluidError::BadFluid));
    assert_eq!(fluid_solve(FlowBody::Mesh(&Mesh::default()), &duct_problem(1.0, 40)).err(), Some(FluidError::NoBody));
    let mut empty_domain = FluidProblem { region: FlowRegion::External, domain: Some(Aabb3::EMPTY), ..duct_problem(1.0, 40) };
    empty_domain.inlet_point = Vec3::ZERO;
    empty_domain.outlet_point = Vec3::new(0.02, 0.0, 0.0);
    assert_eq!(fluid_solve(FlowBody::Mesh(&box_at(Vec3::ZERO, Vec3::new(20.0, 20.0, 20.0))), &empty_domain).err(), Some(FluidError::EmptyDomain));
    let bad_voxels = VoxelBody { nx: 3, ny: 3, nz: 3, h: 0.001, origin: Vec3::ZERO, solid: vec![false; 5] };
    assert_eq!(fluid_solve(FlowBody::Voxels(&bad_voxels), &duct_problem(1.0, 40)).err(), Some(FluidError::BadVoxelBody));
    let mut hostile = duct_problem(f64::NAN, 40);
    hostile.resolution = usize::MAX;
    assert!(fluid_solve(FlowBody::Mesh(&duct()), &hostile).is_err());
    assert_eq!(fluid_named("air").name, "Air");
    assert_eq!(fluid_named("nothing").name, FLUIDS[0].name);
}

#[test]
fn flow_around_a_body_accelerates_past_it_and_drags() {
    // A stubby wing in a tunnel. The domain is given explicitly: a body needs room behind it
    // for the wake and very little beside it. Small on purpose.
    let wing = box_at(Vec3::ZERO, Vec3::new(40.0, 60.0, 6.0));
    let body = Aabb3 { min: Vec3::ZERO, max: Vec3::new(0.040, 0.060, 0.006) };
    let domain = Aabb3 { min: body.min - Vec3::new(0.030, 0.008, 0.020), max: body.max + Vec3::new(0.060, 0.008, 0.020) };
    let problem = FluidProblem {
        region: FlowRegion::External,
        domain: Some(domain),
        inlet_point: Vec3::new(domain.min.x, 0.030, 0.003),
        outlet_point: Vec3::new(domain.max.x, 0.030, 0.003),
        inlet_velocity: 0.020,
        density: 1.204,
        viscosity: 1.813e-5,
        resolution: 80,
        max_steps: 30_000,
        ..Default::default()
    };
    let result = fluid_solve(FlowBody::Mesh(&wing), &problem).unwrap_or_default();
    assert!(result.converged, "{}", result.advice);
    let expected = problem.density * problem.inlet_velocity * 0.040 / problem.viscosity;
    assert!((result.reynolds - expected).abs() < expected * 0.15, "Re {} vs {expected}", result.reynolds);
    assert!(result.max_velocity > problem.inlet_velocity * 1.1, "the flow should accelerate around the body");
    assert!(result.max_velocity < problem.inlet_velocity * 5.0);
    assert!(result.drag_force.x.is_finite() && result.drag_force.x > 0.0, "{:?}", result.drag_force);
    assert!(
        result.drag_force.z.abs() < result.drag_force.x.abs() * 0.2,
        "a symmetric body at zero incidence should not lift: {:?}",
        result.drag_force
    );
}

#[test]
fn a_voxel_stated_body_gives_the_same_flow_as_the_solid() {
    let shaped = duct_problem(0.005, 100);
    let from_shape = fluid_solve(FlowBody::Mesh(&duct()), &shaped).unwrap_or_default();
    assert!(from_shape.converged, "{}", from_shape.advice);
    let gridded = VoxelBody {
        nx: from_shape.grid[0],
        ny: from_shape.grid[1],
        nz: from_shape.grid[2],
        h: from_shape.h,
        origin: from_shape.origin,
        solid: from_shape.solid.clone(),
    };
    let from_grid = fluid_solve(FlowBody::Voxels(&gridded), &shaped).unwrap_or_default();
    assert!(from_grid.converged, "{}", from_grid.advice);
    assert_eq!(from_grid.grid, from_shape.grid);
    assert!((from_grid.pressure_drop - from_shape.pressure_drop).abs() < 1e-6 * from_shape.pressure_drop.abs() + 1e-9);
    assert!((from_grid.max_velocity - from_shape.max_velocity).abs() < 1e-9);
    assert!((from_grid.reynolds - from_shape.reynolds).abs() < 1e-9);
}

#[test]
fn a_heat_sink_in_moving_air_is_solvable_at_all() {
    // Air fast enough to cool anything is past the Reynolds number a grid this size resolves;
    // with the unresolved motion carried as an eddy viscosity the solve stands up, faster air
    // runs cooler and costs more pressure, and every watt still comes out.
    let (sx, sy, sz) = (0.025, 0.025, 0.016);
    let resolution = 24;
    let h = sx / resolution as f64;
    let (nx, ny, nz) = ((sx / h).round() as usize, (sy / h).round() as usize, (sz / h).round() as usize);
    let mut solid = vec![false; nx * ny * nz];
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let (z, y) = ((k as f64 + 0.5) * h, (j as f64 + 0.5) * h);
                let base = z < 0.003;
                let fin = ((y / 0.005) as usize).is_multiple_of(2) && z < 0.012;
                if base || fin {
                    solid[(k * ny + j) * nx + i] = true;
                }
            }
        }
    }
    let sink = VoxelBody { nx, ny, nz, h, origin: Vec3::ZERO, solid };
    let mut answers = Vec::new();
    for speed in [0.15, 1.0, 3.0] {
        let problem = FluidProblem {
            region: FlowRegion::External,
            inlet_point: Vec3::new(0.0, sy / 2.0, sz / 2.0),
            outlet_point: Vec3::new(sx, sy / 2.0, sz / 2.0),
            inlet_velocity: speed,
            density: 1.204,
            viscosity: 1.813e-5,
            fluid_conductivity: 0.0257,
            fluid_heat_capacity: 1005.0,
            solid_conductivity: 205.0,
            solid_density: 2700.0,
            solid_heat_capacity: 900.0,
            solve_heat: true,
            heat_power: 5.0,
            inlet_temperature: 293.15,
            max_steps: 8000,
            tolerance: 1e-4,
            ..Default::default()
        };
        let result = fluid_solve(FlowBody::Voxels(&sink), &problem).unwrap_or_default();
        assert!(result.converged, "air at {speed} m/s (Re {}) has to be solvable: {}", result.reynolds, result.advice);
        assert!(result.heat_solved, "{}", result.advice);
        assert!((result.heat_removed - 5.0).abs() < 0.5, "{} W of 5 W: {}", result.heat_removed, result.advice);
        answers.push((result.reynolds, result.max_temperature - 273.15, result.pressure_drop));
    }
    assert!(answers[2].0 > 2000.0, "the fast case has to be past the laminar range");
    assert!(answers[1].1 < answers[0].1 && answers[2].1 < answers[1].1, "faster air runs cooler: {answers:?}");
    assert!(answers[1].2 > answers[0].2 && answers[2].2 > answers[1].2, "and costs more pressure: {answers:?}");
}
