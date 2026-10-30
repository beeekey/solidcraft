//! The solve stated on a body: voxelise it, hold it at one place, push on it at another.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Mesh, Vec3};

use crate::buckling::{BucklingResult, buckling};
use crate::large_displacement::limit_load;
use crate::material::OrthotropicMaterial;
use crate::voxel_fem::{ContactReport, LoadCase, VoxelFemProblem, node_map, solve_voxels};
use crate::voxelise::{MM, Region, VoxelGrid, voxelise_many};

/// The coarsest grid worth solving.
pub const MIN_RESOLUTION: usize = 6;
/// The finest grid a single solve may ask for.
pub const MAX_RESOLUTION: usize = 60;
/// The most cells a grid may hold, whatever the resolution.
pub const MAX_CELLS: usize = 400_000;

/// What a stress analysis needs beyond "hold it here, push it there".
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MeshFemOptions {
    /// Gravity as an acceleration (m/s²); zero switches it off.
    pub gravity: Vec3,
    /// The mount only pushes back: a foot on a table rather than a bolt through a flange.
    pub resting_support: bool,
    /// Which way "off the support" is.
    pub lift_direction: Option<Vec3>,
    /// Also solve for the load factor at which it buckles.
    pub solve_buckling: bool,
    /// And follow it past that: bow the member into its buckling mode by the straightness its
    /// section is sold to, and walk the load on until no equilibrium is left. Costs a nonlinear
    /// solve per load step; does nothing unless `solve_buckling` is on as well.
    pub follow_to_collapse: bool,
    /// With several bodies: the interfaces between them may open (a contact that pushes but does
    /// not pull) instead of being bonded as one. `None` bonds them.
    #[serde(default)]
    pub contact: Option<ContactOptions>,
}

/// How separate bodies touch.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ContactOptions {
    /// Friction coefficient at the interfaces; 0 lets the parts slide freely.
    pub friction: f64,
    /// A clamp holding the interfaces shut (N), spread over them: what a bolt does.
    pub preload_n: f64,
}

/// Several bodies solved together: their meshes (mm), each one's stiffness relative to the
/// material given (1 for the same material), and which body the held and the loaded regions
/// belong to.
#[derive(Debug, Clone)]
pub struct Assembly<'a> {
    pub meshes: Vec<&'a Mesh>,
    pub modulus_scale: Vec<f64>,
    pub fixed: (usize, Region),
    pub load: (usize, Region),
}

/// The answer for a body.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MeshFemResult {
    pub converged: bool,
    /// Final |r|² / |f|².
    pub residual_ratio: f64,
    pub grid: [usize; 3],
    pub voxels: usize,
    /// Cell size (m).
    pub h: f64,
    /// Corner of cell (0, 0, 0) (m).
    pub origin: Vec3,
    /// How many elements span the body's thinnest dimension. Six is where a bending answer
    /// settles; below four it should not be trusted.
    pub elements_through_thinnest: usize,
    /// Empty when the mesh is fine for the job; otherwise what is wrong and what would fix it.
    pub mesh_advice: String,
    /// m
    pub max_displacement: f64,
    pub max_displacement_at: Vec3,
    /// Pa, at element centres.
    pub max_von_mises: f64,
    pub max_von_mises_at: Vec3,
    /// Mean displacement of the loaded face along the force (m): what beam formulas predict.
    pub load_displacement: f64,
    /// Held nodes that let go, with a resting support.
    pub lifted_nodes: usize,
    /// Dense per-voxel von Mises stress (Pa).
    pub voxel_stress: Vec<f64>,
    /// The full stress tensor, six per voxel (xx, yy, zz, xy, yz, zx; Pa): what a direction-aware
    /// check (the layer bond of a printed part, the grain of a natural one) reads.
    #[serde(default)]
    pub voxel_stress_tensor: Vec<f64>,
    /// Dense per-voxel material mask.
    pub solid: Vec<bool>,
    /// Nodal displacement, three per grid node (m).
    pub node_displacement: Vec<f64>,
    /// The buckling load factor, when asked for; 0 when nothing is in compression.
    pub buckling_factor: f64,
    /// How much an imperfect member's bow grows at the applied load, and the bending stress (Pa)
    /// that comes with it.
    pub buckling_amplification: f64,
    pub second_order_stress: f64,
    pub buckling_advice: String,
    /// The buckling mode, three per grid node, normalised so the largest motion is one.
    pub buckling_mode: Vec<f64>,
    /// The load factor a member with that bow actually collapses at; always below the eigenvalue.
    /// 0 when it was not asked for.
    pub collapse_factor: f64,
    /// How far the collapsing member had moved sideways at that load (m).
    pub collapse_deflection: f64,
    pub collapse_advice: String,
    /// What the interfaces between the bodies did, when there were several in contact.
    #[serde(default)]
    pub contact: Option<ContactReport>,
    /// Which body each voxel belongs to (empty for one body).
    #[serde(default)]
    pub body_of: Vec<usize>,
}

/// Why a body could not be analysed.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum MeshFemError {
    #[error("need a closed body and a valid material")]
    Input,
    #[error("voxelisation found no interior: raise the resolution or check that the body is closed")]
    NoInterior,
    #[error("the grid would hold more than {MAX_CELLS} cells")]
    TooFine,
    #[error("the fixed or the load region hit no material")]
    RegionMissed,
    #[error("{0}")]
    Solve(String),
    #[error("the bodies came apart: every contact under one of them opened, so nothing holds it against the load")]
    CameApart,
}

/// Holds the body (a closed mesh in mm) at `fixed`, spreads `force` (N) over `load`, and solves.
/// `resolution` is the number of cells along the longest side, clamped to
/// `MIN_RESOLUTION..=MAX_RESOLUTION`.
pub fn solve_mesh(
    mesh: &Mesh,
    fixed: &Region,
    load: &Region,
    force: Vec3,
    material: &OrthotropicMaterial,
    density: f64,
    resolution: usize,
    options: &MeshFemOptions,
) -> Result<MeshFemResult, MeshFemError> {
    let assembly = Assembly { meshes: vec![mesh], modulus_scale: vec![1.0], fixed: (0, fixed.clone()), load: (0, load.clone()) };
    solve_assembly(&assembly, force, material, density, resolution, options)
}

/// [`solve_mesh`] for several bodies on one grid: bonded where they touch, or in contact when
/// `options.contact` says so. Every body is the given material, scaled by `modulus_scale`.
pub fn solve_assembly(
    assembly: &Assembly<'_>,
    force: Vec3,
    material: &OrthotropicMaterial,
    density: f64,
    resolution: usize,
    options: &MeshFemOptions,
) -> Result<MeshFemResult, MeshFemError> {
    material.validate().map_err(|_| MeshFemError::Input)?;
    let finite_region = |r: &Region| match r {
        Region::NearestWall(p) => p.is_finite(),
        Region::Face(t) => !t.is_empty(),
    };
    let (fixed_body, fixed) = (&assembly.fixed.0, &assembly.fixed.1);
    let (load_body, load) = (&assembly.load.0, &assembly.load.1);
    if assembly.meshes.is_empty()
        || assembly.meshes.len() > 64
        || assembly.modulus_scale.len() != assembly.meshes.len()
        || assembly.modulus_scale.iter().any(|x| !x.is_finite() || !(1e-6..=1e6).contains(x))
        || *fixed_body >= assembly.meshes.len()
        || *load_body >= assembly.meshes.len()
        || !finite_region(fixed)
        || !finite_region(load)
        || !force.is_finite()
        || !density.is_finite()
    {
        return Err(MeshFemError::Input);
    }
    let resolution = resolution.clamp(MIN_RESOLUTION, MAX_RESOLUTION);
    let (grid, body_of): (VoxelGrid, Vec<usize>) = voxelise_many(&assembly.meshes, resolution, MAX_CELLS).ok_or(MeshFemError::TooFine)?;
    let voxels = grid.filled();
    if voxels == 0 {
        return Err(MeshFemError::NoInterior);
    }
    let mut result = MeshFemResult {
        grid: [grid.nx, grid.ny, grid.nz],
        voxels,
        h: grid.h,
        origin: grid.origin,
        elements_through_thinnest: grid.nx.min(grid.ny).min(grid.nz),
        ..Default::default()
    };
    if result.elements_through_thinnest < 6 {
        let thinnest = grid.size.x.min(grid.size.y).min(grid.size.z);
        let longest = grid.size.x.max(grid.size.y).max(grid.size.z);
        let needed = (6.0 * longest / thinnest.max(f64::MIN_POSITIVE)).ceil() as usize;
        let severity = if result.elements_through_thinnest < 4 { "badly under-reported" } else { "under-reported" };
        result.mesh_advice = format!(
            "only {} element(s) through the thinnest wall ({:.1} mm): bending stress is {severity}, use resolution {needed} or more",
            result.elements_through_thinnest,
            thinnest * MM
        );
        if needed > MAX_RESOLUTION {
            result.mesh_advice.push_str(&format!(" -- but a single solve is capped at {MAX_RESOLUTION}, so a finer answer needs a simpler model"));
        }
    }

    let fixed_mesh = assembly.meshes.get(*fixed_body).ok_or(MeshFemError::Input)?;
    let load_mesh = assembly.meshes.get(*load_body).ok_or(MeshFemError::Input)?;
    let fixed: Vec<usize> = grid.region(fixed_mesh, fixed).into_iter().filter(|v| body_of.get(*v) == Some(fixed_body)).collect();
    let loaded: Vec<usize> = grid.region(load_mesh, load).into_iter().filter(|v| body_of.get(*v) == Some(load_body)).collect();
    if fixed.is_empty() || loaded.is_empty() {
        return Err(MeshFemError::RegionMissed);
    }

    let mut problem = VoxelFemProblem::new(grid.nx, grid.ny, grid.nz, grid.h, *material);
    problem.origin = grid.origin;
    problem.solid = grid.solid.clone();
    for &voxel in &fixed {
        problem.held[voxel] = true;
    }
    let several = assembly.meshes.len() > 1;
    if several && assembly.modulus_scale.iter().any(|x| (x - 1.0).abs() > 1e-12) {
        problem.element_scale = body_of.iter().map(|b| assembly.modulus_scale.get(*b).copied().unwrap_or(1.0)).collect();
    }
    if let (true, Some(contact)) = (several, options.contact) {
        problem.body_of = body_of.clone();
        problem.contact_friction = contact.friction.max(0.0);
        problem.contact_preload = contact.preload_n.max(0.0);
    }
    problem.gravity = options.gravity;
    problem.density = density;
    problem.no_tension_supports = options.resting_support;
    problem.support_normal = options.lift_direction.unwrap_or(Vec3::Z);
    problem.max_iterations = 1600;
    problem.tolerance = 1e-8;
    problem.cases.push(LoadCase { voxels: loaded.clone(), force, name: "applied".into() });

    let solved = solve_voxels(&problem);
    if solved.contact.came_apart {
        return Err(MeshFemError::CameApart);
    }
    let (Some(case), Some(u)) = (solved.cases.first(), solved.case_displacement.first()) else {
        return Err(MeshFemError::Solve(if solved.advice.is_empty() { "the solve failed".into() } else { solved.advice }));
    };
    if !problem.body_of.is_empty() {
        result.contact = Some(solved.contact.clone());
    }
    if several {
        result.body_of = body_of.clone();
    }
    result.converged = case.converged;
    result.residual_ratio = case.residual_ratio;
    result.lifted_nodes = case.lifted_nodes;
    if options.solve_buckling {
        let folded: BucklingResult = buckling(&problem, &solved, 12);
        result.buckling_factor = if folded.solved { folded.load_factor } else { 0.0 };
        result.buckling_amplification = folded.amplification;
        result.second_order_stress = folded.second_order_stress;
        result.buckling_advice = if folded.solved { String::new() } else { folded.advice.clone() };
        if folded.solved {
            let biggest = folded.mode.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            result.buckling_mode = folded.mode.iter().map(|v| if biggest > 0.0 { v / biggest } else { 0.0 }).collect();
            if options.follow_to_collapse {
                let collapse = limit_load(&problem, &folded, 1.0 / 500.0, 1.2, 12);
                if collapse.solved && !collapse.carried_the_whole_load {
                    result.collapse_factor = collapse.limit_load_fraction;
                    result.collapse_deflection = collapse.path.last().map(|s| s.lateral).unwrap_or(0.0);
                } else {
                    result.collapse_advice = if collapse.advice.is_empty() { "the collapse solve found no limit".into() } else { collapse.advice };
                }
            }
        }
    }

    // Nodes are read through the solver's map: with contact they are compacted per body.
    let nodes = node_map(&problem);
    let body_at = |voxel: usize| if problem.body_of.is_empty() { 0 } else { problem.body_of.get(voxel).copied().unwrap_or(0) };
    let displacement = |node: usize| {
        Vec3::new(u.get(3 * node).copied().unwrap_or(0.0), u.get(3 * node + 1).copied().unwrap_or(0.0), u.get(3 * node + 2).copied().unwrap_or(0.0))
    };
    let mut seen = vec![false; u.len() / 3];
    for (voxel, &solid) in grid.solid.iter().enumerate() {
        if !solid {
            continue;
        }
        let (i, j, k) = grid.ijk(voxel);
        for dz in 0..=1 {
            for dy in 0..=1 {
                for dx in 0..=1 {
                    let Some(node) = nodes.node(i + dx, j + dy, k + dz, body_at(voxel)) else { continue };
                    if seen.get(node).copied().unwrap_or(true) {
                        continue;
                    }
                    seen[node] = true;
                    let magnitude = displacement(node).len();
                    if magnitude > result.max_displacement {
                        result.max_displacement = magnitude;
                        result.max_displacement_at = grid.origin + Vec3::new((i + dx) as f64, (j + dy) as f64, (k + dz) as f64) * grid.h;
                    }
                }
            }
        }
    }
    let force_norm = force.len();
    if force_norm > 0.0 {
        let mut seen = vec![false; u.len() / 3];
        let (mut sum, mut count) = (0.0, 0usize);
        for &voxel in &loaded {
            let (i, j, k) = grid.ijk(voxel);
            for dz in 0..=1 {
                for dy in 0..=1 {
                    for dx in 0..=1 {
                        let Some(node) = nodes.node(i + dx, j + dy, k + dz, body_at(voxel)) else { continue };
                        if seen.get(node).copied().unwrap_or(true) {
                            continue;
                        }
                        seen[node] = true;
                        sum += displacement(node).dot(force) / force_norm;
                        count += 1;
                    }
                }
            }
        }
        result.load_displacement = if count > 0 { sum / count as f64 } else { 0.0 };
    }
    for (voxel, &stress) in solved.voxel_stress.iter().enumerate() {
        if stress > result.max_von_mises {
            result.max_von_mises = stress;
            let (i, j, k) = grid.ijk(voxel);
            result.max_von_mises_at = grid.origin + Vec3::new(i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5) * grid.h;
        }
    }
    result.voxel_stress = solved.voxel_stress;
    result.voxel_stress_tensor = solved.voxel_stress_tensor;
    result.solid = grid.solid;
    result.node_displacement = u.clone();
    Ok(result)
}
