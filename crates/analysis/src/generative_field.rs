//! Material as a field: the density method.
//!
//! A skeleton can only ever propose struts and plates. The second representation is material as
//! a density field over the design space, free to be anything between nothing and solid at every
//! point, refined by SIMP: solve, measure how much each element contributes, move material
//! towards where it earns its keep, repeat. The filter radius sets the minimum feature size, and
//! so is the manufacturing constraint. The search finds the layout, the field refines it into a
//! body; a field can also start from nothing but the boundary conditions.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Mesh, Vec3};

use super::{Design, GenerativeProblem, MAX_GRID_CELLS, build_design_grid, rasterise};
use crate::material::OrthotropicMaterial;
use crate::voxel_fem::{LoadCase, VoxelFemProblem, solve_voxels};
use crate::voxelise::MM;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FieldOptions {
    /// The field is refined on a finer grid than the search: this runs once.
    pub resolution: usize,
    /// How much of the design space may end up as material.
    pub volume_fraction: f64,
    /// Nothing thinner than this survives (m); 0 means about two voxels.
    pub minimum_feature: f64,
    pub iterations: usize,
    /// SIMP exponent.
    pub penalty: f64,
    pub smoothing_passes: usize,
    /// Keep tuning the volume until it reaches the target safety factor, or the space runs out.
    pub meet_target: bool,
    pub meet_target_tries: usize,
}

impl Default for FieldOptions {
    fn default() -> Self {
        FieldOptions {
            resolution: 40,
            volume_fraction: 0.3,
            minimum_feature: 0.0,
            iterations: 40,
            penalty: 3.0,
            smoothing_passes: 4,
            meet_target: true,
            meet_target_tries: 3,
        }
    }
}

/// The refined body: densities, the material they threshold to, and what the finite elements
/// said about it at that resolution.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FieldBody {
    pub solved: bool,
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    pub h: f64,
    pub origin: Vec3,
    /// 0..1, one per voxel.
    pub density: Vec<f64>,
    /// Thresholded and cleaned.
    pub mask: Vec<bool>,
    /// kg of the thresholded body.
    pub mass: f64,
    pub volume_fraction: f64,
    /// Pa, worst load case, at this resolution.
    pub max_stress: f64,
    pub safety_factor: f64,
    pub max_displacement: f64,
    pub iterations: usize,
    pub compliance_history: Vec<f64>,
    pub advice: String,
}

fn refine_once(problem: &GenerativeProblem, seed: &Design, options: &FieldOptions) -> FieldBody {
    let mut body = FieldBody::default();
    let resolution = options.resolution.clamp(12, 72);
    let grid = build_design_grid(problem, Some(resolution));
    if grid.nx == 0 || grid.space.is_empty() {
        body.advice = "the design space is empty".into();
        return body;
    }
    let (nx, ny, nz) = (grid.nx, grid.ny, grid.nz);
    let count = grid.count();
    body.nx = nx;
    body.ny = ny;
    body.nz = nz;
    body.h = grid.h;
    body.origin = grid.origin;
    let allowed = grid.space.clone();
    let mut fixed_solid = vec![false; count];
    for voxels in grid.mount_voxels.iter().chain(grid.load_voxels.iter()) {
        for &v in voxels {
            if v < count {
                fixed_solid[v] = true;
            }
        }
    }
    let room = allowed.iter().filter(|a| **a).count();
    if room == 0 {
        body.advice = "the design space has no room at this resolution".into();
        return body;
    }
    // Seeded from the skeleton: starting a field from that beats a uniform grey block.
    let seed_mask = rasterise(&grid, seed);
    let fraction = options.volume_fraction.clamp(0.02, 0.9);
    let mut density = vec![0.0; count];
    for v in 0..count {
        if !allowed[v] {
            continue;
        }
        density[v] = if seed_mask[v] { 1.0 } else { fraction.min(0.5) };
        if fixed_solid[v] {
            density[v] = 1.0;
        }
    }
    let minimum_feature = if options.minimum_feature > 0.0 { options.minimum_feature } else { grid.h * 2.0 };
    let radius = (minimum_feature / grid.h).max(1.2);
    let reach = (radius.ceil() as i64).clamp(1, 8);
    let weight = |di: i64, dj: i64, dk: i64| (radius - ((di * di + dj * dj + dk * dk) as f64).sqrt()).max(0.0);
    let neighbourhood = |v: usize, f: &mut dyn FnMut(usize, f64)| {
        let (i, j, k) = grid.ijk(v);
        for dk in -reach..=reach {
            for dj in -reach..=reach {
                for di in -reach..=reach {
                    let (ni, nj, nk) = (i as i64 + di, j as i64 + dj, k as i64 + dk);
                    if !grid.in_bounds(ni, nj, nk) {
                        continue;
                    }
                    let n = grid.at(ni as usize, nj as usize, nk as usize);
                    if allowed[n] {
                        f(n, weight(di, dj, dk));
                    }
                }
            }
        }
    };
    let mut weight_sum = vec![0.0; count];
    for v in 0..count {
        if !allowed[v] {
            continue;
        }
        let mut sum = 0.0;
        neighbourhood(v, &mut |_, w| sum += w);
        weight_sum[v] = sum.max(1e-12);
    }
    let material = OrthotropicMaterial::isotropic(problem.youngs_modulus, problem.poisson_ratio);
    let mut fem = VoxelFemProblem::new(nx, ny, nz, grid.h, material);
    fem.origin = grid.origin;
    fem.solid = allowed.clone();
    fem.held = (0..count).map(|v| allowed[v] && grid.mount_mask[v]).collect();
    fem.exclude_from_peak = fixed_solid.clone();
    fem.max_iterations = 900;
    fem.tolerance = 1e-6;
    for (index, voxels) in grid.load_voxels.iter().enumerate() {
        if let Some(load) = problem.loads.get(index) {
            fem.cases.push(LoadCase { voxels: voxels.clone(), force: load.force, name: load.region.name.clone() });
        }
    }
    if fem.cases.is_empty() {
        body.advice = "nothing pushes on it, so there is nothing to refine against".into();
        return body;
    }
    let penalty = options.penalty.clamp(1.0, 5.0);
    let minimum_stiffness = 1e-6;
    let mut filtered = vec![0.0; count];
    let iterations = options.iterations.clamp(4, 200);
    for step in 0..iterations {
        // What the physics sees is the smoothed field: that stops the checkerboard.
        for v in 0..count {
            if !allowed[v] {
                filtered[v] = 0.0;
                continue;
            }
            let mut sum = 0.0;
            neighbourhood(v, &mut |n, w| sum += w * density[n]);
            filtered[v] = (sum / weight_sum[v]).clamp(0.0, 1.0);
            if fixed_solid[v] {
                filtered[v] = 1.0;
            }
        }
        fem.element_scale =
            (0..count).map(|v| if allowed[v] { minimum_stiffness + filtered[v].powf(penalty) * (1.0 - minimum_stiffness) } else { 0.0 }).collect();
        let result = solve_voxels(&fem);
        if !result.solved {
            body.advice = result.advice;
            return body;
        }
        body.compliance_history.push(result.cases.iter().map(|c| c.compliance).sum());
        // Sensitivity of the compliance to each element's density.
        let sensitivity: Vec<f64> = (0..count)
            .map(|v| {
                if !allowed[v] || fixed_solid[v] {
                    0.0
                } else {
                    -penalty * 2.0 * result.voxel_energy.get(v).copied().unwrap_or(0.0) / filtered[v].max(1e-3)
                }
            })
            .collect();
        let mut smoothed = vec![0.0; count];
        for v in 0..count {
            if !allowed[v] {
                continue;
            }
            let mut sum = 0.0;
            neighbourhood(v, &mut |n, w| sum += w * sensitivity[n]);
            smoothed[v] = sum / weight_sum[v];
        }
        // Optimality criteria update, the volume held by bisection on the Lagrange multiplier.
        let (mut low, mut high) = (1e-12, 1e12);
        let target = fraction * room as f64;
        let mut updated = vec![0.0; count];
        for _ in 0..60 {
            if (high - low) / (low + high) <= 1e-4 {
                break;
            }
            let middle = 0.5 * (low + high);
            let mut volume = 0.0;
            for v in 0..count {
                if !allowed[v] {
                    continue;
                }
                if fixed_solid[v] {
                    updated[v] = 1.0;
                    volume += 1.0;
                    continue;
                }
                let ratio = ((-smoothed[v]).max(0.0) / middle).sqrt();
                let moved = (density[v] * ratio).clamp(density[v] - 0.2, density[v] + 0.2);
                updated[v] = moved.clamp(0.0, 1.0);
                volume += updated[v];
            }
            if volume > target {
                low = middle;
            } else {
                high = middle;
            }
        }
        let change = (0..count).map(|v| (updated[v] - density[v]).abs()).fold(0.0, f64::max);
        density = updated;
        body.iterations = step + 1;
        if change < 0.01 {
            break;
        }
    }
    body.density = filtered.clone();
    // Threshold, then keep only what is joined to a mount: faint islands are not part of it.
    // A field stopped early is grey where its members will be, so the cut is lowered, step by
    // step, until every load is carried to a mount; a body that holds nothing is no answer.
    let connected_mask = |threshold: f64| -> (Vec<bool>, bool) {
        let mut mask: Vec<bool> = (0..count).map(|v| allowed[v] && (filtered[v] >= threshold || fixed_solid[v])).collect();
        let mut reached = vec![false; count];
        let mut queue: Vec<usize> = (0..count).filter(|v| mask[*v] && grid.mount_mask[*v]).collect();
        for &v in &queue {
            reached[v] = true;
        }
        let mut head = 0;
        while head < queue.len() {
            let v = queue[head];
            head += 1;
            for n in grid.neighbours(v) {
                if mask[n] && !reached[n] {
                    reached[n] = true;
                    queue.push(n);
                }
            }
        }
        for v in 0..count {
            if mask[v] && !reached[v] {
                mask[v] = false;
            }
        }
        let loads_carried = grid.load_voxels.iter().all(|voxels| voxels.iter().any(|v| *v < count && mask[*v]));
        (mask, loads_carried)
    };
    let mut threshold = 0.5;
    let (mut mask, mut carried) = connected_mask(threshold);
    while !carried && threshold > 0.1 + 1e-9 {
        threshold -= 0.05;
        (mask, carried) = connected_mask(threshold);
    }
    if !carried {
        body.advice = "the field never joined every load to a mount: more iterations, a larger volume fraction or a finer grid".into();
    } else if threshold < 0.5 - 1e-9 {
        body.advice = format!("thresholded at {threshold:.2} rather than 0.5 so every load is carried to a mount; more iterations would sharpen it");
    }
    let solid_voxels = mask.iter().filter(|m| **m).count();
    body.volume_fraction = solid_voxels as f64 / room as f64;
    body.mass = solid_voxels as f64 * grid.h * grid.h * grid.h * problem.density;
    // What the thresholded body, not the grey field, actually carries.
    fem.solid = mask.clone();
    fem.element_scale.clear();
    fem.held = (0..count).map(|v| mask[v] && grid.mount_mask[v]).collect();
    let check = solve_voxels(&fem);
    body.mask = mask;
    if check.solved {
        body.max_stress = check.worst_von_mises;
        body.max_displacement = check.worst_displacement;
        body.safety_factor = if check.worst_von_mises > 0.0 { problem.yield_strength / check.worst_von_mises } else { 0.0 };
        body.solved = true;
    } else {
        body.advice = format!("the refined body did not solve: {}", check.advice);
        return body;
    }
    if body.safety_factor < problem.target_safety_factor {
        body.advice = format!(
            "the refined body reaches safety factor {:.2} against a target of {:.2}: allow it more volume, or a coarser minimum feature.",
            body.safety_factor, problem.target_safety_factor
        );
    }
    body
}

/// Refines a skeleton design (or an empty one: a field from nothing but the boundary conditions)
/// into a density field on a finer grid, hunting for the lightest volume fraction that still
/// reaches the target safety factor.
pub fn refine_to_field(problem: &GenerativeProblem, seed: &Design, options: &FieldOptions) -> FieldBody {
    let mut attempt = options.clone();
    let mut body = refine_once(problem, seed, &attempt);
    if !options.meet_target || !body.solved {
        return body;
    }
    let target = problem.target_safety_factor;
    let (mut too_little, mut enough) = (0.0, 0.0);
    let mut best = FieldBody::default();
    if body.safety_factor >= target {
        enough = attempt.volume_fraction;
        best = body.clone();
    } else {
        too_little = attempt.volume_fraction;
    }
    let tries = options.meet_target_tries.min(8);
    for _ in 0..tries {
        let next = if enough == 0.0 {
            let shortfall = if body.safety_factor > 0.0 { target / body.safety_factor } else { 2.0 };
            let next = (attempt.volume_fraction * shortfall.clamp(1.2, 2.0)).min(0.85);
            if next <= attempt.volume_fraction + 1e-6 {
                body.advice = "even filling most of the design space does not reach the target safety factor: the space, the material or the load has to change.".into();
                break;
            }
            next
        } else {
            if enough - too_little < 0.02 {
                break;
            }
            0.5 * (too_little + enough)
        };
        attempt.volume_fraction = next;
        let tried = refine_once(problem, seed, &attempt);
        if !tried.solved {
            break;
        }
        if tried.safety_factor >= target {
            enough = next;
            best = tried.clone();
        } else {
            too_little = next;
        }
        body = tried;
    }
    if enough > 0.0 {
        best.advice.clear();
        if (enough - options.volume_fraction).abs() > 1e-9 {
            best.advice = format!(
                "the volume fraction was tuned from {:.0}% to {:.0}% -- the lightest body that still reaches the target safety factor.",
                options.volume_fraction * 100.0,
                enough * 100.0
            );
        }
        return best;
    }
    body
}

/// Fills the voxels whose absence would make the skin non-manifold: two voxels touching only
/// along an edge or at a corner.
pub(crate) fn fill_diagonal_contacts(mask: &mut [bool], nx: usize, ny: usize, nz: usize) {
    let at = |i: i64, j: i64, k: i64| (k as usize * ny + j as usize) * nx + i as usize;
    let in_bounds = |i: i64, j: i64, k: i64| i >= 0 && j >= 0 && k >= 0 && (i as usize) < nx && (j as usize) < ny && (k as usize) < nz;
    let steps: [[i64; 3]; 3] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for _ in 0..3 {
        let mut filled = mask.to_vec();
        let mut changed = false;
        let solid = |i: i64, j: i64, k: i64| in_bounds(i, j, k) && mask[at(i, j, k)];
        for k in 0..nz as i64 {
            for j in 0..ny as i64 {
                for i in 0..nx as i64 {
                    if !solid(i, j, k) {
                        continue;
                    }
                    for a in 0..3 {
                        for b in 0..3 {
                            if a == b {
                                continue;
                            }
                            for sa in [-1, 1] {
                                for sb in [-1, 1] {
                                    let (ai, aj, ak) = (i + steps[a][0] * sa, j + steps[a][1] * sa, k + steps[a][2] * sa);
                                    let (bi, bj, bk) = (i + steps[b][0] * sb, j + steps[b][1] * sb, k + steps[b][2] * sb);
                                    let (di, dj, dk) = (ai + steps[b][0] * sb, aj + steps[b][1] * sb, ak + steps[b][2] * sb);
                                    if !solid(di, dj, dk) || solid(ai, aj, ak) || solid(bi, bj, bk) || !in_bounds(ai, aj, ak) {
                                        continue;
                                    }
                                    filled[at(ai, aj, ak)] = true;
                                    changed = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        mask.copy_from_slice(&filled);
        if !changed {
            break;
        }
    }
}

/// The surface of a voxel body as a closed triangle mesh in mm: every face of every voxel that
/// has nothing behind it, corners welded, wound outwards, then Taubin-smoothed (a shrinking pass
/// followed by an expanding one, so the staircase goes and the volume stays), each vertex kept
/// within half a voxel of where it started so a web one voxel thick survives.
pub fn mesh_from_mask(mask: &[bool], nx: usize, ny: usize, nz: usize, h: f64, origin: Vec3, smoothing_passes: usize) -> Option<Mesh> {
    let count = nx.checked_mul(ny)?.checked_mul(nz)?;
    if count == 0 || mask.len() != count || !(h > 0.0) || count > MAX_GRID_CELLS * 4 {
        return None;
    }
    let mut mask = mask.to_vec();
    fill_diagonal_contacts(&mut mask, nx, ny, nz);
    let solid = |i: i64, j: i64, k: i64| {
        i >= 0
            && j >= 0
            && k >= 0
            && (i as usize) < nx
            && (j as usize) < ny
            && (k as usize) < nz
            && mask[(k as usize * ny + j as usize) * nx + i as usize]
    };
    let (lx, ly) = (nx + 1, ny + 1);
    let mut vertex_of: Vec<Option<u32>> = vec![None; lx * ly * (nz + 1)];
    let mut points: Vec<Vec3> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut corner = |i: usize, j: usize, k: usize| -> u32 {
        let slot = (k * ly + j) * lx + i;
        if let Some(v) = vertex_of[slot] {
            return v;
        }
        let v = points.len() as u32;
        points.push((origin + Vec3::new(i as f64, j as f64, k as f64) * h) * MM);
        vertex_of[slot] = Some(v);
        v
    };
    let mut quad = |a: u32, b: u32, c: u32, d: u32| {
        triangles.push([a, b, c]);
        triangles.push([a, c, d]);
    };
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let (ii, jj, kk) = (i as i64, j as i64, k as i64);
                if !solid(ii, jj, kk) {
                    continue;
                }
                if !solid(ii + 1, jj, kk) {
                    let (a, b, c, d) = (corner(i + 1, j, k), corner(i + 1, j + 1, k), corner(i + 1, j + 1, k + 1), corner(i + 1, j, k + 1));
                    quad(a, b, c, d);
                }
                if !solid(ii - 1, jj, kk) {
                    let (a, b, c, d) = (corner(i, j, k), corner(i, j, k + 1), corner(i, j + 1, k + 1), corner(i, j + 1, k));
                    quad(a, b, c, d);
                }
                if !solid(ii, jj + 1, kk) {
                    let (a, b, c, d) = (corner(i, j + 1, k), corner(i, j + 1, k + 1), corner(i + 1, j + 1, k + 1), corner(i + 1, j + 1, k));
                    quad(a, b, c, d);
                }
                if !solid(ii, jj - 1, kk) {
                    let (a, b, c, d) = (corner(i, j, k), corner(i + 1, j, k), corner(i + 1, j, k + 1), corner(i, j, k + 1));
                    quad(a, b, c, d);
                }
                if !solid(ii, jj, kk + 1) {
                    let (a, b, c, d) = (corner(i, j, k + 1), corner(i + 1, j, k + 1), corner(i + 1, j + 1, k + 1), corner(i, j + 1, k + 1));
                    quad(a, b, c, d);
                }
                if !solid(ii, jj, kk - 1) {
                    let (a, b, c, d) = (corner(i, j, k), corner(i, j + 1, k), corner(i + 1, j + 1, k), corner(i + 1, j, k));
                    quad(a, b, c, d);
                }
            }
        }
    }
    if triangles.is_empty() {
        return None;
    }
    if smoothing_passes > 0 {
        let mut neighbours: Vec<Vec<u32>> = vec![Vec::new(); points.len()];
        for t in &triangles {
            for n in 0..3 {
                neighbours[t[n] as usize].push(t[(n + 1) % 3]);
                neighbours[t[n] as usize].push(t[(n + 2) % 3]);
            }
        }
        let anchors = points.clone();
        let (lambda, mu) = (0.5, -0.53);
        let limit = h * MM * 0.5;
        for pass in 0..smoothing_passes.min(50) * 2 {
            let weight = if pass % 2 == 0 { lambda } else { mu };
            let mut moved = points.clone();
            for (v, out) in moved.iter_mut().enumerate() {
                if neighbours[v].is_empty() {
                    continue;
                }
                let sum = neighbours[v].iter().fold(Vec3::ZERO, |acc, n| acc + points[*n as usize]);
                let average = sum / neighbours[v].len() as f64;
                let mut next = points[v] + (average - points[v]) * weight;
                let drift = next - anchors[v];
                let distance = drift.len();
                if distance > limit {
                    next = anchors[v] + drift * (limit / distance);
                }
                *out = next;
            }
            points = moved;
        }
    }
    // Vertex normals from the faces around them, for display.
    let mut normals = vec![Vec3::ZERO; points.len()];
    for t in &triangles {
        let (a, b, c) = (points[t[0] as usize], points[t[1] as usize], points[t[2] as usize]);
        let n = (b - a).cross(c - a);
        for i in t {
            normals[*i as usize] += n;
        }
    }
    for n in &mut normals {
        *n = n.normalized().unwrap_or(Vec3::Z);
    }
    let tri_face = vec![0u32; triangles.len()];
    Some(Mesh { positions: points, normals, triangles, tri_face, ..Default::default() })
}

/// The mask, run-length encoded, so a refined body can be stored and rebuilt without running the
/// optimiser again.
pub fn encode_field(body: &FieldBody) -> String {
    let mut text = format!("f1 {} {} {} {:e} {:e} {:e} {:e} ", body.nx, body.ny, body.nz, body.h, body.origin.x, body.origin.y, body.origin.z);
    let mut current = body.mask.first().copied().unwrap_or(false);
    let mut run = 0usize;
    for &value in &body.mask {
        if value == current {
            run += 1;
            continue;
        }
        text.push_str(&format!("{}:{run} ", u8::from(current)));
        current = value;
        run = 1;
    }
    if run > 0 {
        text.push_str(&format!("{}:{run}", u8::from(current)));
    }
    text
}

pub fn decode_field(text: &str) -> FieldBody {
    let mut body = FieldBody::default();
    let mut tokens = text.split_whitespace();
    if tokens.next() != Some("f1") {
        return body;
    }
    let mut number = || tokens.next().and_then(|t| t.parse::<f64>().ok());
    let (Some(nx), Some(ny), Some(nz), Some(h), Some(ox), Some(oy), Some(oz)) =
        (number(), number(), number(), number(), number(), number(), number())
    else {
        return body;
    };
    if !(nx >= 1.0 && ny >= 1.0 && nz >= 1.0 && nx * ny * nz <= (MAX_GRID_CELLS * 4) as f64) || !(h > 0.0) || !h.is_finite() {
        return body;
    }
    body.nx = nx as usize;
    body.ny = ny as usize;
    body.nz = nz as usize;
    body.h = h;
    body.origin = Vec3::new(ox, oy, oz);
    let expected = body.nx * body.ny * body.nz;
    for token in tokens {
        let Some((value, run)) = token.split_once(':') else { continue };
        let (Ok(value), Ok(run)) = (value.parse::<u8>(), run.parse::<usize>()) else { continue };
        if body.mask.len() + run > expected {
            body.mask.clear();
            return body;
        }
        body.mask.extend(std::iter::repeat_n(value != 0, run));
    }
    if body.mask.len() != expected {
        body.mask.clear();
        return body;
    }
    body.density = body.mask.iter().map(|m| if *m { 1.0 } else { 0.0 }).collect();
    body.solved = true;
    body
}
