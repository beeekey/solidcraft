//! Large displacement: what happens after the eigenvalue.
//!
//! Everything in `voxel_fem` is linear: equilibrium is written on the undeformed shape, so a
//! buckling load is an eigenvalue rather than something that happens. This is the other kind of
//! solve. Each element carries its own rotation, the best fit between where its corners started
//! and where they are now, and its internal force is the small-strain force in that rotated
//! frame put back into global axes: f = R Ke (Rᵀ x − x₀), exactly zero for any rigid rotation.
//! The load is walked on in steps and each step solved by Newton–Raphson until the residual, the
//! real out-of-balance force on the deformed shape, is gone. Past the limit load no equilibrium
//! exists, so Newton cannot converge, which is how the limit announces itself.

use serde::{Deserialize, Serialize};
use solidcraft_geom::Vec3;

use crate::buckling::{BucklingResult, grid_dofs};
use crate::element::Element;
use crate::voxel_fem::VoxelFemProblem;

/// Options of a large-displacement solve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LargeDisplacementOptions {
    /// Equal steps the load is walked on in.
    pub load_steps: usize,
    pub max_newton_iterations: usize,
    /// Relative to the applied load: an out-of-balance force this much smaller is equilibrium.
    pub tolerance: f64,
    pub load_case: usize,
    /// An offset added to the rest positions, three per grid node: the structure starts crooked.
    /// Nothing buckles without it.
    pub imperfection: Vec<f64>,
}

impl Default for LargeDisplacementOptions {
    fn default() -> Self {
        LargeDisplacementOptions { load_steps: 8, max_newton_iterations: 40, tolerance: 1e-6, load_case: 0, imperfection: Vec::new() }
    }
}

/// One load step that reached equilibrium.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Step {
    /// Of the load case as given.
    pub load_fraction: f64,
    /// Largest nodal movement (m).
    pub displacement: f64,
    /// Largest movement across the load (m).
    pub lateral: f64,
    pub newton_iterations: usize,
    /// Largest element rotation.
    pub rotation_degrees: f64,
}

/// The answer.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LargeDisplacementResult {
    pub solved: bool,
    pub advice: String,
    pub path: Vec<Step>,
    pub carried_the_whole_load: bool,
    /// The largest fraction of the load an equilibrium was found for (1 when all of it).
    pub limit_load_fraction: f64,
    /// Displacement at the last converged step, three per grid node.
    pub displacement: Vec<f64>,
}

type Matrix3 = [f64; 9];

const IDENTITY: Matrix3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

fn determinant3(m: &Matrix3) -> f64 {
    m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6])
}

fn invert_transpose3(m: &Matrix3) -> Option<Matrix3> {
    let det = determinant3(m);
    if !(det.abs() > 1e-300) {
        return None;
    }
    let inverse = [
        (m[4] * m[8] - m[5] * m[7]) / det,
        (m[2] * m[7] - m[1] * m[8]) / det,
        (m[1] * m[5] - m[2] * m[4]) / det,
        (m[5] * m[6] - m[3] * m[8]) / det,
        (m[0] * m[8] - m[2] * m[6]) / det,
        (m[2] * m[3] - m[0] * m[5]) / det,
        (m[3] * m[7] - m[4] * m[6]) / det,
        (m[1] * m[6] - m[0] * m[7]) / det,
        (m[0] * m[4] - m[1] * m[3]) / det,
    ];
    let mut out = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            out[3 * i + j] = inverse[3 * j + i];
        }
    }
    Some(out)
}

/// The rotation closest to `a`: the R of the polar decomposition A = R U, by Higham's iteration
/// R ← (R + R⁻ᵀ) / 2, which converges quadratically from any A with a positive determinant.
pub fn closest_rotation(a: &Matrix3) -> Matrix3 {
    if determinant3(a) <= 0.0 {
        return IDENTITY;
    }
    let mut r = *a;
    for _ in 0..24 {
        let Some(inverse_transpose) = invert_transpose3(&r) else { return IDENTITY };
        let mut change = 0.0_f64;
        for e in 0..9 {
            let next = 0.5 * (r[e] + inverse_transpose[e]);
            change = change.max((next - r[e]).abs());
            r[e] = next;
        }
        if change < 1e-15 {
            break;
        }
    }
    r
}

fn rotation_angle(r: &Matrix3) -> f64 {
    let trace = r[0] + r[4] + r[8];
    ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos()
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Walks the load on and follows the structure.
pub fn large_displacement(problem: &VoxelFemProblem, options: &LargeDisplacementOptions) -> LargeDisplacementResult {
    let mut result = LargeDisplacementResult::default();
    let (nx, ny, nz, h) = (problem.nx, problem.ny, problem.nz, problem.h);
    let voxel_count = nx * ny * nz;
    if nx == 0 || ny == 0 || nz == 0 || !(h > 0.0) || !h.is_finite() || problem.solid.len() != voxel_count || problem.held.len() != voxel_count {
        result.advice = "the grid, the masks and the material must all be given".into();
        return result;
    }
    let Ok(element) = Element::build(&problem.material, h) else {
        result.advice = "the material is not usable".into();
        return result;
    };
    if !problem.body_of.is_empty() {
        result.advice = "a separating contact and a large-displacement solve are two different solvers; run them one at a time".into();
        return result;
    }
    let Some(load) = problem.cases.get(options.load_case) else {
        result.advice = "that load case does not exist".into();
        return result;
    };
    let elements: Vec<usize> = (0..voxel_count).filter(|v| problem.solid[*v]).collect();
    if elements.is_empty() {
        result.advice = "there is no material to push on".into();
        return result;
    }
    let (nnx, nny, nnz) = (nx + 1, ny + 1, nz + 1);
    let node_count = nnx * nny * nnz;
    let dof_count = node_count * 3;
    let dofs: Vec<[usize; 24]> = elements.iter().map(|v| grid_dofs(problem, *v)).collect();

    // Rest geometry, crooked if an imperfection was given.
    let mut rest = vec![0.0; dof_count];
    for k in 0..nnz {
        for j in 0..nny {
            for i in 0..nnx {
                let node = (k * nny + j) * nnx + i;
                rest[3 * node] = problem.origin.x + i as f64 * h;
                rest[3 * node + 1] = problem.origin.y + j as f64 * h;
                rest[3 * node + 2] = problem.origin.z + k as f64 * h;
            }
        }
    }
    if options.imperfection.len() == dof_count {
        for (d, offset) in options.imperfection.iter().enumerate() {
            rest[d] += offset;
        }
    } else if !options.imperfection.is_empty() {
        result.advice = "the imperfection must give three numbers for every grid node".into();
        return result;
    }

    let mut fixed = vec![false; dof_count];
    for (index, voxel) in elements.iter().enumerate() {
        if problem.held[*voxel] {
            for d in dofs[index] {
                fixed[d] = true;
            }
        }
    }

    // The load case, spread over the nodes of the voxels it acts on as the linear solver does.
    let mut external = vec![0.0; dof_count];
    {
        let mut weight = vec![0.0; node_count];
        let mut total = 0.0;
        for voxel in &load.voxels {
            if *voxel >= voxel_count || !problem.solid[*voxel] {
                continue;
            }
            for d in grid_dofs(problem, *voxel).iter().step_by(3) {
                weight[d / 3] += 1.0;
                total += 1.0;
            }
        }
        if total <= 0.0 {
            result.advice = "the load has no material to push on".into();
            return result;
        }
        for (node, w) in weight.iter().enumerate() {
            if *w > 0.0 {
                let share = w / total;
                external[3 * node] = load.force.x * share;
                external[3 * node + 1] = load.force.y * share;
                external[3 * node + 2] = load.force.z * share;
            }
        }
    }
    let load_norm = dot(&external, &external).sqrt();
    if !(load_norm > 0.0) {
        result.advice = "that load case applies no force".into();
        return result;
    }
    let load_direction = load.force.normalized().unwrap_or(Vec3::Z);

    let e_ref = problem.material.reference();
    let graded = problem.element_scale.len() == voxel_count;
    let modulus: Vec<f64> = elements.iter().map(|v| if graded { e_ref * problem.element_scale[*v].max(0.0) } else { e_ref }).collect();

    // The element stiffness is written for a cube square to the axes, so an element whose rest
    // shape is tilted (what an imperfection makes it) gets its own tilt fitted once, and its
    // strain is measured from there.
    let half = h / 2.0;
    let cube: [[f64; 3]; 8] = [
        [-half, -half, -half],
        [half, -half, -half],
        [half, half, -half],
        [-half, half, -half],
        [-half, -half, half],
        [half, -half, half],
        [half, half, half],
        [-half, half, half],
    ];
    let node_of = |dofs: &[usize; 24], n: usize| dofs[3 * n] / 3;
    let mut rest_local = vec![[0.0_f64; 24]; elements.len()];
    for (e, dofs) in dofs.iter().enumerate() {
        let mut centre = [0.0; 3];
        for n in 0..8 {
            let node = node_of(dofs, n);
            for c in 0..3 {
                centre[c] += rest[3 * node + c] / 8.0;
            }
        }
        let mut fit = [0.0; 9];
        for (n, corner) in cube.iter().enumerate() {
            let node = node_of(dofs, n);
            for a in 0..3 {
                for b in 0..3 {
                    fit[3 * a + b] += (rest[3 * node + a] - centre[a]) * corner[b];
                }
            }
        }
        let r0 = closest_rotation(&fit);
        for n in 0..8 {
            let node = node_of(dofs, n);
            for c in 0..3 {
                let mut unrotated = 0.0;
                for a in 0..3 {
                    unrotated += r0[3 * a + c] * (rest[3 * node + a] - centre[a]);
                }
                rest_local[e][3 * n + c] = unrotated;
            }
        }
    }

    let mut rotation = vec![IDENTITY; elements.len()];
    // Internal force on the deformed shape, and the rotations it used.
    let internal_force = |u: &[f64], out: &mut [f64], rotation: &mut [Matrix3]| {
        out.fill(0.0);
        for (e, dofs) in dofs.iter().enumerate() {
            let mut now_centre = [0.0; 3];
            for n in 0..8 {
                let node = node_of(dofs, n);
                for c in 0..3 {
                    now_centre[c] += (rest[3 * node + c] + u[3 * node + c]) / 8.0;
                }
            }
            let mut fit = [0.0; 9];
            let mut moved = [[0.0; 3]; 8];
            for (n, corner) in cube.iter().enumerate() {
                let node = node_of(dofs, n);
                for c in 0..3 {
                    moved[n][c] = rest[3 * node + c] + u[3 * node + c] - now_centre[c];
                }
                for a in 0..3 {
                    for b in 0..3 {
                        fit[3 * a + b] += moved[n][a] * corner[b];
                    }
                }
            }
            let r = closest_rotation(&fit);
            rotation[e] = r;
            let mut local = [0.0; 24];
            for n in 0..8 {
                for c in 0..3 {
                    let mut unrotated = 0.0;
                    for a in 0..3 {
                        unrotated += r[3 * a + c] * moved[n][a];
                    }
                    local[3 * n + c] = unrotated - rest_local[e][3 * n + c];
                }
            }
            let mut force = [0.0; 24];
            for (a, out_force) in force.iter_mut().enumerate() {
                let mut sum = 0.0;
                for (b, local_b) in local.iter().enumerate() {
                    sum += element.ke[a][b] * local_b;
                }
                *out_force = sum * modulus[e];
            }
            for n in 0..8 {
                let node = node_of(dofs, n);
                for c in 0..3 {
                    let mut global = 0.0;
                    for a in 0..3 {
                        global += r[3 * c + a] * force[3 * n + a];
                    }
                    out[3 * node + c] += global;
                }
            }
        }
    };
    // The tangent the Newton step uses: each element's stiffness carried into its own rotation.
    let multiply_tangent = |x: &[f64], y: &mut [f64], rotation: &[Matrix3]| {
        y.fill(0.0);
        for (e, dofs) in dofs.iter().enumerate() {
            let r = &rotation[e];
            let mut local = [0.0; 24];
            for n in 0..8 {
                let node = node_of(dofs, n);
                for c in 0..3 {
                    let mut sum = 0.0;
                    for a in 0..3 {
                        sum += r[3 * a + c] * x[3 * node + a];
                    }
                    local[3 * n + c] = sum;
                }
            }
            let mut force = [0.0; 24];
            for (a, out_force) in force.iter_mut().enumerate() {
                let mut sum = 0.0;
                for (b, local_b) in local.iter().enumerate() {
                    sum += element.ke[a][b] * local_b;
                }
                *out_force = sum * modulus[e];
            }
            for n in 0..8 {
                let node = node_of(dofs, n);
                for c in 0..3 {
                    let mut global = 0.0;
                    for a in 0..3 {
                        global += r[3 * c + a] * force[3 * n + a];
                    }
                    y[3 * node + c] += global;
                }
            }
        }
        for (d, f) in fixed.iter().enumerate() {
            if *f {
                y[d] = x[d];
            }
        }
    };
    let preconditioner = |rotation: &[Matrix3], inverse_diagonal: &mut [f64]| {
        let mut diagonal = vec![0.0; dof_count];
        for (e, dofs) in dofs.iter().enumerate() {
            let r = &rotation[e];
            for n in 0..8 {
                let node = node_of(dofs, n);
                for c in 0..3 {
                    let mut sum = 0.0;
                    for a in 0..3 {
                        for b in 0..3 {
                            sum += r[3 * c + a] * element.ke[3 * n + a][3 * n + b] * r[3 * c + b];
                        }
                    }
                    diagonal[3 * node + c] += sum * modulus[e];
                }
            }
        }
        for d in 0..dof_count {
            inverse_diagonal[d] = if fixed[d] || !(diagonal[d] > 0.0) { 1.0 } else { 1.0 / diagonal[d] };
        }
    };

    let mut inverse_diagonal = vec![0.0; dof_count];
    let mut step = vec![0.0; dof_count];
    let mut residual = vec![0.0; dof_count];
    let mut direction = vec![0.0; dof_count];
    let mut applied = vec![0.0; dof_count];
    let mut preconditioned = vec![0.0; dof_count];
    let mut internal = vec![0.0; dof_count];
    let mut out_of_balance = vec![0.0; dof_count];
    let mut displacement = vec![0.0; dof_count];
    let mut last_good = displacement.clone();
    let steps = options.load_steps.max(1);
    let size_limit = 0.5 * h * nx.max(ny).max(nz) as f64;
    let cg_limit = (dof_count * 2 + 200).min(20_000);

    for s in 1..=steps {
        let fraction = s as f64 / steps as f64;
        let mut converged = false;
        let mut iterations = 0;
        while iterations < options.max_newton_iterations.max(1) {
            internal_force(&displacement, &mut internal, &mut rotation);
            let mut norm = 0.0;
            for d in 0..dof_count {
                out_of_balance[d] = if fixed[d] { 0.0 } else { fraction * external[d] - internal[d] };
                norm += out_of_balance[d] * out_of_balance[d];
            }
            if norm.sqrt() <= options.tolerance * fraction * load_norm {
                converged = true;
                break;
            }
            iterations += 1;
            preconditioner(&rotation, &mut inverse_diagonal);
            // Preconditioned conjugate gradients on the tangent.
            step.fill(0.0);
            residual.copy_from_slice(&out_of_balance);
            for d in 0..dof_count {
                if fixed[d] {
                    residual[d] = 0.0;
                }
                preconditioned[d] = inverse_diagonal[d] * residual[d];
            }
            direction.copy_from_slice(&preconditioned);
            let mut rz = dot(&residual, &preconditioned);
            let target = dot(&out_of_balance, &out_of_balance) * 1e-16;
            for _ in 0..cg_limit {
                if rz <= target || rz <= 0.0 {
                    break;
                }
                multiply_tangent(&direction, &mut applied, &rotation);
                for d in 0..dof_count {
                    if fixed[d] {
                        applied[d] = 0.0;
                    }
                }
                let denominator = dot(&direction, &applied);
                if !(denominator.abs() > 1e-300) {
                    break;
                }
                let alpha = rz / denominator;
                for d in 0..dof_count {
                    step[d] += alpha * direction[d];
                    residual[d] -= alpha * applied[d];
                    preconditioned[d] = inverse_diagonal[d] * residual[d];
                }
                let next = dot(&residual, &preconditioned);
                let beta = next / rz;
                rz = next;
                for d in 0..dof_count {
                    direction[d] = preconditioned[d] + beta * direction[d];
                }
            }
            let mut biggest = 0.0_f64;
            for d in 0..dof_count {
                if fixed[d] {
                    step[d] = 0.0;
                }
                displacement[d] += step[d];
                biggest = biggest.max(displacement[d].abs());
            }
            if !biggest.is_finite() || biggest > size_limit {
                break;
            }
        }
        if !converged {
            displacement.copy_from_slice(&last_good);
            break;
        }
        last_good.copy_from_slice(&displacement);
        let mut record = Step { load_fraction: fraction, newton_iterations: iterations, ..Default::default() };
        for node in 0..node_count {
            let d = Vec3::new(displacement[3 * node], displacement[3 * node + 1], displacement[3 * node + 2]);
            record.displacement = record.displacement.max(d.len());
            let along = d.dot(load_direction);
            record.lateral = record.lateral.max((d.len2() - along * along).max(0.0).sqrt());
        }
        let worst_rotation = rotation.iter().map(rotation_angle).fold(0.0_f64, f64::max);
        record.rotation_degrees = worst_rotation.to_degrees();
        result.path.push(record);
        result.limit_load_fraction = fraction;
    }
    result.displacement = displacement;
    result.solved = !result.path.is_empty();
    result.carried_the_whole_load = result.path.len() == steps;
    if !result.solved {
        result.advice =
            "no equilibrium was found even at the first step: the load is far past what this structure can carry, or it is held by nothing".into();
    } else if !result.carried_the_whole_load {
        result.advice = format!(
            "the structure carried {}% of the load and no equilibrium exists past that: this is a limit point, and what happens after it is a collapse, not a deflection",
            (result.limit_load_fraction * 100.0).round()
        );
    }
    result
}

/// The load a real member of this shape collapses at. The eigenvalue is a perfect member's;
/// this bends the member into its own buckling mode by `bow_fraction` of its length, pushes it
/// with `past_critical` times the eigenvalue load, and reports how much of that it held, as a
/// factor on the original load so the two read side by side.
pub fn limit_load(
    problem: &VoxelFemProblem,
    buckling: &BucklingResult,
    bow_fraction: f64,
    past_critical: f64,
    load_steps: usize,
) -> LargeDisplacementResult {
    let mut result = LargeDisplacementResult::default();
    if !buckling.solved || buckling.mode.is_empty() || !(buckling.load_factor > 0.0) {
        result.advice = "there is no buckling mode to bend the member into, so there is nothing for a collapse load to mean here".into();
        return result;
    }
    let Some(case) = buckling.load_case.filter(|c| *c < problem.cases.len()) else {
        result.advice = "the buckling result does not name a load case of this problem".into();
        return result;
    };
    let dof_count = (problem.nx + 1) * (problem.ny + 1) * (problem.nz + 1) * 3;
    if buckling.mode.len() != dof_count {
        result.advice = "the buckling mode does not match this grid".into();
        return result;
    }
    let largest = buckling.mode.chunks_exact(3).map(|m| Vec3::new(m[0], m[1], m[2]).len()).fold(0.0_f64, f64::max);
    if !(largest > 0.0) {
        result.advice = "the buckling mode is flat: nothing to bow".into();
        return result;
    }
    let span = problem.h * problem.nx.max(problem.ny).max(problem.nz) as f64;
    let amplitude = bow_fraction.max(0.0) * span / largest;
    let options = LargeDisplacementOptions {
        load_steps: load_steps.max(1),
        load_case: case,
        imperfection: buckling.mode.iter().map(|m| m * amplitude).collect(),
        ..Default::default()
    };
    let mut pushed = problem.clone();
    let factor = buckling.load_factor * past_critical.max(1.0);
    pushed.cases[case].force = pushed.cases[case].force * factor;
    result = large_displacement(&pushed, &options);
    result.limit_load_fraction *= factor;
    for step in &mut result.path {
        step.load_fraction *= factor;
    }
    if result.solved && result.carried_the_whole_load {
        result.advice = format!("it carried everything it was pushed with: try again with more than {past_critical} times the eigenvalue load");
    }
    result
}
