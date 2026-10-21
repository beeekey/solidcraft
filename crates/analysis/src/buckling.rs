//! Linear buckling: the load factor at which the stiffness, softened by the stress it already
//! carries, stops being positive definite:
//!
//! ```text
//! (K + λ Kg) φ = 0
//! ```
//!
//! Kg is the geometric stiffness built from the solved stress state; the smallest positive λ is
//! how many times the applied load the structure takes before it folds, and φ is how it folds.
//! Solved by inverse iteration, which needs only the matrix-vector products the solver has.

use serde::{Deserialize, Serialize};

use crate::element::{CORNER, Element};
use crate::voxel_fem::{VoxelFemProblem, VoxelFemResult};

/// The buckling answer.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BucklingResult {
    pub solved: bool,
    /// Applied load × this factor = the load it buckles at. Below 1 means it buckles first.
    pub load_factor: f64,
    pub iterations: usize,
    /// Which load case was checked (the governing one).
    pub load_case: Option<usize>,
    /// The mode shape, three per grid node, when one was found.
    pub mode: Vec<f64>,
    /// How much a real member's sideways bow grows at the applied load: 1 / (1 − 1/λ).
    pub amplification: f64,
    /// The initial bow assumed (m): the member's length over 500.
    pub assumed_imperfection: f64,
    /// The extra bending stress that bow carries at the applied load (Pa).
    pub second_order_stress: f64,
    pub advice: String,
}

/// The two operators of a one-body grid and the fixed set they share.
pub(crate) struct Operators {
    pub dof_count: usize,
    pub fixed: Vec<bool>,
    elements: Vec<usize>,
    dofs: Vec<[usize; 24]>,
    element: Element,
    scale: Vec<f64>,
    stresses: Vec<f64>,
    h: f64,
}

impl Operators {
    pub(crate) fn build(problem: &VoxelFemProblem, result: &VoxelFemResult) -> Option<Operators> {
        let (nx, ny, nz, h) = (problem.nx, problem.ny, problem.nz, problem.h);
        let voxel_count = nx * ny * nz;
        if !result.solved || result.voxel_stress_tensor.len() != voxel_count * 6 || !(h > 0.0) || !problem.body_of.is_empty() {
            return None;
        }
        if problem.solid.len() != voxel_count || problem.held.len() != voxel_count {
            return None;
        }
        let elements: Vec<usize> = (0..voxel_count).filter(|v| problem.solid[*v]).collect();
        if elements.is_empty() {
            return None;
        }
        let element = Element::build(&problem.material, h).ok()?;
        let dof_count = (nx + 1) * (ny + 1) * (nz + 1) * 3;
        let dofs: Vec<[usize; 24]> = elements.iter().map(|v| grid_dofs(problem, *v)).collect();
        let mut fixed = vec![false; dof_count];
        for (index, voxel) in elements.iter().enumerate() {
            if problem.held[*voxel] {
                for d in dofs[index] {
                    fixed[d] = true;
                }
            }
        }
        let e_ref = problem.material.reference();
        let graded = problem.element_scale.len() == voxel_count;
        let scale = elements.iter().map(|v| if graded { e_ref * problem.element_scale[*v].max(0.0) } else { e_ref }).collect();
        Some(Operators { dof_count, fixed, elements, dofs, element, scale, stresses: result.voxel_stress_tensor.clone(), h })
    }

    /// y = K x, with the fixed degrees of freedom passed through.
    pub(crate) fn multiply_k(&self, x: &[f64], y: &mut [f64]) {
        y.fill(0.0);
        for (index, dofs) in self.dofs.iter().enumerate() {
            let scale = self.scale[index];
            for a in 0..24 {
                let mut sum = 0.0;
                for b in 0..24 {
                    sum += self.element.ke[a][b] * x[dofs[b]];
                }
                y[dofs[a]] += sum * scale;
            }
        }
        for (d, fixed) in self.fixed.iter().enumerate() {
            if *fixed {
                y[d] = x[d];
            }
        }
    }

    /// y = Kg x: the geometric stiffness of every element under its constant stress state,
    /// Kg = ∫ Gᵀ S G dV, with the fixed degrees of freedom zeroed.
    pub(crate) fn multiply_g(&self, x: &[f64], y: &mut [f64]) {
        y.fill(0.0);
        let h = self.h;
        let gauss = 1.0 / 3.0_f64.sqrt();
        let weight = (h / 2.0).powi(3);
        for (index, voxel) in self.elements.iter().enumerate() {
            let stress = &self.stresses[voxel * 6..voxel * 6 + 6];
            if stress.iter().all(|s| *s == 0.0) {
                continue;
            }
            let sigma = [[stress[0], stress[3], stress[5]], [stress[3], stress[1], stress[4]], [stress[5], stress[4], stress[2]]];
            let mut kg = [[0.0_f64; 8]; 8];
            for gx in [-gauss, gauss] {
                for gy in [-gauss, gauss] {
                    for gz in [-gauss, gauss] {
                        let mut dn = [[0.0_f64; 3]; 8];
                        for (n, corner) in CORNER.iter().enumerate() {
                            let [sx, sy, sz] = *corner;
                            dn[n][0] = 0.125 * sx * (1.0 + sy * gy) * (1.0 + sz * gz) * (2.0 / h);
                            dn[n][1] = 0.125 * sy * (1.0 + sx * gx) * (1.0 + sz * gz) * (2.0 / h);
                            dn[n][2] = 0.125 * sz * (1.0 + sx * gx) * (1.0 + sy * gy) * (2.0 / h);
                        }
                        for a in 0..8 {
                            for b in 0..8 {
                                let mut coupling = 0.0;
                                for i in 0..3 {
                                    for j in 0..3 {
                                        coupling += dn[a][i] * sigma[i][j] * dn[b][j];
                                    }
                                }
                                kg[a][b] += coupling * weight;
                            }
                        }
                    }
                }
            }
            let dofs = &self.dofs[index];
            for a in 0..8 {
                for c in 0..3 {
                    let mut sum = 0.0;
                    for b in 0..8 {
                        sum += kg[a][b] * x[dofs[3 * b + c]];
                    }
                    y[dofs[3 * a + c]] += sum;
                }
            }
        }
        for (d, fixed) in self.fixed.iter().enumerate() {
            if *fixed {
                y[d] = 0.0;
            }
        }
    }
}

/// The 24 degrees of freedom of a voxel's element on a one-body grid.
pub(crate) fn grid_dofs(problem: &VoxelFemProblem, voxel: usize) -> [usize; 24] {
    let (nnx, nny) = (problem.nx + 1, problem.ny + 1);
    let (i, j, k) = problem.ijk(voxel);
    let node = |i: usize, j: usize, k: usize| (k * nny + j) * nnx + i;
    let nodes = [
        node(i, j, k),
        node(i + 1, j, k),
        node(i + 1, j + 1, k),
        node(i, j + 1, k),
        node(i, j, k + 1),
        node(i + 1, j, k + 1),
        node(i + 1, j + 1, k + 1),
        node(i, j + 1, k + 1),
    ];
    let mut dofs = [0usize; 24];
    for (n, node) in nodes.into_iter().enumerate() {
        for c in 0..3 {
            dofs[3 * n + c] = 3 * node + c;
        }
    }
    dofs
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// The load factor a given shape would buckle at: the Rayleigh quotient (φᵀ K φ) / (φᵀ −Kg φ).
/// The eigen-solve finds the shape that minimises this; handing it a shape is how the solve is
/// checked against a case whose answer is known. 0 when the shape does not fit the problem.
pub fn buckling_rayleigh(problem: &VoxelFemProblem, result: &VoxelFemResult, mode: &[f64]) -> f64 {
    let Some(ops) = Operators::build(problem, result) else { return 0.0 };
    if mode.len() != ops.dof_count {
        return 0.0;
    }
    let shape: Vec<f64> = mode.iter().enumerate().map(|(d, v)| if ops.fixed[d] { 0.0 } else { *v }).collect();
    let mut work = vec![0.0; shape.len()];
    ops.multiply_k(&shape, &mut work);
    let top = dot(&shape, &work);
    ops.multiply_g(&shape, &mut work);
    let bottom = -dot(&shape, &work);
    if bottom.abs() > 1e-300 { top / bottom } else { 0.0 }
}

/// A small deterministic generator for the symmetry-breaking noise (xorshift64*).
struct Noise(u64);

impl Noise {
    fn next(&mut self) -> f64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        let bits = x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11;
        (bits as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    }
}

/// Buckling of a solved problem. `result` must come from `solve_voxels` on the same problem: the
/// stress state it carries is what makes Kg. Twelve iterations is the working default: started
/// from the solved displacement the quotient settles in a handful, and each costs a linear solve.
pub fn buckling(problem: &VoxelFemProblem, result: &VoxelFemResult, max_iterations: usize) -> BucklingResult {
    let mut out = BucklingResult::default();
    let Some(ops) = Operators::build(problem, result) else {
        out.advice = "there is no solved stress state to build the geometric stiffness from".into();
        return out;
    };
    out.load_case = result.worst_case;
    let any_compression = result.voxel_stress_tensor.chunks_exact(6).any(|s| s[0] < 0.0 || s[1] < 0.0 || s[2] < 0.0);
    if !any_compression {
        out.advice = "nothing is in compression, so there is no buckling mode".into();
        return out;
    }
    let dof_count = ops.dof_count;
    let mut x = vec![0.0; dof_count];
    let mut y = vec![0.0; dof_count];
    let mut rhs = vec![0.0; dof_count];
    let mut r = vec![0.0; dof_count];
    let mut p = vec![0.0; dof_count];
    let mut ap = vec![0.0; dof_count];
    // Started from the displacement field, which already leans the way the structure wants to
    // move; a little noise breaks the symmetry so the iteration can find the bending mode rather
    // than a pure translation of the load path.
    let mut noise = Noise(0x9E37_79B9_7F4A_7C15);
    let lean = result.worst_case.and_then(|c| result.case_displacement.get(c)).or_else(|| result.case_displacement.first());
    match lean.filter(|u| u.len() == dof_count) {
        Some(u) => {
            let biggest = u.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            for d in 0..dof_count {
                x[d] = if ops.fixed[d] { 0.0 } else { u[d] + 0.05 * biggest * noise.next() };
            }
        }
        None => {
            for (d, value) in x.iter_mut().enumerate() {
                *value = if ops.fixed[d] { 0.0 } else { noise.next() };
            }
        }
    }
    let mut lambda = 0.0;
    for iteration in 0..max_iterations.max(4) {
        ops.multiply_g(&x, &mut rhs);
        for v in rhs.iter_mut() {
            *v = -*v;
        }
        let rhs_norm = dot(&rhs, &rhs).sqrt();
        if !(rhs_norm > 0.0) {
            out.advice = "nothing is in compression, so there is no buckling mode".into();
            return out;
        }
        // Solve K y = rhs by conjugate gradients.
        y.fill(0.0);
        r.copy_from_slice(&rhs);
        p.copy_from_slice(&rhs);
        let mut rr = dot(&r, &r);
        let rr0 = if rr > 0.0 { rr } else { 1.0 };
        let mut inner = 0;
        while inner < problem.max_iterations && rr > 1e-10 * rr0 {
            ops.multiply_k(&p, &mut ap);
            let p_ap = dot(&p, &ap);
            if !(p_ap.abs() > 1e-300) {
                break;
            }
            let alpha = rr / p_ap;
            for d in 0..dof_count {
                y[d] += alpha * p[d];
                r[d] -= alpha * ap[d];
            }
            let rr_new = dot(&r, &r);
            let beta = rr_new / rr;
            rr = rr_new;
            for d in 0..dof_count {
                p[d] = r[d] + beta * p[d];
            }
            inner += 1;
        }
        let norm = dot(&y, &y).sqrt();
        if !(norm > 0.0) {
            out.advice = "the buckling iteration collapsed".into();
            return out;
        }
        for d in 0..dof_count {
            x[d] = y[d] / norm;
        }
        // Rayleigh quotient.
        ops.multiply_k(&x, &mut ap);
        let top = dot(&x, &ap);
        ops.multiply_g(&x, &mut rhs);
        let bottom = -dot(&x, &rhs);
        out.iterations = iteration + 1;
        if !(bottom.abs() > 1e-300) {
            continue;
        }
        let next = top / bottom;
        let settled = iteration > 2 && (next - lambda).abs() < 1e-3 * next.abs();
        lambda = next;
        if settled {
            break;
        }
    }
    if !(lambda > 0.0) || !lambda.is_finite() {
        out.advice = "no positive buckling mode: the structure is in tension, or the compression it carries cannot fold it".into();
        return out;
    }
    out.solved = true;
    out.load_factor = lambda;
    out.mode = x;

    // What a real member does before it gets there: a member with a small bow deflects by
    // 1 / (1 − 1/λ) times that bow as the load comes on, and the bending stress with it.
    if lambda > 1.0 {
        out.amplification = 1.0 / (1.0 - 1.0 / lambda);
        let length = problem.nx.max(problem.ny).max(problem.nz) as f64 * problem.h;
        out.assumed_imperfection = length / 500.0;
        let depth = problem.nx.min(problem.ny).min(problem.nz) as f64 * problem.h;
        let applied = out.load_case.and_then(|c| problem.cases.get(c)).map(|c| c.force.len()).unwrap_or(0.0);
        if applied > 0.0 && depth > 0.0 {
            // Bending stress P·e / Z, with e the bow once it has grown and Z the section modulus
            // of a square of that depth.
            let modulus = depth.powi(3) / 6.0;
            let bow = out.assumed_imperfection * out.amplification;
            out.second_order_stress = applied * bow / modulus;
        }
    }
    out
}
