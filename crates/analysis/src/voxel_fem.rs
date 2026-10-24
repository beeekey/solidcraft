//! The solve stated on a voxel grid.
//!
//! The caller says which voxels are material, which are clamped, and what pushes where. Each load
//! case is solved on its own: a bracket has to survive the braking load and the cornering load
//! separately, not their sum.

use serde::{Deserialize, Serialize};
use solidcraft_geom::Vec3;

use crate::element::{CORNER, Element};
use crate::material::OrthotropicMaterial;

/// One load case: a total force spread over a set of voxels.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LoadCase {
    /// Dense voxel indices `(k * ny + j) * nx + i` the force acts on.
    pub voxels: Vec<usize>,
    /// Newtons, the total over the set.
    pub force: Vec3,
    pub name: String,
}

/// A problem on a regular grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoxelFemProblem {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    /// Voxel size (m).
    pub h: f64,
    /// Corner of voxel (0, 0, 0).
    pub origin: Vec3,
    /// `nx*ny*nz`, true = material.
    pub solid: Vec<bool>,
    /// `nx*ny*nz`, true = clamped (all three degrees of freedom).
    pub held: Vec<bool>,
    pub cases: Vec<LoadCase>,
    /// Per-element stiffness multiplier for a graded material (a density optimiser's field).
    /// Empty means every element is solid.
    pub element_scale: Vec<f64>,
    /// Which body each voxel belongs to, when the problem has more than one and the interfaces
    /// between them may open. Empty means one body: touching voxels share their nodes and are
    /// bonded. Filled in, interface nodes are duplicated per body and linked by a contact that
    /// pushes but does not pull.
    pub body_of: Vec<usize>,
    /// Friction coefficient at those contacts. 0 lets the parts slide freely; above zero a closed
    /// link also resists sliding up to `mu` times what presses it together, and a link asked for
    /// more slips and is then modelled as carrying no shear (which understates the joint).
    pub contact_friction: f64,
    /// A clamp holding the interfaces shut (N), spread over the links: what a bolt does. Applied as
    /// a self-equilibrating pair of forces across each closed link.
    pub contact_preload: f64,
    /// Voxels whose stress must not count as the peak: a clamp and a point load are singularities
    /// of linear elasticity, so their stress grows without limit as the mesh is refined. It is
    /// still computed and reported in `worst_von_mises_anywhere`.
    pub exclude_from_peak: Vec<bool>,
    /// Gravity as an acceleration; zero switches it off.
    pub gravity: Vec3,
    /// kg/m³, needed only for gravity.
    pub density: f64,
    /// Supports that only push back: fix everything, look at the reactions, release whatever is
    /// being pulled off, solve again until the set stops changing.
    pub no_tension_supports: bool,
    /// Which way "off the support" is; nodes pulled along it let go.
    pub support_normal: Vec3,
    pub material: OrthotropicMaterial,
    pub max_iterations: usize,
    /// Relative residual, |r|² / |f|².
    pub tolerance: f64,
}

impl VoxelFemProblem {
    /// An all-solid, unheld, unloaded grid of `nx × ny × nz` voxels of size `h`.
    pub fn new(nx: usize, ny: usize, nz: usize, h: f64, material: OrthotropicMaterial) -> Self {
        let count = nx * ny * nz;
        Self {
            nx,
            ny,
            nz,
            h,
            origin: Vec3::ZERO,
            solid: vec![true; count],
            held: vec![false; count],
            cases: Vec::new(),
            element_scale: Vec::new(),
            body_of: Vec::new(),
            contact_friction: 0.0,
            contact_preload: 0.0,
            exclude_from_peak: Vec::new(),
            gravity: Vec3::ZERO,
            density: 0.0,
            no_tension_supports: false,
            support_normal: Vec3::Z,
            material,
            max_iterations: 900,
            tolerance: 1e-8,
        }
    }

    /// Dense index of voxel `(i, j, k)`.
    pub fn index(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.ny + j) * self.nx + i
    }

    /// Voxel `(i, j, k)` of a dense index.
    pub fn ijk(&self, voxel: usize) -> (usize, usize, usize) {
        (voxel % self.nx, (voxel / self.nx) % self.ny, voxel / (self.nx * self.ny))
    }

    /// Centre of a voxel (m).
    pub fn centre(&self, voxel: usize) -> Vec3 {
        let (i, j, k) = self.ijk(voxel);
        self.origin + Vec3::new((i as f64 + 0.5) * self.h, (j as f64 + 0.5) * self.h, (k as f64 + 0.5) * self.h)
    }

    fn voxel_count(&self) -> usize {
        self.nx * self.ny * self.nz
    }
}

/// What the contact interfaces did.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ContactReport {
    /// Node pairs across the interfaces.
    pub links: usize,
    /// How many lifted rather than pushed.
    pub open_links: usize,
    /// Active-set passes it took to settle.
    pub rounds: usize,
    /// A body ended up held by nothing: every contact under it opened. There is no answer then.
    pub came_apart: bool,
    /// With friction on: closed links holding, and closed links asked for more shear than the
    /// friction under them can give.
    pub stuck_links: usize,
    pub sliding_links: usize,
    /// Enough of the joint let go that a body is no longer held against the load and slides or
    /// falls rather than reaching a static answer.
    pub slid_free: bool,
}

/// One load case's answer.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct VoxelFemCase {
    pub converged: bool,
    /// Final |r|² / |f|², what `converged` was decided on.
    pub residual_ratio: f64,
    /// f·u (J), the inverse of stiffness.
    pub compliance: f64,
    /// Pa, outside the excluded voxels.
    pub max_von_mises: f64,
    /// Pa, including them.
    pub max_von_mises_anywhere: f64,
    /// Held nodes that let go, when the supports only push back.
    pub lifted_nodes: usize,
    /// m
    pub max_displacement: f64,
}

/// The whole answer.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VoxelFemResult {
    pub contact: ContactReport,
    pub solved: bool,
    /// Why it could not be solved, when it could not.
    pub advice: String,
    pub cases: Vec<VoxelFemCase>,
    /// Governing numbers: maxima over the cases, since a design must survive its worst case.
    pub worst_von_mises: f64,
    pub worst_von_mises_anywhere: f64,
    pub worst_displacement: f64,
    pub worst_compliance: f64,
    /// Index of the governing case, or `None` when no case solved.
    pub worst_case: Option<usize>,
    /// Nodal displacement per load case, three per node (m). On one body the node index is the
    /// grid index `(k * (ny+1) + j) * (nx+1) + i`; with contact, nodes are compacted and
    /// `node_count` says how many there are.
    pub case_displacement: Vec<Vec<f64>>,
    pub node_count: usize,
    /// Per-voxel von Mises stress of the governing case (Pa), dense.
    pub voxel_stress: Vec<f64>,
    /// Per-voxel strain energy of the governing case (J). Summed it is the compliance; per element
    /// it is the sensitivity a topology optimiser needs.
    pub voxel_energy: Vec<f64>,
    /// The full stress tensor of the governing case, six per voxel (xx, yy, zz, xy, yz, zx).
    pub voxel_stress_tensor: Vec<f64>,
    pub elements: usize,
}

/// How a solve's displacement vector is indexed: for one body, the grid node
/// `(k·(ny+1) + j)·(nx+1) + i`; with contact, the compact node of each (grid node, body) pair,
/// `None` where no element of that body touches the node.
#[derive(Debug, Clone)]
pub struct NodeMap {
    nx: usize,
    ny: usize,
    body_count: usize,
    contact: bool,
    node_id: Vec<Option<usize>>,
}

impl NodeMap {
    /// The node of grid node `(i, j, k)` as seen by `body`.
    pub fn node(&self, i: usize, j: usize, k: usize, body: usize) -> Option<usize> {
        let g = (k * (self.ny + 1) + j) * (self.nx + 1) + i;
        if self.contact { self.node_id.get(g * self.body_count + body).copied().flatten() } else { Some(g) }
    }
}

/// The node map [`solve_voxels`] used for `problem`.
pub fn node_map(problem: &VoxelFemProblem) -> NodeMap {
    let elements: Vec<usize> = (0..problem.voxel_count()).filter(|&v| problem.solid.get(v).copied().unwrap_or(false)).collect();
    let topology = Topology::build(problem, elements);
    NodeMap { nx: problem.nx, ny: problem.ny, body_count: topology.body_count, contact: topology.contact, node_id: topology.node_id }
}

/// A link across a contact interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct ContactLink {
    /// The node on the lower side along `axis`.
    node_a: usize,
    /// The node on the higher side.
    node_b: usize,
    /// 0 = x, 1 = y, 2 = z: the interface normal.
    axis: usize,
}

/// The mesh the grid becomes: which compact node each (grid node, body) pair maps to.
struct Topology<'p> {
    problem: &'p VoxelFemProblem,
    elements: Vec<usize>,
    contact: bool,
    body_count: usize,
    /// (grid node * body_count + body) → compact node, when `contact`.
    node_id: Vec<Option<usize>>,
    node_count: usize,
}

impl<'p> Topology<'p> {
    fn build(problem: &'p VoxelFemProblem, elements: Vec<usize>) -> Self {
        let (nx, ny, nz) = (problem.nx, problem.ny, problem.nz);
        let grid_nodes = (nx + 1) * (ny + 1) * (nz + 1);
        let contact = problem.body_of.len() == problem.voxel_count();
        let mut body_count = 1;
        if contact {
            for &voxel in &elements {
                body_count = body_count.max(problem.body_of.get(voxel).copied().unwrap_or(0) + 1);
            }
        }
        let mut topology = Topology { problem, elements, contact, body_count, node_id: Vec::new(), node_count: grid_nodes };
        if contact {
            topology.node_id = vec![None; grid_nodes * body_count];
            topology.node_count = 0;
            let elements = topology.elements.clone();
            for voxel in elements {
                let (i, j, k) = problem.ijk(voxel);
                let body = topology.body_at(voxel);
                for dz in 0..=1 {
                    for dy in 0..=1 {
                        for dx in 0..=1 {
                            let g = ((k + dz) * (ny + 1) + (j + dy)) * (nx + 1) + (i + dx);
                            let slot = g * body_count + body;
                            if let Some(entry) = topology.node_id.get_mut(slot)
                                && entry.is_none()
                            {
                                *entry = Some(topology.node_count);
                                topology.node_count += 1;
                            }
                        }
                    }
                }
            }
        }
        topology
    }

    fn body_at(&self, voxel: usize) -> usize {
        if self.contact { self.problem.body_of.get(voxel).copied().unwrap_or(0) } else { 0 }
    }

    /// Compact node of grid node `(i, j, k)` in `body`; `None` when no element uses that pairing.
    fn node_at(&self, i: usize, j: usize, k: usize, body: usize) -> Option<usize> {
        let g = (k * (self.problem.ny + 1) + j) * (self.problem.nx + 1) + i;
        if self.contact { self.node_id.get(g * self.body_count + body).copied().flatten() } else { Some(g) }
    }

    /// The 24 degrees of freedom of a voxel's element.
    fn element_dofs(&self, voxel: usize) -> [usize; 24] {
        let (i, j, k) = self.problem.ijk(voxel);
        let b = self.body_at(voxel);
        let corners =
            [(i, j, k), (i + 1, j, k), (i + 1, j + 1, k), (i, j + 1, k), (i, j, k + 1), (i + 1, j, k + 1), (i + 1, j + 1, k + 1), (i, j + 1, k + 1)];
        let mut dofs = [0usize; 24];
        for (n, (ci, cj, ck)) in corners.into_iter().enumerate() {
            // Every corner of an element was registered when the topology was built, so the
            // lookup cannot fail; 0 is only a formality for the type.
            let node = self.node_at(ci, cj, ck, b).unwrap_or(0);
            for comp in 0..3 {
                dofs[3 * n + comp] = 3 * node + comp;
            }
        }
        dofs
    }

    /// Every pair of adjacent voxels in different bodies shares a face whose four corners become
    /// four node pairs that may push but not pull.
    fn contact_links(&self) -> Vec<ContactLink> {
        let mut links = Vec::new();
        if !self.contact {
            return links;
        }
        let problem = self.problem;
        let (nx, ny, nz) = (problem.nx, problem.ny, problem.nz);
        let steps = [[1usize, 0, 0], [0, 1, 0], [0, 0, 1]];
        for &voxel in &self.elements {
            let (i, j, k) = problem.ijk(voxel);
            let a = self.body_at(voxel);
            for (axis, step) in steps.iter().enumerate() {
                let (ni, nj, nk) = (i + step[0], j + step[1], k + step[2]);
                if ni >= nx || nj >= ny || nk >= nz {
                    continue;
                }
                let other = problem.index(ni, nj, nk);
                if !problem.solid.get(other).copied().unwrap_or(false) {
                    continue;
                }
                let b = self.body_at(other);
                if a == b {
                    continue;
                }
                for u in 0..=1 {
                    for v in 0..=1 {
                        let mut corner = [i, j, k];
                        corner[axis] += 1; // the shared plane
                        corner[(axis + 1) % 3] += u;
                        corner[(axis + 2) % 3] += v;
                        let node_a = self.node_at(corner[0], corner[1], corner[2], a);
                        let node_b = self.node_at(corner[0], corner[1], corner[2], b);
                        if let (Some(node_a), Some(node_b)) = (node_a, node_b)
                            && node_a != node_b
                        {
                            links.push(ContactLink { node_a, node_b, axis });
                        }
                    }
                }
            }
        }
        links.sort_unstable();
        links.dedup();
        links
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Solves every load case of the problem.
pub fn solve_voxels(problem: &VoxelFemProblem) -> VoxelFemResult {
    let mut result = VoxelFemResult::default();
    let (nx, ny, nz, h) = (problem.nx, problem.ny, problem.nz, problem.h);
    let voxel_count = problem.voxel_count();
    if nx == 0 || ny == 0 || nz == 0 || !(h > 0.0) || !h.is_finite() || problem.solid.len() != voxel_count || problem.held.len() != voxel_count {
        result.advice = "the grid, the masks and the material must all be given".into();
        return result;
    }
    let element = match Element::build(&problem.material, h) {
        Ok(element) => element,
        Err(error) => {
            result.advice = error.to_string();
            return result;
        }
    };
    if problem.cases.is_empty() {
        result.advice = "nothing pushes on it, so there is nothing to solve".into();
        return result;
    }
    let elements: Vec<usize> = (0..voxel_count).filter(|&v| problem.solid[v]).collect();
    result.elements = elements.len();
    if elements.is_empty() {
        result.advice = "no material".into();
        return result;
    }

    let topology = Topology::build(problem, elements);
    let dof_count = topology.node_count * 3;
    result.node_count = topology.node_count;

    let mut fixed_dof = vec![false; dof_count];
    let mut held_nodes = 0usize;
    for &voxel in &topology.elements {
        if !problem.held[voxel] {
            continue;
        }
        let dofs = topology.element_dofs(voxel);
        held_nodes += 8;
        for d in dofs {
            if let Some(slot) = fixed_dof.get_mut(d) {
                *slot = true;
            }
        }
    }
    if held_nodes == 0 {
        // Without a support the stiffness matrix is singular and conjugate gradients wanders off
        // into a rigid-body mode. Saying so beats a large, confident, meaningless displacement.
        result.advice = "nothing holds it: no held voxel carries material".into();
        return result;
    }

    let links = topology.contact_links();
    result.contact.links = links.len();
    // A penalty stiff enough that the interface does not visibly squash and not so stiff that
    // the iteration cannot see past it: an order above one element (whose scale is E·h).
    let contact_stiffness = 10.0 * problem.material.reference() * h;
    let mut link_active = vec![true; links.len()];
    let friction = problem.contact_friction.max(0.0);
    let mut link_sticks = vec![friction > 0.0; links.len()];
    let preload_per_link = if links.is_empty() { 0.0 } else { problem.contact_preload.max(0.0) / links.len() as f64 };

    let e_ref = problem.material.reference();
    let graded = problem.element_scale.len() == voxel_count;
    let stiffness_of = |voxel: usize| if graded { e_ref * problem.element_scale[voxel].max(0.0) } else { e_ref };
    let element_dofs: Vec<[usize; 24]> = topology.elements.iter().map(|&v| topology.element_dofs(v)).collect();

    // K x, element by element.
    let multiply_elements = |x: &[f64], y: &mut [f64]| {
        y.fill(0.0);
        for (index, &voxel) in topology.elements.iter().enumerate() {
            let dofs = &element_dofs[index];
            let scale = stiffness_of(voxel);
            for a in 0..24 {
                let mut sum = 0.0;
                for b in 0..24 {
                    sum += element.ke[a][b] * x[dofs[b]];
                }
                y[dofs[a]] += sum * scale;
            }
        }
    };

    result.voxel_stress = vec![0.0; voxel_count];
    result.voxel_energy = vec![0.0; voxel_count];
    result.voxel_stress_tensor = vec![0.0; voxel_count * 6];
    result.solved = true;

    let mut u = vec![0.0; dof_count];
    let mut f = vec![0.0; dof_count];
    let mut r = vec![0.0; dof_count];
    let mut p = vec![0.0; dof_count];
    let mut ap = vec![0.0; dof_count];
    let mut z = vec![0.0; dof_count];
    let mut inv_diag = vec![0.0; dof_count];
    let any_gravity = problem.gravity.len() > 0.0 && problem.density > 0.0;

    for load in &problem.cases {
        let mut outcome = VoxelFemCase::default();
        f.fill(0.0);
        let mut weight = vec![0.0; topology.node_count];
        let mut total_weight = 0.0;
        for &voxel in &load.voxels {
            if voxel >= voxel_count || !problem.solid[voxel] {
                continue;
            }
            let dofs = topology.element_dofs(voxel);
            for n in 0..8 {
                weight[dofs[3 * n] / 3] += 1.0;
                total_weight += 1.0;
            }
        }
        if total_weight <= 0.0 && !any_gravity {
            // The load has nothing to push on: a candidate that grew no material where the force
            // is applied. That is an answer about the design, not an error.
            result.cases.push(outcome);
            continue;
        }
        for (node, w) in weight.iter().enumerate() {
            if *w <= 0.0 {
                continue;
            }
            let share = w / total_weight;
            f[3 * node] = load.force.x * share;
            f[3 * node + 1] = load.force.y * share;
            f[3 * node + 2] = load.force.z * share;
        }
        // Self-weight: every element's mass shared over its eight nodes, in every load case.
        if any_gravity {
            let element_mass = problem.density * h * h * h;
            let share = problem.gravity * (element_mass / 8.0);
            for (index, &voxel) in topology.elements.iter().enumerate() {
                let scale = if graded { problem.element_scale[voxel].max(0.0) } else { 1.0 };
                let dofs = &element_dofs[index];
                for n in 0..8 {
                    f[dofs[3 * n]] += share.x * scale;
                    f[dofs[3 * n + 1]] += share.y * scale;
                    f[dofs[3 * n + 2]] += share.z * scale;
                }
            }
        }
        // The clamp goes on first and is internal: A is pushed towards B along the normal and B
        // towards A by the same amount.
        if preload_per_link > 0.0 {
            for (l, link) in links.iter().enumerate() {
                if !link_active[l] {
                    continue;
                }
                f[3 * link.node_a + link.axis] += preload_per_link;
                f[3 * link.node_b + link.axis] -= preload_per_link;
            }
        }
        let applied = f.clone();

        let mut released = vec![false; topology.node_count];
        let support_rounds = if problem.no_tension_supports {
            4
        } else if links.is_empty() {
            1
        } else if friction > 0.0 {
            10
        } else {
            6
        };
        let (mut rr, mut rr0) = (0.0, 1.0);
        for round in 0..support_rounds {
            let mut active_dof = fixed_dof.clone();
            if problem.no_tension_supports {
                for (node, gone) in released.iter().enumerate() {
                    if *gone {
                        for c in 0..3 {
                            active_dof[3 * node + c] = false;
                        }
                    }
                }
            }
            let multiply_active = |x: &[f64], y: &mut [f64]| {
                multiply_elements(x, y);
                // The contacts, along their normals only, so the bodies slide freely across each
                // other unless friction makes a link stick.
                for (l, link) in links.iter().enumerate() {
                    if !link_active[l] {
                        continue;
                    }
                    let a = 3 * link.node_a + link.axis;
                    let b = 3 * link.node_b + link.axis;
                    let push = contact_stiffness * (x[a] - x[b]);
                    y[a] += push;
                    y[b] -= push;
                    if !link_sticks[l] {
                        continue;
                    }
                    for t in 1..=2 {
                        let axis = (link.axis + t) % 3;
                        let ta = 3 * link.node_a + axis;
                        let tb = 3 * link.node_b + axis;
                        let hold = contact_stiffness * (x[ta] - x[tb]);
                        y[ta] += hold;
                        y[tb] -= hold;
                    }
                }
                for (d, active) in active_dof.iter().enumerate() {
                    if *active {
                        y[d] = x[d];
                    }
                }
            };
            // Jacobi preconditioner: the diagonal of K, assembled as K is applied. Plain conjugate
            // gradients crawl on a graded field whose near-void elements are a millionth as stiff.
            inv_diag.fill(0.0);
            for (index, &voxel) in topology.elements.iter().enumerate() {
                let dofs = &element_dofs[index];
                let scale = stiffness_of(voxel);
                for a in 0..24 {
                    inv_diag[dofs[a]] += element.ke[a][a] * scale;
                }
            }
            for (l, link) in links.iter().enumerate() {
                if !link_active[l] {
                    continue;
                }
                inv_diag[3 * link.node_a + link.axis] += contact_stiffness;
                inv_diag[3 * link.node_b + link.axis] += contact_stiffness;
                if !link_sticks[l] {
                    continue;
                }
                for t in 1..=2 {
                    let axis = (link.axis + t) % 3;
                    inv_diag[3 * link.node_a + axis] += contact_stiffness;
                    inv_diag[3 * link.node_b + axis] += contact_stiffness;
                }
            }
            for (d, value) in inv_diag.iter_mut().enumerate() {
                *value = if active_dof[d] || !(*value > 0.0) { 1.0 } else { 1.0 / *value };
            }

            f.copy_from_slice(&applied);
            for (d, active) in active_dof.iter().enumerate() {
                if *active {
                    f[d] = 0.0;
                }
            }
            u.fill(0.0);

            // Preconditioned conjugate gradients.
            r.copy_from_slice(&f);
            for d in 0..dof_count {
                z[d] = r[d] * inv_diag[d];
            }
            p.copy_from_slice(&z);
            let mut rz = dot(&r, &z);
            rr = dot(&r, &r);
            rr0 = if rr > 0.0 { rr } else { 1.0 };
            let mut iteration = 0;
            while iteration < problem.max_iterations && rr > problem.tolerance * rr0 {
                multiply_active(&p, &mut ap);
                let p_ap = dot(&p, &ap);
                if !(p_ap.abs() > 1e-300) {
                    break;
                }
                let alpha = rz / p_ap;
                for d in 0..dof_count {
                    u[d] += alpha * p[d];
                    r[d] -= alpha * ap[d];
                }
                for d in 0..dof_count {
                    z[d] = r[d] * inv_diag[d];
                }
                let rz_new = dot(&r, &z);
                let beta = if rz > 0.0 { rz_new / rz } else { 0.0 };
                rz = rz_new;
                // The convergence test stays on the true residual.
                rr = dot(&r, &r);
                for d in 0..dof_count {
                    p[d] = z[d] + beta * p[d];
                }
                iteration += 1;
            }

            // The contacts settle the same way a resting support does: solve, see which ones are
            // asked to pull, let those go, solve again. A set that shrinks until nothing holds the
            // body leaves a singular system, and that is the answer: the joint slid or fell apart.
            if !links.is_empty() && rr > problem.tolerance * rr0 {
                result.contact.slid_free = true;
                break;
            }
            if !links.is_empty() {
                let mut opened = 0;
                for (l, link) in links.iter().enumerate() {
                    if !link_active[l] {
                        continue;
                    }
                    let a = 3 * link.node_a + link.axis;
                    let b = 3 * link.node_b + link.axis;
                    // node_b sits on the far side along the normal, so B moving further along it
                    // than A is the gap opening.
                    if u[b] - u[a] > 0.0 {
                        link_active[l] = false;
                        opened += 1;
                    }
                }
                let mut slipped = 0;
                if friction > 0.0 {
                    for (l, link) in links.iter().enumerate() {
                        if !link_active[l] || !link_sticks[l] {
                            continue;
                        }
                        let a = 3 * link.node_a + link.axis;
                        let b = 3 * link.node_b + link.axis;
                        let normal = contact_stiffness * (u[a] - u[b]);
                        let mut shear = 0.0;
                        for t in 1..=2 {
                            let axis = (link.axis + t) % 3;
                            let slide = contact_stiffness * (u[3 * link.node_a + axis] - u[3 * link.node_b + axis]);
                            shear += slide * slide;
                        }
                        let shear = shear.sqrt();
                        if !(normal > 0.0) || shear > friction * normal {
                            link_sticks[l] = false;
                            slipped += 1;
                        }
                    }
                }
                result.contact.rounds += 1;
                if (opened > 0 || slipped > 0) && round + 1 < support_rounds {
                    continue;
                }
            }
            if !problem.no_tension_supports {
                break;
            }

            // Reactions at the held nodes: a support that has to pull is not a support, so that
            // node lets go and the next round is solved without it.
            let mut reaction = vec![0.0; dof_count];
            multiply_elements(&u, &mut reaction);
            let mut changed = false;
            let n = problem.support_normal;
            for node in 0..topology.node_count {
                if released[node] || !fixed_dof[3 * node] {
                    continue;
                }
                let along = reaction[3 * node] * n.x + reaction[3 * node + 1] * n.y + reaction[3 * node + 2] * n.z;
                if along < 0.0 {
                    released[node] = true;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        if problem.no_tension_supports {
            outcome.lifted_nodes = released.iter().filter(|g| **g).count();
        }
        if !links.is_empty() {
            let open = link_active.iter().filter(|a| !**a).count();
            result.contact.open_links = result.contact.open_links.max(open);
            if friction > 0.0 {
                let stuck = links.iter().enumerate().filter(|(l, _)| link_active[*l] && link_sticks[*l]).count();
                let sliding = links.iter().enumerate().filter(|(l, _)| link_active[*l] && !link_sticks[*l]).count();
                result.contact.stuck_links = result.contact.stuck_links.max(stuck);
                result.contact.sliding_links = result.contact.sliding_links.max(sliding);
            }
            if open == links.len() {
                result.contact.came_apart = true;
            }
        }
        outcome.converged = rr <= problem.tolerance * rr0;
        outcome.residual_ratio = if rr0 > 0.0 { rr / rr0 } else { 1.0 };
        outcome.compliance = dot(&f, &u);
        for node in 0..topology.node_count {
            let d = Vec3::new(u[3 * node], u[3 * node + 1], u[3 * node + 2]).len();
            outcome.max_displacement = outcome.max_displacement.max(d);
        }

        let mut case_stress = vec![0.0; voxel_count];
        let mut case_energy = vec![0.0; voxel_count];
        let mut case_tensor = vec![0.0; voxel_count * 6];
        let exclude = problem.exclude_from_peak.len() == voxel_count;
        for (index, &voxel) in topology.elements.iter().enumerate() {
            let dofs = &element_dofs[index];
            let local_e = stiffness_of(voxel);
            // Strain energy: half uᵀ Ke u with the element's own share of the stiffness.
            let mut energy = 0.0;
            for a in 0..24 {
                let mut sum = 0.0;
                for b in 0..24 {
                    sum += element.ke[a][b] * u[dofs[b]];
                }
                energy += u[dofs[a]] * sum;
            }
            case_energy[voxel] = 0.5 * local_e * energy;

            let mut strain = [0.0; 6];
            for (n, corner) in CORNER.iter().enumerate() {
                let dx = 0.125 * corner[0] * (2.0 / h);
                let dy = 0.125 * corner[1] * (2.0 / h);
                let dz = 0.125 * corner[2] * (2.0 / h);
                let (ux, uy, uz) = (u[dofs[3 * n]], u[dofs[3 * n + 1]], u[dofs[3 * n + 2]]);
                strain[0] += dx * ux;
                strain[1] += dy * uy;
                strain[2] += dz * uz;
                strain[3] += dy * ux + dx * uy;
                strain[4] += dz * uy + dy * uz;
                strain[5] += dz * ux + dx * uz;
            }
            let mut stress = [0.0; 6];
            for (a, value) in stress.iter_mut().enumerate() {
                for (b, strain_b) in strain.iter().enumerate() {
                    *value += local_e * element.c[a][b] * strain_b;
                }
            }
            let von_mises = (0.5 * ((stress[0] - stress[1]).powi(2) + (stress[1] - stress[2]).powi(2) + (stress[2] - stress[0]).powi(2))
                + 3.0 * (stress[3] * stress[3] + stress[4] * stress[4] + stress[5] * stress[5]))
                .max(0.0)
                .sqrt();
            case_stress[voxel] = von_mises;
            case_tensor[voxel * 6..voxel * 6 + 6].copy_from_slice(&stress);
            outcome.max_von_mises_anywhere = outcome.max_von_mises_anywhere.max(von_mises);
            if exclude && problem.exclude_from_peak[voxel] {
                continue;
            }
            outcome.max_von_mises = outcome.max_von_mises.max(von_mises);
        }
        result.worst_von_mises_anywhere = result.worst_von_mises_anywhere.max(outcome.max_von_mises_anywhere);
        if outcome.max_von_mises > result.worst_von_mises || result.worst_case.is_none() {
            result.worst_von_mises = outcome.max_von_mises;
            result.worst_case = Some(result.cases.len());
            result.voxel_stress = case_stress;
            result.voxel_energy = case_energy;
            result.voxel_stress_tensor = case_tensor;
        }
        result.worst_displacement = result.worst_displacement.max(outcome.max_displacement);
        result.worst_compliance = result.worst_compliance.max(outcome.compliance);
        result.case_displacement.push(u.clone());
        result.cases.push(outcome);
    }
    result
}
