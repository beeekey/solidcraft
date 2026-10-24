//! Flow and heat: what the air or coolant actually does around a part.
//!
//! A lattice Boltzmann method (D3Q19, BGK) for incompressible flow on the same voxel grid the
//! stress solver uses, with the part as a no-slip obstacle. LBM suits this: the geometry arrives
//! as a voxel mask, walls are handled by bounce-back rather than a body-conforming mesh, and
//! every step is local. Heat rides on top: once the flow is steady, a steady advection-diffusion
//! equation is solved on the same grid with the fluid's diffusivity where there is fluid and the
//! solid's where there is part, so conduction through the metal and convection into the coolant
//! are one system. That is conjugate heat transfer.
//!
//! What it is for: pressure drop through a channel, drag on a shape, how much heat a flow carries
//! away. What it is not: a turbulence model. Everything is laminar, and the solver says so when
//! the Reynolds number leaves that range, carrying the unresolved motion as a Smagorinsky eddy
//! viscosity rather than reporting a confident number from physics it is not modelling.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Aabb3, Mesh, Vec3};

use crate::voxelise::{MM, VoxelGrid, voxelise};

// --- D3Q19: one direction at rest, six through the faces, twelve through the edges -----------
const Q: usize = 19;
const C: [[i32; 3]; Q] = [
    [0, 0, 0],
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, -1, 0],
    [0, 0, 1],
    [0, 0, -1],
    [1, 1, 0],
    [-1, -1, 0],
    [1, -1, 0],
    [-1, 1, 0],
    [1, 0, 1],
    [-1, 0, -1],
    [1, 0, -1],
    [-1, 0, 1],
    [0, 1, 1],
    [0, -1, -1],
    [0, 1, -1],
    [0, -1, 1],
];
const W: [f64; Q] = [
    1.0 / 3.0,
    1.0 / 18.0,
    1.0 / 18.0,
    1.0 / 18.0,
    1.0 / 18.0,
    1.0 / 18.0,
    1.0 / 18.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
    1.0 / 36.0,
];
/// The direction that undoes each one, for bounce-back off a wall.
const OPPOSITE: [usize; Q] = [0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17];
/// Lattice speed of sound, squared.
const CS2: f64 = 1.0 / 3.0;

/// The equilibrium distribution at this density and velocity, to second order in u.
#[inline]
fn equilibrium(q: usize, rho: f64, ux: f64, uy: f64, uz: f64) -> f64 {
    let cu = C[q][0] as f64 * ux + C[q][1] as f64 * uy + C[q][2] as f64 * uz;
    let u2 = ux * ux + uy * uy + uz * uz;
    W[q] * rho * (1.0 + cu / CS2 + cu * cu / (2.0 * CS2 * CS2) - u2 / (2.0 * CS2))
}

/// Where the fluid is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FlowRegion {
    /// Through the part: the fluid fills the void the part encloses, grown from the inlet.
    Internal,
    /// Around the part: the part sits in a channel and the fluid flows past.
    External,
}

/// A body already on a grid rather than a mesh to be classified (a generative search hands the
/// mask over). The grid is the domain: it is never padded.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoxelBody {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    /// m
    pub h: f64,
    /// m
    pub origin: Vec3,
    pub solid: Vec<bool>,
}

/// The coolants on offer, in one place.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FluidProperties {
    pub name: &'static str,
    /// kg/m³
    pub density: f64,
    /// Pa·s, dynamic
    pub viscosity: f64,
    /// W/m·K
    pub conductivity: f64,
    /// J/kg·K
    pub heat_capacity: f64,
}

pub const FLUIDS: [FluidProperties; 4] = [
    FluidProperties { name: "Water", density: 998.0, viscosity: 1.002e-3, conductivity: 0.598, heat_capacity: 4182.0 },
    FluidProperties { name: "Air", density: 1.204, viscosity: 1.813e-5, conductivity: 0.0257, heat_capacity: 1005.0 },
    FluidProperties { name: "Oil (ISO VG 46)", density: 870.0, viscosity: 0.040, conductivity: 0.145, heat_capacity: 1900.0 },
    FluidProperties { name: "Ethylene glycol 50%", density: 1070.0, viscosity: 3.8e-3, conductivity: 0.38, heat_capacity: 3300.0 },
];

/// The named fluid (case-insensitive), or water when the name is not one of them.
pub fn fluid_named(name: &str) -> FluidProperties {
    FLUIDS.iter().find(|f| f.name.eq_ignore_ascii_case(name.trim())).cloned().unwrap_or_else(|| FLUIDS[0].clone())
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FluidProblem {
    pub region: FlowRegion,
    /// Roughly where the fluid enters and leaves (m): each picks the nearest face of the flow
    /// domain, which becomes the inlet or outlet plane.
    pub inlet_point: Vec3,
    pub outlet_point: Vec3,
    /// m/s, uniform over the inlet.
    pub inlet_velocity: f64,
    /// kg/m³ and Pa·s (water at 20 °C by default).
    pub density: f64,
    pub viscosity: f64,
    /// The box the fluid occupies for external flow (m). None pads the part's own bounds, which
    /// is a guess: a wing needs far more room behind it than beside it.
    pub domain: Option<Aabb3>,
    /// Cells across the longest side.
    pub resolution: usize,
    pub max_steps: usize,
    /// Relative change in velocity per batch of steps.
    pub tolerance: f64,
    // --- heat ---
    pub solve_heat: bool,
    /// K
    pub inlet_temperature: f64,
    /// Held temperature of the solid (K), used when `heat_power` is zero.
    pub wall_temperature: f64,
    /// Total power dissipated in the solid (W). When positive the solid's temperature is solved
    /// rather than held: the conjugate case.
    pub heat_power: f64,
    pub fluid_conductivity: f64,
    pub fluid_heat_capacity: f64,
    pub solid_conductivity: f64,
    pub solid_density: f64,
    pub solid_heat_capacity: f64,
}

impl Default for FluidProblem {
    fn default() -> Self {
        FluidProblem {
            region: FlowRegion::Internal,
            inlet_point: Vec3::ZERO,
            outlet_point: Vec3::X,
            inlet_velocity: 1.0,
            density: 998.0,
            viscosity: 1.002e-3,
            domain: None,
            resolution: 40,
            max_steps: 20_000,
            tolerance: 1e-5,
            solve_heat: false,
            inlet_temperature: 293.15,
            wall_temperature: 353.15,
            heat_power: 0.0,
            fluid_conductivity: 0.598,
            fluid_heat_capacity: 4182.0,
            solid_conductivity: 205.0,
            solid_density: 2700.0,
            solid_heat_capacity: 900.0,
        }
    }
}

/// The most cells a flow grid may hold.
pub const MAX_FLOW_CELLS: usize = 3_000_000;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FluidResult {
    pub converged: bool,
    pub steps: usize,
    pub residual: f64,
    pub grid: [usize; 3],
    /// Cell size (m) and the grid's origin (m).
    pub h: f64,
    pub origin: Vec3,
    pub fluid: Vec<bool>,
    pub solid: Vec<bool>,
    /// m/s per cell.
    pub velocity_x: Vec<f64>,
    pub velocity_y: Vec<f64>,
    pub velocity_z: Vec<f64>,
    /// Pa, gauge, per cell.
    pub pressure: Vec<f64>,
    /// K per cell, when heat was solved.
    pub temperature: Vec<f64>,
    pub max_velocity: f64,
    /// What the inlet actually carried (m/s).
    pub mean_inlet_velocity: f64,
    /// Pa, inlet to outlet.
    pub pressure_drop: f64,
    /// m³/s
    pub volume_flow_rate: f64,
    pub reynolds: f64,
    /// N on the part, by momentum exchange.
    pub drag_force: Vec3,
    pub heat_solved: bool,
    pub inlet_temperature: f64,
    /// K, flow-weighted mean.
    pub outlet_temperature: f64,
    pub max_temperature: f64,
    /// W carried out by the fluid.
    pub heat_removed: f64,
    /// Empty when the answer can be taken at face value; otherwise what is wrong with it.
    pub advice: String,
    /// Which axis the flow runs along (0 = x, 1 = y, 2 = z) and which way (+1 or −1).
    pub flow_axis: usize,
    pub flow_sign: f64,
    /// Cells across the narrowest part of the passage (or across the body, for external flow).
    /// Below about eight the profile cannot be resolved and the pressure drop is wrong.
    pub cells_across_passage: usize,
}

/// Why a flow could not be set up.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum FluidError {
    #[error("need a body to flow around or through")]
    NoBody,
    #[error("the voxel body's grid and mask disagree")]
    BadVoxelBody,
    #[error("need a positive velocity, density and viscosity")]
    BadFluid,
    #[error("the fluid domain has no volume")]
    EmptyDomain,
    #[error("the inlet and the outlet are the same face; put them on opposite sides of the part")]
    SameFace,
    #[error("the grid would hold more than {MAX_FLOW_CELLS} cells")]
    TooFine,
}

/// Which face of a box a point is nearest: axis (0, 1, 2) and side (false = low, true = high).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Face {
    axis: usize,
    high: bool,
}

fn nearest_face(min: Vec3, max: Vec3, point: Vec3) -> Face {
    let distances = [
        (point.x - min.x).abs(),
        (point.x - max.x).abs(),
        (point.y - min.y).abs(),
        (point.y - max.y).abs(),
        (point.z - min.z).abs(),
        (point.z - max.z).abs(),
    ];
    let mut best = 0;
    for (i, d) in distances.iter().enumerate() {
        if *d < distances[best] {
            best = i;
        }
    }
    Face { axis: best / 2, high: best % 2 == 1 }
}

/// The body as the solver wants it: a grid with the part marked.
pub enum FlowBody<'a> {
    /// A closed mesh in mm; the solver chooses the grid.
    Mesh(&'a Mesh),
    /// A mask already on a grid; the grid is the domain.
    Voxels(&'a VoxelBody),
}

/// Solves the flow, and the temperature field when `solve_heat` is set.
pub fn fluid_solve(body: FlowBody<'_>, problem: &FluidProblem) -> Result<FluidResult, FluidError> {
    let mut result = FluidResult::default();
    if !(problem.inlet_velocity > 0.0) || !(problem.density > 0.0) || !(problem.viscosity > 0.0) || !problem.inlet_velocity.is_finite() {
        return Err(FluidError::BadFluid);
    }
    if !problem.inlet_point.is_finite() || !problem.outlet_point.is_finite() {
        return Err(FluidError::BadFluid);
    }
    // --- the domain and the grid ---
    let (nx, ny, nz, h, origin, solid, body_min, body_max): (usize, usize, usize, f64, Vec3, Vec<bool>, Vec3, Vec3) = match body {
        FlowBody::Voxels(given) => {
            let cells = given.nx.checked_mul(given.ny).and_then(|c| c.checked_mul(given.nz)).ok_or(FluidError::BadVoxelBody)?;
            if given.nx == 0
                || given.ny == 0
                || given.nz == 0
                || !(given.h > 0.0)
                || !given.h.is_finite()
                || given.solid.len() != cells
                || !given.origin.is_finite()
            {
                return Err(FluidError::BadVoxelBody);
            }
            if cells > MAX_FLOW_CELLS {
                return Err(FluidError::TooFine);
            }
            let (mut lo, mut hi) = ([given.nx, given.ny, given.nz], [0usize; 3]);
            let mut any = false;
            for (v, s) in given.solid.iter().enumerate() {
                if !*s {
                    continue;
                }
                any = true;
                let at = [v % given.nx, (v / given.nx) % given.ny, v / (given.nx * given.ny)];
                for axis in 0..3 {
                    lo[axis] = lo[axis].min(at[axis]);
                    hi[axis] = hi[axis].max(at[axis]);
                }
            }
            let (bmin, bmax) = if any {
                (
                    given.origin + Vec3::new(lo[0] as f64, lo[1] as f64, lo[2] as f64) * given.h,
                    given.origin + Vec3::new(hi[0] as f64 + 1.0, hi[1] as f64 + 1.0, hi[2] as f64 + 1.0) * given.h,
                )
            } else {
                (given.origin, given.origin)
            };
            (given.nx, given.ny, given.nz, given.h, given.origin, given.solid.clone(), bmin, bmax)
        }
        FlowBody::Mesh(mesh) => {
            if mesh.triangles.is_empty() {
                return Err(FluidError::NoBody);
            }
            let bounds = mesh.bounds();
            if bounds.is_empty() {
                return Err(FluidError::NoBody);
            }
            let (bmin, bmax) = (bounds.min / MM, bounds.max / MM);
            let (mut dmin, mut dmax) = (bmin, bmax);
            if let Some(domain) = &problem.domain {
                let size = domain.max - domain.min;
                if !(size.x > 0.0 && size.y > 0.0 && size.z > 0.0) || !size.is_finite() {
                    return Err(FluidError::EmptyDomain);
                }
                dmin = domain.min;
                dmax = domain.max;
            } else if problem.region == FlowRegion::External {
                // Room for the flow to develop before the part and recover after it.
                let size = bmax - bmin;
                let pad = Vec3::new(size.x * 0.75, size.y * 0.5, size.z * 0.5);
                dmin = bmin - pad;
                dmax = bmax + pad;
            }
            let resolution = problem.resolution.clamp(8, 160);
            let size = dmax - dmin;
            let longest = size.x.max(size.y).max(size.z);
            if !(longest > 0.0) {
                return Err(FluidError::EmptyDomain);
            }
            let h = longest / resolution as f64;
            let cells_along = |extent: f64| ((extent / h).round() as usize).max(3);
            let (nx, ny, nz) = (cells_along(size.x), cells_along(size.y), cells_along(size.z));
            let cells = nx.checked_mul(ny).and_then(|c| c.checked_mul(nz)).ok_or(FluidError::TooFine)?;
            if cells > MAX_FLOW_CELLS {
                return Err(FluidError::TooFine);
            }
            // Classify the cell centres against the mesh (mm).
            let index = mesh.inside_index();
            let mut solid = vec![false; cells];
            for k in 0..nz {
                for j in 0..ny {
                    for i in 0..nx {
                        let centre = (dmin + Vec3::new(i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5) * h) * MM;
                        solid[(k * ny + j) * nx + i] = index.contains(centre);
                    }
                }
            }
            (nx, ny, nz, h, dmin, solid, bmin, bmax)
        }
    };
    let cells = nx * ny * nz;
    let domain_max = origin + Vec3::new(nx as f64, ny as f64, nz as f64) * h;
    result.grid = [nx, ny, nz];
    result.h = h;
    result.origin = origin;
    result.solid = solid;
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    let extent = |axis: usize| [nx, ny, nz][axis];

    let inlet = nearest_face(origin, domain_max, problem.inlet_point);
    let outlet = nearest_face(origin, domain_max, problem.outlet_point);
    if inlet == outlet {
        return Err(FluidError::SameFace);
    }

    // --- solid / fluid ---
    let mut fluid = vec![false; cells];
    if problem.region == FlowRegion::External {
        for (cell, solid) in fluid.iter_mut().zip(&result.solid) {
            *cell = !solid;
        }
    } else {
        // Internal flow is what the inlet can reach: grown from the inlet face, which keeps a
        // drilled passage and leaves the air outside the part out of it.
        let mut queue: VecDeque<usize> = VecDeque::new();
        let push = |i: i64, j: i64, k: i64, fluid: &mut Vec<bool>, queue: &mut VecDeque<usize>| {
            if i < 0 || j < 0 || k < 0 || i as usize >= nx || j as usize >= ny || k as usize >= nz {
                return;
            }
            let v = at(i as usize, j as usize, k as usize);
            if result.solid[v] || fluid[v] {
                return;
            }
            fluid[v] = true;
            queue.push_back(v);
        };
        let s = if inlet.high { extent(inlet.axis) as i64 - 1 } else { 0 };
        let (ea, eb) = match inlet.axis {
            0 => (ny, nz),
            1 => (nx, nz),
            _ => (nx, ny),
        };
        for a in 0..ea as i64 {
            for b in 0..eb as i64 {
                match inlet.axis {
                    0 => push(s, a, b, &mut fluid, &mut queue),
                    1 => push(a, s, b, &mut fluid, &mut queue),
                    _ => push(a, b, s, &mut fluid, &mut queue),
                }
            }
        }
        while let Some(v) = queue.pop_front() {
            let (i, j, k) = ((v % nx) as i64, ((v / nx) % ny) as i64, (v / (nx * ny)) as i64);
            push(i + 1, j, k, &mut fluid, &mut queue);
            push(i - 1, j, k, &mut fluid, &mut queue);
            push(i, j + 1, k, &mut fluid, &mut queue);
            push(i, j - 1, k, &mut fluid, &mut queue);
            push(i, j, k + 1, &mut fluid, &mut queue);
            push(i, j, k - 1, &mut fluid, &mut queue);
        }
    }
    let fluid_cells = fluid.iter().filter(|f| **f).count();
    result.fluid = fluid;
    if fluid_cells < 27 {
        result.advice = "the inlet reaches almost no open space: check the inlet point, the flow region, or raise the resolution".into();
        return Ok(result);
    }
    let fluid = &result.fluid;

    // How many cells span the narrowest part of the passage, along the two axes across the flow.
    let flow_axis = inlet.axis;
    result.flow_axis = flow_axis;
    result.flow_sign = if inlet.high { -1.0 } else { 1.0 };
    {
        let mut narrowest = usize::MAX;
        for axis in 0..3 {
            if axis == flow_axis {
                continue;
            }
            let (mut run, mut best) = (0, 0);
            for s in 0..extent(axis) {
                let i = if axis == 0 { s } else { nx / 2 };
                let j = if axis == 1 { s } else { ny / 2 };
                let k = if axis == 2 { s } else { nz / 2 };
                if fluid[at(i, j, k)] {
                    run += 1;
                    best = best.max(run);
                } else {
                    run = 0;
                }
            }
            if best > 0 {
                narrowest = narrowest.min(best);
            }
        }
        result.cells_across_passage = if narrowest == usize::MAX { 0 } else { narrowest };
    }

    // --- lattice units ---
    // Inside a passage the Reynolds length is the passage width; around a body it is the body
    // itself along the flow, not the tunnel drawn around it.
    let mut characteristic = result.cells_across_passage.max(1) as f64 * h;
    if problem.region == FlowRegion::External {
        let body_size = body_max - body_min;
        characteristic = [body_size.x, body_size.y, body_size.z][flow_axis].max(h);
        result.cells_across_passage = (characteristic / h).round() as usize;
    }
    result.reynolds = problem.density * problem.inlet_velocity * characteristic / problem.viscosity;
    // The lattice velocity is a free choice and decides the relaxation time τ = 3 u N / Re + ½.
    // Pick the velocity that puts τ near 0.6, then clamp it to the range where LBM is honest.
    let passage = result.cells_across_passage.max(1) as f64;
    let wanted = 0.1 * result.reynolds.max(1e-6) / (3.0 * passage);
    let u_lattice = wanted.clamp(0.005, 0.1);
    let nu_lattice = u_lattice * passage / result.reynolds.max(1e-6);
    let mut tau = 3.0 * nu_lattice + 0.5;
    let mut advice = String::new();
    // Below about 0.51 plain BGK is unstable. The subgrid model carries the unresolved motion as
    // an extra viscosity from the local strain rate (Smagorinsky), so a hard-sheared cell relaxes
    // more slowly and the solve stays standing. It models what the grid cannot see; it does not
    // predict a turbulent pressure drop.
    let subgrid = tau < 0.55;
    if subgrid {
        advice.push_str(&format!(
            "Reynolds number {} is past what this grid resolves, so the unresolved motion is carried by a subgrid eddy viscosity (Smagorinsky). Treat the pressure drop as indicative and raise the resolution if it matters. ",
            result.reynolds as i64
        ));
        tau = tau.max(0.51);
    }
    if result.reynolds > 2300.0 {
        advice.push_str(&format!(
            "Reynolds number {} is past the laminar range; the subgrid model keeps the solve standing but does not predict a turbulent pressure drop, which will be higher. ",
            result.reynolds as i64
        ));
    }
    if result.cells_across_passage < 8 {
        advice.push_str(&format!(
            "only {} cells across the passage: the velocity profile cannot be resolved and the pressure drop is unreliable, use a higher resolution. ",
            result.cells_across_passage
        ));
    }
    let dt = u_lattice * h / problem.inlet_velocity; // seconds per step
    let omega = 1.0 / tau;

    // --- the lattice ---
    let mut f = vec![0.0; cells * Q];
    let mut f_new = vec![0.0; cells * Q];
    let mut rho = vec![1.0; cells];
    let mut ux = vec![0.0; cells];
    let mut uy = vec![0.0; cells];
    let mut uz = vec![0.0; cells];
    let inlet_sign = if inlet.high { -1.0 } else { 1.0 };
    let mut inlet_vector = [0.0; 3];
    inlet_vector[flow_axis] = inlet_sign * u_lattice;
    for v in 0..cells {
        if !fluid[v] {
            continue;
        }
        ux[v] = inlet_vector[0];
        uy[v] = inlet_vector[1];
        uz[v] = inlet_vector[2];
        for q in 0..Q {
            f[v * Q + q] = equilibrium(q, 1.0, ux[v], uy[v], uz[v]);
        }
    }
    let on_face = |face: Face, i: usize, j: usize, k: usize| {
        let s = [i, j, k][face.axis];
        if face.high { s + 1 == extent(face.axis) } else { s == 0 }
    };
    let plane_cells = |face: Face| -> Vec<usize> {
        let s = if face.high { extent(face.axis) - 1 } else { 0 };
        let mut out = Vec::new();
        let (ea, eb) = match face.axis {
            0 => (ny, nz),
            1 => (nx, nz),
            _ => (nx, ny),
        };
        for a in 0..ea {
            for b in 0..eb {
                let (i, j, k) = match face.axis {
                    0 => (s, a, b),
                    1 => (a, s, b),
                    _ => (a, b, s),
                };
                out.push(at(i, j, k));
            }
        }
        out
    };
    let inlet_cells = plane_cells(inlet);
    let outlet_cells = plane_cells(outlet);

    // One step of the lattice: collide, stream, then rebuild the inlet and outlet planes.
    let step_lattice = |f: &mut Vec<f64>, f_new: &mut Vec<f64>, rho: &mut Vec<f64>, ux: &mut Vec<f64>, uy: &mut Vec<f64>, uz: &mut Vec<f64>| {
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let v = at(i, j, k);
                    if !fluid[v] {
                        continue;
                    }
                    let (mut r, mut mx, mut my, mut mz) = (0.0, 0.0, 0.0, 0.0);
                    for q in 0..Q {
                        let value = f[v * Q + q];
                        r += value;
                        mx += value * C[q][0] as f64;
                        my += value * C[q][1] as f64;
                        mz += value * C[q][2] as f64;
                    }
                    if !(r > 1e-9) {
                        r = 1.0;
                    }
                    let (vx, vy, vz) = (mx / r, my / r, mz / r);
                    rho[v] = r;
                    ux[v] = vx;
                    uy[v] = vy;
                    uz[v] = vz;
                    // The local relaxation rate: with the subgrid model, the momentum flux of the
                    // non-equilibrium part measures how hard this cell is sheared, and
                    // τ_total = (τ + √(τ² + 18 C² |Q| / ρ)) / 2, which is τ where the flow is smooth.
                    let mut local_omega = omega;
                    if subgrid {
                        let (mut qxx, mut qyy, mut qzz, mut qxy, mut qyz, mut qzx) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                        for q in 0..Q {
                            let neq = f[v * Q + q] - equilibrium(q, r, vx, vy, vz);
                            let (cx, cy, cz) = (C[q][0] as f64, C[q][1] as f64, C[q][2] as f64);
                            qxx += neq * cx * cx;
                            qyy += neq * cy * cy;
                            qzz += neq * cz * cz;
                            qxy += neq * cx * cy;
                            qyz += neq * cy * cz;
                            qzx += neq * cz * cx;
                        }
                        let magnitude = (qxx * qxx + qyy * qyy + qzz * qzz + 2.0 * (qxy * qxy + qyz * qyz + qzx * qzx)).sqrt();
                        let smagorinsky = 0.16;
                        let effective = (tau + (tau * tau + 18.0 * smagorinsky * smagorinsky * magnitude / r.max(1e-9)).sqrt()) / 2.0;
                        local_omega = 1.0 / effective.max(tau);
                    }
                    for q in 0..Q {
                        let eq = equilibrium(q, r, vx, vy, vz);
                        let post = f[v * Q + q] + local_omega * (eq - f[v * Q + q]);
                        let (ni, nj, nk) = (i as i64 + C[q][0] as i64, j as i64 + C[q][1] as i64, k as i64 + C[q][2] as i64);
                        if ni < 0 || nj < 0 || nk < 0 || ni as usize >= nx || nj as usize >= ny || nk as usize >= nz {
                            // Leaves the box: bounce back unless it left through the outlet.
                            if on_face(outlet, i, j, k) {
                                continue;
                            }
                            f_new[v * Q + OPPOSITE[q]] = post;
                            continue;
                        }
                        let n = at(ni as usize, nj as usize, nk as usize);
                        if !fluid[n] {
                            f_new[v * Q + OPPOSITE[q]] = post; // no-slip wall
                        } else {
                            f_new[n * Q + q] = post;
                        }
                    }
                }
            }
        }
        std::mem::swap(f, f_new);
        let mut rebuild_plane = |face: Face, is_inlet: bool, plane: &[usize]| {
            let step: i64 = if face.high { -1 } else { 1 };
            for &v in plane {
                if !fluid[v] {
                    continue;
                }
                let (i, j, k) = (v % nx, (v / nx) % ny, v / (nx * ny));
                let mut ijk = [i as i64, j as i64, k as i64];
                ijk[face.axis] += step;
                if ijk.iter().any(|c| *c < 0) || ijk[0] as usize >= nx || ijk[1] as usize >= ny || ijk[2] as usize >= nz {
                    continue;
                }
                let n = at(ijk[0] as usize, ijk[1] as usize, ijk[2] as usize);
                if !fluid[n] {
                    continue;
                }
                let (mut rn, mut mx, mut my, mut mz) = (0.0, 0.0, 0.0, 0.0);
                for q in 0..Q {
                    let value = f[n * Q + q];
                    rn += value;
                    mx += value * C[q][0] as f64;
                    my += value * C[q][1] as f64;
                    mz += value * C[q][2] as f64;
                }
                if !(rn > 1e-9) {
                    rn = 1.0;
                }
                let (nx0, ny0, nz0) = (mx / rn, my / rn, mz / rn);
                let target_rho = if is_inlet { rn } else { 1.0 };
                let (tx, ty, tz) = if is_inlet { (inlet_vector[0], inlet_vector[1], inlet_vector[2]) } else { (nx0, ny0, nz0) };
                for q in 0..Q {
                    let neq = f[n * Q + q] - equilibrium(q, rn, nx0, ny0, nz0);
                    f[v * Q + q] = equilibrium(q, target_rho, tx, ty, tz) + neq;
                }
                rho[v] = target_rho;
                ux[v] = tx;
                uy[v] = ty;
                uz[v] = tz;
            }
        };
        rebuild_plane(inlet, true, &inlet_cells);
        rebuild_plane(outlet, false, &outlet_cells);
    };

    // --- time stepping, checked every fifty steps ---
    const BATCH: usize = 50;
    let mut previous = vec![0.0; cells];
    let mut step = 0;
    let mut unstable = false;
    while step < problem.max_steps {
        let take = BATCH.min(problem.max_steps - step);
        for _ in 0..take {
            step_lattice(&mut f, &mut f_new, &mut rho, &mut ux, &mut uy, &mut uz);
        }
        step += take;
        let (mut change, mut magnitude) = (0.0, 0.0);
        for v in 0..cells {
            if !fluid[v] {
                continue;
            }
            let speed = (ux[v] * ux[v] + uy[v] * uy[v] + uz[v] * uz[v]).sqrt();
            change += (speed - previous[v]).abs();
            magnitude += speed;
            previous[v] = speed;
        }
        // Check the sums themselves: a diverging solve makes both NaN, and NaN > 0 is false.
        let finite = magnitude.is_finite() && change.is_finite();
        result.residual = if finite && magnitude > 0.0 { change / magnitude } else { 1.0 };
        result.steps = step;
        if !finite {
            advice.push_str(&format!("the solve went unstable at step {step}. "));
            if problem.region == FlowRegion::External && result.cells_across_passage < 20 {
                advice.push_str(&format!(
                    "The body is only {} cells across in a domain many times its size, so almost none of the mesh is on the part. Raise the resolution, or draw a tighter fluid domain. ",
                    result.cells_across_passage
                ));
            } else {
                advice.push_str("Lower the velocity or raise the resolution. ");
            }
            unstable = true;
            break;
        }
        if step > 100 && result.residual < problem.tolerance {
            result.converged = true;
            break;
        }
    }
    if unstable {
        result.advice = advice;
        return Ok(result);
    }
    if !result.converged {
        advice.push_str(&format!(
            "the flow had not settled after {} steps (residual {:.3e}); the numbers are a snapshot, not a steady state. ",
            result.steps, result.residual
        ));
    }

    // --- back to SI ---
    let velocity_scale = h / dt;
    let pressure_scale = problem.density * velocity_scale * velocity_scale;
    result.velocity_x = vec![0.0; cells];
    result.velocity_y = vec![0.0; cells];
    result.velocity_z = vec![0.0; cells];
    result.pressure = vec![0.0; cells];
    for v in 0..cells {
        if !fluid[v] {
            continue;
        }
        result.velocity_x[v] = ux[v] * velocity_scale;
        result.velocity_y[v] = uy[v] * velocity_scale;
        result.velocity_z[v] = uz[v] * velocity_scale;
        result.pressure[v] = (rho[v] - 1.0) * CS2 * pressure_scale;
        let speed = Vec3::new(result.velocity_x[v], result.velocity_y[v], result.velocity_z[v]).len();
        if speed.is_finite() {
            result.max_velocity = result.max_velocity.max(speed);
        }
    }
    let plane_average = |plane: &[usize], field: &[f64]| -> (f64, f64) {
        let (mut sum, mut area) = (0.0, 0.0);
        for &v in plane {
            if fluid[v] {
                sum += field[v];
                area += 1.0;
            }
        }
        (if area > 0.0 { sum / area } else { 0.0 }, area)
    };
    let along_flow = match flow_axis {
        0 => &result.velocity_x,
        1 => &result.velocity_y,
        _ => &result.velocity_z,
    };
    let (inlet_pressure, inlet_area) = plane_average(&inlet_cells, &result.pressure);
    let (outlet_pressure, _) = plane_average(&outlet_cells, &result.pressure);
    result.pressure_drop = inlet_pressure - outlet_pressure;
    result.mean_inlet_velocity = plane_average(&inlet_cells, along_flow).0.abs();
    result.volume_flow_rate = result.mean_inlet_velocity * inlet_area * h * h;

    // Drag by momentum exchange: every link that bounced off the part carried momentum into it.
    {
        let (mut fx, mut fy, mut fz) = (0.0, 0.0, 0.0);
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let v = at(i, j, k);
                    if !fluid[v] {
                        continue;
                    }
                    for q in 1..Q {
                        let (ni, nj, nk) = (i as i64 + C[q][0] as i64, j as i64 + C[q][1] as i64, k as i64 + C[q][2] as i64);
                        if ni < 0 || nj < 0 || nk < 0 || ni as usize >= nx || nj as usize >= ny || nk as usize >= nz {
                            continue;
                        }
                        if !result.solid[at(ni as usize, nj as usize, nk as usize)] {
                            continue;
                        }
                        let exchange = f[v * Q + q] + f[v * Q + OPPOSITE[q]];
                        fx += exchange * C[q][0] as f64;
                        fy += exchange * C[q][1] as f64;
                        fz += exchange * C[q][2] as f64;
                    }
                }
            }
        }
        let force_scale = problem.density * h * h * h * h / (dt * dt);
        result.drag_force = Vec3::new(fx * force_scale, fy * force_scale, fz * force_scale);
        if !result.drag_force.is_finite() {
            result.drag_force = Vec3::ZERO;
            advice.push_str("the drag could not be measured because the solve did not stay finite; lower the velocity or raise the resolution. ");
        }
    }

    // --- heat ---
    if problem.solve_heat && !result.converged {
        advice.push_str("heat was not solved because the flow never settled: fix the flow first. ");
    }
    if problem.solve_heat && result.converged {
        // Steady advection-diffusion on the same grid: upwind transport, central conduction,
        // Gauss–Seidel. Upwinding keeps it diagonally dominant.
        let conjugate = problem.heat_power > 0.0;
        let solid_cells = result.solid.iter().filter(|s| **s).count();
        let source_per_cell = if conjugate && solid_cells > 0 { problem.heat_power / solid_cells as f64 } else { 0.0 };
        let mut temperature = vec![problem.inlet_temperature; cells];
        let mass_flow = problem.density * result.volume_flow_rate;
        let capacity_rate = mass_flow * problem.fluid_heat_capacity;
        if conjugate {
            // Start the solid where energy says it must end up.
            let rise = if mass_flow > 0.0 { problem.heat_power / (mass_flow * problem.fluid_heat_capacity) } else { 0.0 };
            for (t, solid) in temperature.iter_mut().zip(&result.solid) {
                if *solid {
                    *t = problem.inlet_temperature + rise;
                }
            }
        } else {
            for (t, solid) in temperature.iter_mut().zip(&result.solid) {
                if *solid {
                    *t = problem.wall_temperature;
                }
            }
        }
        let conductivity_of = |v: usize| if result.solid[v] { problem.solid_conductivity } else { problem.fluid_conductivity };
        let outlet_mean = |temperature: &[f64]| {
            let (mut weighted, mut weight) = (0.0, 0.0);
            for &v in &outlet_cells {
                if fluid[v] {
                    let speed = along_flow[v].abs();
                    weighted += speed * temperature[v];
                    weight += speed;
                }
            }
            if weight > 0.0 { weighted / weight } else { problem.inlet_temperature }
        };
        let rho_cp = problem.density * problem.fluid_heat_capacity;
        let mut best_ratio = f64::INFINITY;
        let mut since_better = 0;
        for sweep in 0..20_000 {
            let (mut change, mut magnitude) = (0.0, 0.0);
            for k in 0..nz {
                for j in 0..ny {
                    for i in 0..nx {
                        let v = at(i, j, k);
                        let is_solid = result.solid[v];
                        if !is_solid && !fluid[v] {
                            continue;
                        }
                        if is_solid && !conjugate {
                            continue; // held temperature
                        }
                        if fluid[v] && on_face(inlet, i, j, k) {
                            temperature[v] = problem.inlet_temperature;
                            continue;
                        }
                        let (mut numerator, mut denominator) = (0.0, 0.0);
                        // Conduction with the harmonic mean at the interface.
                        let k_here = conductivity_of(v);
                        for c in C.iter().take(7).skip(1) {
                            let (ni, nj, nk) = (i as i64 + c[0] as i64, j as i64 + c[1] as i64, k as i64 + c[2] as i64);
                            if ni < 0 || nj < 0 || nk < 0 || ni as usize >= nx || nj as usize >= ny || nk as usize >= nz {
                                continue; // adiabatic outer wall
                            }
                            let n = at(ni as usize, nj as usize, nk as usize);
                            if !fluid[n] && !result.solid[n] {
                                continue;
                            }
                            let k_there = conductivity_of(n);
                            let k_face = 2.0 * k_here * k_there / (k_here + k_there).max(1e-30);
                            numerator += k_face * temperature[n];
                            denominator += k_face;
                        }
                        // Advection, upwind: heat arrives from where the flow came from.
                        if !is_solid {
                            let speeds = [result.velocity_x[v], result.velocity_y[v], result.velocity_z[v]];
                            for (axis, speed) in speeds.iter().enumerate() {
                                if speed.abs() < 1e-12 {
                                    continue;
                                }
                                let mut ijk = [i as i64, j as i64, k as i64];
                                ijk[axis] += if *speed > 0.0 { -1 } else { 1 };
                                if ijk.iter().any(|c| *c < 0) || ijk[0] as usize >= nx || ijk[1] as usize >= ny || ijk[2] as usize >= nz {
                                    continue;
                                }
                                let n = at(ijk[0] as usize, ijk[1] as usize, ijk[2] as usize);
                                if !fluid[n] {
                                    continue;
                                }
                                let weight = rho_cp * speed.abs() * h;
                                numerator += weight * temperature[n];
                                denominator += weight;
                            }
                        }
                        if is_solid && source_per_cell > 0.0 {
                            numerator += source_per_cell / h;
                        }
                        if denominator <= 0.0 {
                            continue;
                        }
                        let solved = numerator / denominator;
                        change += (solved - temperature[v]).abs();
                        magnitude += solved.abs();
                        temperature[v] = solved;
                    }
                }
            }
            // The slowest mode is the level of the solid as a whole; the energy balance fixes it
            // directly every hundred sweeps.
            if conjugate && capacity_rate > 0.0 && sweep % 100 == 99 {
                let carried = capacity_rate * (outlet_mean(&temperature) - problem.inlet_temperature);
                let shift = 0.5 * (problem.heat_power - carried) / capacity_rate;
                if shift.is_finite() && shift.abs() < 1000.0 {
                    for (t, solid) in temperature.iter_mut().zip(&result.solid) {
                        if *solid {
                            *t += shift;
                        }
                    }
                }
            }
            let ratio = if magnitude > 0.0 { change / magnitude } else { 1.0 };
            if ratio < 1e-10 && sweep > 20 {
                break;
            }
            if ratio < best_ratio * 0.98 {
                best_ratio = ratio;
                since_better = 0;
            } else {
                since_better += 1;
                if since_better > 1500 && sweep > 500 {
                    break;
                }
            }
        }
        result.heat_solved = true;
        result.inlet_temperature = problem.inlet_temperature;
        result.max_temperature = (0..cells).filter(|v| fluid[*v] || result.solid[*v]).map(|v| temperature[v]).fold(0.0, f64::max);
        result.outlet_temperature = outlet_mean(&temperature);
        result.heat_removed = capacity_rate * (result.outlet_temperature - result.inlet_temperature);
        if conjugate {
            let imbalance = (result.heat_removed - problem.heat_power).abs() / problem.heat_power;
            if imbalance > 0.05 {
                advice.push_str(&format!(
                    "the heat carried out ({:.2} W) does not match the {:.2} W put in, so the temperature field had not settled; treat the temperatures as indicative. ",
                    result.heat_removed, problem.heat_power
                ));
            }
        }
        result.temperature = temperature;
    }
    result.advice = advice;
    Ok(result)
}

/// A grid for a mesh at a resolution, as `VoxelBody` (for callers that want the mask first).
pub fn voxel_body(mesh: &Mesh, resolution: usize) -> Option<VoxelBody> {
    let grid: VoxelGrid = voxelise(mesh, resolution, MAX_FLOW_CELLS)?;
    Some(VoxelBody { nx: grid.nx, ny: grid.ny, nz: grid.nz, h: grid.h, origin: grid.origin, solid: grid.solid })
}
