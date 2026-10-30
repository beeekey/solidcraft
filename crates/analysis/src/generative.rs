//! Generative design: inventing the shape instead of analysing one.
//!
//! Given where the part is held, what pushes on it and how much room there is, the search
//! proposes several finished designs. A candidate is a skeleton: struts of some radius between
//! anchor points and free interior nodes, plus triangular plates where a web carries the load
//! better than a bar. That representation is small enough to mutate and recombine, and it
//! rasterises onto the voxel grid with no reconstruction step.
//!
//! Scoring happens in two stages: pin-jointed statics on the skeleton itself (a fraction of a
//! millisecond, enough to throw away mechanisms and overloaded layouts), then voxel finite
//! elements on the survivors, the arbiter, which sees joints, plates and bending. A heat sink is
//! scored by conduction with a convective film and verified by the flow solver; a bracket in a
//! stream by finite elements for strength and the flow solver for drag.
//!
//! The field representation ([`field`]) refines a skeleton into a density field, and the checks
//! ([`check`]) verify what was built on a finer mesh with buckling, fatigue and makeability.

#[path = "generative_check.rs"]
pub mod check;
#[path = "generative_field.rs"]
pub mod field;

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Aabb3, Mesh, Vec3};

use crate::buckling::buckling;
use crate::fluid::{FlowBody, FlowRegion, FluidProblem, VoxelBody, fluid_solve};
use crate::makeability::{MakeabilityRules, MakeabilityScan, VoxelFrame, scan_makeability};
use crate::material::OrthotropicMaterial;
use crate::voxel_fem::{LoadCase, VoxelFemProblem, VoxelFemResult, solve_voxels};
use crate::voxelise::MM;

pub use check::{CheckOptions, DesignCheck, SurfaceFinish, check_design, cycles_to_failure, size_factor, surface_factor};
pub use field::{FieldBody, FieldOptions, decode_field, encode_field, mesh_from_mask, refine_to_field};

/// What the search is trying to achieve.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DesignGoal {
    /// Carry the loads with as little material as possible, scored by finite elements.
    CarryLoad,
    /// Get the heat out of the source areas into the coolant stream, scored by conduction with a
    /// film and verified by the conjugate flow solver.
    RemoveHeat,
    /// Carry the loads while disturbing the stream as little as possible.
    LowDrag,
}

/// The stream the part sits in, straight through the design space along one axis.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StreamSettings {
    /// 0 = X, 1 = Y, 2 = Z
    pub axis: usize,
    pub reversed: bool,
    /// m/s
    pub velocity: f64,
    pub density: f64,
    pub viscosity: f64,
    pub conductivity: f64,
    pub heat_capacity: f64,
    /// K
    pub inlet_temperature: f64,
    /// How many candidates the search may spend a full flow solve on.
    pub solver_checks: usize,
    pub solver_steps: usize,
    /// The grid the flow check runs on; 0 means half again as fine as the search grid.
    pub check_resolution: usize,
}

impl Default for StreamSettings {
    fn default() -> Self {
        StreamSettings {
            axis: 0,
            reversed: false,
            velocity: 0.15,
            density: 1.204,
            viscosity: 1.813e-5,
            conductivity: 0.0257,
            heat_capacity: 1005.0,
            inlet_temperature: 293.15,
            solver_checks: 6,
            solver_steps: 6000,
            check_resolution: 0,
        }
    }
}

/// A region of space: a box (m) or a closed mesh (mm, as SolidCraft's meshes are).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RegionShape {
    Box(Aabb3),
    Mesh(Mesh),
}

impl RegionShape {
    /// The region's bounding box (m).
    pub fn bounds(&self) -> Aabb3 {
        match self {
            RegionShape::Box(b) => *b,
            RegionShape::Mesh(m) => {
                let b = m.bounds();
                Aabb3 { min: b.min / MM, max: b.max / MM }
            }
        }
    }

    fn is_empty(&self) -> bool {
        match self {
            RegionShape::Box(b) => !(b.max.x > b.min.x && b.max.y > b.min.y && b.max.z > b.min.z),
            RegionShape::Mesh(m) => m.triangles.is_empty(),
        }
    }
}

/// An area of the part attached to the outside world: a mounting pad, a face a load comes in
/// through, or the footprint of something that gets hot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DesignRegion {
    pub shape: RegionShape,
    pub name: String,
    /// Bolted, welded or glued: takes tension as well as compression. Clear it for a foot that
    /// merely rests on something.
    pub bonded: bool,
    /// Which way is off the support, for an unbonded mount.
    pub lift_direction: Vec3,
}

impl DesignRegion {
    pub fn new(shape: RegionShape, name: &str) -> Self {
        DesignRegion { shape, name: name.into(), bonded: true, lift_direction: Vec3::Z }
    }
}

/// A load case: one force over one region. Cases are always solved separately.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DesignLoad {
    pub region: DesignRegion,
    /// Newtons, the total over the region.
    pub force: Vec3,
    /// Heat put into this area (W), for RemoveHeat.
    pub watts: f64,
    /// How many times this load happens in the life the part has to last; 0 never repeats.
    pub cycles: f64,
    /// The stress ratio of the cycle: 0 comes on and off, −1 fully reverses.
    pub stress_ratio: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GenerativeProblem {
    /// Where material is allowed. None means the box around the mounts and loads grown by half
    /// its own size, which is a guess and is reported as one.
    pub design_space: Option<RegionShape>,
    pub mounts: Vec<DesignRegion>,
    pub loads: Vec<DesignLoad>,
    pub youngs_modulus: f64,
    pub poisson_ratio: f64,
    pub yield_strength: f64,
    /// kg/m³
    pub density: f64,
    pub target_safety_factor: f64,
    pub goal: DesignGoal,
    pub stream: StreamSettings,
    /// Gravity; zero switches it off.
    pub gravity: Vec3,
    /// Constraints during the search, not a report afterwards.
    pub manufacturing: MakeabilityRules,
    /// The buckling load factor the design has to clear; 0 leaves it out of the search.
    pub target_buckling_factor: f64,
    /// The fatigue damage the design may accumulate (1.0 is failure by Miner's rule); 0 leaves
    /// fatigue out.
    pub target_fatigue_damage: f64,
    /// Pa; 0 = 0.45 × yield and 1.5 × yield.
    pub endurance_limit: f64,
    pub ultimate_strength: f64,
    pub solid_conductivity: f64,
    pub solid_heat_capacity: f64,
    /// K
    pub temperature_limit: f64,
    /// Voxels along the longest side.
    pub resolution: usize,
    pub population: usize,
    pub generations: usize,
    /// How many distinct designs to return.
    pub solutions: usize,
    pub seed: u64,
    /// Interior nodes a layout may use.
    pub free_nodes: usize,
    /// Strut radius bounds (m).
    pub min_radius: f64,
    pub max_radius: f64,
}

impl Default for GenerativeProblem {
    fn default() -> Self {
        GenerativeProblem {
            design_space: None,
            mounts: Vec::new(),
            loads: Vec::new(),
            youngs_modulus: 200e9,
            poisson_ratio: 0.3,
            yield_strength: 250e6,
            density: 7850.0,
            target_safety_factor: 2.0,
            goal: DesignGoal::CarryLoad,
            stream: StreamSettings::default(),
            gravity: Vec3::new(0.0, 0.0, -9.81),
            manufacturing: MakeabilityRules::default(),
            target_buckling_factor: 0.0,
            target_fatigue_damage: 0.0,
            endurance_limit: 0.0,
            ultimate_strength: 0.0,
            solid_conductivity: 205.0,
            solid_heat_capacity: 900.0,
            temperature_limit: 358.15,
            resolution: 22,
            population: 30,
            generations: 25,
            solutions: 3,
            seed: 1,
            free_nodes: 3,
            min_radius: 0.0025,
            max_radius: 0.025,
        }
    }
}

/// A strut: a round bar between two nodes.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DesignMember {
    pub a: usize,
    pub b: usize,
    pub radius: f64,
}

/// A plate: a triangular web between three nodes, of some thickness.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DesignPanel {
    pub a: usize,
    pub b: usize,
    pub c: usize,
    pub thickness: f64,
}

/// One candidate design, and how it did.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Design {
    /// Anchors first (mounts, then loads, in the order given), then the free interior nodes.
    pub nodes: Vec<Vec3>,
    pub members: Vec<DesignMember>,
    pub panels: Vec<DesignPanel>,
    /// Reaches the target safety factor.
    pub feasible: bool,
    /// Every load actually reaches a mount.
    pub connected: bool,
    /// kg, from the voxels it fills.
    pub mass: f64,
    /// Pa, worst load case.
    pub max_stress: f64,
    pub safety_factor: f64,
    /// m, worst load case.
    pub max_displacement: f64,
    /// N/m, worst case.
    pub stiffness: f64,
    /// Lower is better.
    pub fitness: f64,
    pub generation: usize,
    /// False = truss estimate only.
    pub scored_by_fem: bool,
    pub peak_temperature: f64,
    pub pressure_drop: f64,
    pub drag: f64,
    pub heat_removed: f64,
    pub checked_by_flow_solver: bool,
    pub buckling_factor: f64,
    pub fatigue_damage: f64,
    pub unmakeable_fraction: f64,
    pub verdict: String,
}

/// The voxel grid the search scores on, shared by every candidate.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DesignGrid {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    pub h: f64,
    pub origin: Vec3,
    /// Inside the design space.
    pub space: Vec<bool>,
    pub mount_mask: Vec<bool>,
    pub mount_voxels: Vec<Vec<usize>>,
    pub load_voxels: Vec<Vec<usize>>,
    /// The centre of each mount region, then of each loaded region.
    pub anchors: Vec<Vec3>,
    pub assumed_space: bool,
    /// A region thinner than one voxel was snapped to the nearest cells.
    pub snapped_regions: bool,
}

impl DesignGrid {
    pub fn count(&self) -> usize {
        self.nx * self.ny * self.nz
    }
    pub fn at(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.ny + j) * self.nx + i
    }
    pub fn ijk(&self, v: usize) -> (usize, usize, usize) {
        (v % self.nx, (v / self.nx) % self.ny, v / (self.nx * self.ny))
    }
    pub fn centre(&self, v: usize) -> Vec3 {
        let (i, j, k) = self.ijk(v);
        self.origin + Vec3::new(i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5) * self.h
    }
    pub fn frame(&self) -> VoxelFrame {
        VoxelFrame { nx: self.nx, ny: self.ny, nz: self.nz, h: self.h, origin: self.origin }
    }
    /// The same space, pads and anchors on a grid `factor` times finer along each axis, so a
    /// skeleton can be drawn closer to its true round struts than the search needed to see it.
    pub fn refined(&self, factor: usize) -> DesignGrid {
        let f = factor.max(1);
        if f == 1 || self.space.len() != self.count() || self.mount_mask.len() != self.count() {
            return self.clone();
        }
        let (nx, ny, nz) = (self.nx * f, self.ny * f, self.nz * f);
        let fine_at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
        let children = |v: usize| -> Vec<usize> {
            let (i, j, k) = self.ijk(v);
            let mut out = Vec::with_capacity(f * f * f);
            for dk in 0..f {
                for dj in 0..f {
                    for di in 0..f {
                        out.push(fine_at(i * f + di, j * f + dj, k * f + dk));
                    }
                }
            }
            out
        };
        let count = nx * ny * nz;
        let mut space = vec![false; count];
        let mut mount_mask = vec![false; count];
        for v in 0..self.count() {
            if self.space[v] || self.mount_mask[v] {
                for c in children(v) {
                    space[c] = self.space[v];
                    mount_mask[c] = self.mount_mask[v];
                }
            }
        }
        let expand = |groups: &Vec<Vec<usize>>| -> Vec<Vec<usize>> {
            groups.iter().map(|g| g.iter().filter(|v| **v < self.count()).flat_map(|v| children(*v)).collect()).collect()
        };
        DesignGrid {
            nx,
            ny,
            nz,
            h: self.h / f as f64,
            origin: self.origin,
            space,
            mount_mask,
            mount_voxels: expand(&self.mount_voxels),
            load_voxels: expand(&self.load_voxels),
            anchors: self.anchors.clone(),
            assumed_space: self.assumed_space,
            snapped_regions: self.snapped_regions,
        }
    }
    fn in_bounds(&self, i: i64, j: i64, k: i64) -> bool {
        i >= 0 && j >= 0 && k >= 0 && (i as usize) < self.nx && (j as usize) < self.ny && (k as usize) < self.nz
    }
    fn neighbours(&self, v: usize) -> impl Iterator<Item = usize> + '_ {
        let (i, j, k) = self.ijk(v);
        STEPS.iter().filter_map(move |s| {
            let (ni, nj, nk) = (i as i64 + s[0], j as i64 + s[1], k as i64 + s[2]);
            self.in_bounds(ni, nj, nk).then(|| self.at(ni as usize, nj as usize, nk as usize))
        })
    }
}

const STEPS: [[i64; 3]; 6] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GenerativeResult {
    pub solved: bool,
    pub advice: String,
    /// Best first, and deliberately different from each other.
    pub designs: Vec<Design>,
    pub evaluations: usize,
    pub fem_evaluations: usize,
    pub stream_checks: usize,
    /// The best mass at the end of each generation.
    pub mass_history: Vec<f64>,
    pub grid: DesignGrid,
}

/// Seeded xorshift64*: the same seed gives the same run on every platform.
#[derive(Clone)]
pub(crate) struct Rng(u64);

impl Rng {
    pub(crate) fn new(seed: u64) -> Rng {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(0xD1B5_4A32_D192_ED03) | 1)
    }
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// In [0, 1).
    pub(crate) fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub(crate) fn index(&mut self, n: usize) -> usize {
        if n == 0 { 0 } else { ((self.unit() * n as f64) as usize).min(n - 1) }
    }
    pub(crate) fn normal(&mut self, mean: f64, sigma: f64) -> f64 {
        let u1 = self.unit().max(1e-300);
        let u2 = self.unit();
        mean + sigma * (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

fn distance_to_segment(point: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let denominator = ab.dot(ab);
    let t = if denominator > 0.0 { ((point - a).dot(ab) / denominator).clamp(0.0, 1.0) } else { 0.0 };
    (point - (a + ab * t)).len()
}

/// Distance from a point to a triangle: inside the projection the perpendicular, outside the
/// nearest edge.
fn distance_to_triangle(point: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let normal = (b - a).cross(c - a);
    let area2 = normal.len();
    if !(area2 > 1e-18) {
        return distance_to_segment(point, a, b);
    }
    let unit = normal / area2;
    let perpendicular = (point - a).dot(unit);
    let projected = point - unit * perpendicular;
    let u = (b - a).cross(projected - a).dot(unit);
    let v = (c - b).cross(projected - b).dot(unit);
    let w = (a - c).cross(projected - c).dot(unit);
    if u >= 0.0 && v >= 0.0 && w >= 0.0 {
        return perpendicular.abs();
    }
    distance_to_segment(point, a, b).min(distance_to_segment(point, b, c)).min(distance_to_segment(point, c, a))
}

/// The union bounding box of everything the problem mentions (m).
fn problem_bounds(problem: &GenerativeProblem) -> (Aabb3, bool) {
    if let Some(space) = &problem.design_space {
        return (space.bounds(), false);
    }
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for shape in problem.mounts.iter().map(|m| &m.shape).chain(problem.loads.iter().map(|l| &l.region.shape)) {
        if shape.is_empty() {
            continue;
        }
        let b = shape.bounds();
        lo = lo.min(b.min);
        hi = hi.max(b.max);
    }
    if !lo.is_finite() || !hi.is_finite() {
        return (Aabb3 { min: Vec3::ZERO, max: Vec3::ZERO }, true);
    }
    let size = hi - lo;
    let margin = 0.5 * size.x.max(size.y).max(size.z).max(1e-6);
    let m = Vec3::new(margin, margin, margin);
    (Aabb3 { min: lo - m, max: hi + m }, true)
}

/// Every voxel whose centre lies inside the shape.
fn mark_inside(shape: &RegionShape, grid: &DesignGrid) -> Vec<usize> {
    let mut out = Vec::new();
    match shape {
        RegionShape::Box(b) => {
            for v in 0..grid.count() {
                let c = grid.centre(v);
                if c.x >= b.min.x && c.x <= b.max.x && c.y >= b.min.y && c.y <= b.max.y && c.z >= b.min.z && c.z <= b.max.z {
                    out.push(v);
                }
            }
        }
        RegionShape::Mesh(m) => {
            if m.triangles.is_empty() {
                return out;
            }
            let index = m.inside_index();
            for v in 0..grid.count() {
                if index.contains(grid.centre(v) * MM) {
                    out.push(v);
                }
            }
        }
    }
    out
}

/// A pad drawn thinner than one voxel can miss every voxel centre; its nearest cells are used
/// instead, as a slab one voxel thick along the axis it was drawn flat on.
fn snap_to_nearest(shape: &RegionShape, grid: &DesignGrid) -> Vec<usize> {
    let mut out = Vec::new();
    if shape.is_empty() {
        return out;
    }
    let b = shape.bounds();
    let reach = grid.h;
    let far = grid.origin + Vec3::new(grid.nx as f64, grid.ny as f64, grid.nz as f64) * grid.h;
    if b.max.x < grid.origin.x - reach
        || b.max.y < grid.origin.y - reach
        || b.max.z < grid.origin.z - reach
        || b.min.x > far.x + reach
        || b.min.y > far.y + reach
        || b.min.z > far.z + reach
    {
        return out;
    }
    let centre = (b.min + b.max) * 0.5;
    let mut best: Option<(usize, f64)> = None;
    for v in 0..grid.count() {
        if !grid.space[v] {
            continue;
        }
        let d = (grid.centre(v) - centre).len();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((v, d));
        }
    }
    let Some((best, _)) = best else { return out };
    let (bi, bj, bk) = grid.ijk(best);
    for v in 0..grid.count() {
        if !grid.space[v] {
            continue;
        }
        let (i, j, k) = grid.ijk(v);
        let at = grid.centre(v);
        let inside_box = at.x >= b.min.x - grid.h
            && at.x <= b.max.x + grid.h
            && at.y >= b.min.y - grid.h
            && at.y <= b.max.y + grid.h
            && at.z >= b.min.z - grid.h
            && at.z <= b.max.z + grid.h;
        let on_slab = i == bi || j == bj || k == bk;
        if inside_box && on_slab {
            out.push(v);
        }
    }
    if out.is_empty() {
        out.push(best);
    }
    out
}

fn centre_of(grid: &DesignGrid, voxels: &[usize]) -> Vec3 {
    if voxels.is_empty() {
        return Vec3::ZERO;
    }
    voxels.iter().map(|v| grid.centre(*v)).fold(Vec3::ZERO, |a, b| a + b) / voxels.len() as f64
}

/// The most cells a search grid may hold.
pub const MAX_GRID_CELLS: usize = 400_000;

/// Builds the scoring grid: the design space, the mount and load regions. `resolution_override`
/// builds the same grid at a different fineness.
pub fn build_design_grid(problem: &GenerativeProblem, resolution_override: Option<usize>) -> DesignGrid {
    let mut grid = DesignGrid::default();
    let (bounds, assumed) = problem_bounds(problem);
    grid.assumed_space = assumed;
    let size = bounds.max - bounds.min;
    let longest = size.x.max(size.y).max(size.z);
    if !(longest > 0.0) || !longest.is_finite() {
        return grid;
    }
    let resolution = resolution_override.unwrap_or(problem.resolution).clamp(8, 96);
    grid.h = longest / resolution as f64;
    let cells = |extent: f64| ((extent / grid.h).round() as usize).max(1);
    grid.nx = cells(size.x);
    grid.ny = cells(size.y);
    grid.nz = cells(size.z);
    grid.origin = bounds.min;
    let Some(count) = grid.nx.checked_mul(grid.ny).and_then(|c| c.checked_mul(grid.nz)).filter(|c| *c <= MAX_GRID_CELLS) else {
        return DesignGrid::default();
    };
    grid.space = vec![problem.design_space.is_none(); count];
    if let Some(space) = &problem.design_space {
        for v in mark_inside(space, &grid) {
            grid.space[v] = true;
        }
    }
    grid.mount_mask = vec![false; count];
    for mount in &problem.mounts {
        let mut voxels = mark_inside(&mount.shape, &grid);
        if voxels.is_empty() {
            voxels = snap_to_nearest(&mount.shape, &grid);
            grid.snapped_regions = !voxels.is_empty();
        }
        // The region is what the user drew, so it wins over the design space's boundary.
        for &v in &voxels {
            grid.space[v] = true;
            grid.mount_mask[v] = true;
        }
        grid.anchors.push(centre_of(&grid, &voxels));
        grid.mount_voxels.push(voxels);
    }
    for load in &problem.loads {
        let mut voxels = mark_inside(&load.region.shape, &grid);
        if voxels.is_empty() {
            voxels = snap_to_nearest(&load.region.shape, &grid);
            grid.snapped_regions = grid.snapped_regions || !voxels.is_empty();
        }
        for &v in &voxels {
            grid.space[v] = true;
        }
        grid.anchors.push(centre_of(&grid, &voxels));
        grid.load_voxels.push(voxels);
    }
    grid
}

/// Fills the grid with one candidate's material: struts, plates, and the mount and load pads,
/// all clipped to the design space.
pub fn rasterise(grid: &DesignGrid, design: &Design) -> Vec<bool> {
    let count = grid.count();
    let mut solid = vec![false; count];
    if count == 0 || !(grid.h > 0.0) || grid.space.len() != count {
        return solid;
    }
    let n = design.nodes.len();
    let cell_of = |x: f64, lo: f64| ((x - lo) / grid.h).floor();
    let sweep = |solid: &mut Vec<bool>, low: Vec3, high: Vec3, reach: f64, distance: &dyn Fn(Vec3) -> f64| {
        let i0 = cell_of(low.x - reach, grid.origin.x).max(0.0) as usize;
        let j0 = cell_of(low.y - reach, grid.origin.y).max(0.0) as usize;
        let k0 = cell_of(low.z - reach, grid.origin.z).max(0.0) as usize;
        let i1 = (((high.x + reach - grid.origin.x) / grid.h).ceil().max(0.0) as usize).min(grid.nx - 1);
        let j1 = (((high.y + reach - grid.origin.y) / grid.h).ceil().max(0.0) as usize).min(grid.ny - 1);
        let k1 = (((high.z + reach - grid.origin.z) / grid.h).ceil().max(0.0) as usize).min(grid.nz - 1);
        for k in k0..=k1 {
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let v = grid.at(i, j, k);
                    if !grid.space[v] || solid[v] {
                        continue;
                    }
                    if distance(grid.centre(v)) <= reach {
                        solid[v] = true;
                    }
                }
            }
        }
    };
    let voxel_of = |p: Vec3| -> Option<(i64, i64, i64)> {
        let (i, j, k) = (cell_of(p.x, grid.origin.x) as i64, cell_of(p.y, grid.origin.y) as i64, cell_of(p.z, grid.origin.z) as i64);
        grid.in_bounds(i, j, k).then_some((i, j, k))
    };
    // A distance test alone leaves gaps on a slanted strut thinner than the grid, and the flood
    // fill counts only face-to-face neighbours, so the spine is walked as well, one axis at a
    // time, which is connected by construction.
    let mark_spine = |solid: &mut Vec<bool>, a: Vec3, b: Vec3| {
        let span = (b - a).len();
        let steps = ((span / (grid.h * 0.25)).ceil() as usize).clamp(2, 100_000);
        let mut last: Option<(i64, i64, i64)> = None;
        for step in 0..=steps {
            let point = a + (b - a) * (step as f64 / steps as f64);
            let Some((i, j, k)) = voxel_of(point) else {
                last = None;
                continue;
            };
            if let Some((mut li, mut lj, mut lk)) = last {
                while li != i || lj != j || lk != k {
                    if li != i {
                        li += if i > li { 1 } else { -1 };
                    } else if lj != j {
                        lj += if j > lj { 1 } else { -1 };
                    } else {
                        lk += if k > lk { 1 } else { -1 };
                    }
                    let v = grid.at(li as usize, lj as usize, lk as usize);
                    if grid.space[v] {
                        solid[v] = true;
                    }
                }
            }
            let v = grid.at(i as usize, j as usize, k as usize);
            if grid.space[v] {
                solid[v] = true;
            }
            last = Some((i, j, k));
        }
    };
    for member in &design.members {
        if member.a >= n || member.b >= n || member.a == member.b {
            continue;
        }
        let (a, b) = (design.nodes[member.a], design.nodes[member.b]);
        // A strut thinner than a voxel would rasterise to nothing, so it is drawn at least one
        // voxel thick and the mass follows from the voxels.
        let radius = member.radius.max(grid.h * 0.5);
        sweep(&mut solid, a.min(b), a.max(b), radius, &|p| distance_to_segment(p, a, b));
        mark_spine(&mut solid, a, b);
    }
    for panel in &design.panels {
        if panel.a >= n || panel.b >= n || panel.c >= n || panel.a == panel.b || panel.b == panel.c || panel.a == panel.c {
            continue;
        }
        let (a, b, c) = (design.nodes[panel.a], design.nodes[panel.b], design.nodes[panel.c]);
        let half = (panel.thickness * 0.5).max(grid.h * 0.5);
        sweep(&mut solid, a.min(b).min(c), a.max(b).max(c), half, &|p| distance_to_triangle(p, a, b, c));
    }
    // The pads are part of every candidate.
    for voxels in grid.mount_voxels.iter().chain(grid.load_voxels.iter()) {
        for &v in voxels {
            if v < count {
                solid[v] = true;
            }
        }
    }
    solid
}

/// Is every load area joined to a mount through material?
pub fn connected_to_mounts(grid: &DesignGrid, solid: &[bool]) -> bool {
    let count = grid.count();
    if grid.load_voxels.is_empty() || solid.len() != count || grid.mount_mask.len() != count {
        return false;
    }
    let mut seen = vec![false; count];
    let mut queue: VecDeque<usize> = VecDeque::new();
    for v in 0..count {
        if solid[v] && grid.mount_mask[v] && !seen[v] {
            seen[v] = true;
            queue.push_back(v);
        }
    }
    while let Some(v) = queue.pop_front() {
        for n in grid.neighbours(v) {
            if solid[n] && !seen[n] {
                seen[n] = true;
                queue.push_back(n);
            }
        }
    }
    grid.load_voxels.iter().all(|voxels| voxels.iter().any(|v| seen.get(*v).copied().unwrap_or(false)))
}

// --- pin-jointed statics on the skeleton: the cheap first opinion --------------------------

struct TrussOutcome {
    stable: bool,
    max_stress: f64,
    max_displacement: f64,
    mass: f64,
    /// Worst axial stress in each member, in the order given.
    member_stress: Vec<f64>,
}

fn truss_solve(problem: &GenerativeProblem, design: &Design, mount_count: usize) -> TrussOutcome {
    let mut outcome =
        TrussOutcome { stable: false, max_stress: 0.0, max_displacement: 0.0, mass: 0.0, member_stress: vec![0.0; design.members.len()] };
    let node_count = design.nodes.len();
    if node_count == 0 || design.members.is_empty() {
        return outcome;
    }
    struct Bar {
        a: usize,
        b: usize,
        area: f64,
        length: f64,
        direction: Vec3,
    }
    let mut bars: Vec<Bar> = Vec::new();
    let mut member_of_bar: Vec<Option<usize>> = Vec::new();
    for (index, member) in design.members.iter().enumerate() {
        if member.a == member.b || member.a >= node_count || member.b >= node_count {
            continue;
        }
        let delta = design.nodes[member.b] - design.nodes[member.a];
        let span = delta.len();
        if !(span > 1e-9) {
            continue;
        }
        member_of_bar.push(Some(index));
        bars.push(Bar {
            a: member.a,
            b: member.b,
            area: std::f64::consts::PI * member.radius * member.radius,
            length: span,
            direction: delta / span,
        });
    }
    // A plate is approximated by bars along its edges, a ranking estimate only.
    for panel in &design.panels {
        for (p, q) in [(panel.a, panel.b), (panel.b, panel.c), (panel.c, panel.a)] {
            if p >= node_count || q >= node_count || p == q {
                continue;
            }
            let delta = design.nodes[q] - design.nodes[p];
            let span = delta.len();
            if !(span > 1e-9) {
                continue;
            }
            member_of_bar.push(None);
            bars.push(Bar { a: p, b: q, area: panel.thickness * span / 3.0, length: span, direction: delta / span });
        }
    }
    if bars.is_empty() {
        return outcome;
    }
    outcome.mass = bars.iter().map(|b| problem.density * b.area * b.length).sum();
    let mut used = vec![false; node_count];
    for bar in &bars {
        used[bar.a] = true;
        used[bar.b] = true;
    }
    let mut dof_of = vec![None; node_count * 3];
    let mut free = 0usize;
    for node in 0..node_count {
        if node < mount_count || !used[node] {
            continue;
        }
        for c in 0..3 {
            dof_of[node * 3 + c] = Some(free);
            free += 1;
        }
    }
    if free == 0 {
        outcome.stable = true;
        return outcome;
    }
    let mut k = vec![0.0; free * free];
    let mut mean_stiffness = 0.0;
    for bar in &bars {
        let stiffness = problem.youngs_modulus * bar.area / bar.length;
        mean_stiffness += stiffness;
        let d = [bar.direction.x, bar.direction.y, bar.direction.z];
        let ends = [bar.a, bar.b];
        let sign = [-1.0, 1.0];
        for p in 0..2 {
            for q in 0..2 {
                for r in 0..3 {
                    for s in 0..3 {
                        if let (Some(row), Some(col)) = (dof_of[ends[p] * 3 + r], dof_of[ends[q] * 3 + s]) {
                            k[row * free + col] += sign[p] * sign[q] * stiffness * d[r] * d[s];
                        }
                    }
                }
            }
        }
    }
    mean_stiffness /= bars.len() as f64;
    // A random layout is often a mechanism; a tiny ridge keeps the elimination from dividing by
    // zero, and the resulting enormous displacement marks it unstable.
    for d in 0..free {
        k[d * free + d] += mean_stiffness * 1e-7;
    }
    // LU with partial pivoting, once, then a solve per load case.
    let mut pivot: Vec<usize> = (0..free).collect();
    for column in 0..free {
        let mut best = column;
        let mut magnitude = k[column * free + column].abs();
        for row in column + 1..free {
            let value = k[row * free + column].abs();
            if value > magnitude {
                magnitude = value;
                best = row;
            }
        }
        if !(magnitude > 0.0) {
            return outcome;
        }
        if best != column {
            pivot.swap(best, column);
            for c in 0..free {
                k.swap(best * free + c, column * free + c);
            }
        }
        let diagonal = k[column * free + column];
        for row in column + 1..free {
            let factor = k[row * free + column] / diagonal;
            k[row * free + column] = factor;
            if factor == 0.0 {
                continue;
            }
            for c in column + 1..free {
                k[row * free + c] -= factor * k[column * free + c];
            }
        }
    }
    let span = design.nodes.last().map(|l| (*l - design.nodes[0]).len()).unwrap_or(0.0);
    let unstable_at = span.max(1e-3) * 0.5;
    outcome.stable = true;
    for (case_index, load) in problem.loads.iter().enumerate() {
        let anchor = mount_count + case_index;
        if anchor >= node_count {
            continue;
        }
        let mut force = vec![0.0; free];
        for (c, component) in [load.force.x, load.force.y, load.force.z].iter().enumerate() {
            if let Some(dof) = dof_of[anchor * 3 + c] {
                force[dof] = *component;
            }
        }
        let mut y = vec![0.0; free];
        for row in 0..free {
            let mut sum = force[pivot[row]];
            for c in 0..row {
                sum -= k[row * free + c] * y[c];
            }
            y[row] = sum;
        }
        let mut u = vec![0.0; free];
        for row in (0..free).rev() {
            let mut sum = y[row];
            for c in row + 1..free {
                sum -= k[row * free + c] * u[c];
            }
            let diagonal = k[row * free + row];
            u[row] = if diagonal.abs() > 0.0 { sum / diagonal } else { 0.0 };
        }
        let displacement = |node: usize| -> Vec3 {
            let pick = |c: usize| dof_of[node * 3 + c].map(|d| u[d]).unwrap_or(0.0);
            Vec3::new(pick(0), pick(1), pick(2))
        };
        for node in 0..node_count {
            let magnitude = displacement(node).len();
            if !magnitude.is_finite() || magnitude > unstable_at {
                outcome.stable = false;
            }
            outcome.max_displacement = outcome.max_displacement.max(magnitude);
        }
        for (index, bar) in bars.iter().enumerate() {
            let elongation = (displacement(bar.b) - displacement(bar.a)).dot(bar.direction);
            let stress = (problem.youngs_modulus * elongation / bar.length).abs();
            if !stress.is_finite() {
                continue;
            }
            outcome.max_stress = outcome.max_stress.max(stress);
            if let Some(member) = member_of_bar[index] {
                outcome.member_stress[member] = outcome.member_stress[member].max(stress);
            }
        }
    }
    outcome
}

/// Laminar flow over a flat plate: Nu = 0.664 Re^½ Pr^⅓, the correlation every fin calculator
/// is built on, which makes the first thermal stage cheap.
fn film_coefficient(problem: &GenerativeProblem, along_stream: f64) -> f64 {
    let s = &problem.stream;
    if !(along_stream > 0.0) || !(s.velocity > 0.0) || !(s.viscosity > 0.0) || !(s.conductivity > 0.0) {
        return 0.0;
    }
    let reynolds = s.density * s.velocity * along_stream / s.viscosity;
    let prandtl = s.viscosity * s.heat_capacity / s.conductivity;
    let nusselt = 0.664 * reynolds.max(1.0).sqrt() * prandtl.max(1e-6).cbrt();
    nusselt * s.conductivity / along_stream
}

struct ConductionOutcome {
    solved: bool,
    peak_temperature: f64,
    wetted_area: f64,
    frontal_area: f64,
}

/// Steady conduction in the candidate with a convective boundary wherever it meets the stream,
/// finite volume on the same voxels, solved by Gauss–Seidel. The design space is the duct, so
/// its walls are walls: a face pressed against one is not cooled.
fn conduction_solve(
    problem: &GenerativeProblem,
    grid: &DesignGrid,
    solid: &[bool],
    sources: &[Vec<usize>],
    watts: &[f64],
    shape_only: bool,
) -> ConductionOutcome {
    let mut outcome = ConductionOutcome { solved: false, peak_temperature: 0.0, wetted_area: 0.0, frontal_area: 0.0 };
    let (nx, ny, nz, h) = (grid.nx, grid.ny, grid.nz, grid.h);
    let count = solid.len();
    if count != grid.count() {
        return outcome;
    }
    let axis = problem.stream.axis.min(2);
    let along_stream = [nx, ny, nz][axis] as f64 * h;
    let film = film_coefficient(problem, along_stream);
    let k = problem.solid_conductivity;
    if !(k > 0.0) || !(film > 0.0) {
        return outcome;
    }
    let solid_link = k * h;
    let film_link = film * h * h;
    let mut diagonal = vec![0.0; count];
    let mut source = vec![0.0; count];
    let mut material = Vec::new();
    for v in 0..count {
        if !solid[v] {
            continue;
        }
        material.push(v);
        diagonal[v] += 1e-12;
        let (i, j, kk) = grid.ijk(v);
        for s in &STEPS {
            let (ni, nj, nk) = (i as i64 + s[0], j as i64 + s[1], kk as i64 + s[2]);
            if !grid.in_bounds(ni, nj, nk) {
                continue;
            }
            if solid[grid.at(ni as usize, nj as usize, nk as usize)] {
                diagonal[v] += solid_link;
            } else {
                diagonal[v] += film_link;
                source[v] += film_link * problem.stream.inlet_temperature;
                outcome.wetted_area += h * h;
            }
        }
    }
    if material.is_empty() {
        return outcome;
    }
    for (s, voxels) in sources.iter().enumerate() {
        let Some(w) = watts.get(s) else { break };
        let in_material = voxels.iter().filter(|v| solid.get(**v).copied().unwrap_or(false)).count();
        if in_material == 0 {
            continue;
        }
        let each = w / in_material as f64;
        for &v in voxels {
            if solid.get(v).copied().unwrap_or(false) {
                source[v] += each;
            }
        }
    }
    let mut temperature = vec![problem.stream.inlet_temperature; count];
    let sweeps = if shape_only { 0 } else { 400 };
    for _ in 0..sweeps {
        let mut change = 0.0_f64;
        for &v in &material {
            let mut sum = source[v];
            for n in grid.neighbours(v) {
                if solid[n] {
                    sum += solid_link * temperature[n];
                }
            }
            let updated = sum / diagonal[v];
            change = change.max((updated - temperature[v]).abs());
            temperature[v] = updated;
        }
        if change < 1e-6 {
            break;
        }
    }
    outcome.solved = true;
    outcome.peak_temperature = material.iter().map(|v| temperature[*v]).fold(0.0, f64::max);
    // Frontal area: what the stream runs into, once per line through the grid along the flow.
    let (across1, across2) = match axis {
        0 => (ny, nz),
        1 => (nx, nz),
        _ => (nx, ny),
    };
    let extent = [nx, ny, nz][axis];
    for a in 0..across1 {
        for b in 0..across2 {
            for s in 0..extent {
                let (i, j, kk) = match axis {
                    0 => (s, a, b),
                    1 => (a, s, b),
                    _ => (a, b, s),
                };
                if solid[grid.at(i, j, kk)] {
                    outcome.frontal_area += h * h;
                    break;
                }
            }
        }
    }
    outcome
}

struct StreamOutcome {
    solved: bool,
    peak_temperature: f64,
    pressure_drop: f64,
    drag: f64,
    heat_removed: f64,
    advice: String,
}

/// The drag (N) the flow solver finds on a design placed in the problem's stream, for comparing
/// designs the search did and did not verify; `None` when the stream could not be solved.
pub fn stream_drag(problem: &GenerativeProblem, grid: &DesignGrid, design: &Design) -> Option<f64> {
    let solid = rasterise(grid, design);
    let outcome = stream_solve(problem, grid, &solid, &[], &[], false);
    outcome.solved.then_some(outcome.drag)
}

/// The flow solver's own verdict on a candidate: the arbiter for the stream goals.
fn stream_solve(
    problem: &GenerativeProblem,
    grid: &DesignGrid,
    solid: &[bool],
    sources: &[Vec<usize>],
    watts: &[f64],
    with_heat: bool,
) -> StreamOutcome {
    let mut outcome = StreamOutcome { solved: false, peak_temperature: 0.0, pressure_drop: 0.0, drag: 0.0, heat_removed: 0.0, advice: String::new() };
    let body = VoxelBody { nx: grid.nx, ny: grid.ny, nz: grid.nz, h: grid.h, origin: grid.origin, solid: solid.to_vec() };
    let axis = problem.stream.axis.min(2);
    let extent = Vec3::new(grid.nx as f64, grid.ny as f64, grid.nz as f64) * grid.h;
    let middle = grid.origin + extent * 0.5;
    let (mut inlet, mut outlet) = (middle, middle);
    let (low, high) = ([grid.origin.x, grid.origin.y, grid.origin.z], [grid.origin.x + extent.x, grid.origin.y + extent.y, grid.origin.z + extent.z]);
    let set = |v: &mut Vec3, axis: usize, value: f64| match axis {
        0 => v.x = value,
        1 => v.y = value,
        _ => v.z = value,
    };
    set(&mut inlet, axis, if problem.stream.reversed { high[axis] } else { low[axis] });
    set(&mut outlet, axis, if problem.stream.reversed { low[axis] } else { high[axis] });
    let mut total = 0.0;
    if with_heat {
        for (s, voxels) in sources.iter().enumerate() {
            if let Some(w) = watts.get(s)
                && voxels.iter().any(|v| solid.get(*v).copied().unwrap_or(false))
            {
                total += w;
            }
        }
    }
    let fluid = FluidProblem {
        region: FlowRegion::External,
        inlet_point: inlet,
        outlet_point: outlet,
        inlet_velocity: problem.stream.velocity,
        density: problem.stream.density,
        viscosity: problem.stream.viscosity,
        domain: None,
        resolution: 40,
        max_steps: problem.stream.solver_steps.max(200),
        tolerance: 1e-4,
        solve_heat: with_heat,
        inlet_temperature: problem.stream.inlet_temperature,
        wall_temperature: problem.stream.inlet_temperature,
        heat_power: total,
        fluid_conductivity: problem.stream.conductivity,
        fluid_heat_capacity: problem.stream.heat_capacity,
        solid_conductivity: problem.solid_conductivity,
        solid_density: problem.density,
        solid_heat_capacity: problem.solid_heat_capacity,
    };
    match fluid_solve(FlowBody::Voxels(&body), &fluid) {
        Ok(result) => {
            outcome.advice = result.advice.clone();
            if !result.converged {
                return outcome;
            }
            outcome.solved = true;
            outcome.pressure_drop = result.pressure_drop;
            outcome.drag = [result.drag_force.x, result.drag_force.y, result.drag_force.z][axis].abs();
            if with_heat && result.heat_solved {
                outcome.peak_temperature = result.max_temperature;
                outcome.heat_removed = result.heat_removed;
            } else if with_heat {
                outcome.solved = false;
            }
        }
        Err(e) => outcome.advice = e.to_string(),
    }
    outcome
}

/// A fully-stressed design: give every member the section it actually needs, gently.
fn size_members(problem: &GenerativeProblem, design: &mut Design, mount_count: usize) {
    if design.members.is_empty() {
        return;
    }
    let allowed = problem.yield_strength / problem.target_safety_factor.max(1e-9);
    for _ in 0..3 {
        let outcome = truss_solve(problem, design, mount_count);
        if !outcome.stable || outcome.member_stress.len() != design.members.len() {
            return;
        }
        let mut changed = false;
        for (index, member) in design.members.iter_mut().enumerate() {
            let stress = outcome.member_stress[index];
            if !(stress > 0.0) {
                continue;
            }
            let factor = (stress / allowed).sqrt().clamp(0.5, 2.0);
            let wanted = (member.radius * factor).clamp(problem.min_radius, problem.max_radius);
            if (wanted - member.radius).abs() > 1e-9 {
                changed = true;
            }
            member.radius = wanted;
        }
        if !changed {
            return;
        }
    }
}

/// Miner's rule over the load cases: each one's stress amplitude gives a life, each one's cycle
/// count spends part of it, and the sum is what is used up. One is failure.
pub(crate) fn fatigue_damage_of(problem: &GenerativeProblem, result: &VoxelFemResult) -> f64 {
    let endurance = if problem.endurance_limit > 0.0 { problem.endurance_limit } else { 0.45 * problem.yield_strength };
    let ultimate = if problem.ultimate_strength > 0.0 { problem.ultimate_strength } else { 1.5 * problem.yield_strength };
    let mut damage = 0.0;
    for (index, case) in result.cases.iter().enumerate() {
        let Some(load) = problem.loads.get(index) else { break };
        if load.cycles <= 0.0 {
            continue;
        }
        let peak = case.max_von_mises;
        if !(peak > 0.0) {
            continue;
        }
        let ratio = load.stress_ratio.clamp(-1.0, 0.99);
        let amplitude = peak * (1.0 - ratio) / 2.0;
        let mean = peak * (1.0 + ratio) / 2.0;
        let life = cycles_to_failure(amplitude, mean, endurance, ultimate);
        if life <= 0.0 {
            return 1e6;
        }
        damage += load.cycles / life;
    }
    damage
}

/// One number the search minimises. Mass is always the objective; the constraints are penalties
/// steep enough that a design which misses one can never beat one that meets it.
fn fitness_of(problem: &GenerativeProblem, design: &Design, mass_reference: f64, span_length: f64) -> f64 {
    if !design.connected {
        return 1e6 + design.mass / mass_reference.max(1e-12);
    }
    let mass = design.mass / mass_reference.max(1e-12);
    let allowed = problem.yield_strength / problem.target_safety_factor.max(1e-9);
    let overload = (design.max_stress / allowed.max(1e-9) - 1.0).max(0.0);
    let slack = span_length / 200.0;
    let deflection = 0.05 * (design.max_displacement / slack.max(1e-9));
    let mut extra = 0.0;
    if problem.target_buckling_factor > 0.0 {
        let factor = if design.buckling_factor > 0.0 { design.buckling_factor } else { 1e9 };
        let short = (problem.target_buckling_factor / factor - 1.0).max(0.0);
        extra += 4.0 * short * short + 0.6 * short;
    }
    if problem.target_fatigue_damage > 0.0 && design.fatigue_damage > 0.0 {
        let over = (design.fatigue_damage / problem.target_fatigue_damage - 1.0).max(0.0);
        extra += 4.0 * over * over + 0.6 * over;
    }
    if problem.manufacturing.weight > 0.0 && design.unmakeable_fraction > 0.0 {
        extra += problem.manufacturing.weight * 3.0 * design.unmakeable_fraction;
    }
    match problem.goal {
        DesignGoal::CarryLoad => mass + 3.0 * overload * overload + 0.5 * overload + deflection + extra,
        DesignGoal::RemoveHeat => {
            let budget = (problem.temperature_limit - problem.stream.inlet_temperature).max(1e-6);
            let rise = design.peak_temperature - problem.stream.inlet_temperature;
            let over = (rise / budget - 1.0).max(0.0);
            if !(design.peak_temperature > 0.0) {
                return 1e5 + mass;
            }
            let dynamic = 0.5 * problem.stream.density * problem.stream.velocity * problem.stream.velocity;
            let pumping = 0.02 * design.pressure_drop / dynamic.max(1e-9);
            mass + 6.0 * over * over + over + pumping + extra
        }
        DesignGoal::LowDrag => {
            let dynamic = 0.5 * problem.stream.density * problem.stream.velocity * problem.stream.velocity * span_length * span_length;
            let drag_term = design.drag / dynamic.max(1e-12);
            mass + 3.0 * overload * overload + 0.5 * overload + deflection + 2.0 * drag_term + extra
        }
    }
}

/// Everything the search needs that does not change between candidates.
struct Context<'a> {
    problem: &'a GenerativeProblem,
    grid: &'a DesignGrid,
    mount_count: usize,
    anchor_count: usize,
    span: f64,
    mass_reference: f64,
    material: OrthotropicMaterial,
    check_grid: Option<DesignGrid>,
    /// What the cheap thermal estimate is multiplied by to agree with the flow solver.
    estimate_correction: f64,
    heat_sources: Vec<Vec<usize>>,
    heat_watts: Vec<f64>,
}

/// The finite-element problem for a rasterised candidate on a grid.
pub(crate) fn fem_problem(
    problem: &GenerativeProblem,
    grid: &DesignGrid,
    solid: Vec<bool>,
    material: OrthotropicMaterial,
    max_iterations: usize,
    tolerance: f64,
) -> VoxelFemProblem {
    let count = grid.count();
    let mut fem = VoxelFemProblem::new(grid.nx, grid.ny, grid.nz, grid.h, material);
    fem.origin = grid.origin;
    fem.held = (0..count).map(|v| solid[v] && grid.mount_mask[v]).collect();
    fem.solid = solid;
    // The pads are the user's own geometry and the clamp is where linear elasticity puts its
    // singularity; neither governs the safety factor.
    let mut exclude = vec![false; count];
    for voxels in grid.mount_voxels.iter().chain(grid.load_voxels.iter()) {
        for &v in voxels {
            if v < count {
                exclude[v] = true;
            }
        }
    }
    let pads = exclude.clone();
    for (v, pad) in pads.iter().enumerate().take(count) {
        if !pad {
            continue;
        }
        let (i, j, k) = grid.ijk(v);
        for dk in -1..=1 {
            for dj in -1..=1 {
                for di in -1..=1 {
                    let (ni, nj, nk) = (i as i64 + di, j as i64 + dj, k as i64 + dk);
                    if grid.in_bounds(ni, nj, nk) {
                        exclude[grid.at(ni as usize, nj as usize, nk as usize)] = true;
                    }
                }
            }
        }
    }
    fem.exclude_from_peak = exclude;
    fem.gravity = problem.gravity;
    fem.density = problem.density;
    for mount in &problem.mounts {
        if !mount.bonded {
            fem.no_tension_supports = true;
            fem.support_normal = mount.lift_direction;
        }
    }
    fem.max_iterations = max_iterations;
    fem.tolerance = tolerance;
    for (index, voxels) in grid.load_voxels.iter().enumerate() {
        if let Some(load) = problem.loads.get(index) {
            fem.cases.push(LoadCase { voxels: voxels.clone(), force: load.force, name: load.region.name.clone() });
        }
    }
    fem
}

/// Scores a candidate with real finite elements: the arbiter.
fn score_by_fem(context: &Context<'_>, design: &mut Design) {
    let grid = context.grid;
    let solid = rasterise(grid, design);
    design.connected = connected_to_mounts(grid, &solid);
    let voxel_mass = grid.h * grid.h * grid.h * context.problem.density;
    design.mass = solid.iter().filter(|s| **s).count() as f64 * voxel_mass;
    if !design.connected {
        design.verdict = "the load never reaches a mount".into();
        design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
        return;
    }
    let fem = fem_problem(context.problem, grid, solid.clone(), context.material, 600, 1e-6);
    let any_force = fem.cases.iter().any(|c| c.force.len() > 0.0);
    if !any_force {
        // A heat-sink study carries watts, not newtons.
        design.scored_by_fem = true;
        design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
        return;
    }
    let result = solve_voxels(&fem);
    design.scored_by_fem = true;
    if !result.solved {
        design.connected = false;
        design.verdict = result.advice;
        design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
        return;
    }
    design.max_stress = result.worst_von_mises;
    design.max_displacement = result.worst_displacement;
    design.safety_factor = if result.worst_von_mises > 0.0 { context.problem.yield_strength / result.worst_von_mises } else { 0.0 };
    let worst_force = context.problem.loads.iter().map(|l| l.force.len()).fold(0.0, f64::max);
    design.stiffness = if design.max_displacement > 0.0 { worst_force / design.max_displacement } else { 0.0 };
    design.feasible = design.safety_factor >= context.problem.target_safety_factor;
    if context.problem.target_buckling_factor > 0.0 {
        let folded = buckling(&fem, &result, 12);
        design.buckling_factor = if folded.solved { folded.load_factor } else { 0.0 };
    }
    if context.problem.target_fatigue_damage > 0.0 {
        design.fatigue_damage = fatigue_damage_of(context.problem, &result);
    }
    if context.problem.manufacturing.weight > 0.0 {
        design.unmakeable_fraction = scan_makeability(&grid.frame(), &solid, &context.problem.manufacturing).worst_fraction;
    }
    design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
    design.verdict = if design.feasible {
        format!("safety factor {:.2}, above the target of {:.2}", design.safety_factor, context.problem.target_safety_factor)
    } else {
        format!("safety factor {:.2}, below the target of {:.2} -- this one would yield", design.safety_factor, context.problem.target_safety_factor)
    };
}

/// The cheap thermal opinion: conduction with a convective film where it meets the stream.
fn score_by_conduction(context: &Context<'_>, design: &mut Design) {
    let grid = context.grid;
    let solid = rasterise(grid, design);
    design.connected = connected_to_mounts(grid, &solid);
    let voxel_mass = grid.h * grid.h * grid.h * context.problem.density;
    design.mass = solid.iter().filter(|s| **s).count() as f64 * voxel_mass;
    design.scored_by_fem = true;
    if !design.connected {
        design.verdict = "the heat never reaches a mount".into();
        design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
        return;
    }
    let outcome = conduction_solve(context.problem, grid, &solid, &context.heat_sources, &context.heat_watts, false);
    if !outcome.solved {
        design.verdict = "no conduction path for the heat".into();
        design.peak_temperature = 0.0;
        design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
        return;
    }
    let inlet = context.problem.stream.inlet_temperature;
    design.peak_temperature = inlet + (outcome.peak_temperature - inlet) * context.estimate_correction;
    design.feasible = design.peak_temperature <= context.problem.temperature_limit;
    let across = [
        grid.ny as f64 * grid.h * grid.nz as f64 * grid.h,
        grid.nx as f64 * grid.h * grid.nz as f64 * grid.h,
        grid.nx as f64 * grid.h * grid.ny as f64 * grid.h,
    ];
    let axis = context.problem.stream.axis.min(2);
    let blockage = outcome.frontal_area / across[axis].max(1e-12);
    design.pressure_drop = 0.5 * context.problem.stream.density * context.problem.stream.velocity * context.problem.stream.velocity * blockage * 2.0;
    design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
    design.verdict =
        format!("estimated {:.1} C at the hottest point, {:.1} cm2 in the stream", design.peak_temperature - 273.15, outcome.wetted_area * 1e4);
}

/// The arbiter for the stream goals: the flow solver itself, on its own finer grid.
fn verify_by_stream(context: &mut Context<'_>, design: &mut Design, stream_checks: &mut usize) {
    let grid = context.check_grid.clone().unwrap_or_else(|| context.grid.clone());
    let solid = rasterise(&grid, design);
    if !design.connected {
        return;
    }
    let want_heat = context.problem.goal == DesignGoal::RemoveHeat;
    let mut sources = Vec::new();
    let mut watts = Vec::new();
    for (l, load) in context.problem.loads.iter().enumerate() {
        if load.watts <= 0.0 {
            continue;
        }
        if let Some(v) = grid.load_voxels.get(l) {
            sources.push(v.clone());
        }
        watts.push(load.watts);
    }
    let outcome = stream_solve(context.problem, &grid, &solid, &sources, &watts, want_heat);
    *stream_checks += 1;
    if !outcome.solved {
        design.verdict.push_str(&if outcome.advice.is_empty() {
            "; the flow solver could not settle this case".to_string()
        } else {
            format!("; flow solver: {}", outcome.advice)
        });
        return;
    }
    // Calibrate the cheap estimate against what was just measured.
    if want_heat && outcome.peak_temperature > 0.0 {
        let inlet = context.problem.stream.inlet_temperature;
        let estimated = design.peak_temperature - inlet;
        let solved = outcome.peak_temperature - inlet;
        if estimated > 1e-6 && solved > 1e-6 {
            let current = context.estimate_correction;
            let measured = current * solved / estimated;
            let calibrated = if current == 1.0 { measured } else { 0.5 * current + 0.5 * measured };
            context.estimate_correction = calibrated.clamp(0.25, 6.0);
        }
    }
    design.checked_by_flow_solver = true;
    design.pressure_drop = outcome.pressure_drop;
    design.drag = outcome.drag;
    if want_heat {
        design.peak_temperature = outcome.peak_temperature;
        design.heat_removed = outcome.heat_removed;
        design.feasible = design.peak_temperature <= context.problem.temperature_limit;
    }
    design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
    design.verdict = if want_heat {
        format!(
            "flow solver: {:.1} C at the hottest point, {:.1} W into the coolant, {:.2} Pa across the duct",
            design.peak_temperature - 273.15,
            design.heat_removed,
            design.pressure_drop
        )
    } else {
        format!("flow solver: {:.3} N of drag, {:.2} Pa across the duct", design.drag, design.pressure_drop)
    };
    if !outcome.advice.is_empty() {
        design.verdict.push_str(&format!(" ({})", outcome.advice));
    }
}

fn score_candidate(context: &Context<'_>, design: &mut Design) {
    if context.problem.goal == DesignGoal::RemoveHeat {
        score_by_conduction(context, design);
        return;
    }
    score_by_fem(context, design);
    if context.problem.goal == DesignGoal::LowDrag && design.connected && !design.checked_by_flow_solver {
        // Frontal area times the dynamic pressure: the first term of every drag estimate.
        let solid = rasterise(context.grid, design);
        let shape = conduction_solve(context.problem, context.grid, &solid, &[], &[], true);
        let frontal = if shape.frontal_area > 0.0 { shape.frontal_area } else { design.mass / context.problem.density.max(1.0) };
        design.drag = 1.1 * 0.5 * context.problem.stream.density * context.problem.stream.velocity * context.problem.stream.velocity * frontal;
        design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
    }
}

/// The cheap opinion: statics on the skeleton, no voxels at all.
fn score_by_truss(context: &Context<'_>, design: &mut Design) {
    let outcome = truss_solve(context.problem, design, context.mount_count);
    design.scored_by_fem = false;
    design.connected = outcome.stable && outcome.mass > 0.0;
    design.mass = outcome.mass;
    design.max_stress = outcome.max_stress;
    design.max_displacement = outcome.max_displacement;
    design.safety_factor = if outcome.max_stress > 0.0 { context.problem.yield_strength / outcome.max_stress } else { 0.0 };
    design.feasible = false;
    design.fitness = fitness_of(context.problem, design, context.mass_reference, context.span);
    design.verdict = if outcome.stable { "estimated from the skeleton".into() } else { "a mechanism: it folds up rather than carrying".into() };
}

/// How different two designs are: half where the material went, half how it is joined up.
fn design_distance(a: &Design, b: &Design, span: f64) -> f64 {
    let nodes = a.nodes.len().min(b.nodes.len());
    let mut positions = 0.0;
    if nodes > 0 {
        positions = (0..nodes).map(|n| (a.nodes[n] - b.nodes[n]).len()).sum::<f64>() / (nodes as f64 * span.max(1e-9));
    }
    let key = |m: &DesignMember| (m.a.min(m.b), m.a.max(m.b));
    let mut left: Vec<(usize, usize)> = a.members.iter().map(key).collect();
    let mut right: Vec<(usize, usize)> = b.members.iter().map(key).collect();
    left.sort_unstable();
    right.sort_unstable();
    let (mut i, mut j, mut shared) = (0, 0, 0usize);
    while i < left.len() && j < right.len() {
        match left[i].cmp(&right[j]) {
            std::cmp::Ordering::Less => i += 1,
            std::cmp::Ordering::Greater => j += 1,
            std::cmp::Ordering::Equal => {
                shared += 1;
                i += 1;
                j += 1;
            }
        }
    }
    let total = left.len() + right.len() - shared;
    let topology = if total > 0 { 1.0 - shared as f64 / total as f64 } else { 0.0 };
    0.5 * positions.min(1.0) + 0.5 * topology
}

/// Whether a design has anything the builder would make: a member with two different,
/// in-range, non-coincident endpoints, or a panel with three.
pub fn design_has_something_to_build(design: &Design) -> bool {
    let n = design.nodes.len();
    let apart = |i: usize, j: usize| (design.nodes[i] - design.nodes[j]).len() > 1e-9;
    design.members.iter().any(|m| m.a < n && m.b < n && m.a != m.b && apart(m.a, m.b))
        || design.panels.iter().any(|p| p.a < n && p.b < n && p.c < n && apart(p.a, p.b) && apart(p.b, p.c) && apart(p.a, p.c))
}

/// The search. `progress` is called at the end of each generation with the best design so far;
/// returning false stops the run early and keeps what has been found.
pub fn generative_search(problem: &GenerativeProblem, mut progress: Option<&mut dyn FnMut(usize, &Design) -> bool>) -> GenerativeResult {
    let mut result = GenerativeResult::default();
    if problem.mounts.is_empty() {
        result.advice = "nothing holds the part: add at least one mount sketch".into();
        return result;
    }
    if problem.loads.is_empty() {
        result.advice = "nothing pushes on the part: add at least one load sketch".into();
        return result;
    }
    if !(problem.youngs_modulus > 0.0) || !(problem.yield_strength > 0.0) || !(problem.density > 0.0) || !problem.youngs_modulus.is_finite() {
        result.advice = "the material needs a stiffness, a yield strength and a density".into();
        return result;
    }
    if !(problem.min_radius > 0.0) || !(problem.max_radius >= problem.min_radius) || !problem.max_radius.is_finite() {
        result.advice = "the strut radius bounds must be positive, the largest at least the smallest".into();
        return result;
    }
    result.grid = build_design_grid(problem, None);
    let grid = result.grid.clone();
    if grid.nx == 0 || grid.space.is_empty() {
        result.advice = "the design space is empty -- check the sketches and the space body".into();
        return result;
    }
    for (m, voxels) in grid.mount_voxels.iter().enumerate() {
        if voxels.is_empty() {
            result.advice = format!(
                "mount \"{}\" does not touch the design space at this resolution",
                problem.mounts.get(m).map(|r| r.name.as_str()).unwrap_or("")
            );
            return result;
        }
    }
    for (l, voxels) in grid.load_voxels.iter().enumerate() {
        if voxels.is_empty() {
            result.advice = format!(
                "load \"{}\" does not touch the design space at this resolution",
                problem.loads.get(l).map(|r| r.region.name.as_str()).unwrap_or("")
            );
            return result;
        }
    }
    let extent = Vec3::new(grid.nx as f64, grid.ny as f64, grid.nz as f64) * grid.h;
    let space_voxels = grid.space.iter().filter(|s| **s).count();
    let mut context = Context {
        problem,
        grid: &grid,
        mount_count: problem.mounts.len(),
        anchor_count: grid.anchors.len(),
        span: extent.len(),
        mass_reference: space_voxels as f64 * grid.h * grid.h * grid.h * problem.density,
        material: OrthotropicMaterial::isotropic(problem.youngs_modulus, problem.poisson_ratio),
        check_grid: None,
        estimate_correction: 1.0,
        heat_sources: Vec::new(),
        heat_watts: Vec::new(),
    };
    for (l, load) in problem.loads.iter().enumerate() {
        if load.watts <= 0.0 {
            continue;
        }
        context.heat_sources.push(grid.load_voxels[l].clone());
        context.heat_watts.push(load.watts);
    }
    if problem.goal == DesignGoal::RemoveHeat && context.heat_sources.is_empty() {
        result.advice = "nothing gets hot: give a loaded area a power in watts".into();
        return result;
    }
    let uses_stream = matches!(problem.goal, DesignGoal::RemoveHeat | DesignGoal::LowDrag);
    if uses_stream {
        let fine =
            if problem.stream.check_resolution > 0 { problem.stream.check_resolution } else { (problem.resolution as f64 * 1.5).round() as usize };
        let check = build_design_grid(problem, Some(fine));
        if check.nx > 0 {
            context.check_grid = Some(check);
        }
    }

    let free_nodes = problem.free_nodes.min(8);
    let node_count = context.anchor_count + free_nodes;
    let mut rng = Rng::new(problem.seed);
    let space_centres: Vec<Vec3> = (0..grid.count()).filter(|v| grid.space[*v]).map(|v| grid.centre(v)).collect();
    if space_centres.is_empty() {
        result.advice = "the design space is empty -- check the sketches and the space body".into();
        return result;
    }
    let snap_into_space = |point: Vec3| -> Vec3 {
        let i = ((point.x - grid.origin.x) / grid.h).floor() as i64;
        let j = ((point.y - grid.origin.y) / grid.h).floor() as i64;
        let k = ((point.z - grid.origin.z) / grid.h).floor() as i64;
        if grid.in_bounds(i, j, k) && grid.space[grid.at(i as usize, j as usize, k as usize)] {
            return point;
        }
        space_centres.iter().copied().min_by(|a, b| (*a - point).len().total_cmp(&(*b - point).len())).unwrap_or(point)
    };
    let mid_radius = (problem.min_radius * problem.max_radius).sqrt();
    let mount_count = context.mount_count;
    let load_count = problem.loads.len();
    let make_nodes = |rng: &mut Rng, random: bool| -> Vec<Vec3> {
        let mut nodes = grid.anchors.clone();
        for n in 0..free_nodes {
            if random {
                nodes.push(space_centres[rng.index(space_centres.len())]);
            } else {
                let from = grid.anchors[n % mount_count];
                let to = grid.anchors[mount_count + (n % load_count)];
                nodes.push(snap_into_space((from + to) * 0.5));
            }
        }
        nodes
    };
    let nearest_mount = |point: Vec3| -> usize {
        (0..mount_count).min_by(|a, b| (grid.anchors[*a] - point).len().total_cmp(&(grid.anchors[*b] - point).len())).unwrap_or(0)
    };

    // Seeds: the designs a person would try first.
    let mut population: Vec<Design> = Vec::new();
    {
        let mut direct = Design { nodes: make_nodes(&mut rng, false), ..Default::default() };
        for l in 0..load_count {
            let anchor = mount_count + l;
            direct.members.push(DesignMember { a: nearest_mount(grid.anchors[anchor]), b: anchor, radius: mid_radius });
        }
        population.push(direct.clone());
        let mut fan = direct.clone();
        fan.members.clear();
        for l in 0..load_count {
            for m in 0..mount_count {
                fan.members.push(DesignMember { a: m, b: mount_count + l, radius: mid_radius });
            }
        }
        population.push(fan);
        if problem.goal == DesignGoal::RemoveHeat && free_nodes >= 2 {
            let mut finned = direct.clone();
            finned.members.clear();
            let hub = context.anchor_count;
            for m in 0..mount_count {
                finned.members.push(DesignMember { a: m, b: hub, radius: mid_radius });
            }
            for n in 0..free_nodes.saturating_sub(1) {
                finned.panels.push(DesignPanel {
                    a: 0,
                    b: context.anchor_count + n,
                    c: context.anchor_count + n + 1,
                    thickness: problem.min_radius.max(0.0015),
                });
            }
            for l in 0..load_count {
                finned.members.push(DesignMember { a: 0, b: mount_count + l, radius: mid_radius });
            }
            population.push(finned);
        }
        if free_nodes > 0 {
            let mut star = direct.clone();
            star.members.clear();
            let hub = context.anchor_count;
            for m in 0..mount_count {
                star.members.push(DesignMember { a: m, b: hub, radius: mid_radius });
            }
            for l in 0..load_count {
                star.members.push(DesignMember { a: hub, b: mount_count + l, radius: mid_radius });
            }
            population.push(star);
        }
    }
    for seed in &mut population {
        size_members(problem, seed, mount_count);
    }
    let population_size = problem.population.clamp(6, 200);
    while population.len() < population_size {
        let mut candidate = Design { nodes: make_nodes(&mut rng, true), ..Default::default() };
        let members = 2 + (rng.unit() * 5.0) as usize;
        for _ in 0..members {
            let (a, b) = (rng.index(node_count), rng.index(node_count));
            if a == b {
                continue;
            }
            candidate.members.push(DesignMember { a, b, radius: problem.min_radius + rng.unit() * (problem.max_radius - problem.min_radius) });
        }
        for l in 0..load_count {
            let anchor = mount_count + l;
            if !candidate.members.iter().any(|m| m.a == anchor || m.b == anchor) {
                let other = rng.index(node_count);
                candidate.members.push(DesignMember { a: anchor, b: other, radius: mid_radius });
            }
        }
        population.push(candidate);
    }

    let span = context.span;
    let anchor_count = context.anchor_count;
    let mutate = |rng: &mut Rng, design: &mut Design| {
        let sigma = span * 0.12;
        if free_nodes > 0 && rng.unit() < 0.7 {
            let node = anchor_count + rng.index(free_nodes);
            if node < design.nodes.len() {
                let jitter = Vec3::new(rng.normal(0.0, sigma), rng.normal(0.0, sigma), rng.normal(0.0, sigma));
                design.nodes[node] = snap_into_space(design.nodes[node] + jitter);
            }
        }
        if !design.members.is_empty() && rng.unit() < 0.8 {
            let index = rng.index(design.members.len());
            let factor = 0.6 + rng.unit() * 0.9;
            design.members[index].radius = (design.members[index].radius * factor).clamp(problem.min_radius, problem.max_radius);
        }
        if rng.unit() < 0.3 {
            let (a, b) = (rng.index(node_count), rng.index(node_count));
            if a != b {
                design.members.push(DesignMember { a, b, radius: problem.min_radius + rng.unit() * (problem.max_radius - problem.min_radius) });
            }
        }
        if design.members.len() > load_count + 1 && rng.unit() < 0.3 {
            let index = rng.index(design.members.len());
            design.members.remove(index);
        }
        if !design.members.is_empty() && rng.unit() < 0.25 {
            let index = rng.index(design.members.len());
            let node = rng.index(node_count);
            if rng.unit() < 0.5 {
                design.members[index].a = node;
            } else {
                design.members[index].b = node;
            }
        }
        let plate_chance = if problem.goal == DesignGoal::RemoveHeat { 0.5 } else { 0.2 };
        if node_count >= 3 && rng.unit() < plate_chance {
            let (a, b, c) = (rng.index(node_count), rng.index(node_count), rng.index(node_count));
            if a != b && b != c && a != c {
                design.panels.push(DesignPanel {
                    a,
                    b,
                    c,
                    thickness: (problem.min_radius * 2.0 * (0.5 + rng.unit())).clamp(problem.min_radius, problem.max_radius),
                });
            }
        }
        if !design.panels.is_empty() && rng.unit() < if problem.goal == DesignGoal::RemoveHeat { 0.15 } else { 0.3 } {
            let index = rng.index(design.panels.len());
            design.panels.remove(index);
        }
        if rng.unit() < 0.25 {
            let scale = 0.75 + rng.unit() * 0.6;
            for member in &mut design.members {
                member.radius = (member.radius * scale).clamp(problem.min_radius, problem.max_radius);
            }
        }
    };
    let crossover = |rng: &mut Rng, mother: &Design, father: &Design| -> Design {
        let mut child = Design { nodes: mother.nodes.clone(), ..Default::default() };
        for n in anchor_count..node_count {
            if n < father.nodes.len() && n < child.nodes.len() && rng.unit() < 0.5 {
                child.nodes[n] = father.nodes[n];
            }
        }
        for member in &mother.members {
            if rng.unit() < 0.7 {
                child.members.push(*member);
            }
        }
        for member in &father.members {
            if rng.unit() < 0.5 {
                child.members.push(*member);
            }
        }
        child.panels = if rng.unit() < 0.5 { mother.panels.clone() } else { father.panels.clone() };
        if child.members.is_empty() {
            child.members = mother.members.clone();
        }
        child
    };

    let generations = problem.generations.clamp(1, 200);
    let fem_per_generation = (population_size / 3).max(4);
    let mut stream_checks_left = if uses_stream { problem.stream.solver_checks.min(50) } else { 0 };
    let check_every = (generations / stream_checks_left.max(1)).max(1);
    let mut scored_history: Vec<Design> = Vec::new();
    let by_fitness = |a: &Design, b: &Design| a.fitness.total_cmp(&b.fitness);
    for generation in 0..generations {
        for design in &mut population {
            // A design already judged keeps that judgement: an estimate must never overwrite a
            // measurement.
            if design.scored_by_fem {
                continue;
            }
            design.generation = generation;
            if problem.goal == DesignGoal::RemoveHeat {
                score_by_conduction(&context, design);
            } else {
                score_by_truss(&context, design);
            }
            result.evaluations += 1;
        }
        population.sort_by(by_fitness);
        let mut scored = 0;
        for design in &mut population {
            if scored >= fem_per_generation {
                break;
            }
            if !design.scored_by_fem {
                score_candidate(&context, design);
                scored += 1;
            }
        }
        result.fem_evaluations += scored;
        population.sort_by(by_fitness);
        // The flow solver is a budget spent on the leader at intervals, so what the search is
        // climbing towards is checked against the real thing.
        if uses_stream
            && stream_checks_left > 0
            && !population[0].checked_by_flow_solver
            && (generation % check_every == 0 || generation + 1 == generations)
        {
            let before = context.estimate_correction;
            let mut leader = population[0].clone();
            verify_by_stream(&mut context, &mut leader, &mut result.stream_checks);
            population[0] = leader;
            stream_checks_left -= 1;
            if (context.estimate_correction - before).abs() > 0.05 * before {
                for design in &mut population {
                    if !design.checked_by_flow_solver {
                        design.scored_by_fem = false;
                    }
                }
                for design in &mut population {
                    if !design.scored_by_fem {
                        score_by_conduction(&context, design);
                        result.evaluations += 1;
                    }
                }
            }
            population.sort_by(by_fitness);
        }
        scored_history.extend(population.iter().filter(|d| d.scored_by_fem).cloned());
        if scored_history.len() > 20_000 {
            scored_history.sort_by(|a, b| b.feasible.cmp(&a.feasible).then(a.fitness.total_cmp(&b.fitness)));
            scored_history.truncate(5_000);
        }
        result.mass_history.push(population[0].mass);
        if let Some(progress) = progress.as_deref_mut()
            && !progress(generation, &population[0])
        {
            break;
        }
        if generation + 1 >= generations {
            break;
        }
        // One elite per topology, not one elite overall: a lineage that is merely behind is not
        // the same thing as one that is worse.
        let mut next: Vec<Design> = Vec::new();
        next.push(population[0].clone());
        if population.len() > 1 {
            next.push(population[1].clone());
        }
        let mut kept_shapes: Vec<usize> = next.iter().map(|d| d.members.len()).collect();
        for design in &population {
            if next.len() >= population_size / 3 {
                break;
            }
            if !kept_shapes.contains(&design.members.len()) {
                kept_shapes.push(design.members.len());
                next.push(design.clone());
            }
        }
        let allowed_stress = problem.yield_strength / problem.target_safety_factor.max(1e-9);
        while next.len() < population_size {
            let tournament = |rng: &mut Rng| -> usize {
                let n = population.len();
                rng.index(n).min(rng.index(n)).min(rng.index(n))
            };
            let mother_index = tournament(&mut rng);
            let father_index = tournament(&mut rng);
            let mother = &population[mother_index];
            let mut child = crossover(&mut rng, mother, &population[father_index]);
            // Sizing from the measurement: when the FEM has judged the parent overloaded, the
            // child starts from sections scaled to what the FEM measured.
            if mother.scored_by_fem && mother.max_stress > allowed_stress {
                let factor = (mother.max_stress / allowed_stress).sqrt().clamp(1.0, 1.8);
                for member in &mut child.members {
                    member.radius = (member.radius * factor).clamp(problem.min_radius, problem.max_radius);
                }
                for panel in &mut child.panels {
                    panel.thickness = (panel.thickness * factor).clamp(problem.min_radius, problem.max_radius);
                }
            }
            mutate(&mut rng, &mut child);
            if rng.unit() < 0.7 {
                size_members(problem, &mut child, mount_count);
            }
            child.scored_by_fem = false;
            child.feasible = false;
            next.push(child);
        }
        population = next;
    }

    // The answer: the best designs that are actually different from each other.
    scored_history.sort_by(|a, b| b.feasible.cmp(&a.feasible).then(a.fitness.total_cmp(&b.fitness)));
    let wanted = problem.solutions.clamp(1, 10);
    for design in &scored_history {
        if result.designs.len() >= wanted {
            break;
        }
        if !design_has_something_to_build(design) {
            continue;
        }
        let distinct = result.designs.iter().all(|kept| {
            if design_distance(design, kept, context.span) < 0.2 {
                return false;
            }
            let mass_gap = (design.mass - kept.mass).abs() / kept.mass.max(1e-12);
            let key_gap = if design.peak_temperature > 0.0 {
                (design.peak_temperature - kept.peak_temperature).abs() / kept.peak_temperature.max(1e-12)
            } else {
                (design.max_stress - kept.max_stress).abs() / kept.max_stress.max(1e-12)
            };
            !(mass_gap < 0.01 && key_gap < 0.01)
        });
        if distinct {
            result.designs.push(design.clone());
        }
    }
    if uses_stream && problem.stream.solver_checks > 0 {
        let mut designs = std::mem::take(&mut result.designs);
        for design in &mut designs {
            if !design.checked_by_flow_solver {
                verify_by_stream(&mut context, design, &mut result.stream_checks);
            }
        }
        designs.sort_by(|a, b| b.feasible.cmp(&a.feasible).then(a.fitness.total_cmp(&b.fitness)));
        result.designs = designs;
    }
    result.solved = !result.designs.is_empty();
    if !result.solved {
        result.advice = "no candidate joined the loads to the mounts -- widen the design space or raise the resolution".into();
        return result;
    }
    let any_feasible = result.designs.iter().any(|d| d.feasible);
    let mut advice = String::new();
    if !any_feasible {
        advice.push_str(&match problem.goal {
            DesignGoal::RemoveHeat => format!(
                "none of these holds the part under {:.1} C: raise the coolant flow, allow more room, or accept a hotter part. ",
                problem.temperature_limit - 273.15
            ),
            _ => format!(
                "none of these reaches a safety factor of {:.2}: allow thicker struts, more room, or a stronger material. ",
                problem.target_safety_factor
            ),
        });
    }
    if uses_stream {
        let checked = result.designs.iter().filter(|d| d.checked_by_flow_solver).count();
        if checked < result.designs.len() {
            advice.push_str(&format!(
                "the flow solver could not settle {} of these, so their numbers are the conduction estimate rather than a solved flow -- lower the stream velocity or raise the check resolution. ",
                result.designs.len() - checked
            ));
        }
    }
    if grid.assumed_space {
        advice.push_str("no design space was given, so the search used the box around the sketches grown by half its size. ");
    }
    if grid.snapped_regions {
        advice.push_str("an area is thinner than one voxel, so it was snapped to the nearest cells -- raise the resolution for a closer fit. ");
    }
    if result.designs.len() < wanted {
        advice.push_str(&format!(
            "only {} genuinely different design{} were found; asking for {wanted} would have meant returning near-copies.",
            result.designs.len(),
            if result.designs.len() == 1 { "" } else { "s" }
        ));
    }
    result.advice = advice.trim_end().to_string();
    result
}

/// The cheap thermal estimate for one candidate, exposed so it can be checked against the flow
/// solver directly.
pub fn estimate_peak_temperature(problem: &GenerativeProblem, grid: &DesignGrid, design: &Design) -> f64 {
    let solid = rasterise(grid, design);
    let mut sources = Vec::new();
    let mut watts = Vec::new();
    for (l, load) in problem.loads.iter().enumerate() {
        if load.watts <= 0.0 {
            continue;
        }
        if let Some(v) = grid.load_voxels.get(l) {
            sources.push(v.clone());
        }
        watts.push(load.watts);
    }
    let outcome = conduction_solve(problem, grid, &solid, &sources, &watts, false);
    if outcome.solved { outcome.peak_temperature } else { 0.0 }
}

/// The makeability scan on a design grid.
pub fn scan_design_makeability(grid: &DesignGrid, solid: &[bool], rules: &MakeabilityRules) -> MakeabilityScan {
    scan_makeability(&grid.frame(), solid, rules)
}

/// Real geometry for a chosen design, as a closed triangle mesh in mm: the design rasterised on
/// its grid (struts, plates and pads, trimmed to the design space) and skinned.
pub fn design_mesh(grid: &DesignGrid, design: &Design, smoothing_passes: usize) -> Option<Mesh> {
    if !design_has_something_to_build(design) {
        return None;
    }
    // The search drew struts on its coarse grid; the body is drawn on a finer one so that its
    // skin follows the round struts rather than the search's stair-steps, which a finer check
    // would otherwise read as notches.
    let factor = ((MAX_GRID_CELLS / 2) as f64 / grid.count().max(1) as f64).cbrt().floor().clamp(1.0, 4.0) as usize;
    let fine = grid.refined(factor);
    let mask = rasterise(&fine, design);
    mesh_from_mask(&mask, fine.nx, fine.ny, fine.nz, fine.h, fine.origin, smoothing_passes)
}

/// A one-line summary of a design, for a list the user chooses from.
pub fn describe_design(design: &Design) -> String {
    let mut text = format!("{:.1} g, ", design.mass * 1000.0);
    if design.peak_temperature > 0.0 {
        text.push_str(&format!("{:.1} C peak", design.peak_temperature - 273.15));
        if design.heat_removed > 0.0 {
            text.push_str(&format!(", {:.1} W into the coolant", design.heat_removed));
        }
        text.push_str(&format!(", {:.2} Pa", design.pressure_drop));
    } else {
        text.push_str(&format!("safety {:.2}, {:.3} mm deflection", design.safety_factor, design.max_displacement * 1000.0));
        if design.drag > 0.0 {
            text.push_str(&format!(", {:.3} N drag", design.drag));
        }
    }
    text.push_str(&format!(", {} struts", design.members.len()));
    if !design.panels.is_empty() {
        text.push_str(&format!(" and {} plates", design.panels.len()));
    }
    if !design.checked_by_flow_solver && (design.peak_temperature > 0.0 || design.drag > 0.0) {
        text.push_str(" (estimated)");
    }
    text
}

#[cfg(test)]
#[path = "generative_tests.rs"]
mod tests;
