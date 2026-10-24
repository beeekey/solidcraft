//! Implicit lattices: a body filled with a triply periodic minimal surface, a strut lattice or a
//! honeycomb, under a solid skin.
//!
//! The idea is implicit modelling's: describe the fill not by its boundary but by a scalar field over
//! space, negative inside and positive outside, and compose fields instead of doing booleans. A
//! lattice is a trigonometric expression, a strut is the distance to a segment, a skin is a
//! shell of the body's own distance field, and the body's interior clips all of it with a `max`.
//! Nothing here can fail the way a B-rep boolean fails: there is no topology to get wrong until
//! the very end, when the field is meshed by marching tetrahedra into a closed triangle surface.
//!
//! Units: millimetres throughout, like SolidCraft's meshes. The level-set value of a TPMS is
//! turned into a distance by its gradient, so wall thicknesses really are in millimetres.

use std::collections::HashMap;
use std::f64::consts::PI;

use solidcraft_geom::{Mesh, Vec3};

/// Most sample nodes one fill may evaluate. The field is sampled once per node and meshed from
/// the samples, so this bounds both time and memory.
pub const MAX_SAMPLES: usize = 4_000_000;
/// Samples per lattice cell along each axis, the useful range: four is very coarse, sixteen is
/// smooth and sixteen times the work of eight.
pub const MIN_SAMPLES_PER_CELL: usize = 4;
pub const MAX_SAMPLES_PER_CELL: usize = 16;

/// Triply periodic minimal surfaces by their usual names. Each is the zero set of a
/// trigonometric expression with period 2π on every axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TpmsType {
    Gyroid,
    SchwarzP,
    Diamond,
    Neovius,
    Lidinoid,
    SplitP,
    Iwp,
}

impl TpmsType {
    pub const ALL: [TpmsType; 7] =
        [TpmsType::Gyroid, TpmsType::SchwarzP, TpmsType::Diamond, TpmsType::Neovius, TpmsType::Lidinoid, TpmsType::SplitP, TpmsType::Iwp];

    /// The name a command uses: `gyroid`, `schwarz_p`, `diamond`, `neovius`, `lidinoid`,
    /// `split_p`, `iwp`.
    pub fn key(self) -> &'static str {
        match self {
            TpmsType::Gyroid => "gyroid",
            TpmsType::SchwarzP => "schwarz_p",
            TpmsType::Diamond => "diamond",
            TpmsType::Neovius => "neovius",
            TpmsType::Lidinoid => "lidinoid",
            TpmsType::SplitP => "split_p",
            TpmsType::Iwp => "iwp",
        }
    }

    /// By key or display name, case and separators ignored.
    pub fn from_name(name: &str) -> Option<TpmsType> {
        let n = normalise(name);
        TpmsType::ALL.iter().copied().find(|t| normalise(t.key()) == n || normalise(t.display()) == n)
    }

    pub fn display(self) -> &'static str {
        match self {
            TpmsType::Gyroid => "Gyroid",
            TpmsType::SchwarzP => "Schwarz P",
            TpmsType::Diamond => "Diamond",
            TpmsType::Neovius => "Neovius",
            TpmsType::Lidinoid => "Lidinoid",
            TpmsType::SplitP => "Split P",
            TpmsType::Iwp => "IWP",
        }
    }
}

fn normalise(s: &str) -> String {
    s.chars().filter(|c| c.is_ascii_alphanumeric()).map(|c| c.to_ascii_lowercase()).collect()
}

/// The raw expression at a point of the unit-period (2π) cell.
pub fn tpms_value(kind: TpmsType, x: f64, y: f64, z: f64) -> f64 {
    let (cx, cy, cz) = (x.cos(), y.cos(), z.cos());
    let (sx, sy, sz) = (x.sin(), y.sin(), z.sin());
    match kind {
        TpmsType::Gyroid => sx * cy + sy * cz + sz * cx,
        TpmsType::SchwarzP => cx + cy + cz,
        TpmsType::Diamond => sx * sy * sz + sx * cy * cz + cx * sy * cz + cx * cy * sz,
        TpmsType::Neovius => 3.0 * (cx + cy + cz) + 4.0 * cx * cy * cz,
        TpmsType::Lidinoid => {
            let (s2x, s2y, s2z) = ((2.0 * x).sin(), (2.0 * y).sin(), (2.0 * z).sin());
            let (c2x, c2y, c2z) = ((2.0 * x).cos(), (2.0 * y).cos(), (2.0 * z).cos());
            0.5 * (s2x * cy * sz + s2y * cz * sx + s2z * cx * sy) - 0.5 * (c2x * c2y + c2y * c2z + c2z * c2x) + 0.15
        }
        TpmsType::SplitP => {
            let (s2x, s2y, s2z) = ((2.0 * x).sin(), (2.0 * y).sin(), (2.0 * z).sin());
            let (c2x, c2y, c2z) = ((2.0 * x).cos(), (2.0 * y).cos(), (2.0 * z).cos());
            1.1 * (s2x * cy * sz + s2y * cz * sx + s2z * cx * sy) - 0.2 * (c2x * c2y + c2y * c2z + c2z * c2x) - 0.4 * (c2x + c2y + c2z)
        }
        TpmsType::Iwp => {
            let (c2x, c2y, c2z) = ((2.0 * x).cos(), (2.0 * y).cos(), (2.0 * z).cos());
            2.0 * (cx * cy + cy * cz + cz * cx) - (c2x + c2y + c2z)
        }
    }
}

/// The level-set value `f − level` of a TPMS of period `cell` at `p` (mm), turned into a
/// distance (mm) by its gradient: exact for a plane and close for anything smooth at the scale of
/// the finite-difference step.
pub fn tpms_distance(kind: TpmsType, cell: f64, level: f64, p: Vec3) -> f64 {
    let k = 2.0 * PI / cell;
    let f = |q: Vec3| tpms_value(kind, k * q.x, k * q.y, k * q.z);
    let value = f(p) - level;
    let step = 0.002 / k;
    let gradient = gradient_magnitude(&f, p, step).max(1e-6 * k);
    value / gradient
}

fn gradient_magnitude(f: &dyn Fn(Vec3) -> f64, p: Vec3, step: f64) -> f64 {
    let gx = (f(p + Vec3::new(step, 0.0, 0.0)) - f(p - Vec3::new(step, 0.0, 0.0))) / (2.0 * step);
    let gy = (f(p + Vec3::new(0.0, step, 0.0)) - f(p - Vec3::new(0.0, step, 0.0))) / (2.0 * step);
    let gz = (f(p + Vec3::new(0.0, 0.0, step)) - f(p - Vec3::new(0.0, 0.0, step))) / (2.0 * step);
    (gx * gx + gy * gy + gz * gz).sqrt()
}

/// Strut unit cells, the graph lattices: a set of straight struts in a unit cube, tiled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BeamCell {
    SimpleCubic,
    BodyCentred,
    FaceCentred,
    Octet,
    Diamond,
    Kelvin,
}

impl BeamCell {
    pub const ALL: [BeamCell; 6] =
        [BeamCell::SimpleCubic, BeamCell::BodyCentred, BeamCell::FaceCentred, BeamCell::Octet, BeamCell::Diamond, BeamCell::Kelvin];

    pub fn key(self) -> &'static str {
        match self {
            BeamCell::SimpleCubic => "simple_cubic",
            BeamCell::BodyCentred => "body_centred",
            BeamCell::FaceCentred => "face_centred",
            BeamCell::Octet => "octet",
            BeamCell::Diamond => "diamond",
            BeamCell::Kelvin => "kelvin",
        }
    }

    pub fn display(self) -> &'static str {
        match self {
            BeamCell::SimpleCubic => "Simple cubic",
            BeamCell::BodyCentred => "Body-centred cubic",
            BeamCell::FaceCentred => "Face-centred cubic",
            BeamCell::Octet => "Octet",
            BeamCell::Diamond => "Diamond",
            BeamCell::Kelvin => "Kelvin",
        }
    }

    pub fn from_name(name: &str) -> Option<BeamCell> {
        let n = normalise(name);
        BeamCell::ALL.iter().copied().find(|c| {
            normalise(c.key()) == n
                || normalise(c.display()) == n
                || (n == "bcc" && *c == BeamCell::BodyCentred)
                || (n == "fcc" && *c == BeamCell::FaceCentred)
                || (n == "bodycentered" && *c == BeamCell::BodyCentred)
                || (n == "facecentered" && *c == BeamCell::FaceCentred)
        })
    }
}

fn cube_corners() -> [Vec3; 8] {
    let mut out = [Vec3::ZERO; 8];
    for (i, c) in out.iter_mut().enumerate() {
        *c = Vec3::new((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64);
    }
    out
}

fn face_centres() -> [Vec3; 6] {
    [
        Vec3::new(0.0, 0.5, 0.5),
        Vec3::new(1.0, 0.5, 0.5),
        Vec3::new(0.5, 0.0, 0.5),
        Vec3::new(0.5, 1.0, 0.5),
        Vec3::new(0.5, 0.5, 0.0),
        Vec3::new(0.5, 0.5, 1.0),
    ]
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

/// The struts of one cell in unit-cube coordinates.
pub fn beam_cell_struts(cell: BeamCell) -> Vec<(Vec3, Vec3)> {
    let corners = cube_corners();
    let mut struts: Vec<(Vec3, Vec3)> = Vec::new();
    match cell {
        BeamCell::SimpleCubic => {
            for (i, a) in corners.iter().enumerate() {
                for b in corners.iter().skip(i + 1) {
                    if near(a.dist(*b), 1.0) {
                        struts.push((*a, *b));
                    }
                }
            }
        }
        BeamCell::BodyCentred => {
            for c in corners {
                struts.push((c, Vec3::new(0.5, 0.5, 0.5)));
            }
        }
        BeamCell::FaceCentred | BeamCell::Octet => {
            let centres = face_centres();
            for centre in centres {
                for corner in corners {
                    // The four corners of that face.
                    if near(corner.dist(centre), 0.5_f64.sqrt()) {
                        struts.push((corner, centre));
                    }
                }
            }
            if cell == BeamCell::Octet {
                for (i, a) in centres.iter().enumerate() {
                    for b in centres.iter().skip(i + 1) {
                        if near(a.dist(*b), 0.5_f64.sqrt()) {
                            struts.push((*a, *b));
                        }
                    }
                }
            }
        }
        BeamCell::Diamond => {
            // The diamond cubic: an FCC sub-lattice A and its copy B shifted by a quarter
            // diagonal, every B point bonded to its four nearest A points.
            let base_a = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.5, 0.5), Vec3::new(0.5, 0.0, 0.5), Vec3::new(0.5, 0.5, 0.0)];
            let mut a = Vec::new();
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        for p in base_a {
                            a.push(p + Vec3::new(dx as f64, dy as f64, dz as f64));
                        }
                    }
                }
            }
            for p in base_a {
                let b = p + Vec3::new(0.25, 0.25, 0.25);
                for q in &a {
                    if near(q.dist(b), 3.0_f64.sqrt() / 4.0) {
                        struts.push((b, *q));
                    }
                }
            }
        }
        BeamCell::Kelvin => {
            // The truncated octahedron: vertices at the permutations of (0, ±1/4, ±1/2), edges
            // between vertices √2/4 apart. They tile space body-centred, so a copy sits at the
            // cube's centre and one at its corner.
            let mut vertices: Vec<Vec3> = Vec::new();
            let orders = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
            for order in orders {
                for sa in [-1.0, 1.0] {
                    for sb in [-1.0, 1.0] {
                        let mut c = [0.0; 3];
                        c[order[1]] = sa * 0.25;
                        c[order[2]] = sb * 0.5;
                        let candidate = Vec3::new(c[0], c[1], c[2]);
                        if !vertices.iter().any(|v| v.dist(candidate) < 1e-9) {
                            vertices.push(candidate);
                        }
                    }
                }
            }
            for centre in [Vec3::new(0.5, 0.5, 0.5), Vec3::ZERO] {
                for (i, a) in vertices.iter().enumerate() {
                    for b in vertices.iter().skip(i + 1) {
                        if near(a.dist(*b), 2.0_f64.sqrt() / 4.0) {
                            struts.push((*a + centre, *b + centre));
                        }
                    }
                }
            }
        }
    }
    struts
}

/// The cell's struts plus those of the neighbouring cells that come within `margin` of the unit
/// cube, so a point near a face sees the strut on the far side of it too.
fn extended_struts(cell: BeamCell, margin: f64) -> Vec<(Vec3, Vec3)> {
    let base = beam_cell_struts(cell);
    let mut extended: Vec<(Vec3, Vec3)> = Vec::new();
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                let shift = Vec3::new(dx as f64, dy as f64, dz as f64);
                for (p, q) in &base {
                    let (a, b) = (*p + shift, *q + shift);
                    let (lo, hi) = (a.min(b), a.max(b));
                    if hi.x < -margin || lo.x > 1.0 + margin || hi.y < -margin || lo.y > 1.0 + margin || hi.z < -margin || lo.z > 1.0 + margin {
                        continue;
                    }
                    let duplicate =
                        extended.iter().any(|(e0, e1)| (e0.dist(a) < 1e-9 && e1.dist(b) < 1e-9) || (e0.dist(b) < 1e-9 && e1.dist(a) < 1e-9));
                    if !duplicate {
                        extended.push((a, b));
                    }
                }
            }
        }
    }
    extended
}

/// Distance (mm) from `p` to the nearest strut axis of a beam lattice of `cell_mm`, whose
/// struts are given by `extended_struts`.
fn beam_distance(struts: &[(Vec3, Vec3)], cell_mm: f64, p: Vec3) -> f64 {
    let local = Vec3::new(frac(p.x / cell_mm), frac(p.y / cell_mm), frac(p.z / cell_mm));
    let mut nearest = f64::INFINITY;
    for (a, b) in struts {
        nearest = nearest.min(local.dist_to_segment(*a, *b));
    }
    nearest * cell_mm
}

fn frac(x: f64) -> f64 {
    x - x.floor()
}

/// Signed distance (mm) to the mid-surface of a hexagonal honeycomb of `cell_mm` across the
/// flats, its cells running along `axis` (0 = X, 1 = Y, 2 = Z): zero on the cell walls' middle.
fn honeycomb_wall_distance(cell_mm: f64, axis: usize, p: Vec3) -> f64 {
    let circumradius = cell_mm / 3.0_f64.sqrt();
    let inradius = cell_mm / 2.0;
    let (u, v) = match axis {
        0 => (p.y, p.z),
        1 => (p.z, p.x),
        _ => (p.x, p.y),
    };
    // Flat-top axial coordinates, rounded to the nearest cell centre.
    let q = (2.0 / 3.0 * u) / circumradius;
    let r = (-1.0 / 3.0 * u + 3.0_f64.sqrt() / 3.0 * v) / circumradius;
    let s = -q - r;
    let (mut rq, mut rr, rs) = (q.round(), r.round(), s.round());
    let (dq, dr, ds) = ((rq - q).abs(), (rr - r).abs(), (rs - s).abs());
    if dq > dr && dq > ds {
        rq = -rr - rs;
    } else if dr > ds {
        rr = -rq - rs;
    }
    let cu = circumradius * 1.5 * rq;
    let cv = circumradius * 3.0_f64.sqrt() * (rr + rq / 2.0);
    let (ou, ov) = ((u - cu).abs(), (v - cv).abs());
    let norm = ov.max(ou * 3.0_f64.sqrt() / 2.0 + ov / 2.0);
    (norm - inradius).abs()
}

/// What fills the body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Family {
    /// A sheet TPMS: the mid-surface thickened to the wall thickness. Two interpenetrating
    /// channels separated by one continuous wall, the heat-exchanger form.
    WalledTpms(TpmsType),
    /// A solid-network TPMS: the region `f < level`. `level` in about −1.4..1.4 for the gyroid
    /// sets the volume fraction; 0 is half full.
    SolidTpms(TpmsType),
    /// Struts of the given diameter along a cell's edges, rounded where they meet.
    Beam(BeamCell),
    /// A hexagonal honeycomb with its cells along an axis (0 = X, 1 = Y, 2 = Z).
    Honeycomb { axis: usize },
}

impl Family {
    pub fn key(self) -> &'static str {
        match self {
            Family::WalledTpms(_) => "walled_tpms",
            Family::SolidTpms(_) => "solid_tpms",
            Family::Beam(_) => "beam",
            Family::Honeycomb { .. } => "honeycomb",
        }
    }

    pub fn cell_name(self) -> &'static str {
        match self {
            Family::WalledTpms(t) | Family::SolidTpms(t) => t.display(),
            Family::Beam(c) => c.display(),
            Family::Honeycomb { .. } => "Hexagonal",
        }
    }
}

/// How the wall thickness (or strut diameter) varies through the body.
pub enum Grading<'a> {
    /// `thickness_mm` everywhere.
    Uniform,
    /// From `thickness_min_mm` at `from` to `thickness_max_mm` at `to`, along the line between
    /// them, clamped beyond either end.
    AlongAxis { from: Vec3, to: Vec3 },
    /// From `thickness_min_mm` at `centre` to `thickness_max_mm` at `radius` from it.
    FromPoint { centre: Vec3, radius: f64 },
    /// A caller's field in 0..1 (0 → `thickness_min_mm`, 1 → `thickness_max_mm`): a stress
    /// result, say.
    Field(&'a dyn Fn(Vec3) -> f64),
}

/// What to fill a body with.
pub struct LatticeOptions<'a> {
    pub family: Family,
    /// Period of the lattice (mm).
    pub cell_mm: f64,
    /// Wall thickness or strut diameter (mm) with a uniform grading.
    pub thickness_mm: f64,
    /// The graded range (mm); ignored by `Grading::Uniform`.
    pub thickness_min_mm: f64,
    pub thickness_max_mm: f64,
    /// Solid TPMS only: the level of the surface.
    pub level: f64,
    /// A solid skin this thick inside the body's surface (mm); 0 for none.
    pub skin_mm: f64,
    pub grading: Grading<'a>,
    /// Samples per cell along each axis (`MIN_SAMPLES_PER_CELL..=MAX_SAMPLES_PER_CELL`).
    pub samples_per_cell: usize,
}

impl Default for LatticeOptions<'_> {
    fn default() -> Self {
        LatticeOptions {
            family: Family::WalledTpms(TpmsType::Gyroid),
            cell_mm: 10.0,
            thickness_mm: 1.0,
            thickness_min_mm: 0.5,
            thickness_max_mm: 2.0,
            level: 0.0,
            skin_mm: 0.0,
            grading: Grading::Uniform,
            samples_per_cell: 8,
        }
    }
}

/// The filled body.
#[derive(Debug, Clone, Default)]
pub struct LatticeResult {
    /// The surface of the fill, a closed triangle mesh (mm).
    pub mesh: Mesh,
    /// The fill's volume over the body's: the meshed surface's volume where the body's own
    /// volume is known, else the mean coverage of the body's samples.
    pub volume_fraction: f64,
    pub triangles: usize,
    /// Sample nodes along each axis, their spacing (mm) and the position of node (0, 0, 0).
    pub grid: [usize; 3],
    pub h_mm: f64,
    pub origin: Vec3,
    /// Per node: inside the body.
    pub inside: Vec<bool>,
    /// Per node: how much of the node's cell the fill occupies, 0 to 1 (the signed distance
    /// turned into a coverage, which does not alias the way counting nodes does).
    pub coverage: Vec<f32>,
    /// Empty when nothing is wrong; otherwise what to change.
    pub advice: String,
}

impl LatticeResult {
    /// Node index.
    pub fn index(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.grid[1] + j) * self.grid[0] + i
    }

    /// Position of a node (mm).
    pub fn node(&self, n: usize) -> Vec3 {
        let (nx, ny) = (self.grid[0].max(1), self.grid[1].max(1));
        let (i, j, k) = (n % nx, (n / nx) % ny, n / (nx * ny));
        self.origin + Vec3::new(i as f64, j as f64, k as f64) * self.h_mm
    }

    /// The fraction of body samples that are filled among those for which `pick` is true: the
    /// density of the fill in one region of the body.
    pub fn fraction_where(&self, pick: impl Fn(Vec3) -> bool) -> f64 {
        let (mut body, mut filled) = (0usize, 0.0_f64);
        for n in 0..self.inside.len() {
            if self.inside.get(n).copied().unwrap_or(false) && pick(self.node(n)) {
                body += 1;
                filled += f64::from(self.coverage.get(n).copied().unwrap_or(0.0));
            }
        }
        if body == 0 { 0.0 } else { filled / body as f64 }
    }
}

/// Felzenszwalb's one-dimensional squared distance transform, in place: `f` holds 0 at feature
/// samples and a large value elsewhere; on return it holds the squared distance (in samples) to
/// the nearest feature.
fn distance_transform_1d(f: &mut [f64], d: &mut Vec<f64>, v: &mut Vec<usize>, z: &mut Vec<f64>) {
    let n = f.len();
    if n == 0 {
        return;
    }
    v.clear();
    v.resize(n, 0);
    z.clear();
    z.resize(n + 1, 0.0);
    d.clear();
    d.resize(n, 0.0);
    let mut k = 0usize;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    let intersection = |f: &[f64], q: usize, p: usize| -> f64 {
        let (fq, fp) = (f.get(q).copied().unwrap_or(FAR), f.get(p).copied().unwrap_or(FAR));
        ((fq + (q * q) as f64) - (fp + (p * p) as f64)) / (2.0 * q as f64 - 2.0 * p as f64)
    };
    for q in 1..n {
        let mut s = intersection(f, q, v[k]);
        while k > 0 && s <= z[k] {
            k -= 1;
            s = intersection(f, q, v[k]);
        }
        k += 1;
        v[k] = q;
        z[k] = s;
        if k + 1 < z.len() {
            z[k + 1] = f64::INFINITY;
        }
    }
    k = 0;
    for (q, out) in d.iter_mut().enumerate() {
        while k + 1 < z.len() && z[k + 1] < q as f64 {
            k += 1;
        }
        let vk = v[k];
        let dq = q as f64 - vk as f64;
        *out = dq * dq + f.get(vk).copied().unwrap_or(FAR);
    }
    f.copy_from_slice(d);
}

/// "Not a feature": behaves like infinity in the parabola intersections without making NaNs.
const FAR: f64 = 1e20;

/// Squared Euclidean distance (in samples) from every node to the nearest node where `feature`
/// is true, separably along the three axes.
pub(crate) fn distance_transform_3d(feature: &[bool], nx: usize, ny: usize, nz: usize) -> Vec<f64> {
    let mut f: Vec<f64> = feature.iter().map(|&b| if b { 0.0 } else { FAR }).collect();
    let (mut line, mut d, mut v, mut z) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    for k in 0..nz {
        for j in 0..ny {
            line.clear();
            line.extend((0..nx).map(|i| f[at(i, j, k)]));
            distance_transform_1d(&mut line, &mut d, &mut v, &mut z);
            for i in 0..nx {
                f[at(i, j, k)] = line[i];
            }
        }
    }
    for k in 0..nz {
        for i in 0..nx {
            line.clear();
            line.extend((0..ny).map(|j| f[at(i, j, k)]));
            distance_transform_1d(&mut line, &mut d, &mut v, &mut z);
            for j in 0..ny {
                f[at(i, j, k)] = line[j];
            }
        }
    }
    for j in 0..ny {
        for i in 0..nx {
            line.clear();
            line.extend((0..nz).map(|k| f[at(i, j, k)]));
            distance_transform_1d(&mut line, &mut d, &mut v, &mut z);
            for k in 0..nz {
                f[at(i, j, k)] = line[k];
            }
        }
    }
    f
}

/// Fills a closed body (a mesh in mm) with a lattice. `None` for an empty or degenerate body,
/// impossible options (a cell or thickness that is not a positive finite number), or a fill that
/// produced no surface.
pub fn fill(body: &Mesh, options: &LatticeOptions<'_>) -> Option<LatticeResult> {
    let finite_positive = |x: f64| x.is_finite() && x > 0.0;
    if body.triangles.is_empty()
        || !finite_positive(options.cell_mm)
        || !options.level.is_finite()
        || !options.skin_mm.is_finite()
        || options.skin_mm < 0.0
    {
        return None;
    }
    let thickness_ok = match options.grading {
        Grading::Uniform => finite_positive(options.thickness_mm),
        _ => {
            finite_positive(options.thickness_min_mm)
                && finite_positive(options.thickness_max_mm)
                && options.thickness_max_mm >= options.thickness_min_mm
        }
    };
    if !thickness_ok && !matches!(options.family, Family::SolidTpms(_)) {
        return None;
    }
    if let Grading::AlongAxis { from, to } = options.grading
        && (!from.is_finite() || !to.is_finite())
    {
        return None;
    }
    if let Grading::FromPoint { centre, radius } = options.grading
        && (!centre.is_finite() || !finite_positive(radius))
    {
        return None;
    }
    if let Family::Honeycomb { axis } = options.family
        && axis > 2
    {
        return None;
    }
    let bounds = body.bounds();
    if bounds.is_empty() {
        return None;
    }
    let size = bounds.size();
    if !size.is_finite() || !(size.x > 0.0 && size.y > 0.0 && size.z > 0.0) {
        return None;
    }
    let mut advice = Vec::new();
    let samples_per_cell = options.samples_per_cell.clamp(MIN_SAMPLES_PER_CELL, MAX_SAMPLES_PER_CELL);
    if samples_per_cell < 6 {
        advice.push(format!("{samples_per_cell} samples per cell is coarse: the walls will look faceted; 8 or more are smooth"));
    }
    let mut h = options.cell_mm / samples_per_cell as f64;
    // Two nodes of margin on every side, so the surface closes outside the body.
    let count_for = |h: f64| -> Option<usize> {
        let n = |extent: f64| ((extent / h).ceil() as usize).checked_add(4);
        n(size.x)?.checked_mul(n(size.y)?)?.checked_mul(n(size.z)?)
    };
    let mut count = count_for(h)?;
    if count > MAX_SAMPLES {
        let asked = h;
        // The margin nodes keep the cube-root estimate a little short; widen until it fits.
        for _ in 0..8 {
            h *= (count as f64 / MAX_SAMPLES as f64).cbrt().max(1.0) * 1.01;
            count = count_for(h)?;
            if count <= MAX_SAMPLES {
                break;
            }
        }
        if count > MAX_SAMPLES {
            return None;
        }
        advice.push(format!(
            "resolution limited: the sample spacing was widened from {asked:.3} mm to {h:.3} mm to stay within {MAX_SAMPLES} samples; a coarser cell or fewer samples per cell keeps the detail"
        ));
    }
    let nx = (size.x / h).ceil() as usize + 4;
    let ny = (size.y / h).ceil() as usize + 4;
    let nz = (size.z / h).ceil() as usize + 4;
    // The nodes span the body with at least a sample and a half of margin on every side.
    let span = Vec3::new((nx - 1) as f64, (ny - 1) as f64, (nz - 1) as f64) * h;
    let origin = bounds.min - (span - size) * 0.5;
    let node = |n: usize| -> Vec3 {
        let (i, j, k) = (n % nx, (n / nx) % ny, n / (nx * ny));
        origin + Vec3::new(i as f64, j as f64, k as f64) * h
    };

    // Inside/outside at every node, then the body's signed distance from the two distance
    // transforms, the surface placed halfway between the last inside and the first outside node.
    let index = body.inside_index();
    let inside: Vec<bool> = (0..count).map(|n| index.contains(node(n))).collect();
    let body_samples = inside.iter().filter(|b| **b).count();
    if body_samples == 0 {
        return None;
    }
    let outside: Vec<bool> = inside.iter().map(|b| !b).collect();
    let to_outside = distance_transform_3d(&outside, nx, ny, nz);
    let to_inside = distance_transform_3d(&inside, nx, ny, nz);
    let body_distance: Vec<f64> =
        (0..count).map(|n| if inside[n] { -(to_outside[n].sqrt() - 0.5) * h } else { (to_inside[n].sqrt() - 0.5) * h }).collect();

    let thinnest = size.x.min(size.y).min(size.z);
    if options.cell_mm > thinnest {
        advice.push(format!(
            "the cell ({:.1} mm) is larger than the body's thinnest side ({thinnest:.1} mm): less than one cell fits across it",
            options.cell_mm
        ));
    }
    let thinnest_wall = match options.grading {
        Grading::Uniform => options.thickness_mm,
        _ => options.thickness_min_mm,
    };
    if !matches!(options.family, Family::SolidTpms(_)) && thinnest_wall < 2.0 * h {
        advice.push(format!(
            "walls of {thinnest_wall:.2} mm are thinner than two samples ({:.2} mm): they will come out ragged or vanish; raise samples_per_cell or the thickness",
            2.0 * h
        ));
    }
    if options.skin_mm > 0.0 && options.skin_mm < h {
        advice.push(format!("a skin of {:.2} mm is thinner than one sample ({h:.2} mm) and will be patchy", options.skin_mm));
    }

    let thickness_at = |p: Vec3| -> f64 {
        let (lo, hi) = (options.thickness_min_mm, options.thickness_max_mm);
        match &options.grading {
            Grading::Uniform => options.thickness_mm,
            Grading::AlongAxis { from, to } => {
                let delta = *to - *from;
                let span = delta.len();
                let along = if span > 0.0 { (p - *from).dot(delta) / (span * span) } else { 0.0 };
                lo + (hi - lo) * along.clamp(0.0, 1.0)
            }
            Grading::FromPoint { centre, radius } => lo + (hi - lo) * (p.dist(*centre) / *radius).clamp(0.0, 1.0),
            Grading::Field(f) => {
                let t = f(p);
                lo + (hi - lo) * if t.is_finite() { t.clamp(0.0, 1.0) } else { 0.0 }
            }
        }
    };
    let struts = match options.family {
        Family::Beam(cell) => extended_struts(cell, 0.35),
        _ => Vec::new(),
    };
    let lattice_at = |p: Vec3| -> f64 {
        match options.family {
            Family::WalledTpms(kind) => tpms_distance(kind, options.cell_mm, 0.0, p).abs() - thickness_at(p) / 2.0,
            Family::SolidTpms(kind) => tpms_distance(kind, options.cell_mm, options.level, p),
            Family::Beam(_) => beam_distance(&struts, options.cell_mm, p) - thickness_at(p) / 2.0,
            Family::Honeycomb { axis } => honeycomb_wall_distance(options.cell_mm, axis, p) - thickness_at(p) / 2.0,
        }
    };

    // The combined field: the lattice clipped to the body, united with the body's skin.
    let mut field: Vec<f64> = vec![0.0; count];
    let mut coverage: Vec<f32> = vec![0.0; count];
    let mut covered = 0.0_f64;
    for n in 0..count {
        let d = body_distance[n];
        // Nothing to evaluate far outside: the field is the body's distance there.
        let value = if d > 2.0 * h {
            d
        } else {
            let p = node(n);
            let lattice = d.max(lattice_at(p));
            if options.skin_mm > 0.0 { lattice.min(d.max(-(d + options.skin_mm))) } else { lattice }
        };
        field[n] = if value.is_finite() { value } else { d };
        if inside[n] {
            let c = (0.5 - field[n] / h).clamp(0.0, 1.0);
            coverage[n] = c as f32;
            covered += c;
        }
    }

    let mesh = match marching_tetrahedra(&field, nx, ny, nz, origin, h) {
        Some(mesh) => mesh,
        None => {
            advice.push("the fill left no material: the walls are too thin for the samples, or the cell is too big for the body".to_string());
            Mesh::default()
        }
    };
    let triangles = mesh.triangles.len();
    let body_volume = body.measure().volume.abs();
    let volume_fraction =
        if body_volume > 0.0 && triangles > 0 { (mesh.measure().volume / body_volume).clamp(0.0, 1.0) } else { covered / body_samples as f64 };
    Some(LatticeResult { mesh, volume_fraction, triangles, grid: [nx, ny, nz], h_mm: h, origin, inside, coverage, advice: advice.join("; ") })
}

/// The zero surface of a field sampled at the nodes of a regular grid, as a closed, welded,
/// outward-wound triangle mesh. Each cube is split into the six tetrahedra around its main
/// diagonal (the same diagonal in every cube, so neighbouring cubes share their faces and the
/// surface is watertight); each tetrahedron's crossing is one or two triangles with vertices
/// interpolated along its edges. Negative is inside.
pub fn marching_tetrahedra(field: &[f64], nx: usize, ny: usize, nz: usize, origin: Vec3, h: f64) -> Option<Mesh> {
    let count = nx.checked_mul(ny)?.checked_mul(nz)?;
    if count == 0 || field.len() != count || nx < 2 || ny < 2 || nz < 2 || !(h > 0.0) || !h.is_finite() {
        return None;
    }
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    let node = |n: usize| -> Vec3 {
        let (i, j, k) = (n % nx, (n / nx) % ny, n / (nx * ny));
        origin + Vec3::new(i as f64, j as f64, k as f64) * h
    };
    let inside = |n: usize| field.get(n).is_some_and(|v| *v < 0.0);
    let mut positions: Vec<Vec3> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut vertex_of: HashMap<(u32, u32), u32> = HashMap::new();
    let mut vertex_on = |a: usize, b: usize, positions: &mut Vec<Vec3>| -> Option<u32> {
        let key = (a.min(b) as u32, a.max(b) as u32);
        if let Some(v) = vertex_of.get(&key) {
            return Some(*v);
        }
        let (fa, fb) = (*field.get(a)?, *field.get(b)?);
        let t = if (fa - fb).abs() > 1e-300 { (fa / (fa - fb)).clamp(0.0, 1.0) } else { 0.5 };
        let p = node(a).lerp(node(b), t);
        let v = u32::try_from(positions.len()).ok()?;
        positions.push(p);
        vertex_of.insert(key, v);
        Some(v)
    };
    // The six tetrahedra around the diagonal from corner 0 (0,0,0) to corner 7 (1,1,1); corner
    // bits are x + 2y + 4z.
    const TETS: [[usize; 4]; 6] = [[0, 1, 3, 7], [0, 3, 2, 7], [0, 2, 6, 7], [0, 6, 4, 7], [0, 4, 5, 7], [0, 5, 1, 7]];
    // Winding is decided from the grid nodes, never from the interpolated points, which may
    // coincide when the field is exactly zero at a node.
    let mut emit = |a: u32, b: u32, c: u32, outward: bool| {
        if outward { triangles.push([a, b, c]) } else { triangles.push([a, c, b]) }
    };
    for k in 0..nz - 1 {
        for j in 0..ny - 1 {
            for i in 0..nx - 1 {
                let corners = [
                    at(i, j, k),
                    at(i + 1, j, k),
                    at(i, j + 1, k),
                    at(i + 1, j + 1, k),
                    at(i, j, k + 1),
                    at(i + 1, j, k + 1),
                    at(i, j + 1, k + 1),
                    at(i + 1, j + 1, k + 1),
                ];
                let signs: Vec<bool> = corners.iter().map(|c| inside(*c)).collect();
                if signs.iter().all(|s| *s) || !signs.iter().any(|s| *s) {
                    continue;
                }
                for tet in TETS {
                    let ins: Vec<usize> = tet.iter().copied().filter(|c| signs[*c]).collect();
                    let outs: Vec<usize> = tet.iter().copied().filter(|c| !signs[*c]).collect();
                    if ins.is_empty() || outs.is_empty() {
                        continue;
                    }
                    let pos = |c: usize| node(corners[c]);
                    match (ins.len(), outs.len()) {
                        (1, 3) | (3, 1) => {
                            let apex_inside = ins.len() == 1;
                            let (apex, base) = if apex_inside { (ins[0], &outs) } else { (outs[0], &ins) };
                            let (Some(b0), Some(b1), Some(b2)) = (base.first(), base.get(1), base.get(2)) else { continue };
                            let v0 = vertex_on(corners[apex], corners[*b0], &mut positions)?;
                            let v1 = vertex_on(corners[apex], corners[*b1], &mut positions)?;
                            let v2 = vertex_on(corners[apex], corners[*b2], &mut positions)?;
                            // det > 0: the base b0 b1 b2 is wound with its normal away from the apex.
                            let (p, q0, q1, q2) = (pos(apex), pos(*b0), pos(*b1), pos(*b2));
                            let away_from_apex = (q0 - p).dot((q1 - p).cross(q2 - p)) > 0.0;
                            // Outward is away from an inside apex, towards an outside one.
                            emit(v0, v1, v2, away_from_apex == apex_inside);
                        }
                        (2, 2) => {
                            let (Some(i0), Some(i1), Some(o0), Some(o1)) = (ins.first(), ins.get(1), outs.first(), outs.get(1)) else { continue };
                            let v00 = vertex_on(corners[*i0], corners[*o0], &mut positions)?;
                            let v01 = vertex_on(corners[*i0], corners[*o1], &mut positions)?;
                            let v10 = vertex_on(corners[*i1], corners[*o0], &mut positions)?;
                            let v11 = vertex_on(corners[*i1], corners[*o1], &mut positions)?;
                            // The quad v00 v01 v11 v10 at the edges' midpoints has the normal
                            // (O1−O0) × (I1−I0); outward points from the inside pair to the outside.
                            let (pi0, pi1, po0, po1) = (pos(*i0), pos(*i1), pos(*o0), pos(*o1));
                            let outward = (po1 - po0).cross(pi1 - pi0).dot(po0 + po1 - pi0 - pi1) > 0.0;
                            emit(v00, v01, v11, outward);
                            emit(v00, v11, v10, outward);
                        }
                        _ => {}
                    }
                }
            }
        }
    }
    if triangles.is_empty() {
        return None;
    }
    // Where the field is exactly zero at a node, several edges meet in one point: weld those
    // vertices and drop the triangles that collapsed to nothing.
    let mut canonical: HashMap<[u64; 3], u32> = HashMap::new();
    let mut remap: Vec<u32> = Vec::with_capacity(positions.len());
    let mut welded: Vec<Vec3> = Vec::with_capacity(positions.len());
    for p in &positions {
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let v = *canonical.entry(key).or_insert_with(|| {
            welded.push(*p);
            (welded.len() - 1) as u32
        });
        remap.push(v);
    }
    let triangles: Vec<[u32; 3]> = triangles
        .iter()
        .map(|t| {
            [
                remap.get(t[0] as usize).copied().unwrap_or(0),
                remap.get(t[1] as usize).copied().unwrap_or(0),
                remap.get(t[2] as usize).copied().unwrap_or(0),
            ]
        })
        .filter(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
        .collect();
    if triangles.is_empty() {
        return None;
    }
    let mut mesh = Mesh { positions: welded, triangles, ..Default::default() };
    mesh.normals = vec![Vec3::ZERO; mesh.positions.len()];
    for t in &mesh.triangles {
        if let Some([a, b, c]) = mesh.tri(t) {
            let n = (b - a).cross(c - a);
            for v in t {
                if let Some(slot) = mesh.normals.get_mut(*v as usize) {
                    *slot += n;
                }
            }
        }
    }
    for n in &mut mesh.normals {
        *n = n.normalized().unwrap_or(Vec3::Z);
    }
    mesh.tri_face = vec![0; mesh.triangles.len()];
    Some(mesh)
}
