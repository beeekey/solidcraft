//! Implicit modelling: a body as a signed distance sampled on a grid, so that two bodies can be
//! joined with a rounded seam, a body hollowed by a thickness that varies along it, or the cells
//! of an analysis turned back into a solid. The surface comes back as a mesh, through the same
//! marching tetrahedra the lattices use.

use solidcraft_geom::{Mesh, Vec3};

use crate::lattice::{distance_transform_3d, marching_tetrahedra};

/// The most samples one field may hold.
pub const MAX_SAMPLES: usize = 6_000_000;

/// A scalar field sampled at the nodes of a regular grid (mm): negative inside the material.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    /// Node (0, 0, 0) (mm).
    pub origin: Vec3,
    /// Node spacing (mm).
    pub h: f64,
    pub values: Vec<f64>,
}

impl Field {
    pub fn node(&self, n: usize) -> Vec3 {
        let (i, j, k) = (n % self.nx, (n / self.nx) % self.ny, n / (self.nx * self.ny));
        self.origin + Vec3::new(i as f64, j as f64, k as f64) * self.h
    }

    /// The same grid with every value replaced.
    pub fn map(&self, f: impl Fn(usize, f64) -> f64) -> Field {
        Field { values: self.values.iter().enumerate().map(|(n, v)| f(n, *v)).collect(), ..self.clone() }
    }

    /// The surface where the field crosses zero.
    pub fn mesh(&self) -> Option<Mesh> {
        marching_tetrahedra(&self.values, self.nx, self.ny, self.nz, self.origin, self.h)
    }

    /// How much of the grid is inside (negative), as a count of nodes.
    pub fn inside_nodes(&self) -> usize {
        self.values.iter().filter(|v| **v < 0.0).count()
    }
}

/// The grid that covers `min..=max` grown by `padding` on every side at spacing `h`, or `None`
/// when it would hold more than `max_samples` nodes or the box is degenerate.
fn grid_for(min: Vec3, max: Vec3, h: f64, padding: f64, max_samples: usize) -> Option<(usize, usize, usize, Vec3)> {
    if !(h > 0.0) || !h.is_finite() || !(padding >= 0.0) || !min.is_finite() || !max.is_finite() {
        return None;
    }
    let lo = min - Vec3::new(padding, padding, padding);
    let size = (max - min) + Vec3::new(2.0 * padding, 2.0 * padding, 2.0 * padding);
    if !(size.x > 0.0 && size.y > 0.0 && size.z > 0.0) {
        return None;
    }
    let n = |extent: f64| ((extent / h).ceil() as usize).checked_add(3);
    let (nx, ny, nz) = (n(size.x)?, n(size.y)?, n(size.z)?);
    if nx.checked_mul(ny)?.checked_mul(nz)? > max_samples {
        return None;
    }
    Some((nx, ny, nz, lo - Vec3::new(h, h, h)))
}

/// Signed distances to each mesh (mm), all on one grid covering them together, grown by `padding`.
/// `None` when a mesh is empty or the grid would be too fine.
pub fn sample_meshes(meshes: &[&Mesh], h: f64, padding: f64, max_samples: usize) -> Option<Vec<Field>> {
    if meshes.is_empty() || meshes.iter().any(|m| m.triangles.is_empty()) {
        return None;
    }
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for m in meshes {
        let b = m.bounds();
        min = Vec3::new(min.x.min(b.min.x), min.y.min(b.min.y), min.z.min(b.min.z));
        max = Vec3::new(max.x.max(b.max.x), max.y.max(b.max.y), max.z.max(b.max.z));
    }
    let (nx, ny, nz, origin) = grid_for(min, max, h, padding, max_samples)?;
    let count = nx * ny * nz;
    let node = |n: usize| -> Vec3 {
        let (i, j, k) = (n % nx, (n / nx) % ny, n / (nx * ny));
        origin + Vec3::new(i as f64, j as f64, k as f64) * h
    };
    let mut fields = Vec::with_capacity(meshes.len());
    for m in meshes {
        let index = m.inside_index();
        let inside: Vec<bool> = (0..count).map(|n| index.contains(node(n))).collect();
        if !inside.iter().any(|b| *b) {
            return None;
        }
        let outside: Vec<bool> = inside.iter().map(|b| !b).collect();
        let to_outside = distance_transform_3d(&outside, nx, ny, nz);
        let to_inside = distance_transform_3d(&inside, nx, ny, nz);
        let values = (0..count).map(|n| if inside[n] { -(to_outside[n].sqrt() - 0.5) * h } else { (to_inside[n].sqrt() - 0.5) * h }).collect();
        fields.push(Field { nx, ny, nz, origin, h, values });
    }
    Some(fields)
}

/// One mesh's signed distance field.
pub fn sample_mesh(mesh: &Mesh, h: f64, padding: f64, max_samples: usize) -> Option<Field> {
    sample_meshes(&[mesh], h, padding, max_samples)?.pop()
}

/// A minimum that rounds the crease between two surfaces over a width `k`; the plain minimum
/// when `k` is zero.
pub fn smooth_min(a: f64, b: f64, k: f64) -> f64 {
    if !(k > 0.0) {
        return a.min(b);
    }
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    b * (1.0 - h) + a * h - k * h * (1.0 - h)
}

/// How two fields are combined.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Boolean {
    Join,
    Cut,
    Intersect,
}

impl Boolean {
    pub fn from_name(name: &str) -> Option<Boolean> {
        match name.trim().to_ascii_lowercase().as_str() {
            "join" | "union" | "unite" | "fuse" => Some(Boolean::Join),
            "cut" | "subtract" | "difference" => Some(Boolean::Cut),
            "intersect" | "intersection" | "common" => Some(Boolean::Intersect),
            _ => None,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Boolean::Join => "join",
            Boolean::Cut => "cut",
            Boolean::Intersect => "intersect",
        }
    }
}

/// `a` combined with `b` (same grid) with the seam rounded over `blend` (mm). `None` when the
/// grids differ.
pub fn blend(a: &Field, b: &Field, op: Boolean, blend: f64) -> Option<Field> {
    if a.values.len() != b.values.len() || a.nx != b.nx || a.ny != b.ny || a.nz != b.nz {
        return None;
    }
    let k = blend.max(0.0);
    Some(a.map(|n, da| {
        let db = b.values[n];
        match op {
            Boolean::Join => smooth_min(da, db, k),
            Boolean::Intersect => -smooth_min(-da, -db, k),
            Boolean::Cut => -smooth_min(-da, db, k),
        }
    }))
}

/// How a thickness varies over the body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Grading {
    Uniform(f64),
    /// From `min` at `from` to `max` at `to`, held outside.
    AlongAxis {
        from: Vec3,
        to: Vec3,
        min: f64,
        max: f64,
    },
    /// `min` at `centre`, `max` at `radius` and beyond.
    FromPoint {
        centre: Vec3,
        radius: f64,
        min: f64,
        max: f64,
    },
}

impl Grading {
    pub fn at(&self, p: Vec3) -> f64 {
        match *self {
            Grading::Uniform(t) => t,
            Grading::AlongAxis { from, to, min, max } => {
                let axis = to - from;
                let len2 = axis.dot(axis);
                let t = if len2 > 0.0 { ((p - from).dot(axis) / len2).clamp(0.0, 1.0) } else { 0.0 };
                min + (max - min) * t
            }
            Grading::FromPoint { centre, radius, min, max } => {
                let t = if radius > 0.0 { ((p - centre).len() / radius).clamp(0.0, 1.0) } else { 0.0 };
                min + (max - min) * t
            }
        }
    }

    /// The largest thickness the grading reaches.
    pub fn reach(&self) -> f64 {
        match *self {
            Grading::Uniform(t) => t.abs(),
            Grading::AlongAxis { min, max, .. } | Grading::FromPoint { min, max, .. } => min.abs().max(max.abs()),
        }
    }
}

/// The body's skin: material within `thickness` of the surface, inside.
pub fn shell(body: &Field, thickness: &Grading) -> Field {
    body.map(|n, d| {
        let t = thickness.at(body.node(n));
        d.max(-(d + t))
    })
}

/// The body grown by `distance` (shrunk when negative).
pub fn offset(body: &Field, distance: &Grading) -> Field {
    body.map(|n, d| d - distance.at(body.node(n)))
}

/// The cells of a voxel grid (`grid`, corner `origin`, cell `h`, both mm) that `keep` says yes
/// to, as a closed surface. The field is sampled at the cell centres (kept −½, empty +½, with an
/// empty layer around the grid), so the surface runs along the cell faces, a one-cell strut
/// keeps its full width, and only the corners are eased by half a cell.
pub fn cells_mesh(grid: [usize; 3], origin: Vec3, h: f64, keep: impl Fn(usize) -> bool) -> Option<Mesh> {
    let [nx, ny, nz] = grid;
    if nx == 0 || ny == 0 || nz == 0 || !(h > 0.0) {
        return None;
    }
    let (mx, my, mz) = (nx + 2, ny + 2, nz + 2);
    let field: Vec<f64> = (0..mx.checked_mul(my)?.checked_mul(mz)?)
        .map(|n| {
            let (i, j, k) = (n % mx, (n / mx) % my, n / (mx * my));
            if i == 0 || j == 0 || k == 0 || i > nx || j > ny || k > nz {
                return 0.5;
            }
            if keep(((k - 1) * ny + (j - 1)) * nx + (i - 1)) { -0.5 } else { 0.5 }
        })
        .collect();
    marching_tetrahedra(&field, mx, my, mz, origin - Vec3::new(h / 2.0, h / 2.0, h / 2.0), h)
}

/// A sample spacing for a body of this size: about `cells` across its longest side, never finer
/// than `finest` mm.
pub fn spacing_for(size: Vec3, cells: f64, finest: f64) -> f64 {
    let longest = size.x.max(size.y).max(size.z);
    (longest / cells.max(1.0)).max(finest)
}
