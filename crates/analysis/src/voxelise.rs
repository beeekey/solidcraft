//! Which cells of a regular grid lie inside a closed mesh.
//!
//! Every analysis starts here. The mesh is in millimetres, as SolidCraft's meshes are; the grid
//! it produces is in metres, as the solver wants.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Mesh, Vec3};

/// Millimetres per metre.
pub const MM: f64 = 1000.0;

/// Where a body is held or loaded.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Region {
    /// The material cells on the side of the body's box nearest this point (mm).
    NearestWall(Vec3),
    /// The material cells touching these triangles of the mesh (indices into `Mesh::triangles`):
    /// a picked face.
    Face(Vec<usize>),
}

/// A regular grid of cells, with which of them hold material.
#[derive(Debug, Clone, PartialEq)]
pub struct VoxelGrid {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    /// Cell size (m).
    pub h: f64,
    /// Corner of cell (0, 0, 0) (m).
    pub origin: Vec3,
    /// `nx*ny*nz`, x fastest.
    pub solid: Vec<bool>,
    /// Size of the mesh's bounding box (m).
    pub size: Vec3,
}

impl VoxelGrid {
    pub fn index(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.ny + j) * self.nx + i
    }

    pub fn ijk(&self, voxel: usize) -> (usize, usize, usize) {
        (voxel % self.nx, (voxel / self.nx) % self.ny, voxel / (self.nx * self.ny))
    }

    /// Centre of a cell (m).
    pub fn centre(&self, voxel: usize) -> Vec3 {
        let (i, j, k) = self.ijk(voxel);
        self.origin + Vec3::new(i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5) * self.h
    }

    /// Cells holding material.
    pub fn filled(&self) -> usize {
        self.solid.iter().filter(|s| **s).count()
    }

    fn solid_at(&self, i: isize, j: isize, k: isize) -> bool {
        if i < 0 || j < 0 || k < 0 {
            return false;
        }
        let (i, j, k) = (i as usize, j as usize, k as usize);
        i < self.nx && j < self.ny && k < self.nz && self.solid[self.index(i, j, k)]
    }

    /// Material cells with an empty neighbour: the body's skin.
    fn surface_cells(&self) -> Vec<usize> {
        let mut out = Vec::new();
        for voxel in 0..self.solid.len() {
            if !self.solid[voxel] {
                continue;
            }
            let (i, j, k) = self.ijk(voxel);
            let (i, j, k) = (i as isize, j as isize, k as isize);
            let exposed = !(self.solid_at(i - 1, j, k)
                && self.solid_at(i + 1, j, k)
                && self.solid_at(i, j - 1, k)
                && self.solid_at(i, j + 1, k)
                && self.solid_at(i, j, k - 1)
                && self.solid_at(i, j, k + 1));
            if exposed {
                out.push(voxel);
            }
        }
        out
    }

    /// The cells a region names.
    pub fn region(&self, mesh: &Mesh, region: &Region) -> Vec<usize> {
        match region {
            Region::NearestWall(point) => self.wall_slab(*point / MM),
            Region::Face(triangles) => self.face_slab(mesh, triangles),
        }
    }

    /// The solid cells on the bounding-box wall nearest `point` (m): the slab a fixture or a load
    /// is spread over.
    pub fn wall_slab(&self, point: Vec3) -> Vec<usize> {
        let walls = [
            (self.origin.x, self.origin.x + self.size.x),
            (self.origin.y, self.origin.y + self.size.y),
            (self.origin.z, self.origin.z + self.size.z),
        ];
        let coords = [point.x, point.y, point.z];
        let mut best = f64::INFINITY;
        let mut axis = 0;
        let mut min_side = true;
        for a in 0..3 {
            let to_min = (coords[a] - walls[a].0).abs();
            if to_min < best {
                best = to_min;
                axis = a;
                min_side = true;
            }
            let to_max = (coords[a] - walls[a].1).abs();
            if to_max < best {
                best = to_max;
                axis = a;
                min_side = false;
            }
        }
        let counts = [self.nx, self.ny, self.nz];
        let mut out = Vec::new();
        for voxel in 0..self.solid.len() {
            if !self.solid[voxel] {
                continue;
            }
            let (i, j, k) = self.ijk(voxel);
            let idx = [i, j, k];
            if (min_side && idx[axis] == 0) || (!min_side && idx[axis] + 1 == counts[axis]) {
                out.push(voxel);
            }
        }
        out
    }

    /// The skin cells whose centre lies within a cell of the given triangles (mesh in mm): the
    /// cells a picked face touches.
    pub fn face_slab(&self, mesh: &Mesh, triangles: &[usize]) -> Vec<usize> {
        let tris: Vec<[Vec3; 3]> = triangles.iter().filter_map(|t| mesh.triangles.get(*t)).filter_map(|t| mesh.tri(t)).collect();
        if tris.is_empty() {
            return Vec::new();
        }
        let reach = 0.87 * self.h; // half the cell's diagonal: a cell cut by the face counts
        let mut out = Vec::new();
        for voxel in self.surface_cells() {
            let centre = self.centre(voxel) * MM;
            let near = tris.iter().any(|[a, b, c]| point_triangle_distance(centre, *a, *b, *c) <= reach * MM);
            if near {
                out.push(voxel);
            }
        }
        out
    }
}

/// Distance from `p` to the triangle `abc` (Ericson, Real-Time Collision Detection, 5.1.5).
pub fn point_triangle_distance(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.len();
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return bp.len();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (p - (a + ab * v)).len();
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return cp.len();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (p - (a + ac * w)).len();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + (c - b) * w)).len();
    }
    let denom = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    (p - (a + ab * v + ac * w)).len()
}

/// Voxelises a closed mesh (mm) so that its longest side has `resolution` cells. `max_cells` caps
/// the grid; `None` when the mesh is empty or the grid would exceed the cap.
pub fn voxelise(mesh: &Mesh, resolution: usize, max_cells: usize) -> Option<VoxelGrid> {
    voxelise_many(&[mesh], resolution, max_cells).map(|(grid, _)| grid)
}

/// Voxelises several closed meshes (mm) on one grid over their common box, the longest side in
/// `resolution` cells: the grid with every cell inside any of them, and which mesh each cell
/// belongs to (the first that holds it, where two overlap). `None` when there is nothing to
/// voxelise or the grid would exceed `max_cells`.
pub fn voxelise_many(meshes: &[&Mesh], resolution: usize, max_cells: usize) -> Option<(VoxelGrid, Vec<usize>)> {
    if meshes.is_empty() || meshes.iter().any(|m| m.triangles.is_empty()) || resolution == 0 {
        return None;
    }
    let mut bounds = meshes.first()?.bounds();
    for m in meshes.iter().skip(1) {
        let b = m.bounds();
        bounds.min = bounds.min.min(b.min);
        bounds.max = bounds.max.max(b.max);
    }
    if bounds.is_empty() {
        return None;
    }
    let size_mm = bounds.size();
    let longest = size_mm.x.max(size_mm.y).max(size_mm.z);
    if !(longest > 0.0) || !longest.is_finite() {
        return None;
    }
    let h_mm = longest / resolution as f64;
    let cells = |extent: f64| ((extent / h_mm).round() as usize).max(1);
    let (nx, ny, nz) = (cells(size_mm.x), cells(size_mm.y), cells(size_mm.z));
    let count = nx.checked_mul(ny)?.checked_mul(nz)?;
    if count > max_cells {
        return None;
    }
    // The cells cover the box with the slack shared on both sides, so a cell centre never lands
    // on a face of the body (a 10 mm wall in 6.7 mm cells would otherwise have one centre on its
    // surface and classify as a single row).
    let slack = Vec3::new(nx as f64 * h_mm - size_mm.x, ny as f64 * h_mm - size_mm.y, nz as f64 * h_mm - size_mm.z) * 0.5;
    let origin_mm = bounds.min - slack;
    let indices: Vec<_> = meshes.iter().map(|m| m.inside_index()).collect();
    let mut solid = vec![false; count];
    let mut body_of = vec![0usize; count];
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let centre = origin_mm + Vec3::new((i as f64 + 0.5) * h_mm, (j as f64 + 0.5) * h_mm, (k as f64 + 0.5) * h_mm);
                if let Some(body) = indices.iter().position(|index| index.contains(centre)) {
                    let cell = (k * ny + j) * nx + i;
                    solid[cell] = true;
                    body_of[cell] = body;
                }
            }
        }
    }
    Some((
        VoxelGrid { nx, ny, nz, h: h_mm / MM, origin: origin_mm / MM, solid, size: Vec3::new(nx as f64, ny as f64, nz as f64) * (h_mm / MM) },
        body_of,
    ))
}
