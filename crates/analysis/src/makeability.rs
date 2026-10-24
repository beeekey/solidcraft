//! Can this shape be made, measured on its voxels.
//!
//! Three questions, will it print without support, will it come out of a two-part mould, can a
//! cutter reach every surface, are the same shape of question: can something get at this surface
//! from outside? All three are answered by walking a voxel grid rather than by trusting a rule of
//! thumb. The scan serves the generative search and a drawn part alike, so there is one
//! implementation of it rather than two that drift.
//!
//! The tool test is a morphological erosion (two linear passes over the grid whatever the part
//! looks like) rather than a walk of the cutter's cross-section per face, whose cost depends on
//! how much air the part has in it.

use serde::{Deserialize, Serialize};
use solidcraft_geom::Vec3;

/// A voxel grid, without the design-search baggage.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VoxelFrame {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    /// Cell size (m).
    pub h: f64,
    /// Corner of cell (0, 0, 0) (m).
    pub origin: Vec3,
}

/// What the shape must satisfy to be makeable at all. In the search these are constraints
/// during the search; pointed at a drawn part they are a report.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MakeabilityRules {
    /// Printed without support: nothing may overhang past this angle from vertical, measured
    /// against the build direction.
    pub printable_without_support: bool,
    pub build_axis: usize,
    pub overhang_degrees: f64,
    /// Mouldable or castable: the part must come out of a two-part mould pulled along this axis,
    /// with this much draft.
    pub mouldable: bool,
    pub pull_axis: usize,
    pub draft_degrees: f64,
    /// Machinable: every surface has to be reachable by a tool of this radius (m) coming straight
    /// in along one of the six axes.
    pub machinable: bool,
    pub tool_radius: f64,
    /// How hard to push. Weight 0 reports the numbers without letting them change the ranking.
    pub weight: f64,
    /// Record where, not only how much: off for the search, on for a report.
    pub locate: bool,
    /// The part's own bounding box is the stock, so a face lying on it was never machined.
    pub stock_is_bounding_box: bool,
    /// Cap on the connected groups reported.
    pub max_regions: usize,
    /// Whether the tool may cut with its side as well as its end: the reachable set is then the
    /// free space opened by the tool (erode, then dilate back), so a pocket wall any shop would
    /// cut in one pass is reachable, and a slot narrower than the tool still opens to nothing.
    pub flank_cutting: bool,
}

impl Default for MakeabilityRules {
    fn default() -> Self {
        MakeabilityRules {
            printable_without_support: false,
            build_axis: 2,
            overhang_degrees: 45.0,
            mouldable: false,
            pull_axis: 2,
            draft_degrees: 2.0,
            machinable: false,
            tool_radius: 0.003,
            weight: 1.0,
            locate: false,
            stock_is_bounding_box: false,
            max_regions: 32,
            flank_cutting: false,
        }
    }
}

/// A connected group of surface voxels that share a problem.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ProblemRegion {
    /// m, centroid of the group
    pub centre: Vec3,
    /// m, one voxel centre of it (a marker seat)
    pub sample: Vec3,
    pub voxels: usize,
    /// m², voxels × h²
    pub area: f64,
}

/// What is wrong with making a shape, measured on its voxels.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct MakeabilityScan {
    pub surface_voxels: f64,
    /// Down-facing surface steeper than the overhang limit, which a printer would have to support.
    pub overhang_fraction: f64,
    /// Surface trapped between two mould halves: material above it and below it along the pull.
    pub undercut_fraction: f64,
    /// Surface too square to the pull to release without dragging.
    pub draft_fraction: f64,
    /// Surface no straight tool of the given radius can reach from any of the six directions.
    pub unreachable_fraction: f64,
    /// The worst of whichever rules were asked for.
    pub worst_fraction: f64,
    pub unreachable_regions: Vec<ProblemRegion>,
    pub undercut_regions: Vec<ProblemRegion>,
    pub overhang_regions: Vec<ProblemRegion>,
    /// Machined surface faces reached by each approach, in the order +X −X +Y −Y +Z −Z.
    pub reached_by_axis: [usize; 6],
    /// Faces that are not on the stock boundary: the surface a cutter has to make.
    pub machined_faces: usize,
    /// The smallest set of the six approaches that together reach every machined face.
    pub setup_count: usize,
    pub setup_axes: [bool; 6],
    /// Void inside the stock box that no straight approach can clear: an enclosed cavity (m³).
    pub trapped_volume: f64,
    pub trapped_sample: Option<Vec3>,
}

/// Walks the grid and answers whichever of the three questions were asked.
pub fn scan_makeability(grid: &VoxelFrame, solid: &[bool], rules: &MakeabilityRules) -> MakeabilityScan {
    let mut scan = MakeabilityScan::default();
    let (nx, ny, nz) = (grid.nx as i64, grid.ny as i64, grid.nz as i64);
    if nx <= 0 || ny <= 0 || nz <= 0 || solid.len() != (nx * ny * nz) as usize {
        return scan;
    }
    let material = |i: i64, j: i64, k: i64| -> bool {
        if i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz {
            return false;
        }
        solid[((k * ny + j) * nx + i) as usize]
    };
    let build = rules.build_axis.min(2);
    let pull = rules.pull_axis.min(2);
    let overhang_cos = rules.overhang_degrees.to_radians().cos();
    let overhang_tan = rules.overhang_degrees.clamp(0.0, 89.0).to_radians().tan();
    let draft_sin = rules.draft_degrees.to_radians().sin();
    let tool_cells = ((rules.tool_radius / grid.h.max(1e-9)).ceil() as i64).clamp(1, 64);

    // Every question is asked of a cell one step outside the material, and a cutter's
    // cross-section reaches `tool_cells` further still: pad the index space by that much.
    let pad = if rules.machinable { tool_cells } else { 0 } + 1;
    let (px, py, pz) = (nx + 2 * pad, ny + 2 * pad, nz + 2 * pad);
    let pcount = (px * py * pz) as usize;
    let pstride = [1usize, px as usize, (px * py) as usize];
    let pdim = [px as usize, py as usize, pz as usize];
    let pat = |i: i64, j: i64, k: i64| -> usize { (((k + pad) * py + (j + pad)) * px + (i + pad)) as usize };
    let lo = [-pad, -pad, -pad];
    let hi = [nx + pad, ny + pad, nz + pad];

    // out[c] = there is no material from c inclusive to the far side along (axis, direction).
    let clear_along = |axis: usize, dir: i64| -> Vec<bool> {
        let mut out = vec![false; pcount];
        let (a1, a2) = ((axis + 1) % 3, (axis + 2) % 3);
        let mut p = [0i64; 3];
        p[a1] = lo[a1];
        while p[a1] < hi[a1] {
            p[a2] = lo[a2];
            while p[a2] < hi[a2] {
                let mut clear = true;
                if dir > 0 {
                    p[axis] = hi[axis] - 1;
                    while p[axis] >= lo[axis] {
                        clear = clear && !material(p[0], p[1], p[2]);
                        out[pat(p[0], p[1], p[2])] = clear;
                        p[axis] -= 1;
                    }
                } else {
                    p[axis] = lo[axis];
                    while p[axis] < hi[axis] {
                        clear = clear && !material(p[0], p[1], p[2]);
                        out[pat(p[0], p[1], p[2])] = clear;
                        p[axis] += 1;
                    }
                }
                p[a2] += 1;
            }
            p[a1] += 1;
        }
        out
    };
    // A minimum (erode) or maximum (dilate) over a window, one axis at a time, by prefix counts.
    let window = |input: &[bool], axis: usize, t: usize, erode: bool| -> Vec<bool> {
        let mut out = vec![false; pcount];
        let n = pdim[axis];
        let s = pstride[axis];
        let (a1, a2) = ((axis + 1) % 3, (axis + 2) % 3);
        let mut counts = vec![0usize; n + 1];
        for u in 0..pdim[a1] {
            for v in 0..pdim[a2] {
                let base = u * pstride[a1] + v * pstride[a2];
                for m in 0..n {
                    let value = input[base + m * s];
                    counts[m + 1] = counts[m] + usize::from(if erode { !value } else { value });
                }
                for m in 0..n {
                    let a = m.saturating_sub(t);
                    let b = (m + t).min(n - 1);
                    let inside = counts[b + 1] - counts[a];
                    out[base + m * s] = if erode { inside == 0 } else { inside > 0 };
                }
            }
        }
        out
    };

    // Which of the six approaches a tool of this radius can reach each cell from, one bit per
    // direction, in the order +X −X +Y −Y +Z −Z.
    let mut reach_mask: Vec<u8> = Vec::new();
    if rules.machinable {
        reach_mask = vec![0; pcount];
        let t = tool_cells as usize;
        for face in 0..6 {
            let axis = face / 2;
            let dir = if face % 2 == 0 { 1 } else { -1 };
            let clear = clear_along(axis, dir);
            let tmp = window(&clear, (axis + 1) % 3, t, true);
            let mut eroded = window(&tmp, (axis + 2) % 3, t, true);
            if rules.flank_cutting {
                let tmp = window(&eroded, (axis + 1) % 3, t, false);
                eroded = window(&tmp, (axis + 2) % 3, t, false);
                for (c, e) in eroded.iter_mut().enumerate() {
                    if *e && !clear[c] {
                        *e = false;
                    }
                }
            }
            let bit = 1u8 << face;
            for (c, e) in eroded.iter().enumerate() {
                if *e {
                    reach_mask[c] |= bit;
                }
            }
        }
    }
    let (clear_pull_up, clear_pull_down) = if rules.mouldable { (clear_along(pull, 1), clear_along(pull, -1)) } else { (Vec::new(), Vec::new()) };

    // The part's own bounding box is the stock it was cut from.
    let mut bb_min = [nx, ny, nz];
    let mut bb_max = [-1i64, -1, -1];
    if rules.stock_is_bounding_box {
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    if !material(i, j, k) {
                        continue;
                    }
                    for (a, p) in [i, j, k].iter().enumerate() {
                        bb_min[a] = bb_min[a].min(*p);
                        bb_max[a] = bb_max[a].max(*p);
                    }
                }
            }
        }
    }

    let mut mask_counts = [0usize; 64];
    let mut flag_unreachable = if rules.locate { vec![false; pcount] } else { Vec::new() };
    let mut flag_undercut = if rules.locate { vec![false; pcount] } else { Vec::new() };
    let mut flag_overhang = if rules.locate { vec![false; pcount] } else { Vec::new() };
    let (mut surface, mut overhang, mut undercut, mut draft, mut unreachable) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let steps: [[i64; 3]; 6] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                if !material(i, j, k) {
                    continue;
                }
                for (face, step) in steps.iter().enumerate() {
                    let (ni, nj, nk) = (i + step[0], j + step[1], k + step[2]);
                    if material(ni, nj, nk) {
                        continue;
                    }
                    surface += 1.0;
                    let axis = face / 2;
                    let direction: i64 = if face % 2 == 0 { 1 } else { -1 };
                    let void_cell = pat(ni, nj, nk);
                    // Printing: a face pointing down the build direction is an overhang unless it
                    // rests on the plate (the grid's floor) or on a stair of material one layer
                    // down within the reach the overhang angle allows (one cell sideways at 45°).
                    let on_plate = [ni, nj, nk][build] < 0;
                    let supported = || {
                        let reach = overhang_tan.floor().min(8.0) as i64;
                        let (a1, a2) = ((build + 1) % 3, (build + 2) % 3);
                        let mut q = [ni, nj, nk];
                        let base = q;
                        for d1 in -reach..=reach {
                            for d2 in -reach..=reach {
                                q[a1] = base[a1] + d1;
                                q[a2] = base[a2] + d2;
                                if material(q[0], q[1], q[2]) {
                                    return true;
                                }
                            }
                        }
                        false
                    };
                    if rules.printable_without_support && axis == build && direction < 0 && !on_plate && overhang_cos < 1.0 && !supported() {
                        overhang += 1.0;
                        if rules.locate {
                            flag_overhang[void_cell] = true;
                        }
                    }
                    // Moulding: released when the mould material touching it can be withdrawn
                    // along the pull; blocked both ways is an undercut.
                    if rules.mouldable {
                        let up = clear_pull_up[void_cell];
                        let down = clear_pull_down[void_cell];
                        if !up && !down {
                            undercut += 1.0;
                            if rules.locate {
                                flag_undercut[void_cell] = true;
                            }
                        } else if axis != pull && draft_sin > 0.0 {
                            draft += 1.0;
                        }
                    }
                    // Machining: can a straight tool of this radius get to this face?
                    if rules.machinable && reach_mask[void_cell] == 0 {
                        unreachable += 1.0;
                        if rules.locate {
                            flag_unreachable[void_cell] = true;
                        }
                    }
                    let on_stock = rules.stock_is_bounding_box
                        && bb_max[0] >= 0
                        && [ni, nj, nk].iter().enumerate().any(|(a, q)| *q < bb_min[a] || *q > bb_max[a]);
                    if !on_stock {
                        scan.machined_faces += 1;
                        let mask = if rules.machinable { reach_mask[void_cell] } else { 0 };
                        mask_counts[mask as usize] += 1;
                        for d in 0..6 {
                            if mask & (1 << d) != 0 {
                                scan.reached_by_axis[d] += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    // The setup count: the smallest set of approaches that between them reach every surface a
    // cutter has to make, solved over all 64 subsets. Faces no approach reaches are undercuts and
    // are left out of the cover.
    if rules.machinable && scan.machined_faces > 0 {
        let coverable: usize = mask_counts[1..].iter().sum();
        let mut best_bits = 7;
        let mut best = 0usize;
        for subset in 1..64usize {
            let reached: usize = (1..64).filter(|m| m & subset != 0).map(|m| mask_counts[m]).sum();
            if reached != coverable {
                continue;
            }
            let bits = subset.count_ones() as usize;
            let better = bits < best_bits || (bits == best_bits && (subset & (1 << 4)) != 0 && (best & (1 << 4)) == 0);
            if better {
                best_bits = bits;
                best = subset;
            }
        }
        if best != 0 {
            scan.setup_count = best_bits;
            for d in 0..6 {
                scan.setup_axes[d] = best & (1 << d) != 0;
            }
        }
    }
    // An enclosed cavity, as opposed to an undercut that opens through a slot: connectivity to
    // the outside air rather than reach.
    if rules.stock_is_bounding_box && bb_max[0] >= 0 {
        let mut outside = vec![false; pcount];
        let mut queue: Vec<[i64; 3]> = Vec::new();
        let push = |i: i64, j: i64, k: i64, outside: &mut Vec<bool>, queue: &mut Vec<[i64; 3]>| {
            if i < -pad || j < -pad || k < -pad || i >= nx + pad || j >= ny + pad || k >= nz + pad {
                return;
            }
            let c = pat(i, j, k);
            if outside[c] || material(i, j, k) {
                return;
            }
            outside[c] = true;
            queue.push([i, j, k]);
        };
        for j in -pad..ny + pad {
            for i in -pad..nx + pad {
                push(i, j, -pad, &mut outside, &mut queue);
                push(i, j, nz + pad - 1, &mut outside, &mut queue);
            }
        }
        for k in -pad..nz + pad {
            for i in -pad..nx + pad {
                push(i, -pad, k, &mut outside, &mut queue);
                push(i, ny + pad - 1, k, &mut outside, &mut queue);
            }
            for j in -pad..ny + pad {
                push(-pad, j, k, &mut outside, &mut queue);
                push(nx + pad - 1, j, k, &mut outside, &mut queue);
            }
        }
        let mut head = 0;
        while head < queue.len() {
            let [i, j, k] = queue[head];
            head += 1;
            push(i + 1, j, k, &mut outside, &mut queue);
            push(i - 1, j, k, &mut outside, &mut queue);
            push(i, j + 1, k, &mut outside, &mut queue);
            push(i, j - 1, k, &mut outside, &mut queue);
            push(i, j, k + 1, &mut outside, &mut queue);
            push(i, j, k - 1, &mut outside, &mut queue);
        }
        let cell = grid.h * grid.h * grid.h;
        for k in bb_min[2]..=bb_max[2] {
            for j in bb_min[1]..=bb_max[1] {
                for i in bb_min[0]..=bb_max[0] {
                    if material(i, j, k) || outside[pat(i, j, k)] {
                        continue;
                    }
                    scan.trapped_volume += cell;
                    if scan.trapped_sample.is_none() {
                        scan.trapped_sample = Some(grid.origin + Vec3::new(i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5) * grid.h);
                    }
                }
            }
        }
    }
    // Group the flagged faces, so a whole trapped lobe is one finding with one place on it.
    if rules.locate {
        let group = |flag: &mut Vec<bool>| -> Vec<ProblemRegion> {
            let mut out: Vec<ProblemRegion> = Vec::new();
            let mut queue: Vec<[i64; 3]> = Vec::new();
            for k in -pad..nz + pad {
                for j in -pad..ny + pad {
                    for i in -pad..nx + pad {
                        if !flag[pat(i, j, k)] {
                            continue;
                        }
                        let mut region = ProblemRegion::default();
                        let mut sum = Vec3::ZERO;
                        queue.clear();
                        flag[pat(i, j, k)] = false;
                        queue.push([i, j, k]);
                        let mut head = 0;
                        while head < queue.len() {
                            let [x, y, z] = queue[head];
                            head += 1;
                            let centre = grid.origin + Vec3::new(x as f64 + 0.5, y as f64 + 0.5, z as f64 + 0.5) * grid.h;
                            if region.voxels == 0 {
                                region.sample = centre;
                            }
                            sum += centre;
                            region.voxels += 1;
                            for q in [[x + 1, y, z], [x - 1, y, z], [x, y + 1, z], [x, y - 1, z], [x, y, z + 1], [x, y, z - 1]] {
                                if q[0] < -pad || q[1] < -pad || q[2] < -pad || q[0] >= nx + pad || q[1] >= ny + pad || q[2] >= nz + pad {
                                    continue;
                                }
                                let c = pat(q[0], q[1], q[2]);
                                if flag[c] {
                                    flag[c] = false;
                                    queue.push(q);
                                }
                            }
                        }
                        region.centre = sum / region.voxels.max(1) as f64;
                        region.area = region.voxels as f64 * grid.h * grid.h;
                        out.push(region);
                    }
                }
            }
            out.sort_by_key(|r| std::cmp::Reverse(r.voxels));
            if rules.max_regions > 0 && out.len() > rules.max_regions {
                out.truncate(rules.max_regions);
            }
            out
        };
        if rules.machinable {
            scan.unreachable_regions = group(&mut flag_unreachable);
        }
        if rules.mouldable {
            scan.undercut_regions = group(&mut flag_undercut);
        }
        if rules.printable_without_support {
            scan.overhang_regions = group(&mut flag_overhang);
        }
    }
    if surface <= 0.0 {
        return scan;
    }
    scan.surface_voxels = surface;
    scan.overhang_fraction = overhang / surface;
    scan.undercut_fraction = undercut / surface;
    scan.draft_fraction = draft / surface;
    scan.unreachable_fraction = unreachable / surface;
    scan.worst_fraction = [
        if rules.printable_without_support { scan.overhang_fraction } else { 0.0 },
        if rules.mouldable { scan.undercut_fraction } else { 0.0 },
        if rules.machinable { scan.unreachable_fraction } else { 0.0 },
    ]
    .into_iter()
    .fold(0.0, f64::max);
    scan
}

#[cfg(test)]
#[path = "makeability_tests.rs"]
mod tests;
