//! Tracing a raster: the boundaries of its dark (or light) pixels as closed loops in pixel
//! coordinates, simplified and sorted by nesting, so a picture of a part becomes sketch profiles.
//!
//! The loops are the exact pixel-edge staircase, chained with the traced region on a consistent
//! side and the left turn taken at every corner where two pixels touch diagonally, so loops stay
//! separate. Douglas–Peucker then drops the staircase to a tolerance. A loop inside an odd
//! number of others is a hole.

use std::collections::BTreeMap;

/// A grayscale raster, 0 black to 255 white, row 0 at the top.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bitmap {
    pub width: usize,
    pub height: usize,
    pub gray: Vec<u8>,
}

/// The most pixels a bitmap may hold.
pub const MAX_PIXELS: usize = 16_000_000;

impl Bitmap {
    /// `None` for an empty raster, one over [`MAX_PIXELS`], or one whose data does not match.
    pub fn new(width: usize, height: usize, gray: Vec<u8>) -> Option<Bitmap> {
        let count = width.checked_mul(height)?;
        if width == 0 || height == 0 || count > MAX_PIXELS || gray.len() != count {
            return None;
        }
        Some(Bitmap { width, height, gray })
    }

    fn at(&self, x: usize, y: usize) -> u8 {
        self.gray.get(y * self.width + x).copied().unwrap_or(255)
    }
}

/// A closed loop in pixel coordinates (y down), the last point joined to the first.
pub type Contour = Vec<[f64; 2]>;

/// What a trace found: loops with their nesting depth (0 outer, 1 a hole, 2 an island in it).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Traced {
    pub contours: Vec<Contour>,
    pub depths: Vec<usize>,
    /// Loops dropped as specks (area under the minimum) or as degenerate.
    pub dropped: usize,
}

/// The boundaries of every pixel darker than `threshold` (lighter or equal, with `light`).
pub fn trace_contours(bitmap: &Bitmap, threshold: u8, light: bool) -> Vec<Contour> {
    trace_region(bitmap.width, bitmap.height, |x, y| {
        let g = bitmap.at(x, y);
        if light { g >= threshold } else { g < threshold }
    })
}

/// The boundaries of whatever `inside` says is inside, as directed loops with the region on the
/// left in image coordinates.
pub fn trace_region(width: usize, height: usize, inside: impl Fn(usize, usize) -> bool) -> Vec<Contour> {
    if width == 0
        || height == 0
        || width.checked_mul(height).is_none_or(|n| n > MAX_PIXELS)
        || width > i32::MAX as usize / 2
        || height > i32::MAX as usize / 2
    {
        return Vec::new();
    }
    let dark = |x: i64, y: i64| x >= 0 && y >= 0 && (x as usize) < width && (y as usize) < height && inside(x as usize, y as usize);
    // Directed boundary edges around dark pixel squares, keyed by (y, x) of their start.
    let mut edges: BTreeMap<(i32, i32), Vec<(i32, i32)>> = BTreeMap::new();
    let mut push = |from: (i32, i32), to: (i32, i32)| edges.entry((from.1, from.0)).or_default().push(to);
    for y in 0..height {
        for x in 0..width {
            if !inside(x, y) {
                continue;
            }
            let (xi, yi) = (x as i64, y as i64);
            let (x0, y0) = (x as i32, y as i32);
            if !dark(xi, yi - 1) {
                push((x0 + 1, y0), (x0, y0));
            }
            if !dark(xi, yi + 1) {
                push((x0, y0 + 1), (x0 + 1, y0 + 1));
            }
            if !dark(xi - 1, yi) {
                push((x0, y0), (x0, y0 + 1));
            }
            if !dark(xi + 1, yi) {
                push((x0 + 1, y0 + 1), (x0 + 1, y0));
            }
        }
    }
    let mut contours = Vec::new();
    while let Some((&key, _)) = edges.iter().next() {
        let start = (key.1, key.0);
        let mut at = start;
        let mut incoming: Option<(i32, i32)> = None;
        let mut contour: Contour = Vec::new();
        while let Some(outgoing) = edges.get_mut(&(at.1, at.0)) {
            if outgoing.is_empty() {
                edges.remove(&(at.1, at.0));
                break;
            }
            // At a corner where two dark pixels touch diagonally, turn left relative to the
            // incoming direction so the loops stay separate.
            let mut pick = 0;
            if let Some(inc) = incoming
                && outgoing.len() > 1
            {
                let cross = |to: (i32, i32)| inc.0 * (to.1 - at.1) - inc.1 * (to.0 - at.0);
                for (i, to) in outgoing.iter().enumerate() {
                    if cross(*to) < cross(outgoing[pick]) {
                        pick = i;
                    }
                }
            }
            let next = outgoing.swap_remove(pick);
            if outgoing.is_empty() {
                edges.remove(&(at.1, at.0));
            }
            incoming = Some((next.0 - at.0, next.1 - at.1));
            contour.push([f64::from(at.0), f64::from(at.1)]);
            at = next;
            if at == start {
                break;
            }
        }
        if contour.len() >= 4 {
            contours.push(contour);
        }
    }
    contours
}

fn douglas_peucker(points: &[[f64; 2]], first: usize, last: usize, tolerance: f64, keep: &mut [bool]) {
    // Iterative, so a staircase of a million points cannot exhaust the stack.
    let mut stack = vec![(first, last)];
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (Some(pa), Some(pb)) = (points.get(a), points.get(b)) else { continue };
        let (dx, dy) = (pb[0] - pa[0], pb[1] - pa[1]);
        let norm = dx.hypot(dy);
        let mut worst = -1.0;
        let mut index = a;
        for (i, p) in points.iter().enumerate().take(b).skip(a + 1) {
            let (px, py) = (p[0] - pa[0], p[1] - pa[1]);
            let distance = if norm > 1e-12 { (dx * py - dy * px).abs() / norm } else { px.hypot(py) };
            if distance > worst {
                worst = distance;
                index = i;
            }
        }
        if worst > tolerance {
            if let Some(k) = keep.get_mut(index) {
                *k = true;
            }
            stack.push((a, index));
            stack.push((index, b));
        }
    }
}

/// Douglas–Peucker on a closed loop: anchored at its first point and the point farthest from
/// it, each half simplified to `tolerance` (pixels), the second half wrapping round the closure.
pub fn simplify(contour: &[[f64; 2]], tolerance: f64) -> Contour {
    if contour.len() <= 4 || !(tolerance > 0.0) {
        return contour.to_vec();
    }
    let Some(first) = contour.first() else { return Vec::new() };
    let mut far = 0;
    let mut best = -1.0;
    for (i, p) in contour.iter().enumerate().skip(1) {
        let d = (p[0] - first[0]).hypot(p[1] - first[1]);
        if d > best {
            best = d;
            far = i;
        }
    }
    let mut keep = vec![false; contour.len()];
    keep[0] = true;
    keep[far] = true;
    douglas_peucker(contour, 0, far, tolerance, &mut keep);
    let mut wrapped: Vec<[f64; 2]> = contour[far..].to_vec();
    wrapped.push(*first);
    let mut keep_wrapped = vec![false; wrapped.len()];
    if let Some(first_kept) = keep_wrapped.first_mut() {
        *first_kept = true;
    }
    if let Some(last) = keep_wrapped.last_mut() {
        *last = true;
    }
    douglas_peucker(&wrapped, 0, wrapped.len() - 1, tolerance, &mut keep_wrapped);
    for (i, k) in keep_wrapped.iter().enumerate().skip(1) {
        if *k
            && i + 1 < wrapped.len()
            && let Some(slot) = keep.get_mut(far + i)
        {
            *slot = true;
        }
    }
    contour.iter().zip(&keep).filter(|(_, k)| **k).map(|(p, _)| *p).collect()
}

/// Even–odd point-in-polygon test.
pub fn contains_point(contour: &[[f64; 2]], x: f64, y: f64) -> bool {
    let n = contour.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (contour[i][0], contour[i][1]);
        let (xj, yj) = (contour[j][0], contour[j][1]);
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// The loop's area (pixels²), by the shoelace formula.
pub fn area(contour: &[[f64; 2]]) -> f64 {
    let n = contour.len();
    if n < 3 {
        return 0.0;
    }
    (0..n)
        .map(|i| {
            let (a, b) = (contour[i], contour[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        .abs()
        / 2.0
}

/// Traces, simplifies to `tolerance_px`, drops loops under `min_area_px`, and sorts the loops
/// outer first with their nesting depth.
pub fn trace(bitmap: &Bitmap, threshold: u8, light: bool, tolerance_px: f64, min_area_px: f64) -> Traced {
    let raw = trace_contours(bitmap, threshold, light);
    let mut traced = Traced::default();
    let mut kept: Vec<Contour> = Vec::new();
    for contour in &raw {
        if area(contour) < min_area_px.max(0.0) {
            traced.dropped += 1;
            continue;
        }
        let simple = simplify(contour, tolerance_px);
        if simple.len() < 3 || area(&simple) < min_area_px.clamp(0.0, 1.0) {
            traced.dropped += 1;
            continue;
        }
        kept.push(simple);
    }
    // Nesting: how many other loops hold this one's first vertex (nudged, so a shared staircase
    // corner cannot count).
    let depths: Vec<usize> = kept
        .iter()
        .enumerate()
        .map(|(c, loop_)| {
            let Some(probe) = loop_.first() else { return 0 };
            kept.iter().enumerate().filter(|(o, other)| *o != c && contains_point(other, probe[0] + 1e-7, probe[1] + 1e-7)).count()
        })
        .collect();
    let mut order: Vec<usize> = (0..kept.len()).collect();
    order.sort_by_key(|&i| depths[i]);
    traced.contours = order.iter().map(|&i| kept[i].clone()).collect();
    traced.depths = order.iter().map(|&i| depths[i]).collect();
    traced
}

#[cfg(test)]
#[path = "trace_tests.rs"]
mod tests;
