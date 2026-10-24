//! The infill a slicer would lay down where a plane cuts a printed part.
//!
//! The printed-part stress solve homogenises the lattice into one material, which is what makes
//! settings comparable, but it means the infill has no geometry to look at. This draws it: the
//! pattern at its pitch, the wall loops around the outside, and gaps wherever the part has a hole.
//! It is a drawing of the pattern at the right pitch, not a slicer's toolpath: it shows where
//! material is and how much, which is what makes a stress plot on top of it mean something.

use std::f64::consts::PI;

use solidcraft_geom::Vec3;

use crate::print_analysis::{InfillPattern, PrintSettings};

/// One drawn line of the infill where the cut plane meets it (mm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Strand {
    pub from: Vec3,
    pub to: Vec3,
    /// Part of the shell rather than the lattice.
    pub wall: bool,
}

/// One family of parallel walls: the angle they run at in the section and their spacing as a
/// multiple of the base pitch.
fn families_of(pattern: InfillPattern) -> Vec<(f64, f64)> {
    match pattern {
        InfillPattern::Grid | InfillPattern::Rectilinear => vec![(0.0, 1.0), (90.0, 1.0)],
        InfillPattern::Lines => vec![(45.0, 1.0)],
        InfillPattern::CrossHatch => vec![(45.0, 1.0), (135.0, 1.0)],
        InfillPattern::Triangles | InfillPattern::TriHexagon | InfillPattern::Honeycomb => vec![(0.0, 1.0), (60.0, 1.0), (120.0, 1.0)],
        InfillPattern::Cubic => vec![(0.0, 1.0), (90.0, 1.0), (45.0, 1.4)],
        InfillPattern::Lightning => vec![(35.0, 3.0)],
        InfillPattern::Concentric | InfillPattern::Gyroid => Vec::new(),
    }
}

fn set(v: &mut Vec3, axis: usize, value: f64) {
    match axis {
        0 => v.x = value,
        1 => v.y = value,
        _ => v.z = value,
    }
}

fn get(v: Vec3, axis: usize) -> f64 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// The strands where the plane normal to `axis` (0 = x, 1 = y, 2 = z) at `station` cuts the part
/// bounded by `min..max` (mm), with `inside` saying which points are within the part. At most
/// `max_strands` come back (the walls first, then the lattice).
pub fn infill_on_plane(
    settings: &PrintSettings,
    min: Vec3,
    max: Vec3,
    axis: usize,
    station: f64,
    inside: &dyn Fn(Vec3) -> bool,
    max_strands: usize,
) -> Vec<Strand> {
    let mut strands: Vec<Strand> = Vec::new();
    let density = settings.infill_density.clamp(0.0, 1.0);
    let line_width = settings.line_width.max(0.01);
    let axis = axis.min(2);
    let a_axis = if axis == 0 { 1 } else { 0 };
    let b_axis = if axis == 2 { 1 } else { 2 };
    let (lo_a, hi_a) = (get(min, a_axis), get(max, a_axis));
    let (lo_b, hi_b) = (get(min, b_axis), get(max, b_axis));
    let (span_a, span_b) = (hi_a - lo_a, hi_b - lo_b);
    if !(span_a > 0.0 && span_b > 0.0) || !span_a.is_finite() || !span_b.is_finite() {
        return strands;
    }
    let at = |a: f64, b: f64| -> Vec3 {
        let mut p = Vec3::ZERO;
        set(&mut p, axis, station);
        set(&mut p, a_axis, a);
        set(&mut p, b_axis, b);
        p
    };
    let step = (span_a.min(span_b) / 240.0).max(line_width / 4.0);
    let (centre_a, centre_b) = ((lo_a + hi_a) / 2.0, (lo_b + hi_b) / 2.0);
    let reach = (span_a * span_a + span_b * span_b).sqrt();

    // Runs of inside samples along a path become strands; a bore or a rib shows as a gap. A
    // straight path merges its run into one strand; a curved one keeps every sample pair.
    let emit_path = |points: &mut dyn Iterator<Item = Vec3>, wall: bool, straight: bool, strands: &mut Vec<Strand>| {
        let mut running: Option<(Vec3, Vec3)> = None;
        let push = |from: Vec3, to: Vec3, strands: &mut Vec<Strand>| {
            if (to - from).len() > 1e-9 {
                strands.push(Strand { from, to, wall });
            }
        };
        for q in points {
            let a = get(q, a_axis);
            let b = get(q, b_axis);
            let in_box = a >= lo_a && a <= hi_a && b >= lo_b && b <= hi_b;
            if in_box && inside(q) {
                match &mut running {
                    Some((from, last)) if straight => {
                        let _ = from;
                        *last = q;
                    }
                    Some((_, last)) => {
                        push(*last, q, strands);
                        *last = q;
                    }
                    None => running = Some((q, q)),
                }
            } else if let Some((from, to)) = running.take()
                && straight
            {
                push(from, to, strands);
            }
        }
        if let Some((from, to)) = running
            && straight
        {
            push(from, to, strands);
        }
    };

    // The shell: wall loops as inset outlines of the section's box, sampled where the part is.
    let loops = settings.wall_loops as usize;
    for loop_index in 0..loops {
        let inset = (loop_index as f64 + 0.5) * line_width;
        if 2.0 * inset >= span_a.min(span_b) {
            break;
        }
        let count_a = ((span_a / step) as usize).max(24);
        let count_b = ((span_b / step) as usize).max(24);
        let sides: [Vec<Vec3>; 4] = [
            (0..=count_a).map(|s| at(lo_a + inset + (span_a - 2.0 * inset) * s as f64 / count_a as f64, lo_b + inset)).collect(),
            (0..=count_b).map(|s| at(hi_a - inset, lo_b + inset + (span_b - 2.0 * inset) * s as f64 / count_b as f64)).collect(),
            (0..=count_a).map(|s| at(lo_a + inset + (span_a - 2.0 * inset) * s as f64 / count_a as f64, hi_b - inset)).collect(),
            (0..=count_b).map(|s| at(lo_a + inset, lo_b + inset + (span_b - 2.0 * inset) * s as f64 / count_b as f64)).collect(),
        ];
        for side in sides {
            emit_path(&mut side.into_iter(), true, true, &mut strands);
        }
        if strands.len() >= max_strands {
            strands.truncate(max_strands);
            return strands;
        }
    }
    if !(density > 0.0) {
        return strands;
    }

    // The lattice: at density d one line of width w every w/d, shared between the directions.
    let families = families_of(settings.pattern);
    let directions = families.len().max(1) as f64;
    let pitch = line_width / density * directions;
    for (angle, spacing_scale) in &families {
        let spacing = pitch * spacing_scale;
        let (dir_a, dir_b) = angle.to_radians().sin_cos();
        let (dir_a, dir_b) = (dir_b, dir_a); // cos along a, sin along b
        let (normal_a, normal_b) = (-dir_b, dir_a);
        let mut offset = -reach;
        while offset <= reach {
            let mut travel = -reach;
            let line = std::iter::from_fn(|| {
                if travel > reach {
                    return None;
                }
                let q = at(centre_a + normal_a * offset + dir_a * travel, centre_b + normal_b * offset + dir_b * travel);
                travel += step;
                Some(q)
            });
            emit_path(&mut { line }, false, true, &mut strands);
            if strands.len() >= max_strands {
                strands.truncate(max_strands);
                return strands;
            }
            offset += spacing;
        }
    }
    if settings.pattern == InfillPattern::Concentric {
        let largest = reach / 2.0;
        let mut radius = pitch;
        while radius < largest {
            let steps = ((2.0 * PI * radius / step) as usize).max(24);
            let mut s = 0usize;
            let ring = std::iter::from_fn(|| {
                if s > steps {
                    return None;
                }
                let t = 2.0 * PI * s as f64 / steps as f64;
                s += 1;
                Some(at(centre_a + radius * t.cos(), centre_b + radius * t.sin()))
            });
            emit_path(&mut { ring }, false, false, &mut strands);
            if strands.len() >= max_strands {
                strands.truncate(max_strands);
                return strands;
            }
            radius += pitch;
        }
    }
    if settings.pattern == InfillPattern::Gyroid {
        // The gyroid surface where it meets the plane, contoured by marching squares.
        let period = pitch * 2.0;
        let k = 2.0 * PI / period;
        let surface = |p: Vec3| (k * p.x).sin() * (k * p.y).cos() + (k * p.y).sin() * (k * p.z).cos() + (k * p.z).sin() * (k * p.x).cos();
        let count_a = ((span_a / (period / 12.0)) as usize).clamp(8, 600);
        let count_b = ((span_b / (period / 12.0)) as usize).clamp(8, 600);
        for ia in 0..count_a {
            for ib in 0..count_b {
                let a0 = lo_a + span_a * ia as f64 / count_a as f64;
                let a1 = lo_a + span_a * (ia + 1) as f64 / count_a as f64;
                let b0 = lo_b + span_b * ib as f64 / count_b as f64;
                let b1 = lo_b + span_b * (ib + 1) as f64 / count_b as f64;
                let corners = [at(a0, b0), at(a1, b0), at(a1, b1), at(a0, b1)];
                if !corners.iter().any(|c| inside(*c)) {
                    continue;
                }
                let values: Vec<f64> = corners.iter().map(|c| surface(*c)).collect();
                let mut crossings: Vec<Vec3> = Vec::new();
                for n in 0..4 {
                    let m = (n + 1) % 4;
                    if (values[n] > 0.0) == (values[m] > 0.0) {
                        continue;
                    }
                    let t = values[n] / (values[n] - values[m]);
                    crossings.push(corners[n] + (corners[m] - corners[n]) * t);
                }
                for pair in crossings.chunks(2) {
                    if let [from, to] = pair {
                        strands.push(Strand { from: *from, to: *to, wall: false });
                        if strands.len() >= max_strands {
                            return strands;
                        }
                    }
                }
            }
        }
    }
    strands
}
