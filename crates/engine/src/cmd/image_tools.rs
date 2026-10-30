//! Working a reference picture into something a tracer can use, and one optical check:
//! `canvas.remove_background` (a wand that takes everything of one colour joined to a pixel),
//! `canvas.select_object` (the background judged from the picture's edges; what is left, in one
//! piece with its holes filled, is the object), `canvas.rectify` (a photograph shot at an angle
//! squared up from four corners) and `inspect.ray_trace` (first-surface mirror reflections of a
//! ray off the bodies, drawn as a 3D sketch). The picture tools rewrite the canvas's image, so
//! Trace Canvas then sees a clean silhouette.

use std::collections::VecDeque;

use image::{DynamicImage, ImageFormat, RgbaImage};
use serde_json::{Value, json};
use solidcraft_doc::canvas::{Canvas, base64_encode};
use solidcraft_geom::MeshIndex;

use super::CommandSpec;
use super::design::{number, run};
use crate::params::{bad, bool_, str_, vec3};
use crate::{Result, Session};

/// The most pixels a picture may have for these tools.
const MAX_PIXELS: usize = 24_000_000;

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("canvas.remove_background", "Remove Background", remove_background).at("3DEEE", "TRACE").icon("remove_background").params(
        "canvas: id or name; seed_px?: [x, y] the pixel the background is sampled at (default [2, 2], a corner); tolerance?: 0 to 1 (default 0.15; \
         raise it for a shaded or noisy background); contiguous?: bool (default true: only what is joined to the seed; false: that colour \
         wherever it is); fill?: white (default) | black | transparent: what the background becomes → canvas, background_share, pixels. The wand \
         takes everything of the seed's colour joined to it and paints it out, so Trace Canvas sees the object alone",
    ),
    CommandSpec::new("canvas.select_object", "Select Object", select_object).at("3DEEE", "TRACE").icon("select_object").params(
        "canvas: id or name; tolerance?: 0 to 1 (default 0.15); fill?: white (default) | black | transparent; min_pixels?: specks under this \
         size go (default: a two-thousandth of the picture) → canvas, object_share, size_mm (about), touches_border, pixels. The background is \
         judged from the edges of the picture (whatever most of the border agrees on); what is left, in one piece with its holes filled, is the \
         object, and the background is painted out",
    ),
    CommandSpec::new("canvas.rectify", "Shoot It Square On", rectify).at("3DEEE", "TRACE").icon("rectify").params(
        "canvas: id or name; corners: [[x, y] × 4] the corners of a rectangle in the photograph, in canvas-plane mm (or pixels with pixel: true), in \
         order round the rectangle from one corner; width_mm, height_mm: the rectangle's true size; margin?: the picture kept around it, as a \
         fraction of the rectangle (default 0) → canvas, pixels, width_mm, height_mm. The picture is resampled so the rectangle is square on \
         and to scale, and the canvas takes the rectangle's width",
    ),
    CommandSpec::new("inspect.ray_trace", "Ray Trace", ray_trace).at("3DEEE", "TRACE").icon("ray_trace").params(
        "origin: [x, y, z]; direction: [x, y, z]; bounces?: how many reflections at most (0 to 32, default 4); exit_mm?: the last leg's drawn \
         length (default 200); bodies?: [names] (default: all); name? → sketch, points, hits: [{body, at, normal}], length_mm, escaped. A ray \
         reflected off the bodies as first-surface mirrors, drawn as a 3D sketch polyline",
    ),
];

// --- pixels -----------------------------------------------------------------------------------

struct Picture {
    w: usize,
    h: usize,
    rgba: RgbaImage,
}

impl Picture {
    fn colour_distance(&self, a: usize, b: usize) -> f64 {
        let (pa, pb) = (self.rgba.as_raw(), self.rgba.as_raw());
        let (ia, ib) = (4 * a, 4 * b);
        let d = |c: usize| f64::from(pa[ia + c]) - f64::from(pb[ib + c]);
        (d(0) * d(0) + d(1) * d(1) + d(2) * d(2)).sqrt() / (255.0 * 3f64.sqrt())
    }

    /// Grows regions from `seeds` over pixels within `tolerance` of their neighbour and within
    /// `cap` times the tolerance of the region's own seed, so a gradient cannot eat the picture.
    fn flood(&self, seeds: &[usize], tolerance: f64, cap: f64) -> Vec<bool> {
        let n = self.w * self.h;
        let mut picked = vec![false; n];
        let mut origin = vec![0usize; n];
        let mut queue = VecDeque::new();
        for &seed in seeds {
            if seed < n && !picked[seed] {
                picked[seed] = true;
                origin[seed] = seed;
                queue.push_back(seed);
            }
        }
        while let Some(here) = queue.pop_front() {
            let (x, y) = (here % self.w, here / self.w);
            let steps: [(isize, isize); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
            for (dx, dy) in steps {
                let (nx, ny) = (x as isize + dx, y as isize + dy);
                if nx < 0 || ny < 0 || nx >= self.w as isize || ny >= self.h as isize {
                    continue;
                }
                let next = ny as usize * self.w + nx as usize;
                if picked[next] || self.colour_distance(here, next) > tolerance || self.colour_distance(origin[here], next) > tolerance * cap {
                    continue;
                }
                picked[next] = true;
                origin[next] = origin[here];
                queue.push_back(next);
            }
        }
        picked
    }

    /// 4-connected components of a mask, labelled from 1; the count comes back too.
    fn components(&self, mask: &[bool]) -> (Vec<u32>, usize) {
        let mut label = vec![0u32; mask.len()];
        let mut count = 0usize;
        let mut queue = VecDeque::new();
        for start in 0..mask.len() {
            if !mask[start] || label[start] != 0 {
                continue;
            }
            count += 1;
            let id = count as u32;
            label[start] = id;
            queue.push_back(start);
            while let Some(here) = queue.pop_front() {
                let (x, y) = (here % self.w, here / self.w);
                let steps: [(isize, isize); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
                for (dx, dy) in steps {
                    let (nx, ny) = (x as isize + dx, y as isize + dy);
                    if nx < 0 || ny < 0 || nx >= self.w as isize || ny >= self.h as isize {
                        continue;
                    }
                    let next = ny as usize * self.w + nx as usize;
                    if mask[next] && label[next] == 0 {
                        label[next] = id;
                        queue.push_back(next);
                    }
                }
            }
        }
        (label, count)
    }

    fn border(&self) -> Vec<usize> {
        let mut b = Vec::with_capacity(2 * (self.w + self.h));
        for x in 0..self.w {
            b.push(x);
            b.push((self.h - 1) * self.w + x);
        }
        for y in 0..self.h {
            b.push(y * self.w);
            b.push(y * self.w + self.w - 1);
        }
        b
    }

    /// The largest piece of the mask plus any piece of at least `minimum` pixels, with the gaps
    /// that cannot reach the edge of the picture filled in.
    fn clean(&self, mask: &[bool], minimum: usize) -> Vec<bool> {
        let (label, count) = self.components(mask);
        if count == 0 {
            return mask.to_vec();
        }
        let mut sizes = vec![0usize; count + 1];
        for &l in &label {
            sizes[l as usize] += 1;
        }
        let biggest = (1..=count).max_by_key(|i| sizes[*i]).unwrap_or(1) as u32;
        let mut cleaned: Vec<bool> = label.iter().map(|l| *l != 0 && (*l == biggest || sizes[*l as usize] >= minimum.max(1))).collect();
        let gaps: Vec<bool> = cleaned.iter().map(|b| !b).collect();
        let (gap_label, gap_count) = self.components(&gaps);
        let mut touches = vec![false; gap_count + 1];
        for b in self.border() {
            touches[gap_label[b] as usize] = true;
        }
        for (p, c) in cleaned.iter_mut().enumerate() {
            let g = gap_label[p];
            if !*c && g != 0 && !touches[g as usize] {
                *c = true;
            }
        }
        cleaned
    }

    /// Paints the masked pixels with the fill.
    fn paint(&mut self, mask: &[bool], fill: Fill) {
        let raw = self.rgba.as_mut();
        for (p, on) in mask.iter().enumerate() {
            if !*on {
                continue;
            }
            let px = &mut raw[4 * p..4 * p + 4];
            match fill {
                Fill::White => px.copy_from_slice(&[255, 255, 255, 255]),
                Fill::Black => px.copy_from_slice(&[0, 0, 0, 255]),
                Fill::Transparent => px.copy_from_slice(&[255, 255, 255, 0]),
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Fill {
    White,
    Black,
    Transparent,
}

fn fill_of(p: &Value, cmd: &str) -> Result<Fill> {
    match str_(p, "fill").map(|f| f.trim().to_ascii_lowercase()).as_deref() {
        None | Some("white") => Ok(Fill::White),
        Some("black") => Ok(Fill::Black),
        Some("transparent") | Some("clear") => Ok(Fill::Transparent),
        Some(_) => Err(bad(cmd, "`fill` is white, black or transparent")),
    }
}

fn canvas_index(s: &Session, p: &Value, cmd: &str) -> Result<usize> {
    let key = match p.get("canvas") {
        Some(Value::Number(n)) => n.to_string(),
        Some(Value::String(x)) => x.clone(),
        _ => return Err(bad(cmd, "`canvas` must be an id or a name")),
    };
    s.doc.canvases.iter().position(|c| c.id.to_string() == key || c.name == key).ok_or_else(|| bad(cmd, format!("no canvas `{key}`")))
}

/// Decode a canvas's picture within [`MAX_PIXELS`]: the header's dimensions are checked before
/// any pixel is allocated, and the decoder itself is told the same limits, so a hostile header
/// claiming a huge picture is refused instead of allocated.
pub(super) fn decode_limited(bytes: &[u8], cmd: &str) -> Result<DynamicImage> {
    let mut limits = image::Limits::default();
    limits.max_image_width = Some(MAX_PIXELS as u32);
    limits.max_image_height = Some(MAX_PIXELS as u32);
    limits.max_alloc = Some((MAX_PIXELS as u64).saturating_mul(4));
    let reader = || -> Result<image::ImageReader<std::io::Cursor<&[u8]>>> {
        let mut reader = image::ImageReader::new(std::io::Cursor::new(bytes))
            .with_guessed_format()
            .map_err(|e| bad(cmd, format!("the canvas's image cannot be read: {e}")))?;
        reader.limits(limits.clone());
        Ok(reader)
    };
    let (w, h) = reader()?.into_dimensions().map_err(|e| bad(cmd, format!("the canvas's image cannot be decoded: {e}")))?;
    if (w as usize).checked_mul(h as usize).is_none_or(|n| n > MAX_PIXELS) {
        return Err(bad(cmd, format!("the picture is {w} × {h}: at most {MAX_PIXELS} pixels")));
    }
    reader()?.decode().map_err(|e| bad(cmd, format!("the canvas's image cannot be decoded: {e}")))
}

fn picture_of(canvas: &Canvas, cmd: &str) -> Result<Picture> {
    let bytes = canvas.bytes().ok_or_else(|| bad(cmd, "the canvas holds no image"))?;
    let rgba = decode_limited(&bytes, cmd)?.to_rgba8();
    let (w, h) = (rgba.width() as usize, rgba.height() as usize);
    if w < 2 || h < 2 || w.checked_mul(h).is_none_or(|n| n > MAX_PIXELS) {
        return Err(bad(cmd, format!("the picture must be at least 2 × 2 and at most {MAX_PIXELS} pixels")));
    }
    Ok(Picture { w, h, rgba })
}

/// Writes the picture back into the canvas as a PNG.
fn store(s: &mut Session, i: usize, picture: &Picture, cmd: &str) -> Result<()> {
    let mut bytes: Vec<u8> = Vec::new();
    DynamicImage::ImageRgba8(picture.rgba.clone())
        .write_to(&mut std::io::Cursor::new(&mut bytes), ImageFormat::Png)
        .map_err(|e| bad(cmd, format!("the picture cannot be encoded: {e}")))?;
    if bytes.len() > solidcraft_doc::canvas::MAX_CANVAS_BYTES {
        return Err(bad(cmd, "the picture is too big to keep in the design"));
    }
    let c = s.doc_mut().canvases.get_mut(i).ok_or_else(|| bad(cmd, "canvas"))?;
    c.data = base64_encode(&bytes);
    c.format = "png".into();
    c.pixels = [picture.w as u32, picture.h as u32];
    Ok(())
}

fn tolerance_of(p: &Value, cmd: &str) -> Result<f64> {
    number(cmd, p, "tolerance", Some(0.15), 0.0..=1.0).map(|t| t.unwrap_or(0.15))
}

fn remove_background(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "canvas.remove_background";
    let i = canvas_index(s, p, cmd)?;
    let canvas = s.doc.canvases[i].clone();
    let mut picture = picture_of(&canvas, cmd)?;
    let tolerance = tolerance_of(p, cmd)?;
    let fill = fill_of(p, cmd)?;
    let contiguous = bool_(p, "contiguous").unwrap_or(true);
    let seed = match p.get("seed_px") {
        None | Some(Value::Null) => [2.0_f64.min((picture.w - 1) as f64), 2.0_f64.min((picture.h - 1) as f64)],
        Some(v) => {
            let q: Option<Vec<f64>> = v.as_array().filter(|a| a.len() == 2).map(|a| a.iter().filter_map(Value::as_f64).collect());
            match q {
                Some(q) if q.len() == 2 && q[0] >= 0.0 && q[1] >= 0.0 && q[0] < picture.w as f64 && q[1] < picture.h as f64 => [q[0], q[1]],
                _ => return Err(bad(cmd, format!("`seed_px` is a pixel [x, y] inside the {} × {} picture", picture.w, picture.h))),
            }
        }
    };
    let seed = seed[1] as usize * picture.w + seed[0] as usize;
    let mask: Vec<bool> = if contiguous {
        picture.flood(&[seed], tolerance, 3.0)
    } else {
        (0..picture.w * picture.h).map(|q| picture.colour_distance(seed, q) <= tolerance).collect()
    };
    let share = mask.iter().filter(|b| **b).count() as f64 / mask.len() as f64;
    picture.paint(&mask, fill);
    store(s, i, &picture, cmd)?;
    Ok(
        json!({"canvas": canvas.id, "background_share": share, "pixels": [picture.w, picture.h], "seed_px": [seed % picture.w, seed / picture.w], "tolerance": tolerance}),
    )
}

fn select_object(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "canvas.select_object";
    let i = canvas_index(s, p, cmd)?;
    let canvas = s.doc.canvases[i].clone();
    let mut picture = picture_of(&canvas, cmd)?;
    let tolerance = tolerance_of(p, cmd)?;
    let fill = fill_of(p, cmd)?;
    let total = picture.w * picture.h;
    let minimum = number(cmd, p, "min_pixels", Some((total / 2000).max(4) as f64), 1.0..=1e9)?.unwrap_or((total / 2000).max(4) as f64) as usize;
    // The border votes: the median colour is the background, and only pixels near it seed the
    // flood, so a part running off the edge does not get eaten.
    let border = picture.border();
    let raw = picture.rgba.as_raw();
    let mut channels: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for &b in &border {
        for c in 0..3 {
            channels[c].push(f64::from(raw[4 * b + c]) / 255.0);
        }
    }
    let mut dominant = [0.0; 3];
    for c in 0..3 {
        channels[c].sort_by(f64::total_cmp);
        dominant[c] = channels[c].get(channels[c].len() / 2).copied().unwrap_or(0.0);
    }
    let window = tolerance.max(0.08);
    let seeds: Vec<usize> = border
        .iter()
        .copied()
        .filter(|b| {
            let d: f64 = (0..3).map(|c| (f64::from(raw[4 * b + c]) / 255.0 - dominant[c]).powi(2)).sum::<f64>().sqrt() / 3f64.sqrt();
            d <= window
        })
        .collect();
    let used = seeds.len() as f64 / border.len().max(1) as f64;
    let seeds = if seeds.len() < border.len() / 4 { border.clone() } else { seeds };
    let background = picture.flood(&seeds, tolerance, 3.0);
    let object: Vec<bool> = background.iter().map(|b| !b).collect();
    let object = picture.clean(&object, minimum);
    let count = object.iter().filter(|b| **b).count();
    if count == 0 {
        return Err(bad(cmd, "nothing was left over: at this tolerance the whole picture counts as background; try a smaller one"));
    }
    let paint: Vec<bool> = object.iter().map(|b| !b).collect();
    picture.paint(&paint, fill);
    store(s, i, &picture, cmd)?;
    let share = count as f64 / total as f64;
    Ok(json!({
        "canvas": canvas.id,
        "object_share": share,
        "size_mm": canvas.width * share.sqrt(),
        "touches_border": used < 0.95,
        "border_used": used,
        "pixels": [picture.w, picture.h],
        "tolerance": tolerance,
    }))
}

/// The homography from the rectified picture (pixels) back to the source picture (pixels), as
/// the eight unknowns of the 3 × 3 matrix with its last entry 1.
fn homography(from: [[f64; 2]; 4], to: [[f64; 2]; 4], cmd: &str) -> Result<[f64; 9]> {
    let mut a = [[0.0_f64; 9]; 8];
    for i in 0..4 {
        let (x, y) = (to[i][0], to[i][1]);
        let (u, v) = (from[i][0], from[i][1]);
        a[2 * i] = [x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y, u];
        a[2 * i + 1] = [0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y, v];
    }
    for column in 0..8 {
        let best = (column..8).max_by(|r, q| a[*r][column].abs().total_cmp(&a[*q][column].abs())).unwrap_or(column);
        if a[best][column].abs() < 1e-12 {
            return Err(bad(cmd, "those four corners do not make a quadrilateral: three are in a line, or two are the same point"));
        }
        a.swap(best, column);
        for row in 0..8 {
            if row == column {
                continue;
            }
            let factor = a[row][column] / a[column][column];
            if factor == 0.0 {
                continue;
            }
            let pivot = a[column];
            for (cell, p) in a[row].iter_mut().zip(pivot.iter()).skip(column) {
                *cell -= factor * p;
            }
        }
    }
    let mut h = [0.0; 9];
    for i in 0..8 {
        h[i] = a[i][8] / a[i][i];
    }
    h[8] = 1.0;
    Ok(h)
}

fn rectify(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "canvas.rectify";
    let i = canvas_index(s, p, cmd)?;
    let canvas = s.doc.canvases[i].clone();
    let picture = picture_of(&canvas, cmd)?;
    let width = number(cmd, p, "width_mm", None, 0.01..=1e6)?.ok_or_else(|| bad(cmd, "give `width_mm`: the rectangle's true width"))?;
    let height = number(cmd, p, "height_mm", None, 0.01..=1e6)?.ok_or_else(|| bad(cmd, "give `height_mm`: the rectangle's true height"))?;
    let margin = number(cmd, p, "margin", Some(0.0), 0.0..=2.0)?.unwrap_or(0.0);
    let corners =
        p.get("corners").and_then(Value::as_array).filter(|a| a.len() == 4).ok_or_else(|| bad(cmd, "`corners` lists four [x, y] points"))?;
    let in_pixels = bool_(p, "pixel").unwrap_or(false);
    let (gw, gh) = (picture.w as f64, picture.h as f64);
    let mm_per_pixel = canvas.width / gw;
    let image_height = mm_per_pixel * gh;
    let (sin, cos) = canvas.angle.sin_cos();
    let to_pixel = |q: [f64; 2]| -> [f64; 2] {
        if in_pixels {
            return q;
        }
        let rel = [q[0] - canvas.center.x, q[1] - canvas.center.y];
        let mut u = rel[0] * cos + rel[1] * sin;
        let v = -rel[0] * sin + rel[1] * cos;
        if canvas.flip {
            u = -u;
        }
        [(u / canvas.width + 0.5) * gw, (0.5 - v / image_height) * gh]
    };
    let mut src = [[0.0; 2]; 4];
    for (k, c) in corners.iter().enumerate() {
        let q: Option<Vec<f64>> = c.as_array().filter(|a| a.len() == 2).map(|a| a.iter().filter_map(Value::as_f64).collect());
        let q = q
            .filter(|q| q.len() == 2 && q.iter().all(|x| x.is_finite() && x.abs() < 1e7))
            .ok_or_else(|| bad(cmd, format!("corner {} is not a point [x, y]", k + 1)))?;
        src[k] = to_pixel([q[0], q[1]]);
    }
    // Output size: the source's pixels per mm along the rectangle's longer side, within reason.
    let span = |a: [f64; 2], b: [f64; 2]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let px_per_mm = ((span(src[0], src[1]) + span(src[2], src[3])) / 2.0 / width).max((span(src[1], src[2]) + span(src[3], src[0])) / 2.0 / height);
    if !(px_per_mm > 0.0) || !px_per_mm.is_finite() {
        return Err(bad(cmd, "the corners lie on one another"));
    }
    let (out_w_mm, out_h_mm) = (width * (1.0 + 2.0 * margin), height * (1.0 + 2.0 * margin));
    let out_w = ((out_w_mm * px_per_mm).round() as usize).clamp(2, 8192);
    let out_h = ((out_h_mm * px_per_mm).round() as usize).clamp(2, 8192);
    let (mx, my) = (margin * width * px_per_mm, margin * height * px_per_mm);
    let (rw, rh) = (width * px_per_mm, height * px_per_mm);
    let dst = [[mx, my], [mx + rw, my], [mx + rw, my + rh], [mx, my + rh]];
    let h = homography(src, dst, cmd)?;
    let raw = picture.rgba.as_raw();
    let sample = |u: f64, v: f64, c: usize| -> f64 {
        let cu = u.clamp(0.0, gw - 1.0);
        let cv = v.clamp(0.0, gh - 1.0);
        let i0 = (cu as usize).min(picture.w - 1);
        let j0 = (cv as usize).min(picture.h - 1);
        let (i1, j1) = ((i0 + 1).min(picture.w - 1), (j0 + 1).min(picture.h - 1));
        let (fx, fy) = (cu - i0 as f64, cv - j0 as f64);
        let at = |x: usize, y: usize| f64::from(raw[4 * (y * picture.w + x) + c]);
        at(i0, j0) * (1.0 - fx) * (1.0 - fy) + at(i1, j0) * fx * (1.0 - fy) + at(i0, j1) * (1.0 - fx) * fy + at(i1, j1) * fx * fy
    };
    let mut out = RgbaImage::new(out_w as u32, out_h as u32);
    for y in 0..out_h {
        for x in 0..out_w {
            let (dx, dy) = (x as f64 + 0.5, y as f64 + 0.5);
            let w = h[6] * dx + h[7] * dy + h[8];
            if w.abs() < 1e-12 {
                continue;
            }
            let u = (h[0] * dx + h[1] * dy + h[2]) / w;
            let v = (h[3] * dx + h[4] * dy + h[5]) / w;
            let inside = u >= -0.5 && v >= -0.5 && u <= gw - 0.5 && v <= gh - 0.5;
            let px = out.get_pixel_mut(x as u32, y as u32);
            if inside {
                for c in 0..4 {
                    px.0[c] = sample(u, v, c).round().clamp(0.0, 255.0) as u8;
                }
            } else {
                px.0 = [255, 255, 255, 0];
            }
        }
    }
    let rectified = Picture { w: out_w, h: out_h, rgba: out };
    store(s, i, &rectified, cmd)?;
    let c = s.doc_mut().canvases.get_mut(i).ok_or_else(|| bad(cmd, "canvas"))?;
    c.width = out_w_mm;
    c.angle = 0.0;
    c.flip = false;
    Ok(json!({"canvas": canvas.id, "pixels": [out_w, out_h], "width_mm": out_w_mm, "height_mm": out_h_mm, "px_per_mm": px_per_mm}))
}

// --- optics -----------------------------------------------------------------------------------

fn ray_trace(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.ray_trace";
    let origin = p.get("origin").and_then(vec3).filter(|q| q.len() <= 1e7).ok_or_else(|| bad(cmd, "`origin` is a point [x, y, z]"))?;
    let mut dir = p.get("direction").and_then(vec3).and_then(|d| d.normalized()).ok_or_else(|| bad(cmd, "`direction` is [x, y, z], not zero"))?;
    let bounces = number(cmd, p, "bounces", Some(4.0), 0.0..=32.0)?.unwrap_or(4.0) as usize;
    let exit = number(cmd, p, "exit_mm", Some(200.0), 0.1..=1e6)?.unwrap_or(200.0);
    let st = s.world_state();
    let names: Vec<String> = match p.get("bodies") {
        None | Some(Value::Null) => st.bodies.iter().map(|b| b.name.clone()).collect(),
        Some(Value::Array(a)) => {
            let mut names = Vec::new();
            for v in a {
                let n = v.as_str().ok_or_else(|| bad(cmd, "`bodies` must be body names"))?;
                if st.body(n).is_none() {
                    return Err(bad(cmd, format!("no body `{n}`")));
                }
                names.push(n.to_string());
            }
            names
        }
        Some(_) => return Err(bad(cmd, "`bodies` must be a list of body names")),
    };
    if names.is_empty() {
        return Err(bad(cmd, "no bodies to reflect off"));
    }
    let meshes: Vec<(String, std::sync::Arc<solidcraft_geom::Mesh>)> =
        names.iter().filter_map(|n| st.body(n).map(|b| (n.clone(), b.mesh()))).collect();
    let indexes: Vec<MeshIndex> = meshes.iter().map(|(_, m)| MeshIndex::new(m.as_ref())).collect();
    let mut path = vec![origin];
    let mut hits = Vec::new();
    let mut current = origin;
    for _ in 0..bounces {
        let start = current + dir * 1e-6;
        let mut nearest: Option<(f64, usize, usize)> = None;
        for (b, (index, (_, mesh))) in indexes.iter().zip(&meshes).enumerate() {
            if let Some((t, tri)) = index.raycast(mesh.as_ref(), start, dir)
                && t > 1e-9
                && nearest.is_none_or(|(best, _, _)| t < best)
            {
                nearest = Some((t, b, tri));
            }
        }
        let Some((t, b, tri)) = nearest else { break };
        let (name, mesh) = &meshes[b];
        let hit = start + dir * t;
        let Some([a, bb, c]) = mesh.triangles.get(tri).and_then(|tr| mesh.tri(tr)) else { break };
        let Some(mut n) = (bb - a).cross(c - a).normalized() else { break };
        if n.dot(dir) > 0.0 {
            n = -n;
        }
        let reflected = dir - n * (2.0 * dir.dot(n));
        let Some(r) = reflected.normalized() else { break };
        path.push(hit);
        hits.push(json!({"body": name, "at": [hit.x, hit.y, hit.z], "normal": [n.x, n.y, n.z]}));
        dir = r;
        current = hit;
    }
    // Out of mirrors (the ray left), or out of bounces with a mirror still ahead.
    let escaped = bounces == 0 || hits.len() < bounces;
    path.push(current + dir * exit);
    let length: f64 = path.windows(2).map(|w| (w[1] - w[0]).len()).sum();
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| "Ray".to_string());
    let sketch = run(s, "sketch.create", json!({"plane": "XY", "name": name}))?;
    run(s, "sketch.options", json!({"sketch_3d": true}))?;
    let pts: Vec<[f64; 3]> = path.iter().map(|q| [q.x, q.y, q.z]).collect();
    run(s, "sketch.line3d", json!({"points": pts}))?;
    run(s, "sketch.finish", json!({}))?;
    Ok(json!({
        "sketch": sketch.get("sketch").cloned().unwrap_or(Value::Null),
        "points": pts,
        "hits": hits,
        "bounces": hits.len(),
        "length_mm": length,
        "escaped": escaped,
    }))
}

#[cfg(test)]
#[path = "image_tools_tests.rs"]
mod tests;
