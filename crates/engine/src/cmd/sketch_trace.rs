//! Tracing an inserted image into sketch profiles: the dark (or light) areas of a canvas become
//! closed polylines on the canvas's plane, placed exactly over the picture, so a scanned part or
//! a logo is drawn in one step and then extruded like anything else.

use serde_json::{Value, json};
use solidcraft_analysis::trace::{Bitmap, MAX_PIXELS, trace};
use solidcraft_doc::canvas::base64_decode;
use solidcraft_geom::Vec2;

use super::CommandSpec;
use crate::params::{bad, bool_, num, str_};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new("sketch.trace_canvas", "Trace Canvas", trace_canvas).at("3DEEE", "TRACE").icon("trace_canvas").params(
    "canvas: id or name of an inserted canvas; threshold?: 0 to 255 (default 128: pixels darker than this are traced); light?: true traces the light \
     areas instead; tolerance_px?: how far the drawn polyline may stray from the pixel staircase (default 1 pixel; 0 keeps every step); min_area_px?: \
     specks smaller than this are dropped (default 4); max_points?: refuse a trace with more points than this (default 20000); name? → a sketch on the \
     canvas's plane holding one closed polyline per traced loop, placed over the picture (its width, centre, angle and flip): sketch, contours, holes, \
     points, mm_per_pixel, dropped. Trace a clean, high-contrast picture: a photograph needs its background removed first",
)];

/// Most points a trace may draw.
const DEFAULT_MAX_POINTS: usize = 20_000;

fn trace_canvas(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "sketch.trace_canvas";
    let key = match p.get("canvas") {
        Some(Value::Number(n)) => n.to_string(),
        Some(Value::String(x)) => x.clone(),
        _ => return Err(bad(cmd, "`canvas` must be the id or the name of an inserted canvas")),
    };
    let canvas =
        s.doc.canvases.iter().find(|c| c.id.to_string() == key || c.name == key).cloned().ok_or_else(|| bad(cmd, format!("no canvas `{key}`")))?;
    let threshold = match p.get("threshold") {
        None | Some(Value::Null) => 128u8,
        Some(v) => v.as_f64().filter(|t| (0.0..=255.0).contains(t)).map(|t| t as u8).ok_or_else(|| bad(cmd, "`threshold` is 0 to 255"))?,
    };
    let light = bool_(p, "light").unwrap_or(false);
    let tolerance = num(p, "tolerance_px").unwrap_or(1.0);
    if !(0.0..=100.0).contains(&tolerance) {
        return Err(bad(cmd, "`tolerance_px` is 0 to 100"));
    }
    let min_area = num(p, "min_area_px").unwrap_or(4.0);
    if !(0.0..=1e9).contains(&min_area) {
        return Err(bad(cmd, "`min_area_px` must be a non-negative number"));
    }
    let max_points = match p.get("max_points") {
        None | Some(Value::Null) => DEFAULT_MAX_POINTS,
        Some(v) => v.as_u64().filter(|n| (3..=200_000).contains(n)).map(|n| n as usize).ok_or_else(|| bad(cmd, "`max_points` is 3 to 200000"))?,
    };
    let (w, h) = (canvas.pixels[0] as usize, canvas.pixels[1] as usize);
    if w == 0 || h == 0 || w.checked_mul(h).is_none_or(|n| n > MAX_PIXELS) {
        return Err(bad(cmd, format!("the canvas is too big to trace: at most {MAX_PIXELS} pixels")));
    }
    let bytes = base64_decode(&canvas.data).ok_or_else(|| bad(cmd, "the canvas holds no image"))?;
    let gray = super::image_tools::decode_limited(&bytes, cmd)?.to_luma8();
    let (gw, gh) = (gray.width() as usize, gray.height() as usize);
    if gw.checked_mul(gh).is_none_or(|n| n > MAX_PIXELS) {
        return Err(bad(cmd, format!("the image is too big to trace: at most {MAX_PIXELS} pixels")));
    }
    let bitmap = Bitmap::new(gw, gh, gray.into_raw()).ok_or_else(|| bad(cmd, "the image is empty"))?;
    let traced = trace(&bitmap, threshold, light, tolerance, min_area);
    if traced.contours.is_empty() {
        return Err(bad(cmd, "nothing to trace at this threshold: change `threshold`, or set `light` for a dark picture"));
    }
    let points: usize = traced.contours.iter().map(Vec::len).sum();
    if points > max_points {
        return Err(bad(
            cmd,
            format!("the trace has {points} points (at most {max_points}): raise `tolerance_px` or `min_area_px`, or `max_points`"),
        ));
    }
    // Pixel to plane: the canvas spans `width` across, its height by the aspect, centred,
    // mirrored by `flip`, turned by `angle`; image rows run downward.
    let mm_per_pixel = canvas.width / gw as f64;
    let height = mm_per_pixel * gh as f64;
    let (sin, cos) = canvas.angle.sin_cos();
    let place = |q: [f64; 2]| -> [f64; 2] {
        let mut u = (q[0] / gw as f64 - 0.5) * canvas.width;
        if canvas.flip {
            u = -u;
        }
        let v = (0.5 - q[1] / gh as f64) * height;
        [canvas.center.x + u * cos - v * sin, canvas.center.y + u * sin + v * cos]
    };
    let (vals, _) = s.doc.param_values();
    let component = usize::try_from(canvas.component).unwrap_or(0);
    let plane = s.doc.resolve_plane_in(&vals, &canvas.plane, component, Some(&s.model.state()))?;
    let name = str_(p, "name").map(str::to_string).unwrap_or_else(|| format!("Trace of {}", canvas.name));
    let sketch = s.execute(
        "sketch.create",
        &json!({"plane": {"origin": [plane.origin.x, plane.origin.y, plane.origin.z], "x_dir": [plane.x.x, plane.x.y, plane.x.z], "y_dir": [plane.y.x, plane.y.y, plane.y.z]}, "name": name}),
    )?;
    let mut holes = 0usize;
    for (contour, depth) in traced.contours.iter().zip(&traced.depths) {
        let pts: Vec<[f64; 2]> = contour.iter().map(|q| place(*q)).collect();
        if pts.len() < 3 {
            continue;
        }
        if depth % 2 == 1 {
            holes += 1;
        }
        s.execute("sketch.line", &json!({"points": pts, "closed": true}))?;
    }
    let finished = s.execute("sketch.finish", &json!({}))?;
    let _ = Vec2::ZERO;
    Ok(json!({
        "sketch": finished.get("sketch").cloned().or_else(|| sketch.get("sketch").cloned()),
        "name": name,
        "canvas": canvas.name,
        "contours": traced.contours.len(),
        "holes": holes,
        "points": points,
        "dropped": traced.dropped,
        "mm_per_pixel": mm_per_pixel,
        "threshold": threshold,
    }))
}

#[cfg(test)]
#[path = "sketch_trace_tests.rs"]
mod tests;
