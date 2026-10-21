use image::{ImageFormat, Rgba, RgbaImage};
use serde_json::{Value, json};
use solidcraft_doc::canvas::base64_encode;

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or(f64::NAN)
}

/// A 64 × 64 picture: a light ground, a dark 20 × 20 square with a 4 × 4 hole, and a speck.
fn picture() -> String {
    let mut img = RgbaImage::from_pixel(64, 64, Rgba([235, 235, 230, 255]));
    for y in 22..42 {
        for x in 22..42 {
            img.put_pixel(x, y, Rgba([20, 20, 20, 255]));
        }
    }
    for y in 30..34 {
        for x in 30..34 {
            img.put_pixel(x, y, Rgba([235, 235, 230, 255]));
        }
    }
    img.put_pixel(5, 50, Rgba([30, 30, 30, 255]));
    let mut bytes = Vec::new();
    image::DynamicImage::ImageRgba8(img).write_to(&mut std::io::Cursor::new(&mut bytes), ImageFormat::Png).unwrap();
    base64_encode(&bytes)
}

fn canvas_pixels(s: &Session) -> (u32, u32, RgbaImage) {
    let c = &s.doc.canvases[0];
    let img = image::load_from_memory(&c.bytes().unwrap()).unwrap().to_rgba8();
    (c.pixels[0], c.pixels[1], img)
}

#[test]
fn the_wand_paints_the_background_out_and_the_square_traces_alone() {
    let mut s = Session::default();
    run(&mut s, "canvas.insert", json!({"data": picture(), "plane": "XY", "width": 64, "name": "Photo"}));
    let r = run(&mut s, "canvas.remove_background", json!({"canvas": "Photo", "seed_px": [2, 2]}));
    // Everything but the square and the speck, plus the hole which the flood cannot reach.
    let expected = (64.0 * 64.0 - 400.0 - 1.0) / (64.0 * 64.0);
    assert!((f(&r["background_share"]) - expected).abs() < 0.005, "{r}");
    let (_, _, img) = canvas_pixels(&s);
    assert_eq!(img.get_pixel(2, 2).0, [255, 255, 255, 255]);
    assert_eq!(img.get_pixel(30, 25).0, [20, 20, 20, 255], "the square stays");
    assert_eq!(img.get_pixel(31, 31).0, [235, 235, 230, 255], "the hole is not joined to the seed");
    let t = run(&mut s, "sketch.trace_canvas", json!({"canvas": "Photo", "name": "Traced"}));
    assert_eq!(t["contours"], 2, "the square and its hole: {t}");
    // Non-contiguous: the hole goes too.
    let mut s2 = Session::default();
    run(&mut s2, "canvas.insert", json!({"data": picture(), "plane": "XY", "width": 64, "name": "Photo"}));
    let r = run(&mut s2, "canvas.remove_background", json!({"canvas": "Photo", "contiguous": false, "fill": "transparent"}));
    assert!(f(&r["background_share"]) > expected, "{r}");
    let (_, _, img) = canvas_pixels(&s2);
    assert_eq!(img.get_pixel(31, 31).0[3], 0, "the hole is transparent");
    let mut err = |p: Value| s2.execute("canvas.remove_background", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"canvas": "Photo", "seed_px": [500, 2]})).contains("seed_px"));
    assert!(err(json!({"canvas": "Photo", "fill": "pink"})).contains("fill"));
    assert!(err(json!({"canvas": "Nothing"})).contains("Nothing"));
}

#[test]
fn select_object_keeps_the_biggest_piece_with_its_hole_filled() {
    let mut s = Session::default();
    run(&mut s, "canvas.insert", json!({"data": picture(), "plane": "XY", "width": 64, "name": "Photo"}));
    let r = run(&mut s, "canvas.select_object", json!({"canvas": "Photo"}));
    assert!((f(&r["object_share"]) - 400.0 / 4096.0).abs() < 0.002, "the square with its hole filled, the speck gone: {r}");
    assert_eq!(r["touches_border"], false);
    assert!((f(&r["size_mm"]) - 64.0 * (400.0_f64 / 4096.0).sqrt()).abs() < 0.1, "{r}");
    let (_, _, img) = canvas_pixels(&s);
    assert_eq!(img.get_pixel(5, 50).0, [255, 255, 255, 255], "the speck is painted out");
    assert_eq!(img.get_pixel(31, 31).0, [235, 235, 230, 255], "the hole is kept as part of the object");
    assert_eq!(img.get_pixel(25, 25).0, [20, 20, 20, 255]);
    // A blank picture is all background.
    let mut blank = RgbaImage::from_pixel(16, 16, Rgba([200, 200, 200, 255]));
    blank.put_pixel(8, 8, Rgba([201, 200, 200, 255]));
    let mut bytes = Vec::new();
    image::DynamicImage::ImageRgba8(blank).write_to(&mut std::io::Cursor::new(&mut bytes), ImageFormat::Png).unwrap();
    let mut s2 = Session::default();
    run(&mut s2, "canvas.insert", json!({"data": base64_encode(&bytes), "plane": "XY", "width": 16, "name": "Blank"}));
    assert!(s2.execute("canvas.select_object", &json!({"canvas": "Blank"})).unwrap_err().to_string().contains("nothing"));
}

#[test]
fn rectify_squares_up_a_skewed_rectangle() {
    // The square's corners given as a skewed quadrilateral (in canvas mm: 1 px = 1 mm, centred):
    // the picture is resampled so those map to a 40 × 40 rectangle.
    let mut s = Session::default();
    run(&mut s, "canvas.insert", json!({"data": picture(), "plane": "XY", "width": 64, "name": "Photo"}));
    // Pixel (22, 22) is at plane (22 − 32, 32 − 22) = (−10, 10); rows run downward.
    let r = run(
        &mut s,
        "canvas.rectify",
        json!({"canvas": "Photo", "corners": [[-10, 10], [10, 10], [10, -10], [-10, -10]], "width_mm": 40, "height_mm": 40, "margin": 0.25}),
    );
    assert!((f(&r["width_mm"]) - 60.0).abs() < 1e-9, "{r}");
    let (w, h, img) = canvas_pixels(&s);
    assert_eq!((w, h), (30, 30), "{r}");
    // The square now fills the middle 20 × 20 of a 30 × 30 picture (1 px per 2 mm), dark inside and light around.
    assert!(img.get_pixel(8, 8).0[0] < 60, "{:?}", img.get_pixel(8, 8));
    assert!(img.get_pixel(2, 2).0[0] > 200, "{:?}", img.get_pixel(2, 2));
    assert!(s.doc.canvases[0].angle == 0.0 && (s.doc.canvases[0].width - 60.0).abs() < 1e-9);
    let mut err = |p: Value| s.execute("canvas.rectify", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"canvas": "Photo", "corners": [[0, 0], [1, 0], [2, 0], [3, 0]], "width_mm": 10, "height_mm": 10})).contains("quadrilateral"));
    assert!(err(json!({"canvas": "Photo", "corners": [[0, 0], [1, 0], [1, 1]], "width_mm": 10, "height_mm": 10})).contains("four"));
    assert!(err(json!({"canvas": "Photo", "corners": [[0, 0], [1, 0], [1, 1], [0, 1]], "height_mm": 10})).contains("width_mm"));
}

#[test]
fn a_ray_reflects_off_a_block_into_a_3d_sketch() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 20, "width": 40, "height": 40, "corner": [100, -20, 0], "body_name": "Mirror"}));
    let r = run(&mut s, "inspect.ray_trace", json!({"origin": [0, 0, 20], "direction": [1, 0, 0], "bounces": 3, "exit_mm": 50}));
    assert_eq!(r["bounces"], 1, "{r}");
    assert_eq!(r["hits"][0]["body"], "Mirror");
    let hit: Vec<f64> = r["hits"][0]["at"].as_array().map(|a| a.iter().map(f).collect()).unwrap_or_default();
    assert!((hit[0] - 100.0).abs() < 1e-6 && hit[2].abs() < 20.0 + 1e-6, "{r}");
    let pts = r["points"].as_array().cloned().unwrap_or_default();
    assert_eq!(pts.len(), 3);
    assert!((f(&pts[2][0]) - 50.0).abs() < 1e-6, "reflected straight back and drawn 50 mm: {r}");
    assert_eq!(r["escaped"], true);
    assert!((f(&r["length_mm"]) - 150.0).abs() < 1e-6);
    assert!(r["sketch"].is_number() || r["sketch"].is_string(), "{r}");
    // Between two facing mirrors the ray runs out of bounces.
    run(&mut s, "solid.box", json!({"length": 20, "width": 40, "height": 40, "corner": [-120, -20, 0], "body_name": "Other"}));
    let r = run(&mut s, "inspect.ray_trace", json!({"origin": [0, 0, 20], "direction": [1, 0, 0], "bounces": 4}));
    assert_eq!(r["bounces"], 4, "{r}");
    assert_eq!(r["escaped"], false);
    let mut err = |p: Value| s.execute("inspect.ray_trace", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"direction": [1, 0, 0]})).contains("origin"));
    assert!(err(json!({"origin": [0, 0, 0], "direction": [0, 0, 0]})).contains("direction"));
    assert!(err(json!({"origin": [0, 0, 0], "direction": [1, 0, 0], "bodies": ["Nothing"]})).contains("Nothing"));
}
