use serde_json::{Value, json};
use solidcraft_doc::canvas::base64_encode;

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

/// A PNG of `w × h` white pixels with a black rectangle (x, y, w, h) and, optionally, a white
/// hole inside it.
fn png(w: u32, h: u32, dark: (u32, u32, u32, u32), hole: Option<(u32, u32, u32, u32)>) -> String {
    let mut img = image::GrayImage::from_pixel(w, h, image::Luma([255u8]));
    for y in dark.1..dark.1 + dark.3 {
        for x in dark.0..dark.0 + dark.2 {
            img.put_pixel(x, y, image::Luma([0u8]));
        }
    }
    if let Some(hole) = hole {
        for y in hole.1..hole.1 + hole.3 {
            for x in hole.0..hole.0 + hole.2 {
                img.put_pixel(x, y, image::Luma([255u8]));
            }
        }
    }
    let mut bytes: Vec<u8> = Vec::new();
    image::DynamicImage::ImageLuma8(img).write_to(&mut std::io::Cursor::new(&mut bytes), image::ImageFormat::Png).unwrap_or_default();
    base64_encode(&bytes)
}

#[test]
fn a_dark_rectangle_on_a_canvas_becomes_a_profile_the_picture_s_size() {
    // A 100 × 50 pixel picture inserted 200 mm wide, centred at (10, 20): 2 mm per pixel. The
    // black rectangle from pixel (20, 10) to (60, 40) lands at x = 10 + (20..60 − 50)·2 =
    // −50..30, y = 20 + (25 − 10..40)·2 = −10..50, and extrudes to that area.
    let mut s = Session::default();
    run(&mut s, "canvas.insert", json!({"data": png(100, 50, (20, 10, 40, 30), None), "width": 200, "center": [10, 20], "name": "Picture"}));
    let r = run(&mut s, "sketch.trace_canvas", json!({"canvas": "Picture"}));
    assert_eq!(r["contours"], 1, "{r}");
    assert_eq!(r["holes"], 0);
    assert_eq!(r["points"], 4);
    assert_eq!(r["mm_per_pixel"], 2.0);
    run(&mut s, "solid.extrude", json!({"distance": 5, "operation": "new"}));
    let st = s.world_state();
    let b = st.bodies.first().unwrap_or_else(|| panic!("a body"));
    let m = solidcraft_kernel::measure(&b.body).unwrap_or_else(|_| panic!("measure"));
    assert!((m.volume - 80.0 * 60.0 * 5.0).abs() < 1e-6, "{}", m.volume);
    let bb = b.mesh().bounds();
    assert!((bb.min.x + 50.0).abs() < 1e-6 && (bb.max.x - 30.0).abs() < 1e-6, "{bb:?}");
    assert!((bb.min.y + 10.0).abs() < 1e-6 && (bb.max.y - 50.0).abs() < 1e-6, "{bb:?}");
}

#[test]
fn a_hole_in_the_picture_is_a_hole_in_the_profile_and_the_canvas_turns_and_flips_with_it() {
    let mut s = Session::default();
    run(
        &mut s,
        "canvas.insert",
        json!({"data": png(80, 80, (10, 10, 60, 60), Some((30, 30, 20, 20))), "width": 80, "angle": 90, "flip": true, "name": "Ring"}),
    );
    let r = run(&mut s, "sketch.trace_canvas", json!({"canvas": "Ring", "threshold": 100}));
    assert_eq!(r["contours"], 2, "{r}");
    assert_eq!(r["holes"], 1);
    run(&mut s, "solid.extrude", json!({"distance": 2, "operation": "new", "profiles": [{"point": [-25, 0]}]}));
    let st = s.world_state();
    let b = st.bodies.first().unwrap_or_else(|| panic!("a body"));
    let m = solidcraft_kernel::measure(&b.body).unwrap_or_else(|_| panic!("measure"));
    assert!((m.volume - (60.0 * 60.0 - 20.0 * 20.0) * 2.0).abs() < 1e-6, "{}", m.volume);
}

#[test]
fn a_blank_canvas_and_bad_parameters_are_refused() {
    let mut s = Session::default();
    run(&mut s, "canvas.insert", json!({"data": png(20, 20, (0, 0, 0, 0), None), "width": 20, "name": "Blank"}));
    let e = s.execute("sketch.trace_canvas", &json!({"canvas": "Blank"})).err().map(|e| e.to_string()).unwrap_or_default();
    assert!(e.contains("nothing to trace"), "{e}");
    let e = s.execute("sketch.trace_canvas", &json!({"canvas": "Blank", "threshold": 300})).err().map(|e| e.to_string()).unwrap_or_default();
    assert!(e.contains("threshold"), "{e}");
    let e = s.execute("sketch.trace_canvas", &json!({"canvas": "Nope"})).err().map(|e| e.to_string()).unwrap_or_default();
    assert!(e.contains("no canvas"), "{e}");
    assert!(s.world_state().bodies.is_empty());
}
