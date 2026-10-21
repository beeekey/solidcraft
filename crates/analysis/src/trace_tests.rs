use super::{Bitmap, area, contains_point, simplify, trace, trace_contours};

/// A white raster with the given dark rectangles (x, y, w, h).
fn picture(width: usize, height: usize, dark: &[(usize, usize, usize, usize)]) -> Bitmap {
    let mut gray = vec![255u8; width * height];
    for &(x0, y0, w, h) in dark {
        for y in y0..y0 + h {
            for x in x0..x0 + w {
                gray[y * width + x] = 0;
            }
        }
    }
    Bitmap::new(width, height, gray).unwrap_or(Bitmap { width: 0, height: 0, gray: Vec::new() })
}

#[test]
fn a_dark_square_traces_to_one_loop_of_four_corners() {
    let bitmap = picture(10, 10, &[(2, 3, 4, 4)]);
    let raw = trace_contours(&bitmap, 128, false);
    assert_eq!(raw.len(), 1);
    assert_eq!(raw[0].len(), 16, "the staircase has a point per pixel edge");
    let simple = simplify(&raw[0], 0.5);
    assert_eq!(simple.len(), 4, "{simple:?}");
    assert!((area(&simple) - 16.0).abs() < 1e-9);
    for corner in [[2.0, 3.0], [6.0, 3.0], [6.0, 7.0], [2.0, 7.0]] {
        assert!(simple.contains(&corner), "{simple:?} lacks {corner:?}");
    }
    assert!(contains_point(&simple, 4.0, 5.0) && !contains_point(&simple, 1.0, 1.0));
}

#[test]
fn a_ring_is_an_outer_loop_and_a_hole() {
    let mut bitmap = picture(12, 12, &[(1, 1, 10, 10)]);
    for y in 4..8 {
        for x in 4..8 {
            bitmap.gray[y * 12 + x] = 255;
        }
    }
    let traced = trace(&bitmap, 128, false, 0.5, 1.0);
    assert_eq!(traced.contours.len(), 2, "{traced:?}");
    assert_eq!(traced.depths, vec![0, 1]);
    assert!((area(&traced.contours[0]) - 100.0).abs() < 1e-9);
    assert!((area(&traced.contours[1]) - 16.0).abs() < 1e-9);
}

#[test]
fn light_areas_can_be_traced_instead_and_specks_are_dropped() {
    let mut bitmap = picture(10, 10, &[(0, 0, 10, 10)]);
    bitmap.gray[3 * 10 + 3] = 255;
    // Everything is dark but one light pixel: tracing the light finds that pixel.
    let light = trace(&bitmap, 128, true, 0.5, 0.0);
    assert_eq!(light.contours.len(), 1);
    assert!((area(&light.contours[0]) - 1.0).abs() < 1e-9);
    // Tracing the dark finds the frame and the one-pixel hole; a minimum area drops the hole.
    let dark = trace(&bitmap, 128, false, 0.5, 4.0);
    assert_eq!(dark.contours.len(), 1, "{dark:?}");
    assert_eq!(dark.dropped, 1);
}

#[test]
fn pixels_touching_at_a_corner_stay_separate_loops() {
    let bitmap = picture(6, 6, &[(1, 1, 2, 2), (3, 3, 2, 2)]);
    let raw = trace_contours(&bitmap, 128, false);
    assert_eq!(raw.len(), 2, "{raw:?}");
    for loop_ in &raw {
        assert!((area(loop_) - 4.0).abs() < 1e-9, "{loop_:?}");
    }
}

#[test]
fn hostile_rasters_are_refused_not_panicked() {
    assert!(Bitmap::new(0, 5, Vec::new()).is_none());
    assert!(Bitmap::new(5, 5, vec![0; 3]).is_none());
    assert!(Bitmap::new(usize::MAX, 2, Vec::new()).is_none());
    assert!(Bitmap::new(5000, 5000, vec![0; 25_000_000]).is_none());
    let blank = picture(4, 4, &[]);
    assert!(trace(&blank, 128, false, 1.0, 1.0).contours.is_empty());
    assert_eq!(simplify(&[], 1.0).len(), 0);
    assert!(!contains_point(&[[0.0, 0.0], [1.0, 0.0]], 0.5, 0.5));
}
