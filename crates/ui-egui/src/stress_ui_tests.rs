//! The Stress Analysis dialog: it opens from its command, computes from the picked body and
//! faces, shows the answer, and paints the stress field without upsetting a frame.

use serde_json::json;
use solidcraft_engine::Sel;
use solidcraft_engine::geom::Vec3;

use crate::dialogs::Kind;
use crate::dialogs_stress::{FacePick, StressForm, compute, ramp};
use crate::scenario::Harness;

/// A 100 × 10 × 10 bar along X, with the dialog open and its three picks made.
fn bar_with_dialog() -> Harness {
    let mut h = Harness::new();
    h.app.home.open = false;
    h.app.set_origin_shown(false);
    h.app.run("solid.box", json!({"length": 100, "width": 10, "height": 10, "body_name": "Bar"})).unwrap();
    h.app.start("inspect.stress");
    let d = h.app.dialog.as_mut().expect("the dialog opened");
    assert!(matches!(d.kind, Kind::Stress(_)), "a Stress dialog");
    d.inputs[0].items = vec![Sel::Body { name: "Bar".into() }];
    d.inputs[1].items = vec![Sel::Face { body: "Bar".into(), index: 0, point: Vec3::new(0.0, 5.0, 5.0) }];
    d.inputs[2].items = vec![Sel::Face { body: "Bar".into(), index: 1, point: Vec3::new(100.0, 5.0, 5.0) }];
    h
}

#[test]
fn compute_answers_from_the_picks_and_paints_a_field() {
    let mut h = bar_with_dialog();
    let form = StressForm { force: ["0".into(), "0".into(), "-100".into()], resolution: "20".into(), ..Default::default() };
    let (r, field) = compute(
        &mut h.app,
        &form,
        Some("Bar"),
        Some(FacePick { index: 0, point: Vec3::new(0.0, 5.0, 5.0) }),
        Some(FacePick { index: 1, point: Vec3::new(100.0, 5.0, 5.0) }),
    );
    assert!(r.get("error").is_none(), "{r}");
    assert!(r["max_von_mises_mpa"].as_f64().unwrap_or(0.0) > 0.0, "{r}");
    let field = field.expect("a stress field to paint");
    assert_eq!(field.grid[0], 20);
    assert!(field.max_mpa > 0.0);
    assert_eq!(field.cells.iter().filter(|c| c.is_some()).count(), r["voxels"].as_u64().unwrap_or(0) as usize);
    // Put the answer in the dialog and draw a frame: the overlay paints the cells.
    if let Some(d) = h.app.dialog.as_mut()
        && let Kind::Stress(f) = &mut d.kind
    {
        f.result = Some(r);
        f.field = Some(field);
    }
    h.step(&json!({"call": "ui.view", "params": {"view": "fit"}})).unwrap();
    h.frames(2);
    assert!(matches!(h.app.dialog.as_ref().map(|d| &d.kind), Some(Kind::Stress(_))), "the dialog stays open");
}

#[test]
fn a_bad_force_is_an_error_in_the_dialog_not_a_panic() {
    let mut h = bar_with_dialog();
    let form = StressForm { force: ["ten".into(), "0".into(), "0".into()], ..Default::default() };
    let (r, field) = compute(
        &mut h.app,
        &form,
        Some("Bar"),
        Some(FacePick { index: 0, point: Vec3::new(0.0, 5.0, 5.0) }),
        Some(FacePick { index: 1, point: Vec3::new(100.0, 5.0, 5.0) }),
    );
    assert!(r["error"].as_str().unwrap_or_default().contains("newtons"), "{r}");
    assert!(field.is_none());
}

#[test]
fn the_colour_ramp_runs_through_the_themes_curvature_stops() {
    let stops = crate::theme::Tokens::get().analysis.curvature;
    assert_eq!(ramp(0.0), crate::theme::srgb(stops[0]));
    assert_eq!(ramp(0.5), crate::theme::srgb(stops[1]));
    assert_eq!(ramp(1.0), crate::theme::srgb(stops[2]));
    assert_ne!(ramp(0.0), ramp(1.0));
    assert_eq!(ramp(-3.0), ramp(0.0));
    assert_eq!(ramp(7.0), ramp(1.0));
}
