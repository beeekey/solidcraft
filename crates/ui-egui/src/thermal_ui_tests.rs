//! The Thermal Analysis dialog computes from its picks and paints the temperature field.

use serde_json::json;
use solidcraft_engine::Sel;
use solidcraft_engine::geom::Vec3;

use crate::dialogs::Kind;
use crate::dialogs_stress::FacePick;
use crate::dialogs_thermal::{ThermalForm, compute};
use crate::scenario::Harness;

#[test]
fn compute_answers_from_the_picks_and_paints_a_field() {
    let mut h = Harness::new();
    h.app.home.open = false;
    h.app.ui.show_origin = false;
    h.app.run("solid.box", json!({"length": 60, "width": 10, "height": 60, "body_name": "Plate"})).unwrap();
    h.app.start("inspect.thermal");
    let d = h.app.dialog.as_mut().expect("the dialog opened");
    assert!(matches!(d.kind, Kind::Thermal(_)));
    d.inputs[0].items = vec![Sel::Body { name: "Plate".into() }];
    d.inputs[1].items = vec![Sel::Face { body: "Plate".into(), index: 0, point: Vec3::new(30.0, 5.0, 0.0) }];
    let form = ThermalForm { watts: "5".into(), resolution: "20".into(), emissivity: "0.8".into(), ..Default::default() };
    let (r, field) = compute(&mut h.app, &form, Some("Plate"), Some(FacePick { index: 0, point: Vec3::new(30.0, 5.0, 0.0) }), None);
    assert!(r.get("error").is_none(), "{r}");
    assert!(r["peak_c"].as_f64().unwrap_or(0.0) > 20.0, "{r}");
    let field = field.expect("a temperature field");
    assert!(field.max_mpa > field.min, "{field:?}");
    if let Some(d) = h.app.dialog.as_mut()
        && let Kind::Thermal(f) = &mut d.kind
    {
        f.result = Some(r);
        f.field = Some(field);
    }
    h.step(&json!({"call": "ui.view", "params": {"view": "fit"}})).unwrap();
    h.frames(2);
    assert!(matches!(h.app.dialog.as_ref().map(|d| &d.kind), Some(Kind::Thermal(_))));
    // Garbage in the form is an error in the dialog, not a panic.
    let bad = ThermalForm { watts: "lots".into(), ..Default::default() };
    let (r, field) = compute(&mut h.app, &bad, Some("Plate"), Some(FacePick { index: 0, point: Vec3::new(30.0, 5.0, 0.0) }), None);
    assert!(r["error"].as_str().unwrap_or_default().contains("watts"), "{r}");
    assert!(field.is_none());
}
