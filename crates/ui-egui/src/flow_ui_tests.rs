//! The Flow Analysis dialog computes from its picks and reports the flow.

use serde_json::json;
use solidcraft_engine::Sel;
use solidcraft_engine::geom::Vec3;

use crate::dialogs::Kind;
use crate::dialogs_flow::{FlowForm, compute};
use crate::scenario::Harness;

#[test]
fn compute_answers_from_the_picks() {
    let mut h = Harness::new();
    h.app.home.open = false;
    h.app.set_origin_shown(false);
    h.app.run("solid.box", json!({"length": 60, "width": 28, "height": 12, "body_name": "Duct"})).unwrap();
    h.app.run("solid.box", json!({"length": 60, "width": 20, "height": 4, "corner": [0, 4, 4], "operation": "cut", "targets": ["Duct"]})).unwrap();
    h.app.start("inspect.flow");
    let d = h.app.dialog.as_mut().expect("the dialog opened");
    assert!(matches!(d.kind, Kind::Flow(_)));
    d.inputs[0].items = vec![Sel::Body { name: "Duct".into() }];
    let form = FlowForm { velocity: "0.005".into(), resolution: "60".into(), ..Default::default() };
    let r = compute(&mut h.app, &form, Some("Duct"), Some(Vec3::new(0.0, 14.0, 6.0)), Some(Vec3::new(60.0, 14.0, 6.0)));
    assert!(r.get("error").is_none(), "{r}");
    assert!(r["pressure_drop_pa"].as_f64().unwrap_or(0.0) > 0.0, "{r}");
    if let Some(d) = h.app.dialog.as_mut()
        && let Kind::Flow(f) = &mut d.kind
    {
        f.result = Some(r);
    }
    h.frames(2);
    assert!(matches!(h.app.dialog.as_ref().map(|d| &d.kind), Some(Kind::Flow(_))));
    let bad = FlowForm { velocity: "fast".into(), ..Default::default() };
    let r = compute(&mut h.app, &bad, Some("Duct"), Some(Vec3::ZERO), Some(Vec3::X));
    assert!(r["error"].as_str().unwrap_or_default().contains("m/s"), "{r}");
}
