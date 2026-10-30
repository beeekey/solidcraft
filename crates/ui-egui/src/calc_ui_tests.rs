//! The calculators' dialog: it opens from each calculator command, computes from its fields,
//! parses a tolerance chain, and turns a bad number into an error row rather than a panic.

use serde_json::json;

use crate::dialogs::Kind;
use crate::dialogs_calc::{CALCULATORS, CalcForm, FieldKind, compute, label_of, parse_chain, show_number};
use crate::scenario::Harness;

#[test]
fn every_calculator_opens_its_dialog_and_the_bolted_joint_computes_a_verdict() {
    let mut h = Harness::new();
    h.app.home.open = false;
    for id in CALCULATORS {
        h.app.start(id);
        let d = h.app.dialog.as_ref().unwrap_or_else(|| panic!("{id} opened no dialog"));
        match &d.kind {
            Kind::Calculator(f) => assert_eq!(f.command, *id),
            other => panic!("{id} opened {other:?}"),
        }
        assert!(d.inputs.is_empty(), "{id} takes no picks");
    }
    h.app.start("inspect.bolted_joint");
    let mut form = CalcForm::for_command("inspect.bolted_joint").expect("a form");
    let r = compute(&mut h.app, &form, &[]);
    assert!(r.get("error").is_none(), "{r}");
    assert!(r["verdict"].as_str().unwrap_or_default().starts_with("M8 8.8"), "{r}");
    assert!(r["tightening_torque_nm"].as_f64().unwrap_or(0.0) > 10.0, "{r}");
    form.result = Some(r);
    if let Some(d) = h.app.dialog.as_mut() {
        d.kind = Kind::Calculator(Box::new(form));
    }
    h.frames(2);
    assert!(matches!(h.app.dialog.as_ref().map(|d| &d.kind), Some(Kind::Calculator(_))), "the dialog stays open");
    // A calculator changes nothing: no undo step, no body.
    assert!(h.app.session.undo.is_empty());
    assert!(h.app.session.world_state().bodies.is_empty());
}

#[test]
fn the_fields_become_the_commands_parameters() {
    let mut form = CalcForm::for_command("inspect.thread_engagement").expect("a form");
    for f in &mut form.fields {
        match f.key {
            "material" => f.choice = 1,
            "through" => f.on = true,
            "engagement_mm" => f.value = "12,5".into(),
            _ => {}
        }
    }
    assert_eq!(form.params(), Ok(json!({"size": "M6", "material": "aluminium", "engagement_mm": 12.5, "through": true})));
    // An empty number is left to the command's default; an empty choice is not sent.
    let damping = CalcForm::for_command("inspect.damping").expect("a form");
    assert!(matches!(damping.fields.iter().find(|f| f.key == "material").map(|f| f.kind), Some(FieldKind::Choice(_))));
    assert_eq!(damping.params(), Ok(json!({"mass_kg": 25.0, "rate_n_mm": 12.0, "excitation_hz": 10.0})));
    assert_eq!(CalcForm::for_command("solid.extrude"), None);
}

#[test]
fn a_tolerance_chain_is_parsed_line_by_line() {
    let chain = parse_chain("Plate 10 ±0.1\n5 +-0.05 -1\nEnd cap 15 14.9 15.2\n\n  Gap 2 +/-0.2 +1").expect("a chain");
    assert_eq!(
        chain,
        json!([
            {"name": "Plate", "nominal_mm": 10.0, "plus_minus_mm": 0.1, "sensitivity": 1.0},
            {"nominal_mm": 5.0, "plus_minus_mm": 0.05, "sensitivity": -1.0},
            {"name": "End cap", "nominal_mm": 15.0, "lower_mm": 14.9, "upper_mm": 15.2, "sensitivity": 1.0},
            {"name": "Gap", "nominal_mm": 2.0, "plus_minus_mm": 0.2, "sensitivity": 1.0},
        ])
    );
    assert!(parse_chain("").unwrap_err().contains("at least one"));
    assert!(parse_chain("Plate").unwrap_err().contains("line 1"));
    assert!(parse_chain("Plate 10 ±0.1 0.2 0.3").unwrap_err().contains("line 1"));
    // The whole form runs through the command.
    let mut h = Harness::new();
    let form = CalcForm::for_command("inspect.tolerance_stack").expect("a form");
    let r = compute(&mut h.app, &form, &[]);
    assert!(r.get("error").is_none(), "{r}");
    assert!((r["nominal_mm"].as_f64().unwrap_or(0.0) + 0.5).abs() < 1e-9, "{r}");
    assert!(r["ranked"].as_array().is_some_and(|a| a.len() == 3), "{r}");
}

#[test]
fn a_bad_number_is_an_error_row_not_a_panic() {
    let mut h = Harness::new();
    let mut form = CalcForm::for_command("inspect.bolted_joint").expect("a form");
    if let Some(f) = form.fields.iter_mut().find(|f| f.key == "clamped_mm") {
        f.value = "thirty".into();
    }
    let r = compute(&mut h.app, &form, &[]);
    assert!(r["error"].as_str().unwrap_or_default().contains("Clamped length (mm) needs a number"), "{r}");
    // A refusal from the command is an error too.
    let mut form = CalcForm::for_command("inspect.bolted_joint").expect("a form");
    if let Some(f) = form.fields.iter_mut().find(|f| f.key == "size") {
        f.value = "M7".into();
    }
    let r = compute(&mut h.app, &form, &[]);
    assert!(r["error"].as_str().unwrap_or_default().contains("metric size"), "{r}");
}

#[test]
fn reply_keys_read_as_labels_and_numbers_are_short() {
    assert_eq!(label_of("bolt_stress_mpa"), "Bolt stress (MPa)");
    assert_eq!(label_of("tightening_torque_nm"), "Tightening torque (N·m)");
    assert_eq!(label_of("stress_area_mm2"), "Stress area (mm²)");
    assert_eq!(label_of("separation_load_n"), "Separation load (N)");
    assert_eq!(label_of("load_factor"), "Load factor");
    assert_eq!(show_number(26100.0), "26100");
    assert_eq!(show_number(0.123456), "0.123");
    assert_eq!(show_number(2.5), "2.5");
    assert_eq!(show_number(f64::NAN), "—");
}
