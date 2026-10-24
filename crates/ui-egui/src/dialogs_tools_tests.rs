//! The 3DEEE tab's forms: every tool opens a dialog, its fields become the command's
//! parameters (nested keys, JSON lists, points), picks land as names and points, and the
//! defaults build something.

use serde_json::json;
use solidcraft_engine::Sel;
use solidcraft_engine::geom::Vec3;

use crate::dialogs::Kind;
use crate::dialogs_calc::{CalcForm, compute};
use crate::dialogs_tools::TOOLS;
use crate::scenario::Harness;
use crate::selection::SelInput;

#[test]
fn every_tool_is_on_the_3deee_tab_and_opens_its_form() {
    let mut h = Harness::new();
    h.app.home.open = false;
    for id in TOOLS {
        let spec = solidcraft_engine::find_command(id).unwrap_or_else(|| panic!("{id} is not a command"));
        assert_eq!(spec.tab, "3DEEE", "{id} is on tab {:?}", spec.tab);
        assert!(!spec.panel.is_empty(), "{id} has no panel");
        h.app.start(id);
        let d = h.app.dialog.as_ref().unwrap_or_else(|| panic!("{id} opened no dialog"));
        match &d.kind {
            Kind::Calculator(f) => {
                assert_eq!(f.command, *id);
                assert_eq!(d.inputs.len(), f.picks.len(), "{id}: inputs follow the picks");
                // The defaults are valid parameters.
                f.params().unwrap_or_else(|e| panic!("{id}: {e}"));
            }
            other => panic!("{id} opened {other:?}"),
        }
        h.frames(1);
        crate::dialogs::cancel(&mut h.app);
    }
    // Every panel of the tab has at least one command, and every command of the tab has a form.
    for panel in crate::toolbar::panels("3DEEE") {
        let on_panel: Vec<&str> =
            solidcraft_engine::command_specs().into_iter().filter(|c| c.tab == "3DEEE" && c.panel == panel).map(|c| c.id).collect();
        assert!(!on_panel.is_empty(), "panel {panel} is empty");
        for id in on_panel {
            assert!(CalcForm::for_command(id).is_some(), "{id} on 3DEEE › {panel} has no form");
        }
    }
}

#[test]
fn fields_become_parameters_with_nested_keys_lists_and_points() {
    let frame = CalcForm::for_command("design.frame").expect("a form");
    let p = frame.params().expect("parameters");
    assert_eq!(p["profile"], json!({"family": "square_tube", "size": "40x40x3"}));
    assert_eq!(p["members"].as_array().map(Vec::len), Some(4));
    assert_eq!(p["ends"], json!("mitre"));
    assert_eq!(p["trim"], json!(true));
    assert!(p.get("rotation_deg").is_none(), "an empty number is left to the command");
    let pulley = CalcForm::for_command("design.pulley").expect("a form");
    assert_eq!(pulley.params().expect("parameters")["centre"], json!([0.0, 0.0, 0.0]));
    let mut bad = CalcForm::for_command("design.pipe_route").expect("a form");
    if let Some(f) = bad.fields.iter_mut().find(|f| f.key == "points") {
        f.value = "[[0, 0, 0], [100,".into();
    }
    assert!(bad.params().unwrap_err().contains("not valid JSON"));
    let mut bad = CalcForm::for_command("inspect.fibre").expect("a form");
    if let Some(f) = bad.fields.iter_mut().find(|f| f.key == "force_n") {
        f.value = "2000, 0".into();
    }
    assert!(bad.params().unwrap_err().contains("three numbers"));
}

#[test]
fn picks_land_as_body_names_and_points() {
    let h = Harness::new();
    let weld = CalcForm::for_command("design.fillet_weld").expect("a form");
    let mut inputs: Vec<SelInput> = weld.inputs();
    assert_eq!(inputs.iter().map(|i| i.label).collect::<Vec<_>>(), ["Body", "Edge"]);
    assert!(weld.params_from(&h.app.session, &inputs).unwrap_err().contains("pick the body"));
    inputs[0].items = vec![Sel::Face { body: "Bracket".into(), index: 2, point: Vec3::new(1.0, 2.0, 3.0) }];
    assert!(weld.params_from(&h.app.session, &inputs).unwrap_err().contains("pick the edge"));
    inputs[1].items = vec![Sel::Edge { body: "Bracket".into(), index: 4, point: Vec3::new(20.0, 5.0, 5.0) }];
    let p = weld.params_from(&h.app.session, &inputs).expect("parameters");
    assert_eq!(p, json!({"body": "Bracket", "edge": [20.0, 5.0, 5.0], "leg_mm": 4.0}));
    // An optional body pick left empty is left to the command's default.
    let print = CalcForm::for_command("inspect.print").expect("a form");
    let p = print.params_from(&h.app.session, &print.inputs()).expect("parameters");
    assert!(p.get("body").is_none() && p["critical_angle_deg"] == json!(45.0), "{p}");
}

#[test]
fn a_generator_applies_on_ok_and_a_builder_reports_what_it_built() {
    let mut h = Harness::new();
    h.app.home.open = false;
    h.app.start("design.spur_gear");
    let d = h.app.dialog.clone().expect("the dialog");
    assert!(matches!(&d.kind, Kind::Calculator(f) if f.apply));
    crate::dialogs::run_dialog(&mut h.app, &d).expect("the gear builds");
    assert_eq!(h.app.session.world_state().bodies.len(), 1);
    // Brickify: a block picked, rods and profiles off, Build; the bill of materials comes back.
    h.app.run("solid.box", json!({"length": 32, "width": 16, "height": 19.2, "corner": [100, 0, 0], "body_name": "Block"})).expect("a block");
    crate::dialogs::cancel(&mut h.app);
    h.app.start("construct.brickify");
    let mut form = match h.app.dialog.as_ref().map(|d| &d.kind) {
        Some(Kind::Calculator(f)) => (**f).clone(),
        other => panic!("{other:?}"),
    };
    assert!(!form.apply && form.button == "Build");
    for f in &mut form.fields {
        if matches!(f.key, "profiles" | "rods") {
            f.on = false;
        }
    }
    let mut inputs = form.inputs();
    inputs[0].items = vec![Sel::Body { name: "Block".into() }];
    let r = compute(&mut h.app, &form, &inputs);
    assert!(r.get("error").is_none(), "{r}");
    assert_eq!(r["coverage"], json!(1.0), "{r}");
    assert_eq!(r["bill_of_materials"]["Brick 2x4"], json!(1), "{r}");
    assert_eq!(h.app.session.world_state().bodies.len(), 3, "the kit is a body");
    // The system choosers reach the command: plates instead of bricks, the 3030 series, M10 rods.
    for f in &mut form.fields {
        match f.key {
            "brick" => f.choice = 1,
            "profile_series" => f.choice = 1,
            "rod" => f.choice = 2,
            _ => {}
        }
    }
    let p = form.params_from(&h.app.session, &inputs).expect("parameters");
    assert_eq!(p["brick"], json!("Plates (8 mm studs, 3.2 mm thin)"), "{p}");
    assert_eq!(p["profile_series"], json!("3030"), "{p}");
    assert_eq!(p["rod"], json!("M10"), "{p}");
    assert_eq!(p["bolt"], json!("M4"), "{p}");
    let r = compute(&mut h.app, &form, &inputs);
    assert!(r.get("error").is_none(), "{r}");
    assert_eq!(r["brick"], json!("plate"), "{r}");
    assert_eq!(r["bill_of_materials"]["Plate 2x4"], json!(3), "{r}");
}

#[test]
fn sketch_lines_become_frame_members_and_pipe_waypoints_and_faces_become_regions() {
    let mut h = Harness::new();
    h.app.home.open = false;
    h.app.run("sketch.create", json!({"plane": "XY"})).expect("a sketch");
    h.app.run("sketch.line", json!({"points": [[0, 0], [400, 0], [400, 300]]})).expect("two lines");
    h.app.run("sketch.line", json!({"points": [[0, 50], [50, 50]]})).expect("a loose line");
    h.app.run("sketch.finish", json!({})).expect("finished");
    let frame = CalcForm::for_command("design.frame").expect("a form");
    let mut inputs = frame.inputs();
    assert_eq!(inputs[0].label, "Sketch lines");
    inputs[0].items = vec![Sel::SketchCurve { id: "l1".into() }, Sel::SketchCurve { id: "l2".into() }];
    let p = frame.params_from(&h.app.session, &inputs).expect("parameters");
    assert_eq!(
        p["members"],
        json!([{"from": [0.0, 0.0, 0.0], "to": [400.0, 0.0, 0.0]}, {"from": [400.0, 0.0, 0.0], "to": [400.0, 300.0, 0.0]}]),
        "{p}"
    );
    // Left empty, the JSON members apply (four of them).
    assert_eq!(frame.params_from(&h.app.session, &frame.inputs()).expect("parameters")["members"].as_array().map(Vec::len), Some(4));
    let route = CalcForm::for_command("design.pipe_route").expect("a form");
    let mut inputs = route.inputs();
    inputs[0].items = vec![Sel::SketchCurve { id: "l2".into() }, Sel::SketchCurve { id: "l1".into() }];
    let p = route.params_from(&h.app.session, &inputs).expect("parameters");
    assert_eq!(p["points"], json!([[0.0, 0.0, 0.0], [400.0, 0.0, 0.0], [400.0, 300.0, 0.0]]), "{p}");
    inputs[0].items.push(Sel::SketchCurve { id: "l3".into() });
    assert!(route.params_from(&h.app.session, &inputs).unwrap_err().contains("end to end"));
    // Faces as regions: the load faces carry the typed force, and the picked body is the space.
    h.app.run("solid.box", json!({"length": 120, "width": 40, "height": 60, "corner": [0, 100, 0], "body_name": "Blank"})).expect("a blank");
    let study = CalcForm::for_command("generative.study").expect("a form");
    let mut inputs = study.inputs();
    assert_eq!(inputs.iter().map(|i| i.label).collect::<Vec<_>>(), ["Mount faces", "Load faces", "Design space body"]);
    inputs[0].items = vec![Sel::Face { body: "Blank".into(), index: 0, point: Vec3::new(0.0, 120.0, 30.0) }];
    inputs[1].items = vec![Sel::Face { body: "Blank".into(), index: 1, point: Vec3::new(120.0, 120.0, 30.0) }];
    inputs[2].items = vec![Sel::Body { name: "Blank".into() }];
    let p = study.params_from(&h.app.session, &inputs).expect("parameters");
    assert_eq!(p["mounts"], json!([{"face": 0, "point": [0.0, 120.0, 30.0], "body": "Blank", "name": "mount faces 1"}]), "{p}");
    assert_eq!(p["loads"][0]["force_n"], json!([0.0, 0.0, -1000.0]), "{p}");
    assert_eq!(p["design_space"], json!("Blank"));
    assert!(p.get("_force_n").is_none(), "the force field is not a parameter of its own");
}

#[test]
fn the_printed_stress_forms_judge_a_bar_twice_and_table_every_pattern() {
    let mut h = Harness::new();
    h.app.home.open = false;
    h.app.run("solid.box", json!({"length": 80, "width": 10, "height": 10, "body_name": "Bar"})).expect("a bar");
    let mut form = CalcForm::for_command("inspect.printed_stress").expect("a form");
    assert_eq!(form.inputs().iter().map(|i| i.label).collect::<Vec<_>>(), ["Body", "Fixed face", "Load face"]);
    assert!(!form.apply && form.button == "Compute");
    for f in &mut form.fields {
        if f.key == "resolution" {
            f.value = "8".into();
        }
    }
    let mut inputs = form.inputs();
    inputs[0].items = vec![Sel::Body { name: "Bar".into() }];
    assert!(form.params_from(&h.app.session, &inputs).unwrap_err().contains("pick the fixed face"));
    inputs[1].items = vec![Sel::Face { body: "Bar".into(), index: 0, point: Vec3::new(0.0, 5.0, 5.0) }];
    inputs[2].items = vec![Sel::Face { body: "Bar".into(), index: 1, point: Vec3::new(80.0, 5.0, 5.0) }];
    let p = form.params_from(&h.app.session, &inputs).expect("parameters");
    assert_eq!(p["material"], json!("PLA"));
    assert_eq!(p["pattern"], json!("Grid"));
    assert_eq!(p["up"], json!([0.0, 0.0, 1.0]));
    let r = compute(&mut h.app, &form, &inputs);
    assert!(r.get("error").is_none(), "{r}");
    assert_eq!(r["governs"], json!("within a layer"), "{r}");
    assert!(r["safety_interlayer"].as_f64().is_some_and(|x| x > 0.0), "{r}");
    assert_eq!(h.app.session.world_state().bodies.len(), 1, "a check changes nothing");

    let mut compare = CalcForm::for_command("inspect.compare_infill").expect("a form");
    for f in &mut compare.fields {
        if f.key == "resolution" {
            f.value = "6".into();
        }
    }
    assert!(compare.fields.iter().all(|f| f.key != "pattern"), "every pattern runs, so none is chosen");
    let r = compute(&mut h.app, &compare, &inputs);
    assert!(r.get("error").is_none(), "{r}");
    assert_eq!(r["patterns"].as_array().map(Vec::len), Some(11), "{r}");
    assert!(r["stiffest"].is_string() && r["verdict"].as_str().unwrap_or("").starts_with("same "), "{r}");
}
