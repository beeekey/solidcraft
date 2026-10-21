use serde_json::{Value, json};

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

/// A 100 × 10 × 10 mm bar along X.
fn bar() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 100, "width": 10, "height": 10, "body_name": "Bar"}));
    s
}

#[test]
fn a_pulled_bar_obeys_hookes_law() {
    let mut s = bar();
    let r = run(
        &mut s,
        "inspect.stress",
        json!({"fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [1000, 0, 0], "young_modulus_pa": 210e9, "poisson_ratio": 0.3, "resolution": 30}),
    );
    assert_eq!(r["body"], "Bar", "{r}");
    assert_eq!(r["converged"], true, "{r}");
    // δ = FL/EA = 1000 · 0.1 / (210e9 · 1e-4) = 4.76 µm; the clamped end reads a little stiff.
    let analytic_mm = 1000.0 * 0.1 / (210e9 * 1e-4) * 1000.0;
    let got = f(&r["load_displacement_mm"]);
    assert!((got - analytic_mm).abs() < 0.08 * analytic_mm, "{got} vs {analytic_mm}: {r}");
    // σ = F/A = 10 MPa.
    let peak = f(&r["max_von_mises_mpa"]);
    assert!((peak - 10.0).abs() < 1.5, "{peak}: {r}");
    assert!(r["verdict"].as_str().unwrap_or_default().starts_with("holds"), "{}", r["verdict"]);
    // Three elements through a 10 mm wall is too coarse for bending, and the answer says so.
    assert_eq!(r["warnings"].as_array().map(Vec::len), Some(1), "{r}");
    assert!(r["mesh_advice"].as_str().unwrap_or_default().contains("thinnest"), "{r}");
}

#[test]
fn the_material_comes_from_the_handbook_when_not_given() {
    let mut s = bar();
    // The default material is a generic plastic; the bar yields under 10 MPa only if the handbook
    // says so, and the answer names what it assumed.
    let r = run(&mut s, "inspect.stress", json!({"fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [0, 0, -50], "resolution": 20}));
    assert_eq!(r["material"], "Default", "{r}");
    assert_eq!(f(&r["young_modulus_pa"]), 2.5e9);
    assert!(f(&r["safety_factor"]).is_finite(), "{r}");
    assert!(r["material_note"].is_string());
    // Steel is far stiffer, so the same load moves it far less.
    run(&mut s, "material.assign", json!({"bodies": ["Bar"], "material": "Steel"}));
    let steel = run(&mut s, "inspect.stress", json!({"fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [0, 0, -50], "resolution": 20}));
    assert_eq!(steel["material"], "Steel", "{steel}");
    assert!(f(&steel["max_displacement_mm"]) < f(&r["max_displacement_mm"]) / 20.0, "{steel}");
    assert!(steel["verdict"].as_str().unwrap_or_default().starts_with("holds"), "{}", steel["verdict"]);
}

#[test]
fn a_cantilever_is_stressed_at_its_root() {
    let mut s = bar();
    let r = run(
        &mut s,
        "inspect.stress",
        json!({"body": "Bar", "fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [0, 0, -100], "young_modulus_pa": 210e9, "resolution": 30}),
    );
    // Euler–Bernoulli: 0.19 mm at the tip; trilinear hexes are a little stiff in bending.
    let tip = f(&r["load_displacement_mm"]);
    assert!(tip > 0.4 * 0.19 && tip < 1.3 * 0.19, "{tip}: {r}");
    assert!(f(&r["max_von_mises_at"][0]) < 20.0, "{}", r["max_von_mises_at"]);
}

#[test]
fn bad_input_is_refused_with_a_reason() {
    let mut s = bar();
    let mut err = |p: Value| s.execute("inspect.stress", &p).map(|_| ()).unwrap_err().to_string();
    assert!(err(json!({"load": [100, 5, 5], "force_n": [1, 0, 0]})).contains("fixed"));
    assert!(err(json!({"fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [0, 0, 0]})).contains("force_n"));
    assert!(err(json!({"fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [1, 0, 0], "resolution": 500})).contains("resolution"));
    assert!(err(json!({"fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [1, 0, 0], "young_modulus_pa": -3})).contains("young_modulus_pa"));
    assert!(err(json!({"body": "Nope", "fixed": [0, 5, 5], "load": [100, 5, 5], "force_n": [1, 0, 0]})).contains("Nope"));
    let mut empty = Session::default();
    assert!(empty.execute("inspect.stress", &json!({"fixed": [0, 0, 0], "load": [1, 0, 0], "force_n": [1, 0, 0]})).is_err());
}

#[test]
fn a_squashed_strut_reports_its_buckling_load() {
    // A 300 × 15 × 15 mm steel strut pushed along its axis with 1 kN: Euler's fixed-free column
    // folds at π² E I / (4 L²) ≈ 23 kN. At 60 elements along the mesh is 3 across, the same
    // column the solver's own Euler test uses, good to a third either way.
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 300, "width": 15, "height": 15, "body_name": "Strut"}));
    let r = run(
        &mut s,
        "inspect.stress",
        json!({"fixed": [0, 7.5, 7.5], "load": [300, 7.5, 7.5], "force_n": [-1000, 0, 0], "young_modulus_pa": 200e9, "resolution": 60, "buckling": true}),
    );
    let b = &r["buckling"];
    assert!(b.is_object(), "{r}");
    assert_eq!(r["grid"], json!([60, 3, 3]), "{r}");
    let inertia = 0.015_f64.powi(4) / 12.0;
    let critical = std::f64::consts::PI.powi(2) * 200e9 * inertia / (4.0 * 0.3 * 0.3);
    let expected = critical / 1000.0;
    let got = f(&b["load_factor"]);
    assert!((got - expected).abs() < 0.35 * expected, "{got} vs {expected}: {r}");
    assert!(f(&b["amplification"]) > 1.0, "{r}");
    // Without the option the key is absent, and a pulled strut has no mode.
    let plain = run(&mut s, "inspect.stress", json!({"fixed": [0, 7.5, 7.5], "load": [300, 7.5, 7.5], "force_n": [1000, 0, 0], "resolution": 20}));
    assert!(plain["buckling"].is_null(), "{plain}");
    let pulled = run(
        &mut s,
        "inspect.stress",
        json!({"fixed": [0, 7.5, 7.5], "load": [300, 7.5, 7.5], "force_n": [1000, 0, 0], "resolution": 20, "buckling": true}),
    );
    assert_eq!(f(&pulled["buckling"]["load_factor"]), 0.0, "{pulled}");
    assert!(pulled["buckling"]["advice"].as_str().unwrap_or_default().contains("compression"), "{pulled}");
}

#[test]
fn two_bodies_are_solved_together_bonded_or_in_contact() {
    // A bar held at its end with a second bar butted against it. Pushed, the pair in contact
    // answers like the bonded pair; pulled, the contact comes apart and the command says so.
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 20, "height": 20, "body_name": "Left"}));
    run(&mut s, "solid.box", json!({"length": 40, "width": 20, "height": 20, "corner": [40, 0, 0], "operation": "new", "body_name": "Right"}));
    let p = |force: [f64; 3], contact: Value| {
        json!({"bodies": ["Left", "Right"], "fixed": {"point": [0, 10, 10], "body": "Left"}, "load": {"point": [80, 10, 10], "body": "Right"},
               "force_n": force, "young_modulus_pa": 210e9, "resolution": 24, "contact": contact})
    };
    let bonded = run(&mut s, "inspect.stress", p([-1000.0, 0.0, 0.0], Value::Null));
    assert_eq!(bonded["bodies"], json!(["Left", "Right"]), "{bonded}");
    assert!(bonded["contact"].is_null());
    let touching = run(&mut s, "inspect.stress", p([-1000.0, 0.0, 0.0], json!(true)));
    assert!(touching["contact"]["links"].as_u64().unwrap_or(0) > 0, "{touching}");
    assert_eq!(touching["contact"]["open_links"], 0);
    let (a, b) = (f(&bonded["load_displacement_mm"]).abs(), f(&touching["load_displacement_mm"]).abs());
    assert!((a - b).abs() < 0.15 * a, "{a} vs {b}");
    let e = s.execute("inspect.stress", &p([1000.0, 0.0, 0.0], json!({"friction": 0.2}))).err().map(|e| e.to_string()).unwrap_or_default();
    assert!(e.contains("came apart"), "{e}");
    let e = s
        .execute("inspect.stress", &json!({"body": "Left", "fixed": [0, 10, 10], "load": [40, 10, 10], "force_n": [1, 0, 0], "contact": true}))
        .err()
        .map(|e| e.to_string())
        .unwrap_or_default();
    assert!(e.contains("needs `bodies`"), "{e}");
}
