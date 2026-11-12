//! Checked twice over: against values ISO 286 prints, and against the standard's own formula
//! IT = k·i with i = 0.45·∛D + 0.001·D, which catches a mistyped digit no reading would.

use super::*;

fn microns(nominal_mm: f64, class: &str) -> (f64, f64) {
    let found = fit_limits(nominal_mm, class).unwrap_or_else(|e| panic!("{class} at {nominal_mm} mm: {e}"));
    (found.limits.lower_mm * 1e3, found.limits.upper_mm * 1e3)
}

#[test]
fn the_deviations_are_the_ones_iso_286_prints() {
    for (nominal, class, lower, upper) in [
        (25.0, "H7", 0.0, 21.0),
        (25.0, "g6", -20.0, -7.0),
        (25.0, "h6", -13.0, 0.0),
        (25.0, "k6", 2.0, 15.0),
        (25.0, "n6", 15.0, 28.0),
        (25.0, "p6", 22.0, 35.0),
        (25.0, "s6", 35.0, 48.0),
        (50.0, "H7", 0.0, 25.0),
        (50.0, "g6", -25.0, -9.0),
        (10.0, "h6", -9.0, 0.0),
        (6.0, "H7", 0.0, 12.0),
        (100.0, "s6", 71.0, 93.0),
        // A step boundary: 30 is the last size of "over 18 up to 30", 31 the first of the next.
        (30.0, "H7", 0.0, 21.0),
        (31.0, "H7", 0.0, 25.0),
    ] {
        let (l, u) = microns(nominal, class);
        assert!((l - lower).abs() < 1e-9, "{class} at {nominal}: lower {l} vs {lower}");
        assert!((u - upper).abs() < 1e-9, "{class} at {nominal}: upper {u} vs {upper}");
    }
}

#[test]
fn the_hole_special_rule_adds_the_right_delta() {
    // K, M, N at IT8 and finer and P onward at IT7 and finer take delta = IT(n) − IT(n−1): at
    // 25 mm, IT7 − IT6 = 21 − 13 = 8.
    for (class, lower, upper) in [("K7", -15.0, 6.0), ("N7", -28.0, -7.0), ("P7", -35.0, -14.0)] {
        let (l, u) = microns(25.0, class);
        assert!((l - lower).abs() < 1e-9 && (u - upper).abs() < 1e-9, "{class}: {l} {u}");
    }
}

#[test]
fn the_table_agrees_with_the_standards_own_formula() {
    let factors = [7.0, 10.0, 16.0, 25.0, 40.0, 64.0, 100.0, 160.0, 250.0]; // IT5..IT13
    for row in grade_rows() {
        if row.upto_mm <= 3.0 {
            continue; // below 3 mm the standard's values are not from this formula
        }
        let d = (row.over_mm * row.upto_mm).sqrt();
        let i = 0.45 * d.cbrt() + 0.001 * d;
        for grade in 5..=13 {
            let formula = factors[grade - 5] * i;
            let tabulated = row.it_um[grade - 1];
            assert!(
                (tabulated - formula).abs() < (0.06 * formula).max(1.0),
                "IT{grade} over {} to {}: table {tabulated}, formula {formula}",
                row.over_mm,
                row.upto_mm
            );
        }
    }
}

#[test]
fn every_row_holds_together_whatever_the_numbers_are() {
    let rows = grade_rows();
    for row in &rows {
        for grade in 2..=18 {
            assert!(row.it_um[grade - 1] > row.it_um[grade - 2], "IT{grade} must be looser than IT{}", grade - 1);
        }
    }
    for grade in 1..=18 {
        let mut previous = 0.0;
        for row in &rows {
            assert!(row.it_um[grade - 1] >= previous, "IT{grade} went backwards");
            previous = row.it_um[grade - 1];
        }
        assert!(rows[rows.len() - 1].it_um[grade - 1] > rows[0].it_um[grade - 1]);
    }
    for row in &rows {
        let at = 0.5 * (row.over_mm + row.upto_mm);
        let w = |c: &str| fit_limits(at, c).unwrap_or_else(|e| panic!("{c}: {e}")).limits;
        assert!(w("H7").lower_mm.abs() < 1e-12);
        assert!(w("h7").upper_mm.abs() < 1e-12);
        let js = w("js7");
        assert!((js.upper_mm + js.lower_mm).abs() < 1e-12);
        assert!((js.width() - standard_tolerance(at, 7).unwrap_or(0.0)).abs() < 1e-12);
        assert!(w("H11").width() > w("H9").width());
        assert!(w("H9").width() > w("H7").width());
        assert!(w("H7").width() > w("H6").width());
        assert!(w("d9").upper_mm < w("e9").upper_mm);
        assert!(w("e9").upper_mm < w("f9").upper_mm);
        assert!(w("f9").upper_mm < w("g9").upper_mm);
        assert!(w("g9").upper_mm < w("h9").upper_mm);
        assert!(w("k6").lower_mm < w("n6").lower_mm);
        assert!(w("n6").lower_mm < w("p6").lower_mm);
        assert!(w("p6").lower_mm < w("s6").lower_mm);
    }
}

#[test]
fn the_classic_fits_come_out_as_their_names_say() {
    let clearances = |shaft: &str| {
        let hole = fit_limits(25.0, "H7").unwrap_or_else(|e| panic!("{e}")).limits;
        let pin = fit_limits(25.0, shaft).unwrap_or_else(|e| panic!("{e}")).limits;
        (hole.min_size() - pin.max_size(), hole.max_size() - pin.min_size())
    };
    let (g6min, g6max) = clearances("g6");
    assert!((g6min * 1e3 - 7.0).abs() < 1e-6, "a close running fit never binds");
    assert!((g6max * 1e3 - 41.0).abs() < 1e-6);
    let (h6min, h6max) = clearances("h6");
    assert!(h6min.abs() < 1e-9 && (h6max * 1e3 - 34.0).abs() < 1e-6);
    let (p6min, p6max) = clearances("p6");
    assert!(p6max < 0.0 && (p6min * 1e3 + 35.0).abs() < 1e-6);
    let (k6min, k6max) = clearances("k6");
    assert!(k6min < 0.0 && k6max > 0.0, "H7/k6 straddles");
}

#[test]
fn what_it_cannot_stand_behind_it_refuses_by_name() {
    for (nominal, class, mentions) in [
        (25.0, "H20", "IT1 to IT18"),
        (25.0, "Q7", "Q"),
        (25.0, "H", "letter and a grade"),
        (620.0, "H7", "0 to 500 mm"),
        (0.0, "H7", "0 to 500 mm"),
        (25.0, "j6", "irregular"),
        (25.0, "N9", "above IT8"),
    ] {
        let refusal = fit_limits(nominal, class).err().unwrap_or_else(|| panic!("{class} at {nominal} must be refused"));
        assert!(refusal.contains(mentions), "{class}: {refusal}");
    }
    assert!(fit_limits(f64::NAN, "H7").is_err());
    assert!(parse_fit_class("H7777").is_none());
    assert!(parse_fit_class("ABCD7").is_none());
}

#[test]
fn a_source_is_given_for_every_answer() {
    for class in ["H7", "g6", "js9", "K7", "s6"] {
        assert!(fit_limits(25.0, class).unwrap_or_else(|e| panic!("{e}")).source.contains("ISO 286"));
    }
    assert!(known_fit_classes().contains(&"H7".to_string()));
}

fn press(shaft: &str, hub_outer: f64, shaft_bore: f64) -> FitResult {
    check_fit(&FitDuty {
        nominal_mm: 25.0,
        hole_class: "H7".into(),
        shaft_class: shaft.into(),
        engagement_length_mm: 20.0,
        hub_outer_diameter_mm: hub_outer,
        shaft_bore_diameter_mm: shaft_bore,
        ..Default::default()
    })
    .unwrap_or_else(|e| panic!("{e}"))
}

#[test]
fn a_press_fit_is_two_different_fits_and_both_are_reported() {
    // A 25 mm H7/s6 press in steel, hub 50 mm across, 20 mm of engagement. H7 is 0/+21, s6 is
    // +35/+48: interference runs from 14 µm at the loosest to 48 µm at the tightest.
    let fit = press("s6", 50.0, 0.0);
    assert_eq!(fit.kind, FitKind::Interference);
    assert!((fit.max_interference_mm * 1e3 - 48.0).abs() < 1e-6);
    assert!((fit.min_interference_mm * 1e3 - 14.0).abs() < 1e-6);
    assert!(fit.max_retained_torque_nm > fit.min_retained_torque_nm * 3.0);
    // Lamé worked by hand: p = 151.2 MPa, F = 35.63 kN, T = 445.3 N·m, hoop = 252.0 MPa.
    assert!((fit.contact_pressure_pa * 1e-6 - 151.2).abs() < 0.1, "{}", fit.contact_pressure_pa);
    assert!((fit.assembly_force_n * 1e-3 - 35.63).abs() < 0.01, "{}", fit.assembly_force_n);
    assert!((fit.max_retained_torque_nm - 445.3).abs() < 0.1);
    assert!((fit.hub_hoop_stress_pa * 1e-6 - 252.0).abs() < 0.1);
    // A textbook H7/s6 on a two-to-one hub in ordinary steel is at yield: 252 MPa against 250.
    assert!(fit.hub_safety < 1.0);
    assert!(fit.advice.contains("HUB YIELDS"), "{}", fit.advice);
    assert!((fit.max_retained_torque_nm - fit.assembly_force_n * 0.025 / 2.0).abs() < 1e-9);
    assert!(fit.hub_hoop_stress_pa > fit.contact_pressure_pa);
    assert!(fit.shrink_temperature_rise_k > 0.0);
}

#[test]
fn the_pressure_moves_the_way_physics_says() {
    assert!(press("s6", 50.0, 0.0).contact_pressure_pa > press("p6", 50.0, 0.0).contact_pressure_pa);
    assert!(press("s6", 30.0, 0.0).contact_pressure_pa < press("s6", 50.0, 0.0).contact_pressure_pa);
    assert!(press("s6", 50.0, 15.0).contact_pressure_pa < press("s6", 50.0, 0.0).contact_pressure_pa);
    let twice = check_fit(&FitDuty {
        nominal_mm: 25.0,
        hole_class: "H7".into(),
        shaft_class: "s6".into(),
        hub_outer_diameter_mm: 50.0,
        engagement_length_mm: 40.0,
        ..Default::default()
    })
    .unwrap_or_else(|e| panic!("{e}"));
    let once = press("s6", 50.0, 0.0);
    assert!((twice.contact_pressure_pa - once.contact_pressure_pa).abs() < 1e-6);
    assert!((twice.assembly_force_n - once.assembly_force_n * 2.0).abs() < 1e-6);
}

#[test]
fn a_clearance_fit_is_told_it_does_not_grip() {
    let fit = press("g6", 50.0, 0.0);
    assert_eq!(fit.kind, FitKind::Clearance);
    assert_eq!(fit.assembly_force_n, 0.0);
    assert_eq!(fit.min_retained_torque_nm, 0.0);
    assert!(fit.advice.contains("does not grip"), "{}", fit.advice);
}

#[test]
fn what_the_pressure_needs_and_did_not_get_is_named() {
    let fit =
        check_fit(&FitDuty { nominal_mm: 25.0, hole_class: "H7".into(), shaft_class: "s6".into(), engagement_length_mm: 20.0, ..Default::default() })
            .unwrap_or_else(|e| panic!("{e}"));
    assert!((fit.max_interference_mm * 1e3 - 48.0).abs() < 1e-6);
    assert_eq!(fit.assembly_force_n, 0.0);
    assert!(fit.advice.contains("outer diameter"), "{}", fit.advice);
    let refused = check_fit(&FitDuty::default()).err().unwrap_or_default();
    assert!(refused.contains("classes"), "{refused}");
    let unknown = check_fit(&FitDuty {
        hub_material: "Unobtainium".into(),
        ..FitDuty {
            nominal_mm: 25.0,
            hole_class: "H7".into(),
            shaft_class: "s6".into(),
            engagement_length_mm: 20.0,
            hub_outer_diameter_mm: 50.0,
            ..Default::default()
        }
    })
    .unwrap_or_else(|e| panic!("{e}"));
    assert!(unknown.advice.contains("Unobtainium"));
}

#[test]
fn a_rolling_bearing_wants_the_interference_on_the_ring_that_turns() {
    let on_shaft = check_fit(&FitDuty { nominal_mm: 25.0, hole_class: "H7".into(), shaft_class: "k6".into(), ..Default::default() })
        .unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(on_shaft.kind, FitKind::Transition);
    let in_housing = check_fit(&FitDuty { nominal_mm: 52.0, hole_class: "H7".into(), shaft_class: "h6".into(), ..Default::default() })
        .unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(in_housing.kind, FitKind::Clearance);
}

#[test]
fn measured_sizes_work_without_classes() {
    let fit = check_fit(&FitDuty { hole_min_mm: 25.0, hole_max_mm: 25.021, shaft_min_mm: 24.98, shaft_max_mm: 24.993, ..Default::default() })
        .unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(fit.kind, FitKind::Clearance);
    assert!((fit.min_clearance_mm - 0.007).abs() < 1e-9);
}
