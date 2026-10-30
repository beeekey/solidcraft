use super::*;

fn class(text: &str) -> GeneralClass {
    GeneralClass::parse(text).unwrap_or_else(|| panic!("{text} is a class"))
}

#[test]
fn the_general_tolerance_table_is_the_one_iso_2768_prints() {
    for (nominal, cls, plus_minus) in [
        (25.0, "m", 0.2),
        (25.0, "f", 0.1),
        (25.0, "c", 0.5),
        (25.0, "v", 1.0),
        (2.0, "m", 0.1),
        (200.0, "c", 1.2),
        (500.0, "f", 0.3),
        (1500.0, "m", 1.2),
        (100.0, "m", 0.3),
        (350.0, "v", 2.5),
    ] {
        let found = general_linear(nominal, class(cls)).unwrap_or_else(|e| panic!("{nominal} {cls}: {e}"));
        assert!((found.plus_minus - plus_minus).abs() < 1e-9, "{nominal} mm in class {cls}: {}", found.plus_minus);
        assert!(found.source.contains("ISO 2768-1"));
    }
}

#[test]
fn a_chamfer_is_not_a_length_and_an_angle_is_neither() {
    let medium = class("ISO 2768-m");
    assert!((general_linear(1.0, medium).unwrap_or_else(|e| panic!("{e}")).plus_minus - 0.1).abs() < 1e-9);
    assert!((general_radius_or_chamfer(1.0, medium).unwrap_or_else(|e| panic!("{e}")).plus_minus - 0.2).abs() < 1e-9);
    let angle = general_angular(25.0, medium).unwrap_or_else(|e| panic!("{e}"));
    assert!((angle.plus_minus - 0.5).abs() < 1e-9);
    assert!(general_angular(500.0, medium).unwrap_or_else(|e| panic!("{e}")).plus_minus < angle.plus_minus);
}

#[test]
fn the_general_classes_run_in_order_at_every_size() {
    for mm in [1.0, 5.0, 25.0, 100.0, 300.0, 800.0, 1500.0] {
        let fine = general_linear(mm, GeneralClass::Fine).unwrap_or_else(|e| panic!("{e}")).plus_minus;
        let medium = general_linear(mm, GeneralClass::Medium).unwrap_or_else(|e| panic!("{e}")).plus_minus;
        let coarse = general_linear(mm, GeneralClass::Coarse).unwrap_or_else(|e| panic!("{e}")).plus_minus;
        assert!(fine < medium && medium < coarse, "{mm} mm");
    }
    let mut previous = 0.0;
    for mm in [1.0, 5.0, 25.0, 100.0, 300.0, 800.0, 1500.0] {
        let now = general_linear(mm, GeneralClass::Medium).unwrap_or_else(|e| panic!("{e}")).plus_minus;
        assert!(now >= previous, "{mm} mm");
        previous = now;
    }
}

#[test]
fn where_the_standard_prints_a_dash_it_refuses_rather_than_returning_zero() {
    assert!(GeneralClass::parse("ISO 2768-k").is_none(), "a class nobody recognises must not become the middle one");
    let dash = general_linear(2.0, GeneralClass::VeryCoarse).err().unwrap_or_default();
    assert!(dash.contains("dash") && dash.contains("individually"), "{dash}");
    let tiny = general_linear(0.4, GeneralClass::Medium).err().unwrap_or_default();
    assert!(tiny.contains("individually"), "{tiny}");
    let huge = general_linear(2500.0, GeneralClass::Medium).err().unwrap_or_default();
    assert!(huge.contains("2000.0"), "{huge}");
    assert!(general_linear(f64::NAN, GeneralClass::Medium).is_err());
}

#[test]
fn a_drawing_note_is_read_the_way_a_drawing_writes_it() {
    for note in ["m", "M", "ISO 2768-m", "ISO 2768-mK", "2768-m"] {
        assert_eq!(GeneralClass::parse(note), Some(GeneralClass::Medium), "{note}");
    }
    assert_eq!(GeneralClass::parse("ISO 2768-fH"), Some(GeneralClass::Fine));
    assert_eq!(GeneralClass::parse(""), None);
    assert_eq!(GeneralClass::parse("ISO 2768-"), None);
}
