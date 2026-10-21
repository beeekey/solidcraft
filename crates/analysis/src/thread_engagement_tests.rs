use super::{Material, Severity, check_engagement, engagement_factor};

#[test]
fn the_factors_are_the_skills_own() {
    assert_eq!(engagement_factor(Material::Steel), 1.5);
    assert_eq!(engagement_factor(Material::Aluminium), 2.0);
    assert_eq!(engagement_factor(Material::Plastic), 2.5);
}

#[test]
fn an_m6_through_an_eight_millimetre_plate_is_short_in_aluminium_and_in_steel() {
    let alu = check_engagement("M6", Material::Aluminium, 0.008, true).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(alu.severity, Severity::Warning);
    assert!((alu.needed - 2.0 * 0.006).abs() < 1e-12);
    assert!(alu.detail.contains("8.00 mm through"), "{}", alu.detail);
    assert!(alu.detail.contains("nut or a heat-set insert"), "{}", alu.detail);
    // Steel asks for 1.5 D = 9 mm; still short, by one millimetre.
    let steel = check_engagement("M6", Material::Steel, 0.008, true).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(steel.severity, Severity::Warning);
    assert!((steel.needed - 1.5 * 0.006).abs() < 1e-12);
}

#[test]
fn a_blind_m6_fifteen_deep_in_aluminium_is_clean_and_thirty_deep_is_a_note() {
    let clean = check_engagement("M6", Material::Aluminium, 0.015, false).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(clean.severity, Severity::Ok, "{}", clean.detail);
    let deep = check_engagement("M6", Material::Aluminium, 0.030, false).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(deep.severity, Severity::Note);
    assert!(deep.detail.contains("tap breakage"), "{}", deep.detail);
    // Through the part, extra depth is not a choice, so it is not a note.
    let through = check_engagement("M6", Material::Aluminium, 0.030, true).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(through.severity, Severity::Ok);
}

#[test]
fn nonsense_is_refused() {
    assert!(check_engagement("6mm", Material::Steel, 0.01, false).is_err());
    assert!(check_engagement("M6", Material::Steel, f64::NAN, false).is_err());
    assert!(check_engagement("M6", Material::Steel, -1.0, false).is_err());
    assert_eq!(Material::parse("Aluminum"), Some(Material::Aluminium));
    assert_eq!(Material::parse("wood"), None);
}
