use super::*;

#[test]
fn two_plates_get_the_screw_the_catalogue_lists() {
    // Two 8 mm plates with a 6.6 mm hole: ISO 273 medium for M6. ISO 7089 M6 is 1.6, ISO 4032
    // M6 is 5.2, pitch 1: 1.6 + 16 + 5.2 + 2 = 24.8 rounds up to 25.
    let (matched, choice) = choose_fastener(6.6, 16.0, true, true).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(matched.thread, "M6");
    assert_eq!(matched.series, "Medium");
    assert!((choice.needed_mm - 24.8).abs() < 1e-9);
    assert!((choice.length_mm - 25.0).abs() < 1e-9);
    assert!(choice.description.contains("M6 x 25"), "{}", choice.description);
}

#[test]
fn without_a_nut_the_screw_stops_at_the_far_face_and_says_so() {
    let (_, choice) = choose_fastener(6.6, 16.0, true, false).unwrap_or_else(|e| panic!("{e}"));
    assert!((choice.needed_mm - 17.6).abs() < 1e-9);
    assert!((choice.length_mm - 20.0).abs() < 1e-9);
    assert!(choice.description.contains("tapped"));
}

#[test]
fn the_round_trip_through_iso_273_closes() {
    for size in BOREHOLES {
        for (name, d) in [("Fine", size.clearance_fine), ("Medium", size.clearance_medium), ("Coarse", size.clearance_coarse)] {
            let m = match_clearance(d).unwrap_or_else(|e| panic!("{e}"));
            assert_eq!(m.thread, size.thread);
            assert_eq!(m.series, name);
        }
        assert!(tapping_drill(size) < size.diameter);
        assert!(size.clearance_fine < size.clearance_medium && size.clearance_medium < size.clearance_coarse);
        assert!(size.counterbore_diameter > size.socket_head_diameter);
    }
    assert_eq!(find_borehole("m6").map(|s| s.thread), Some("M6"));
    assert!(find_borehole("M7").is_none());
}

#[test]
fn a_hole_drilled_for_nothing_is_refused_by_name() {
    let refusal = match_clearance(7.3).err().unwrap_or_default();
    assert!(refusal.contains("no ISO 273 clearance"), "{refusal}");
    assert!(refusal.contains("nearest"), "{refusal}");
    assert!(match_clearance(f64::NAN).is_err());
    assert!(match_clearance(-1.0).is_err());
    // M16 has drill data but no washer in the catalogue.
    let refusal = choose_fastener(17.5, 20.0, true, true).err().unwrap_or_default();
    assert!(refusal.contains("catalogue for M16"), "{refusal}");
    assert!(choose_fastener(6.6, 0.0, true, true).is_err());
    assert!(choose_fastener(6.6, 500.0, true, true).err().unwrap_or_default().contains("past 200"));
}
