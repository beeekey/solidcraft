use super::*;

fn part(name: &str, nominal_mm: f64, plus_minus_mm: f64, sensitivity: f64) -> Contributor {
    Contributor { name: name.into(), nominal_mm, lower_mm: -plus_minus_mm, upper_mm: plus_minus_mm, sensitivity }
}

#[test]
fn worst_case_and_rss_are_both_exact_on_a_chain_of_equals() {
    // n contributors of half-width t: worst case n·t, RSS t·√n, exactly.
    for n in 1..=8 {
        let chain: Vec<Contributor> = (0..n).map(|_| part("p", 10.0, 0.1, 1.0)).collect();
        let stack = stack_up(&chain, None).unwrap_or_else(|e| panic!("{e}"));
        assert!((stack.worst_case_half_width_mm - 0.1 * n as f64).abs() < 1e-12, "n = {n}");
        assert!((stack.rss_half_width_mm - 0.1 * (n as f64).sqrt()).abs() < 1e-12, "n = {n}");
        assert!((stack.nominal_mm - 10.0 * n as f64).abs() < 1e-9);
    }
}

#[test]
fn worst_case_always_bounds_rss_and_they_meet_at_one() {
    let mut state: u32 = 12345;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        state
    };
    for trial in 0..400 {
        let n = 1 + (next() % 8) as usize;
        let chain: Vec<Contributor> = (0..n)
            .map(|_| {
                part("p", 1.0 + f64::from(next() % 1000) / 10.0, 0.01 + f64::from(next() % 500) / 1000.0, if next() % 2 == 1 { 1.0 } else { -1.0 })
            })
            .collect();
        let stack = stack_up(&chain, None).unwrap_or_else(|e| panic!("{e}"));
        assert!(stack.worst_case_half_width_mm >= stack.rss_half_width_mm - 1e-15, "trial {trial}");
        if n == 1 {
            assert!((stack.worst_case_half_width_mm - stack.rss_half_width_mm).abs() < 1e-15);
        }
        let shares: f64 = stack.ranked.iter().map(|r| r.variance_share).sum();
        assert!((shares - 1.0).abs() < 1e-9);
        for w in stack.ranked.windows(2) {
            assert!(w[0].variance_share >= w[1].variance_share);
        }
    }
}

#[test]
fn the_ranking_points_at_the_tolerance_worth_buying() {
    let stack = stack_up(&[part("plate", 10.0, 0.05, 1.0), part("spacer", 5.0, 0.05, 1.0), part("casting", 40.0, 0.40, 1.0)], None)
        .unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(stack.ranked.len(), 3);
    assert_eq!(stack.ranked[0].name, "casting");
    assert!((stack.ranked[0].worst_case_share - 0.80).abs() < 0.01);
    assert!(stack.ranked[0].variance_share > 0.95);
    assert!(stack.advice.contains("casting"), "{}", stack.advice);
}

#[test]
fn a_subtracted_length_comes_off_the_nominal_and_adds_to_the_spread() {
    let stack = stack_up(&[part("housing", 50.0, 0.10, 1.0), part("shaft", 30.0, 0.05, -1.0), part("bearing", 12.0, 0.02, -1.0)], None)
        .unwrap_or_else(|e| panic!("{e}"));
    assert!((stack.nominal_mm - 8.0).abs() < 1e-9, "50 − 30 − 12");
    assert!((stack.worst_case_half_width_mm - 0.17).abs() < 1e-12);
}

#[test]
fn a_gap_that_only_fits_on_the_assumption_says_so() {
    let want = StackRequirement { minimum_mm: 0.2, maximum_mm: 1.8 };
    let stack =
        stack_up(&[part("a", 20.0, 0.30, 1.0), part("b", 20.0, 0.30, 1.0), part("c", 20.0, 0.30, 1.0), part("d", 59.0, 0.30, -1.0)], Some(want))
            .unwrap_or_else(|e| panic!("{e}"));
    // Mean 1.0, worst case ±1.2 (does not fit 0.2..1.8), RSS ±0.6 (fits).
    assert_eq!(stack.worst_case_passes, Some(false));
    assert_eq!(stack.rss_passes, Some(true));
    assert!(stack.advice.contains("assumption doing the work"), "{}", stack.advice);
    assert!(stack.sigma_at_limit.unwrap_or(0.0) > 3.0);
}

#[test]
fn nonsense_is_refused() {
    assert!(stack_up(&[], None).is_err());
    assert!(stack_up(&[part("a", 1.0, 0.1, 1.0)], Some(StackRequirement { minimum_mm: 2.0, maximum_mm: 1.0 })).is_err());
    let mut inverted = part("a", 1.0, 0.1, 1.0);
    inverted.upper_mm = -0.2;
    assert!(stack_up(&[inverted], None).is_err());
    assert!(stack_up(&[part("a", f64::NAN, 0.1, 1.0)], None).is_err());
}
