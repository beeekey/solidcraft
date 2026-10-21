//! Will the whole chain go together, not just one pair.
//!
//! Add up everything between the two faces that have to meet, each with its own tolerance, and
//! see whether the gap that comes out is one you can live with. Two answers, because there are
//! two honest ways to add tolerances up: worst case assumes every part is at its worst end at
//! once, which is the number you can promise; RSS (root sum of squares) assumes the parts are
//! independent and normally distributed, which is the spread you will actually see. RSS is
//! smaller because it assumes something, and the advice says so every time.
//!
//! Lengths in millimetres.

use serde::{Deserialize, Serialize};

/// One length in the chain.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Contributor {
    pub name: String,
    pub nominal_mm: f64,
    /// Signed deviations (mm).
    pub lower_mm: f64,
    pub upper_mm: f64,
    /// +1 adds along the chain, −1 subtracts, 2 for a pair of the same part.
    pub sensitivity: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StackContribution {
    pub name: String,
    pub half_width_mm: f64,
    /// Fraction of the worst-case band.
    pub worst_case_share: f64,
    /// Fraction of the RSS variance.
    pub variance_share: f64,
}

/// What the gap has to be.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StackRequirement {
    pub minimum_mm: f64,
    pub maximum_mm: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StackResult {
    pub verdict: String,
    pub advice: String,
    /// The sum of the nominals.
    pub nominal_mm: f64,
    /// With asymmetric deviations folded in.
    pub mean_mm: f64,
    pub worst_case_min_mm: f64,
    pub worst_case_max_mm: f64,
    pub worst_case_half_width_mm: f64,
    pub rss_min_mm: f64,
    pub rss_max_mm: f64,
    pub rss_half_width_mm: f64,
    /// Largest variance share first: the one worth tightening.
    pub ranked: Vec<StackContribution>,
    pub worst_case_passes: Option<bool>,
    pub rss_passes: Option<bool>,
    /// How many standard deviations of the RSS spread fit inside the requirement, reading each
    /// ± as three sigma.
    pub sigma_at_limit: Option<f64>,
}

/// Most contributors one chain may hold.
pub const MAX_CHAIN: usize = 10_000;

pub fn stack_up(chain: &[Contributor], want: Option<StackRequirement>) -> Result<StackResult, String> {
    if chain.is_empty() {
        return Err("a stack-up needs a chain to add up, and none was given.".into());
    }
    if chain.len() > MAX_CHAIN {
        return Err(format!("a chain of at most {MAX_CHAIN} contributors; {} were given.", chain.len()));
    }
    if let Some(w) = want {
        if !w.minimum_mm.is_finite() || !w.maximum_mm.is_finite() {
            return Err("the requirement must be two numbers.".into());
        }
        if w.maximum_mm < w.minimum_mm {
            return Err(format!("the requirement is inverted: a maximum of {:.3} mm is below the minimum of {:.3} mm.", w.maximum_mm, w.minimum_mm));
        }
    }
    let (mut nominal, mut mean, mut worst_half, mut variance) = (0.0, 0.0, 0.0, 0.0);
    for item in chain {
        if ![item.nominal_mm, item.lower_mm, item.upper_mm, item.sensitivity].iter().all(|x| x.is_finite()) {
            return Err(format!("'{}' has a value that is not a number.", item.name));
        }
        if item.upper_mm < item.lower_mm {
            return Err(format!("'{}' has its upper deviation below its lower.", item.name));
        }
        let half = 0.5 * (item.upper_mm - item.lower_mm);
        let centre = item.nominal_mm + 0.5 * (item.upper_mm + item.lower_mm);
        nominal += item.sensitivity * item.nominal_mm;
        mean += item.sensitivity * centre;
        worst_half += item.sensitivity.abs() * half;
        let contribution = item.sensitivity * half;
        variance += contribution * contribution;
    }
    let rss_half = variance.sqrt();
    let mut ranked: Vec<StackContribution> = chain
        .iter()
        .map(|item| {
            let half_width = item.sensitivity.abs() * 0.5 * (item.upper_mm - item.lower_mm);
            StackContribution {
                name: item.name.clone(),
                half_width_mm: half_width,
                worst_case_share: if worst_half > 0.0 { half_width / worst_half } else { 0.0 },
                variance_share: if variance > 0.0 { half_width * half_width / variance } else { 0.0 },
            }
        })
        .collect();
    // Ranked by variance share, not worst-case share: halving a contributor's band takes three
    // quarters of its variance out, so that order says which tolerance is worth paying to tighten.
    ranked.sort_by(|a, b| b.variance_share.partial_cmp(&a.variance_share).unwrap_or(core::cmp::Ordering::Equal));

    let (wc_min, wc_max) = (mean - worst_half, mean + worst_half);
    let (rss_min, rss_max) = (mean - rss_half, mean + rss_half);
    let verdict = format!("{mean:.3} mm, worst case {wc_min:.3} to {wc_max:.3} mm, RSS {rss_min:.3} to {rss_max:.3} mm");
    let mut advice = String::new();
    if chain.len() == 1 {
        advice.push_str("one contributor, so RSS is the worst case and buys nothing. ");
    }
    advice.push_str(
        "Worst case assumes every part is at its worst end at once; RSS assumes they are independent and normally distributed, with each +/- read as three sigma. RSS is the smaller number BECAUSE it assumes that -- if the parts come from one machine, or one operator drifting, they are not independent and the worst case is the one to design to.",
    );
    let (mut worst_case_passes, mut rss_passes, mut sigma_at_limit) = (None, None, None);
    if let Some(w) = want {
        let wc = wc_min >= w.minimum_mm && wc_max <= w.maximum_mm;
        let rss = rss_min >= w.minimum_mm && rss_max <= w.maximum_mm;
        let room = (w.maximum_mm - mean).min(mean - w.minimum_mm);
        let sigma = rss_half / 3.0;
        let at_limit = if sigma > 1e-15 { room / sigma } else { 0.0 };
        advice.push_str(&format!(
            " Against {:.3} to {:.3} mm: {}, and RSS {} at {:.0} sigma.",
            w.minimum_mm,
            w.maximum_mm,
            if wc { "the worst case fits" } else { "THE WORST CASE DOES NOT FIT" },
            if rss { "fits" } else { "does not fit" },
            at_limit
        ));
        if !wc && rss {
            advice.push_str(" That gap is the assumption doing the work, and it is worth being sure of it before relying on it.");
        }
        worst_case_passes = Some(wc);
        rss_passes = Some(rss);
        sigma_at_limit = Some(at_limit);
    }
    if let Some(top) = ranked.first().filter(|t| t.variance_share > 0.0) {
        advice.push_str(&format!(
            " The largest share of the spread is '{}' at {:.0}% of the variance; that is the one worth tightening first.",
            top.name,
            top.variance_share * 100.0
        ));
    }
    Ok(StackResult {
        verdict,
        advice,
        nominal_mm: nominal,
        mean_mm: mean,
        worst_case_min_mm: wc_min,
        worst_case_max_mm: wc_max,
        worst_case_half_width_mm: worst_half,
        rss_min_mm: rss_min,
        rss_max_mm: rss_max,
        rss_half_width_mm: rss_half,
        ranked,
        worst_case_passes,
        rss_passes,
        sigma_at_limit,
    })
}

#[cfg(test)]
#[path = "stack_up_tests.rs"]
mod tests;
