//! Parametric Check (`inspect.parametric_check`): the test for modelled-against-drawn that a
//! design review does by hand. Every driving parameter is nudged down and up on a scratch copy
//! of the design, the model is rebuilt, and what happened is classified: the model FOLLOWS (some
//! body measures differently), the parameter is INERT (nothing downstream changed: it drives
//! nothing), or the design BROKE: a feature FAILS, a body EMPTIES (vanishes), a body DETACHES
//! into more shells, or shells MERGE. The design itself is never touched.

use serde_json::{Value, json};
use solidcraft_geom::Vec3;

use super::CommandSpec;
use crate::params::{bad, bool_};
use crate::{Result, Session};

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new("inspect.parametric_check", "Parametric Check", parametric_check)
    .at("3DEEE", "CALCULATE")
    .icon("parametric_check")
    .noundo()
    .params(
        "percent?: how far each parameter is nudged down and up (0.1 to 90, default 10); include_model?: true also sweeps sketch dimensions and \
         feature inputs that are plain numbers (default: user parameters only); max_drivers?: cap on the parameters swept (1 to 200, default 40) \
         → drivers: [{name, source, unit, baseline, down: {asked, verdict, note}, up: {…}, worst}], checked, survived (the model followed both \
         ways), inert (nothing downstream changed), broke (a feature failed, a body emptied, detached into more shells or merged), \
         preexisting_failures, verdict, advice, warnings. Every rebuild runs on a scratch copy: the design is left exactly as it was",
    )];

/// Worst first in order of badness; the number is the rank.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Verdict {
    Follows,
    Inert,
    Merges,
    Detaches,
    Empties,
    Fails,
}

impl Verdict {
    fn name(self) -> &'static str {
        match self {
            Verdict::Follows => "follows",
            Verdict::Inert => "inert",
            Verdict::Merges => "MERGES",
            Verdict::Detaches => "DETACHES",
            Verdict::Empties => "EMPTIES",
            Verdict::Fails => "FAILS",
        }
    }
}

/// What a body measures: enough to tell "changed" from "not". A hole moved along a plate keeps
/// the volume, the shell count and the box, so the centroid is in; a pair of holes spreading
/// about the centre keeps the centroid too, so the second moments are in.
#[derive(Clone, Debug, PartialEq)]
struct Measure {
    volume: f64,
    shells: usize,
    min: Vec3,
    max: Vec3,
    centroid: Vec3,
    inertia: [f64; 3],
}

fn measure_all(s: &Session) -> Vec<(String, Measure)> {
    let st = s.world_state();
    let mut out = Vec::with_capacity(st.bodies.len());
    for b in &st.bodies {
        let mesh = b.mesh();
        let (volume, shells, min, max, centroid) = match solidcraft_kernel::measure(&b.body) {
            Ok(m) => (m.volume, m.shells, m.bbox.min, m.bbox.max, m.centroid),
            Err(_) => {
                let mm = mesh.measure();
                let bb = mesh.bounds();
                (mm.volume, 1, bb.min, bb.max, mm.centroid)
            }
        };
        let inertia = mesh.mass_properties().map(|p| [p.inertia[0][0], p.inertia[1][1], p.inertia[2][2]]).unwrap_or([0.0; 3]);
        out.push((b.name.clone(), Measure { volume, shells, min, max, centroid, inertia }));
    }
    out.sort_by(|a, b| a.0.cmp(&b.0));
    out
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

fn same_vec(a: Vec3, b: Vec3) -> bool {
    near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z)
}

fn same(a: &Measure, b: &Measure) -> bool {
    a.shells == b.shells
        && near(a.volume, b.volume)
        && same_vec(a.min, b.min)
        && same_vec(a.max, b.max)
        && same_vec(a.centroid, b.centroid)
        && a.inertia.iter().zip(&b.inertia).all(|(x, y)| near(*x, *y))
}

/// Feature errors, by feature id.
fn failures(s: &Session) -> Vec<(u64, String, String)> {
    s.model.results.iter().filter_map(|r| r.error.as_ref().map(|e| (r.id, r.name.clone(), e.clone()))).collect()
}

/// A failure that means "built nothing" rather than "went wrong".
fn is_empty_failure(message: &str) -> bool {
    let m = message.to_ascii_lowercase();
    m.contains("nothing") || m.contains("no body") || m.contains("empty") || m.contains("no profile") || m.contains("vanish")
}

struct Driver {
    name: String,
    source: &'static str,
    unit: String,
    baseline: f64,
}

fn drivers_of(s: &Session, include_model: bool, cap: usize) -> (Vec<Driver>, usize, usize) {
    let rows = s.doc.param_rows();
    let (mut user, mut model) = (0usize, 0usize);
    let mut out = Vec::new();
    for r in rows {
        let Some(value) = r.value.filter(|v| v.is_finite()) else { continue };
        if r.error.is_some() {
            continue;
        }
        let driving = match r.source {
            "user" => {
                user += 1;
                true
            }
            _ => {
                // A dimension written as an expression of other parameters is driven, not
                // driving: writing a literal into it would report a lie.
                let plain = r.uses.is_empty();
                if plain {
                    model += 1;
                }
                include_model && plain
            }
        };
        if driving && out.len() < cap {
            out.push(Driver { name: r.name, source: r.source, unit: r.unit, baseline: value });
        }
    }
    (out, user, model)
}

struct Outcome {
    asked: f64,
    verdict: Verdict,
    note: String,
}

/// Nudges one driver to `asked` on a scratch copy and says what the model did.
fn sweep(s: &Session, driver: &Driver, asked: f64, baseline: &[(String, Measure)], baseline_failures: &[(u64, String, String)]) -> Outcome {
    let mut t = s.scratch();
    let expression = if driver.unit.is_empty() { format!("{asked}") } else { format!("{asked} {}", driver.unit) };
    if let Err(e) = t.execute("parameters.change", &json!({"name": driver.name, "expression": expression})) {
        return Outcome { asked, verdict: Verdict::Fails, note: format!("the change was refused: {e}") };
    }
    t.refresh();
    let after = failures(&t);
    let new_failures: Vec<&(u64, String, String)> = after.iter().filter(|(id, _, _)| !baseline_failures.iter().any(|(b, _, _)| b == id)).collect();
    if let Some((_, name, message)) = new_failures.first() {
        let empties = new_failures.iter().all(|(_, _, m)| is_empty_failure(m));
        let verdict = if empties { Verdict::Empties } else { Verdict::Fails };
        let note = if empties { format!("{name} built nothing: {message}") } else { format!("{name} failed: {message}") };
        return Outcome { asked, verdict, note };
    }
    let now = measure_all(&t);
    let shells = |m: &[(String, Measure)]| m.iter().map(|(_, x)| x.shells).sum::<usize>();
    if now.len() < baseline.len() {
        let gone: Vec<&str> = baseline.iter().filter(|(n, _)| !now.iter().any(|(m, _)| m == n)).map(|(n, _)| n.as_str()).collect();
        return Outcome { asked, verdict: Verdict::Empties, note: format!("a body vanished: {}", gone.join(", ")) };
    }
    let (s0, s1) = (shells(baseline), shells(&now));
    if now.len() > baseline.len() || s1 > s0 {
        return Outcome {
            asked,
            verdict: Verdict::Detaches,
            note: format!("the model came apart: {} body(ies) of {s1} shell(s), from {} of {s0}", now.len(), baseline.len()),
        };
    }
    if s1 < s0 && s0 > 1 {
        return Outcome { asked, verdict: Verdict::Merges, note: format!("separate pieces merged: {s0} shell(s) became {s1}") };
    }
    let changed = now.iter().any(|(name, m)| baseline.iter().find(|(n, _)| n == name).is_none_or(|(_, b)| !same(b, m)));
    if !changed {
        return Outcome { asked, verdict: Verdict::Inert, note: "nothing downstream measured any different".into() };
    }
    let volume = |m: &[(String, Measure)]| m.iter().map(|(_, x)| x.volume).sum::<f64>();
    let (v0, v1) = (volume(baseline), volume(&now));
    let mut note = format!("the model followed: volume {v1:.1} mm³ from {v0:.1}");
    if near(v0, v1) {
        let shift = now
            .iter()
            .filter_map(|(name, m)| baseline.iter().find(|(n, _)| n == name).map(|(_, b)| (m.centroid - b.centroid).len()))
            .fold(0.0_f64, f64::max);
        if shift > 1e-9 {
            note = format!("something moved without changing the volume: a centroid shifted {shift:.3} mm");
        } else {
            note = "something moved symmetrically, keeping the volume and the centroid: the second moments changed".into();
        }
    }
    Outcome { asked, verdict: Verdict::Follows, note }
}

fn parametric_check(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "inspect.parametric_check";
    let number = |key: &str, default: f64, range: std::ops::RangeInclusive<f64>| -> Result<f64> {
        match p.get(key) {
            None | Some(Value::Null) => Ok(default),
            Some(v) => v
                .as_f64()
                .filter(|x| x.is_finite() && range.contains(x))
                .ok_or_else(|| bad(cmd, format!("`{key}` must be a number from {} to {}", range.start(), range.end()))),
        }
    };
    let percent = number("percent", 10.0, 0.1..=90.0)?;
    let cap = number("max_drivers", 40.0, 1.0..=200.0)? as usize;
    let include_model = bool_(p, "include_model").unwrap_or(false);
    s.refresh();
    let (drivers, user_count, model_count) = drivers_of(s, include_model, cap);
    let baseline_failures = failures(s);
    let baseline = measure_all(s);
    let mut survived = 0usize;
    let mut inert = 0usize;
    let mut broke = 0usize;
    let mut rows = Vec::with_capacity(drivers.len());
    let outcome_json = |o: &Outcome| json!({"asked": o.asked, "verdict": o.verdict.name(), "note": o.note});
    for d in &drivers {
        let step = if d.baseline.abs() > 0.0 { d.baseline * percent / 100.0 } else { percent / 100.0 };
        let down = sweep(s, d, d.baseline - step, &baseline, &baseline_failures);
        let up = sweep(s, d, d.baseline + step, &baseline, &baseline_failures);
        let worst = down.verdict.max(up.verdict);
        match worst {
            Verdict::Follows => survived += 1,
            Verdict::Inert => inert += 1,
            _ => broke += 1,
        }
        rows.push(json!({
            "name": d.name,
            "source": d.source,
            "unit": d.unit,
            "baseline": d.baseline,
            "down": outcome_json(&down),
            "up": outcome_json(&up),
            "worst": worst.name(),
        }));
    }
    let mut warnings: Vec<String> = Vec::new();
    if !baseline_failures.is_empty() {
        warnings.push(format!("{} feature(s) already fail before any change", baseline_failures.len()));
    }
    let skipped = user_count + if include_model { model_count } else { 0 };
    if skipped > drivers.len() {
        warnings.push(format!("{} driver(s) beyond `max_drivers` were not swept", skipped - drivers.len()));
    }
    let mut advice: Vec<String> = Vec::new();
    if inert > 0 {
        advice.push("an inert parameter drives nothing: delete it or wire it into a dimension".into());
    }
    if broke > 0 {
        advice.push("a parameter that breaks the model at ±10 % is a drawn dimension masquerading as a parameter: model the relation (a hole positioned from the edge it belongs to, a wall as an offset) instead of a number that happens to fit".into());
    }
    if !include_model && model_count > 0 {
        advice.push(format!("{model_count} plain sketch and feature dimension(s) were not swept; `include_model: true` sweeps them too"));
    }
    let verdict = if drivers.is_empty() {
        "no parameters to check".to_string()
    } else if broke == 0 && inert == 0 {
        format!("all {} parameter(s) drive the model and it follows them ±{percent}%", drivers.len())
    } else {
        format!("{} of {} parameter(s) follow, {inert} inert, {broke} break the model", survived, drivers.len())
    };
    Ok(json!({
        "drivers": rows,
        "checked": drivers.len(),
        "survived": survived,
        "inert": inert,
        "broke": broke,
        "percent": percent,
        "preexisting_failures": baseline_failures.iter().map(|(_, n, e)| json!({"feature": n, "error": e})).collect::<Vec<_>>(),
        "verdict": verdict,
        "advice": advice.join(". "),
        "warnings": warnings,
    }))
}

#[cfg(test)]
#[path = "parametric_check_tests.rs"]
mod tests;
