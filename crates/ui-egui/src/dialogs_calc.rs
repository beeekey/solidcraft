//! The form dialog: one form for every "numbers in" command that has no dialog of its own. The
//! calculators (bolted joint, thread engagement, bearing life, damping, fits, general tolerances,
//! tolerance stack, hole stack, bamboo joint, parametric check) compute and show their reply as
//! rows: figures, then the verdict, the advice and the warnings, and change nothing in the design.
//! The tools of the 3DEEE tab (`dialogs_tools`) use the same form: the generators apply on OK,
//! the builders (lattice, brick kit, generative design) run from a button and report what they
//! built, the checks compute. Fields come from a table per command; picks (a body, a face, an
//! edge) are the dialog's selection inputs and land in the parameters as names and points.

use egui::RichText;
use serde_json::{Map, Value, json};

use solidcraft_engine::geom::Vec3;
use solidcraft_engine::{Sel, Session};

use crate::SolveApp;
use crate::dialogs::{check, combo, field, form_row};
use crate::selection::{Accept, SelInput};
use crate::theme::Tokens;

/// How a field is edited.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FieldKind {
    /// A short text (a size, a class, a designation).
    Text,
    /// A number; empty leaves the command's default.
    Number,
    Bool,
    /// One of a fixed list; an empty label means "not given".
    Choice(&'static [&'static str]),
    /// A tolerance chain, one contributor per line.
    Chain,
    /// A JSON value (a list of sections, members, waypoints, regions…), edited as text.
    Json,
    /// Three numbers `x, y, z`.
    Point,
}

/// What a selection input's picks become in the parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PickShape {
    /// A body's name (the body of a picked face, edge or vertex counts).
    Body,
    /// The names of every body picked.
    Bodies,
    /// The point on the face, edge or vertex picked.
    Point,
    /// Sketch lines as members `[{from, to}]` (a 3D sketch's lines too).
    Lines,
    /// Sketch lines joined end to end as waypoints `[[x, y, z]…]`.
    Waypoints,
    /// Faces as regions `[{face, point, body}]`; with `force`, each carries the force of that
    /// (hidden) field as `force_n`.
    Regions { force: Option<&'static str> },
}

/// A selection input whose picks become a parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct Pick {
    pub key: &'static str,
    pub label: &'static str,
    pub accept: Accept,
    pub multi: bool,
    pub required: bool,
    pub shape: PickShape,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CalcField {
    pub key: &'static str,
    pub label: &'static str,
    pub kind: FieldKind,
    /// Text, Number and Chain fields.
    pub value: String,
    pub on: bool,
    pub choice: usize,
}

impl CalcField {
    pub(crate) fn text(key: &'static str, label: &'static str, value: &str) -> Self {
        CalcField { key, label, kind: FieldKind::Text, value: value.into(), on: false, choice: 0 }
    }
    pub(crate) fn number(key: &'static str, label: &'static str, value: &str) -> Self {
        CalcField { key, label, kind: FieldKind::Number, value: value.into(), on: false, choice: 0 }
    }
    pub(crate) fn flag(key: &'static str, label: &'static str, on: bool) -> Self {
        CalcField { key, label, kind: FieldKind::Bool, value: String::new(), on, choice: 0 }
    }
    pub(crate) fn choice(key: &'static str, label: &'static str, options: &'static [&'static str], choice: usize) -> Self {
        CalcField { key, label, kind: FieldKind::Choice(options), value: String::new(), on: false, choice }
    }
    fn chain(key: &'static str, label: &'static str, value: &str) -> Self {
        CalcField { key, label, kind: FieldKind::Chain, value: value.into(), on: false, choice: 0 }
    }
    pub(crate) fn json(key: &'static str, label: &'static str, value: &str) -> Self {
        CalcField { key, label, kind: FieldKind::Json, value: value.into(), on: false, choice: 0 }
    }
    pub(crate) fn point(key: &'static str, label: &'static str, value: &str) -> Self {
        CalcField { key, label, kind: FieldKind::Point, value: value.into(), on: false, choice: 0 }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CalcForm {
    pub command: &'static str,
    pub title: &'static str,
    /// One line under the fields saying what the calculator does.
    pub about: &'static str,
    pub fields: Vec<CalcField>,
    pub result: Option<Value>,
    /// The field values the result was computed from.
    pub of: Option<Vec<String>>,
    /// Selection inputs, in the dialog's order.
    pub picks: Vec<Pick>,
    /// OK runs the command and closes (a generator); otherwise a button runs it and the reply
    /// stays in the dialog.
    pub apply: bool,
    /// The button's label when the form does not apply: "Compute", or "Build" / "Run" for a
    /// builder whose reply is worth reading.
    pub button: &'static str,
}

const GRADES: &[&str] = &["4.6", "5.8", "8.8", "10.9", "12.9", "A2-70"];
const THREAD_MATERIALS: &[&str] = &["steel", "aluminium", "plastic"];
const DAMPING_MATERIALS: &[&str] = &["", "Aluminium", "Steel", "Titanium", "Carbon fibre", "Bamboo", "Wood (ash, hickory)", "Rubber (natural)"];
const FIT_MATERIALS: &[&str] = &["Steel", "Stainless Steel", "Aluminum", "Brass", "Bronze", "Copper", "Titanium", "Cast Iron"];
const TOLERANCE_KINDS: &[&str] = &["linear", "radius", "angle"];

/// The ten calculators, by command id.
pub const CALCULATORS: &[&str] = &[
    "inspect.bolted_joint",
    "inspect.thread_engagement",
    "inspect.bearing_life",
    "inspect.damping",
    "inspect.fit",
    "inspect.general_tolerance",
    "inspect.tolerance_stack",
    "inspect.hole_stack",
    "inspect.bamboo_joint",
    "inspect.parametric_check",
];

impl CalcForm {
    /// The form for a calculator command, `None` for any other command.
    pub fn for_command(id: &str) -> Option<CalcForm> {
        let (command, title, about, fields): (&'static str, &'static str, &'static str, Vec<CalcField>) = match id {
            "inspect.bolted_joint" => (
                "inspect.bolted_joint",
                "BOLTED JOINT",
                "two springs in parallel: the bolt over its grip, the clamped parts under the head",
                vec![
                    CalcField::text("size", "Size", "M8"),
                    CalcField::choice("grade", "Grade", GRADES, 2),
                    CalcField::number("clamped_mm", "Clamped length (mm)", "30"),
                    CalcField::number("preload_n", "Preload (N)", ""),
                    CalcField::number("external_n", "External load (N)", ""),
                    CalcField::number("shear_n", "Shear load (N)", ""),
                    CalcField::number("friction", "Face friction", "0.15"),
                    CalcField::number("nut_factor", "Nut factor K", "0.20"),
                    CalcField::number("cycles", "Load cycles", ""),
                ],
            ),
            "inspect.thread_engagement" => (
                "inspect.thread_engagement",
                "THREAD ENGAGEMENT",
                "how many diameters of thread a screw needs in this material",
                vec![
                    CalcField::text("size", "Size", "M6"),
                    CalcField::choice("material", "Material", THREAD_MATERIALS, 0),
                    CalcField::number("engagement_mm", "Engagement (mm)", "9"),
                    CalcField::flag("through", "Through hole", false),
                ],
            ),
            "inspect.bearing_life" => (
                "inspect.bearing_life",
                "BEARING LIFE",
                "ISO 281 L10 life from the table or from C and C0",
                vec![
                    CalcField::text("designation", "Designation", "6001"),
                    CalcField::number("dynamic_n", "Dynamic rating C (N)", ""),
                    CalcField::number("static_n", "Static rating C0 (N)", ""),
                    CalcField::number("radial_n", "Radial load (N)", "500"),
                    CalcField::number("axial_n", "Axial load (N)", ""),
                    CalcField::number("rpm", "Speed (rpm)", "200"),
                    CalcField::number("wheel_mm", "Wheel diameter (mm)", ""),
                    CalcField::flag("roller", "Roller bearing", false),
                ],
            ),
            "inspect.damping" => (
                "inspect.damping",
                "DAMPING AND ISOLATION",
                "a mass on a spring: natural frequency, damping and what reaches the load",
                vec![
                    CalcField::number("mass_kg", "Mass (kg)", "25"),
                    CalcField::number("rate_n_mm", "Spring rate (N/mm)", "12"),
                    CalcField::number("damping_ns_m", "Damper (N·s/m)", ""),
                    CalcField::number("loss_factor", "Loss factor", ""),
                    CalcField::choice("material", "Material", DAMPING_MATERIALS, 0),
                    CalcField::number("excitation_hz", "Excitation (Hz)", "10"),
                ],
            ),
            "inspect.fit" => (
                "inspect.fit",
                "FIT",
                "ISO 286 limits, the clearance or interference, and what a press fit holds",
                vec![
                    CalcField::number("nominal_mm", "Nominal (mm)", "20"),
                    CalcField::text("hole", "Hole class", "H7"),
                    CalcField::text("shaft", "Shaft class", "g6"),
                    CalcField::number("engagement_length_mm", "Engagement length (mm)", ""),
                    CalcField::number("hub_outer_diameter_mm", "Hub outer diameter (mm)", ""),
                    CalcField::number("shaft_bore_diameter_mm", "Shaft bore (mm)", ""),
                    CalcField::choice("hub_material", "Hub material", FIT_MATERIALS, 0),
                    CalcField::choice("shaft_material", "Shaft material", FIT_MATERIALS, 0),
                    CalcField::number("friction", "Friction", "0.15"),
                    CalcField::number("smoothing_mm", "Smoothing (mm)", ""),
                ],
            ),
            "inspect.general_tolerance" => (
                "inspect.general_tolerance",
                "GENERAL TOLERANCE",
                "what an untoleranced dimension is allowed under ISO 2768",
                vec![
                    CalcField::text("class", "Class", "m"),
                    CalcField::number("nominal_mm", "Nominal (mm)", "50"),
                    CalcField::choice("kind", "Kind", TOLERANCE_KINDS, 0),
                ],
            ),
            "inspect.tolerance_stack" => (
                "inspect.tolerance_stack",
                "TOLERANCE STACK",
                "one contributor per line: name nominal ±tol, or name nominal lower upper; -1 at the end subtracts",
                vec![
                    CalcField::chain("chain", "Chain", "Plate 10 ±0.1\nSpacer 5 ±0.05\nPocket 15.5 ±0.1 -1"),
                    CalcField::number("minimum_mm", "Gap at least (mm)", ""),
                    CalcField::number("maximum_mm", "Gap at most (mm)", ""),
                ],
            ),
            "inspect.hole_stack" => (
                "inspect.hole_stack",
                "HOLE STACK",
                "the screw a clearance hole was drilled for, and the hardware that goes with it",
                vec![
                    CalcField::number("hole_diameter_mm", "Hole diameter (mm)", "6.6"),
                    CalcField::number("grip_mm", "Grip (mm)", "12"),
                    CalcField::flag("washer", "Washer", true),
                    CalcField::flag("nut", "Nut", true),
                ],
            ),
            "inspect.bamboo_joint" => (
                "inspect.bamboo_joint",
                "BAMBOO JOINT",
                "a culm glued into a socket: shear-out, the glue line, and crushing under the moment",
                vec![
                    CalcField::number("culm_diameter_mm", "Culm diameter (mm)", "32"),
                    CalcField::number("wall_mm", "Wall (mm)", "8"),
                    CalcField::number("engagement_mm", "Engagement (mm)", "60"),
                    CalcField::number("glue_gap_mm", "Glue gap (mm)", "0.3"),
                    CalcField::number("adhesive_shear_mpa", "Adhesive shear (MPa)", "8"),
                    CalcField::number("bamboo_shear_mpa", "Bamboo shear (MPa)", "5"),
                    CalcField::number("bamboo_crush_mpa", "Bamboo crush (MPa)", "11"),
                    CalcField::flag("node_inside_socket", "Node inside socket", false),
                    CalcField::number("axial_n", "Axial force (N)", ""),
                    CalcField::number("moment_nm", "Moment (N·m)", ""),
                    CalcField::number("safety", "Safety factor", "3"),
                ],
            ),
            "inspect.parametric_check" => (
                "inspect.parametric_check",
                "PARAMETRIC CHECK",
                "every parameter nudged down and up on a copy: does the model follow, ignore it or break",
                vec![
                    CalcField::number("percent", "Nudge (%)", "10"),
                    CalcField::flag("include_model", "Sketch and feature dimensions too", false),
                    CalcField::number("max_drivers", "At most", "40"),
                ],
            ),
            _ => return crate::dialogs_tools::form(id),
        };
        Some(CalcForm { command, title, about, fields, result: None, of: None, picks: Vec::new(), apply: false, button: "Compute" })
    }

    /// The dialog's selection inputs for this form's picks.
    pub fn inputs(&self) -> Vec<SelInput> {
        self.picks.iter().map(|p| SelInput::new(p.label, p.accept, p.multi)).collect()
    }

    /// Defaults the design can give: the last inserted canvas for a canvas field.
    pub fn prefill(&mut self, s: &Session) {
        for f in &mut self.fields {
            if f.key == "canvas" && f.value.is_empty() {
                f.value = s.doc.canvases.last().map(|c| c.name.clone()).unwrap_or_default();
            }
        }
    }

    /// The parameters from the fields and the picks (see [`PickShape`]). A pick left empty
    /// leaves what the fields say (a JSON field, or the command's default).
    pub fn params_from(&self, s: &Session, inputs: &[SelInput]) -> Result<Value, String> {
        let mut p = self.params()?;
        for (pick, input) in self.picks.iter().zip(inputs) {
            let items = &input.items;
            let what = pick.label.to_ascii_lowercase();
            if items.is_empty() {
                if pick.required {
                    return Err(format!("pick the {what}"));
                }
                continue;
            }
            let names = || -> Vec<String> {
                items
                    .iter()
                    .filter_map(|x| match x {
                        Sel::Body { name } => Some(name.clone()),
                        Sel::Face { body, .. } | Sel::Edge { body, .. } | Sel::Vertex { body, .. } => Some(body.clone()),
                        _ => None,
                    })
                    .collect()
            };
            let value = match pick.shape {
                PickShape::Body => json!(names().first()),
                PickShape::Bodies => json!(names()),
                PickShape::Point => match items.first() {
                    Some(Sel::Face { point, .. } | Sel::Edge { point, .. } | Sel::Vertex { point, .. }) => json!([point.x, point.y, point.z]),
                    _ => return Err(format!("pick the {what}")),
                },
                PickShape::Lines => {
                    let members: Vec<Value> =
                        segments_of(s, items)?.into_iter().map(|(a, b)| json!({"from": [a.x, a.y, a.z], "to": [b.x, b.y, b.z]})).collect();
                    json!(members)
                }
                PickShape::Waypoints => {
                    let points: Vec<Value> = chain(segments_of(s, items)?)?.into_iter().map(|q| json!([q.x, q.y, q.z])).collect();
                    json!(points)
                }
                PickShape::Regions { force } => {
                    let force = force.map(|key| self.fields.iter().find(|f| f.key == key).map(|f| f.value.trim().to_string()).unwrap_or_default());
                    let force = match force.as_deref() {
                        None | Some("") => None,
                        Some(text) => {
                            let xyz: Option<Vec<f64>> = text.split(',').map(parse_number).collect();
                            match xyz {
                                Some(xyz) if xyz.len() == 3 => Some(json!(xyz)),
                                _ => return Err("the force needs three numbers: x, y, z".into()),
                            }
                        }
                    };
                    let mut regions = Vec::new();
                    for (i, item) in items.iter().enumerate() {
                        let Sel::Face { body, index, point } = item else { return Err(format!("the {what} must be faces")) };
                        let mut r = json!({"face": index, "point": [point.x, point.y, point.z], "body": body, "name": format!("{what} {}", i + 1)});
                        if let Some(f) = &force {
                            r["force_n"] = f.clone();
                        }
                        regions.push(r);
                    }
                    json!(regions)
                }
            };
            insert_path(&mut p, pick.key, value);
        }
        Ok(p)
    }

    fn values(&self) -> Vec<String> {
        self.fields
            .iter()
            .map(|f| match f.kind {
                FieldKind::Bool => f.on.to_string(),
                FieldKind::Choice(_) => f.choice.to_string(),
                _ => f.value.clone(),
            })
            .collect()
    }

    /// The command's parameters from the fields, or what is wrong with them. A key with dots
    /// (`profile.family`) lands in a nested object.
    pub fn params(&self) -> Result<Value, String> {
        let mut p = Value::Object(Map::new());
        for f in self.fields.iter().filter(|f| !f.key.starts_with('_')) {
            let v = f.value.trim();
            let value = match f.kind {
                FieldKind::Text => {
                    if v.is_empty() {
                        continue;
                    }
                    json!(v)
                }
                FieldKind::Number => {
                    if v.is_empty() {
                        continue;
                    }
                    json!(parse_number(v).ok_or_else(|| format!("{} needs a number", f.label))?)
                }
                FieldKind::Bool => json!(f.on),
                FieldKind::Choice(options) => match options.get(f.choice).filter(|o| !o.is_empty()) {
                    Some(o) => json!(o),
                    None => continue,
                },
                FieldKind::Chain => parse_chain(v)?,
                FieldKind::Json => {
                    if v.is_empty() {
                        continue;
                    }
                    serde_json::from_str::<Value>(v).map_err(|e| format!("{}: not valid JSON ({e})", f.label))?
                }
                FieldKind::Point => {
                    if v.is_empty() {
                        continue;
                    }
                    let xyz: Option<Vec<f64>> = v.split(',').map(parse_number).collect();
                    match xyz {
                        Some(xyz) if xyz.len() == 3 => json!(xyz),
                        _ => return Err(format!("{} needs three numbers: x, y, z", f.label)),
                    }
                }
            };
            insert_path(&mut p, f.key, value);
        }
        Ok(p)
    }
}

/// The world segments of picked sketch curves: a line of a plane sketch, or the polyline of a
/// 3D sketch's line; anything else is refused by its id.
fn segments_of(s: &Session, items: &[Sel]) -> Result<Vec<(Vec3, Vec3)>, String> {
    let st = s.world_state();
    let mut out = Vec::new();
    for item in items {
        let Sel::SketchCurve { id, sketch } = item else { return Err("pick sketch lines".into()) };
        let mut found = false;
        // A curve picked into the dialog names its sketch; curve ids repeat between sketches.
        for ss in st.sketches.iter().rev().filter(|ss| sketch.is_none_or(|k| ss.feature == k)) {
            if let Some(ci) = ss.sketch.curve_index(id) {
                let Some(solidcraft_engine::sketch::CurveKind::Line { a, b }) = ss.sketch.curves.get(ci).map(|c| c.kind.clone()) else {
                    return Err(format!("`{id}` is not a line: pick straight lines"));
                };
                let (Some(pa), Some(pb)) = (ss.sketch.point(a), ss.sketch.point(b)) else { return Err(format!("`{id}` has no ends")) };
                out.push((ss.plane.to_world(pa), ss.plane.to_world(pb)));
                found = true;
                break;
            }
            if let Some(wi) = ss.sketch.wire_index(id) {
                let pts = ss.sketch.wires.get(wi).map(|w| w.pts.clone()).unwrap_or_default();
                if pts.len() < 2 {
                    return Err(format!("`{id}` is a point, not a line"));
                }
                out.extend(pts.windows(2).map(|w| (w[0], w[1])));
                found = true;
                break;
            }
        }
        if !found {
            return Err(format!("no sketch has a curve `{id}`"));
        }
    }
    Ok(out)
}

/// Segments joined end to end into one polyline (either way round), or why they do not join.
fn chain(mut segments: Vec<(Vec3, Vec3)>) -> Result<Vec<Vec3>, String> {
    if segments.is_empty() {
        return Err("pick the lines of the route".into());
    }
    let near = |a: Vec3, b: Vec3| (a - b).dot(a - b) < 1e-12;
    let (a, b) = segments.remove(0);
    let mut path = vec![a, b];
    while !segments.is_empty() {
        let (start, end) = (*path.first().unwrap_or(&a), *path.last().unwrap_or(&b));
        if let Some(i) = segments.iter().position(|(p, q)| near(*p, end) || near(*q, end)) {
            let (p, q) = segments.remove(i);
            path.push(if near(p, end) { q } else { p });
        } else if let Some(i) = segments.iter().position(|(p, q)| near(*p, start) || near(*q, start)) {
            let (p, q) = segments.remove(i);
            path.insert(0, if near(p, start) { q } else { p });
        } else {
            return Err("the lines must join end to end, in one chain".into());
        }
    }
    Ok(path)
}

/// Puts `value` at `key` in `p`; a dotted key (`profile.family`) goes into nested objects.
fn insert_path(p: &mut Value, key: &str, value: Value) {
    let mut at = p;
    let mut parts = key.split('.').peekable();
    while let Some(part) = parts.next() {
        if !at.is_object() {
            *at = Value::Object(Map::new());
        }
        let Some(map) = at.as_object_mut() else { return };
        if parts.peek().is_none() {
            map.insert(part.to_string(), value);
            return;
        }
        at = map.entry(part.to_string()).or_insert_with(|| Value::Object(Map::new()));
    }
}

fn parse_number(s: &str) -> Option<f64> {
    s.trim().replace(',', ".").parse::<f64>().ok().filter(|x| x.is_finite())
}

/// A tolerance chain, one contributor per line: `[name] nominal ±tol [-1]` or
/// `[name] nominal lower upper [-1]`. `±` may be written `+-` or `+/-`; a trailing `-1` (or `-`)
/// makes the contributor subtract.
pub fn parse_chain(text: &str) -> Result<Value, String> {
    let mut chain = Vec::new();
    for (i, line) in text.lines().enumerate().filter(|(_, l)| !l.trim().is_empty()) {
        let row = i + 1;
        let mut tokens: Vec<&str> = line.split_whitespace().collect();
        let mut sensitivity = 1.0;
        if let Some(last) = tokens.last().copied()
            && matches!(last, "-1" | "-" | "+1" | "+")
            && tokens.len() >= 3
        {
            sensitivity = if last.starts_with('-') { -1.0 } else { 1.0 };
            tokens.pop();
        }
        let is_tol = |t: &str| t.starts_with('±') || t.starts_with("+-") || t.starts_with("+/-");
        let first_number =
            tokens.iter().position(|t| parse_number(t).is_some() || is_tol(t)).ok_or_else(|| format!("line {row}: no nominal size"))?;
        let name = tokens.iter().take(first_number).copied().collect::<Vec<_>>().join(" ");
        let rest = tokens.get(first_number..).unwrap_or(&[]);
        let nominal = rest.first().and_then(|t| parse_number(t)).ok_or_else(|| format!("line {row}: the nominal size must be a number"))?;
        let mut c = json!({"nominal_mm": nominal, "sensitivity": sensitivity});
        if !name.is_empty() {
            c["name"] = json!(name);
        }
        match rest.get(1..).unwrap_or(&[]) {
            [tol] => {
                let t = tol.trim_start_matches('±').trim_start_matches("+/-").trim_start_matches("+-");
                let t = parse_number(t).ok_or_else(|| format!("line {row}: the tolerance must be a number like ±0.1"))?;
                c["plus_minus_mm"] = json!(t.abs());
            }
            [lower, upper] => {
                let (lo, hi) = (parse_number(lower), parse_number(upper));
                let (Some(lo), Some(hi)) = (lo, hi) else { return Err(format!("line {row}: lower and upper must be numbers")) };
                c["lower_mm"] = json!(lo);
                c["upper_mm"] = json!(hi);
            }
            _ => return Err(format!("line {row}: write `name nominal ±tol` or `name nominal lower upper`")),
        }
        chain.push(c);
    }
    if chain.is_empty() {
        return Err("the chain needs at least one contributor, one per line".into());
    }
    Ok(Value::Array(chain))
}

/// The dialog's rows. Returns true when Enter applies (a form that applies on OK); a computing
/// form runs from its button instead.
pub fn rows(app: &mut SolveApp, ui: &mut egui::Ui, f: &mut CalcForm, inputs: &[SelInput]) -> bool {
    let t = Tokens::get();
    let mut enter = false;
    for (i, fld) in f.fields.iter_mut().enumerate() {
        match fld.kind {
            FieldKind::Text | FieldKind::Number | FieldKind::Point => {
                enter |= form_row(ui, fld.label, |ui| field(ui, &mut fld.value));
            }
            FieldKind::Bool => form_row(ui, fld.label, |ui| check(ui, &mut fld.on)),
            FieldKind::Choice(options) => form_row(ui, fld.label, |ui| combo(ui, &format!("calc_{i}"), options, &mut fld.choice)),
            // Several lines of text: the label on its own row, the text box under it.
            FieldKind::Chain | FieldKind::Json => {
                form_row(ui, fld.label, |_ui| {});
                crate::ui_kit::text_area(ui, fld.label, egui::Id::new(("calc_text", i)), &mut fld.value, 4, 1 << 16);
            }
        }
    }
    if f.apply {
        form_row(ui, "", |ui| {
            ui.label(RichText::new(f.about).color(t.text_dim));
        });
        return enter;
    }
    let now = f.values();
    let stale = f.result.is_some() && f.of.as_ref() != Some(&now);
    form_row(ui, "", |ui| {
        let label = if stale { format!("{} again", f.button) } else { f.button.to_string() };
        let button = crate::ui_kit::button(ui, label, crate::ui_kit::Button::Normal);
        crate::scenario::publish_handle("calc:compute", button.rect.center());
        if button.clicked() {
            f.result = Some(compute(app, f, inputs));
            f.of = Some(now);
        }
    });
    if f.result.is_none() {
        form_row(ui, "", |ui| {
            ui.label(RichText::new(f.about).color(t.text_dim));
        });
    }
    if let Some(r) = f.result.as_ref() {
        result_rows(ui, r);
    }
    false
}

/// Runs the command with the form's values and picks; an error is a reply with `error`.
pub(crate) fn compute(app: &mut SolveApp, f: &CalcForm, inputs: &[SelInput]) -> Value {
    let p = match f.params_from(&app.session, inputs) {
        Ok(p) => p,
        Err(e) => return json!({ "error": e }),
    };
    match app.run(f.command, p) {
        Ok(v) => v,
        Err(e) => json!({ "error": e }),
    }
}

/// A reply key as a row label: the unit suffix into brackets, underscores into spaces.
pub fn label_of(key: &str) -> String {
    let units: &[(&str, &str)] = &[
        ("_mm3", " (mm³)"),
        ("_mm2", " (mm²)"),
        ("_mm", " (mm)"),
        ("_mpa", " (MPa)"),
        ("_gpa", " (GPa)"),
        ("_pa", " (Pa)"),
        ("_nm", " (N·m)"),
        ("_n_m", " (N/m)"),
        ("_n", " (N)"),
        ("_hz", " (Hz)"),
        ("_kg", " (kg)"),
        ("_deg", " (°)"),
        ("_k", " (K)"),
        ("_c", " (°C)"),
        ("_w", " (W)"),
        ("_g", " (g)"),
    ];
    let (stem, unit) = units.iter().find_map(|(s, u)| key.strip_suffix(s).map(|stem| (stem, *u))).unwrap_or((key, ""));
    let mut words = stem.replace('_', " ");
    if let Some(first) = words.get(..1) {
        let upper = first.to_ascii_uppercase();
        words.replace_range(..1, &upper);
    }
    format!("{words}{unit}")
}

/// A number as the dialog shows it: whole numbers plain, otherwise up to three decimals.
pub fn show_number(x: f64) -> String {
    if !x.is_finite() {
        return "—".into();
    }
    if x.fract() == 0.0 && x.abs() < 1e15 {
        return format!("{x:.0}");
    }
    if x.abs() >= 1e6 {
        return format!("{x:.3e}");
    }
    let s = format!("{x:.3}");
    let s = s.trim_end_matches('0');
    s.trim_end_matches('.').to_string()
}

fn scalar(v: &Value) -> Option<String> {
    match v {
        Value::Null => Some("—".into()),
        Value::Bool(b) => Some(if *b { "yes" } else { "no" }.into()),
        Value::Number(n) => Some(n.as_f64().map(show_number).unwrap_or_else(|| n.to_string())),
        Value::String(s) => Some(s.clone()),
        _ => None,
    }
}

/// An object as one line: `key value, key value`.
fn one_line(v: &Value) -> String {
    match v {
        Value::Object(m) => {
            m.iter().filter_map(|(k, x)| scalar(x).map(|s| format!("{} {s}", label_of(k).to_ascii_lowercase()))).collect::<Vec<_>>().join(", ")
        }
        Value::Array(a) => a.iter().map(one_line).collect::<Vec<_>>().join("; "),
        other => scalar(other).unwrap_or_default(),
    }
}

/// The reply's rows: the figures by key, nested objects indented, lists one line per item;
/// then the verdict, the advice and the warnings.
pub fn result_rows(ui: &mut egui::Ui, r: &Value) {
    let t = Tokens::get();
    if let Some(e) = r.get("error").and_then(Value::as_str) {
        form_row(ui, "", |ui| {
            ui.label(RichText::new(e).color(t.error));
        });
        return;
    }
    let Some(map) = r.as_object() else { return };
    figure_rows(ui, map, "");
    if let Some(v) = map.get("verdict").and_then(Value::as_str) {
        form_row(ui, "Verdict", |ui| {
            ui.label(RichText::new(v).strong());
        });
    }
    if let Some(a) = map.get("advice").and_then(Value::as_str).filter(|a| !a.is_empty()) {
        form_row(ui, "Advice", |ui| {
            ui.add(egui::Label::new(RichText::new(a).color(t.text_dim)).wrap());
        });
    }
    for w in map.get("warnings").and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str) {
        form_row(ui, "", |ui| {
            ui.add(egui::Label::new(RichText::new(w).color(t.warning)).wrap());
        });
    }
}

fn figure_rows(ui: &mut egui::Ui, map: &Map<String, Value>, indent: &str) {
    for (k, v) in map {
        if matches!(k.as_str(), "verdict" | "advice" | "warnings" | "error") {
            continue;
        }
        let label = format!("{indent}{}", label_of(k));
        match v {
            Value::Object(inner) => {
                form_row(ui, &label, |ui| {
                    ui.label("");
                });
                figure_rows(ui, inner, &format!("{indent}  "));
            }
            Value::Array(items) if items.is_empty() => {}
            Value::Array(items) if items.iter().all(|x| scalar(x).is_some()) => {
                form_row(ui, &label, |ui| {
                    ui.add(egui::Label::new(items.iter().filter_map(scalar).collect::<Vec<_>>().join(", ")).wrap());
                });
            }
            Value::Array(items) => {
                form_row(ui, &label, |ui| {
                    ui.label("");
                });
                for item in items {
                    form_row(ui, "", |ui| {
                        ui.add(egui::Label::new(one_line(item)).wrap());
                    });
                }
            }
            other => {
                if let Some(s) = scalar(other) {
                    form_row(ui, &label, |ui| {
                        ui.label(s);
                    });
                }
            }
        }
    }
}
