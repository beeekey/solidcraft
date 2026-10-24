//! Whether this part can be milled, turned and drilled: measured, with the limit each
//! measurement was judged against, and where on the part it is. Millimetres throughout.
//!
//! The pure rules and the shop's tool tables live here so they can be checked without a
//! document; the engine command recognises the features (holes from cylindrical faces, corner
//! radii from concave cylinders, the thinnest wall from the mesh, the body of revolution from
//! the vertices) and hands them in. The tool-access scan runs on a voxel grid: from each of the
//! six axis directions a square cutter column is slid to every surface cell, which finds the
//! setups a part needs, the undercuts no straight approach reaches, and sealed voids.

use serde::{Deserialize, Serialize};
use solidcraft_geom::Vec3;

// --- the tools a shop has on the shelf -------------------------------------------------------

/// Standard 3-axis end mill diameters (mm), ascending.
pub const END_MILL_DIAMETERS: [f64; 9] = [3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0];
/// ISO 1832 turning insert nose radii (mm).
pub const INSERT_NOSE_RADII: [f64; 3] = [0.4, 0.8, 1.2];
/// Parting and grooving blade widths (mm).
pub const GROOVE_TOOL_WIDTHS: [f64; 3] = [2.0, 3.0, 4.0];

/// ISO 235 twist drill diameters (mm): 1.0–2.5 by 0.05, 2.5–10 by 0.1; above 10 mm this is the
/// common stocking practice in 0.5 steps rather than the standard.
pub fn iso235_drill_sizes() -> Vec<f64> {
    let mut sizes: Vec<f64> = (100..=250).step_by(5).map(|n| n as f64 / 100.0).collect();
    sizes.extend((26..=100).map(|n| n as f64 / 10.0));
    sizes.extend((105..=500).step_by(5).map(|n| n as f64 / 10.0));
    sizes
}

/// Whole and half millimetres from 1 to 50: the sizes a shop reaches for first.
pub fn preferred_drill_sizes() -> Vec<f64> {
    (10..=500).step_by(5).map(|n| n as f64 / 10.0).collect()
}

/// The nearest entry of a table to `value`; 0 for an empty table.
pub fn nearest_from_table(table: &[f64], value: f64) -> f64 {
    table.iter().copied().min_by(|a, b| (a - value).abs().total_cmp(&(b - value).abs())).unwrap_or(0.0)
}

/// Whether `value` is in the table within `tolerance` (mm).
pub fn is_in_table(table: &[f64], value: f64, tolerance: f64) -> bool {
    !table.is_empty() && (value - nearest_from_table(table, value)).abs() <= tolerance
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Material {
    Aluminium,
    Steel,
    Plastic,
}

impl Material {
    /// "Aluminium" | "Aluminum" | "Steel" | "Plastic", case-insensitive; `None` otherwise, so a
    /// material nobody recognises does not silently become the middle one.
    pub fn parse(text: &str) -> Option<Material> {
        match text.trim().to_ascii_lowercase().as_str() {
            "aluminium" | "aluminum" => Some(Material::Aluminium),
            "steel" => Some(Material::Steel),
            "plastic" => Some(Material::Plastic),
            _ => None,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Material::Aluminium => "Aluminium",
            Material::Steel => "Steel",
            Material::Plastic => "Plastic",
        }
    }
}

/// The wall a material will hold (mm).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WallLimits {
    /// Below this it will not stand up to cutting.
    pub absolute: f64,
    /// Below this it is difficult and expensive.
    pub sensible: f64,
    /// A wall taller than this many times its thickness chatters.
    pub height_ratio: f64,
    /// True when the material has no row of its own and the aluminium numbers were used.
    pub borrowed: bool,
}

pub fn wall_limits(material: Material) -> WallLimits {
    match material {
        Material::Aluminium => WallLimits { absolute: 0.5, sensible: 1.0, height_ratio: 15.0, borrowed: false },
        Material::Steel => WallLimits { absolute: 0.8, sensible: 1.5, height_ratio: 15.0, borrowed: false },
        // No plastic row exists in the guidance these came from; saying the aluminium numbers
        // were borrowed is honest, inventing a row is not.
        Material::Plastic => WallLimits { absolute: 0.5, sensible: 1.0, height_ratio: 15.0, borrowed: true },
    }
}

// --- what the check assumed ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Process {
    Auto,
    Milling3Axis,
    Turning,
}

/// Every default is defensible and every one is wrong for somebody; the report echoes them.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachiningSetup {
    pub process: Process,
    pub material: Material,
    /// mm, the middle of the standard set.
    pub cutter_diameter: f64,
    /// mm, the practical 3-axis minimum.
    pub min_corner_radius: f64,
    /// A corner radius this much over the tool radius keeps the cutter out of a full slot.
    pub corner_clearance: f64,
    pub insert_nose_radius: f64,
    pub blind_drill_ratio: f64,
    pub through_drill_ratio: f64,
    /// How many setups before it is worth a warning: a judgement, exposed to be argued with.
    pub setup_warn_at: usize,
}

impl Default for MachiningSetup {
    fn default() -> Self {
        MachiningSetup {
            process: Process::Auto,
            material: Material::Aluminium,
            cutter_diameter: 6.0,
            min_corner_radius: 1.5,
            corner_clearance: 1.2,
            insert_nose_radius: 0.8,
            blind_drill_ratio: 5.0,
            through_drill_ratio: 10.0,
            setup_warn_at: 3,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Severity {
    Note,
    Warning,
    Refuse,
}

impl Severity {
    pub fn name(self) -> &'static str {
        match self {
            Severity::Note => "NOTE",
            Severity::Warning => "WARNING",
            Severity::Refuse => "REFUSE",
        }
    }
}

/// One measured fact about making this part: the value, the limit it was judged against and
/// the place, because a machinist cannot act on a verdict.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Finding {
    pub severity: Severity,
    pub rule: String,
    pub detail: String,
    pub measured: f64,
    pub limit: f64,
    /// "mm" | "mm²" | "mm³" | "ratio" | "deg" | "count" | "" (qualitative)
    pub unit: String,
    pub at: Option<Vec3>,
}

fn finding(severity: Severity, rule: &str, detail: String, measured: f64, limit: f64, unit: &str, at: Option<Vec3>) -> Finding {
    Finding { severity, rule: rule.into(), detail, measured, limit, unit: unit.into(), at }
}

/// An inside corner running along the tool axis: the thing a round cutter cannot cut sharp.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InternalCorner {
    pub at: Vec3,
    /// 0 for a sharp corner.
    pub radius: f64,
    /// How far it runs: the pocket depth there.
    pub depth: f64,
    pub sharp: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pocket {
    pub floor_at: Vec3,
    pub depth: f64,
    /// Twice the largest circle that fits the floor.
    pub narrowest_width: f64,
    pub floor_corner_sharp: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Hole {
    pub axis_point: Vec3,
    pub axis: Vec3,
    pub diameter: f64,
    pub depth: f64,
    pub through: bool,
    pub entry_at: Vec3,
    pub entry_face_curved: bool,
    /// Worst end-face tilt off square, degrees.
    pub entry_deviation_degrees: f64,
    pub flat_bottom: bool,
    /// Least material around the wall (mm); 0 = not measured.
    pub thinnest_surround: f64,
    pub broken_into: bool,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Groove {
    pub station: f64,
    pub width: f64,
    pub depth: f64,
}

/// How round the part is about an axis.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RevolutionReport {
    pub axis_point: Vec3,
    pub axis: Vec3,
    pub is_body_of_revolution: bool,
    /// π ∫ rmax(s)² ds
    pub revolved_volume: f64,
    pub actual_volume: f64,
    /// The flats, keyways and cross-holes.
    pub off_axis_volume: f64,
    pub length: f64,
    pub max_diameter: f64,
    pub slenderness: f64,
    pub grooves: Vec<Groove>,
    pub flats: usize,
    pub cross_holes: usize,
}

/// A concave shoulder fillet on a turned part.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShoulderFillet {
    pub radius: f64,
    pub at: Vec3,
}

// --- the rules ---------------------------------------------------------------------------------

pub fn check_corners(corners: &[InternalCorner], setup: &MachiningSetup) -> Vec<Finding> {
    let mut out = Vec::new();
    let cutter_radius = 0.5 * setup.cutter_diameter;
    for corner in corners {
        let at = Some(corner.at);
        if corner.sharp {
            out.push(finding(
                Severity::Refuse,
                "internal corner radius",
                "a sharp internal vertical corner cannot be cut: the cutter is round. Radius it to at least the cutter radius.".into(),
                0.0,
                cutter_radius,
                "mm",
                at,
            ));
            continue;
        }
        if corner.radius < cutter_radius - 1e-9 {
            out.push(finding(
                Severity::Refuse,
                "internal corner radius",
                "the cutter does not fit this corner. Open the radius, or name a smaller cutter.".into(),
                corner.radius,
                cutter_radius,
                "mm",
                at,
            ));
            continue;
        }
        if corner.radius < setup.min_corner_radius - 1e-9 {
            out.push(finding(
                Severity::Warning,
                "internal corner radius",
                "below the practical minimum for 3-axis work.".into(),
                corner.radius,
                setup.min_corner_radius,
                "mm",
                at,
            ));
        }
        if corner.depth > 0.0 && corner.radius < corner.depth / 3.0 - 1e-9 {
            out.push(finding(
                Severity::Note,
                "corner radius against pocket depth",
                "good practice is a corner radius of at least a third of the depth.".into(),
                corner.radius,
                corner.depth / 3.0,
                "mm",
                at,
            ));
        }
        if corner.radius < setup.corner_clearance * cutter_radius - 1e-9 {
            out.push(finding(
                Severity::Note,
                "corner radius against cutter radius",
                "the cutter is full-slotting into this corner: chatter, deflection and a poor finish. A radius 20-30% over the tool's buys it back."
                    .into(),
                corner.radius,
                setup.corner_clearance * cutter_radius,
                "mm",
                at,
            ));
        }
        let nearest = nearest_from_table(&END_MILL_DIAMETERS, 2.0 * corner.radius);
        if (2.0 * corner.radius - nearest).abs() > 5e-3 {
            out.push(finding(
                Severity::Note,
                "non-standard corner radius",
                format!("this radius asks for a cutter nobody stocks; the nearest standard end mill would give R{:.2}.", 0.5 * nearest),
                corner.radius,
                0.5 * nearest,
                "mm",
                at,
            ));
        }
    }
    out
}

pub fn check_pockets(pockets: &[Pocket], setup: &MachiningSetup) -> Vec<Finding> {
    let mut out = Vec::new();
    for pocket in pockets {
        if pocket.narrowest_width <= 0.0 {
            continue;
        }
        let at = Some(pocket.floor_at);
        let aspect = pocket.depth / pocket.narrowest_width;
        if aspect > 6.0 {
            out.push(finding(
                Severity::Refuse,
                "pocket depth vs narrowest width",
                "past six times its width a pocket needs special tooling, or a redesign.".into(),
                aspect,
                6.0,
                "ratio",
                at,
            ));
        } else if aspect > 4.0 {
            out.push(finding(
                Severity::Warning,
                "pocket depth vs narrowest width",
                "long-reach or reduced-neck tooling, 30-50% more cycle time and a rougher finish.".into(),
                aspect,
                4.0,
                "ratio",
                at,
            ));
        } else if aspect > 3.0 {
            out.push(finding(
                Severity::Note,
                "pocket depth vs narrowest width",
                "past the reach of a standard end mill, whose flute length is about three diameters.".into(),
                aspect,
                3.0,
                "ratio",
                at,
            ));
        }
        if pocket.narrowest_width < setup.cutter_diameter - 1e-9 {
            out.push(finding(
                Severity::Refuse,
                "pocket narrower than the cutter",
                "no part of this floor is reachable by the named cutter.".into(),
                pocket.narrowest_width,
                setup.cutter_diameter,
                "mm",
                at,
            ));
        }
        if pocket.floor_corner_sharp {
            out.push(finding(Severity::Note, "floor-to-wall corner", "a square end mill leaves a sharp floor corner and a bull-nose leaves a radius. Decide which, and say so, rather than leaving it to the machinist.".into(), 0.0, 0.0, "", at));
        }
    }
    out
}

/// The thinnest wall against what the material will hold.
pub fn check_wall(thinnest: f64, at: Vec3, setup: &MachiningSetup) -> Vec<Finding> {
    let limits = wall_limits(setup.material);
    let borrowed = if limits.borrowed { " (no plastic row exists in the guidance, so the aluminium limits are used and this says so)" } else { "" };
    let mut out = Vec::new();
    if thinnest < limits.absolute - 1e-9 {
        out.push(finding(
            Severity::Refuse,
            "wall thickness",
            format!("thinner than {} will stand up to being cut{borrowed}.", setup.material.name()),
            thinnest,
            limits.absolute,
            "mm",
            Some(at),
        ));
    } else if thinnest < limits.sensible - 1e-9 {
        out.push(finding(
            Severity::Warning,
            "wall thickness",
            format!("thin enough to be difficult and expensive{borrowed}."),
            thinnest,
            limits.sensible,
            "mm",
            Some(at),
        ));
    }
    out
}

pub fn check_holes(holes: &[Hole], setup: &MachiningSetup) -> Vec<Finding> {
    let mut out = Vec::new();
    let iso = iso235_drill_sizes();
    let preferred = preferred_drill_sizes();
    for hole in holes {
        if hole.diameter <= 0.0 {
            continue;
        }
        let at = Some(hole.entry_at);
        let aspect = hole.depth / hole.diameter;
        let limit = if hole.through { setup.through_drill_ratio } else { setup.blind_drill_ratio };
        if aspect > limit {
            let detail = if hole.through {
                "past ten diameters even drilling from both ends: this is gun drilling, and it should be called for by name."
            } else {
                "past five diameters for an ordinary jobber drill."
            };
            out.push(finding(Severity::Refuse, "hole depth vs diameter", detail.into(), aspect, limit, "ratio", at));
        }
        if !is_in_table(&iso, hole.diameter, 5e-3) {
            let nearest = nearest_from_table(&iso, hole.diameter);
            out.push(finding(
                Severity::Warning,
                "non-standard drill size",
                format!("no ISO 235 drill is this size; the nearest is {nearest:.2} mm."),
                hole.diameter,
                nearest,
                "mm",
                at,
            ));
        } else if !is_in_table(&preferred, hole.diameter, 5e-3) {
            let nearest = nearest_from_table(&preferred, hole.diameter);
            out.push(finding(
                Severity::Note,
                "drill size off the whole and half millimetres",
                format!("a special tool for no reason; {nearest:.1} mm is on the wall."),
                hole.diameter,
                nearest,
                "mm",
                at,
            ));
        }
        if hole.thinnest_surround > 0.0 && hole.thinnest_surround < 0.5 * hole.diameter {
            out.push(finding(
                Severity::Refuse,
                "material around a hole",
                "less than half a diameter of material on one side: the wall collapses while it is being drilled.".into(),
                hole.thinnest_surround,
                0.5 * hole.diameter,
                "mm",
                at,
            ));
        }
        if hole.entry_face_curved {
            out.push(finding(
                Severity::Warning,
                "drill entry face",
                "the drill lands on a curve, where it wanders and breaks. Add a spot-face, and model it.".into(),
                0.0,
                0.0,
                "",
                at,
            ));
        } else if hole.entry_deviation_degrees > 1.0 {
            out.push(finding(
                Severity::Warning,
                "drill entry face",
                "the entry face is not square to the axis. Add a spot-face, and model it.".into(),
                hole.entry_deviation_degrees,
                1.0,
                "deg",
                at,
            ));
        }
        if hole.flat_bottom {
            out.push(finding(
                Severity::Note,
                "flat-bottomed blind hole",
                "a drill leaves a cone. A flat bottom is a second operation with an end mill, and should be called for deliberately.".into(),
                0.0,
                0.0,
                "",
                at,
            ));
        }
        if hole.broken_into {
            out.push(finding(
                Severity::Note,
                "bore broken into by another feature",
                "something has cut into this bore, so it is no longer a complete cylinder.".into(),
                0.0,
                0.0,
                "",
                at,
            ));
        }
    }
    // A drill breaking into an existing bore deflects and often snaps.
    for a in 0..holes.len() {
        for b in a + 1..holes.len() {
            let (one, two) = (&holes[a], &holes[b]);
            let gap = one.axis_point - two.axis_point;
            let along = gap.dot(two.axis);
            let distance = (gap - two.axis * along).len();
            let touching = 0.5 * (one.diameter + two.diameter);
            if distance < touching {
                out.push(finding(
                    Severity::Warning,
                    "intersecting holes",
                    "a drill breaking into an existing bore deflects and often snaps. Where it cannot be avoided, specify the order.".into(),
                    distance,
                    touching,
                    "mm",
                    Some(one.axis_point),
                ));
            }
        }
    }
    out
}

pub fn check_turning(revolution: &RevolutionReport, fillets: &[ShoulderFillet]) -> Vec<Finding> {
    let mut out = Vec::new();
    if revolution.actual_volume <= 0.0 {
        return out;
    }
    let s = revolution.slenderness;
    if s > 10.0 {
        out.push(finding(
            Severity::Warning,
            "slenderness",
            "past ten to one it needs a steady rest, or it chatters badly enough to ruin the tolerance and the finish.".into(),
            s,
            10.0,
            "ratio",
            None,
        ));
    } else if s > 5.0 {
        out.push(finding(
            Severity::Warning,
            "slenderness",
            "past five to one it needs a tailstock centre; if the centre has to stay, model it.".into(),
            s,
            5.0,
            "ratio",
            None,
        ));
    } else if s > 3.0 {
        out.push(finding(
            Severity::Note,
            "slenderness",
            "past what the chuck holds alone, and approaching where deflection shows.".into(),
            s,
            3.0,
            "ratio",
            None,
        ));
    }
    if !revolution.is_body_of_revolution {
        let mut detail = format!("this is not a body of revolution: {:.1} mm³ of it is off the axis", revolution.off_axis_volume);
        if revolution.flats > 0 {
            detail.push_str(&format!(", {} flat(s)", revolution.flats));
        }
        if revolution.cross_holes > 0 {
            detail.push_str(&format!(", {} cross-hole(s)", revolution.cross_holes));
        }
        detail.push_str(". That is a second operation on a mill or with live tooling -- fine, but it changes the price and it breaks concentricity if the part is re-chucked.");
        out.push(finding(Severity::Note, "off-axis material", detail, revolution.off_axis_volume, 0.0, "mm³", None));
    }
    for groove in &revolution.grooves {
        if groove.width <= 0.0 {
            continue;
        }
        if !is_in_table(&GROOVE_TOOL_WIDTHS, groove.width, 0.2) {
            let nearest = nearest_from_table(&GROOVE_TOOL_WIDTHS, groove.width);
            out.push(finding(
                Severity::Note,
                "groove width",
                format!("grooving blades come in fixed widths; the nearest stocked is {nearest:.1} mm."),
                groove.width,
                nearest,
                "mm",
                None,
            ));
        }
    }
    for fillet in fillets {
        if fillet.radius < 0.2 {
            out.push(finding(
                Severity::Warning,
                "shoulder fillet radius",
                "below R0.2 needs a special insert or EDM.".into(),
                fillet.radius,
                0.2,
                "mm",
                Some(fillet.at),
            ));
        } else if !is_in_table(&INSERT_NOSE_RADII, fillet.radius, 0.01) {
            let nearest = nearest_from_table(&INSERT_NOSE_RADII, fillet.radius);
            out.push(finding(
                Severity::Note,
                "shoulder fillet radius",
                format!("design the shoulder to a standard nose radius and it costs nothing; the nearest is R{nearest:.1}."),
                fillet.radius,
                nearest,
                "mm",
                Some(fillet.at),
            ));
        }
    }
    out
}

/// Worst first, and within a severity the largest offence first.
pub fn sort_findings(findings: &mut [Finding]) {
    findings.sort_by(|a, b| b.severity.cmp(&a.severity).then_with(|| b.measured.total_cmp(&a.measured)));
}

// --- tool access on a voxel grid ---------------------------------------------------------------

/// A surface the straight approaches cannot reach.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UnreachableRegion {
    /// Surface cell faces in it.
    pub faces: usize,
    /// mm²
    pub area: f64,
    /// A point in the air beside it (mm).
    pub sample: Vec3,
}

/// What the six straight approaches found.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AccessScan {
    /// Surface cell faces that are not the stock's own outside.
    pub machined_faces: usize,
    pub surface_faces: usize,
    /// Machined faces no approach reaches, as a fraction of all surface faces.
    pub unreachable_fraction: f64,
    pub unreachable_regions: Vec<UnreachableRegion>,
    /// Approach directions needed, in the order +X −X +Y −Y +Z −Z.
    pub setup_axes: [bool; 6],
    pub setup_count: usize,
    /// Machined faces reached from each direction.
    pub reached_by_axis: [usize; 6],
    /// Air sealed inside the material.
    pub has_trapped: bool,
    /// mm³
    pub trapped_volume: f64,
    pub trapped_sample: Option<Vec3>,
}

pub const AXIS_NAMES: [&str; 6] = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];
const STEPS: [[i64; 3]; 6] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];

/// A grid of cells, `solid` true where there is material; `h` is the cell size (mm) and
/// `origin` the corner of cell (0, 0, 0) (mm).
pub struct AccessGrid<'a> {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    pub h: f64,
    pub origin: Vec3,
    pub solid: &'a [bool],
}

impl AccessGrid<'_> {
    fn at(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.ny + j) * self.nx + i
    }
    fn ijk(&self, v: usize) -> (usize, usize, usize) {
        (v % self.nx, (v / self.nx) % self.ny, v / (self.nx * self.ny))
    }
    fn solid_at(&self, i: i64, j: i64, k: i64) -> bool {
        i >= 0
            && j >= 0
            && k >= 0
            && (i as usize) < self.nx
            && (j as usize) < self.ny
            && (k as usize) < self.nz
            && self.solid.get(self.at(i as usize, j as usize, k as usize)).copied().unwrap_or(false)
    }
    fn centre(&self, v: usize) -> Vec3 {
        let (i, j, k) = self.ijk(v);
        self.origin + Vec3::new(i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5) * self.h
    }
}

/// Slides a square cutter column of radius `tool_radius` (mm) to every surface cell from each
/// of the six directions. A face counts as reached when some column that fits (every cell of
/// its cross-section clear to the boundary) passes within the tool radius of the face's air
/// cell: an end mill cuts a pocket wall with its flank. Stock is the bounding box: faces on it
/// are the billet's own outside and need no machining.
pub fn scan_access(grid: &AccessGrid<'_>, tool_radius: f64) -> AccessScan {
    let mut scan = AccessScan::default();
    let (nx, ny, nz) = (grid.nx, grid.ny, grid.nz);
    let cells = nx * ny * nz;
    if cells == 0 || grid.solid.len() != cells || !(grid.h > 0.0) {
        return scan;
    }
    let t = ((tool_radius / grid.h).ceil() as usize).max(1);
    let dims = [nx, ny, nz];
    // The stock: the extent of the material.
    let (mut lo, mut hi) = ([usize::MAX; 3], [0usize; 3]);
    for v in 0..cells {
        if grid.solid[v] {
            let (i, j, k) = grid.ijk(v);
            for (a, c) in [i, j, k].into_iter().enumerate() {
                lo[a] = lo[a].min(c);
                hi[a] = hi[a].max(c);
            }
        }
    }
    if lo[0] == usize::MAX {
        return scan;
    }
    // Per direction: `clear` = empty from here to the boundary along d; `fits` = clear over the
    // whole (2t+1)² cross-section; `reach` = a fitting column within t of here across d.
    let mut reach_by_dir: Vec<Vec<bool>> = Vec::with_capacity(6);
    for (d, step) in STEPS.iter().enumerate() {
        let axis = d / 2;
        let mut clear = vec![false; cells];
        // Walk from the boundary inward along -step.
        let extent = dims[axis];
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let c = [i, j, k];
                    // Process in order so that the cell ahead (c + step) is already done: iterate
                    // with the axis coordinate reversed when the step is positive.
                    let s = c[axis];
                    let pos = if step[axis] > 0 { extent - 1 - s } else { s };
                    let mut cc = c;
                    cc[axis] = pos;
                    let v = grid.at(cc[0], cc[1], cc[2]);
                    if grid.solid[v] {
                        continue;
                    }
                    let ahead = [cc[0] as i64 + step[0], cc[1] as i64 + step[1], cc[2] as i64 + step[2]];
                    let outside = ahead[axis] < 0 || ahead[axis] as usize >= extent;
                    clear[v] = outside || clear[grid.at(ahead[0] as usize, ahead[1] as usize, ahead[2] as usize)];
                }
            }
        }
        // Erode over the cross-section (min), then dilate (max), each separable over the two
        // axes across the approach.
        let across: Vec<usize> = (0..3).filter(|a| *a != axis).collect();
        let fits = separable(&clear, dims, &across, t, false);
        let reach = separable(&fits, dims, &across, t, true);
        reach_by_dir.push(reach);
    }
    // Every surface face.
    let mut face_reached_by: Vec<[bool; 6]> = Vec::new();
    let mut unreached_air: Vec<usize> = Vec::new();
    let mut machined = 0usize;
    let mut surface = 0usize;
    for v in 0..cells {
        if !grid.solid[v] {
            continue;
        }
        let (i, j, k) = grid.ijk(v);
        for (f, step) in STEPS.iter().enumerate() {
            let n = [i as i64 + step[0], j as i64 + step[1], k as i64 + step[2]];
            if grid.solid_at(n[0], n[1], n[2]) {
                continue;
            }
            surface += 1;
            let axis = f / 2;
            let extreme = if step[axis] > 0 { [i, j, k][axis] == hi[axis] } else { [i, j, k][axis] == lo[axis] };
            if extreme {
                continue; // the stock's own outside
            }
            machined += 1;
            let in_grid = n.iter().all(|c| *c >= 0) && (n[0] as usize) < nx && (n[1] as usize) < ny && (n[2] as usize) < nz;
            let mut reached = [false; 6];
            if in_grid {
                let nv = grid.at(n[0] as usize, n[1] as usize, n[2] as usize);
                for d in 0..6 {
                    reached[d] = reach_by_dir[d][nv];
                }
                if !reached.iter().any(|r| *r) {
                    unreached_air.push(nv);
                }
            }
            face_reached_by.push(reached);
        }
    }
    scan.surface_faces = surface;
    scan.machined_faces = machined;
    for reached in &face_reached_by {
        for (d, hit) in reached.iter().enumerate() {
            if *hit {
                scan.reached_by_axis[d] += 1;
            }
        }
    }
    // Setups: the fewest directions that cover every reachable machined face (greedy).
    let mut uncovered: Vec<&[bool; 6]> = face_reached_by.iter().filter(|r| r.iter().any(|x| *x)).collect();
    while !uncovered.is_empty() {
        let best = (0..6).max_by_key(|d| uncovered.iter().filter(|r| r[*d]).count()).unwrap_or(0);
        if uncovered.iter().filter(|r| r[best]).count() == 0 {
            break;
        }
        scan.setup_axes[best] = true;
        uncovered.retain(|r| !r[best]);
    }
    scan.setup_count = scan.setup_axes.iter().filter(|a| **a).count();
    scan.unreachable_fraction = if surface > 0 { unreached_air.len() as f64 / surface as f64 } else { 0.0 };
    // Unreachable regions: the air cells beside unreached faces, clustered by adjacency.
    unreached_air.sort_unstable();
    unreached_air.dedup();
    let mut region_of: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    let members: std::collections::HashSet<usize> = unreached_air.iter().copied().collect();
    let mut regions: Vec<UnreachableRegion> = Vec::new();
    for &start in &unreached_air {
        if region_of.contains_key(&start) {
            continue;
        }
        let id = regions.len();
        let mut count = 0usize;
        let mut stack = vec![start];
        region_of.insert(start, id);
        while let Some(v) = stack.pop() {
            count += 1;
            let (i, j, k) = grid.ijk(v);
            for step in &STEPS {
                let n = [i as i64 + step[0], j as i64 + step[1], k as i64 + step[2]];
                if n.iter().any(|c| *c < 0) || n[0] as usize >= nx || n[1] as usize >= ny || n[2] as usize >= nz {
                    continue;
                }
                let nv = grid.at(n[0] as usize, n[1] as usize, n[2] as usize);
                if members.contains(&nv) && !region_of.contains_key(&nv) {
                    region_of.insert(nv, id);
                    stack.push(nv);
                }
            }
        }
        regions.push(UnreachableRegion { faces: count, area: count as f64 * grid.h * grid.h, sample: grid.centre(start) });
    }
    regions.sort_by_key(|r| std::cmp::Reverse(r.faces));
    scan.unreachable_regions = regions;
    // Trapped air: empty cells not connected to the grid's border.
    let mut seen = vec![false; cells];
    let mut stack: Vec<usize> = Vec::new();
    for (v, cell_seen) in seen.iter_mut().enumerate() {
        let (i, j, k) = grid.ijk(v);
        let border = i == 0 || j == 0 || k == 0 || i + 1 == nx || j + 1 == ny || k + 1 == nz;
        if border && !grid.solid[v] {
            *cell_seen = true;
            stack.push(v);
        }
    }
    while let Some(v) = stack.pop() {
        let (i, j, k) = grid.ijk(v);
        for step in &STEPS {
            let n = [i as i64 + step[0], j as i64 + step[1], k as i64 + step[2]];
            if n.iter().any(|c| *c < 0) || n[0] as usize >= nx || n[1] as usize >= ny || n[2] as usize >= nz {
                continue;
            }
            let nv = grid.at(n[0] as usize, n[1] as usize, n[2] as usize);
            if !grid.solid[nv] && !seen[nv] {
                seen[nv] = true;
                stack.push(nv);
            }
        }
    }
    let mut trapped = 0usize;
    for (v, cell_seen) in seen.iter().enumerate() {
        if !grid.solid[v] && !cell_seen {
            trapped += 1;
            if scan.trapped_sample.is_none() {
                scan.trapped_sample = Some(grid.centre(v));
            }
        }
    }
    scan.has_trapped = trapped > 0;
    scan.trapped_volume = trapped as f64 * grid.h * grid.h * grid.h;
    scan
}

/// A separable min (erode) or max (dilate) over a window of ±t along each of `axes`.
fn separable(input: &[bool], dims: [usize; 3], axes: &[usize], t: usize, dilate: bool) -> Vec<bool> {
    let mut current = input.to_vec();
    let (nx, ny, nz) = (dims[0], dims[1], dims[2]);
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    for &axis in axes {
        let mut next = current.clone();
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let c = [i, j, k];
                    let s = c[axis] as i64;
                    let mut value = !dilate;
                    for o in -(t as i64)..=(t as i64) {
                        let p = s + o;
                        let inside = p >= 0 && (p as usize) < dims[axis];
                        // Outside the grid counts as clear (air beyond the part).
                        let sample = if inside {
                            let mut cc = c;
                            cc[axis] = p as usize;
                            current[at(cc[0], cc[1], cc[2])]
                        } else {
                            !dilate
                        };
                        if dilate {
                            value |= inside && sample;
                        } else {
                            value &= sample;
                        }
                    }
                    next[at(i, j, k)] = value;
                }
            }
        }
        current = next;
    }
    current
}

/// The report as text, assumptions first, shared by every caller so they cannot drift apart.
pub fn describe(
    setup: &MachiningSetup,
    scan: Option<&AccessScan>,
    revolution: Option<&RevolutionReport>,
    findings: &[Finding],
    caveats: &[String],
) -> String {
    let mut out = String::new();
    let process = match setup.process {
        Process::Turning => "turning",
        Process::Milling3Axis => "3-axis milling",
        Process::Auto => "unresolved process",
    };
    out.push_str(&format!(
        "assumed: {process}, {}, D{:.1} end mill, R{:.1} practical minimum\n",
        setup.material.name(),
        setup.cutter_diameter,
        setup.min_corner_radius
    ));
    if let (Process::Turning, Some(r)) = (setup.process, revolution) {
        out.push_str(&format!("turned: length {:.2} mm, diameter {:.2} mm, slenderness {:.2}:1\n", r.length, r.max_diameter, r.slenderness));
    } else if let Some(scan) = scan {
        let axes: Vec<&str> = (0..6).filter(|d| scan.setup_axes[*d]).map(|d| AXIS_NAMES[d]).collect();
        out.push_str(&format!(
            "setups: {}{}   machined faces {}   unreachable {} region(s)\n",
            scan.setup_count,
            if axes.is_empty() { String::new() } else { format!(" ({})", axes.join(" ")) },
            scan.machined_faces,
            scan.unreachable_regions.len()
        ));
    }
    if findings.is_empty() {
        out.push_str("nothing to report against the rules this checks.\n");
    }
    for f in findings {
        out.push_str(&format!("{}  {}", f.severity.name(), f.rule));
        match f.unit.as_str() {
            "" => {}
            "mm" => out.push_str(&format!("  {:.2} mm, limit {:.2} mm", f.measured, f.limit)),
            "ratio" => out.push_str(&format!("  {:.2}, limit {:.2}", f.measured, f.limit)),
            "deg" => out.push_str(&format!("  {:.1} deg, limit {:.1} deg", f.measured, f.limit)),
            "mm²" => out.push_str(&format!("  {:.1} mm²", f.measured)),
            "mm³" => out.push_str(&format!("  {:.1} mm³", f.measured)),
            _ => out.push_str(&format!("  {:.0}, limit {:.0}", f.measured, f.limit)),
        }
        if let Some(at) = f.at {
            out.push_str(&format!("  at ({:.1}, {:.1}, {:.1}) mm", at.x, at.y, at.z));
        }
        out.push_str(&format!("\n        {}\n", f.detail));
    }
    for c in caveats {
        out.push_str(&format!("NOT CHECKED  {c}\n"));
    }
    out
}

#[cfg(test)]
#[path = "machining_tests.rs"]
mod tests;
