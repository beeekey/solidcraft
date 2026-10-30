//! Can this part be printed, and what will it cost: measured from the triangle mesh.
//!
//! Overhangs steeper than the critical angle need support; the part must be watertight and fit
//! the build volume; support is bounded above by the prisms under the overhanging triangles;
//! walls are measured by firing a ray inward from every triangle; a flat underside anchored on
//! two sides within a span prints as a bridge. The orientation advisor runs the same measures in
//! the six axis directions and ranks them. Everything is in millimetres, like the mesh.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Mesh, MeshIndex, Vec3};

use crate::material::OrthotropicMaterial;
use crate::voxel_fem::{LoadCase, VoxelFemProblem, solve_voxels};
use crate::voxelise::{MM, Region, VoxelGrid, voxelise};

/// What the printability measures came to.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PrintReport {
    /// Every edge of the mesh is shared by two triangles: a closed skin.
    pub watertight: bool,
    /// Within the printer's volume in some axis order.
    pub fits_build_volume: bool,
    /// mm² of down-facing area past the critical angle, not seated on the plate.
    pub overhang_area_mm2: f64,
    /// Projected footprint of that overhang, mm².
    pub support_contact_area_mm2: f64,
    pub layer_count: usize,
    /// Extent along the build direction, mm.
    pub height_mm: f64,
    /// mm³
    pub volume_mm3: f64,
}

/// One triangle as the measures see it.
struct Tri {
    a: Vec3,
    b: Vec3,
    c: Vec3,
    area: f64,
    /// Outward normal's component along "up".
    alignment: f64,
    high: f64,
    centroid_station: f64,
}

fn triangles(mesh: &Mesh, up: Vec3) -> (Vec<Tri>, f64) {
    let mut out = Vec::with_capacity(mesh.triangles.len());
    let mut lowest = f64::INFINITY;
    for t in &mesh.triangles {
        let Some([a, b, c]) = mesh.tri(t) else { continue };
        let cross = (b - a).cross(c - a);
        let double_area = cross.len();
        if !(double_area > 1e-18) || !double_area.is_finite() {
            continue;
        }
        let normal = cross / double_area;
        let (sa, sb, sc) = (a.dot(up), b.dot(up), c.dot(up));
        let low = sa.min(sb).min(sc);
        let high = sa.max(sb).max(sc);
        lowest = lowest.min(low);
        out.push(Tri { a, b, c, area: double_area / 2.0, alignment: normal.dot(up), high, centroid_station: (sa + sb + sc) / 3.0 });
    }
    (out, lowest)
}

/// The volume a closed mesh encloses (mm³), by the divergence theorem over its triangles. Works
/// on meshes whose vertices are not welded, which `Mesh::mass_properties` refuses.
pub fn volume_of(mesh: &Mesh) -> f64 {
    let v6: f64 = mesh.triangles.iter().filter_map(|t| mesh.tri(t)).map(|[a, b, c]| a.dot(b.cross(c))).sum();
    if v6.is_finite() { v6.abs() / 6.0 } else { 0.0 }
}

/// Whether every edge is shared by an even number of triangles (positions compared to a
/// micron, so faces that meet along a seam count as joined).
pub fn watertight(mesh: &Mesh) -> bool {
    use std::collections::HashMap;
    if mesh.triangles.is_empty() {
        return false;
    }
    let key = |p: Vec3| ((p.x * 1e6).round() as i64, (p.y * 1e6).round() as i64, (p.z * 1e6).round() as i64);
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), i32> = HashMap::new();
    for t in &mesh.triangles {
        let Some(ps) = mesh.tri(t) else { return false };
        let ks = [key(ps[0]), key(ps[1]), key(ps[2])];
        for e in 0..3 {
            let (p, q) = (ks[e], ks[(e + 1) % 3]);
            let edge = if p <= q { (p, q) } else { (q, p) };
            *edges.entry(edge).or_insert(0) += if p <= q { 1 } else { -1 };
        }
    }
    // A closed, consistently oriented skin traverses every edge once each way.
    edges.values().all(|n| *n == 0)
}

/// Analyses the part printed along `build_direction`. `critical_angle` is the overhang threshold
/// from the horizontal in radians (45° typical: faces whose outward normal points further down
/// than that need support). `build_volume` is the printer's volume in mm.
pub fn analyze(mesh: &Mesh, build_direction: Vec3, critical_angle: f64, build_volume: Vec3, layer_height: f64) -> PrintReport {
    let mut report = PrintReport::default();
    let Some(up) = build_direction.normalized() else { return report };
    if mesh.triangles.is_empty() {
        return report;
    }
    report.watertight = watertight(mesh);
    let (tris, lowest) = triangles(mesh, up);
    if tris.is_empty() {
        return report;
    }
    let highest = tris.iter().map(|t| t.high).fold(f64::NEG_INFINITY, f64::max);
    // A triangle needs support when its outward normal points down steeper than the critical
    // angle, except triangles seated on the plate.
    let threshold = -critical_angle.sin();
    for t in &tris {
        if t.alignment >= threshold - 1e-9 {
            continue;
        }
        let seated = t.alignment < -0.9999 && t.high < lowest + 1e-6;
        if seated {
            continue;
        }
        report.overhang_area_mm2 += t.area;
        report.support_contact_area_mm2 += t.area * t.alignment.abs();
    }
    let bounds = mesh.bounds();
    let size = bounds.size();
    let mut sizes = [size.x, size.y, size.z];
    let mut printer = [build_volume.x, build_volume.y, build_volume.z];
    sizes.sort_by(f64::total_cmp);
    printer.sort_by(f64::total_cmp);
    report.fits_build_volume = sizes.iter().zip(&printer).all(|(s, p)| *s <= *p);
    report.height_mm = if highest > lowest { highest - lowest } else { 0.0 };
    report.layer_count = if layer_height > 1e-9 { (report.height_mm / layer_height - 1e-9).ceil().max(0.0) as usize } else { 0 };
    report.volume_mm3 = volume_of(mesh);
    report
}

/// One candidate build orientation with its report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OrientationOption {
    pub build_direction: Vec3,
    pub name: &'static str,
    pub report: PrintReport,
}

/// The six axis-aligned build orientations, best first: fits the printer beats not, then the
/// least support-needing overhang, then the least height.
pub fn advise_orientation(mesh: &Mesh, critical_angle: f64, build_volume: Vec3, layer_height: f64) -> Vec<OrientationOption> {
    let axes: [(Vec3, &'static str); 6] = [(Vec3::Z, "+Z"), (-Vec3::Z, "-Z"), (Vec3::X, "+X"), (-Vec3::X, "-X"), (Vec3::Y, "+Y"), (-Vec3::Y, "-Y")];
    let mut options: Vec<OrientationOption> = axes
        .iter()
        .map(|(d, name)| OrientationOption { build_direction: *d, name, report: analyze(mesh, *d, critical_angle, build_volume, layer_height) })
        .collect();
    options.sort_by(|a, b| {
        b.report.fits_build_volume.cmp(&a.report.fits_build_volume).then_with(|| {
            if (a.report.overhang_area_mm2 - b.report.overhang_area_mm2).abs() > 1e-9 {
                a.report.overhang_area_mm2.total_cmp(&b.report.overhang_area_mm2)
            } else {
                a.report.height_mm.total_cmp(&b.report.height_mm)
            }
        })
    });
    options
}

/// Material and time, measured from the mesh. `support_volume_mm3` is an upper bound: each
/// support-needing triangle contributes the prism straight down to the plate.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PrintEstimate {
    pub material_volume_mm3: f64,
    pub support_volume_mm3: f64,
    /// Part plus support at the given density.
    pub material_grams: f64,
    pub time_seconds: f64,
    pub layer_count: usize,
}

/// `density` kg/m³ (PLA ≈ 1240), `flow_rate` mm³/s (8 typical), `layer_overhead` seconds per
/// layer for travel and retraction (≈ 1).
pub fn estimate(
    mesh: &Mesh,
    build_direction: Vec3,
    critical_angle: f64,
    layer_height: f64,
    density: f64,
    flow_rate: f64,
    layer_overhead: f64,
) -> PrintEstimate {
    let mut out = PrintEstimate::default();
    let Some(up) = build_direction.normalized() else { return out };
    let report = analyze(mesh, build_direction, critical_angle, Vec3::new(1e9, 1e9, 1e9), layer_height);
    out.material_volume_mm3 = report.volume_mm3;
    out.layer_count = report.layer_count;
    let (tris, lowest) = triangles(mesh, up);
    let threshold = -critical_angle.sin();
    for t in &tris {
        if t.alignment >= threshold - 1e-9 || (t.alignment < -0.9999 && t.high < lowest + 1e-6) {
            continue;
        }
        out.support_volume_mm3 += t.area * (-t.alignment) * (t.centroid_station - lowest).max(0.0);
    }
    let total_mm3 = out.material_volume_mm3 + out.support_volume_mm3;
    out.material_grams = total_mm3 * 1e-9 * density * 1000.0;
    out.time_seconds = if flow_rate > 0.0 { total_mm3 / flow_rate } else { 0.0 } + out.layer_count as f64 * layer_overhead;
    out
}

/// The thinnest wall of the part (mm) and where it is: from every triangle's centroid a ray is
/// cast along the inward normal, and the nearest exit is that wall's thickness. `None` when no
/// ray crosses the part.
pub fn thinnest_wall(mesh: &Mesh) -> Option<(f64, Vec3)> {
    let index = MeshIndex::new(mesh);
    let mut best: Option<(f64, Vec3)> = None;
    for t in &mesh.triangles {
        let Some([a, b, c]) = mesh.tri(t) else { continue };
        let Some(normal) = (b - a).cross(c - a).normalized() else { continue };
        let centroid = (a + b + c) / 3.0;
        let inward = -normal;
        let origin = centroid + inward * 1e-6;
        if let Some((distance, _)) = index.raycast(mesh, origin, inward)
            && distance > 1e-9
        {
            let thickness = distance + 1e-6;
            if best.is_none_or(|(d, _)| thickness < d) {
                best = Some((thickness, centroid));
            }
        }
    }
    best
}

/// Bridges: flat down-facing regions anchored on two opposite sides within `max_span` print
/// without support.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct BridgeReport {
    /// mm² printable as bridges.
    pub bridged_area_mm2: f64,
    /// mm, the longest span actually bridged.
    pub longest_span_mm: f64,
}

pub fn bridges(mesh: &Mesh, build_direction: Vec3, max_span: f64) -> BridgeReport {
    let mut report = BridgeReport::default();
    let Some(up) = build_direction.normalized() else { return report };
    let u_axis = up.cross(Vec3::X).normalized().or_else(|| up.cross(Vec3::Y).normalized()).unwrap_or(Vec3::Y);
    let v_axis = up.cross(u_axis).normalized().unwrap_or(Vec3::Z);
    let (tris, lowest) = triangles(mesh, up);
    let index = MeshIndex::new(mesh);
    // A ray that enters material: its first hit is a front-facing triangle.
    let enter = |origin: Vec3, direction: Vec3| -> Option<f64> {
        let (distance, hit) = index.raycast(mesh, origin, direction)?;
        let [a, b, c] = mesh.tri(mesh.triangles.get(hit)?)?;
        let normal = (b - a).cross(c - a);
        (normal.dot(direction) < 0.0 && distance > 1e-9).then_some(distance)
    };
    for t in &tris {
        if t.alignment > -0.9999 || t.high < lowest + 1e-6 {
            continue; // only flat undersides not seated on the plate
        }
        let centroid = (t.a + t.b + t.c) / 3.0;
        let origin = centroid - up * 1e-3;
        let mut best_span = f64::INFINITY;
        for angle in [0.0, std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_2, 3.0 * std::f64::consts::FRAC_PI_4] {
            let direction = u_axis * angle.cos() + v_axis * angle.sin();
            if let (Some(forward), Some(backward)) = (enter(origin, direction), enter(origin, -direction)) {
                best_span = best_span.min(forward + backward);
            }
        }
        if best_span <= max_span {
            report.bridged_area_mm2 += t.area;
            report.longest_span_mm = report.longest_span_mm.max(best_span);
        }
    }
    report
}

// --- what a printed part is actually made of ------------------------------------------------

/// The infill patterns a slicer offers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InfillPattern {
    Grid,
    Lines,
    Rectilinear,
    Triangles,
    TriHexagon,
    Cubic,
    Gyroid,
    Honeycomb,
    Concentric,
    CrossHatch,
    Lightning,
}

pub const INFILL_PATTERNS: &[(InfillPattern, &str)] = &[
    (InfillPattern::Grid, "Grid"),
    (InfillPattern::Lines, "Lines"),
    (InfillPattern::Rectilinear, "Rectilinear"),
    (InfillPattern::Triangles, "Triangles"),
    (InfillPattern::TriHexagon, "Tri-hexagon"),
    (InfillPattern::Cubic, "Cubic"),
    (InfillPattern::Gyroid, "Gyroid"),
    (InfillPattern::Honeycomb, "Honeycomb"),
    (InfillPattern::Concentric, "Concentric"),
    (InfillPattern::CrossHatch, "Cross-hatch"),
    (InfillPattern::Lightning, "Lightning"),
];

impl InfillPattern {
    pub fn from_name(name: &str) -> Option<InfillPattern> {
        INFILL_PATTERNS.iter().find(|(_, n)| n.eq_ignore_ascii_case(name.trim())).map(|(p, _)| *p)
    }
    pub fn name(self) -> &'static str {
        INFILL_PATTERNS.iter().find(|(p, _)| *p == self).map(|(_, n)| *n).unwrap_or("Grid")
    }
}

/// A printed filament as a solid: in-plane values, so a load pulling layers apart is flattered
/// until the layer bond below is applied.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Filament {
    pub name: &'static str,
    /// kg/m³
    pub density: f64,
    /// Pa
    pub young_modulus: f64,
    pub poisson_ratio: f64,
    /// Pa
    pub yield_strength: f64,
}

pub const FILAMENTS: [Filament; 6] = [
    Filament { name: "PLA", density: 1240.0, young_modulus: 3.4e9, poisson_ratio: 0.36, yield_strength: 50e6 },
    Filament { name: "PLA+", density: 1250.0, young_modulus: 3.0e9, poisson_ratio: 0.36, yield_strength: 45e6 },
    Filament { name: "PETG", density: 1270.0, young_modulus: 1.9e9, poisson_ratio: 0.37, yield_strength: 45e6 },
    Filament { name: "ABS", density: 1040.0, young_modulus: 2.0e9, poisson_ratio: 0.35, yield_strength: 32e6 },
    Filament { name: "ASA", density: 1070.0, young_modulus: 2.0e9, poisson_ratio: 0.35, yield_strength: 35e6 },
    Filament { name: "TPU 95A", density: 1210.0, young_modulus: 0.026e9, poisson_ratio: 0.48, yield_strength: 7e6 },
];

/// The named filament (case-insensitive, "(printed)" suffix ignored), or PLA.
pub fn filament_named(name: &str) -> Filament {
    let wanted = name.trim().trim_end_matches("(printed)").trim();
    FILAMENTS.iter().find(|f| f.name.eq_ignore_ascii_case(wanted)).copied().unwrap_or(FILAMENTS[0])
}

/// The settings a slicer would be given.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PrintSettings {
    pub material: String,
    pub pattern: InfillPattern,
    /// Fraction, 0..1.
    pub infill_density: f64,
    /// Perimeters.
    pub wall_loops: u32,
    /// mm
    pub line_width: f64,
    /// mm
    pub layer_height: f64,
    pub top_bottom_layers: u32,
}

impl Default for PrintSettings {
    fn default() -> Self {
        PrintSettings {
            material: "PLA".into(),
            pattern: InfillPattern::Grid,
            infill_density: 0.20,
            wall_loops: 2,
            line_width: 0.42,
            layer_height: 0.20,
            top_bottom_layers: 4,
        }
    }
}

/// What those settings amount to, mechanically.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EffectiveMaterial {
    pub material: Option<OrthotropicMaterial>,
    /// Pa, within a layer.
    pub tensile_in_plane: f64,
    /// Pa, across layers: the usual failure.
    pub tensile_interlayer: f64,
    /// kg/m³, effective.
    pub density: f64,
    /// Of the part's volume that is plastic.
    pub solid_fraction: f64,
    /// How much of that is walls and skins.
    pub shell_fraction: f64,
    pub mass_grams: f64,
    pub note: String,
}

/// How stiff an infill is relative to solid material, per direction, at a given density. Where
/// the lattice carries load by stretching its members, stiffness rises in proportion to density;
/// where it bends them, closer to the square. Patterns whose walls run continuously up the part
/// carry Z far better than sideways load.
struct PatternResponse {
    in_plane: f64,
    vertical: f64,
    exponent: f64,
    shear: f64,
    note: &'static str,
}

fn response_of(pattern: InfillPattern) -> PatternResponse {
    let r = |in_plane, vertical, exponent, shear, note| PatternResponse { in_plane, vertical, exponent, shear, note };
    match pattern {
        InfillPattern::Grid => r(0.50, 1.00, 1.0, 0.25, "grid walls run straight up the part, so it is much stiffer in Z than sideways"),
        InfillPattern::Lines => r(0.45, 0.85, 1.0, 0.15, "lines alternate direction each layer and do not bond into columns, so shear is weak"),
        InfillPattern::Rectilinear => r(0.48, 0.90, 1.0, 0.18, ""),
        InfillPattern::Triangles => r(0.40, 0.95, 1.0, 0.35, "three directions carry shear better than a grid"),
        InfillPattern::TriHexagon => r(0.42, 0.95, 1.0, 0.38, ""),
        InfillPattern::Cubic => {
            r(0.35, 0.35, 1.0, 0.30, "cubic spreads stiffness over all three axes, so it is the most even but never the stiffest in any one")
        }
        InfillPattern::Gyroid => r(
            0.55,
            0.55,
            1.5,
            0.45,
            "gyroid is close to isotropic and the best of these in shear; its walls bend rather than stretch, so stiffness falls away faster as density drops",
        ),
        InfillPattern::Honeycomb => r(0.30, 1.00, 1.5, 0.25, "honeycomb cells are columns: excellent in Z, modest sideways"),
        InfillPattern::Concentric => {
            r(0.60, 0.40, 1.0, 0.20, "concentric follows the perimeter, so it stiffens the part in plane and does little for Z")
        }
        InfillPattern::CrossHatch => r(0.45, 0.90, 1.0, 0.32, ""),
        InfillPattern::Lightning => {
            r(0.10, 0.30, 2.0, 0.08, "lightning exists to hold up the top skin, not to carry load; treat any strength number from it with suspicion")
        }
    }
}

/// The effective material for a part of the given volume (mm³) and surface area (mm²). Walls are
/// a fixed thickness, so on a small part they are most of the material.
pub fn effective_material(settings: &PrintSettings, volume_mm3: f64, surface_area_mm2: f64) -> EffectiveMaterial {
    let mut effective = EffectiveMaterial::default();
    if !(volume_mm3 > 0.0) || !volume_mm3.is_finite() || !surface_area_mm2.is_finite() {
        return effective;
    }
    let filament = filament_named(&settings.material);
    let (solid_e, solid_strength, solid_density) = (filament.young_modulus, filament.yield_strength, filament.density);
    let wall_thickness = (settings.wall_loops as f64 * settings.line_width).max(0.0);
    let skin_thickness = (settings.top_bottom_layers as f64 * settings.layer_height).max(0.0);
    // Surface area times wall thickness over-counts at edges and corners, so it is clamped.
    let shell_volume = surface_area_mm2 * wall_thickness.max(skin_thickness) * 0.5 + surface_area_mm2 * wall_thickness.min(skin_thickness) * 0.5;
    let shell_fraction = (shell_volume / volume_mm3).clamp(0.0, 1.0);
    let density = settings.infill_density.clamp(0.0, 1.0);
    let infill_fraction = (1.0 - shell_fraction) * density;
    effective.shell_fraction = shell_fraction;
    effective.solid_fraction = (shell_fraction + infill_fraction).clamp(0.0, 1.0);
    effective.density = solid_density * effective.solid_fraction;
    effective.mass_grams = effective.density * volume_mm3 * 1e-9 * 1000.0;

    let response = response_of(settings.pattern);
    let lattice = density.powf(response.exponent);
    let in_plane = shell_fraction + (1.0 - shell_fraction) * response.in_plane * lattice;
    let vertical = shell_fraction + (1.0 - shell_fraction) * response.vertical * lattice;
    let shear = shell_fraction + (1.0 - shell_fraction) * response.shear * lattice;
    // The layer bond: strands welding to their neighbours, around half of in-plane across a wide
    // published spread. Layer height is given only a small influence; nozzle temperature, which
    // nothing here knows, dominates.
    let reference = 0.20;
    let bond = (0.55 + 0.06 * (settings.layer_height.max(0.05) / reference).ln()).clamp(0.45, 0.65);
    let mut material = OrthotropicMaterial::isotropic(solid_e, filament.poisson_ratio);
    material.ex = (solid_e * in_plane).max(1.0);
    material.ey = material.ex;
    material.ez = (solid_e * vertical * bond).max(1.0);
    material.nu_xy = filament.poisson_ratio;
    // Loaded within a layer, the part contracts across the layers a little less than solid
    // plastic would; loaded across them, reciprocity (ν_zx/E_z = ν_yz/E_y) fixes the other ratio,
    // which keeps the material transversely isotropic about the build direction.
    material.nu_yz = filament.poisson_ratio * 0.8;
    material.nu_zx = material.nu_yz * material.ez / material.ey;
    material.gxy = (solid_e * shear / (2.0 * (1.0 + material.nu_xy))).max(1.0);
    material.gyz = (material.gxy * bond).max(1.0);
    material.gzx = material.gyz;
    effective.material = Some(material);
    effective.tensile_in_plane = solid_strength * in_plane;
    effective.tensile_interlayer = solid_strength * vertical * bond;

    let mut note = response.note.to_string();
    let mut add = |text: &str| {
        if !note.is_empty() {
            note.push(' ');
        }
        note.push_str(text);
    };
    if settings.wall_loops <= 1 {
        add("a single wall carries most of the load on a thin part and is the first thing to increase");
    }
    if density >= 0.99 {
        add("at full density the pattern hardly matters; what is left is the layer bond");
    }
    add(
        "interlayer strength is modelled at roughly half of in-plane, which is the published band; it depends on nozzle temperature more than on anything set here",
    );
    effective.note = note;
    effective
}

/// The effective material of a mesh (mm).
pub fn effective_material_of(settings: &PrintSettings, mesh: &Mesh) -> EffectiveMaterial {
    let volume = volume_of(mesh);
    let area: f64 = mesh.triangles.iter().filter_map(|t| mesh.tri(t)).map(|[a, b, c]| (b - a).cross(c - a).len() / 2.0).sum();
    effective_material(settings, volume, area)
}

// --- topology optimisation: keep the material that works ------------------------------------

/// The result of the stress-driven material removal.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TopologyMask {
    pub grid: [usize; 3],
    /// m
    pub h: f64,
    pub origin: Vec3,
    pub solid: Vec<bool>,
    /// The cells kept.
    pub kept: Vec<bool>,
    pub voxels_total: usize,
    pub voxels_kept: usize,
    pub kept_fraction: f64,
    pub advice: String,
}

/// The most cells the optimiser may work on.
pub const MAX_OPTIMISE_CELLS: usize = 400_000;

/// Voxelises the part (mm), solves the voxel FEM with unit modulus (the ranking is scale-free),
/// and keeps the top `target_fraction` of each cross-section along the load path by strain
/// energy, always keeping the fixed and loaded cells and the path between them connected.
/// Floating islands are dropped. The kernel-side intersection that turns the mask into a solid
/// is not here.
pub fn topology_optimize(mesh: &Mesh, fixed: &Region, load: &Region, force: Vec3, target_fraction: f64, resolution: usize) -> Option<TopologyMask> {
    if !(target_fraction > 0.0 && target_fraction < 1.0) || !force.is_finite() || mesh.triangles.is_empty() {
        return None;
    }
    let grid: VoxelGrid = voxelise(mesh, resolution.clamp(6, 48), MAX_OPTIMISE_CELLS)?;
    let fixed_cells = grid.region(mesh, fixed);
    let load_cells = grid.region(mesh, load);
    if fixed_cells.is_empty() || load_cells.is_empty() {
        return None;
    }
    let mut problem = VoxelFemProblem::new(grid.nx, grid.ny, grid.nz, grid.h, OrthotropicMaterial::isotropic(1.0, 0.3));
    problem.origin = grid.origin;
    problem.solid = grid.solid.clone();
    for &v in &fixed_cells {
        problem.held[v] = true;
    }
    problem.cases.push(LoadCase { voxels: load_cells.clone(), force, name: "load".into() });
    problem.max_iterations = 2000;
    let fem = solve_voxels(&problem);
    if !fem.solved || fem.voxel_energy.len() != grid.solid.len() {
        return None;
    }
    let (nx, ny, nz) = (grid.nx, grid.ny, grid.nz);
    let energy = &fem.voxel_energy;
    let solid_cells: Vec<usize> = (0..grid.solid.len()).filter(|v| grid.solid[*v]).collect();
    // Slabs along the dominant direction from the fixed cells to the loaded ones: a global
    // threshold keeps a stump at the support and strands the load; per-slab keeping removes the
    // low-stress core of every section while the path stays continuous.
    let centre = |cells: &[usize]| cells.iter().map(|v| grid.centre(*v)).fold(Vec3::ZERO, |s, p| s + p) / cells.len().max(1) as f64;
    let path = centre(&load_cells) - centre(&fixed_cells);
    let abs = [path.x.abs(), path.y.abs(), path.z.abs()];
    let slab_axis = if abs[0] >= abs[1] && abs[0] >= abs[2] {
        0
    } else if abs[1] >= abs[2] {
        1
    } else {
        2
    };
    let slab_count = [nx, ny, nz][slab_axis];
    let mut slabs: Vec<Vec<usize>> = vec![Vec::new(); slab_count];
    for &v in &solid_cells {
        let (i, j, k) = grid.ijk(v);
        let s = [i, j, k][slab_axis];
        if let Some(slab) = slabs.get_mut(s) {
            slab.push(v);
        }
    }
    for slab in &mut slabs {
        slab.sort_by(|a, b| energy[*b].total_cmp(&energy[*a]));
    }
    let mut keep_fraction = target_fraction;
    let mut kept = vec![false; grid.solid.len()];
    let rebuild = |keep_fraction: f64, kept: &mut Vec<bool>| {
        kept.fill(false);
        for slab in &slabs {
            let target = (keep_fraction * slab.len() as f64).ceil() as usize;
            for &v in slab.iter().take(target) {
                kept[v] = true;
            }
        }
        for &v in fixed_cells.iter().chain(&load_cells) {
            kept[v] = true;
        }
    };
    let connected = |kept: &[bool]| -> (bool, Vec<bool>) {
        let mut seen = vec![false; kept.len()];
        let mut frontier: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
        for &v in &fixed_cells {
            if kept[v] && !seen[v] {
                seen[v] = true;
                frontier.push_back(v);
            }
        }
        while let Some(v) = frontier.pop_front() {
            let (i, j, k) = grid.ijk(v);
            let (i, j, k) = (i as i64, j as i64, k as i64);
            for (di, dj, dk) in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)] {
                let (ii, jj, kk) = (i + di, j + dj, k + dk);
                if ii < 0 || jj < 0 || kk < 0 || ii as usize >= nx || jj as usize >= ny || kk as usize >= nz {
                    continue;
                }
                let n = grid.index(ii as usize, jj as usize, kk as usize);
                if kept[n] && !seen[n] {
                    seen[n] = true;
                    frontier.push_back(n);
                }
            }
        }
        (load_cells.iter().all(|v| seen[*v]), seen)
    };
    let reachable;
    loop {
        rebuild(keep_fraction, &mut kept);
        let (ok, seen) = connected(&kept);
        if ok {
            reachable = seen;
            break;
        }
        keep_fraction += 0.05;
        if keep_fraction >= 1.0 {
            rebuild(1.0, &mut kept);
            reachable = connected(&kept).1;
            break;
        }
    }
    let mut voxels_kept = 0;
    for v in 0..kept.len() {
        if kept[v] && !reachable[v] {
            kept[v] = false;
        }
        if kept[v] {
            voxels_kept += 1;
        }
    }
    let total = solid_cells.len();
    let advice = if keep_fraction > target_fraction + 1e-9 {
        format!(
            "the load path needed {:.0}% of the material to stay connected, more than the {:.0}% asked for",
            keep_fraction * 100.0,
            target_fraction * 100.0
        )
    } else {
        String::new()
    };
    let _ = reachable;
    Some(TopologyMask {
        grid: [nx, ny, nz],
        h: grid.h,
        origin: grid.origin,
        solid: grid.solid,
        kept,
        voxels_total: total,
        voxels_kept,
        kept_fraction: if total > 0 { voxels_kept as f64 / total as f64 } else { 0.0 },
        advice,
    })
}

/// Millimetres of the cell size, for callers that report in mm.
pub fn cell_mm(mask: &TopologyMask) -> f64 {
    mask.h * MM
}

#[cfg(test)]
#[path = "print_analysis_tests.rs"]
mod tests;
