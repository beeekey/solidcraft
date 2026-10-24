//! A stress solve of a part as it will be printed.
//!
//! A part off a filament printer is a thin solid shell around a lattice, bonded layer to layer,
//! and analysing it as solid plastic overstates it, most of all across the layers, which is where
//! printed parts come apart. [`crate::print_analysis::effective_material`] turns the slicer's
//! settings into one orthotropic material; this module stacks its layers along the build
//! direction, solves, and judges the answer twice: within a layer (von Mises against the in-plane
//! strength) and between layers (tension normal to the layers against the interlayer bond). The
//! comparison runs the same part and load through every infill pattern, because the numbers only
//! mean something next to each other.
//!
//! The model is homogenised: it will not find a stress concentration at one infill strand, and the
//! pattern coefficients are representative engineering values rather than measurements of a
//! particular printer, filament and temperature.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Mesh, Vec3};

use crate::material::OrthotropicMaterial;
use crate::mesh_solve::{MeshFemError, MeshFemOptions, MeshFemResult, solve_mesh};
use crate::print_analysis::{EffectiveMaterial, INFILL_PATTERNS, InfillPattern, PrintSettings, effective_material_of};
use crate::voxelise::Region;

/// The homogenised material with its layers normal to `up`: the effective material's in-plane
/// constants across the build direction and its vertical constants along it. `None` when the
/// settings gave no material or `up` has no direction.
pub fn printed_material(effective: &EffectiveMaterial, up: Vec3) -> Option<OrthotropicMaterial> {
    let m = effective.material?;
    let up = up.normalized()?;
    // The material frame's x axis is the one the solver points along `fibre_direction`, so the
    // build direction becomes x and the two in-plane directions become y and z. Each constant
    // moves with its pair of axes: ν_x'y' is the strain across the layers per strain along the
    // build direction, which was ν_zx; the shear between the build direction and the plane was
    // g_zx; and so on round.
    Some(OrthotropicMaterial {
        ex: m.ez,
        ey: m.ex,
        ez: m.ey,
        nu_xy: m.nu_zx,
        nu_yz: m.nu_xy,
        nu_zx: m.nu_yz,
        gxy: m.gzx,
        gyz: m.gxy,
        gzx: m.gyz,
        fibre_direction: up,
    })
}

/// The stresses the layer bond sees, from a solve's stress tensors.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LayerStress {
    /// Peak tension normal to the layers (Pa); 0 when no layer is in tension.
    pub max_normal: f64,
    pub max_normal_voxel: Option<usize>,
    /// Peak shear across the layers (Pa).
    pub max_shear: f64,
    pub max_shear_voxel: Option<usize>,
    /// Dense, signed stress normal to the layers (Pa), zero outside the material.
    pub normal: Vec<f64>,
}

/// Normal and shear traction on the layer planes (normal `up`) at every material voxel of a
/// stress tensor field (six per voxel: xx, yy, zz, xy, yz, zx). `None` when the field does not
/// match the mask or `up` has no direction.
pub fn layer_stress(tensor: &[f64], solid: &[bool], up: Vec3) -> Option<LayerStress> {
    let n = up.normalized()?;
    if tensor.len() != solid.len() * 6 {
        return None;
    }
    let mut out = LayerStress { normal: vec![0.0; solid.len()], ..Default::default() };
    for (voxel, filled) in solid.iter().enumerate() {
        if !filled {
            continue;
        }
        let s = &tensor[6 * voxel..6 * voxel + 6];
        let (sxx, syy, szz, sxy, syz, szx) = (s[0], s[1], s[2], s[3], s[4], s[5]);
        let traction = Vec3::new(sxx * n.x + sxy * n.y + szx * n.z, sxy * n.x + syy * n.y + syz * n.z, szx * n.x + syz * n.y + szz * n.z);
        let normal = traction.dot(n);
        let shear = (traction - n * normal).len();
        if !normal.is_finite() || !shear.is_finite() {
            continue;
        }
        if let Some(cell) = out.normal.get_mut(voxel) {
            *cell = normal;
        }
        if normal > out.max_normal {
            out.max_normal = normal;
            out.max_normal_voxel = Some(voxel);
        }
        if shear > out.max_shear {
            out.max_shear = shear;
            out.max_shear_voxel = Some(voxel);
        }
    }
    Some(out)
}

/// Which of the two failures a printed part is nearer to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Governs {
    /// Von Mises against the in-plane strength.
    WithinLayer,
    /// Tension normal to the layers against the interlayer bond.
    BetweenLayers,
}

impl Governs {
    pub fn name(self) -> &'static str {
        match self {
            Governs::WithinLayer => "within a layer",
            Governs::BetweenLayers => "between layers",
        }
    }
}

/// The answer for a printed part.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrintedStressReport {
    pub settings: PrintSettings,
    pub up: Vec3,
    pub effective: EffectiveMaterial,
    /// The material the solve ran with, layers normal to `up`.
    pub material: OrthotropicMaterial,
    pub solve: MeshFemResult,
    pub layer: LayerStress,
    /// m, the centre of the voxel with the peak layer tension.
    pub max_layer_normal_at: Option<Vec3>,
    /// In-plane strength over the peak von Mises stress; infinite when nothing is stressed.
    pub safety_in_plane: f64,
    /// Interlayer strength over the peak tension normal to the layers; infinite when no layer is
    /// pulled apart.
    pub safety_interlayer: f64,
    pub governs: Governs,
}

fn ratio(strength: f64, stress: f64) -> f64 {
    if stress > 0.0 && stress.is_finite() { strength / stress } else { f64::INFINITY }
}

/// Holds the body (a closed mesh in mm) at `fixed`, spreads `force` (N) over `load`, and solves it
/// as printed with `settings`, the layers stacked along `up`. `resolution` as [`solve_mesh`].
#[allow(clippy::too_many_arguments)]
pub fn printed_stress(
    mesh: &Mesh,
    fixed: &Region,
    load: &Region,
    force: Vec3,
    settings: &PrintSettings,
    up: Vec3,
    resolution: usize,
    options: &MeshFemOptions,
) -> Result<PrintedStressReport, MeshFemError> {
    let up = up.normalized().ok_or(MeshFemError::Input)?;
    let effective = effective_material_of(settings, mesh);
    let material = printed_material(&effective, up).ok_or(MeshFemError::Input)?;
    let solve = solve_mesh(mesh, fixed, load, force, &material, effective.density, resolution, options)?;
    let layer = layer_stress(&solve.voxel_stress_tensor, &solve.solid, up).ok_or_else(|| MeshFemError::Solve("no stress field".into()))?;
    let max_layer_normal_at = layer.max_normal_voxel.map(|v| {
        let [nx, ny, _] = solve.grid;
        let plane = nx.max(1) * ny.max(1);
        let (i, j, k) = (v % nx.max(1), (v / nx.max(1)) % ny.max(1), v / plane);
        solve.origin + Vec3::new(i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5) * solve.h
    });
    let safety_in_plane = ratio(effective.tensile_in_plane, solve.max_von_mises);
    let safety_interlayer = ratio(effective.tensile_interlayer, layer.max_normal);
    let governs = if safety_interlayer < safety_in_plane { Governs::BetweenLayers } else { Governs::WithinLayer };
    Ok(PrintedStressReport {
        settings: settings.clone(),
        up,
        effective,
        material,
        solve,
        layer,
        max_layer_normal_at,
        safety_in_plane,
        safety_interlayer,
        governs,
    })
}

/// One pattern's row of the comparison.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InfillComparison {
    pub pattern: InfillPattern,
    pub mass_grams: f64,
    /// Pa, within a layer.
    pub young_in_plane: f64,
    /// Pa, across the layers.
    pub young_across_layers: f64,
    /// m
    pub max_displacement: f64,
    /// m, along the force, at the loaded face.
    pub load_displacement: f64,
    pub max_von_mises: f64,
    pub max_layer_normal: f64,
    pub safety_in_plane: f64,
    pub safety_interlayer: f64,
    pub governs: Governs,
    pub note: String,
}

/// The same part and load printed with every infill pattern, in the order of
/// [`INFILL_PATTERNS`]. Mass is set by the density alone, so the pattern is a free choice of
/// where the stiffness goes.
#[allow(clippy::too_many_arguments)]
pub fn compare_infill(
    mesh: &Mesh,
    fixed: &Region,
    load: &Region,
    force: Vec3,
    settings: &PrintSettings,
    up: Vec3,
    resolution: usize,
    options: &MeshFemOptions,
) -> Result<Vec<InfillComparison>, MeshFemError> {
    let mut rows = Vec::with_capacity(INFILL_PATTERNS.len());
    for (pattern, _) in INFILL_PATTERNS {
        let settings = PrintSettings { pattern, ..settings.clone() };
        let r = printed_stress(mesh, fixed, load, force, &settings, up, resolution, options)?;
        rows.push(InfillComparison {
            pattern,
            mass_grams: r.effective.mass_grams,
            young_in_plane: r.material.ey,
            young_across_layers: r.material.ex,
            max_displacement: r.solve.max_displacement,
            load_displacement: r.solve.load_displacement,
            max_von_mises: r.solve.max_von_mises,
            max_layer_normal: r.layer.max_normal,
            safety_in_plane: r.safety_in_plane,
            safety_interlayer: r.safety_interlayer,
            governs: r.governs,
            note: r.effective.note,
        });
    }
    Ok(rows)
}
