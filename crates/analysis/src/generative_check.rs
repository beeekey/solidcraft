//! The checks the search itself cannot make: a proper mesh, the buckling a slender strut does
//! long before it yields, a fatigue life if the load repeats, and whether the thing can be made.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Mesh, Vec3};

use super::{GenerativeProblem, build_design_grid, fatigue_damage_of, fem_problem};
use crate::buckling::buckling;
use crate::makeability::{MakeabilityRules, MakeabilityScan, scan_makeability};
use crate::material::OrthotropicMaterial;
use crate::voxel_fem::solve_voxels;
use crate::voxelise::MM;

/// How the surface was left, which is most of what decides a fatigue life.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SurfaceFinish {
    Polished,
    Ground,
    Machined,
    HotRolled,
    AsForged,
    AsPrinted,
}

impl SurfaceFinish {
    pub const NAMES: [&'static str; 6] = ["Polished", "Ground", "Machined", "Hot-rolled", "As forged", "As printed"];

    pub fn from_name(name: &str) -> SurfaceFinish {
        match name.trim().to_ascii_lowercase().as_str() {
            "polished" => SurfaceFinish::Polished,
            "ground" => SurfaceFinish::Ground,
            "hot-rolled" | "hot rolled" => SurfaceFinish::HotRolled,
            "as forged" | "forged" => SurfaceFinish::AsForged,
            "as printed" | "printed" => SurfaceFinish::AsPrinted,
            _ => SurfaceFinish::Machined,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            SurfaceFinish::Polished => "Polished",
            SurfaceFinish::Ground => "Ground",
            SurfaceFinish::Machined => "Machined",
            SurfaceFinish::HotRolled => "Hot-rolled",
            SurfaceFinish::AsForged => "As forged",
            SurfaceFinish::AsPrinted => "As printed",
        }
    }
}

/// Marin surface factor ka = a · Sut^b with the published coefficients per finish (Sut in Pa).
pub fn surface_factor(finish: SurfaceFinish, ultimate_strength: f64) -> f64 {
    if !(ultimate_strength > 0.0) {
        return 1.0;
    }
    let megapascals = ultimate_strength / 1e6;
    let (a, b) = match finish {
        SurfaceFinish::Polished => return 1.0,
        SurfaceFinish::Ground => (1.58, -0.085),
        SurfaceFinish::Machined => (4.51, -0.265),
        SurfaceFinish::HotRolled => (57.7, -0.718),
        // A printed skin has no published class; the forged coefficients stand in for it.
        SurfaceFinish::AsForged | SurfaceFinish::AsPrinted => (272.0, -0.995),
    };
    (a * megapascals.powf(b)).clamp(0.05, 1.0)
}

/// Marin size factor kb, from the section the bending stress runs through (m).
pub fn size_factor(section_metres: f64) -> f64 {
    let mm = section_metres * 1000.0;
    if !(mm > 0.0) || mm <= 2.79 {
        1.0
    } else if mm <= 51.0 {
        1.24 * mm.powf(-0.107)
    } else if mm <= 254.0 {
        1.51 * mm.powf(-0.157)
    } else {
        1.51 * 254f64.powf(-0.157)
    }
}

/// Fatigue life by Basquin's S-N line with a Goodman mean-stress correction: the cycles this
/// stress amplitude survives. 1e30 below the knee, 0 when it fails on the first cycle.
pub fn cycles_to_failure(stress_amplitude: f64, mean_stress: f64, endurance_limit: f64, ultimate_strength: f64) -> f64 {
    if !(stress_amplitude > 0.0) || !(endurance_limit > 0.0) || !(ultimate_strength > 0.0) {
        return 0.0;
    }
    let mean = mean_stress.max(0.0);
    if mean >= ultimate_strength {
        return 0.0;
    }
    let equivalent = stress_amplitude / (1.0 - mean / ultimate_strength);
    if equivalent <= endurance_limit {
        return 1e30;
    }
    let high = 0.9 * ultimate_strength;
    if equivalent >= high {
        return 1000.0;
    }
    let b = -(high / endurance_limit).log10() / 3.0;
    let a = high / 1000f64.powf(b);
    (equivalent / a).powf(1.0 / b)
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CheckOptions {
    /// The mesh the verification runs on.
    pub resolution: usize,
    /// Loads that repeat rather than sit there.
    pub cyclic_loading: bool,
    /// Pa; 0 takes the usual first estimates from the yield strength.
    pub endurance_limit: f64,
    pub ultimate_strength: f64,
    pub finish: SurfaceFinish,
    /// Printed flat on the bed, along this axis, with this overhang limit.
    pub print_axis: usize,
    pub overhang_degrees: f64,
}

impl Default for CheckOptions {
    fn default() -> Self {
        CheckOptions {
            resolution: 44,
            cyclic_loading: false,
            endurance_limit: 0.0,
            ultimate_strength: 0.0,
            finish: SurfaceFinish::Machined,
            print_axis: 2,
            overhang_degrees: 45.0,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DesignCheck {
    pub solved: bool,
    pub buckling_from_eigen_solve: bool,
    pub cycles_to_failure: f64,
    pub fatigue_damage: f64,
    pub blocks_to_failure: f64,
    pub makeability: MakeabilityScan,
    pub resolution: usize,
    pub elements_through_thinnest: usize,
    /// Pa
    pub max_stress: f64,
    pub safety_factor: f64,
    /// m
    pub max_displacement: f64,
    /// kg, from the mesh
    pub mass: f64,
    pub buckling_assessed: bool,
    pub buckling_factor: f64,
    pub buckling_note: String,
    pub fatigue_assessed: bool,
    pub fatigue_factor: f64,
    pub fatigue_note: String,
    /// m² needing support
    pub overhang_area: f64,
    /// m²
    pub surface_area: f64,
    /// m
    pub thinnest_feature: f64,
    pub manufacturing_note: String,
    pub verdict: String,
    pub advice: String,
}

/// Verifies a built body (a closed mesh in mm) properly, on a finer mesh than the search used.
pub fn check_design(problem: &GenerativeProblem, body: &Mesh, options: &CheckOptions) -> DesignCheck {
    let mut check = DesignCheck::default();
    if body.triangles.is_empty() {
        check.advice = "there is no body to check".into();
        return check;
    }
    // The signed-volume sum works on a mesh whose facets share no vertices (a mesh body's
    // tessellation comes back that way), where the welded-edge check behind `mass_properties`
    // would refuse it.
    let volume_m3 = crate::print_analysis::volume_of(body).abs() / (MM * MM * MM);
    check.mass = volume_m3 * problem.density;
    check.surface_area = body.triangles.iter().filter_map(|t| body.tri(t)).map(|[a, b, c]| 0.5 * (b - a).cross(c - a).len()).sum::<f64>() / (MM * MM);

    let resolution = options.resolution.clamp(16, 96);
    let grid = build_design_grid(problem, Some(resolution));
    if grid.nx == 0 {
        check.advice = "the design space is empty".into();
        return check;
    }
    let count = grid.count();
    // A cell counts as material when its centre is inside, or when at least half of eight
    // sub-centres are: a strut thinner than a cell is then still a chain of cells joined face to
    // face, not corner to corner, which linear elements would read as a notch.
    let index = body.inside_index();
    let quarter = grid.h * 0.25;
    let mut solid: Vec<bool> = (0..count)
        .map(|v| {
            let c = grid.centre(v);
            if index.contains(c * MM) {
                return true;
            }
            let mut inside = 0;
            for dz in [-1.0, 1.0] {
                for dy in [-1.0, 1.0] {
                    for dx in [-1.0, 1.0] {
                        if index.contains((c + Vec3::new(dx, dy, dz) * quarter) * MM) {
                            inside += 1;
                        }
                    }
                }
            }
            inside >= 4
        })
        .collect();
    super::field::fill_diagonal_contacts(&mut solid, grid.nx, grid.ny, grid.nz);
    for voxels in grid.mount_voxels.iter().chain(grid.load_voxels.iter()) {
        for &v in voxels {
            if v < count {
                solid[v] = true;
            }
        }
    }
    let material = OrthotropicMaterial::isotropic(problem.youngs_modulus, problem.poisson_ratio);
    let fem = fem_problem(problem, &grid, solid.clone(), material, 2000, 1e-7);
    let biggest_force = problem.loads.iter().map(|l| l.force.len()).fold(0.0, f64::max);
    let any_gravity = problem.gravity.len() > 0.0;
    let mut result = None;
    if biggest_force > 0.0 || any_gravity {
        let solved = solve_voxels(&fem);
        if !solved.solved {
            check.advice = solved.advice;
            return check;
        }
        // A load the body does not carry to a mount floats off: the solve returns a displacement
        // the size of a planet rather than a stress.
        let span = (grid.nx as f64).max(grid.ny as f64).max(grid.nz as f64) * grid.h;
        if !solved.worst_displacement.is_finite() || solved.worst_displacement > 10.0 * span {
            check.advice = "the body does not connect every load to a mount: nothing holds the load, so there is no stress to check".into();
            return check;
        }
        check.max_stress = solved.worst_von_mises;
        check.max_displacement = solved.worst_displacement;
        check.safety_factor = if solved.worst_von_mises > 0.0 { problem.yield_strength / solved.worst_von_mises } else { 0.0 };
        result = Some(solved);
    }
    check.solved = true;
    check.resolution = resolution;

    // Elements across the thinnest part of the body.
    {
        let mut thinnest = usize::MAX;
        for axis in 0..3 {
            let extent = [grid.nx, grid.ny, grid.nz][axis];
            let (a1, a2) = match axis {
                0 => (grid.ny, grid.nz),
                1 => (grid.nx, grid.nz),
                _ => (grid.nx, grid.ny),
            };
            for p in 0..a1 {
                for q in 0..a2 {
                    let mut run = 0;
                    for s in 0..extent {
                        let (i, j, k) = match axis {
                            0 => (s, p, q),
                            1 => (p, s, q),
                            _ => (p, q, s),
                        };
                        if solid[grid.at(i, j, k)] {
                            run += 1;
                        } else if run > 0 {
                            thinnest = thinnest.min(run);
                            run = 0;
                        }
                    }
                    if run > 0 {
                        thinnest = thinnest.min(run);
                    }
                }
            }
        }
        check.elements_through_thinnest = if thinnest == usize::MAX { 0 } else { thinnest };
        check.thinnest_feature = check.elements_through_thinnest as f64 * grid.h;
    }

    // --- buckling: asked of the elements, not the members, so it works on any shape ---
    if let Some(result) = &result {
        let folded = buckling(&fem, result, 30);
        if folded.solved {
            check.buckling_assessed = true;
            check.buckling_from_eigen_solve = true;
            check.buckling_factor = folded.load_factor;
            check.buckling_note = format!(
                "buckles at {:.2} times the applied load (eigenvalue solve on the built shape, {} iterations)",
                folded.load_factor, folded.iterations
            );
            if folded.load_factor < 1.0 {
                check.buckling_note.push_str(" -- it folds before it gets there");
            } else if folded.load_factor < 2.0 {
                check.buckling_note.push_str(" -- little margin for a slender part");
            }
        } else {
            check.buckling_note = if folded.advice.is_empty() { "no buckling mode was found".into() } else { folded.advice };
        }
    } else {
        check.buckling_note = "not assessed: nothing pushes on it".into();
    }

    // --- fatigue: a life, not a factor ---
    {
        let total_cycles: f64 = problem.loads.iter().map(|l| l.cycles).sum();
        let cyclic = options.cyclic_loading || total_cycles > 0.0;
        let ultimate = if options.ultimate_strength > 0.0 { options.ultimate_strength } else { 1.5 * problem.yield_strength };
        let polished = if options.endurance_limit > 0.0 { options.endurance_limit } else { 0.45 * problem.yield_strength };
        let ka = surface_factor(options.finish, ultimate);
        let kb = size_factor(if check.thinnest_feature > 0.0 { check.thinnest_feature } else { grid.h * 4.0 });
        let endurance = polished * ka * kb;
        if cyclic && check.max_stress > 0.0 {
            check.fatigue_assessed = true;
            if let Some(result) = &result {
                for (index, case) in result.cases.iter().enumerate() {
                    let Some(load) = problem.loads.get(index) else { break };
                    let peak = case.max_von_mises;
                    if !(peak > 0.0) {
                        continue;
                    }
                    let ratio = load.stress_ratio.clamp(-1.0, 0.99);
                    let amplitude = peak * (1.0 - ratio) / 2.0;
                    let mean = peak * (1.0 + ratio) / 2.0;
                    let life = cycles_to_failure(amplitude, mean, endurance, ultimate);
                    if peak >= check.max_stress * 0.999 {
                        check.cycles_to_failure = life;
                    }
                }
                if check.cycles_to_failure <= 0.0 {
                    let amplitude = check.max_stress / 2.0;
                    check.cycles_to_failure = cycles_to_failure(amplitude, amplitude, endurance, ultimate);
                }
                check.fatigue_damage = fatigue_damage_of(problem, result);
            }
            check.blocks_to_failure = if check.fatigue_damage > 0.0 { 1.0 / check.fatigue_damage } else { 0.0 };
            check.fatigue_factor = if check.fatigue_damage > 0.0 {
                1.0 / check.fatigue_damage
            } else if check.cycles_to_failure >= 1e29 {
                1e9
            } else {
                0.0
            };
            let mut note = if check.cycles_to_failure >= 1e29 {
                "below the endurance limit: it lasts indefinitely at this stress".to_string()
            } else {
                format!("lasts about {:.0} cycles at the governing load", check.cycles_to_failure)
            };
            if total_cycles > 0.0 {
                note.push_str(&format!("; the life asked of it spends {:.3} of that (Miner's rule, 1.0 is failure)", check.fatigue_damage));
                if check.blocks_to_failure > 0.0 && check.blocks_to_failure < 1e6 {
                    note.push_str(&format!(", so about {:.1} lifetimes", check.blocks_to_failure));
                }
            } else {
                note.push_str(
                    "; no cycle counts were given, so there is no life to check against -- give each load case the number of times it happens",
                );
            }
            note.push_str(&format!(
                ". Surface {} (x{ka:.2}) and section size (x{kb:.2}) take the polished endurance limit of {:.0} MPa down to {:.0} MPa",
                options.finish.name(),
                polished / 1e6,
                endurance / 1e6
            ));
            if options.finish == SurfaceFinish::AsPrinted {
                note.push_str(" -- a printed skin has no published surface factor, so the as-forged one stands in for it");
            }
            if options.endurance_limit <= 0.0 || options.ultimate_strength <= 0.0 {
                note.push_str(&format!(
                    ". Assuming an endurance limit of {:.0} MPa and an ultimate strength of {:.0} MPa -- give the real numbers for the real material",
                    endurance / 1e6,
                    ultimate / 1e6
                ));
            }
            check.fatigue_note = note;
        } else if cyclic {
            check.fatigue_note = "no stress to assess".into();
        } else {
            check.fatigue_note = "not assessed: the load was not declared cyclic and no cycle counts were given".into();
        }
    }

    // --- can it be made ---
    {
        let rules = MakeabilityRules {
            mouldable: true,
            machinable: true,
            printable_without_support: true,
            build_axis: options.print_axis.min(2),
            overhang_degrees: options.overhang_degrees,
            ..problem.manufacturing.clone()
        };
        check.makeability = scan_makeability(&grid.frame(), &solid, &rules);
        let voxel_surface = check.makeability.surface_voxels * grid.h * grid.h;
        check.overhang_area = check.makeability.overhang_fraction * voxel_surface.min(check.surface_area.max(voxel_surface));
        let mut note = format!(
            "printed as modelled, {:.1}% of the surface overhangs past {:.0} degrees and needs support; thinnest section about {:.2} mm",
            check.makeability.overhang_fraction * 100.0,
            options.overhang_degrees,
            check.thinnest_feature * 1000.0
        );
        if check.thinnest_feature > 0.0 && check.thinnest_feature < 0.002 {
            note.push_str(", which is below what most processes hold");
        }
        note.push_str(&format!(
            ". Of its surface, {:.1}% is trapped between the halves of a mould pulled along {}, {:.1}% has no draft, and {:.1}% cannot be reached by a {:.1} mm tool coming straight in",
            check.makeability.undercut_fraction * 100.0,
            ['X', 'Y', 'Z'][rules.pull_axis.min(2)],
            check.makeability.draft_fraction * 100.0,
            check.makeability.unreachable_fraction * 100.0,
            rules.tool_radius * 2000.0
        ));
        check.manufacturing_note = note;
    }
    check.verdict = if biggest_force > 0.0 {
        let mut v = format!("safety factor {:.2} on a {}-voxel mesh", check.safety_factor, check.resolution);
        if check.elements_through_thinnest > 0 && check.elements_through_thinnest < 4 {
            v.push_str(&format!(
                " -- only {} elements through the thinnest section, so the bending stress is under-reported",
                check.elements_through_thinnest
            ));
        }
        v
    } else {
        "no forces to check against".into()
    };
    check
}
