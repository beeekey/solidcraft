//! A part sitting in still air.
//!
//! Most parts are not in a duct: they sit on a bench, in a box, on a wall, with nothing blowing
//! on them, and they lose their heat two ways: to the air by free convection, and to everything
//! around them by radiation. This solves what is actually there: steady conduction through the
//! part, with each exposed face losing heat by the free-convection correlation for its
//! orientation and by Stefan–Boltzmann for its emissivity. Both depend on the surface
//! temperature, which is what is being solved for, so it iterates until the two agree.
//!
//! What it is: the calculation an engineer does by hand for a heat sink, done on the real
//! geometry. What it is not: a buoyant flow solve. It does know that a fin can be shadowed by the
//! fin beside it (the channel correlation), and it assumes the air around the part is unbounded
//! unless a sealed box is given.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Aabb3, Mesh, Vec3};

use crate::voxelise::{MM, Region, VoxelGrid, voxelise};

/// Where heat goes in or a temperature is held: a box of the part (mm), or a picked face.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Placement {
    Box(Aabb3),
    Face(Region),
}

impl Default for Placement {
    fn default() -> Self {
        Placement::Box(Aabb3::EMPTY)
    }
}

const STEFAN_BOLTZMANN: f64 = 5.670374419e-8; // W/m²·K⁴
const GRAVITY: f64 = 9.80665;

/// The room the part is sitting in.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AmbientAir {
    /// K
    pub temperature: f64,
    /// Surface emissivity, 0 to 1. Bare aluminium is about 0.05, black anodising about 0.85: on a
    /// natural-convection heat sink that is the difference between radiation carrying a twentieth
    /// of the heat and a third of it.
    pub emissivity: f64,
    /// Which way is up. The same plate facing up and facing down differs by a factor of two.
    pub up: Vec3,
    /// What the part radiates to (K); 0 = the same as the air.
    pub surroundings_temperature: f64,
    /// Outside area (m²) of a sealed box around the part; 0 = no box, the air is the room's. Air
    /// in a closed enclosure heats up until the box sheds the power through its walls, and the
    /// part sits on top of that temperature.
    pub enclosure_area: f64,
    /// How well the box sheds heat, W/m²·K over its outside; 0 asks for free convection plus
    /// radiation off a painted box.
    pub enclosure_film: f64,
    /// The room outside the box (K); 0 = `temperature`.
    pub outside_temperature: f64,
}

impl Default for AmbientAir {
    fn default() -> Self {
        AmbientAir {
            temperature: 293.15,
            emissivity: 0.0,
            up: Vec3::Z,
            surroundings_temperature: 0.0,
            enclosure_area: 0.0,
            enclosure_film: 0.0,
            outside_temperature: 0.0,
        }
    }
}

/// Watts put in over a region of the part.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HeatInput {
    pub region: Placement,
    pub watts: f64,
    pub name: String,
}

/// A region of the part held at a temperature: a cold plate under it, a mount bolted to
/// something massive (mm, K).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HeldTemperature {
    pub region: Placement,
    pub temperature: f64,
    pub name: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalProblem {
    /// W/m·K
    pub conductivity: f64,
    /// Cells along the longest side.
    pub resolution: usize,
    pub sources: Vec<HeatInput>,
    pub held: Vec<HeldTemperature>,
    pub air: AmbientAir,
    /// Film/temperature outer iterations.
    pub max_rounds: usize,
}

impl Default for ThermalProblem {
    fn default() -> Self {
        ThermalProblem { conductivity: 0.0, resolution: 40, sources: Vec::new(), held: Vec::new(), air: AmbientAir::default(), max_rounds: 40 }
    }
}

/// The most cells a thermal grid may hold.
pub const MAX_THERMAL_CELLS: usize = 2_000_000;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ThermalResult {
    pub solved: bool,
    /// K
    pub peak_temperature: f64,
    pub mean_surface_temperature: f64,
    /// K, what it was solved against.
    pub air_temperature: f64,
    /// How far the air inside a sealed box rose above the room outside it.
    pub enclosure_rise: f64,
    /// W leaving by free convection, by radiation, and through held faces.
    pub convected: f64,
    pub radiated: f64,
    pub held_flux: f64,
    /// Area-weighted mean film coefficient, W/m²·K.
    pub film_coefficient: f64,
    /// m²
    pub exposed_area: f64,
    /// The vertical-surface Rayleigh number.
    pub rayleigh: f64,
    /// How much of the exposed surface sits in a channel: how finned the part is.
    pub channelled_fraction: f64,
    /// The narrowest gap found between facing surfaces (m, 0 = none).
    pub narrowest_gap: f64,
    pub rounds: usize,
    pub grid: [usize; 3],
    /// Cell size (m) and the grid's origin (m).
    pub h: f64,
    pub origin: Vec3,
    pub solid: Vec<bool>,
    /// K per cell (ambient where empty).
    pub temperature: Vec<f64>,
    pub advice: String,
}

/// Air at the film temperature the correlations want: power laws fitted to the standard tables,
/// good to a percent or so from 250 K to 450 K.
struct Air {
    viscosity: f64,
    conductivity: f64,
    prandtl: f64,
    expansion: f64,
}

fn air_at(film: f64) -> Air {
    let t = film.clamp(200.0, 800.0);
    Air { viscosity: 1.589e-5 * (t / 300.0).powf(1.75), conductivity: 0.0263 * (t / 300.0).powf(0.85), prandtl: 0.707, expansion: 1.0 / t }
}

/// Free-convection film coefficient for one surface: Churchill–Chu for a vertical surface,
/// McAdams for a horizontal one facing up (+1) or down (−1). `length` is the characteristic
/// length: height for a vertical surface, area over perimeter for a horizontal one.
pub fn free_convection_film(surface: f64, air_temperature: f64, length: f64, facing: i32) -> f64 {
    let difference = surface - air_temperature;
    if !(length > 0.0) || difference.abs() < 1e-9 {
        return 0.0;
    }
    let air = air_at(0.5 * (surface + air_temperature));
    let diffusivity = air.viscosity / air.prandtl;
    let rayleigh = GRAVITY * air.expansion * difference.abs() * length.powi(3) / (air.viscosity * diffusivity);
    if !(rayleigh > 0.0) || !rayleigh.is_finite() {
        return 0.0;
    }
    // A face cooler than the air behaves like the opposite orientation.
    let orientation = if difference >= 0.0 { facing } else { -facing };
    let nusselt = if orientation == 0 {
        let factor = (1.0 + (0.492 / air.prandtl).powf(9.0 / 16.0)).powf(4.0 / 9.0);
        if rayleigh <= 1e9 {
            0.68 + 0.670 * rayleigh.powf(0.25) / factor
        } else {
            let bracket = (1.0 + (0.492 / air.prandtl).powf(9.0 / 16.0)).powf(8.0 / 27.0);
            let root = 0.825 + 0.387 * rayleigh.powf(1.0 / 6.0) / bracket;
            root * root
        }
    } else if orientation > 0 {
        if rayleigh <= 1e7 { 0.54 * rayleigh.powf(0.25) } else { 0.15 * rayleigh.powf(1.0 / 3.0) }
    } else {
        0.27 * rayleigh.powf(0.25)
    };
    if !(nusselt > 0.0) || !nusselt.is_finite() {
        return 0.0;
    }
    nusselt * air.conductivity / length
}

/// Free convection in a channel between two fins, by the Elenbaas correlation for parallel
/// vertical plates. Below a certain spacing the two boundary layers merge and the channel chokes:
/// adding more fins then makes a heat sink worse. Never more than open air would give.
pub fn channel_convection_film(surface: f64, air_temperature: f64, gap: f64, length: f64) -> f64 {
    let difference = (surface - air_temperature).abs();
    if !(gap > 0.0) || !(length > 0.0) || difference < 1e-9 {
        return 0.0;
    }
    let air = air_at(0.5 * (surface + air_temperature));
    let diffusivity = air.viscosity / air.prandtl;
    let rayleigh = GRAVITY * air.expansion * difference * gap.powi(3) / (air.viscosity * diffusivity);
    let grouped = rayleigh * gap / length;
    if !(grouped > 0.0) || !grouped.is_finite() {
        return 0.0;
    }
    let nusselt = (grouped / 24.0) * (1.0 - (-35.0 / grouped).exp()).powf(0.75);
    if !(nusselt > 0.0) || !nusselt.is_finite() {
        return 0.0;
    }
    nusselt * air.conductivity / gap
}

/// The radiative film coefficient that carries the same heat as ε σ (Ts⁴ − Tsur⁴) across
/// (Ts − Tsur): exact at the temperature it is evaluated at, and the outer iteration
/// re-evaluates it, so the answer is the fourth-power law, solved.
pub fn radiative_film(surface: f64, surroundings: f64, emissivity: f64) -> f64 {
    if !(emissivity > 0.0) || !(surface > 0.0) || !(surroundings > 0.0) {
        return 0.0;
    }
    emissivity * STEFAN_BOLTZMANN * (surface * surface + surroundings * surroundings) * (surface + surroundings)
}

/// A face of a material cell that looks at air.
struct Face {
    voxel: usize,
    /// +1 up, −1 down, 0 vertical.
    facing: i32,
    /// The characteristic length for that orientation (m).
    length: f64,
    /// Distance to the surface this one looks at (m), and that cell, when it is in a channel.
    gap: f64,
    facing_voxel: Option<usize>,
}

fn box_has(region: &Aabb3, p_mm: Vec3) -> bool {
    p_mm.x >= region.min.x
        && p_mm.x <= region.max.x
        && p_mm.y >= region.min.y
        && p_mm.y <= region.max.y
        && p_mm.z >= region.min.z
        && p_mm.z <= region.max.z
}

/// The material cells a placement names.
fn cells_of(grid: &VoxelGrid, mesh: &Mesh, material: &[usize], placement: &Placement) -> Vec<usize> {
    match placement {
        Placement::Box(region) => material.iter().copied().filter(|v| box_has(region, grid.centre(*v) * MM)).collect(),
        Placement::Face(region) => grid.region(mesh, region),
    }
}

/// Solves the part (a closed mesh in mm) in still air.
pub fn thermal_solve(mesh: &Mesh, problem: &ThermalProblem) -> ThermalResult {
    let mut result = ThermalResult::default();
    let mut advice = String::new();
    if mesh.triangles.is_empty() {
        result.advice = "there is no part to heat".into();
        return result;
    }
    if !(problem.conductivity > 0.0) || !problem.conductivity.is_finite() {
        result.advice = "the part has no thermal conductivity, so no heat can move through it -- assign a material".into();
        return result;
    }
    let total_watts: f64 = problem.sources.iter().map(|s| s.watts.max(0.0)).filter(|w| w.is_finite()).sum();
    if !(total_watts > 0.0) && problem.held.is_empty() {
        result.advice = "nothing puts heat in and nothing holds a temperature, so the part is simply at room temperature".into();
        return result;
    }
    let resolution = problem.resolution.clamp(8, 120);
    let Some(grid): Option<VoxelGrid> = voxelise(mesh, resolution, MAX_THERMAL_CELLS) else {
        result.advice = "the part has no size, or the grid would be too fine".into();
        return result;
    };
    let (nx, ny, nz, h) = (grid.nx, grid.ny, grid.nz, grid.h);
    let cells = nx * ny * nz;
    result.grid = [nx, ny, nz];
    result.h = h;
    result.origin = grid.origin;
    let material: Vec<usize> = (0..cells).filter(|v| grid.solid[*v]).collect();
    if material.is_empty() {
        result.advice = "the part fell through the grid -- raise the resolution".into();
        return result;
    }
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;

    // Which way is up, and the three lengths the correlations want.
    let up = problem.air.up.normalized().unwrap_or(Vec3::Z);
    let sizes = [grid.size.x, grid.size.y, grid.size.z];
    let longest = sizes.iter().cloned().fold(0.0, f64::max);
    let height = up.x.abs() * sizes[0] + up.y.abs() * sizes[1] + up.z.abs() * sizes[2];
    let mut other: Vec<f64> = Vec::new();
    for (axis, along) in [up.x.abs(), up.y.abs(), up.z.abs()].iter().enumerate() {
        if *along <= 0.5 && other.len() < 2 {
            other.push(sizes[axis]);
        }
    }
    let foot_a = other.first().copied().unwrap_or(longest);
    let foot_b = other.get(1).copied().unwrap_or(foot_a);
    let footprint = (foot_a * foot_b) / (2.0 * (foot_a + foot_b)).max(1e-9);
    let vertical_length = height.max(h);
    let horizontal_length = footprint.max(h * 0.5);

    // A sealed box: solve the box first, it sets the level everything else sits on.
    let mut ambient = problem.air.temperature;
    let mut enclosure_rise = 0.0;
    if problem.air.enclosure_area > 0.0 && total_watts > 0.0 {
        let outside = if problem.air.outside_temperature > 0.0 { problem.air.outside_temperature } else { problem.air.temperature };
        let mut surface = outside + 10.0;
        for _ in 0..40 {
            let film = if problem.air.enclosure_film > 0.0 {
                problem.air.enclosure_film
            } else {
                free_convection_film(surface, outside, (problem.air.enclosure_area / 6.0).sqrt(), 0) + radiative_film(surface, outside, 0.9)
            };
            if !(film > 0.0) {
                break;
            }
            let wanted = outside + total_watts / (film * problem.air.enclosure_area);
            if (wanted - surface).abs() < 1e-6 {
                surface = wanted;
                break;
            }
            surface = 0.5 * (surface + wanted);
        }
        enclosure_rise = (surface - outside).max(0.0);
        ambient = outside + enclosure_rise;
    }
    result.enclosure_rise = enclosure_rise;
    let surroundings = if problem.air.surroundings_temperature > 0.0 { problem.air.surroundings_temperature } else { ambient };

    // Every exposed face of every solid cell, found once.
    let steps: [[i64; 3]; 6] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
    let in_grid = |i: i64, j: i64, k: i64| i >= 0 && j >= 0 && k >= 0 && (i as usize) < nx && (j as usize) < ny && (k as usize) < nz;
    let mut faces: Vec<Face> = Vec::new();
    for &v in &material {
        let (i, j, k) = grid.ijk(v);
        for step in &steps {
            let (ni, nj, nk) = (i as i64 + step[0], j as i64 + step[1], k as i64 + step[2]);
            if in_grid(ni, nj, nk) && grid.solid[at(ni as usize, nj as usize, nk as usize)] {
                continue;
            }
            let along = step[0] as f64 * up.x + step[1] as f64 * up.y + step[2] as f64 * up.z;
            let facing = if along > 0.5 {
                1
            } else if along < -0.5 {
                -1
            } else {
                0
            };
            let mut face =
                Face { voxel: v, facing, length: if facing == 0 { vertical_length } else { horizontal_length }, gap: 0.0, facing_voxel: None };
            // What does this face look at? Material within a few tens of cells means a channel.
            for reach in 1..=64_i64 {
                let (ai, aj, ak) = (i as i64 + step[0] * reach, j as i64 + step[1] * reach, k as i64 + step[2] * reach);
                if !in_grid(ai, aj, ak) {
                    break;
                }
                let ahead = at(ai as usize, aj as usize, ak as usize);
                if grid.solid[ahead] {
                    face.gap = reach as f64 * h;
                    face.facing_voxel = Some(ahead);
                    break;
                }
            }
            faces.push(face);
        }
    }
    result.exposed_area = faces.len() as f64 * h * h;
    let channelled = faces.iter().filter(|f| f.facing_voxel.is_some()).count();
    result.channelled_fraction = if faces.is_empty() { 0.0 } else { channelled as f64 / faces.len() as f64 };
    result.narrowest_gap =
        faces.iter().filter(|f| f.facing_voxel.is_some()).map(|f| f.gap).fold(0.0, |m: f64, g| if m <= 0.0 || g < m { g } else { m });
    if faces.is_empty() {
        result.advice = "the part has no exposed surface at this resolution".into();
        return result;
    }
    let convection_of = |face: &Face, surface: f64| {
        let open = free_convection_film(surface, ambient, face.length, face.facing);
        if face.facing_voxel.is_none() {
            return open;
        }
        open.min(channel_convection_film(surface, ambient, face.gap, face.length))
    };
    let sink_for = |face: &Face, temperature: &[f64]| face.facing_voxel.map(|v| temperature[v]).unwrap_or(surroundings);

    // Where the heat goes in, and where a temperature is held.
    let mut source = vec![0.0; cells];
    for input in &problem.sources {
        if !(input.watts > 0.0) || !input.watts.is_finite() {
            continue;
        }
        let touched = cells_of(&grid, mesh, &material, &input.region);
        if touched.is_empty() {
            let name = if input.name.is_empty() { "a heat source" } else { input.name.as_str() };
            advice.push_str(&format!("\"{name}\" does not land on the part at this resolution, so its {} W were not applied. ", input.watts));
            continue;
        }
        let each = input.watts / touched.len() as f64;
        for v in touched {
            source[v] += each;
        }
    }
    let mut held_value: Vec<Option<f64>> = vec![None; cells];
    for hold in &problem.held {
        if !hold.temperature.is_finite() {
            continue;
        }
        let touched = cells_of(&grid, mesh, &material, &hold.region);
        for &v in &touched {
            held_value[v] = Some(hold.temperature);
        }
        if touched.is_empty() {
            let name = if hold.name.is_empty() { "a held face" } else { hold.name.as_str() };
            advice.push_str(&format!("\"{name}\" does not land on the part at this resolution. "));
        }
    }
    let any_held = held_value.iter().any(Option::is_some);

    // Start at ambient plus a first guess at the rise so the first film coefficients are not
    // computed at zero difference.
    let mut temperature = vec![ambient; cells];
    let guess = if total_watts > 0.0 { (total_watts / (result.exposed_area * 10.0).max(1e-9)).clamp(0.5, 200.0) } else { 0.0 };
    for &v in &material {
        temperature[v] = ambient + guess;
    }
    let solid_link = problem.conductivity * h; // k·A/L with A = h², L = h
    let mut previous_peak = 0.0;
    let emissivity = problem.air.emissivity.clamp(0.0, 1.0);
    let mut loss = vec![0.0; cells];
    let mut drive = vec![0.0; cells];
    for round in 0..problem.max_rounds.clamp(1, 200) {
        loss.fill(0.0);
        drive.fill(0.0);
        for face in &faces {
            let surface = temperature[face.voxel];
            let convective = convection_of(face, surface);
            let sink = sink_for(face, &temperature);
            let radiative = radiative_film(surface, sink, emissivity);
            loss[face.voxel] += (convective + radiative) * h * h;
            drive[face.voxel] += h * h * (convective * ambient + radiative * sink);
        }
        // Relax the conduction to steady state with those coefficients held fixed.
        for sweep in 0..400 {
            let (mut change, mut magnitude) = (0.0, 0.0);
            for &v in &material {
                if let Some(held) = held_value[v] {
                    temperature[v] = held;
                    continue;
                }
                let (i, j, k) = grid.ijk(v);
                let mut numerator = drive[v] + source[v];
                let mut denominator = loss[v];
                for step in &steps {
                    let (ni, nj, nk) = (i as i64 + step[0], j as i64 + step[1], k as i64 + step[2]);
                    if !in_grid(ni, nj, nk) {
                        continue;
                    }
                    let n = at(ni as usize, nj as usize, nk as usize);
                    if grid.solid[n] {
                        numerator += solid_link * temperature[n];
                        denominator += solid_link;
                    }
                }
                if !(denominator > 0.0) {
                    continue;
                }
                let solved = numerator / denominator;
                change += (solved - temperature[v]).abs();
                magnitude += (solved - ambient).abs();
                temperature[v] = solved;
            }
            if magnitude > 0.0 && change / magnitude < 1e-12 && sweep > 5 {
                break;
            }
        }
        // The level of the whole field is the slowest thing: the surface balance fixes it
        // directly, every watt in leaves through the surface. Not when a face is held.
        if !any_held && total_watts > 0.0 {
            let (mut leaving, mut conductance) = (0.0, 0.0);
            for face in &faces {
                let surface = temperature[face.voxel];
                let convective = convection_of(face, surface);
                let sink = sink_for(face, &temperature);
                let radiative = radiative_film(surface, sink, emissivity);
                leaving += h * h * (convective * (surface - ambient) + radiative * (surface - sink));
                conductance += h * h * (convective + radiative);
            }
            if conductance > 0.0 {
                let shift = ((total_watts - leaving) / conductance).clamp(-200.0, 200.0);
                if shift.is_finite() {
                    for &v in &material {
                        temperature[v] += shift;
                    }
                }
            }
        }
        let peak = material.iter().map(|v| temperature[*v]).fold(0.0, f64::max);
        result.rounds = round + 1;
        if round > 0 && (peak - previous_peak).abs() < 1e-6 * peak.max(1.0) {
            break;
        }
        previous_peak = peak;
    }

    // What actually leaves, face by face, from the settled field: the check, computed from the
    // fourth-power law and the correlation directly.
    let (mut convected, mut radiated, mut weighted_film, mut surface_sum) = (0.0, 0.0, 0.0, 0.0);
    for face in &faces {
        let surface = temperature[face.voxel];
        let film = convection_of(face, surface);
        convected += film * h * h * (surface - ambient);
        if emissivity > 0.0 {
            let sink = sink_for(face, &temperature);
            radiated += emissivity * STEFAN_BOLTZMANN * h * h * (surface.powi(4) - sink.powi(4));
        }
        weighted_film += film * h * h;
        surface_sum += surface;
    }
    result.convected = convected;
    result.radiated = radiated;
    result.film_coefficient = if result.exposed_area > 0.0 { weighted_film / result.exposed_area } else { 0.0 };
    result.mean_surface_temperature = surface_sum / faces.len() as f64;
    result.held_flux = total_watts - convected - radiated;
    result.peak_temperature = material.iter().map(|v| temperature[*v]).fold(0.0, f64::max);
    let rise = (result.mean_surface_temperature - ambient).max(0.0);
    if rise > 0.0 {
        let air = air_at(0.5 * (result.mean_surface_temperature + ambient));
        let diffusivity = air.viscosity / air.prandtl;
        result.rayleigh = GRAVITY * air.expansion * rise * vertical_length.powi(3) / (air.viscosity * diffusivity);
    }
    result.air_temperature = ambient;
    result.solid = grid.solid;
    result.temperature = temperature;
    result.solved = true;

    if !any_held && total_watts > 0.0 {
        let leaving = convected + radiated;
        if (leaving - total_watts).abs() / total_watts > 0.02 {
            advice.push_str(&format!("the heat leaving the part ({leaving:.2} W) does not match the {total_watts:.2} W put in, so the temperature field has not settled -- raise the number of rounds. "));
        }
    }
    if result.rayleigh > 1e9 {
        advice.push_str(&format!("the surface is past the laminar range (Rayleigh {:.3e}), so the film coefficient comes from the turbulent correlation and is less certain. ", result.rayleigh));
    }
    if result.enclosure_rise > 0.0 {
        advice.push_str(&format!("the air in the box settles {:.1} K above the room outside it, and the part sits on top of that -- an unbounded room would have flattered this answer by the same amount. ", result.enclosure_rise));
    }
    if emissivity <= 0.0 {
        advice.push_str("radiation was not counted (emissivity 0); a hot part in a room loses a real share of its heat that way. ");
    }
    result.advice = advice;
    result
}
