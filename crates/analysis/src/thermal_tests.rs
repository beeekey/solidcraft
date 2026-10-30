//! Every check here is against something known outside this program: a textbook film
//! coefficient, the Stefan–Boltzmann law, and conservation of energy.

use solidcraft_geom::{Aabb3, Mesh, Vec3};

use crate::thermal::{
    AmbientAir, HeatInput, HeldTemperature, Placement, ThermalProblem, ThermalResult, channel_convection_film, free_convection_film, radiative_film,
    thermal_solve,
};

const STEFAN_BOLTZMANN: f64 = 5.670374419e-8;

/// A closed box mesh in mm from `min`, triangles counter-clockwise from outside.
fn box_at(min: Vec3, a: f64, b: f64, c: f64) -> Mesh {
    let p = |x: f64, y: f64, z: f64| min + Vec3::new(x, y, z);
    let positions = vec![p(0.0, 0.0, 0.0), p(a, 0.0, 0.0), p(a, b, 0.0), p(0.0, b, 0.0), p(0.0, 0.0, c), p(a, 0.0, c), p(a, b, c), p(0.0, b, c)];
    let triangles =
        vec![[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7]];
    let mut mesh = Mesh { positions, triangles, ..Default::default() };
    mesh.normals = vec![Vec3::ZERO; 8];
    mesh.tri_face = (0..12).map(|t| (t / 2) as u32).collect();
    mesh
}

fn region(min: [f64; 3], max: [f64; 3]) -> Placement {
    Placement::Box(Aabb3 { min: Vec3::new(min[0], min[1], min[2]), max: Vec3::new(max[0], max[1], max[2]) })
}

/// A block of aluminium, 60 × 60 × 10 mm, standing so its 60 mm side is up.
fn plate(thickness_mm: f64) -> Mesh {
    box_at(Vec3::ZERO, 60.0, thickness_mm, 60.0)
}

fn plate_problem(watts: f64, emissivity: f64) -> ThermalProblem {
    ThermalProblem {
        conductivity: 205.0,
        resolution: 24,
        air: AmbientAir { temperature: 293.15, emissivity, up: Vec3::Z, ..Default::default() },
        sources: vec![HeatInput { region: region([0.0, 0.0, 0.0], [60.0, 10.0, 10.0]), watts, name: "chip".into() }],
        ..Default::default()
    }
}

fn solve_plate(watts: f64, emissivity: f64) -> ThermalResult {
    thermal_solve(&plate(10.0), &plate_problem(watts, emissivity))
}

#[test]
fn the_vertical_plate_correlation_matches_the_textbook() {
    // A vertical surface 0.3 m high, 30 K above room air, comes out at about 4.5 W/m²·K.
    let film = free_convection_film(293.15 + 30.0, 293.15, 0.3, 0);
    assert!(film > 3.8 && film < 5.2, "{film} W/m²K");
    let up = free_convection_film(323.15, 293.15, 0.1, 1);
    let side = free_convection_film(323.15, 293.15, 0.1, 0);
    let down = free_convection_film(323.15, 293.15, 0.1, -1);
    assert!(up > down, "up {up}, down {down}");
    assert!((down / up - 0.5).abs() < 0.02, "the standard pair is 0.27 against 0.54");
    assert!(side > 0.0);
    // A colder-than-air face is the same problem upside down.
    assert!((free_convection_film(263.15, 293.15, 0.1, -1) - free_convection_film(323.15, 293.15, 0.1, 1)).abs() < 1.0);
    assert_eq!(free_convection_film(293.15, 293.15, 0.3, 0), 0.0);
    // A wide channel matches open air and a narrow one chokes.
    let wide = channel_convection_film(323.15, 293.15, 0.05, 0.05);
    let narrow = channel_convection_film(323.15, 293.15, 0.002, 0.05);
    assert!(narrow < wide, "narrow {narrow}, wide {wide}");
}

#[test]
fn the_radiative_film_carries_exactly_the_fourth_power_law() {
    let (surface, room, emissivity) = (350.0, 293.15, 0.85);
    let film = radiative_film(surface, room, emissivity);
    let linear = film * (surface - room);
    let exact = emissivity * STEFAN_BOLTZMANN * (surface.powi(4) - room.powi(4));
    assert!((linear - exact).abs() < 1e-9 * exact);
    assert_eq!(radiative_film(surface, room, 0.0), 0.0);
}

#[test]
fn every_watt_put_in_leaves_the_surface() {
    let result = solve_plate(5.0, 0.85);
    assert!(result.solved, "{}", result.advice);
    let leaving = result.convected + result.radiated;
    assert!((leaving - 5.0).abs() < 0.05, "in 5 W, out {leaving} W ({} convected, {} radiated)", result.convected, result.radiated);
    assert!(result.peak_temperature > 293.15);
    assert!(result.film_coefficient > 1.0 && result.film_coefficient < 30.0, "{}", result.film_coefficient);
}

#[test]
fn an_isothermal_block_lands_where_hand_calculation_says_it_should() {
    let mut problem = plate_problem(4.0, 0.0);
    problem.conductivity = 5000.0;
    let result = thermal_solve(&plate(10.0), &problem);
    assert!(result.solved, "{}", result.advice);
    assert!(result.peak_temperature - result.mean_surface_temperature < 0.5);
    let rise = result.mean_surface_temperature - 293.15;
    let predicted = result.film_coefficient * result.exposed_area * rise;
    assert!((predicted - 4.0).abs() < 0.2, "{predicted} W against 4 W");
}

#[test]
fn radiation_carries_real_heat_and_black_surfaces_run_cooler() {
    let bare = solve_plate(6.0, 0.05);
    let black = solve_plate(6.0, 0.90);
    assert!(bare.solved && black.solved);
    assert!(black.peak_temperature < bare.peak_temperature, "black {} against bare {}", black.peak_temperature, bare.peak_temperature);
    assert!(black.radiated > bare.radiated);
    assert!(black.radiated / 6.0 > 0.20, "{} W of 6 W by radiation", black.radiated);
    assert!(bare.radiated / 6.0 < 0.10);
    let predicted = 0.90 * STEFAN_BOLTZMANN * black.exposed_area * (black.mean_surface_temperature.powi(4) - 293.15_f64.powi(4));
    assert!((black.radiated - predicted).abs() < 0.1 * predicted.abs());
}

#[test]
fn more_power_is_hotter_and_more_surface_is_cooler() {
    let small = solve_plate(2.0, 0.5);
    let large = solve_plate(8.0, 0.5);
    assert!(small.solved && large.solved);
    assert!(large.peak_temperature > small.peak_temperature);
    let spread = thermal_solve(&box_at(Vec3::ZERO, 120.0, 10.0, 60.0), &plate_problem(8.0, 0.5));
    assert!(spread.solved, "{}", spread.advice);
    assert!(spread.exposed_area > large.exposed_area);
    assert!(spread.peak_temperature < large.peak_temperature);
}

#[test]
fn a_held_face_carries_the_heat_away_instead() {
    let mut problem = plate_problem(10.0, 0.0);
    problem.held.push(HeldTemperature { region: region([0.0, 0.0, 0.0], [60.0, 10.0, 4.0]), temperature: 293.15, name: "cold plate".into() });
    let result = thermal_solve(&plate(10.0), &problem);
    assert!(result.solved, "{}", result.advice);
    assert!(result.held_flux > 5.0, "{} W through the held face of 10 W", result.held_flux);
    assert!(result.peak_temperature - 293.15 < 40.0, "{}", result.peak_temperature);
}

#[test]
fn nonsense_is_refused_rather_than_solved() {
    let mut no_material = plate_problem(5.0, 0.0);
    no_material.conductivity = 0.0;
    let r = thermal_solve(&plate(10.0), &no_material);
    assert!(!r.solved && r.advice.contains("conductivity"));
    let mut no_power = plate_problem(0.0, 0.0);
    no_power.sources.clear();
    let idle = thermal_solve(&plate(10.0), &no_power);
    assert!(!idle.solved && idle.advice.contains("room temperature"));
    assert!(!thermal_solve(&Mesh::default(), &plate_problem(5.0, 0.0)).solved);
    let mut elsewhere = plate_problem(5.0, 0.0);
    elsewhere.sources[0].region = region([1000.0, 1000.0, 1000.0], [1100.0, 1100.0, 1100.0]);
    let missed = thermal_solve(&plate(10.0), &elsewhere);
    assert!(missed.advice.contains("does not land"), "{}", missed.advice);
    let mut hostile = plate_problem(f64::NAN, 0.0);
    hostile.resolution = usize::MAX;
    hostile.air.emissivity = f64::INFINITY;
    let _ = thermal_solve(&plate(10.0), &hostile);
}

#[test]
fn orientation_changes_the_answer() {
    let standing = plate_problem(6.0, 0.0);
    let mut lying = plate_problem(6.0, 0.0);
    lying.air.up = Vec3::Y;
    let a = thermal_solve(&plate(10.0), &standing);
    let b = thermal_solve(&plate(10.0), &lying);
    assert!(a.solved && b.solved);
    assert!(a.film_coefficient != b.film_coefficient);
    assert!((a.peak_temperature - b.peak_temperature).abs() > 0.5, "standing {}, lying {}", a.peak_temperature, b.peak_temperature);
    assert!((a.convected + a.radiated - 6.0).abs() < 0.06);
    assert!((b.convected + b.radiated - 6.0).abs() < 0.06);
}

/// A finned plate: a 60 × 60 × 5 mm base with `count` fins 25 mm tall standing on it.
fn finned_plate(count: usize, thickness: f64) -> Mesh {
    let mut body = box_at(Vec3::ZERO, 60.0, 60.0, 5.0);
    let pitch = 60.0 / count as f64;
    for fin in 0..count {
        let blade = box_at(Vec3::new(fin as f64 * pitch, 0.0, 5.0), thickness, 60.0, 25.0);
        let faces = body.tri_face.iter().max().map(|f| f + 1).unwrap_or(0);
        body.append(&blade, faces);
    }
    body
}

fn solve_finned(count: usize, thickness: f64) -> ThermalResult {
    let problem = ThermalProblem {
        conductivity: 205.0,
        resolution: 48,
        air: AmbientAir { temperature: 293.15, emissivity: 0.0, up: Vec3::Z, ..Default::default() },
        sources: vec![HeatInput { region: region([0.0, 0.0, 0.0], [60.0, 60.0, 5.0]), watts: 15.0, name: String::new() }],
        ..Default::default()
    };
    thermal_solve(&finned_plate(count, thickness), &problem)
}

#[test]
fn fins_in_an_array_choke_when_they_are_too_close_together() {
    let few = solve_finned(4, 3.0);
    let many = solve_finned(14, 3.0);
    assert!(few.solved, "{}", few.advice);
    assert!(many.solved, "{}", many.advice);
    assert!(many.exposed_area > 1.3 * few.exposed_area, "{} against {}", many.exposed_area, few.exposed_area);
    assert!(many.channelled_fraction > few.channelled_fraction);
    assert!(many.channelled_fraction > 0.4, "{}", many.channelled_fraction);
    assert!(many.narrowest_gap < 0.005, "{} mm between fins", many.narrowest_gap * 1000.0);
    assert!(many.film_coefficient < few.film_coefficient, "crowded {} against open {}", many.film_coefficient, few.film_coefficient);
    assert!((few.convected + few.radiated - 15.0).abs() < 0.2);
    assert!((many.convected + many.radiated - 15.0).abs() < 0.2);
}

#[test]
fn there_is_a_best_fin_spacing_rather_than_more_is_better() {
    let mut best = (0usize, f64::INFINITY);
    let mut story = String::new();
    for count in [2usize, 4, 6, 10, 16] {
        let result = solve_finned(count, 3.0);
        assert!(result.solved, "{}", result.advice);
        story.push_str(&format!("{count} fins: {:.1} C   ", result.peak_temperature - 273.15));
        if result.peak_temperature < best.1 {
            best = (count, result.peak_temperature);
        }
    }
    assert!(best.0 > 2 && best.0 < 16, "{story}");
}

#[test]
fn a_sealed_box_heats_its_own_air_and_the_part_sits_on_top_of_it() {
    let open = plate_problem(6.0, 0.5);
    let mut boxed = plate_problem(6.0, 0.5);
    boxed.air.enclosure_area = 0.24;
    boxed.air.outside_temperature = 293.15;
    let in_the_room = thermal_solve(&plate(10.0), &open);
    let in_the_box = thermal_solve(&plate(10.0), &boxed);
    assert!(in_the_room.solved && in_the_box.solved, "{} / {}", in_the_room.advice, in_the_box.advice);
    assert!(in_the_box.enclosure_rise > 1.0 && in_the_box.enclosure_rise < 100.0, "{} K", in_the_box.enclosure_rise);
    assert_eq!(in_the_room.enclosure_rise, 0.0);
    assert!(in_the_box.advice.contains("above the room outside"), "{}", in_the_box.advice);
    let moved = in_the_box.peak_temperature - in_the_room.peak_temperature;
    assert!(
        (moved - in_the_box.enclosure_rise).abs() < 0.35 * in_the_box.enclosure_rise,
        "part moved {moved} K, box air moved {} K",
        in_the_box.enclosure_rise
    );
    assert!((in_the_box.convected + in_the_box.radiated - 6.0).abs() < 0.1);
    let mut bigger = boxed.clone();
    bigger.air.enclosure_area = 1.0;
    let in_a_bigger_box = thermal_solve(&plate(10.0), &bigger);
    assert!(in_a_bigger_box.solved);
    assert!(in_a_bigger_box.enclosure_rise < in_the_box.enclosure_rise);
    assert!(in_a_bigger_box.peak_temperature < in_the_box.peak_temperature);
    let film = free_convection_film(293.15 + in_the_box.enclosure_rise, 293.15, (0.24_f64 / 6.0).sqrt(), 0)
        + radiative_film(293.15 + in_the_box.enclosure_rise, 293.15, 0.9);
    assert!((film * 0.24 * in_the_box.enclosure_rise - 6.0).abs() < 0.2);
}
