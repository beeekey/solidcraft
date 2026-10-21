//! Brickify-style construction planning: a body approximated by a mix of real construction
//! hardware, each family chosen by the caller. Aluminium T-slot profiles (2020, 3030 or 4040)
//! claim straight full-section runs on a coarse grid of their own size; stud bricks fill the rest
//! on their stud grid (classic 8 mm bricks or thin plates, or large 16 mm bricks; catalogue sizes
//! up to 2×8, layers interlocked by alternating the orientation); threaded rods (M6 to M12) clamp
//! tall stud columns and hex bolts (M3 to M8) the short ones, with nuts and washers in the bill of
//! materials. Coverage reports how much of the model's voxelised interior the plan fills.
//! Everything here is in millimetres.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Mesh, Vec3};

/// The classic brick system's stud pitch, brick height, the clearance taken off each brick side,
/// the stud, the smallest profile cell and the default rod: the values the other systems scale.
pub const PITCH: f64 = 8.0;
pub const BRICK_HEIGHT: f64 = 9.6;
pub const BRICK_CLEARANCE: f64 = 0.2;
pub const STUD_DIAMETER: f64 = 4.8;
pub const STUD_HEIGHT: f64 = 1.8;
pub const PROFILE_CELL: f64 = 20.0;
pub const ROD_DIAMETER: f64 = 8.0;
pub const ROD_BORE: f64 = 8.8;

/// Catalogue brick footprints (studs), biggest first.
const BRICK_SIZES: [[usize; 2]; 11] = [[2, 8], [2, 6], [2, 4], [2, 3], [2, 2], [1, 8], [1, 6], [1, 4], [1, 3], [1, 2], [1, 1]];

/// The most stud cells a plan may hold.
pub const MAX_CELLS: usize = 2_000_000;

/// The T-slot series the plan can use (the profile's side in mm).
pub const PROFILE_SERIES: [u32; 3] = [20, 30, 40];
/// Threaded rod sizes (metric thread diameter).
pub const ROD_SIZES: [u32; 4] = [6, 8, 10, 12];
/// Hex bolt sizes (metric thread diameter).
pub const BOLT_SIZES: [u32; 5] = [3, 4, 5, 6, 8];
/// Stock hex bolt lengths (mm), shortest first.
const BOLT_LENGTHS: [f64; 19] = [8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0];

/// A stud brick system: the grid the bricks are laid on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum BrickKind {
    /// Classic bricks: 8 mm studs, 9.6 mm high.
    #[default]
    Brick,
    /// Classic plates: 8 mm studs, a third of a brick high.
    Plate,
    /// Large bricks for small hands: 16 mm studs, 19.2 mm high.
    Large,
}

/// Every brick system, in the order a chooser lists them.
pub const BRICK_KINDS: [BrickKind; 3] = [BrickKind::Brick, BrickKind::Plate, BrickKind::Large];

impl BrickKind {
    /// Stud pitch (mm).
    pub fn pitch(self) -> f64 {
        match self {
            BrickKind::Brick | BrickKind::Plate => PITCH,
            BrickKind::Large => 2.0 * PITCH,
        }
    }
    /// Layer height (mm).
    pub fn height(self) -> f64 {
        match self {
            BrickKind::Brick => BRICK_HEIGHT,
            BrickKind::Plate => BRICK_HEIGHT / 3.0,
            BrickKind::Large => 2.0 * BRICK_HEIGHT,
        }
    }
    /// Stud diameter and height (mm).
    pub fn stud(self) -> (f64, f64) {
        match self {
            BrickKind::Brick | BrickKind::Plate => (STUD_DIAMETER, STUD_HEIGHT),
            BrickKind::Large => (9.4, 4.5),
        }
    }
    /// The short name a script passes.
    pub fn name(self) -> &'static str {
        match self {
            BrickKind::Brick => "brick",
            BrickKind::Plate => "plate",
            BrickKind::Large => "large",
        }
    }
    /// What a chooser shows.
    pub fn label(self) -> &'static str {
        match self {
            BrickKind::Brick => "Bricks (8 mm studs, 9.6 mm high)",
            BrickKind::Plate => "Plates (8 mm studs, 3.2 mm thin)",
            BrickKind::Large => "Large bricks (16 mm studs, 19.2 mm high)",
        }
    }
    /// The word a bill of materials uses for one piece.
    pub fn piece(self) -> &'static str {
        match self {
            BrickKind::Brick => "Brick",
            BrickKind::Plate => "Plate",
            BrickKind::Large => "Large brick",
        }
    }
    /// A system from its name or its label, however it is capitalised: `brick`, `bricks`,
    /// `classic`, `plate`, `plates`, `large`, `big`, or anything starting with those.
    pub fn from_name(text: &str) -> Option<Self> {
        let t = text.trim().to_ascii_lowercase();
        if t.is_empty() {
            return None;
        }
        if t.starts_with("large") || t.starts_with("big") {
            Some(BrickKind::Large)
        } else if t.starts_with("plate") {
            Some(BrickKind::Plate)
        } else if t.starts_with("brick") || t.starts_with("classic") {
            Some(BrickKind::Brick)
        } else {
            None
        }
    }
}

/// What a plan element is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ElementKind {
    #[default]
    Brick,
    Profile,
    Rod,
    Bolt,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Options {
    pub use_bricks: bool,
    pub use_profiles: bool,
    pub use_rods: bool,
    pub use_bolts: bool,
    /// The stud system bricks are laid on.
    pub brick: BrickKind,
    /// T-slot series: 20, 30 or 40.
    pub profile_series: u32,
    /// Two 2020 cells by default, so a hand-sized part still gets a run.
    pub min_profile_length: f64,
    /// Threaded rod thread diameter (mm).
    pub rod_size: u32,
    /// Clamp columns at least this many layers tall with a rod.
    pub rod_min_layers: usize,
    /// Rod and bolt lattice pitch in studs (48 mm on the classic grid).
    pub rod_spacing_studs: usize,
    /// Hex bolt thread diameter (mm).
    pub bolt_size: u32,
    /// Bolt columns at least this many layers tall (and too short for a rod).
    pub bolt_min_layers: usize,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            use_bricks: true,
            use_profiles: true,
            use_rods: true,
            use_bolts: true,
            brick: BrickKind::Brick,
            profile_series: 20,
            min_profile_length: 40.0,
            rod_size: 8,
            rod_min_layers: 4,
            rod_spacing_studs: 6,
            bolt_size: 4,
            bolt_min_layers: 2,
        }
    }
}

/// One piece of hardware in the plan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Element {
    pub kind: ElementKind,
    /// Catalogue name, also the bill-of-materials key.
    pub name: String,
    /// Min corner (bricks, profiles) or axis start (rods, bolts: the thread's lower end), mm.
    pub position: Vec3,
    /// Bricks: studs along x and y.
    pub studs_x: usize,
    pub studs_y: usize,
    /// Profiles, rods and bolts: the length (mm), and the axis a profile runs along (0, 1, 2).
    pub length: f64,
    pub axis: usize,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Plan {
    pub elements: Vec<Element>,
    pub bill_of_materials: BTreeMap<String, usize>,
    /// Solid stud cells of the model, and how many bricks or profiles cover.
    pub cells_total: usize,
    pub cells_covered: usize,
    pub coverage: f64,
    /// The stud grid the plan was laid on (mm, cells).
    pub origin: Vec3,
    pub grid: [usize; 3],
    /// The hardware families the plan was drawn with.
    pub brick: BrickKind,
    pub profile_series: u32,
    pub rod_size: u32,
    pub bolt_size: u32,
    /// Why a family that was switched on placed nothing: the size it needed and did not find.
    pub warnings: Vec<String>,
}

impl Plan {
    /// A human-readable bill of materials and the coverage.
    pub fn summary(&self) -> String {
        let mut out = String::from("Bill of materials:\n");
        for (name, count) in &self.bill_of_materials {
            out.push_str(&format!("  {count} x {name}\n"));
        }
        out.push_str(&format!("Coverage: {} of {} cells ({:.1} %)", self.cells_covered, self.cells_total, self.coverage * 100.0));
        out
    }
}

struct Grid {
    nx: usize,
    ny: usize,
    nz: usize,
    origin: Vec3,
    cell: Vec<bool>,
}

impl Grid {
    fn index(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.ny + j) * self.nx + i
    }
    fn solid(&self, i: usize, j: usize, k: usize) -> bool {
        i < self.nx && j < self.ny && k < self.nz && self.cell[self.index(i, j, k)]
    }
    /// The longest solid run up the column at (i, j): its first layer and its length.
    fn longest_column(&self, i: usize, j: usize) -> (usize, usize) {
        let (mut best_start, mut best_length, mut run, mut run_start) = (0, 0, 0, 0);
        for k in 0..=self.nz {
            if k < self.nz && self.solid(i, j, k) {
                if run == 0 {
                    run_start = k;
                }
                run += 1;
            } else {
                if run > best_length {
                    best_length = run;
                    best_start = run_start;
                }
                run = 0;
            }
        }
        (best_start, best_length)
    }
}

/// Plans the construction of a closed mesh (mm). `None` when the mesh is empty or too big for
/// the stud grid, or the options name a profile series, rod or bolt the catalogue lacks.
pub fn constructify(body: &Mesh, options: &Options) -> Option<Plan> {
    if body.triangles.is_empty() {
        return None;
    }
    if !PROFILE_SERIES.contains(&options.profile_series) || !ROD_SIZES.contains(&options.rod_size) || !BOLT_SIZES.contains(&options.bolt_size) {
        return None;
    }
    let bounds = body.bounds();
    if bounds.is_empty() {
        return None;
    }
    let size = bounds.size();
    let pitch = options.brick.pitch();
    let layer = options.brick.height();
    let inside_index = body.inside_index();
    let inside = |x: f64, y: f64, z: f64| inside_index.contains(Vec3::new(x, y, z));
    // Floor, not round: a partial boundary row cannot hold a brick, and a rounded-up row puts
    // cell centres exactly on the surface.
    let mut grid = Grid {
        nx: ((size.x / pitch + 1e-6).floor() as usize).max(1),
        ny: ((size.y / pitch + 1e-6).floor() as usize).max(1),
        nz: ((size.z / layer + 1e-6).floor() as usize).max(1),
        origin: bounds.min,
        cell: Vec::new(),
    };
    let count = grid.nx.checked_mul(grid.ny)?.checked_mul(grid.nz)?;
    if count > MAX_CELLS {
        return None;
    }
    grid.cell = vec![false; count];
    let mut plan = Plan {
        origin: grid.origin,
        grid: [grid.nx, grid.ny, grid.nz],
        brick: options.brick,
        profile_series: options.profile_series,
        rod_size: options.rod_size,
        bolt_size: options.bolt_size,
        ..Default::default()
    };
    for k in 0..grid.nz {
        for j in 0..grid.ny {
            for i in 0..grid.nx {
                if inside(
                    grid.origin.x + (i as f64 + 0.5) * pitch,
                    grid.origin.y + (j as f64 + 0.5) * pitch,
                    grid.origin.z + (k as f64 + 0.5) * layer,
                ) {
                    let index = grid.index(i, j, k);
                    grid.cell[index] = true;
                    plan.cells_total += 1;
                }
            }
        }
    }

    // --- aluminium T-slot runs on the coarse grid, along the longest axis ---
    let mut profile_boxes: Vec<(Vec3, Vec3)> = Vec::new();
    if options.use_profiles {
        let cell = options.profile_series as f64;
        let counts = [(size.x / cell) as usize, (size.y / cell) as usize, (size.z / cell) as usize];
        let full_cell = |i: usize, j: usize, k: usize| -> bool {
            // All corners and the centre inside: the profile fits entirely.
            for dz in 0..=2 {
                for dy in 0..=2 {
                    for dx in 0..=2 {
                        let eps = 1e-6;
                        let x = (bounds.min.x + i as f64 * cell + dx as f64 * 0.5 * cell).clamp(bounds.min.x + eps, bounds.max.x - eps);
                        let y = (bounds.min.y + j as f64 * cell + dy as f64 * 0.5 * cell).clamp(bounds.min.y + eps, bounds.max.y - eps);
                        let z = (bounds.min.z + k as f64 * cell + dz as f64 * 0.5 * cell).clamp(bounds.min.z + eps, bounds.max.z - eps);
                        if !inside(x, y, z) {
                            return false;
                        }
                    }
                }
            }
            true
        };
        let axis = if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        };
        let min_cells = ((options.min_profile_length / cell).ceil() as usize).max(1);
        let (px, py, pz) = (counts[0].max(1), counts[1].max(1), counts[2].max(1));
        let mut used = vec![false; px * py * pz];
        let p_index = |i: usize, j: usize, k: usize| (k * py + j) * px + i;
        for k in 0..pz {
            for j in 0..py {
                for i in 0..px {
                    if i >= counts[0] || j >= counts[1] || k >= counts[2] {
                        continue;
                    }
                    if used[p_index(i, j, k)] || !full_cell(i, j, k) {
                        continue;
                    }
                    let mut run = 0;
                    loop {
                        let mut probe = [i, j, k];
                        probe[axis] += run;
                        if probe[axis] >= counts[axis] || used[p_index(probe[0], probe[1], probe[2])] || !full_cell(probe[0], probe[1], probe[2]) {
                            break;
                        }
                        run += 1;
                    }
                    if run < min_cells {
                        continue;
                    }
                    for r in 0..run {
                        let mut probe = [i, j, k];
                        probe[axis] += r;
                        used[p_index(probe[0], probe[1], probe[2])] = true;
                    }
                    let length = run as f64 * cell;
                    let corner = bounds.min + Vec3::new(i as f64, j as f64, k as f64) * cell;
                    let series = options.profile_series;
                    let name = format!("T-slot {series}{series} x {} mm", length.round());
                    plan.elements.push(Element {
                        kind: ElementKind::Profile,
                        name: name.clone(),
                        position: corner,
                        studs_x: 0,
                        studs_y: 0,
                        length,
                        axis,
                    });
                    *plan.bill_of_materials.entry(name).or_insert(0) += 1;
                    let mut hi = [corner.x + cell, corner.y + cell, corner.z + cell];
                    hi[axis] = [corner.x, corner.y, corner.z][axis] + length;
                    profile_boxes.push((corner, Vec3::new(hi[0], hi[1], hi[2])));
                }
            }
        }
        if profile_boxes.is_empty() {
            let series = options.profile_series;
            plan.warnings.push(format!(
                "No T-slot {series}{series} run fits: one needs a straight solid run of {:.0} mm with a {series} x {series} mm section inside the body ({:.0} x {:.0} x {:.0} mm).",
                options.min_profile_length, size.x, size.y, size.z
            ));
        }
        // Stud cells inside a claimed profile are consumed and counted as covered.
        for k in 0..grid.nz {
            for j in 0..grid.ny {
                for i in 0..grid.nx {
                    let index = grid.index(i, j, k);
                    if !grid.cell[index] {
                        continue;
                    }
                    let c = grid.origin + Vec3::new((i as f64 + 0.5) * pitch, (j as f64 + 0.5) * pitch, (k as f64 + 0.5) * layer);
                    let tol = 1e-9;
                    if profile_boxes.iter().any(|(lo, hi)| {
                        c.x > lo.x - tol && c.x < hi.x + tol && c.y > lo.y - tol && c.y < hi.y + tol && c.z > lo.z - tol && c.z < hi.z + tol
                    }) {
                        grid.cell[index] = false;
                        plan.cells_covered += 1;
                    }
                }
            }
        }
    }

    // --- bricks, layer by layer, orientation alternating for interlock ---
    if options.use_bricks {
        let mut covered = vec![false; grid.cell.len()];
        for k in 0..grid.nz {
            let x_major = k % 2 == 0;
            for j in 0..grid.ny {
                for i in 0..grid.nx {
                    if !grid.cell[grid.index(i, j, k)] || covered[grid.index(i, j, k)] {
                        continue;
                    }
                    for size in BRICK_SIZES {
                        let (sx, sy) = if x_major { (size[1], size[0]) } else { (size[0], size[1]) };
                        let fits = (0..sy).all(|dy| (0..sx).all(|dx| grid.solid(i + dx, j + dy, k) && !covered[grid.index(i + dx, j + dy, k)]));
                        if !fits {
                            continue;
                        }
                        for dy in 0..sy {
                            for dx in 0..sx {
                                covered[grid.index(i + dx, j + dy, k)] = true;
                            }
                        }
                        plan.cells_covered += sx * sy;
                        let corner = grid.origin + Vec3::new(i as f64 * pitch, j as f64 * pitch, k as f64 * layer);
                        let name = format!("{} {}x{}", options.brick.piece(), size[0].min(size[1]), size[0].max(size[1]));
                        plan.elements.push(Element {
                            kind: ElementKind::Brick,
                            name: name.clone(),
                            position: corner,
                            studs_x: sx,
                            studs_y: sy,
                            length: 0.0,
                            axis: 2,
                        });
                        *plan.bill_of_materials.entry(name).or_insert(0) += 1;
                        break;
                    }
                }
            }
        }
        if !plan.elements.iter().any(|e| e.kind == ElementKind::Brick) {
            plan.warnings.push(format!(
                "No {} fits: the body needs a full stud cell ({pitch} mm pitch, {layer} mm high) inside it that no profile claims.",
                options.brick.label().to_ascii_lowercase()
            ));
        }
    }

    // --- threaded rods clamping tall columns, hex bolts the short ones ---
    if options.use_rods || options.use_bolts {
        let spacing = options.rod_spacing_studs.max(1);
        let rod_min = options.rod_min_layers.max(1);
        let bolt_min = options.bolt_min_layers.max(1);
        // The lattice is centred on the body, so a part narrower than the spacing still gets
        // its middle column clamped.
        let on_lattice = |i: usize, n: usize| (i as i64 - (n / 2) as i64).rem_euclid(spacing as i64) == 0;
        for j in 0..grid.ny {
            for i in 0..grid.nx {
                if !on_lattice(i, grid.nx) || !on_lattice(j, grid.ny) {
                    continue;
                }
                let (start, layers) = grid.longest_column(i, j);
                if layers == 0 {
                    continue;
                }
                let x = grid.origin.x + (i as f64 + 0.5) * pitch;
                let y = grid.origin.y + (j as f64 + 0.5) * pitch;
                let z0 = grid.origin.z + start as f64 * layer;
                let column = layers as f64 * layer;
                if options.use_rods && layers >= rod_min {
                    // A rod stands one diameter past each end for its nut and washer.
                    let d = options.rod_size as f64;
                    let length = column + 2.0 * d;
                    let name = format!("Threaded rod M{} x {} mm", options.rod_size, length.round());
                    plan.elements.push(Element {
                        kind: ElementKind::Rod,
                        name: name.clone(),
                        position: Vec3::new(x, y, z0 - d),
                        studs_x: 0,
                        studs_y: 0,
                        length,
                        axis: 2,
                    });
                    *plan.bill_of_materials.entry(name).or_insert(0) += 1;
                    *plan.bill_of_materials.entry(format!("Nut M{}", options.rod_size)).or_insert(0) += 2;
                    *plan.bill_of_materials.entry(format!("Washer M{}", options.rod_size)).or_insert(0) += 2;
                } else if options.use_bolts && layers >= bolt_min {
                    // The head sits on the column; the thread passes through it and takes a
                    // washer and a nut below: the next stock length above the column plus them.
                    let d = options.bolt_size as f64;
                    let Some(length) = BOLT_LENGTHS.iter().copied().find(|l| *l >= column + 1.5 * d) else {
                        continue;
                    };
                    let name = format!("Hex bolt M{} x {} mm", options.bolt_size, length.round());
                    let top = z0 + column;
                    plan.elements.push(Element {
                        kind: ElementKind::Bolt,
                        name: name.clone(),
                        position: Vec3::new(x, y, top - length),
                        studs_x: 0,
                        studs_y: 0,
                        length,
                        axis: 2,
                    });
                    *plan.bill_of_materials.entry(name).or_insert(0) += 1;
                    *plan.bill_of_materials.entry(format!("Nut M{}", options.bolt_size)).or_insert(0) += 1;
                    *plan.bill_of_materials.entry(format!("Washer M{}", options.bolt_size)).or_insert(0) += 2;
                }
            }
        }
        if !plan.elements.iter().any(|e| matches!(e.kind, ElementKind::Rod | ElementKind::Bolt)) {
            let mut wants = Vec::new();
            if options.use_rods {
                wants.push(format!("{rod_min} layers ({:.1} mm) for a threaded rod M{}", rod_min as f64 * layer, options.rod_size));
            }
            if options.use_bolts {
                wants.push(format!("{bolt_min} layers ({:.1} mm) for a hex bolt M{}", bolt_min as f64 * layer, options.bolt_size));
            }
            plan.warnings.push(format!(
                "Nothing clamps the stack: no column on the stud grid (every {spacing} studs, centred on the body) stands {}.",
                wants.join(" or ")
            ));
        }
    }
    plan.coverage = if plan.cells_total > 0 { plan.cells_covered as f64 / plan.cells_total as f64 } else { 0.0 };
    Some(plan)
}

/// The T-slot profile outline of a series (20, 30, 40) in mm, counter-clockwise, and its centre
/// bore diameter: proportions scaled from the common 2020 family. Each side has a slot opening,
/// a lip, and a T cavity whose inner corners are chamfered at 45° so the cavities of adjacent
/// sides leave a diagonal web between them (the outline never crosses itself).
pub fn tslot_outline(series: u32) -> Option<(Vec<[f64; 2]>, f64)> {
    if !PROFILE_SERIES.contains(&series) {
        return None;
    }
    let a = series as f64;
    let half = a / 2.0;
    let w = 0.30 * half; // slot opening half-width
    let lip = 0.18 * half; // lip depth to the T cavity
    let tw = 0.62 * half; // T cavity half-width under the lip
    let td = 0.50 * half; // T cavity depth from the surface
    let bw = tw - (td - lip); // cavity half-width at its bottom, after the 45° chamfers
    let bore = match series {
        20 => 4.2,
        30 => 6.8,
        _ => 8.4,
    };
    let side: [[f64; 2]; 9] = [
        [-half, half],
        [-w, half],
        [-w, half - lip],
        [-tw, half - lip],
        [-bw, half - td],
        [bw, half - td],
        [tw, half - lip],
        [w, half - lip],
        [w, half],
    ];
    let mut outline = Vec::with_capacity(36);
    for rotation in 0..4 {
        for [x, y] in side {
            outline.push(match rotation {
                0 => [x, y],
                1 => [y, -x],
                2 => [-x, -y],
                _ => [-y, x],
            });
        }
    }
    Some((outline, bore))
}

/// Appends a closed box to a mesh.
fn push_box(mesh: &mut Mesh, min: Vec3, size: Vec3) {
    let base = mesh.positions.len() as u32;
    let p = |x: f64, y: f64, z: f64| min + Vec3::new(x, y, z);
    let (a, b, c) = (size.x, size.y, size.z);
    mesh.positions.extend([p(0.0, 0.0, 0.0), p(a, 0.0, 0.0), p(a, b, 0.0), p(0.0, b, 0.0), p(0.0, 0.0, c), p(a, 0.0, c), p(a, b, c), p(0.0, b, c)]);
    mesh.normals.extend([Vec3::ZERO; 8]);
    let tris: [[u32; 3]; 12] =
        [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7]];
    let face = mesh.tri_face.last().map(|f| f + 1).unwrap_or(0);
    for (t, tri) in tris.iter().enumerate() {
        mesh.triangles.push([base + tri[0], base + tri[1], base + tri[2]]);
        mesh.tri_face.push(face + (t / 2) as u32);
    }
}

/// Appends a closed prism: `outline` (mm, counter-clockwise, star-shaped about its centroid) at
/// `origin`, extruded `height` along +Z.
pub fn push_prism(mesh: &mut Mesh, outline: &[[f64; 2]], origin: Vec3, height: f64) {
    let n = outline.len() as u32;
    if n < 3 {
        return;
    }
    let base = mesh.positions.len() as u32;
    for [x, y] in outline {
        mesh.positions.push(origin + Vec3::new(*x, *y, 0.0));
    }
    for [x, y] in outline {
        mesh.positions.push(origin + Vec3::new(*x, *y, height));
    }
    mesh.normals.extend(std::iter::repeat_n(Vec3::ZERO, 2 * n as usize));
    let face = mesh.tri_face.last().map(|f| f + 1).unwrap_or(0);
    // Caps as fans (fine for a convex or star-shaped polygon about its centroid; the stud and the
    // profile are both star-shaped about the origin) and the sides as quads.
    for t in 1..n - 1 {
        mesh.triangles.push([base, base + t + 1, base + t]);
        mesh.tri_face.push(face);
        mesh.triangles.push([base + n, base + n + t, base + n + t + 1]);
        mesh.tri_face.push(face + 1);
    }
    for i in 0..n {
        let j = (i + 1) % n;
        mesh.triangles.push([base + i, base + j, base + n + j]);
        mesh.triangles.push([base + i, base + n + j, base + n + i]);
        mesh.tri_face.push(face + 2 + i);
        mesh.tri_face.push(face + 2 + i);
    }
}

/// A regular polygon approximating a circle, counter-clockwise.
pub fn circle(radius: f64, sides: usize) -> Vec<[f64; 2]> {
    (0..sides)
        .map(|i| {
            let a = 2.0 * std::f64::consts::PI * i as f64 / sides as f64;
            [radius * a.cos(), radius * a.sin()]
        })
        .collect()
}

/// The plan as geometry (mm): one closed shell per brick, stud, profile, rod and bolt (shank
/// and hex head), in one mesh.
pub fn plan_mesh(plan: &Plan) -> Mesh {
    let mut mesh = Mesh::default();
    let pitch = plan.brick.pitch();
    let layer = plan.brick.height();
    let (stud_d, stud_h) = plan.brick.stud();
    let cell = plan.profile_series as f64;
    for e in &plan.elements {
        match e.kind {
            ElementKind::Brick => {
                let size = Vec3::new(e.studs_x as f64 * pitch - BRICK_CLEARANCE, e.studs_y as f64 * pitch - BRICK_CLEARANCE, layer);
                push_box(&mut mesh, e.position + Vec3::new(BRICK_CLEARANCE / 2.0, BRICK_CLEARANCE / 2.0, 0.0), size);
                for dy in 0..e.studs_y {
                    for dx in 0..e.studs_x {
                        let centre = e.position + Vec3::new((dx as f64 + 0.5) * pitch, (dy as f64 + 0.5) * pitch, layer);
                        push_prism(&mut mesh, &circle(stud_d / 2.0, 16), centre, stud_h);
                    }
                }
            }
            ElementKind::Profile => {
                if let Some((outline, _)) = tslot_outline(plan.profile_series) {
                    // The outline is drawn in the plane across the run; turn it onto the run's axis.
                    let centre = e.position + Vec3::new(cell / 2.0, cell / 2.0, cell / 2.0);
                    match e.axis {
                        2 => push_prism(&mut mesh, &outline, Vec3::new(centre.x, centre.y, e.position.z), e.length),
                        axis => {
                            // Build along +Z then swap axes by rewriting the appended vertices.
                            let start = mesh.positions.len();
                            push_prism(&mut mesh, &outline, Vec3::ZERO, e.length);
                            for p in &mut mesh.positions[start..] {
                                let local = *p;
                                *p = if axis == 0 {
                                    Vec3::new(e.position.x + local.z, centre.y + local.x, centre.z + local.y)
                                } else {
                                    Vec3::new(centre.x + local.x, e.position.y + local.z, centre.z + local.y)
                                };
                            }
                            // Swapping axes mirrors the winding for one of the two cases.
                            if axis == 1 {
                                let first = start_triangles(&mesh, start);
                                for t in &mut mesh.triangles[first..] {
                                    t.swap(1, 2);
                                }
                            }
                        }
                    }
                }
            }
            ElementKind::Rod => {
                push_prism(&mut mesh, &circle(plan.rod_size as f64 / 2.0, 20), e.position, e.length);
            }
            ElementKind::Bolt => {
                let d = plan.bolt_size as f64;
                push_prism(&mut mesh, &circle(d / 2.0, 16), e.position, e.length);
                // The hex head: across corners about 1.9 d, 0.7 d tall, on the thread's upper end.
                push_prism(&mut mesh, &circle(0.95 * d, 6), e.position + Vec3::new(0.0, 0.0, e.length), 0.7 * d);
            }
        }
    }
    mesh
}

/// The first triangle index whose vertices start at `vertex_start`.
fn start_triangles(mesh: &Mesh, vertex_start: usize) -> usize {
    mesh.triangles.iter().position(|t| t[0] as usize >= vertex_start).unwrap_or(mesh.triangles.len())
}

#[cfg(test)]
#[path = "construction_tests.rs"]
mod tests;
