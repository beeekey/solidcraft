//! The forms of the 3DEEE tab: every generator, builder and check that has no dialog of its own
//! gets one here, as a table of fields for [`CalcForm`] (see `dialogs_calc`). The generators
//! apply on OK; the builders (lattice, brick kit, generative design) run from a button and show
//! what they built; the checks compute and show their figures. Each form opens with values
//! that build something, so a first click gives a part to look at.

use crate::dialogs_calc::{CalcField, CalcForm, Pick, PickShape};
use crate::selection::{BODIES, CURVES, EDGES, FACES};

/// The commands with a form here, by id.
pub const TOOLS: &[&str] = &[
    "design.spur_gear",
    "design.gear_rack",
    "design.bevel_gear",
    "design.worm",
    "design.worm_wheel",
    "design.crown_gear",
    "design.shaft",
    "design.spring",
    "design.pulley",
    "design.belt",
    "design.sprocket",
    "design.chain",
    "design.tslot",
    "design.stepper_motor",
    "design.snap_hook",
    "design.insert_boss",
    "design.bamboo_culm",
    "construct.brickify",
    "design.frame",
    "design.pipe_route",
    "design.fillet_weld",
    "construct.lattice",
    "generative.study",
    "generative.free_form",
    "generative.check",
    "inspect.fibre",
    "inspect.print",
    "inspect.print_orient",
    "inspect.printed_stress",
    "inspect.compare_infill",
    "inspect.infill_section",
    "inspect.machining",
    "print.elephant_foot",
    "print.hole_compensation",
    "print.split",
    "construct.topology_optimize",
    "plastic.grill",
    "construct.variable_shell",
    "construct.blend_boolean",
    "inspect.stress_zone",
    "inspect.probe",
    "inspect.export_fem",
    "design.bamboo_connector",
    "design.bamboo_frame",
    "design.bearing",
    "design.damper",
    "design.frame_end_cap",
    "design.frame_notch",
    "design.frame_mitre",
    "design.frame_change_section",
    "design.frame_extend",
    "design.groove_weld",
    "canvas.remove_background",
    "canvas.select_object",
    "canvas.rectify",
    "inspect.ray_trace",
    "sketch.trace_canvas",
];

const BELT_STANDARDS: &[&str] =
    &["GT2-2M", "GT2-3M", "GT2-5M", "HTD-3M", "HTD-5M", "HTD-8M", "HTD-14M", "T2.5", "T5", "T10", "AT5", "AT10", "MXL", "XL", "L", "H"];
const CHAIN_STANDARDS: &[&str] = &["05B", "06B", "08B", "10B", "12B", "ANSI 25", "ANSI 35", "ANSI 40", "ANSI 50", "ANSI 60"];
const SPRING_KINDS: &[&str] = &["compression", "extension", "torsion"];
const BORE_STYLES: &[&str] = &["plain", "keyway", "set_screw"];
const FRAMES: &[&str] = &["NEMA 8", "NEMA 11", "NEMA 14", "NEMA 17", "NEMA 23"];
const INSERT_SIZES: &[&str] = &["M2", "M2.5", "M3", "M4", "M5", "M6"];
const PROFILE_FAMILIES: &[&str] = &["square_tube", "rect_tube", "round_tube", "round_rod", "angle", "channel", "i_beam", "tslot"];
/// Brickify's hardware systems: the brick grids, T-slot series, rod and bolt sizes the
/// construction planner knows, worded as `construct.brickify` reads them.
const BRICK_SYSTEMS: &[&str] = &["Bricks (8 mm studs, 9.6 mm high)", "Plates (8 mm studs, 3.2 mm thin)", "Large bricks (16 mm studs, 19.2 mm high)"];
const PROFILE_SERIES: &[&str] = &["2020", "3030", "4040"];
const ROD_SIZES: &[&str] = &["M6", "M8", "M10", "M12"];
const BOLT_SIZES: &[&str] = &["M3", "M4", "M5", "M6", "M8"];
const FRAME_ENDS: &[&str] = &["mitre", "butt"];
const LATTICE_FAMILIES: &[&str] = &["walled_tpms", "solid_tpms", "beam", "honeycomb"];
const LATTICE_CELLS: &[&str] = &[
    "",
    "gyroid",
    "schwarz_p",
    "diamond",
    "neovius",
    "lidinoid",
    "split_p",
    "iwp",
    "simple_cubic",
    "body_centred",
    "face_centred",
    "octet",
    "kelvin",
];
const AXES: &[&str] = &["", "X", "Y", "Z"];
const GRADINGS: &[&str] = &["uniform", "along_axis", "from_point", "by_stress"];
const GOALS: &[&str] = &["carry_load", "remove_heat", "low_drag"];
const FINISHES: &[&str] = &["", "Polished", "Ground", "Machined", "Hot-rolled", "As forged", "As printed"];
const SPECIES: &[&str] = &["Tonkin", "Moso", "Guadua", "Generic bamboo"];
const FILAMENTS: &[&str] = &["PLA", "PLA+", "PETG", "ABS", "ASA", "TPU 95A"];
const INFILL_PATTERNS: &[&str] =
    &["Grid", "Lines", "Rectilinear", "Triangles", "Tri-hexagon", "Cubic", "Gyroid", "Honeycomb", "Concentric", "Cross-hatch", "Lightning"];
const PROCESSES: &[&str] = &["auto", "milling", "turning"];
const SPLIT_AXES: &[&str] = &["Z", "X", "Y"];
const SHELL_OPS: &[&str] = &["shell", "offset"];
const SHELL_GRADINGS: &[&str] = &["uniform", "along_axis", "from_point"];
const BOOLEANS: &[&str] = &["join", "cut", "intersect"];
const FRAME_KINDS: &[&str] = &["bicycle", "trailer"];
const FILLS: &[&str] = &["white", "black", "transparent"];
const MEMBER_ENDS: &[&str] = &["end", "start", "both"];
const MEMBER_END: &[&str] = &["end", "start"];
const BEARINGS: &[&str] = &[
    "625",
    "608",
    "6000",
    "6001",
    "6002",
    "6003",
    "6200",
    "6201",
    "6202",
    "6204",
    "6802",
    "6803",
    "6805",
    "6806",
    "6900",
    "6901",
    "6902",
    "6903",
    "Headset 41.8",
    "Headset 41.0",
    "Headset 51.8",
];
const CONNECTOR_ARMS: &str =
    r#"[{"direction": [1, 0, 0], "diameter_mm": 32}, {"direction": [0, 1, 0], "diameter_mm": 32}, {"direction": [0, 0, 1], "diameter_mm": 24}]"#;
const STOCK_MATERIALS: &[&str] = &["", "Aluminium", "Steel", "Plastic"];

/// The bracket every generative form starts with: held on a wall at the left, pushed down at
/// the right end.
const MOUNTS: &str = r#"[{"box": {"min": [0, 0, 0], "max": [12, 40, 60]}, "name": "wall"}]"#;
const LOADS: &str = r#"[{"box": {"min": [108, 0, 0], "max": [120, 40, 12]}, "name": "tip", "force_n": [0, 0, -1000]}]"#;
const DESIGN_SPACE: &str = r#"{"box": {"min": [0, 0, 0], "max": [120, 40, 60]}}"#;

fn body(label: &'static str) -> Pick {
    Pick { key: "body", label, accept: BODIES, multi: false, required: false, shape: PickShape::Body }
}

fn face(key: &'static str, label: &'static str) -> Pick {
    Pick { key, label, accept: FACES, multi: false, required: true, shape: PickShape::Point }
}

fn lines(key: &'static str, label: &'static str, shape: PickShape) -> Pick {
    Pick { key, label, accept: CURVES, multi: true, required: false, shape }
}

/// The mounts and loads of a generative form: faces of an existing body, the loads carrying
/// the force typed in the hidden `_force_n` field. Left empty, the JSON regions apply.
fn regions(picks: &mut Vec<Pick>) {
    picks.push(Pick { key: "mounts", label: "Mount faces", accept: FACES, multi: true, required: false, shape: PickShape::Regions { force: None } });
    picks.push(Pick {
        key: "loads",
        label: "Load faces",
        accept: FACES,
        multi: true,
        required: false,
        shape: PickShape::Regions { force: Some("_force_n") },
    });
}

fn force() -> CalcField {
    CalcField::point("_force_n", "Force on the load faces (N): x, y, z", "0, 0, -1000")
}

fn name() -> CalcField {
    CalcField::text("name", "Name", "")
}

/// The form for a tool command, `None` for any other command.
pub fn form(id: &str) -> Option<CalcForm> {
    let mut apply = true;
    let mut button = "Compute";
    let mut picks: Vec<Pick> = Vec::new();
    let (command, title, about, fields): (&'static str, &'static str, &'static str, Vec<CalcField>) = match id {
        "design.spur_gear" => (
            "design.spur_gear",
            "SPUR GEAR",
            "an involute gear in its own component; a helix angle makes it helical, herringbone two helices meeting",
            vec![
                CalcField::number("module", "Module (mm)", "2"),
                CalcField::number("teeth", "Teeth", "20"),
                CalcField::number("width", "Width (mm)", "10"),
                CalcField::number("pressure_angle_deg", "Pressure angle (°)", "20"),
                CalcField::number("bore_mm", "Bore (mm)", "8"),
                CalcField::number("helix_deg", "Helix angle (°)", ""),
                CalcField::flag("herringbone", "Herringbone", false),
                CalcField::flag("internal", "Internal (ring gear)", false),
                CalcField::number("rim_mm", "Rim (mm)", ""),
                name(),
            ],
        ),
        "design.gear_rack" => (
            "design.gear_rack",
            "GEAR RACK",
            "a straight rack that meshes with a spur gear of the same module",
            vec![
                CalcField::number("module", "Module (mm)", "2"),
                CalcField::number("teeth", "Teeth", "10"),
                CalcField::number("width", "Width (mm)", "8"),
                CalcField::number("back_mm", "Back (mm)", ""),
                CalcField::number("pressure_angle_deg", "Pressure angle (°)", "20"),
                name(),
            ],
        ),
        "design.bevel_gear" => (
            "design.bevel_gear",
            "BEVEL GEAR",
            "a straight bevel gear; 45° makes one of a mitre pair",
            vec![
                CalcField::number("module", "Module (mm)", "2"),
                CalcField::number("teeth", "Teeth", "20"),
                CalcField::number("pitch_angle_deg", "Pitch angle (°)", "45"),
                CalcField::number("face_width", "Face width (mm)", "8"),
                CalcField::number("pressure_angle_deg", "Pressure angle (°)", "20"),
                CalcField::number("bore_mm", "Bore (mm)", ""),
                name(),
            ],
        ),
        "design.worm" => (
            "design.worm",
            "WORM",
            "the driving screw of a worm drive, along +Z; mates with a worm wheel of the same module",
            vec![
                CalcField::number("module", "Module (mm)", "2"),
                CalcField::number("pitch_diameter", "Pitch diameter (mm)", "20"),
                CalcField::number("length", "Length (mm)", "40"),
                CalcField::number("starts", "Starts", "1"),
                CalcField::number("pressure_angle_deg", "Pressure angle (°)", "20"),
                CalcField::flag("left_hand", "Left hand", false),
                CalcField::number("bore_mm", "Bore (mm)", ""),
                name(),
            ],
        ),
        "design.worm_wheel" => (
            "design.worm_wheel",
            "WORM WHEEL",
            "the wheel a worm drives: helical teeth at the worm's lead angle, the rim throated round the worm",
            vec![
                CalcField::number("module", "Module (mm)", "2"),
                CalcField::number("teeth", "Teeth", "30"),
                CalcField::number("worm_pitch_diameter", "Worm pitch diameter (mm)", "20"),
                CalcField::number("face_width", "Face width (mm)", "12"),
                CalcField::number("worm_starts", "Worm starts", "1"),
                CalcField::number("pressure_angle_deg", "Pressure angle (°)", "20"),
                CalcField::flag("left_hand", "Left hand", false),
                CalcField::number("bore_mm", "Bore (mm)", "10"),
                name(),
            ],
        ),
        "design.crown_gear" => (
            "design.crown_gear",
            "CROWN GEAR",
            "teeth on the flat face of a disc, driven by a spur pinion of the same module at right angles",
            vec![
                CalcField::number("module", "Module (mm)", "2"),
                CalcField::number("teeth", "Teeth", "30"),
                CalcField::number("face_width", "Face width (mm)", "10"),
                CalcField::number("thickness", "Thickness (mm)", "8"),
                CalcField::number("pressure_angle_deg", "Pressure angle (°)", "20"),
                CalcField::number("bore_mm", "Bore (mm)", "8"),
                name(),
            ],
        ),
        "design.shaft" => (
            "design.shaft",
            "SHAFT",
            "sections chained along +Z, each with a keyway or a circlip groove in its middle",
            vec![
                CalcField::json(
                    "sections",
                    "Sections",
                    "[{\"diameter\": 20, \"length\": 30, \"keyway\": {\"width\": 6, \"depth\": 3.5}},\n {\"diameter\": 25, \"length\": 40, \"groove\": {\"width\": 1.3, \"depth\": 1}},\n {\"diameter\": 20, \"length\": 30}]",
                ),
                CalcField::number("chamfer_mm", "End chamfer (mm)", "1"),
                name(),
            ],
        ),
        "design.spring" => (
            "design.spring",
            "SPRING",
            "a compression spring with closed end coils, an extension spring with hooks, or a torsion spring with legs; the rate is reported",
            vec![
                CalcField::choice("kind", "Kind", SPRING_KINDS, 0),
                CalcField::number("mean_diameter", "Mean diameter (mm)", "20"),
                CalcField::number("wire_diameter", "Wire diameter (mm)", "2"),
                CalcField::number("coils", "Active coils", "6"),
                CalcField::number("free_length", "Free length (mm)", "50"),
                CalcField::number("end_coils", "End coils", "1"),
                CalcField::number("leg_mm", "Legs (mm)", "20"),
                name(),
            ],
        ),
        "design.pulley" => (
            "design.pulley",
            "PULLEY",
            "a toothed pulley of a belt standard, with a bore, flanges, or a plain roller",
            vec![
                CalcField::choice("standard", "Standard", BELT_STANDARDS, 0),
                CalcField::number("teeth", "Teeth", "20"),
                CalcField::number("width", "Width (mm)", "7"),
                CalcField::number("bore_mm", "Bore (mm)", "5"),
                CalcField::choice("bore_style", "Bore style", BORE_STYLES, 0),
                CalcField::flag("flanges", "Flanges", true),
                CalcField::number("flange_mm", "Flange (mm)", "1.5"),
                CalcField::point("centre", "Centre (x, y, z)", "0, 0, 0"),
                name(),
            ],
        ),
        "design.belt" => (
            "design.belt",
            "BELT",
            "the belt round the wheels in order (toothed pulleys by teeth, rollers by diameter); the matching standard belt is reported",
            vec![
                CalcField::choice("standard", "Standard", BELT_STANDARDS, 0),
                CalcField::number("width", "Width (mm)", "6"),
                CalcField::json("wheels", "Wheels", "[{\"centre\": [0, 0], \"teeth\": 20},\n {\"centre\": [100, 0], \"teeth\": 20}]"),
                CalcField::number("z", "Mid-plane z (mm)", "0"),
                CalcField::flag("allow_crossing", "Allow a crossed belt", false),
                CalcField::flag("build", "Build the belt", true),
            ],
        ),
        "design.sprocket" => (
            "design.sprocket",
            "SPROCKET",
            "an ISO 606 roller-chain sprocket",
            vec![
                CalcField::choice("standard", "Chain", CHAIN_STANDARDS, 2),
                CalcField::number("teeth", "Teeth", "17"),
                CalcField::number("bore_mm", "Bore (mm)", "10"),
                CalcField::point("centre", "Centre (x, y, z)", "0, 0, 0"),
                name(),
            ],
        ),
        "design.chain" => (
            "design.chain",
            "CHAIN",
            "the chain between two sprockets: the links that fit, their length, and the rollers and plates as bodies",
            vec![
                CalcField::choice("standard", "Chain", CHAIN_STANDARDS, 2),
                CalcField::json("sprockets", "Sprockets", "[{\"centre\": [0, 0], \"teeth\": 17},\n {\"centre\": [200, 0], \"teeth\": 17}]"),
                CalcField::number("links", "Links", ""),
                CalcField::number("z", "Mid-plane z (mm)", "0"),
                CalcField::flag("build", "Build the chain", true),
            ],
        ),
        "design.tslot" => (
            "design.tslot",
            "T-SLOT PROFILE",
            "an aluminium extrusion of the 20, 30 or 40 series along +Z",
            vec![CalcField::number("series", "Series (20, 30, 40)", "20"), CalcField::number("length", "Length (mm)", "100"), name()],
        ),
        "design.stepper_motor" => (
            "design.stepper_motor",
            "STEPPER MOTOR",
            "a NEMA frame motor with its boss and shaft along +Z from the front face at z = 0",
            vec![
                CalcField::choice("frame", "Frame", FRAMES, 3),
                CalcField::number("body_length_mm", "Body length (mm)", ""),
                CalcField::number("shaft_length_mm", "Shaft length (mm)", ""),
                CalcField::flag("shaft_flat", "Shaft flat", true),
                name(),
            ],
        ),
        "design.snap_hook" => (
            "design.snap_hook",
            "SNAP HOOK",
            "a cantilever snap hook: the arm and the hook with its ramp",
            vec![
                CalcField::number("width", "Width (mm)", "6"),
                CalcField::number("arm_length", "Arm length (mm)", "20"),
                CalcField::number("thickness", "Thickness (mm)", "2"),
                CalcField::number("hook_depth", "Hook depth (mm)", "1.5"),
                CalcField::number("hook_length", "Hook length (mm)", "3"),
                name(),
            ],
        ),
        "design.insert_boss" => (
            "design.insert_boss",
            "HEAT-SET INSERT BOSS",
            "a boss bored for a heat-set threaded insert of the size",
            vec![CalcField::choice("size", "Insert", INSERT_SIZES, 2), CalcField::number("height", "Height (mm)", "10"), name()],
        ),
        "design.bamboo_culm" => (
            "design.bamboo_culm",
            "BAMBOO CULM",
            "a tapering hollow culm along +Z with nodes, collars and diaphragms",
            vec![
                CalcField::number("length", "Length (mm)", "500"),
                CalcField::number("butt_diameter", "Butt diameter (mm)", "32"),
                CalcField::number("tip_diameter", "Tip diameter (mm)", "28"),
                CalcField::number("butt_wall", "Butt wall (mm)", "8"),
                CalcField::number("tip_wall", "Tip wall (mm)", "7"),
                CalcField::number("node_spacing", "Node spacing (mm)", "250"),
                CalcField::number("first_node", "First node (mm)", "125"),
                CalcField::number("node_collar", "Collar proud (mm)", "1.5"),
                CalcField::number("node_collar_height", "Collar height (mm)", "8"),
                CalcField::number("diaphragm", "Diaphragm (mm)", "4"),
                name(),
            ],
        ),
        "construct.brickify" => {
            apply = false;
            button = "Build";
            picks.push(body("Body"));
            (
                "construct.brickify",
                "BRICKIFY",
                "the body laid out on a stud grid and replaced by interlocking bricks, T-slot profile runs, threaded rods and bolts of the systems you pick, with the bill of materials",
                vec![
                    CalcField::flag("bricks", "Bricks", true),
                    CalcField::choice("brick", "Brick system", BRICK_SYSTEMS, 0),
                    CalcField::flag("profiles", "T-slot profiles for straight runs", true),
                    CalcField::choice("profile_series", "Profile series", PROFILE_SERIES, 0),
                    CalcField::number("min_profile_mm", "Shortest profile run (mm)", "40"),
                    CalcField::flag("rods", "Threaded rods for tall columns", true),
                    CalcField::choice("rod", "Rod size", ROD_SIZES, 1),
                    CalcField::flag("bolts", "Hex bolts for short columns", true),
                    CalcField::choice("bolt", "Bolt size", BOLT_SIZES, 1),
                    CalcField::flag("build", "Build the kit", true),
                    CalcField::flag("hide", "Hide the source body", false),
                    name(),
                ],
            )
        }
        "design.frame" => {
            picks.push(lines("members", "Sketch lines", PickShape::Lines));
            (
                "design.frame",
                "FRAME",
                "members of a catalogue profile along the picked sketch lines (or the members listed), mitred where two meet at an end and trimmed where one ends on another",
                vec![
                    CalcField::choice("profile.family", "Profile", PROFILE_FAMILIES, 0),
                    CalcField::text("profile.size", "Size", "40x40x3"),
                    CalcField::json(
                        "members",
                        "Members",
                        "[{\"from\": [0, 0, 0], \"to\": [400, 0, 0]},\n {\"from\": [400, 0, 0], \"to\": [400, 300, 0]},\n {\"from\": [400, 300, 0], \"to\": [0, 300, 0]},\n {\"from\": [0, 300, 0], \"to\": [0, 0, 0]}]",
                    ),
                    CalcField::choice("ends", "Ends", FRAME_ENDS, 0),
                    CalcField::flag("trim", "Trim members ending on another", true),
                    CalcField::number("rotation_deg", "Profile rotation (°)", ""),
                    name(),
                ],
            )
        }
        "design.pipe_route" => {
            picks.push(lines("points", "Sketch lines", PickShape::Waypoints));
            (
                "design.pipe_route",
                "PIPE ROUTE",
                "a tube along the picked sketch lines joined end to end (or the waypoints listed), straight runs with tangent bends, as a Pipe on a 3D sketch",
                vec![
                    CalcField::json("points", "Waypoints", "[[0, 0, 0], [100, 0, 0], [100, 100, 0]]"),
                    CalcField::number("outer_diameter", "Outer diameter (mm)", "12"),
                    CalcField::number("wall", "Wall (mm)", "1"),
                    CalcField::number("bend_radius", "Bend radius (mm)", "24"),
                    name(),
                ],
            )
        }
        "design.fillet_weld" => {
            picks.push(Pick { key: "body", label: "Body", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            picks.push(Pick { key: "edge", label: "Edge", accept: EDGES, multi: false, required: true, shape: PickShape::Point });
            (
                "design.fillet_weld",
                "FILLET WELD",
                "a bead swept along an inside corner's edge of the body, straight or curved; an outside corner is refused",
                vec![CalcField::number("leg_mm", "Leg (mm)", "4"), name()],
            )
        }
        "construct.lattice" => {
            apply = false;
            button = "Build";
            picks.push(body("Body"));
            (
                "construct.lattice",
                "LATTICE",
                "the body filled with a TPMS sheet or solid, struts along a cell's edges or a honeycomb, graded if asked, as a mesh body",
                vec![
                    CalcField::choice("family", "Family", LATTICE_FAMILIES, 0),
                    CalcField::choice("cell", "Cell", LATTICE_CELLS, 0),
                    CalcField::number("cell_mm", "Cell (mm)", "10"),
                    CalcField::number("thickness_mm", "Wall or strut (mm)", "1.5"),
                    CalcField::number("level", "Level (solid TPMS)", ""),
                    CalcField::number("skin_mm", "Skin (mm)", ""),
                    CalcField::choice("axis", "Honeycomb axis", AXES, 0),
                    CalcField::choice("grading", "Grading", GRADINGS, 0),
                    CalcField::number("thickness_min_mm", "Thinnest (mm)", ""),
                    CalcField::number("thickness_max_mm", "Thickest (mm)", ""),
                    CalcField::number("samples_per_cell", "Samples per cell", "8"),
                    CalcField::flag("hide", "Hide the source body", true),
                    name(),
                ],
            )
        }
        "generative.study" => {
            apply = false;
            button = "Run";
            regions(&mut picks);
            picks.push(Pick {
                key: "design_space",
                label: "Design space body",
                accept: BODIES,
                multi: false,
                required: false,
                shape: PickShape::Body,
            });
            (
                "generative.study",
                "GENERATIVE DESIGN",
                "struts and plates between the mount faces and the load faces (or the regions listed), searched for the lightest that carries the load; the best is built",
                vec![
                    force(),
                    CalcField::json("mounts", "Mounts (if no faces picked)", MOUNTS),
                    CalcField::json("loads", "Loads (if no faces picked)", LOADS),
                    CalcField::json("design_space", "Design space (if no body picked)", DESIGN_SPACE),
                    CalcField::text("material", "Material", "Steel"),
                    CalcField::number("safety_factor", "Safety factor", "2"),
                    CalcField::choice("goal", "Goal", GOALS, 0),
                    CalcField::number("resolution", "Resolution", "14"),
                    CalcField::number("population", "Population", "10"),
                    CalcField::number("generations", "Generations", "3"),
                    CalcField::number("solutions", "Solutions kept", "2"),
                    CalcField::number("seed", "Seed", "7"),
                    CalcField::flag("gravity", "Own weight", false),
                    CalcField::number("build", "Build design", "0"),
                    CalcField::text("name", "Name", "Bracket"),
                ],
            )
        }
        "generative.free_form" => {
            apply = false;
            button = "Run";
            regions(&mut picks);
            picks.push(Pick {
                key: "design_space",
                label: "Design space body",
                accept: BODIES,
                multi: false,
                required: false,
                shape: PickShape::Body,
            });
            (
                "generative.free_form",
                "FREE-FORM BODY",
                "a density field between the mount faces and the load faces (or the regions listed), iterated to the stiffest shape of the volume fraction, built as a mesh body",
                vec![
                    force(),
                    CalcField::json("mounts", "Mounts (if no faces picked)", MOUNTS),
                    CalcField::json("loads", "Loads (if no faces picked)", LOADS),
                    CalcField::json("design_space", "Design space (if no body picked)", DESIGN_SPACE),
                    CalcField::text("material", "Material", "Steel"),
                    CalcField::number("volume_fraction", "Volume fraction", "0.3"),
                    CalcField::number("resolution", "Resolution", "16"),
                    CalcField::number("iterations", "Iterations", "8"),
                    CalcField::flag("meet_target", "Tune to the safety factor", true),
                    CalcField::flag("gravity", "Own weight", false),
                    CalcField::text("name", "Name", "Field"),
                ],
            )
        }
        "generative.check" => {
            apply = false;
            picks.push(Pick { key: "body", label: "Body", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            regions(&mut picks);
            (
                "generative.check",
                "CHECK DESIGN",
                "the body held at the mount faces and pushed at the load faces (or the regions listed): safety, buckling, fatigue and how makeable it is",
                vec![
                    force(),
                    CalcField::json("mounts", "Mounts (if no faces picked)", MOUNTS),
                    CalcField::json("loads", "Loads (if no faces picked)", LOADS),
                    CalcField::text("material", "Material", "Steel"),
                    CalcField::number("resolution", "Resolution", "20"),
                    CalcField::flag("cyclic", "Loads repeat", false),
                    CalcField::choice("finish", "Surface finish", FINISHES, 0),
                ],
            )
        }
        "inspect.fibre" => {
            apply = false;
            picks.push(body("Body"));
            picks.push(face("fixed", "Fixed face"));
            picks.push(face("load", "Load face"));
            (
                "inspect.fibre",
                "GRAIN FAILURE",
                "a material with a grain judged along, across and in shear rather than by von Mises",
                vec![
                    CalcField::point("force_n", "Force (N): x, y, z", "2000, 0, 0"),
                    CalcField::point("fibre", "Grain direction: x, y, z", "1, 0, 0"),
                    CalcField::choice("species", "Species", SPECIES, 0),
                    CalcField::number("resolution", "Resolution", "24"),
                ],
            )
        }
        "inspect.print" => {
            apply = false;
            picks.push(body("Body"));
            (
                "inspect.print",
                "PRINT ANALYSIS",
                "overhangs, support, bridges, thin walls, material and time for the body printed in this orientation",
                vec![
                    CalcField::point("up", "Build direction: x, y, z", "0, 0, 1"),
                    CalcField::number("critical_angle_deg", "Overhang angle (°)", "45"),
                    CalcField::number("layer_height_mm", "Layer height (mm)", "0.2"),
                    CalcField::choice("material", "Filament", FILAMENTS, 0),
                    CalcField::number("infill", "Infill (0 to 1)", "0.2"),
                    CalcField::choice("pattern", "Infill pattern", INFILL_PATTERNS, 0),
                    CalcField::number("walls", "Walls", "2"),
                ],
            )
        }
        "inspect.print_orient" => {
            apply = false;
            picks.push(body("Body"));
            (
                "inspect.print_orient",
                "BEST PRINT ORIENTATION",
                "the six axis directions ranked by support and height",
                vec![
                    CalcField::number("critical_angle_deg", "Overhang angle (°)", "45"),
                    CalcField::number("layer_height_mm", "Layer height (mm)", "0.2"),
                ],
            )
        }
        "inspect.printed_stress" => {
            apply = false;
            picks.push(body("Body"));
            picks.push(face("fixed", "Fixed face"));
            picks.push(face("load", "Load face"));
            (
                "inspect.printed_stress",
                "PRINTED STRESS",
                "the part as it comes off the printer: a shell around a lattice, judged within a layer and between layers",
                vec![
                    CalcField::point("force_n", "Force (N): x, y, z", "0, 0, -50"),
                    CalcField::point("up", "Build direction: x, y, z", "0, 0, 1"),
                    CalcField::choice("material", "Filament", FILAMENTS, 0),
                    CalcField::choice("pattern", "Infill pattern", INFILL_PATTERNS, 0),
                    CalcField::number("infill", "Infill (0 to 1)", "0.2"),
                    CalcField::number("walls", "Wall loops", "2"),
                    CalcField::number("line_width_mm", "Line width (mm)", "0.42"),
                    CalcField::number("layer_height_mm", "Layer height (mm)", "0.2"),
                    CalcField::number("top_bottom_layers", "Top and bottom layers", "4"),
                    CalcField::number("resolution", "Resolution", "16"),
                ],
            )
        }
        "inspect.compare_infill" => {
            apply = false;
            picks.push(body("Body"));
            picks.push(face("fixed", "Fixed face"));
            picks.push(face("load", "Load face"));
            (
                "inspect.compare_infill",
                "COMPARE INFILL",
                "the same part and load printed with every infill pattern: mass, stiffness, deflection and the two safety factors",
                vec![
                    CalcField::point("force_n", "Force (N): x, y, z", "0, 0, -50"),
                    CalcField::point("up", "Build direction: x, y, z", "0, 0, 1"),
                    CalcField::choice("material", "Filament", FILAMENTS, 0),
                    CalcField::number("infill", "Infill (0 to 1)", "0.2"),
                    CalcField::number("walls", "Wall loops", "2"),
                    CalcField::number("layer_height_mm", "Layer height (mm)", "0.2"),
                    CalcField::number("top_bottom_layers", "Top and bottom layers", "4"),
                    CalcField::number("resolution", "Resolution", "12"),
                ],
            )
        }
        "inspect.infill_section" => {
            picks.push(body("Body"));
            (
                "inspect.infill_section",
                "INFILL ON A SECTION",
                "the lattice a slicer would lay down where a plane cuts the part, with the wall loops, as a 3D sketch",
                vec![
                    CalcField::choice("axis", "Cut plane normal", SPLIT_AXES, 0),
                    CalcField::number("station_mm", "Station along it (mm, blank = middle)", ""),
                    CalcField::choice("pattern", "Infill pattern", INFILL_PATTERNS, 0),
                    CalcField::number("infill", "Infill (0 to 1)", "0.2"),
                    CalcField::number("walls", "Wall loops", "2"),
                    CalcField::number("line_width_mm", "Line width (mm)", "0.42"),
                    CalcField::text("name", "Sketch name", ""),
                ],
            )
        }
        "inspect.machining" => {
            apply = false;
            picks.push(body("Body"));
            (
                "inspect.machining",
                "MACHINING ANALYSIS",
                "tool access and setups, corner radii, holes, walls and turning rules for the body",
                vec![
                    CalcField::choice("process", "Process", PROCESSES, 0),
                    CalcField::choice("material", "Material", STOCK_MATERIALS, 0),
                    CalcField::number("cutter_mm", "Cutter (mm)", "6"),
                    CalcField::number("min_corner_radius_mm", "Smallest corner radius (mm)", "1.5"),
                    CalcField::number("resolution", "Resolution", "40"),
                ],
            )
        }
        "print.elephant_foot" => {
            picks.push(body("Body"));
            (
                "print.elephant_foot",
                "ELEPHANT FOOT",
                "chamfers the edges that sit on the plate, so the squashed first layer comes out on size",
                vec![CalcField::point("up", "Build direction: x, y, z", "0, 0, 1"), CalcField::number("distance_mm", "Chamfer (mm)", "0.3")],
            )
        }
        "print.hole_compensation" => {
            picks.push(body("Body"));
            (
                "print.hole_compensation",
                "HOLE COMPENSATION",
                "cuts every round bore wider, so printed holes come out on size; bosses and shafts are left alone",
                vec![
                    CalcField::number("delta_mm", "Wider by (mm, on the diameter)", "0.2"),
                    CalcField::number("max_diameter_mm", "Leave bores wider than (mm, 0 = none)", "0"),
                ],
            )
        }
        "print.split" => {
            picks.push(body("Body"));
            (
                "print.split",
                "SPLIT FOR PRINT",
                "cuts a body too big for the printer in two, with alignment pins on the lower half and sockets in the upper",
                vec![
                    CalcField::choice("axis", "Cut plane normal", SPLIT_AXES, 0),
                    CalcField::number("station_mm", "Station along it (mm, blank = middle)", ""),
                    CalcField::number("pin_diameter_mm", "Pin diameter (mm)", "4"),
                    CalcField::number("pin_length_mm", "Pin length (mm)", "6"),
                    CalcField::number("clearance_mm", "Socket clearance (mm)", "0.3"),
                    CalcField::flag("pins", "Alignment pins", true),
                ],
            )
        }
        "construct.topology_optimize" => {
            apply = false;
            button = "Run";
            picks.push(body("Body"));
            picks.push(face("fixed", "Fixed face"));
            picks.push(face("load", "Load face"));
            (
                "construct.topology_optimize",
                "TOPOLOGY OPTIMIZE",
                "keeps the material that carries the load, connected from the mount to the load, as a new mesh body",
                vec![
                    CalcField::point("force_n", "Force (N): x, y, z", "0, 0, -100"),
                    CalcField::number("target_fraction", "Keep (fraction of the material)", "0.5"),
                    CalcField::number("resolution", "Resolution", "24"),
                    CalcField::text("name", "Name", ""),
                    CalcField::flag("hide", "Hide the source body", false),
                ],
            )
        }
        "plastic.grill" => {
            picks.push(Pick { key: "face", label: "Wall face", accept: FACES, multi: false, required: true, shape: PickShape::Point });
            (
                "plastic.grill",
                "GRILL",
                "a vent through a flat wall: slots between ribs inside a rim, cut from a sketch on the wall",
                vec![
                    CalcField::point("direction", "Slots run along: x, y, z", "1, 0, 0"),
                    CalcField::number("width_mm", "Width along the slots (mm)", "20"),
                    CalcField::number("length_mm", "Length across them (mm)", "12"),
                    CalcField::number("slot_width_mm", "Slot width (mm)", "2"),
                    CalcField::number("rib_width_mm", "Rib width (mm)", "1.5"),
                    CalcField::number("rim_width_mm", "Rim width (mm)", "1.5"),
                    CalcField::number("rim_height_mm", "Rim stands proud (mm)", "0"),
                    CalcField::number("spar_width_mm", "Spar width (mm, 0 = none)", "0"),
                    CalcField::number("spar_pitch_mm", "Spar pitch (mm)", "0"),
                    CalcField::number("depth_mm", "Depth (mm, blank = through the wall)", ""),
                ],
            )
        }
        "construct.variable_shell" => {
            apply = false;
            button = "Build";
            picks.push(body("Body"));
            (
                "construct.variable_shell",
                "VARIABLE SHELL",
                "hollows the body to a skin, or grows it, by a thickness that may vary along an axis or from a point; a mesh body",
                vec![
                    CalcField::choice("operation", "Operation", SHELL_OPS, 0),
                    CalcField::number("thickness_mm", "Thickness (mm)", "2"),
                    CalcField::choice("grading", "Grading", SHELL_GRADINGS, 0),
                    CalcField::point("grade_from", "Grade from: x, y, z", ""),
                    CalcField::point("grade_to", "Grade to: x, y, z", ""),
                    CalcField::number("thickness_min_mm", "Thickness at the start (mm)", ""),
                    CalcField::number("thickness_max_mm", "Thickness at the end (mm)", ""),
                    CalcField::number("sample_mm", "Sample spacing (mm, blank = automatic)", ""),
                    CalcField::text("name", "Name", ""),
                    CalcField::flag("hide", "Hide the source body", false),
                ],
            )
        }
        "construct.blend_boolean" => {
            apply = false;
            button = "Build";
            picks.push(body("Body"));
            picks.push(Pick { key: "tool", label: "Tool body", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            (
                "construct.blend_boolean",
                "BLEND BOOLEAN",
                "joins, cuts or intersects two bodies on their distance fields, with the seam rounded; a mesh body",
                vec![
                    CalcField::choice("operation", "Operation", BOOLEANS, 0),
                    CalcField::number("blend_mm", "Blend (mm)", "2"),
                    CalcField::number("sample_mm", "Sample spacing (mm, blank = automatic)", ""),
                    CalcField::text("name", "Name", ""),
                    CalcField::flag("hide", "Hide the source bodies", false),
                ],
            )
        }
        "inspect.stress_zone" => {
            apply = false;
            button = "Build";
            picks.push(body("Body"));
            picks.push(face("fixed", "Fixed face"));
            picks.push(face("load", "Load face"));
            (
                "inspect.stress_zone",
                "STRESS ZONE",
                "the cells above a share of the peak stress as a body of their own: the hot spot, where a fillet or a rib belongs",
                vec![
                    CalcField::point("force_n", "Force (N): x, y, z", "0, 0, -100"),
                    CalcField::number("threshold", "Threshold (fraction of the peak)", "0.5"),
                    CalcField::number("resolution", "Resolution", "20"),
                    CalcField::text("name", "Name", ""),
                ],
            )
        }
        "inspect.probe" => {
            apply = false;
            picks.push(body("Body"));
            picks.push(face("fixed", "Fixed face"));
            picks.push(face("load", "Load face"));
            picks.push(Pick { key: "at", label: "Probe point", accept: FACES, multi: false, required: true, shape: PickShape::Point });
            (
                "inspect.probe",
                "PROBE RESULT",
                "the stress solve read at one point of the body",
                vec![CalcField::point("force_n", "Force (N): x, y, z", "0, 0, -100"), CalcField::number("resolution", "Resolution", "20")],
            )
        }
        "inspect.export_fem" => {
            apply = false;
            picks.push(body("Body"));
            picks.push(face("fixed", "Fixed face"));
            picks.push(face("load", "Load face"));
            (
                "inspect.export_fem",
                "EXPORT FEM CASE",
                "writes the voxel mesh, the held nodes, the load and the material as a complete solver deck",
                vec![
                    CalcField::point("force_n", "Force (N): x, y, z", "0, 0, -100"),
                    CalcField::number("resolution", "Resolution", "24"),
                    CalcField::text("path", "Directory", "fem_case"),
                ],
            )
        }
        "design.bamboo_connector" => (
            "design.bamboo_connector",
            "BAMBOO CONNECTOR",
            "a Muffe: a hub with a glued sleeve per culm, the bores cut to the culm plus the glue gap",
            vec![
                CalcField::json("arms", "Sockets: [{direction, diameter_mm, engagement_mm?}]", CONNECTOR_ARMS),
                CalcField::number("glue_gap_mm", "Glue gap, radial (mm)", "0.3"),
                CalcField::number("wall_mm", "Sleeve wall (mm)", "3"),
                CalcField::number("hub_diameter_mm", "Hub diameter (mm, 0 = as the sleeves)", "0"),
                name(),
            ],
        ),
        "design.bamboo_frame" => (
            "design.bamboo_frame",
            "BAMBOO FRAME",
            "a bicycle or trailer skeleton as culms, metal tubes and connectors, with the cut list in the reply",
            vec![
                CalcField::choice("kind", "Kind", FRAME_KINDS, 0),
                CalcField::number("seat_tube_mm", "Bicycle: seat tube (mm)", "540"),
                CalcField::number("seat_angle_deg", "Seat angle (°)", "73.5"),
                CalcField::number("top_tube_mm", "Top tube, horizontal (mm)", "560"),
                CalcField::number("head_tube_mm", "Head tube (mm)", "130"),
                CalcField::number("head_angle_deg", "Head angle (°)", "71"),
                CalcField::number("chain_stay_mm", "Chain stay (mm)", "430"),
                CalcField::number("bb_drop_mm", "Bottom bracket drop (mm)", "68"),
                CalcField::number("dropout_spacing_mm", "Rear dropout spacing (mm)", "135"),
                CalcField::number("main_diameter_mm", "Main culm diameter (mm)", "34"),
                CalcField::number("length_mm", "Trailer: chassis length (mm)", "900"),
                CalcField::number("width_mm", "Chassis width (mm)", "600"),
                CalcField::number("cross_members", "Cross members", "2"),
                CalcField::number("drawbar_mm", "Drawbar length (mm)", "700"),
                CalcField::number("rail_diameter_mm", "Rail culm diameter (mm)", "32"),
                CalcField::number("culm_wall_mm", "Culm wall (mm)", "8"),
                CalcField::number("engagement", "Engagement (culm diameters)", "2"),
                CalcField::number("node_spacing_mm", "Node spacing (mm, 0 = plain)", "220"),
                name(),
            ],
        ),
        "design.bearing" => (
            "design.bearing",
            "BEARING",
            "a catalogue bearing drawn to its boundary dimensions, with its load ratings in the reply",
            vec![CalcField::choice("designation", "Designation", BEARINGS, 12), CalcField::point("center", "Centre: x, y, z", "0, 0, 0"), name()],
        ),
        "design.damper" => (
            "design.damper",
            "DAMPER",
            "a shock drawn to its mounting dimensions, which is what has to fit; its ratings go to Damping and Isolation",
            vec![
                CalcField::number("eye_to_eye_mm", "Eye to eye, extended (mm)", "165"),
                CalcField::number("stroke_mm", "Stroke (mm)", "40"),
                CalcField::number("body_diameter_mm", "Body diameter (mm)", "32"),
                CalcField::number("rod_diameter_mm", "Rod diameter (mm)", "12"),
                CalcField::number("eye_bore_mm", "Eye bore (mm)", "8"),
                CalcField::number("eye_width_mm", "Eye width (mm)", "12"),
                CalcField::flag("coil_over", "Coil over", false),
                CalcField::number("coil_diameter_mm", "Coil diameter (mm)", "51"),
                CalcField::number("wire_diameter_mm", "Wire diameter (mm)", "4"),
                CalcField::number("spring_rate_n_mm", "Spring rate (N/mm)", "30"),
                CalcField::number("damping_ns_m", "Damping coefficient (N·s/m)", "600"),
                name(),
            ],
        ),
        "design.frame_end_cap" => {
            picks.push(Pick { key: "member", label: "Member", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            (
                "design.frame_end_cap",
                "END CAP",
                "a plate covering the member's cross-section at its end",
                vec![CalcField::number("thickness_mm", "Thickness (mm)", "5"), CalcField::choice("end", "End", MEMBER_ENDS, 0)],
            )
        }
        "design.frame_notch" => {
            picks.push(Pick { key: "member", label: "Member to notch", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            picks.push(Pick { key: "tool", label: "Cutting member", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            ("design.frame_notch", "NOTCH", "one member cut back around another where they cross; the cutting member is kept", vec![])
        }
        "design.frame_mitre" => {
            picks.push(Pick { key: "member_a", label: "First member", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            picks.push(Pick { key: "member_b", label: "Second member", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            ("design.frame_mitre", "MITRE", "two members cut on the plane halving their corner, so the pair closes in a clean joint", vec![])
        }
        "design.frame_change_section" => {
            picks.push(Pick { key: "member", label: "Member", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            (
                "design.frame_change_section",
                "CHANGE SECTION",
                "the member rebuilt on its own axis with a new profile; the old one is hidden",
                vec![
                    CalcField::choice("profile.family", "Profile family", PROFILE_FAMILIES, 0),
                    CalcField::text("profile.size", "Size", "40x40x3"),
                    CalcField::flag("keep", "Keep the old member visible", false),
                ],
            )
        }
        "design.frame_extend" => {
            picks.push(Pick { key: "member", label: "Member", accept: BODIES, multi: false, required: true, shape: PickShape::Body });
            (
                "design.frame_extend",
                "EXTEND MEMBER",
                "the member's end pushed out (or, with a negative amount, cut back) along its axis",
                vec![CalcField::number("amount_mm", "Amount (mm, negative trims)", "10"), CalcField::choice("end", "End", MEMBER_END, 0)],
            )
        }
        "design.groove_weld" => {
            picks.push(Pick { key: "face_a", label: "First face", accept: FACES, multi: false, required: true, shape: PickShape::Point });
            picks.push(Pick { key: "face_b", label: "Second face", accept: FACES, multi: false, required: true, shape: PickShape::Point });
            ("design.groove_weld", "GROOVE WELD", "the gap between two parallel faces filled as the weld's bead, a body of its own", vec![name()])
        }
        "canvas.remove_background" => {
            apply = false;
            button = "Apply";
            (
                "canvas.remove_background",
                "REMOVE BACKGROUND",
                "a wand: everything of the seed pixel's colour joined to it is painted out, so Trace Canvas sees the object alone",
                vec![
                    CalcField::text("canvas", "Canvas", ""),
                    CalcField::json("seed_px", "Seed pixel [x, y]", "[2, 2]"),
                    CalcField::number("tolerance", "Tolerance (0 to 1)", "0.15"),
                    CalcField::flag("contiguous", "Only what is joined to the seed", true),
                    CalcField::choice("fill", "The background becomes", FILLS, 0),
                ],
            )
        }
        "canvas.select_object" => {
            apply = false;
            button = "Apply";
            (
                "canvas.select_object",
                "SELECT OBJECT",
                "the background is judged from the picture's edges; what is left, in one piece with its holes filled, is the object",
                vec![
                    CalcField::text("canvas", "Canvas", ""),
                    CalcField::number("tolerance", "Tolerance (0 to 1)", "0.15"),
                    CalcField::choice("fill", "The background becomes", FILLS, 0),
                ],
            )
        }
        "canvas.rectify" => {
            apply = false;
            button = "Apply";
            (
                "canvas.rectify",
                "SHOOT IT SQUARE ON",
                "a photograph taken at an angle squared up: the four corners of a known rectangle, and its true size",
                vec![
                    CalcField::text("canvas", "Canvas", ""),
                    CalcField::json("corners", "Corners on the canvas plane [[x, y] × 4]", "[[0, 0], [100, 0], [100, 60], [0, 60]]"),
                    CalcField::number("width_mm", "True width (mm)", "100"),
                    CalcField::number("height_mm", "True height (mm)", "60"),
                    CalcField::number("margin", "Margin kept around it (fraction)", "0"),
                    CalcField::flag("pixel", "Corners are in pixels", false),
                ],
            )
        }
        "inspect.ray_trace" => (
            "inspect.ray_trace",
            "RAY TRACE",
            "a ray reflected off the bodies as first-surface mirrors, drawn as a 3D sketch",
            vec![
                CalcField::point("origin", "Origin: x, y, z", "-50, 0, 10"),
                CalcField::point("direction", "Direction: x, y, z", "1, 0, 0"),
                CalcField::number("bounces", "Bounces", "4"),
                CalcField::number("exit_mm", "Last leg (mm)", "200"),
                name(),
            ],
        ),
        "sketch.trace_canvas" => (
            "sketch.trace_canvas",
            "TRACE CANVAS",
            "the dark (or light) areas of an inserted picture as closed sketch profiles placed over it",
            vec![
                CalcField::text("canvas", "Canvas", ""),
                CalcField::number("threshold", "Threshold (0 to 255)", "128"),
                CalcField::flag("light", "Trace the light areas", false),
                CalcField::number("tolerance_px", "Tolerance (pixels)", "1"),
                CalcField::number("min_area_px", "Drop specks under (pixels²)", "4"),
                name(),
            ],
        ),
        _ => return None,
    };
    Some(CalcForm { command, title, about, fields, result: None, of: None, picks, apply, button })
}

#[cfg(test)]
#[path = "dialogs_tools_tests.rs"]
mod tests;
