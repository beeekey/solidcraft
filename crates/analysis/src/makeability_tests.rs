//! The scan on shapes whose answers are obvious.

use solidcraft_geom::Vec3;

use crate::makeability::{MakeabilityRules, VoxelFrame, scan_makeability};

fn frame(nx: usize, ny: usize, nz: usize) -> VoxelFrame {
    VoxelFrame { nx, ny, nz, h: 0.005, origin: Vec3::ZERO }
}

#[test]
fn the_scan_knows_a_trapped_surface_from_an_open_one() {
    let (nx, ny, nz) = (20, 8, 10);
    let grid = frame(nx, ny, nz);
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    let rules = MakeabilityRules { mouldable: true, pull_axis: 2, machinable: true, tool_radius: grid.h, ..Default::default() };
    // A solid block: nothing is trapped.
    let mut block = vec![false; nx * ny * nz];
    for k in 0..nz / 2 {
        for j in 0..ny {
            for i in 0..nx / 2 {
                block[at(i, j, k)] = true;
            }
        }
    }
    let open = scan_makeability(&grid, &block, &rules);
    assert!(open.surface_voxels > 0.0);
    assert!(open.undercut_fraction < 0.01, "a plain block has nothing to trap: {}", open.undercut_fraction);
    // A bridge over a hollow: the inside is trapped between the halves and out of a tool's reach.
    let mut bridge = vec![false; nx * ny * nz];
    for j in 0..ny {
        for i in 0..nx {
            for k in 0..2 {
                bridge[at(i, j, k)] = true;
            }
            for k in nz - 2..nz {
                bridge[at(i, j, k)] = true;
            }
        }
        for k in 0..nz {
            bridge[at(0, j, k)] = true;
            bridge[at(nx - 1, j, k)] = true;
        }
    }
    let closed = scan_makeability(&grid, &bridge, &rules);
    assert!(closed.undercut_fraction > open.undercut_fraction, "the inside of a closed box cannot be released");
    assert!(closed.unreachable_fraction > 0.0, "nor reached by a tool coming straight in");
}

#[test]
fn a_block_has_no_overhang_and_a_shelf_does() {
    let (nx, ny, nz) = (10, 10, 10);
    let grid = frame(nx, ny, nz);
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    let rules = MakeabilityRules { printable_without_support: true, build_axis: 2, overhang_degrees: 45.0, locate: true, ..Default::default() };
    let mut block = vec![false; nx * ny * nz];
    for k in 0..5 {
        for j in 0..10 {
            for i in 0..10 {
                block[at(i, j, k)] = true;
            }
        }
    }
    let flat = scan_makeability(&grid, &block, &rules);
    assert_eq!(flat.overhang_fraction, 0.0);
    // A shelf hanging in the air over nothing.
    let mut shelf = block.clone();
    for j in 0..10 {
        for i in 0..10 {
            shelf[at(i, j, 8)] = true;
        }
    }
    let hanging = scan_makeability(&grid, &shelf, &rules);
    assert!(hanging.overhang_fraction > 0.0, "{}", hanging.overhang_fraction);
    assert!(!hanging.overhang_regions.is_empty());
    assert!(hanging.overhang_regions[0].area > 0.0);
}

#[test]
fn a_plain_block_needs_no_machining_and_a_pocket_needs_one_setup() {
    let (nx, ny, nz) = (12, 12, 8);
    let grid = frame(nx, ny, nz);
    let at = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    let rules = MakeabilityRules { machinable: true, tool_radius: grid.h, stock_is_bounding_box: true, flank_cutting: true, ..Default::default() };
    let block = vec![true; nx * ny * nz];
    let plain = scan_makeability(&grid, &block, &rules);
    assert_eq!(plain.machined_faces, 0, "a block's outside is the billet");
    assert_eq!(plain.setup_count, 0);
    assert_eq!(plain.trapped_volume, 0.0);
    let mut pocket = block.clone();
    for k in 4..nz {
        for j in 3..9 {
            for i in 3..9 {
                pocket[at(i, j, k)] = false;
            }
        }
    }
    let cut = scan_makeability(&grid, &pocket, &rules);
    assert!(cut.machined_faces > 0);
    assert_eq!(cut.setup_count, 1, "{:?}", cut.setup_axes);
    assert!(cut.setup_axes[4], "from +Z");
    assert_eq!(cut.unreachable_fraction, 0.0, "a 30 mm pocket is reachable by a 5 mm tool: {}", cut.unreachable_fraction);
    // A sealed void inside the stock is trapped.
    let mut hollow = block.clone();
    for k in 3..5 {
        for j in 4..8 {
            for i in 4..8 {
                hollow[at(i, j, k)] = false;
            }
        }
    }
    let sealed = scan_makeability(&grid, &hollow, &rules);
    assert!(sealed.trapped_volume > 0.0);
    assert!(sealed.trapped_sample.is_some());
}

#[test]
fn garbage_is_refused() {
    let grid = frame(3, 3, 3);
    assert_eq!(scan_makeability(&grid, &[true; 5], &MakeabilityRules::default()).surface_voxels, 0.0);
    let bad = VoxelFrame { nx: 0, ..grid };
    assert_eq!(scan_makeability(&bad, &[], &MakeabilityRules::default()).surface_voxels, 0.0);
    let huge_tool = MakeabilityRules { machinable: true, tool_radius: 1e9, ..Default::default() };
    let _ = scan_makeability(&grid, &[true; 27], &huge_tool);
}
