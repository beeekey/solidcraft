//! The bamboo joint check against hand arithmetic: the bond area is the culm's own surface, the
//! weaker shear strength governs, the moment is a couple over two patches of L/3.

use std::f64::consts::PI;

use super::{BambooJoint, NODE_BONUS, check_bamboo_joint, engagement_for};

fn near(got: f64, want: f64, rel: f64) -> bool {
    (got - want).abs() <= rel * want.abs().max(1e-12)
}

#[test]
fn a_32_mm_culm_60_mm_deep_shears_the_bamboo_first() {
    let joint = BambooJoint { axial_force: 4000.0, ..BambooJoint::default() };
    let r = check_bamboo_joint(&joint);
    assert!(r.valid, "{}", r.advice);
    let area = PI * 0.032 * 0.060;
    assert!(near(r.bond_area, area, 1e-12));
    assert!(near(r.engagement_ratio, 1.875, 1e-12));
    // 5 MPa bamboo against 8 MPa epoxy: the bamboo governs.
    assert!(r.bamboo_governs);
    assert!(near(r.governing_shear, 5e6, 1e-12));
    assert!(near(r.pull_out_capacity, 5e6 * area, 1e-12));
    assert!(near(r.shear_stress, 4000.0 / area, 1e-12));
    assert!(near(r.axial_safety, 5e6 * area / 4000.0, 1e-9));
    assert!(r.advice.contains("bamboo governs"));
    assert!(r.advice.contains("No node"));
}

#[test]
fn the_moment_is_carried_as_a_couple_over_two_patches() {
    let joint = BambooJoint { bending_moment: 100.0, node_inside_socket: true, ..BambooJoint::default() };
    let r = check_bamboo_joint(&joint);
    assert!(r.valid);
    let patch = 0.032 * 0.060 / 3.0;
    let couple = 1.5 * 100.0 / 0.060;
    assert!(near(r.bearing_stress, couple / patch, 1e-12));
    assert!(near(r.bearing_capacity, 11e6 * NODE_BONUS, 1e-12));
    assert!(near(r.bearing_safety, r.bearing_capacity / r.bearing_stress, 1e-12));
    // The moment capacity is the moment at which the bearing stress reaches the capacity.
    assert!(near(r.moment_capacity, r.bearing_capacity * patch * 0.060 / 1.5, 1e-12));
    assert!(near(r.moment_capacity / 100.0, r.bearing_safety, 1e-9));
    assert!(!r.advice.contains("No node"));
}

#[test]
fn a_better_adhesive_than_the_bamboo_is_called_out() {
    let joint = BambooJoint { adhesive_shear_strength: 4e6, ..BambooJoint::default() };
    let r = check_bamboo_joint(&joint);
    assert!(!r.bamboo_governs);
    assert!(near(r.governing_shear, 4e6, 1e-12));
    assert!(r.advice.contains("adhesive governs"));
}

#[test]
fn a_short_socket_and_a_bad_glue_gap_get_advice() {
    let short = check_bamboo_joint(&BambooJoint { engagement: 0.030, glue_gap: 0.0, ..BambooJoint::default() });
    assert!(short.valid);
    assert!(short.advice.contains("diameters deep"));
    assert!(short.advice.contains("No glue gap"));
    let thick = check_bamboo_joint(&BambooJoint { glue_gap: 0.002, ..BambooJoint::default() });
    assert!(thick.advice.contains("filler joint"));
}

#[test]
fn what_is_not_a_joint_is_refused_with_the_reason() {
    let r = check_bamboo_joint(&BambooJoint { engagement: 0.0, ..BambooJoint::default() });
    assert!(!r.valid && r.advice.contains("engagement"));
    let r = check_bamboo_joint(&BambooJoint { wall_thickness: 0.020, ..BambooJoint::default() });
    assert!(!r.valid && r.advice.contains("tube"));
    let r = check_bamboo_joint(&BambooJoint { bamboo_crush_strength: 0.0, ..BambooJoint::default() });
    assert!(!r.valid && r.advice.contains("strength"));
    let r = check_bamboo_joint(&BambooJoint { axial_force: f64::NAN, ..BambooJoint::default() });
    assert!(!r.valid && r.advice.contains("finite"));
}

#[test]
fn the_engagement_needed_inverts_the_pull_out_capacity() {
    let joint = BambooJoint::default();
    let needed = engagement_for(&joint, 3000.0, 3.0);
    let check = check_bamboo_joint(&BambooJoint { engagement: needed, axial_force: 3000.0, ..joint });
    assert!(near(check.axial_safety, 3.0, 1e-9), "{}", check.axial_safety);
    assert_eq!(engagement_for(&joint, 3000.0, 0.0), 0.0);
    assert_eq!(engagement_for(&BambooJoint { culm_diameter: 0.0, ..joint }, 3000.0, 2.0), 0.0);
}
