//! Whether a socketed bamboo joint holds.
//!
//! A bamboo frame is not limited by its tubes: a 30 mm culm carries more axial load than a rider
//! can put into it. What fails is the joint, in one of three ways:
//!
//! 1. The culm shears out a plug along its own fibres. Bamboo's interlaminar shear strength is
//!    about 5 MPa, a thousandth of its tensile strength along the grain, so unless the adhesive
//!    is worse than that the bamboo governs. This is the usual failure, and the surprising one.
//! 2. The adhesive lets go at the glue line: epoxy on prepared, dry bamboo runs 6 to 10 MPa in
//!    shear, far less on green or oily culm.
//! 3. The culm crushes. A sleeve gripping a hollow tube squeezes it oval, and bamboo takes only
//!    10 to 15 MPa across the fibre. A node inside the socket roughly doubles what the culm
//!    survives, because the diaphragm keeps the section round.
//!
//! So the check is: how much bond area is there, which of the two shear strengths is smaller,
//! and what the bending moment at the socket mouth does to the culm. Lengths are metres, forces
//! newtons, stresses pascals, moments newton-metres.

use std::f64::consts::PI;

/// A node inside the engaged length keeps the section round; doubling the crush strength is the
/// conservative end of what the literature reports for a noded against an internodal section.
pub const NODE_BONUS: f64 = 2.0;

/// One socketed bamboo joint, as built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BambooJoint {
    /// Outside diameter where the culm enters the socket (m).
    pub culm_diameter: f64,
    /// Wall thickness (m).
    pub wall_thickness: f64,
    /// How far the culm goes into the socket (m).
    pub engagement: f64,
    /// Radial glue gap per side (m).
    pub glue_gap: f64,
    /// Shear strength of the adhesive on prepared bamboo (Pa).
    pub adhesive_shear_strength: f64,
    /// Interlaminar shear strength of the bamboo itself (Pa): the one that usually decides.
    pub bamboo_shear_strength: f64,
    /// Compression strength across the fibre (Pa): what a sleeve crushing the culm runs into.
    pub bamboo_crush_strength: f64,
    /// A node lies inside the engaged length.
    pub node_inside_socket: bool,
    /// Axial force pulling the culm out (N).
    pub axial_force: f64,
    /// Bending moment at the socket mouth (N·m).
    pub bending_moment: f64,
}

impl Default for BambooJoint {
    fn default() -> Self {
        BambooJoint {
            culm_diameter: 0.032,
            wall_thickness: 0.008,
            engagement: 0.060,
            glue_gap: 0.0003,
            adhesive_shear_strength: 8e6,
            bamboo_shear_strength: 5e6,
            bamboo_crush_strength: 11e6,
            node_inside_socket: false,
            axial_force: 0.0,
            bending_moment: 0.0,
        }
    }
}

/// What the check found.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BambooJointResult {
    /// False for a joint that is not one; `advice` then says why.
    pub valid: bool,
    pub advice: String,
    /// The culm's own surface over the engagement (m²).
    pub bond_area: f64,
    /// Engagement over diameter.
    pub engagement_ratio: f64,
    /// The lower of the two shear strengths (Pa), and whether it was the bamboo's.
    pub governing_shear: f64,
    pub bamboo_governs: bool,
    /// N
    pub pull_out_capacity: f64,
    /// Pa, from the axial force.
    pub shear_stress: f64,
    /// Capacity over demand; 0 when nothing pulls.
    pub axial_safety: f64,
    /// Bearing stress from the moment (Pa): the moment is carried as a couple, the culm bearing
    /// on the socket near the mouth one way and near the bottom the other. With the two patches
    /// a third of the engagement each, a third of the way in from either end, the lever arm is
    /// 2L/3 and each patch carries 1.5 M / L over an area of D L / 3.
    pub bearing_stress: f64,
    /// Pa, after the node allowance.
    pub bearing_capacity: f64,
    pub bearing_safety: f64,
    /// The moment the joint takes before the culm starts to crush (N·m).
    pub moment_capacity: f64,
}

/// Checks the joint. `valid` is false, with the reason, for a joint that is not one: no
/// engagement, a wall thicker than the radius, a strength that is not positive.
pub fn check_bamboo_joint(joint: &BambooJoint) -> BambooJointResult {
    let mut r = BambooJointResult::default();
    let finite = [
        joint.culm_diameter,
        joint.wall_thickness,
        joint.engagement,
        joint.glue_gap,
        joint.adhesive_shear_strength,
        joint.bamboo_shear_strength,
        joint.bamboo_crush_strength,
        joint.axial_force,
        joint.bending_moment,
    ]
    .iter()
    .all(|x| x.is_finite());
    if !finite {
        r.advice = "every input must be a finite number".into();
        return r;
    }
    if !(joint.culm_diameter > 0.0) || !(joint.engagement > 0.0) {
        r.advice = "a joint needs a culm diameter and an engagement length".into();
        return r;
    }
    if !(joint.wall_thickness > 0.0) || joint.wall_thickness >= joint.culm_diameter / 2.0 {
        r.advice = "the wall must be thinner than the radius: a culm is a tube".into();
        return r;
    }
    if !(joint.adhesive_shear_strength > 0.0) || !(joint.bamboo_shear_strength > 0.0) || !(joint.bamboo_crush_strength > 0.0) {
        r.advice = "every strength must be a positive number".into();
        return r;
    }
    r.valid = true;

    // The glued cylinder is the culm's own outer surface, not the bore's: the bore is bigger by
    // the glue gap, and what shears is the bamboo.
    r.bond_area = PI * joint.culm_diameter * joint.engagement;
    r.engagement_ratio = joint.engagement / joint.culm_diameter;
    r.bamboo_governs = joint.bamboo_shear_strength <= joint.adhesive_shear_strength;
    r.governing_shear = joint.bamboo_shear_strength.min(joint.adhesive_shear_strength);
    r.pull_out_capacity = r.governing_shear * r.bond_area;
    r.shear_stress = joint.axial_force.abs() / r.bond_area;
    r.axial_safety = if r.shear_stress > 0.0 { r.governing_shear / r.shear_stress } else { 0.0 };

    // Bending as a couple across the engagement: the flagpole-in-a-socket estimate, first order.
    let patch_area = joint.culm_diameter * joint.engagement / 3.0;
    let couple = 1.5 * joint.bending_moment.abs() / joint.engagement;
    r.bearing_stress = couple / patch_area;
    r.bearing_capacity = joint.bamboo_crush_strength * if joint.node_inside_socket { NODE_BONUS } else { 1.0 };
    r.bearing_safety = if r.bearing_stress > 0.0 { r.bearing_capacity / r.bearing_stress } else { 0.0 };
    r.moment_capacity = r.bearing_capacity * patch_area * joint.engagement / 1.5;

    let mut advice: Vec<String> = Vec::new();
    if r.engagement_ratio < 1.5 {
        advice.push(format!(
            "The culm sits only {:.1} diameters deep. Below about 1.5 the bending is carried by crushing a short length of culm \
             rather than by the glue line and the joint turns brittle: make the socket longer.",
            r.engagement_ratio
        ));
    }
    if r.bamboo_governs {
        advice.push(format!(
            "The bamboo governs, not the glue: it shears out along its own fibres at {:.1} MPa, so a stronger adhesive changes nothing. More engagement does.",
            joint.bamboo_shear_strength / 1e6
        ));
    } else {
        advice.push(format!(
            "The adhesive governs here, at {:.1} MPa. That is unusual; check that the bamboo's shear strength came from a test and not a handbook.",
            joint.adhesive_shear_strength / 1e6
        ));
    }
    if !joint.node_inside_socket {
        advice.push(
            "No node inside the socket: without a diaphragm the culm goes oval and splits at about half the bearing stress. Cut the culm so a node lands in the engaged length."
                .into(),
        );
    }
    if joint.glue_gap <= 0.0 {
        advice.push(
            "No glue gap: a socket bored to the culm's exact size wipes the epoxy off on the way in and leaves a starved joint. Allow two or three tenths of a millimetre all round."
                .into(),
        );
    } else if joint.glue_gap > 0.001 {
        advice.push("A glue gap over a millimetre is a filler joint; epoxy in a thick layer is weaker and more brittle than in a thin one.".into());
    }
    r.advice = advice.join(" ");
    r
}

/// The engagement (m) a joint needs to carry `force` (N) in pull-out at `safety`: the number to
/// reach for when laying a connector out rather than checking one. 0 for inputs that make no
/// joint.
pub fn engagement_for(joint: &BambooJoint, force: f64, safety: f64) -> f64 {
    let shear = joint.bamboo_shear_strength.min(joint.adhesive_shear_strength);
    if !(joint.culm_diameter > 0.0) || !(shear > 0.0) || !(safety > 0.0) || !force.is_finite() {
        return 0.0;
    }
    force.abs() * safety / (PI * joint.culm_diameter * shear)
}

#[cfg(test)]
#[path = "bamboo_joint_tests.rs"]
mod tests;
