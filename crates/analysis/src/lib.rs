//! Native finite-element analysis on a voxel grid.
//!
//! A body is voxelised on a regular grid; every solid voxel becomes a trilinear hexahedral element
//! (full 2×2×2 Gauss integration, linear elasticity, isotropic or orthotropic); held voxels are
//! clamped, each load is spread over the nodes of the voxels it acts on, and the system is solved
//! by Jacobi-preconditioned conjugate gradients. Displacements and stresses are physical: metres
//! and pascals.
//!
//! The solver answers on the grid ([`VoxelFemProblem`] → [`VoxelFemResult`]) so that a search
//! which invents thousands of candidate shapes can reuse it, and on a mesh ([`solve_mesh`]) for a
//! part somebody drew. Both are the same solve, and the tests hold them to closed-form elasticity:
//! Hooke's law on a rod, Euler–Bernoulli with Timoshenko's shear term on a cantilever.
//!
//! This crate depends on `solidcraft-geom` only: no kernel, no UI, and it builds for wasm32.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

pub mod bamboo_joint;
pub mod bearing;
pub mod bolt;
pub mod buckling;
pub mod construction;
pub mod damping;
pub mod drives;
mod element;
pub mod fibre_failure;
pub mod fits;
pub mod fluid;
pub mod general_tolerances;
pub mod generative;
pub mod handbook;
pub mod hole_stack;
pub mod implicit;
pub mod infill_section;
pub mod large_displacement;
pub mod lattice;
pub mod machining;
pub mod makeability;
mod material;
mod mesh_solve;
pub mod print_analysis;
pub mod printed_stress;
pub mod stack_up;
pub mod thermal;
pub mod thread_engagement;
pub mod trace;
mod voxel_fem;
mod voxelise;

#[cfg(test)]
mod buckling_tests;
#[cfg(test)]
mod fluid_tests;
#[cfg(test)]
mod implicit_tests;
#[cfg(test)]
mod lattice_tests;
#[cfg(test)]
mod printed_stress_tests;
#[cfg(test)]
mod thermal_tests;
#[cfg(test)]
mod voxel_fem_tests;

pub use buckling::{BucklingResult, buckling, buckling_rayleigh};
pub use element::Element;
pub use fibre_failure::{BAMBOO, BambooSpecies, FibreFailureReport, FibreState, FibreStrength, bamboo_strength, check_fibre_failure};
pub use fluid::{
    FLUIDS, FlowBody, FlowRegion, FluidError, FluidProblem, FluidProperties, FluidResult, MAX_FLOW_CELLS, VoxelBody, fluid_named, fluid_solve,
    voxel_body,
};
pub use large_displacement::{LargeDisplacementOptions, LargeDisplacementResult, Step, large_displacement, limit_load};
pub use lattice::{
    BeamCell, Family as LatticeFamily, Grading as LatticeGrading, LatticeOptions, LatticeResult, TpmsType, marching_tetrahedra, tpms_value,
};
pub use material::{MaterialError, OrthotropicMaterial};
pub use mesh_solve::{
    Assembly, ContactOptions, MAX_CELLS, MAX_RESOLUTION, MIN_RESOLUTION, MeshFemError, MeshFemOptions, MeshFemResult, solve_assembly, solve_mesh,
};
pub use printed_stress::{
    Governs, InfillComparison, LayerStress, PrintedStressReport, compare_infill, layer_stress, printed_material, printed_stress,
};
pub use thermal::{
    AmbientAir, HeatInput, HeldTemperature, MAX_THERMAL_CELLS, Placement, ThermalProblem, ThermalResult, channel_convection_film,
    free_convection_film, radiative_film, thermal_solve,
};
pub use voxel_fem::{ContactReport, LoadCase, NodeMap, VoxelFemCase, VoxelFemProblem, VoxelFemResult, node_map, solve_voxels};
pub use voxelise::{MM, Region, VoxelGrid, point_triangle_distance, voxelise, voxelise_many};
