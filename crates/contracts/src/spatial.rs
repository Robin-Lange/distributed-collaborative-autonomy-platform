//! Planar projection shared across decision logic and tools.
//!
//! The autonomy stack treats waypoint behavior as planar motion. This module
//! makes that convention explicit by projecting vectors onto the XY plane.
//!
//! The shared bus frame is the canonical `map` frame: ENU, `+X` east, `+Y`
//! north, `+Z` up. Horizontal motion lives in the X/Y plane; Z carries height
//! and is preserved when constructing trajectory targets.

use glam::Vec3;

/// Projects a 3D vector onto the horizontal X/Y plane (`map` ENU frame).
///
/// Z is cleared because waypoint intent logic is horizontal; vertical height is
/// preserved separately when constructing trajectory commands.
#[must_use]
pub fn project_planar(vector: Vec3) -> Vec3 {
    vector.with_z(0.0)
}

/// Wraps an angle in radians to `(-pi, pi]`, so angular errors always take
/// the short way around.
#[must_use]
pub fn wrap_to_pi(angle: f32) -> f32 {
    use std::f32::consts::PI;
    (angle + PI).rem_euclid(2.0 * PI) - PI
}
