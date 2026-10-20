//! Velocity-level waypoint tracking controller, used by every navigation backend.
//!
//! Ported from the standalone reference crate
//! (`spot_velocity_tracking/spot_velocity_tracking/src/{controller,frames}.rs`)
//! so the onboard agent, not the executor, owns waypoint-to-velocity tracking.
//! On the real platform the sidecar turns each `COMMAND_TYPE_VELOCITY` message
//! into one short-lived `SE2VelocityCommand`; the simulator (`environment_sim`)
//! integrates the same setpoint in avian physics. Neither executor does
//! server-side goal tracking or goal-status feedback, so goal completion is
//! decided here, onboard, every state-driven decision
//! tick.
//!
//! The control law is a plain proportional pull (`K1 * pos_err`) on the
//! map -> `flat_body` tracking error — hardware tuning showed the derivative
//! term was unnecessary, so it was removed. Limits are explicit inputs
//! supplied from the active platform capability report, keeping this control
//! law independent of platform profiles and SDK constraints.

use std::f32::consts::PI;

use glam::Vec3;

use core_msgs::VelocitySetpoint;

/// First-order position gain; places one closed-loop pole at `-K1`.
const K1: f32 = 1.0;

/// Limits used for one planar guidance evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarGuidanceLimits {
    pub max_linear_speed_m_s: f32,
    pub max_yaw_rate_rad_s: f32,
    pub position_tolerance_m: f32,
}

impl PlanarGuidanceLimits {
    fn is_valid(self) -> bool {
        self.max_linear_speed_m_s.is_finite()
            && self.max_linear_speed_m_s > 0.0
            && self.max_yaw_rate_rad_s.is_finite()
            && self.max_yaw_rate_rad_s > 0.0
            && self.position_tolerance_m.is_finite()
            && self.position_tolerance_m >= 0.0
    }
}

/// Wraps an angle to `[-pi, pi)`.
fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

/// Rotates a map-frame vector into a gravity-aligned planar body frame
/// (`+x` forward, `+y` left); the `z` component passes through unchanged.
///
/// This is a live-heading rotation, re-evaluated every tick as the robot
/// turns — it cannot be baked into the static map<->odometry calibration,
/// because `SE2VelocityCommand` takes velocities in the robot's own body
/// frame, whose orientation in `map` changes continuously.
fn rotate_map_to_flat_body(v: Vec3, robot_yaw_map: f32) -> Vec3 {
    let (sin_yaw, cos_yaw) = robot_yaw_map.sin_cos();
    Vec3::new(
        cos_yaw.mul_add(v.x, sin_yaw * v.y),
        cos_yaw.mul_add(v.y, -sin_yaw * v.x),
        v.z,
    )
}

/// Proportional control law in the planar body frame, clamped to the supplied
/// platform limits.
fn vel_controller(pos_err: Vec3, limits: PlanarGuidanceLimits) -> Vec3 {
    let command = K1 * pos_err;
    Vec3::new(
        command
            .x
            .clamp(-limits.max_linear_speed_m_s, limits.max_linear_speed_m_s),
        command
            .y
            .clamp(-limits.max_linear_speed_m_s, limits.max_linear_speed_m_s),
        command
            .z
            .clamp(-limits.max_yaw_rate_rad_s, limits.max_yaw_rate_rad_s),
    )
}

/// Outcome of one velocity-tracking evaluation.
pub enum TrackOutcome {
    /// Within the configured position tolerance: tracking is complete.
    Reached,

    /// Velocity command to drive toward the target this tick.
    Command(VelocitySetpoint),
}

/// Evaluates one velocity-tracking tick toward a position-only target.
///
/// Goals are position-only (a `WaypointIntent` carries a point, no yaw), so
/// completion is planar-distance-only. While still approaching, the robot
/// faces the direction of travel (holonomic travel-facing — the same
/// behavior contract the old sidecar `_holonomic_heading` implemented)
/// rather than any fixed target heading.
///
/// `robot_velocity_map` is `(vx m/s, vy m/s, yaw_rate rad/s)` in the map
/// frame; `target_velocity_map` is the waypoint's desired velocity
/// (`WaypointIntent` is position-only today, so callers pass `Vec3::ZERO`).
///
/// Returns `None` when any input is non-finite; the caller must hold rather
/// than command in that case.
#[must_use]
pub fn track(
    robot_position_map: Vec3,
    robot_yaw_map: f32,
    robot_velocity_map: Vec3,
    target_position_map: Vec3,
    target_velocity_map: Vec3,
    limits: PlanarGuidanceLimits,
) -> Option<TrackOutcome> {
    if !robot_position_map.is_finite()
        || !robot_yaw_map.is_finite()
        || !robot_velocity_map.is_finite()
        || !target_position_map.is_finite()
        || !target_velocity_map.is_finite()
        || !limits.is_valid()
    {
        return None;
    }

    let error = target_position_map - robot_position_map;
    let distance = error.x.hypot(error.y);

    if distance <= limits.position_tolerance_m {
        return Some(TrackOutcome::Reached);
    }

    let desired_yaw = error.y.atan2(error.x);

    // Position error in flat_body: planar error rotated by R(-robot_yaw_map),
    // with the travel-facing yaw error carried in z.
    let pos_err_flat = rotate_map_to_flat_body(Vec3::new(error.x, error.y, 0.0), robot_yaw_map)
        + Vec3::new(0.0, 0.0, wrap_angle(desired_yaw - robot_yaw_map));

    let command = vel_controller(pos_err_flat, limits);

    Some(TrackOutcome::Command(VelocitySetpoint {
        forward_m_s: command.x,
        left_m_s: command.y,
        yaw_rate_rad_s: command.z,
    }))
}

#[cfg(test)]
#[path = "../../../tests/unit/autonomy/guidance.rs"]
mod tracking_tests;
