use super::*;
use std::f32::consts::{FRAC_PI_2, PI};

/// Asserts two floats match within a fixed epsilon (no `approx` crate).
fn close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 1.0e-6,
        "expected {expected}, got {actual}"
    );
}

const fn limits() -> PlanarGuidanceLimits {
    PlanarGuidanceLimits {
        max_linear_speed_m_s: 1.5,
        max_yaw_rate_rad_s: 1.5,
        position_tolerance_m: 0.08,
    }
}

fn planar_velocity(setpoint: VelocitySetpoint) -> (f32, f32, f32) {
    let VelocitySetpoint {
        forward_m_s,
        left_m_s,
        yaw_rate_rad_s,
    } = setpoint;

    (forward_m_s, left_m_s, yaw_rate_rad_s)
}

#[test]
fn gain_is_the_single_closed_loop_pole() {
    close(K1, 1.0);
}

#[test]
fn vel_controller_is_k1_times_position_error() {
    let pos_err = Vec3::new(0.01, -0.02, 0.03);
    let command = vel_controller(pos_err, limits());

    close(command.x, K1 * pos_err.x);
    close(command.y, K1 * pos_err.y);
    close(command.z, K1 * pos_err.z);
}

#[test]
fn vel_controller_clamps_linear_and_yaw_axes_to_their_limits() {
    let pos_err = Vec3::new(10.0, -10.0, PI);
    let command = vel_controller(pos_err, limits());

    close(command.x, limits().max_linear_speed_m_s);
    close(command.y, -limits().max_linear_speed_m_s);
    close(command.z, limits().max_yaw_rate_rad_s);
}

#[test]
fn rotate_map_to_flat_body_rotates_map_axes_into_body() {
    // Robot faces north (yaw = +pi/2): map +X (east) rotates to flat_body -y
    // (the robot's right), and the z component passes through unchanged.
    let v = rotate_map_to_flat_body(Vec3::new(1.0, 0.0, 0.7), FRAC_PI_2);

    close(v.x, 0.0);
    close(v.y, -1.0);
    close(v.z, 0.7);
}

#[test]
fn yaw_error_uses_shortest_wrapped_difference() {
    close(wrap_angle(3.0 * PI), -PI);
}

#[test]
fn track_reports_reached_within_tolerance() {
    let position = Vec3::new(1.0, 1.0, 0.0);
    let target = position + Vec3::new(limits().position_tolerance_m * 0.5, 0.0, 0.0);

    let outcome = track(position, 0.0, Vec3::ZERO, target, Vec3::ZERO, limits())
        .expect("finite inputs must produce Some");

    assert!(matches!(outcome, TrackOutcome::Reached));
}

#[test]
fn track_faces_direction_of_travel_when_far_from_target() {
    // Robot faces east (heading 0) at the origin; target is west and
    // slightly north of the robot, so reaching it requires turning left
    // (CCW, positive yaw rate) away from due-east.
    let position = Vec3::new(0.0, 0.0, 0.0);
    let target = Vec3::new(-1.0, 0.5, 0.0);

    let outcome = track(position, 0.0, Vec3::ZERO, target, Vec3::ZERO, limits())
        .expect("finite inputs must produce Some");

    let TrackOutcome::Command(action) = outcome else {
        panic!("expected a velocity command while far from the target");
    };
    let (_, _, yaw_rate_rad_s) = planar_velocity(action);

    assert!(
        yaw_rate_rad_s > 0.0,
        "expected a positive (CCW) yaw rate turning toward the target, got {yaw_rate_rad_s}"
    );
}

#[test]
fn track_is_proportional_only_measured_velocity_does_not_change_command() {
    // Hardware-tuned choice: the controller is P-only, so an approach already
    // closing at 1 m/s commands exactly the same K1 * pos_err forward as a
    // standing start — measured velocity is finite-guarded but not fed back.
    let target = Vec3::new(1.0, 0.0, 0.0);
    let closing = track(
        Vec3::ZERO,
        0.0,
        Vec3::new(1.0, 0.0, 0.0),
        target,
        Vec3::ZERO,
        limits(),
    )
    .expect("finite inputs must produce Some");
    let standing = track(Vec3::ZERO, 0.0, Vec3::ZERO, target, Vec3::ZERO, limits())
        .expect("finite inputs must produce Some");

    let (TrackOutcome::Command(closing), TrackOutcome::Command(standing)) = (closing, standing)
    else {
        panic!("expected velocity commands while far from the target");
    };

    let (closing_forward, _, _) = planar_velocity(closing);
    let (standing_forward, _, _) = planar_velocity(standing);
    close(closing_forward, K1 * 1.0);
    close(closing_forward, standing_forward);
}

#[test]
fn track_returns_none_for_non_finite_input() {
    let position = Vec3::new(0.0, 0.0, 0.0);
    let target = Vec3::new(1.0, 1.0, 0.0);

    assert!(
        track(
            Vec3::new(f32::NAN, 0.0, 0.0),
            0.0,
            Vec3::ZERO,
            target,
            Vec3::ZERO,
            limits()
        )
        .is_none()
    );
    assert!(track(position, f32::NAN, Vec3::ZERO, target, Vec3::ZERO, limits()).is_none());
    assert!(
        track(
            position,
            0.0,
            Vec3::new(f32::NAN, 0.0, 0.0),
            target,
            Vec3::ZERO,
            limits()
        )
        .is_none()
    );
    assert!(
        track(
            position,
            0.0,
            Vec3::ZERO,
            Vec3::new(f32::INFINITY, 0.0, 0.0),
            Vec3::ZERO,
            limits()
        )
        .is_none()
    );
}
