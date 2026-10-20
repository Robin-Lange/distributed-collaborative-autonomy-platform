use super::*;
use core_msgs::{
    Behavior, behavior_code, command_type_hold, command_type_velocity, frames::RigidTransform,
};

fn identity_frames() -> FrameConversion {
    FrameConversion::new(&RigidTransform::from_rotation_translation(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 0.0],
    ))
}

fn assert_vec3_close(actual: Vec3, expected: Vec3) {
    assert!(
        actual.abs_diff_eq(expected, 1.0e-5),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn map_headings_follow_cardinal_directions() {
    let frames = identity_frames();

    assert_vec3_close(heading_to_bevy_forward(&frames, 0.0), Vec3::X);
    assert_vec3_close(
        heading_to_bevy_forward(&frames, std::f32::consts::FRAC_PI_2),
        Vec3::Y,
    );
    assert_vec3_close(
        heading_to_bevy_forward(&frames, std::f32::consts::PI),
        Vec3::NEG_X,
    );
}

#[test]
fn cylinder_rotation_aligns_its_symmetry_axis_with_heading() {
    let frames = identity_frames();

    for yaw in [0.0, std::f32::consts::FRAC_PI_2, -1.2] {
        let rotated_axis = object_rotation(&frames, yaw) * Vec3::Y;
        assert_vec3_close(rotated_axis, heading_to_bevy_forward(&frames, yaw));
    }
}

#[test]
fn forward_facing_rotation_aligns_mesh_forward_with_heading() {
    let frames = identity_frames();

    for yaw in [0.0, std::f32::consts::FRAC_PI_2, -1.2] {
        let rotated_forward = forward_facing_rotation(&frames, yaw) * Vec3::NEG_Z;
        assert_vec3_close(rotated_forward, heading_to_bevy_forward(&frames, yaw));
    }
}

#[test]
fn hud_is_idle_without_an_intent() {
    assert_eq!(hud_text(None, None), IDLE_HUD_TEXT);
}

#[test]
fn hud_reports_behavior_target_and_latest_command() {
    let intent = WaypointIntent {
        behavior: behavior_code(Behavior::GoTo),
        point: [1.25, -2.5, 0.75],
        ..Default::default()
    };
    let velocity = PlatformCommand {
        command_type: command_type_velocity(),
        ..Default::default()
    };
    let hold = PlatformCommand {
        command_type: command_type_hold(),
        ..Default::default()
    };

    assert_eq!(
        hud_text(Some(&intent), Some(&velocity)),
        "GoTo [1.25, -2.50, 0.75] | cmd: VELOCITY"
    );
    assert_eq!(
        hud_text(Some(&intent), Some(&hold)),
        "GoTo [1.25, -2.50, 0.75] | cmd: HOLD"
    );
}

#[test]
fn hud_surfaces_unknown_wire_codes_without_panicking() {
    let intent = WaypointIntent {
        behavior: u8::MAX,
        point: [0.0; 3],
        ..Default::default()
    };
    let command = PlatformCommand {
        command_type: u8::MAX,
        ..Default::default()
    };

    assert_eq!(
        hud_text(Some(&intent), Some(&command)),
        "UNKNOWN [0.00, 0.00, 0.00] | cmd: UNKNOWN"
    );
}

#[test]
fn manipulation_lifecycle_changes_interpolate_instead_of_teleporting() {
    let mut transform = Transform::default();
    let mut motion = ObjectMotion::new(&transform);

    observe_object_pose(
        &mut transform,
        &mut motion,
        object_lifecycle_free(),
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    observe_object_pose(
        &mut transform,
        &mut motion,
        object_lifecycle_held(),
        Vec3::X,
        Quat::from_rotation_y(1.0),
    );

    assert_eq!(transform.translation, Vec3::ZERO);
    assert!(motion.active);

    advance_object_motion(
        &mut transform,
        &mut motion,
        MANIPULATION_TRANSITION_DURATION_S * 0.5,
    );
    assert_vec3_close(transform.translation, Vec3::X * 0.5);

    advance_object_motion(
        &mut transform,
        &mut motion,
        MANIPULATION_TRANSITION_DURATION_S * 0.5,
    );
    assert_vec3_close(transform.translation, Vec3::X);
    assert!(!motion.active);
}

#[test]
fn held_object_transition_tracks_a_moving_carry_target() {
    let mut transform = Transform::default();
    let mut motion = ObjectMotion::new(&transform);
    motion.lifecycle = Some(object_lifecycle_free());

    observe_object_pose(
        &mut transform,
        &mut motion,
        object_lifecycle_held(),
        Vec3::X,
        Quat::IDENTITY,
    );
    advance_object_motion(
        &mut transform,
        &mut motion,
        MANIPULATION_TRANSITION_DURATION_S * 0.5,
    );
    observe_object_pose(
        &mut transform,
        &mut motion,
        object_lifecycle_held(),
        Vec3::X * 2.0,
        Quat::IDENTITY,
    );
    advance_object_motion(
        &mut transform,
        &mut motion,
        MANIPULATION_TRANSITION_DURATION_S * 0.5,
    );

    assert_vec3_close(transform.translation, Vec3::X * 2.0);
}
