use super::{messages, proto};
use crate::{
    manipulation_failure_reason_no_object_detected, manipulation_failure_reason_unsafe_grip_height,
    manipulation_phase_failed,
};
use serde::Serialize;

fn assert_f32_slice_near(actual: &[f32], expected: &[f32]) {
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.iter().zip(expected) {
        assert!((actual - expected).abs() < f32::EPSILON);
    }
}

fn assert_round_trip<T, P>(value: T)
where
    T: Clone + Serialize + From<P>,
    P: From<T>,
{
    let expected = serde_json::to_value(&value).unwrap();
    let wire = P::from(value);
    let actual = T::from(wire);
    assert_eq!(serde_json::to_value(actual).unwrap(), expected);
}

#[test]
fn vectors_and_planar_poses_round_trip_exactly() {
    let vector = [1.25, -2.5, 3.75];
    let wire: proto::Vec3 = vector.into();
    assert_f32_slice_near(&[wire.x, wire.y, wire.z], &vector);
    let restored: [f32; 3] = wire.into();
    assert_f32_slice_near(&restored, &vector);

    assert_round_trip::<messages::PoseXYZYaw, proto::PoseXyzYaw>(messages::PoseXYZYaw {
        x: 1.0,
        y: -2.0,
        z: 0.3,
        yaw_rad: 1.25,
    });
    assert_round_trip::<messages::Quat, proto::Quat>(messages::Quat {
        x: 0.1,
        y: -0.2,
        z: 0.3,
        w: 0.9,
    });
}

#[test]
fn authority_and_constraint_messages_preserve_every_field() {
    assert_round_trip::<messages::AuthorityState, proto::AuthorityState>(
        messages::AuthorityState {
            agent_id: "spot-7".to_owned(),
            command_authority: 1,
            state_authority: 2,
            physics_authority: 3,
            collision_authority: 4,
            mode: 5,
            reason: "hybrid handoff".to_owned(),
            stamp_ms: 123_456,
        },
    );

    assert_round_trip::<messages::ConstraintEvent, proto::ConstraintEvent>(
        messages::ConstraintEvent {
            agent_id: "spot-7".to_owned(),
            source: 1,
            collision_authority: 3,
            constraint_type: 2,
            severity: 1,
            object_id: "north-wall".to_owned(),
            stamp_ms: 200,
            episode_id: "episode-9".to_owned(),
            detail: "swept".to_owned(),
        },
    );
}

#[test]
fn navigation_messages_preserve_ids_frames_status_and_heading() {
    assert_round_trip::<messages::WaypointIntent, proto::WaypointIntent>(
        messages::WaypointIntent {
            agent_id: "spot-1".to_owned(),
            source: 1,
            behavior: 2,
            point: [3.0, -4.0, 5.0],
            stamp_ms: 10,
            frame_id: "map".to_owned(),
        },
    );

    assert_round_trip::<messages::PlatformCommand, proto::PlatformCommand>(
        messages::PlatformCommand {
            agent_id: "spot-1".to_owned(),
            command_id: "command-42".to_owned(),
            command_type: 2,
            source: 1,
            approved_by: "onboard_agent".to_owned(),
            target_position: [-1.0, 2.0, 0.5],
            stamp_ms: 11,
            frame_id: "map".to_owned(),
            episode_id: "episode-42".to_owned(),
            command_authority: 1,
            velocity: Some(messages::VelocitySetpoint {
                forward_m_s: 0.4,
                left_m_s: -0.2,
                yaw_rate_rad_s: 0.15,
            }),
        },
    );

    // `velocity` is only present on COMMAND_TYPE_VELOCITY commands; absent
    // otherwise.
    assert_round_trip::<messages::PlatformCommand, proto::PlatformCommand>(
        messages::PlatformCommand {
            command_id: "command-43".to_owned(),
            velocity: None,
            ..Default::default()
        },
    );

    assert_round_trip::<messages::CommandStatus, proto::CommandStatus>(messages::CommandStatus {
        agent_id: "spot-1".to_owned(),
        command_id: "command-42".to_owned(),
        status: 1,
        stamp_ms: 12,
    });

    assert_round_trip::<messages::AgentStateMsg, proto::AgentStateMsg>(messages::AgentStateMsg {
        agent_id: "spot-1".to_owned(),
        source: 0,
        state_authority: 2,
        position: [8.0, 9.0, 0.6],
        stamp_ms: 13,
        frame_id: "map".to_owned(),
        heading_yaw_rad: -1.75,
        velocity: [0.4, -0.3, 0.05],
        yaw_rate_rad_s: 0.22,
    });

    assert_round_trip::<messages::CommandVeto, proto::CommandVeto>(messages::CommandVeto {
        agent_id: "spot-1".to_owned(),
        command_id: "command-42".to_owned(),
        source: 1,
        reason: 12,
        detail: "outside reach envelope".to_owned(),
        stamp_ms: 14,
    });
}

#[test]
fn manipulation_messages_preserve_targets_profiles_and_failure_details() {
    assert_round_trip::<messages::ManipulationIntent, proto::ManipulationIntent>(
        messages::ManipulationIntent {
            agent_id: "spot-2".to_owned(),
            object_id: "block_mocap_01".to_owned(),
            verb: 1,
            source: 1,
            stamp_ms: 20,
            frame_id: "map".to_owned(),
        },
    );

    assert_round_trip::<messages::ManipulationCommand, proto::ManipulationCommand>(
        messages::ManipulationCommand {
            agent_id: "spot-2".to_owned(),
            command_id: "manip-20".to_owned(),
            object_id: "block_mocap_01".to_owned(),
            verb: 2,
            approved_by: "onboard_agent".to_owned(),
            target_pose: messages::PoseXYZYaw {
                x: 1.0,
                y: 2.0,
                z: 0.045,
                yaw_rad: -0.5,
            },
            stamp_ms: 21,
            frame_id: "map".to_owned(),
            grasp_profile: 1,
        },
    );

    assert_round_trip::<messages::ManipulationStatus, proto::ManipulationStatus>(
        messages::ManipulationStatus {
            agent_id: "spot-2".to_owned(),
            command_id: "manip-20".to_owned(),
            object_id: "block_mocap_01".to_owned(),
            phase: 4,
            failure_reason: 1,
            stamp_ms: 22,
        },
    );
    for failure_reason in [
        manipulation_failure_reason_no_object_detected(),
        manipulation_failure_reason_unsafe_grip_height(),
    ] {
        assert_round_trip::<messages::ManipulationStatus, proto::ManipulationStatus>(
            messages::ManipulationStatus {
                agent_id: "spot-2".to_owned(),
                command_id: "manip-failed".to_owned(),
                object_id: "block_mocap_01".to_owned(),
                phase: manipulation_phase_failed(),
                failure_reason,
                stamp_ms: 23,
            },
        );
    }
}

#[test]
fn object_state_preserves_pose_validity_and_optional_orientation() {
    let with_orientation = messages::ObjectState {
        object_id: "block_mocap_01".to_owned(),
        lifecycle: 2,
        holder_agent_id: "spot-2".to_owned(),
        pose: messages::PoseXYZYaw {
            x: -1.0,
            y: 2.5,
            z: 0.045,
            yaw_rad: 0.75,
        },
        pose_valid: true,
        stamp_ms: 30,
        orientation_map: Some(messages::Quat {
            x: 0.1,
            y: 0.2,
            z: 0.3,
            w: 0.9,
        }),
    };
    let wire: proto::ObjectState = with_orientation.clone().into();
    assert!(wire.pose.is_some());
    assert!(wire.orientation_map.is_some());
    let restored: messages::ObjectState = wire.into();
    assert_eq!(
        serde_json::to_value(restored).unwrap(),
        serde_json::to_value(with_orientation).unwrap()
    );

    assert_round_trip::<messages::ObjectState, proto::ObjectState>(messages::ObjectState {
        object_id: "cylinder_pink_01".to_owned(),
        pose_valid: false,
        orientation_map: None,
        ..Default::default()
    });
}

#[test]
fn missing_nested_wire_messages_use_zero_defaults_without_panicking() {
    let waypoint: messages::WaypointIntent = proto::WaypointIntent {
        point: None,
        ..Default::default()
    }
    .into();
    assert_f32_slice_near(&waypoint.point, &[0.0; 3]);

    let command: messages::PlatformCommand = proto::PlatformCommand {
        target_position: None,
        ..Default::default()
    }
    .into();
    assert_f32_slice_near(&command.target_position, &[0.0; 3]);

    let state: messages::AgentStateMsg = proto::AgentStateMsg {
        position: None,
        ..Default::default()
    }
    .into();
    assert_f32_slice_near(&state.position, &[0.0; 3]);

    let manipulation: messages::ManipulationCommand = proto::ManipulationCommand {
        target_pose: None,
        ..Default::default()
    }
    .into();
    let target = manipulation.target_pose;
    assert_f32_slice_near(&[target.x, target.y, target.z, target.yaw_rad], &[0.0; 4]);

    let object: messages::ObjectState = proto::ObjectState {
        pose: None,
        orientation_map: None,
        ..Default::default()
    }
    .into();
    assert_eq!(object.pose, messages::PoseXYZYaw::default());
    assert_eq!(object.orientation_map, None);
}
