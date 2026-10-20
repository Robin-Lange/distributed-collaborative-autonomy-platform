use core_msgs::{
    APPROVED_BY_ONBOARD_AGENT, ControlAction, ControlRequest, ControlSource, PoseXYZYaw,
    PositionTarget, frames::MAP_FRAME,
};

use super::*;

fn request(action: ControlAction) -> ControlRequest {
    ControlRequest {
        request_id: "request-01".to_owned(),
        agent_id: "agent-01".to_owned(),
        source: ControlSource::Safety,
        approved_by: APPROVED_BY_ONBOARD_AGENT.to_owned(),
        action,
        safety_episode_id: Some("episode-01".to_owned()),
        stamp_ms: 42,
    }
}

#[test]
fn stop_preserves_spatial_target_frame_and_safety_episode() {
    let request = request(ControlAction::Stop {
        target: PositionTarget::Spatial([1.0, 2.0, 3.0]),
        frame_id: MAP_FRAME.to_owned(),
    });
    let routed = route_control_request(&request);
    let SimControl::Drive(command) = routed else {
        panic!("stop must route to drive");
    };

    assert_eq!(command.command_type, command_type_stop());
    assert_vec3_near(command.target_position, [1.0, 2.0, 3.0]);
    assert_eq!(command.frame_id, MAP_FRAME);
    assert_eq!(command.episode_id, "episode-01");
    assert_eq!(command.source, command_source_none());
}

#[test]
fn spatial_velocity_is_explicitly_unsupported_by_the_planar_simulator() {
    let request = request(ControlAction::SpatialVelocity {
        velocity_m_s: [1.0, 2.0, 3.0],
        yaw_rate_rad_s: 0.5,
    });
    let routed = route_control_request(&request);

    assert!(matches!(routed, SimControl::Unsupported(_)));
}

#[test]
fn planar_velocity_preserves_waypoint_and_frame_for_hybrid_collision_gating() {
    let request = request(ControlAction::PlanarVelocity {
        target: PositionTarget::Planar([4.0, 5.0]),
        forward_m_s: 0.6,
        left_m_s: 0.2,
        yaw_rate_rad_s: 0.1,
        frame_id: MAP_FRAME.to_owned(),
    });
    let routed = route_control_request(&request);
    let SimControl::Drive(command) = routed else {
        panic!("planar velocity must route to drive");
    };

    assert_eq!(command.command_type, command_type_velocity());
    assert_vec3_near(command.target_position, [4.0, 5.0, 0.0]);
    assert_eq!(command.frame_id, MAP_FRAME);
    assert_eq!(
        command.velocity,
        Some(VelocitySetpoint {
            forward_m_s: 0.6,
            left_m_s: 0.2,
            yaw_rate_rad_s: 0.1,
        })
    );
}

#[test]
fn pick_preserves_the_executor_request_id_for_terminal_status_matching() {
    let request = request(ControlAction::Pick {
        object_id: "cylinder_pink_01".to_owned(),
        target_pose: PoseXYZYaw::default(),
        frame_id: MAP_FRAME.to_owned(),
    });
    let routed = route_control_request(&request);
    let SimControl::Manipulate(command) = routed else {
        panic!("pick must route to manipulation");
    };

    assert_eq!(command.command_id, request.request_id);
}

fn assert_vec3_near(actual: [f32; 3], expected: [f32; 3]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert!((actual - expected).abs() < f32::EPSILON);
    }
}
