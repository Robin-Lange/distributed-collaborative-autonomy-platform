use super::*;
use core_msgs::{
    manipulation_phase_failed, manipulation_phase_in_progress, manipulation_phase_succeeded,
    manipulation_verb_drop_off, manipulation_verb_pick_up,
};

const OBJECT_ID: &str = "tracked-object";
const AGENT_ID: &str = "agent_01";
const COMMAND_ID: &str = "manipulation-1";

fn tracked_objects(lifecycle: u8, holder_agent_id: &str) -> HashMap<String, TrackedObject> {
    HashMap::from([(
        OBJECT_ID.to_owned(),
        TrackedObject {
            mocap_body_id: 2,
            lifecycle,
            holder_agent_id: holder_agent_id.to_owned(),
            pose: None,
            orientation_map: None,
            last_seen_ms: 0,
        },
    )])
}

fn command(verb: u8) -> ManipulationCommand {
    ManipulationCommand {
        agent_id: AGENT_ID.to_owned(),
        command_id: COMMAND_ID.to_owned(),
        object_id: OBJECT_ID.to_owned(),
        verb,
        approved_by: APPROVED_BY_ONBOARD_AGENT.to_owned(),
        ..Default::default()
    }
}

fn status(phase: u8) -> ManipulationStatus {
    ManipulationStatus {
        agent_id: AGENT_ID.to_owned(),
        command_id: COMMAND_ID.to_owned(),
        object_id: OBJECT_ID.to_owned(),
        phase,
        ..Default::default()
    }
}

fn mocap_frame(stamp_ms: u64, position: [f32; 3], orientation: [f32; 4]) -> MocapRigidBodyFrame {
    MocapRigidBodyFrame {
        bodies: vec![proto::RigidBodyPose {
            id: 2,
            position: Some(proto::Vec3 {
                x: position[0],
                y: position[1],
                z: position[2],
            }),
            orientation: Some(proto::Quat {
                x: orientation[0],
                y: orientation[1],
                z: orientation[2],
                w: orientation[3],
            }),
        }],
        stamp_ms,
        frame_id: core_msgs::frames::MOCAP_LAB_FRAME.to_owned(),
    }
}

#[test]
fn pickup_succeeds_only_after_the_matching_terminal_status() {
    let mut tracked = tracked_objects(object_lifecycle_free(), "");
    let mut pending = HashMap::new();
    latch_manipulation_command(
        &mut pending,
        &tracked,
        &command(manipulation_verb_pick_up()),
    );

    apply_manipulation_status(
        &mut pending,
        &mut tracked,
        &status(manipulation_phase_in_progress()),
    );
    assert_eq!(tracked[OBJECT_ID].lifecycle, object_lifecycle_free());
    assert!(pending.contains_key(OBJECT_ID));

    apply_manipulation_status(
        &mut pending,
        &mut tracked,
        &status(manipulation_phase_succeeded()),
    );
    assert_eq!(tracked[OBJECT_ID].lifecycle, object_lifecycle_held());
    assert_eq!(tracked[OBJECT_ID].holder_agent_id, AGENT_ID);
    assert!(!pending.contains_key(OBJECT_ID));
}

#[test]
fn failed_pickup_preserves_the_free_object_state() {
    let mut tracked = tracked_objects(object_lifecycle_free(), "");
    let mut pending = HashMap::new();
    latch_manipulation_command(
        &mut pending,
        &tracked,
        &command(manipulation_verb_pick_up()),
    );

    apply_manipulation_status(
        &mut pending,
        &mut tracked,
        &status(manipulation_phase_failed()),
    );

    assert_eq!(tracked[OBJECT_ID].lifecycle, object_lifecycle_free());
    assert!(tracked[OBJECT_ID].holder_agent_id.is_empty());
    assert!(!pending.contains_key(OBJECT_ID));
}

#[test]
fn successful_place_releases_the_object() {
    let mut tracked = tracked_objects(object_lifecycle_held(), AGENT_ID);
    let mut pending = HashMap::new();
    latch_manipulation_command(
        &mut pending,
        &tracked,
        &command(manipulation_verb_drop_off()),
    );

    apply_manipulation_status(
        &mut pending,
        &mut tracked,
        &status(manipulation_phase_succeeded()),
    );

    assert_eq!(tracked[OBJECT_ID].lifecycle, object_lifecycle_free());
    assert!(tracked[OBJECT_ID].holder_agent_id.is_empty());
    assert!(!pending.contains_key(OBJECT_ID));
}

#[test]
fn pose_validity_expires_after_the_stale_boundary_and_reacquires() {
    let mut tracked = tracked_objects(object_lifecycle_free(), "");
    let identity = RigidTransform::from_rotation_translation(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0; 3],
    );
    let first = mocap_frame(1_000, [0.5, -0.2, 0.1], [0.0, 0.0, 0.0, 1.0]);

    apply_mocap_frame(&mut tracked, &identity, DQuat::IDENTITY, &first);

    assert!(object_state_for(OBJECT_ID, &tracked[OBJECT_ID], 1_300).pose_valid);
    let stale = object_state_for(OBJECT_ID, &tracked[OBJECT_ID], 1_301);
    assert!(!stale.pose_valid);
    assert!(stale.orientation_map.is_none());

    let half_turn_sine = std::f32::consts::FRAC_1_SQRT_2;
    let reacquired = mocap_frame(
        2_000,
        [0.7, 0.3, 0.2],
        [0.0, 0.0, half_turn_sine, half_turn_sine],
    );
    apply_mocap_frame(&mut tracked, &identity, DQuat::IDENTITY, &reacquired);

    let current = object_state_for(OBJECT_ID, &tracked[OBJECT_ID], 2_000);
    assert!(current.pose_valid);
    assert!((current.pose.x - 0.7).abs() < f32::EPSILON);
    assert!((current.pose.y - 0.3).abs() < f32::EPSILON);
    assert!((current.pose.z - 0.2).abs() < f32::EPSILON);
    assert!((current.pose.yaw_rad - std::f32::consts::FRAC_PI_2).abs() < 1.0e-6);
    assert!(current.orientation_map.is_some());
}

#[test]
fn missing_and_invalid_bodies_do_not_refresh_pose_freshness() {
    let mut tracked = tracked_objects(object_lifecycle_free(), "");
    let identity = RigidTransform::from_rotation_translation(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0; 3],
    );
    apply_mocap_frame(
        &mut tracked,
        &identity,
        DQuat::IDENTITY,
        &mocap_frame(1_000, [0.5, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0]),
    );

    apply_mocap_frame(
        &mut tracked,
        &identity,
        DQuat::IDENTITY,
        &MocapRigidBodyFrame {
            stamp_ms: 2_000,
            ..Default::default()
        },
    );
    apply_mocap_frame(
        &mut tracked,
        &identity,
        DQuat::IDENTITY,
        &mocap_frame(2_100, [9.0; 3], [0.0; 4]),
    );

    assert_eq!(tracked[OBJECT_ID].last_seen_ms, 1_000);
    assert!(!object_state_for(OBJECT_ID, &tracked[OBJECT_ID], 2_100).pose_valid);
}
