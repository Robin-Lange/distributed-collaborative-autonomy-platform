use super::*;
use core_msgs::{
    AgentStateMsg, ManipulationIntent, ObjectState, PoseXYZYaw, catalog::ObjectCatalog,
    command_source_human_operator, config::ObjectCatalogConfig, manipulation_verb_pick_up,
    object_lifecycle_free, veto_reason_unreachable,
};

fn pickup_intent() -> ManipulationIntent {
    ManipulationIntent {
        agent_id: "agent_01".to_owned(),
        object_id: "cylinder_pink_01".to_owned(),
        verb: manipulation_verb_pick_up(),
        source: command_source_human_operator(),
        stamp_ms: 100,
        frame_id: "map".to_owned(),
    }
}

#[test]
fn accepted_command_and_task_use_the_executor_request_id() {
    let catalog = ObjectCatalog::from_config(ObjectCatalogConfig::default());
    let state = AgentStateMsg {
        agent_id: "agent_01".to_owned(),
        position: [0.5, -1.0, 0.5],
        ..Default::default()
    };
    let object_states = BTreeMap::new();
    let intent = pickup_intent();

    let outcome = next_manipulation_decision(
        "agent_01",
        &catalog,
        200,
        ManipulationInputs {
            intent: Some(&intent),
            request_id: "behavior-request-7",
            agent_state: Some(&state),
            object_states: &object_states,
            trajectory_active: false,
            task_active: false,
        },
    )
    .unwrap();

    assert_eq!(outcome.command.unwrap().command_id, "behavior-request-7");
    assert_eq!(
        outcome.started_task.unwrap().command_id,
        "behavior-request-7"
    );
}

#[test]
fn rejection_uses_the_executor_request_id() {
    let catalog = ObjectCatalog::from_config(ObjectCatalogConfig::default());
    let object_states = BTreeMap::new();
    let intent = pickup_intent();

    let outcome = next_manipulation_decision(
        "agent_01",
        &catalog,
        200,
        ManipulationInputs {
            intent: Some(&intent),
            request_id: "behavior-request-8",
            agent_state: None,
            object_states: &object_states,
            trajectory_active: false,
            task_active: false,
        },
    )
    .unwrap();

    assert_eq!(outcome.veto.unwrap().command_id, "behavior-request-8");
}

#[test]
fn arm_sector_rejection_preserves_the_specific_operator_detail() {
    let catalog = ObjectCatalog::from_config(ObjectCatalogConfig::default());
    let state = AgentStateMsg {
        agent_id: "agent_01".to_owned(),
        position: [1.5, -1.0, 0.5],
        heading_yaw_rad: 0.0,
        ..Default::default()
    };
    let object_states = BTreeMap::new();
    let intent = pickup_intent();

    let outcome = next_manipulation_decision(
        "agent_01",
        &catalog,
        200,
        ManipulationInputs {
            intent: Some(&intent),
            request_id: "behind-platform",
            agent_state: Some(&state),
            object_states: &object_states,
            trajectory_active: false,
            task_active: false,
        },
    )
    .unwrap();
    let veto = outcome.veto.unwrap();

    assert_eq!(veto.command_id, "behind-platform");
    assert_eq!(veto.reason, veto_reason_unreachable());
    assert_eq!(
        veto.detail,
        "cylinder_pink_01 is outside the arm's forward sector: 180.0° from the \
         platform's heading; allowed offset is ±90.0°."
    );
}

#[test]
fn non_finite_object_pose_is_rejected_before_platform_dispatch() {
    let catalog = ObjectCatalog::from_config(ObjectCatalogConfig::default());
    let state = AgentStateMsg {
        agent_id: "agent_01".to_owned(),
        position: [0.0, 0.0, 0.5],
        ..Default::default()
    };
    let intent = ManipulationIntent {
        object_id: "block_mocap_01".to_owned(),
        ..pickup_intent()
    };
    let object_states = BTreeMap::from([(
        intent.object_id.clone(),
        ObjectState {
            object_id: intent.object_id.clone(),
            lifecycle: object_lifecycle_free(),
            pose: PoseXYZYaw {
                x: 0.5,
                y: 0.0,
                z: 0.1,
                yaw_rad: f32::NAN,
            },
            pose_valid: true,
            ..Default::default()
        },
    )]);

    let outcome = next_manipulation_decision(
        "agent_01",
        &catalog,
        200,
        ManipulationInputs {
            intent: Some(&intent),
            request_id: "non-finite-pose",
            agent_state: Some(&state),
            object_states: &object_states,
            trajectory_active: false,
            task_active: false,
        },
    )
    .unwrap();

    assert!(outcome.command.is_none());
    let veto = outcome.veto.unwrap();
    assert_eq!(veto.reason, veto_reason_unreachable());
    assert!(veto.detail.contains("is not finite"));
}

#[test]
fn tracked_pickup_rejects_stale_pose_and_snapshots_exact_valid_pose() {
    let catalog = ObjectCatalog::from_config(ObjectCatalogConfig::default());
    let state = AgentStateMsg {
        agent_id: "agent_01".to_owned(),
        position: [0.0, 0.0, 0.5],
        heading_yaw_rad: 0.0,
        ..Default::default()
    };
    let intent = ManipulationIntent {
        object_id: "block_mocap_01".to_owned(),
        ..pickup_intent()
    };
    let observed_pose = PoseXYZYaw {
        x: 0.5,
        y: 0.1,
        z: 0.12,
        yaw_rad: 0.7,
    };
    let mut object_states = BTreeMap::from([(
        intent.object_id.clone(),
        ObjectState {
            object_id: intent.object_id.clone(),
            lifecycle: object_lifecycle_free(),
            pose: observed_pose,
            pose_valid: false,
            ..Default::default()
        },
    )]);

    let stale_outcome = next_manipulation_decision(
        "agent_01",
        &catalog,
        200,
        ManipulationInputs {
            intent: Some(&intent),
            request_id: "tracked-pick-stale",
            agent_state: Some(&state),
            object_states: &object_states,
            trajectory_active: false,
            task_active: false,
        },
    )
    .unwrap();
    assert!(stale_outcome.command.is_none());
    assert!(
        stale_outcome
            .veto
            .unwrap()
            .detail
            .contains("has no known pose yet")
    );

    object_states.get_mut(&intent.object_id).unwrap().pose_valid = true;
    let accepted = next_manipulation_decision(
        "agent_01",
        &catalog,
        201,
        ManipulationInputs {
            intent: Some(&intent),
            request_id: "tracked-pick-valid",
            agent_state: Some(&state),
            object_states: &object_states,
            trajectory_active: false,
            task_active: false,
        },
    )
    .unwrap();
    let command = accepted.command.unwrap();

    object_states.get_mut(&intent.object_id).unwrap().pose.x = 9.0;
    assert!((command.target_pose.x - observed_pose.x).abs() < f32::EPSILON);
    assert!((command.target_pose.y - observed_pose.y).abs() < f32::EPSILON);
    assert!((command.target_pose.z - observed_pose.z).abs() < f32::EPSILON);
    assert!((command.target_pose.yaw_rad - observed_pose.yaw_rad).abs() < f32::EPSILON);
}
