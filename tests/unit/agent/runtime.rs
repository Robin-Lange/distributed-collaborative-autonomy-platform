use super::*;
use core_msgs::{
    BehaviorStatusReason, ConstraintEvent, constraint_type_robot_fault,
    constraint_type_virtual_wall, manipulation_failure_reason_no_object_detected,
    manipulation_failure_reason_safety_stop, manipulation_failure_reason_unsafe_grip_height,
    manipulation_failure_reason_unspecified, severity_hard_stop, severity_none,
    source_environment_runtime, source_spot_adapter,
};

fn constraint(source: u8, constraint_type: u8, severity: u8) -> ConstraintEvent {
    ConstraintEvent {
        agent_id: "agent_01".to_owned(),
        source,
        constraint_type,
        severity,
        object_id: "boundary:+x".to_owned(),
        stamp_ms: 10,
        ..Default::default()
    }
}

#[test]
fn first_recovery_request_waits_for_the_previous_wall_clear_without_being_consumed() {
    let mut constraints = ActiveConstraints::default();
    constraints.apply(
        constraint(
            source_environment_runtime(),
            constraint_type_virtual_wall(),
            severity_hard_stop(),
        ),
        10,
    );

    assert!(should_wait_for_environment_wall_clear(true, &constraints));

    constraints.apply(
        constraint(
            source_environment_runtime(),
            constraint_type_virtual_wall(),
            severity_none(),
        ),
        11,
    );

    assert!(!should_wait_for_environment_wall_clear(true, &constraints));
}

#[test]
fn platform_faults_do_not_use_the_environment_collision_wait() {
    let mut constraints = ActiveConstraints::default();
    constraints.apply(
        constraint(
            source_spot_adapter(),
            constraint_type_robot_fault(),
            severity_hard_stop(),
        ),
        10,
    );

    assert!(!should_wait_for_environment_wall_clear(true, &constraints));
}

#[test]
fn manipulation_requires_measured_base_motion_to_settle() {
    let mut state = AgentStateMsg {
        velocity: [0.06, 0.0, 0.0],
        ..Default::default()
    };
    assert!(platform_base_is_moving(Some(&state)));

    state.velocity = [0.0, 0.0, 0.0];
    state.yaw_rate_rad_s = 0.11;
    assert!(platform_base_is_moving(Some(&state)));

    state.yaw_rate_rad_s = 0.0;
    assert!(!platform_base_is_moving(Some(&state)));
}

#[test]
fn invalid_motion_feedback_fails_closed_for_manipulation() {
    let state = AgentStateMsg {
        velocity: [f32::NAN, 0.0, 0.0],
        ..Default::default()
    };

    assert!(platform_base_is_moving(Some(&state)));
}

#[test]
fn manipulation_execution_timeout_is_bounded_and_saturating() {
    let task = ActiveManipulationTask {
        command_id: "pick-timeout".to_owned(),
        object_id: "object-1".to_owned(),
        verb: manipulation_verb_pick_up(),
        started_ms: 1_000,
    };

    assert!(!manipulation_task_timed_out(&task, 999));
    assert!(!manipulation_task_timed_out(
        &task,
        1_000 + MANIPULATION_EXECUTION_TIMEOUT_MS - 1
    ));
    assert!(manipulation_task_timed_out(
        &task,
        1_000 + MANIPULATION_EXECUTION_TIMEOUT_MS
    ));
}

#[test]
fn manipulation_execution_failures_keep_their_operator_diagnosis() {
    assert_eq!(
        terminal_manipulation_failure(manipulation_failure_reason_safety_stop()),
        (
            BehaviorStatusReason::SafetyStop,
            "platform manipulation stopped for safety"
        )
    );
    assert_eq!(
        terminal_manipulation_failure(manipulation_failure_reason_no_object_detected()),
        (
            BehaviorStatusReason::ExecutionFailed,
            "gripper reached fully closed without detecting object contact"
        )
    );
    assert_eq!(
        terminal_manipulation_failure(manipulation_failure_reason_unsafe_grip_height()),
        (
            BehaviorStatusReason::ExecutionFailed,
            "requested grip point is below the platform's safe floor clearance"
        )
    );

    let fallback = (
        BehaviorStatusReason::ExecutionFailed,
        "platform manipulation execution failed; inspect the platform runtime log",
    );
    assert_eq!(
        terminal_manipulation_failure(manipulation_failure_reason_unspecified()),
        fallback
    );
    assert_eq!(terminal_manipulation_failure(u8::MAX), fallback);
}

#[test]
fn pickup_success_requires_held_state_for_the_same_agent() {
    let task = ActiveManipulationTask {
        command_id: "pick-1".to_owned(),
        object_id: "object-1".to_owned(),
        verb: manipulation_verb_pick_up(),
        started_ms: 10,
    };
    let mut states = BTreeMap::new();
    states.insert(
        "object-1".to_owned(),
        ObjectState {
            object_id: "object-1".to_owned(),
            lifecycle: object_lifecycle_free(),
            stamp_ms: 10,
            ..Default::default()
        },
    );

    assert_eq!(
        manipulation_confirmation_stamp(&task, &states, "agent_01"),
        None
    );

    {
        let Some(state) = states.get_mut("object-1") else {
            panic!("test object state must exist");
        };
        state.lifecycle = object_lifecycle_held();
        state.holder_agent_id = "agent_02".to_owned();
    }
    assert_eq!(
        manipulation_confirmation_stamp(&task, &states, "agent_01"),
        None
    );

    let Some(state) = states.get_mut("object-1") else {
        panic!("test object state must exist");
    };
    state.holder_agent_id = "agent_01".to_owned();
    state.stamp_ms = 12;
    assert_eq!(
        manipulation_confirmation_stamp(&task, &states, "agent_01"),
        Some(12)
    );
}

#[test]
fn place_success_requires_free_state_without_a_holder() {
    let task = ActiveManipulationTask {
        command_id: "place-1".to_owned(),
        object_id: "object-1".to_owned(),
        verb: manipulation_verb_drop_off(),
        started_ms: 20,
    };
    let mut states = BTreeMap::new();
    states.insert(
        "object-1".to_owned(),
        ObjectState {
            object_id: "object-1".to_owned(),
            lifecycle: object_lifecycle_held(),
            holder_agent_id: "agent_01".to_owned(),
            stamp_ms: 20,
            ..Default::default()
        },
    );

    assert_eq!(
        manipulation_confirmation_stamp(&task, &states, "agent_01"),
        None
    );

    let Some(state) = states.get_mut("object-1") else {
        panic!("test object state must exist");
    };
    state.lifecycle = object_lifecycle_free();
    state.holder_agent_id.clear();
    state.stamp_ms = 21;
    assert_eq!(
        manipulation_confirmation_stamp(&task, &states, "agent_01"),
        Some(21)
    );
}
