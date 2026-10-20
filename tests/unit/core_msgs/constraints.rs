use super::*;
use crate::{
    ConstraintEvent, constraint_type_goal_blocked, constraint_type_robot_fault,
    constraint_type_virtual_wall, constraint_type_wall, severity_hard_stop, severity_none,
    source_environment_runtime, source_spot_adapter,
};

fn event(
    source: u8,
    constraint_type: u8,
    severity: u8,
    object_id: &str,
    stamp_ms: u64,
) -> ConstraintEvent {
    ConstraintEvent {
        agent_id: "spot-1".to_owned(),
        source,
        constraint_type,
        severity,
        object_id: object_id.to_owned(),
        stamp_ms,
        ..Default::default()
    }
}

#[test]
fn apply_activates_refreshes_and_only_clears_the_exact_key() {
    let mut active = ActiveConstraints::default();
    let wall = event(
        source_environment_runtime(),
        constraint_type_wall(),
        severity_hard_stop(),
        "west-wall",
        10,
    );

    assert_eq!(active.apply(wall.clone(), 100), ApplyOutcome::Activated);
    assert_eq!(active.apply(wall, 120), ApplyOutcome::Refreshed);
    assert_eq!(active.len(), 1);

    let wrong_object_clear = event(
        source_environment_runtime(),
        constraint_type_wall(),
        severity_none(),
        "east-wall",
        11,
    );
    assert_eq!(active.apply(wrong_object_clear, 130), ApplyOutcome::Ignored);
    assert_eq!(active.len(), 1);

    let matching_clear = event(
        source_environment_runtime(),
        constraint_type_wall(),
        severity_none(),
        "west-wall",
        12,
    );
    assert_eq!(active.apply(matching_clear, 140), ApplyOutcome::Cleared);
    assert_eq!(active.len(), 0);
}

#[test]
fn apply_ignores_non_hard_stop_severities() {
    let mut active = ActiveConstraints::default();
    let unsupported = event(
        source_environment_runtime(),
        constraint_type_wall(),
        99,
        "wall",
        1,
    );

    assert_eq!(active.apply(unsupported, 10), ApplyOutcome::Ignored);
    assert_eq!(active.len(), 0);
}

#[test]
fn environment_ttl_is_strict_and_refresh_uses_local_receipt_time() {
    let mut active = ActiveConstraints::default();
    let wall = event(
        source_environment_runtime(),
        constraint_type_wall(),
        severity_hard_stop(),
        "wall",
        1,
    );

    active.apply(wall.clone(), 1_000);
    assert!(active.expire(1_000 + ENV_CONSTRAINT_TTL_MS).is_empty());
    assert_eq!(active.len(), 1);

    active.apply(wall, 1_200);
    assert!(active.expire(1_200 + ENV_CONSTRAINT_TTL_MS).is_empty());
    assert_eq!(active.len(), 1);

    let expired = active.expire(1_200 + ENV_CONSTRAINT_TTL_MS + 1);
    assert_eq!(expired.len(), 1);
    assert_eq!(expired[0].object_id, "wall");
    assert_eq!(active.len(), 0);
}

#[test]
fn expiry_is_safe_for_clock_reversal_and_never_expires_spot_faults() {
    let mut active = ActiveConstraints::default();
    active.apply(
        event(
            source_environment_runtime(),
            constraint_type_wall(),
            severity_hard_stop(),
            "wall",
            1,
        ),
        500,
    );
    active.apply(
        event(
            source_spot_adapter(),
            constraint_type_robot_fault(),
            severity_hard_stop(),
            "motor",
            2,
        ),
        500,
    );

    assert!(active.expire(100).is_empty());
    assert_eq!(active.len(), 2);

    let expired = active.expire(10_000);
    assert_eq!(expired.len(), 1);
    assert_eq!(expired[0].object_id, "wall");
    assert_eq!(active.len(), 1);
}

#[test]
fn new_intent_drops_only_strictly_older_goal_blocked_entries() {
    let mut active = ActiveConstraints::default();
    for (constraint_type, object_id, stamp_ms) in [
        (constraint_type_goal_blocked(), "old", 99),
        (constraint_type_goal_blocked(), "equal", 100),
        (constraint_type_goal_blocked(), "new", 101),
        (constraint_type_wall(), "unrelated", 1),
    ] {
        active.apply(
            event(
                source_environment_runtime(),
                constraint_type,
                severity_hard_stop(),
                object_id,
                stamp_ms,
            ),
            10,
        );
    }

    active.drop_goal_blocked_older_than(100);

    assert_eq!(active.len(), 3);
    let summary = active.summary();
    assert!(!summary.contains("[old]"));
    assert!(summary.contains("[equal]"));
    assert!(summary.contains("[new]"));
    assert!(summary.contains("[unrelated]"));
}

#[test]
fn hard_stop_selection_respects_trust_fault_priority_and_recency() {
    let mut active = ActiveConstraints::default();
    active.apply(
        event(
            source_environment_runtime(),
            constraint_type_virtual_wall(),
            severity_hard_stop(),
            "new-wall",
            500,
        ),
        1,
    );
    active.apply(
        event(
            source_spot_adapter(),
            constraint_type_robot_fault(),
            severity_hard_stop(),
            "old-fault",
            100,
        ),
        1,
    );

    let selected = active.active_hard_stop(|_| true).unwrap();
    assert_eq!(selected.object_id, "old-fault");

    let selected_environment = active
        .active_hard_stop(|source| source == source_environment_runtime())
        .unwrap();
    assert_eq!(selected_environment.object_id, "new-wall");
    assert!(active.active_hard_stop(|_| false).is_none());

    active.apply(
        event(
            source_spot_adapter(),
            constraint_type_robot_fault(),
            severity_hard_stop(),
            "new-fault",
            600,
        ),
        2,
    );
    assert_eq!(
        active.active_hard_stop(|_| true).unwrap().object_id,
        "new-fault"
    );
}

#[test]
fn summary_is_stable_and_includes_empty_object_placeholder() {
    let mut active = ActiveConstraints::default();
    active.apply(
        event(
            source_spot_adapter(),
            constraint_type_robot_fault(),
            severity_hard_stop(),
            "motor",
            2,
        ),
        1,
    );
    active.apply(
        event(
            source_environment_runtime(),
            constraint_type_wall(),
            severity_hard_stop(),
            "",
            1,
        ),
        1,
    );

    assert_eq!(
        active.summary(),
        "ENVIRONMENT_RUNTIME/WALL[-], SPOT_ADAPTER/ROBOT_FAULT[motor]"
    );
}
