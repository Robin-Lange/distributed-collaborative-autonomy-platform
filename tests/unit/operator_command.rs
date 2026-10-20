use super::*;

#[test]
fn navigation_command_preserves_canonical_xy_order() {
    let request =
        behavior_request_for_command(&OperatorCommand::GoTo(2.0, 1.0), "agent-01", None, 42)
            .expect("navigation builds a request");

    assert_eq!(
        request.behavior,
        BehaviorPrimitive::NavigateTo {
            target: PositionTarget::Planar([2.0, 1.0]),
            heading_yaw_rad: None,
        }
    );
}

#[test]
fn parser_preserves_canonical_xy_order() {
    let command = parse_line("g 2.0 1.0")
        .expect("navigation command parses")
        .expect("navigation command is not blank");
    let request = behavior_request_for_command(&command, "agent-01", None, 42)
        .expect("navigation builds a request");

    assert_eq!(
        request.behavior,
        BehaviorPrimitive::NavigateTo {
            target: PositionTarget::Planar([2.0, 1.0]),
            heading_yaw_rad: None,
        }
    );
}

#[test]
fn pickup_is_one_primitive_without_an_implicit_approach() {
    let request = behavior_request_for_command(
        &OperatorCommand::PickUp(Some("box".to_owned())),
        "agent-01",
        None,
        42,
    )
    .expect("pickup builds a request");

    assert_eq!(
        request.behavior,
        BehaviorPrimitive::Pick {
            object_id: "box".to_owned(),
        }
    );
}

#[test]
fn place_without_an_explicit_or_observed_object_is_not_sent() {
    assert!(
        behavior_request_for_command(&OperatorCommand::DropOff(None), "agent-01", None, 42,)
            .is_none()
    );
}

#[test]
fn resume_is_a_distinct_operator_action_not_a_behavior_primitive() {
    assert!(matches!(
        parse_line("r").expect("resume parses"),
        Some(OperatorCommand::ResumeAutonomy)
    ));
    assert!(
        behavior_request_for_command(&OperatorCommand::ResumeAutonomy, "agent-01", None, 42,)
            .is_none()
    );
}
