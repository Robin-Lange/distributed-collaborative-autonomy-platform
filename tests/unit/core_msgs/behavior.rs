use super::*;
use crate::AgentScoped;

fn request(behavior: BehaviorPrimitive) -> BehaviorRequest {
    BehaviorRequest {
        request_id: "request-01".to_owned(),
        agent_id: "agent-01".to_owned(),
        source: BehaviorSource::Operator,
        behavior,
        frame_id: "map".to_owned(),
        stamp_ms: 42,
    }
}

#[test]
fn navigation_target_dimension_selects_the_required_capability() {
    let planar = BehaviorPrimitive::NavigateTo {
        target: PositionTarget::Planar([1.0, 2.0]),
        heading_yaw_rad: None,
    };
    let spatial = BehaviorPrimitive::NavigateTo {
        target: PositionTarget::Spatial([1.0, 2.0, 3.0]),
        heading_yaw_rad: None,
    };

    assert_eq!(
        planar.required_capability(),
        CapabilityRequirement::PlanarNavigation
    );
    assert_eq!(
        spatial.required_capability(),
        CapabilityRequirement::SpatialNavigation
    );
}

#[test]
fn invalid_targets_and_empty_object_ids_are_rejected() {
    assert_eq!(
        request(BehaviorPrimitive::NavigateTo {
            target: PositionTarget::Planar([f32::NAN, 0.0]),
            heading_yaw_rad: None,
        })
        .validate(),
        Err(BehaviorRequestError::NonFiniteTarget)
    );
    assert_eq!(
        request(BehaviorPrimitive::Pick {
            object_id: " ".to_owned(),
        })
        .validate(),
        Err(BehaviorRequestError::MissingObjectId)
    );
}

#[test]
fn move_away_preserves_the_current_point_based_contract() {
    let behavior = BehaviorPrimitive::MoveAway {
        from: PositionTarget::Planar([1.0, -2.0]),
    };

    assert_eq!(
        behavior.required_capability(),
        CapabilityRequirement::PlanarNavigation
    );
    assert!(request(behavior).validate().is_ok());
}

#[test]
fn resume_autonomy_is_agent_scoped_and_explicitly_correlated() {
    let resume = ResumeAutonomy {
        request_id: "resume-01".to_owned(),
        agent_id: "agent-01".to_owned(),
        stamp_ms: 42,
    };

    assert_eq!(resume.agent_id(), "agent-01");
    assert_eq!(resume.request_id, "resume-01");
}
