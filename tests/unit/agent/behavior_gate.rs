use core_msgs::{
    BehaviorPrimitive, BehaviorRequest, BehaviorSource, CapabilitySet, NavigationCapability,
    PositionTarget, SpeedLimits, SpeedPresets,
};

use super::*;

fn capabilities() -> CapabilitySet {
    CapabilitySet {
        hold: true,
        navigation: vec![NavigationCapability {
            kind: NavigationKind::Planar,
            speed_presets: SpeedPresets {
                slow: SpeedLimits {
                    max_linear_speed_m_s: 0.5,
                    max_yaw_rate_rad_s: 0.5,
                },
                medium: SpeedLimits {
                    max_linear_speed_m_s: 1.0,
                    max_yaw_rate_rad_s: 1.0,
                },
                fast: SpeedLimits {
                    max_linear_speed_m_s: 1.5,
                    max_yaw_rate_rad_s: 1.5,
                },
            },
            position_tolerance_m: 0.1,
            supports_target_heading: false,
        }],
        manipulation: None,
    }
}

fn request(source: BehaviorSource, behavior: BehaviorPrimitive) -> BehaviorRequest {
    BehaviorRequest {
        request_id: "request-01".to_owned(),
        agent_id: "agent-01".to_owned(),
        source,
        behavior,
        frame_id: MAP_FRAME.to_owned(),
        stamp_ms: 42,
    }
}

#[test]
fn operator_override_remains_latched_until_explicit_resume() {
    let mut gate = BehaviorGate::new(true);
    let operator = request(BehaviorSource::Operator, BehaviorPrimitive::Hold);
    assert_eq!(
        gate.admit(&operator, "agent-01", &capabilities(), Some(true)),
        Ok(())
    );

    let autonomy = request(BehaviorSource::Autonomy, BehaviorPrimitive::Hold);
    assert_eq!(
        gate.admit(&autonomy, "agent-01", &capabilities(), Some(true)),
        Err(Rejection {
            reason: BehaviorStatusReason::AutonomyPaused,
            detail: "operator override is active; explicit resume is required".to_owned(),
        })
    );

    gate.resume_autonomy();
    assert_eq!(
        gate.admit(&autonomy, "agent-01", &capabilities(), Some(true)),
        Ok(())
    );
}

#[test]
fn unsupported_and_disconnected_platforms_are_distinct() {
    let mut gate = BehaviorGate::new(false);
    let pick = request(
        BehaviorSource::Operator,
        BehaviorPrimitive::Pick {
            object_id: "box".to_owned(),
        },
    );
    assert_eq!(
        gate.admit(&pick, "agent-01", &capabilities(), Some(true))
            .expect_err("pick is unsupported")
            .reason,
        BehaviorStatusReason::UnsupportedCapability
    );

    let hold = request(BehaviorSource::Operator, BehaviorPrimitive::Hold);
    assert_eq!(
        gate.admit(&hold, "agent-01", &capabilities(), Some(false))
            .expect_err("adapter is offline")
            .reason,
        BehaviorStatusReason::PlatformUnavailable
    );
}

#[test]
fn target_heading_requires_an_advertised_navigation_feature() {
    let mut gate = BehaviorGate::new(false);
    let navigate = request(
        BehaviorSource::Operator,
        BehaviorPrimitive::NavigateTo {
            target: PositionTarget::Planar([1.0, 2.0]),
            heading_yaw_rad: Some(0.5),
        },
    );

    assert_eq!(
        gate.admit(&navigate, "agent-01", &capabilities(), Some(true))
            .expect_err("heading is unsupported")
            .reason,
        BehaviorStatusReason::UnsupportedCapability
    );
}
