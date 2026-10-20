use super::*;

#[test]
fn capability_sets_do_not_inherit_universal_primitives() {
    let capabilities = CapabilitySet::default();

    assert!(!capabilities.supports(CapabilityRequirement::Hold));
    assert!(!capabilities.supports(CapabilityRequirement::PlanarNavigation));
    assert!(!capabilities.supports(CapabilityRequirement::SpatialNavigation));
    assert!(!capabilities.supports(CapabilityRequirement::Pick));
    assert!(!capabilities.supports(CapabilityRequirement::Place));
}

#[test]
fn navigation_dimensions_are_checked_explicitly() {
    let capabilities = CapabilitySet {
        hold: true,
        navigation: vec![NavigationCapability {
            kind: NavigationKind::Planar,
            speed_presets: SpeedPresets {
                slow: SpeedLimits {
                    max_linear_speed_m_s: 0.4,
                    max_yaw_rate_rad_s: 0.5,
                },
                medium: SpeedLimits {
                    max_linear_speed_m_s: 0.9,
                    max_yaw_rate_rad_s: 1.0,
                },
                fast: SpeedLimits {
                    max_linear_speed_m_s: 1.5,
                    max_yaw_rate_rad_s: 1.5,
                },
            },
            position_tolerance_m: 0.08,
            supports_target_heading: false,
        }],
        manipulation: None,
    };

    assert!(capabilities.supports(CapabilityRequirement::PlanarNavigation));
    assert!(!capabilities.supports(CapabilityRequirement::SpatialNavigation));
}

#[test]
fn unavailable_platform_is_distinct_from_unsupported_capability() {
    let report = CapabilityReport {
        agent_id: "spot-01".to_owned(),
        platform_profile: "spot".to_owned(),
        connected: false,
        unavailable_reason: Some("adapter offline".to_owned()),
        capabilities: CapabilitySet {
            hold: true,
            ..CapabilitySet::default()
        },
        stamp_ms: 42,
    };

    assert_eq!(
        report.require(CapabilityRequirement::Hold),
        Err(CapabilityRejection::PlatformUnavailable {
            reason: "adapter offline".to_owned(),
        })
    );
}
