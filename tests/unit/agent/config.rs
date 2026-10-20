use core_msgs::{NavigationCapability, NavigationKind, SpeedLimits, SpeedPresets};

use super::*;

#[test]
fn capability_handoff_rejects_invalid_navigation_limits() {
    let capabilities = core_msgs::CapabilitySet {
        hold: true,
        navigation: vec![NavigationCapability {
            kind: NavigationKind::Planar,
            speed_presets: SpeedPresets {
                slow: SpeedLimits {
                    max_linear_speed_m_s: 0.0,
                    max_yaw_rate_rad_s: 1.0,
                },
                medium: SpeedLimits {
                    max_linear_speed_m_s: 1.0,
                    max_yaw_rate_rad_s: 1.0,
                },
                fast: SpeedLimits {
                    max_linear_speed_m_s: 2.0,
                    max_yaw_rate_rad_s: 2.0,
                },
            },
            position_tolerance_m: 0.1,
            supports_target_heading: false,
        }],
        manipulation: None,
    };
    let json = serde_json::to_string(&capabilities).expect("capabilities serialize");

    assert_eq!(
        decode_capabilities(json.as_str(), "test"),
        Err("platform profile 'test' has invalid navigation limits".to_owned())
    );
}
