//! Static capabilities implemented by the Spot platform adapter.

mod runtime;

use core_msgs::{
    CapabilitySet, ManipulationCapability, NavigationCapability, NavigationKind, SpeedLimits,
    SpeedPresets,
};

/// Platform profile name used by agent deployment manifests.
pub const PROFILE: &str = "spot";

pub use runtime::{SpotRuntimeCommand, locate_runtime};

/// Returns the capabilities implemented by the Spot adapter.
#[must_use]
pub fn capabilities() -> CapabilitySet {
    CapabilitySet {
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
        manipulation: Some(ManipulationCapability {
            pick: true,
            place: true,
            min_reach_m: 0.30,
            max_reach_m: 0.80,
        }),
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/platform_spot.rs"]
mod tests;
