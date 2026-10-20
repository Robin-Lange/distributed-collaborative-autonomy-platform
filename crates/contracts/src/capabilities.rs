//! Typed platform capabilities and live availability.

use serde::{Deserialize, Serialize};

/// Process environment variable carrying the selected platform's static
/// [`CapabilitySet`] from the local launcher to an agent child.
pub const AGENT_CAPABILITIES_ENV: &str = "DCA_AGENT_CAPABILITIES_JSON";

/// Capability required to execute one behavior primitive.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CapabilityRequirement {
    Hold,
    PlanarNavigation,
    SpatialNavigation,
    Pick,
    Place,
}

/// Dimensionality of a platform's navigation controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavigationKind {
    /// Position is expressed as `[x, y]`.
    Planar,
    /// Position is expressed as `[x, y, z]`.
    Spatial,
}

/// Operator-selectable maximum-speed profile.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpeedPreset {
    Slow,
    #[default]
    Medium,
    Fast,
}

impl SpeedPreset {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Slow => "slow",
            Self::Medium => "medium",
            Self::Fast => "fast",
        }
    }
}

/// Maximum navigation rates for one speed profile.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpeedLimits {
    pub max_linear_speed_m_s: f32,
    pub max_yaw_rate_rad_s: f32,
}

/// The three speed profiles every navigation-capable platform defines.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpeedPresets {
    pub slow: SpeedLimits,
    pub medium: SpeedLimits,
    pub fast: SpeedLimits,
}

impl SpeedPresets {
    #[must_use]
    pub const fn selected(self, preset: SpeedPreset) -> SpeedLimits {
        match preset {
            SpeedPreset::Slow => self.slow,
            SpeedPreset::Medium => self.medium,
            SpeedPreset::Fast => self.fast,
        }
    }
}

/// Limits for one supported navigation kind.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NavigationCapability {
    pub kind: NavigationKind,
    pub speed_presets: SpeedPresets,
    pub position_tolerance_m: f32,
    pub supports_target_heading: bool,
}

/// Manipulation operations and reach advertised by a platform.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ManipulationCapability {
    pub pick: bool,
    pub place: bool,
    pub min_reach_m: f32,
    pub max_reach_m: f32,
}

/// Static capabilities implemented by one platform profile.
///
/// An empty set is valid: platforms do not inherit a universal primitive set.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CapabilitySet {
    pub hold: bool,
    pub navigation: Vec<NavigationCapability>,
    pub manipulation: Option<ManipulationCapability>,
}

impl CapabilitySet {
    /// Returns whether this set implements the requested capability.
    #[must_use]
    pub fn supports(&self, requirement: CapabilityRequirement) -> bool {
        match requirement {
            CapabilityRequirement::Hold => self.hold,
            CapabilityRequirement::PlanarNavigation => self
                .navigation
                .iter()
                .any(|capability| capability.kind == NavigationKind::Planar),
            CapabilityRequirement::SpatialNavigation => self
                .navigation
                .iter()
                .any(|capability| capability.kind == NavigationKind::Spatial),
            CapabilityRequirement::Pick => {
                self.manipulation.is_some_and(|capability| capability.pick)
            }
            CapabilityRequirement::Place => {
                self.manipulation.is_some_and(|capability| capability.place)
            }
        }
    }
}

/// Current platform capability and availability advertisement.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CapabilityReport {
    pub agent_id: String,
    pub platform_profile: String,
    pub connected: bool,
    pub unavailable_reason: Option<String>,
    pub capabilities: CapabilitySet,
    pub stamp_ms: u64,
}

impl CapabilityReport {
    /// Checks both live availability and static capability support.
    ///
    /// # Errors
    ///
    /// Returns [`CapabilityRejection::PlatformUnavailable`] while the platform
    /// is disconnected, or [`CapabilityRejection::UnsupportedCapability`] when
    /// the platform does not implement the requested capability.
    pub fn require(&self, requirement: CapabilityRequirement) -> Result<(), CapabilityRejection> {
        if !self.connected {
            return Err(CapabilityRejection::PlatformUnavailable {
                reason: self
                    .unavailable_reason
                    .clone()
                    .unwrap_or_else(|| "platform is not connected".to_owned()),
            });
        }

        if !self.capabilities.supports(requirement) {
            return Err(CapabilityRejection::UnsupportedCapability { requirement });
        }

        Ok(())
    }
}

/// Why a platform cannot execute a behavior requirement.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CapabilityRejection {
    PlatformUnavailable { reason: String },
    UnsupportedCapability { requirement: CapabilityRequirement },
}

#[cfg(test)]
#[path = "../../../tests/unit/core_msgs/capabilities.rs"]
mod capabilities_tests;
