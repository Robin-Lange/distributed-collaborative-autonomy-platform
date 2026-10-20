//! Configuration types for the onboard agent.
//!
//! `AgentConfig` contains tuning values for decision logic, while
//! `AgentManifest` contains deployment policy: backend, platform, command
//! authority, state timeout, and whether environment constraints are trusted.

use core_msgs::SpeedPreset;
use core_msgs::config::AgentBackend;
use serde::Deserialize;
use std::env;

/// Runtime tuning loaded from the agent's `AppConfig` runtime section.
#[derive(Deserialize)]
pub struct AgentConfig {
    pub agent_id: String,

    pub flee_distance: f32,
}

impl Default for AgentConfig {
    fn default() -> Self {
        Self {
            agent_id: "agent_01".to_owned(),
            flee_distance: 0.8,
        }
    }
}

impl From<&core_msgs::config::AgentRuntimeConfig> for AgentConfig {
    fn from(runtime: &core_msgs::config::AgentRuntimeConfig) -> Self {
        Self {
            agent_id: runtime.agent_id.clone(),
            flee_distance: runtime.flee_distance,
        }
    }
}

/// Deployment policy loaded for one agent.
#[derive(Clone, Debug)]
pub struct AgentManifest {
    pub agent_id: String,

    pub backend: AgentBackend,

    pub platform: String,

    pub max_speed: SpeedPreset,

    pub command_authority: String,

    pub state_timeout_ms: u64,

    pub allow_environment_constraints: bool,

    /// When true, both the human operator and the autonomy planner may command
    /// the platform; the runtime inbox's source-blind latest-wins latch then
    /// decides which most-recent intent drives motion (so the operator preempts
    /// autonomy by commanding again). When false (default) only the single
    /// `command_authority` source is admitted, preserving the exclusive gate.
    pub autonomy_enabled: bool,
}

impl Default for AgentManifest {
    fn default() -> Self {
        Self {
            agent_id: "agent_01".to_owned(),
            backend: AgentBackend::Simulated,
            platform: "spot".to_owned(),
            max_speed: SpeedPreset::Medium,
            command_authority: "human_operator".to_owned(),
            state_timeout_ms: 1000,
            allow_environment_constraints: true,
            autonomy_enabled: false,
        }
    }
}

impl AgentManifest {
    /// Builds the runtime manifest from the configured deployment policy.
    pub(crate) fn new(manifest: &core_msgs::config::AgentManifestConfig) -> Self {
        Self {
            agent_id: manifest.agent_id.clone(),
            backend: manifest.backend,
            platform: manifest.platform.clone(),
            max_speed: manifest.max_speed,
            command_authority: manifest.command_authority.clone(),
            state_timeout_ms: manifest.state_timeout_ms,
            allow_environment_constraints: manifest.allow_environment_constraints,
            autonomy_enabled: manifest.autonomy_enabled,
        }
    }
}

/// Loads the generic static capabilities selected by the launcher.
pub fn load_capabilities(platform_profile: &str) -> core_msgs::CapabilitySet {
    let json = env::var(core_msgs::AGENT_CAPABILITIES_ENV).unwrap_or_else(|_| {
        panic!(
            "{} is required; the onboard agent must be started via the launcher",
            core_msgs::AGENT_CAPABILITIES_ENV
        )
    });

    decode_capabilities(json.as_str(), platform_profile).unwrap_or_else(|error| {
        panic!(
            "invalid {} for platform profile '{platform_profile}': {error}",
            core_msgs::AGENT_CAPABILITIES_ENV
        )
    })
}

fn decode_capabilities(
    json: &str,
    platform_profile: &str,
) -> Result<core_msgs::CapabilitySet, String> {
    let capabilities: core_msgs::CapabilitySet =
        serde_json::from_str(json).map_err(|error| error.to_string())?;

    if capabilities.navigation.iter().any(|capability| {
        let limits = [
            capability.speed_presets.slow,
            capability.speed_presets.medium,
            capability.speed_presets.fast,
        ];
        limits.iter().any(|limit| {
            !limit.max_linear_speed_m_s.is_finite()
                || limit.max_linear_speed_m_s <= 0.0
                || !limit.max_yaw_rate_rad_s.is_finite()
                || limit.max_yaw_rate_rad_s <= 0.0
        }) || !capability.position_tolerance_m.is_finite()
            || capability.position_tolerance_m < 0.0
    }) {
        return Err(format!(
            "platform profile '{platform_profile}' has invalid navigation limits"
        ));
    }

    Ok(capabilities)
}

#[cfg(test)]
#[path = "../../../tests/unit/agent/config.rs"]
mod config_tests;
