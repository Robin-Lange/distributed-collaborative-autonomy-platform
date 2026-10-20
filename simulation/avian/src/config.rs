//! Configuration model for the environment simulator.
//!
//! These values describe both the physics world and the simulator role.

use bevy::prelude::Resource;
use core_msgs::config::AppConfig;

/// Deserialized simulator configuration inserted as a Bevy resource.
#[derive(Resource, Clone)]
pub struct EnvironmentConfig {
    pub(super) app: AppConfig,
}

impl EnvironmentConfig {
    pub(super) const fn new(app: AppConfig) -> Self {
        Self { app }
    }

    pub(super) const fn world_size(&self) -> [f32; 3] {
        self.app.environment.world_size
    }

    pub(super) const fn wall_thickness(&self) -> f32 {
        self.app.environment.wall_thickness
    }

    pub(super) const fn push_angular_deadband_rad(&self) -> f32 {
        self.app
            .environment
            .containment
            .push_angular_deadband_deg
            .to_radians()
    }

    /// Size `[length, width, height]` of the agent with `agent_id`, falling
    /// back to unit dimensions if no such agent is configured. Per-agent (not
    /// first-agent) so held-object carry poses use the carrying agent's own
    /// footprint when agents have different sizes.
    pub(super) fn agent_size(&self, agent_id: &str) -> [f32; 3] {
        self.app
            .agents
            .iter()
            .find(|agent| agent.runtime.agent_id == agent_id)
            .map_or([1.0, 1.0, 1.0], |agent| agent.size)
    }
}
