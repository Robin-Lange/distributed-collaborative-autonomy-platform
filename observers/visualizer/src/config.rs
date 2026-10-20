use bevy::prelude::Resource;
use core_msgs::config::AppConfig;

#[derive(Resource, Clone)]
pub struct VisualizerConfig {
    pub(super) app: AppConfig,
}

impl VisualizerConfig {
    pub(super) const fn new(app: AppConfig) -> Self {
        Self { app }
    }

    pub(super) const fn world_size(&self) -> [f32; 3] {
        self.app.environment.world_size
    }

    pub(super) const fn wall_thickness(&self) -> f32 {
        self.app.environment.wall_thickness
    }

    /// First configured agent's id, or `"unknown"` when no agents are set.
    pub(super) fn agent_id(&self) -> &str {
        if let Some(agent) = self.app.agents.first() {
            &agent.runtime.agent_id
        } else {
            "unknown"
        }
    }
}
