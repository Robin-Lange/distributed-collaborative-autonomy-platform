//! Typed startup configuration for the simulator and agent stack.
//!
//! Each struct maps one logical config section (agent runtime tuning, agent
//! deployment manifest, environment/containment tuning, frame registrations,
//! object catalog). The whole [`AppConfig`] tree is loaded by
//! [`load_app_config`] as a single JSON blob from the `DCA_APP_CONFIG_JSON`
//! environment variable (built once by the launcher's Setup screen and
//! reused by every spawned component), falling back to defaults when it is
//! unset or fails to parse.

use crate::capabilities::SpeedPreset;
use serde::{Deserialize, Serialize};

/// Execution backend selected independently for one configured agent.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum AgentBackend {
    /// Avian owns platform state and physics.
    #[default]
    Simulated,
    /// A platform adapter owns state and execution.
    Hardware,
    /// A platform adapter executes commands while Avian mirrors state for
    /// virtual-world constraints and objects.
    Hybrid,
}

impl AgentBackend {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Simulated => "simulated",
            Self::Hardware => "hardware",
            Self::Hybrid => "hybrid",
        }
    }

    #[must_use]
    pub const fn starts_environment(self) -> bool {
        matches!(self, Self::Simulated | Self::Hybrid)
    }

    #[must_use]
    pub const fn requires_platform(self) -> bool {
        matches!(self, Self::Hardware | Self::Hybrid)
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AppConfig {
    pub agents: Vec<AgentConfig>,
    pub environment: EnvironmentConfig,
    pub frames: FramesConfig,
    pub objects: ObjectCatalogConfig,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AgentConfig {
    pub runtime: AgentRuntimeConfig,
    pub manifest: AgentManifestConfig,
    /// Body extents in meters, `[x, y, z]`.
    pub size: [f32; 3],
    /// Spawn translation in meters, in the Bevy world frame (`+y` up).
    pub initial_position: [f32; 3],
}

impl Default for AgentConfig {
    fn default() -> Self {
        Self {
            runtime: AgentRuntimeConfig::default(),
            manifest: AgentManifestConfig::default(),
            size: [1.1, 0.5, 0.61],
            initial_position: [0.0, 0.305, 0.0],
        }
    }
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            agents: vec![AgentConfig::default()],
            environment: EnvironmentConfig::default(),
            frames: FramesConfig::default(),
            objects: ObjectCatalogConfig::default(),
        }
    }
}

// ---------------------------------------------------------
// Agent runtime tuning
// ---------------------------------------------------------
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AgentRuntimeConfig {
    pub agent_id: String,
    /// Retreat distance, in meters, used to build a `MoveAway` goal.
    pub flee_distance: f32,
}

impl Default for AgentRuntimeConfig {
    fn default() -> Self {
        Self {
            agent_id: "agent_01".to_owned(),
            flee_distance: 0.8,
        }
    }
}

// ---------------------------------------------------------
// Agent deployment manifest
// ---------------------------------------------------------
/// Deployment policy for one agent.
///
/// The launcher edits this policy per agent and passes the complete
/// configuration to every local component.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AgentManifestConfig {
    pub agent_id: String,
    pub platform: String,
    pub backend: AgentBackend,
    #[serde(default)]
    pub max_speed: SpeedPreset,
    pub command_authority: String,
    pub state_timeout_ms: u64,
    pub allow_environment_constraints: bool,

    /// When true, the agent runtime admits both the human operator and the
    /// autonomy planner as command sources (the source-blind latest-wins latch
    /// then arbitrates). Defaults to `false` via `#[serde(default)]` so existing
    /// deployment JSON that predates this field keeps the exclusive gate.
    #[serde(default)]
    pub autonomy_enabled: bool,
}

impl Default for AgentManifestConfig {
    fn default() -> Self {
        Self {
            agent_id: "agent_01".to_owned(),
            platform: "spot".to_owned(),
            backend: AgentBackend::Simulated,
            max_speed: SpeedPreset::Medium,
            command_authority: "human_operator".to_owned(),
            state_timeout_ms: 1000,
            allow_environment_constraints: true,
            autonomy_enabled: false,
        }
    }
}

// ---------------------------------------------------------
// Environment/containment tuning
// ---------------------------------------------------------
/// Arena bounds and containment tuning; all lengths are in meters.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct EnvironmentConfig {
    pub world_size: [f32; 3],
    pub wall_thickness: f32,
    #[serde(default)]
    pub containment: ContainmentConfig,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct ContainmentConfig {
    #[serde(default = "default_state_stale_ms")]
    pub state_stale_ms: u64,
    #[serde(default = "default_push_angular_deadband_deg")]
    pub push_angular_deadband_deg: f32,
}

const fn default_state_stale_ms() -> u64 {
    500
}
const fn default_push_angular_deadband_deg() -> f32 {
    5.0
}

impl Default for ContainmentConfig {
    fn default() -> Self {
        Self {
            state_stale_ms: default_state_stale_ms(),
            push_angular_deadband_deg: default_push_angular_deadband_deg(),
        }
    }
}

impl Default for EnvironmentConfig {
    fn default() -> Self {
        Self {
            world_size: [6.50, 2.0, 4.521],
            wall_thickness: 0.001,
            containment: ContainmentConfig::default(),
        }
    }
}

// ---------------------------------------------------------
// Frame registrations
// ---------------------------------------------------------
/// One frame transform: a row-major 3×3 rotation plus a translation in meters.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct FrameEntryJson {
    pub rotation_matrix_rows: [[f64; 3]; 3],
    #[serde(default)]
    pub translation_m: [f64; 3],
}

impl Default for FrameEntryJson {
    fn default() -> Self {
        Self {
            rotation_matrix_rows: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_m: [0.0, 0.0, 0.0],
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct FramesBlock {
    pub bevy_world_to_map: FrameEntryJson,
    pub mocap_lab_to_map: FrameEntryJson,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct FramesConfig {
    pub frames: FramesBlock,
}

impl Default for FramesConfig {
    fn default() -> Self {
        Self {
            frames: FramesBlock {
                bevy_world_to_map: FrameEntryJson {
                    rotation_matrix_rows: [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
                    translation_m: [0.0, 0.0, 0.0],
                },
                mocap_lab_to_map: FrameEntryJson {
                    rotation_matrix_rows: [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
                    translation_m: [0.0, 0.0, 0.0],
                },
            },
        }
    }
}

// ---------------------------------------------------------
// Object catalog
// ---------------------------------------------------------
use crate::manipulation::ReachEnvelope;

/// A planar object pose in the `map` frame (meters; `yaw_rad` in radians).
#[derive(Serialize, Deserialize, Clone, Debug, Copy)]
pub struct ObjectPose {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub yaw_rad: f32,
}

/// A manipulation-target object as written in the object catalog section.
///
/// `shape` (`"box"` / `"cylinder"`) selects which dimension fields apply;
/// `virtual_initial_pose_map` (a fixed spawn pose) versus `mocap_rigid_body_id`
/// distinguishes a virtual object from a real MOCAP-tracked one. See
/// `catalog::ObjectSpec` for the loaded counterpart and full field docs.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ObjectSpec {
    pub object_id: String,
    pub shape: String,
    #[serde(default)]
    pub length_m: f32,
    #[serde(default)]
    pub radius_m: f32,
    #[serde(default)]
    pub width_m: f32,
    #[serde(default)]
    pub height_m: f32,
    pub color: String,
    pub virtual_initial_pose_map: Option<ObjectPose>,
    pub mocap_rigid_body_id: Option<u32>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ObjectCatalogConfig {
    pub reach_envelope: ReachEnvelope,
    pub objects: Vec<ObjectSpec>,
}

impl Default for ObjectCatalogConfig {
    fn default() -> Self {
        Self {
            reach_envelope: ReachEnvelope::default(),
            objects: vec![
                ObjectSpec {
                    object_id: "cylinder_pink_01".to_owned(),
                    shape: "cylinder".to_owned(),
                    length_m: 0.5,
                    radius_m: 0.15,
                    width_m: 0.0,
                    height_m: 0.0,
                    color: "pink".to_owned(),
                    virtual_initial_pose_map: Some(ObjectPose {
                        x: 1.0,
                        y: -1.0,
                        z: 0.15,
                        yaw_rad: 1.308_997,
                    }),
                    mocap_rigid_body_id: None,
                },
                ObjectSpec {
                    object_id: "block_mocap_01".to_owned(),
                    shape: "box".to_owned(),
                    length_m: 0.30,
                    width_m: 0.09,
                    height_m: 0.09,
                    radius_m: 0.0,
                    color: "orange".to_owned(),
                    virtual_initial_pose_map: None,
                    mocap_rigid_body_id: Some(2),
                },
            ],
        }
    }
}

/// Loads [`AppConfig`] from the `DCA_APP_CONFIG_JSON` environment variable.
///
/// A malformed blob degrades to defaults (logged) instead of aborting the
/// process, so a bad override can never wedge startup before the operator
/// can correct it.
#[must_use]
pub fn load_app_config() -> AppConfig {
    if let Ok(config_json) = std::env::var("DCA_APP_CONFIG_JSON") {
        if let Ok(parsed) = serde_json::from_str(&config_json) {
            return parsed;
        }
        eprintln!("Failed to parse DCA_APP_CONFIG_JSON. Using defaults.");
    }
    AppConfig::default()
}

#[cfg(test)]
#[path = "../../../tests/unit/core_msgs/config.rs"]
mod config_tests;
