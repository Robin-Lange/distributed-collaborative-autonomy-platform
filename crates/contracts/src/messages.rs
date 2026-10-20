//! Shared message types exchanged between processes.
//!
//! These structs define the in-memory Rust representation of the messages used
//! across the autonomy stack. Each message includes an `agent_id` so shared DDS
//! topics can carry data for multiple agents while each process can filter down
//! to the agent it owns or monitors.
//!
//! Most enum-like fields are stored as `u8` wire codes instead of Rust enums.
//! That keeps the serialized format stable across DDS/protobuf boundaries while
//! helper functions elsewhere translate codes into human-readable names.

use serde::{Deserialize, Serialize};

/// Common interface for messages that belong to a specific agent.
///
/// DDS topics may contain messages for many agents. This trait lets generic
/// helpers filter incoming messages by `agent_id` without needing to know the
/// concrete message type.
pub trait AgentScoped {
    fn agent_id(&self) -> &str;
}

/// Snapshot of which subsystem currently owns authority for an agent.
///
/// Authority determines which source is allowed to command the agent, publish
/// state, simulate physics, or report collisions. This is useful for debugging
/// mode switches between simulation, autonomy, and real-platform control.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct AuthorityState {
    pub agent_id: String,
    pub command_authority: u8,
    pub state_authority: u8,
    pub physics_authority: u8,
    pub collision_authority: u8,
    pub mode: u8,
    pub reason: String,
    pub stamp_ms: u64,
}

impl AgentScoped for AuthorityState {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Safety or constraint event observed for an agent.
///
/// Constraint events describe conditions that may limit or stop command
/// execution, such as collisions, blocked movement, or hard-stop safety cases.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ConstraintEvent {
    /// Agent affected by this constraint.
    pub agent_id: String,
    pub source: u8,
    pub collision_authority: u8,
    pub constraint_type: u8,
    pub severity: u8,
    pub object_id: String,
    pub stamp_ms: u64,

    /// Correlates a hard-stop episode across constraint -> hold -> stop ->
    /// clear; empty when not part of a tracked episode.
    pub episode_id: String,

    /// Trigger kind for containment events ("overlap", "swept",
    /// "`stale_state`"), or empty for non-containment constraints.
    pub detail: String,
}

impl AgentScoped for ConstraintEvent {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// High-level waypoint request from an operator or autonomy source.
///
/// This is an intent message, not a guaranteed platform command. Downstream
/// authority and safety logic may approve, transform, or veto the request before
/// it becomes a `PlatformCommand`.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct WaypointIntent {
    pub agent_id: String,
    pub source: u8,
    pub behavior: u8,
    pub point: [f32; 3],
    pub stamp_ms: u64,
    pub frame_id: String,
}

impl AgentScoped for WaypointIntent {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// SE2 velocity setpoint carried by `command_type_velocity` commands.
///
/// Expressed in Spot's gravity-aligned `flat_body` frame: `+x` forward, `+y`
/// left, yaw CCW about `+z`. The sidecar forwards it verbatim as an
/// `SE2VelocityCommand` with `se2_frame_name="flat_body"` — never `"body"`,
/// which the SDK rejects for velocity requests.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct VelocitySetpoint {
    pub forward_m_s: f32,
    pub left_m_s: f32,
    pub yaw_rate_rad_s: f32,
}

/// Low-level command approved for execution by the platform layer.
///
/// A `PlatformCommand` is more concrete than a `WaypointIntent`: it has passed
/// the relevant authority/safety checks and includes a command ID that status
/// messages can reference.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct PlatformCommand {
    pub agent_id: String,
    pub command_id: String,
    pub command_type: u8,
    pub source: u8,
    pub approved_by: String,
    pub target_position: [f32; 3],
    pub stamp_ms: u64,
    pub frame_id: String,

    /// For a HOLD command, the `episode_id` of the triggering constraint;
    /// empty when not applicable. Always empty on trajectory commands.
    pub episode_id: String,

    pub command_authority: u8,

    /// Present only on `command_type_velocity` commands; `None` otherwise.
    pub velocity: Option<VelocitySetpoint>,
}

impl AgentScoped for PlatformCommand {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Status update for a previously issued platform command.
///
/// Status messages allow the rest of the system to determine whether a command
/// is still running, completed successfully, failed, or was otherwise stopped.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct CommandStatus {
    pub agent_id: String,
    pub command_id: String,
    pub status: u8,
    pub stamp_ms: u64,
}

impl AgentScoped for CommandStatus {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Current state sample for an agent.
///
/// This message carries the agent's latest known position and the authority that
/// produced it. Consumers should treat it as a timestamped snapshot rather than
/// a long-lived state object.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct AgentStateMsg {
    pub agent_id: String,
    pub source: u8,

    /// Authority responsible for this state sample.
    pub state_authority: u8,
    pub position: [f32; 3],
    pub stamp_ms: u64,
    pub frame_id: String,

    /// Heading: CCW about map `+Z`, `0` = `+X` (east). Load-bearing: in real
    /// mode this feeds the onboard velocity controller's map -> `flat_body`
    /// error rotation, in addition to navigation/collider rendering.
    pub heading_yaw_rad: f32,

    /// Linear velocity in the `map` frame, m/s (`z` is vertical, unused by the
    /// planar controller). From Spot odometry in real mode.
    pub velocity: [f32; 3],

    /// Yaw rate, rad/s, CCW about map `+Z`. From Spot odometry in real mode.
    pub yaw_rate_rad_s: f32,
}

impl AgentScoped for AgentStateMsg {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Rejection message for a command that was not allowed to execute.
///
/// Vetoes explain why a requested command was rejected, usually because of
/// authority, safety, mode, or policy constraints.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct CommandVeto {
    pub agent_id: String,
    pub command_id: String,
    pub source: u8,
    pub reason: u8,
    pub detail: String,
    pub stamp_ms: u64,
}

impl AgentScoped for CommandVeto {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// A yaw-only planar pose (`map` frame unless otherwise documented).
///
/// The stack is deliberately yaw-only for manipulation: objects sit on the
/// floor, so a full 6-DoF orientation is unneeded complexity. This struct
/// doubles as the catalog's on-disk pose representation (see
/// `core_msgs::catalog`) and the wire pose carried by manipulation messages.
/// A real, MOCAP-tracked object's full orientation rides alongside this on
/// `ObjectState.orientation_map`
/// (observation/rendering only — the manipulation/reach-envelope path below
/// still reads only `yaw_rad`).
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct PoseXYZYaw {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub yaw_rad: f32,
}

/// A unit quaternion `(x, y, z, w)`. Wire-format mirror of `proto::Quat`.
///
/// Carried only where a full orientation is genuinely needed: currently
/// `ObjectState.orientation_map`. Everywhere else on the bus stays yaw-only
/// (`PoseXYZYaw`) by design.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

/// Request to manipulate a known catalog object.
///
/// Mirrors `WaypointIntent`: not yet authority/safety/reachability checked.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ManipulationIntent {
    pub agent_id: String,
    pub object_id: String,
    pub verb: u8,
    pub source: u8,
    pub stamp_ms: u64,
    /// `"map"`; unused while every pose resolves from the object catalog.
    pub frame_id: String,
}

impl AgentScoped for ManipulationIntent {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Approved manipulation command, analogous to `PlatformCommand`.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ManipulationCommand {
    pub agent_id: String,
    pub command_id: String,
    pub object_id: String,
    pub verb: u8,
    pub approved_by: String,

    /// Resolved target pose (catalog pose; overrides are a future addition).
    pub target_pose: PoseXYZYaw,
    pub stamp_ms: u64,
    pub frame_id: String,

    /// Named hand-orientation profile, encoded as a shared wire code
    /// (`manipulation_grasp_profile_*`). Onboard-agent-decided, not the
    /// sidecar's — the executor only derives orientation from it plus
    /// `target_pose.yaw_rad`.
    pub grasp_profile: u8,
}

impl AgentScoped for ManipulationCommand {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Execution status update for a previously issued `ManipulationCommand`.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ManipulationStatus {
    pub agent_id: String,
    pub command_id: String,
    pub object_id: String,
    pub phase: u8,
    pub failure_reason: u8,
    pub stamp_ms: u64,
}

impl AgentScoped for ManipulationStatus {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Current lifecycle/pose of one catalog object.
///
/// Not agent-scoped: an object belongs to the shared catalog, not to one
/// agent's topic filter. Consumers key on `object_id` instead.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ObjectState {
    pub object_id: String,
    pub lifecycle: u8,

    /// Agent currently holding the object; empty when not held.
    pub holder_agent_id: String,
    pub pose: PoseXYZYaw,

    /// Whether `pose` is currently trustworthy (e.g. false when a real-mode
    /// tracker has lost the object).
    pub pose_valid: bool,
    pub stamp_ms: u64,

    /// Full orientation in `map`, present only for a real, MOCAP-tracked
    /// object once `object_tracker` has an observation.
    /// `None` for a virtual catalog object (stays yaw-only via
    /// `pose.yaw_rad`) or before the first observation; also cleared
    /// whenever `pose_valid` is false, so a consumer never renders a stale
    /// orientation alongside a pose it has already discounted.
    pub orientation_map: Option<Quat>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ContactState {
    pub agent_id: String,
    pub source: u8,
    pub collision_authority: u8,
    pub other_entity_id: String,
    pub contact_normal: [f32; 3],
    pub stamp_ms: u64,
}
impl AgentScoped for ContactState {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct InteractionEvent {
    pub agent_id: String,
    pub source: u8,
    pub interaction_authority: u8,
    pub object_id: String,
    pub event_type: String,
    pub reason: String,
    pub stamp_ms: u64,
}
impl AgentScoped for InteractionEvent {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}
