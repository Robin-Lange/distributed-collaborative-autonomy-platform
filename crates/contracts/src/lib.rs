//! Public API for the shared `core_msgs` crate.
//!
//! This crate centralizes the message vocabulary used by the autonomy stack.
//! Other crates import from here instead of redefining message structs, enum
//! codes, topic names, frame conversions, or small shared utilities.
//!
//! Main responsibilities:
//! - expose shared message structs,
//! - expose enum/code helpers,
//! - expose generated protobuf types,
//! - define DDS topic/type names,
//! - provide common time and JSON-loading helpers.

pub mod behavior;
pub mod capabilities;
pub mod catalog;
pub mod config;
pub mod constraints;
mod convert;
pub mod enums;
pub mod frames;
pub mod manipulation;
pub mod messages;
pub mod proto;
pub mod spatial;

use std::time::{SystemTime, UNIX_EPOCH};

// Re-export the stable API from the internal modules so downstream modules
// get one convenient import path through `core_msgs`.
pub use behavior::*;
pub use capabilities::*;
pub use enums::*;
pub use messages::*;
pub use spatial::*;

/// Root namespace used for DDS/Zenoh key expressions.
///
/// All shared topics live under this namespace so independent deployments do
/// not accidentally collide on the same middleware bus.
#[must_use]
pub const fn key_expr_namespace() -> &'static str {
    "decentralized_autonomy"
}

/// `approved_by` label stamped on every command the onboard agent emits.
///
/// Platform executors (`platforms/spot`, `environment_sim`) drop commands that do
/// not carry this label — a misconfiguration guard against raw-topic
/// publishers, not a security boundary. Mirrored as `APPROVED_BY` in
/// `platforms/spot/spot_sidecar.py`; the two must stay identical.
pub const APPROVED_BY_ONBOARD_AGENT: &str = "onboard_agent";

/// Deterministic command id minted for a waypoint intent: `<agent_id>-<stamp_ms>`.
///
/// Shared wire contract: the onboard agent stamps this id on the commands,
/// statuses, and vetoes derived from an intent, and operator UIs rebuild the
/// same id from the intent they sent to correlate that feedback.
#[must_use]
pub fn waypoint_command_id(agent_id: &str, stamp_ms: u64) -> String {
    format!("{agent_id}-{stamp_ms}")
}

/// Deterministic command id minted for a manipulation intent:
/// `<agent_id>-manip-<stamp_ms>`.
///
/// Same shared contract as [`waypoint_command_id`], with a `manip` marker so
/// navigation and manipulation feedback for one agent can never collide.
#[must_use]
pub fn manipulation_command_id(agent_id: &str, stamp_ms: u64) -> String {
    format!("{agent_id}-manip-{stamp_ms}")
}

/// Polling interval used by control-path DDS bridge loops.
///
/// For the onboard agent, environment simulator, and operator bus, each tick is
/// a cheap `try_recv`
/// sweep, and these processes forward commands/state/constraints on the
/// safety-relevant path, so a short interval keeps them responsive.
#[must_use]
pub const fn dds_poll_interval_ms() -> u64 {
    1
}

/// Polling interval used by observer-only DDS bridge loops.
///
/// The MCAP recorder and visualizer consume 20-40 Hz streams for
/// recording/rendering, not
/// control, so a 1 kHz wakeup buys nothing and this longer interval is used
/// instead.
#[must_use]
pub const fn dds_observer_poll_interval_ms() -> u64 {
    5
}

/// DDS topic carrying high-level waypoint requests.
#[must_use]
pub const fn topic_waypoint_intent() -> &'static str {
    "waypoint_intent"
}

/// Zenoh topic carrying typed behavior requests.
#[must_use]
pub const fn topic_behavior_request() -> &'static str {
    "behavior_request"
}

/// Zenoh topic carrying behavior lifecycle updates.
#[must_use]
pub const fn topic_behavior_status() -> &'static str {
    "behavior_status"
}

/// Zenoh topic carrying explicit operator autonomy-resume commands.
#[must_use]
pub const fn topic_resume_autonomy() -> &'static str {
    "resume_autonomy"
}

/// Zenoh topic carrying static capabilities plus live adapter availability.
#[must_use]
pub const fn topic_capability_report() -> &'static str {
    "capability_report"
}

/// Zenoh topic carrying generic controls approved by an agent.
#[must_use]
pub const fn topic_control_request() -> &'static str {
    "control_request"
}

/// Zenoh topic carrying platform execution feedback for generic controls.
#[must_use]
pub const fn topic_control_status() -> &'static str {
    "control_status"
}

/// DDS topic carrying approved platform commands.
#[must_use]
pub const fn topic_platform_command() -> &'static str {
    "platform_command"
}

/// DDS topic carrying agent state snapshots.
#[must_use]
pub const fn topic_agent_state() -> &'static str {
    "agent_state"
}

/// DDS topic carrying safety or constraint events.
#[must_use]
pub const fn topic_constraint_event() -> &'static str {
    "constraint_event"
}

/// DDS topic carrying authority/mode snapshots.
#[must_use]
pub const fn topic_authority_state() -> &'static str {
    "authority_state"
}

/// DDS topic carrying command veto messages.
#[must_use]
pub const fn topic_command_veto() -> &'static str {
    "command_veto"
}

/// DDS topic carrying command execution status updates.
#[must_use]
pub const fn topic_command_status() -> &'static str {
    "command_status"
}

/// DDS topic carrying motion-capture rigid body frames.
#[must_use]
pub const fn topic_mocap_rigid_body() -> &'static str {
    "mocap_rigid_body"
}

/// DDS topic carrying boot-time frame registration (`map` → platform body).
#[must_use]
pub const fn topic_frame_calibration() -> &'static str {
    "frame_calibration"
}

/// DDS topic carrying manipulation requests.
#[must_use]
pub const fn topic_manipulation_intent() -> &'static str {
    "manipulation_intent"
}

/// DDS topic carrying approved manipulation commands.
#[must_use]
pub const fn topic_manipulation_command() -> &'static str {
    "manipulation_command"
}

/// DDS topic carrying manipulation execution status updates.
#[must_use]
pub const fn topic_manipulation_status() -> &'static str {
    "manipulation_status"
}

/// DDS topic carrying catalog object lifecycle/pose state.
#[must_use]
pub const fn topic_object_state() -> &'static str {
    "object_state"
}

/// DDS/protobuf type name for waypoint intent messages.
#[must_use]
pub const fn type_waypoint_intent() -> &'static str {
    "WaypointIntent"
}

/// DDS/protobuf type name for platform command messages.
#[must_use]
pub const fn type_platform_command() -> &'static str {
    "PlatformCommand"
}

/// DDS/protobuf type name for agent state messages.
#[must_use]
pub const fn type_agent_state() -> &'static str {
    "AgentStateMsg"
}

/// DDS/protobuf type name for constraint event messages.
#[must_use]
pub const fn type_constraint_event() -> &'static str {
    "ConstraintEvent"
}

/// DDS/protobuf type name for authority state messages.
#[must_use]
pub const fn type_authority_state() -> &'static str {
    "AuthorityState"
}

/// DDS/protobuf type name for command veto messages.
#[must_use]
pub const fn type_command_veto() -> &'static str {
    "CommandVeto"
}

/// DDS/protobuf type name for command status messages.
#[must_use]
pub const fn type_command_status() -> &'static str {
    "CommandStatus"
}

/// DDS/protobuf type name for motion-capture rigid body frame messages.
#[must_use]
pub const fn type_mocap_rigid_body() -> &'static str {
    "MocapRigidBodyFrame"
}

/// DDS/protobuf type name for frame calibration messages.
#[must_use]
pub const fn type_frame_calibration() -> &'static str {
    "FrameCalibration"
}

/// DDS/protobuf type name for manipulation intent messages.
#[must_use]
pub const fn type_manipulation_intent() -> &'static str {
    "ManipulationIntent"
}

/// DDS/protobuf type name for manipulation command messages.
#[must_use]
pub const fn type_manipulation_command() -> &'static str {
    "ManipulationCommand"
}

/// DDS/protobuf type name for manipulation status messages.
#[must_use]
pub const fn type_manipulation_status() -> &'static str {
    "ManipulationStatus"
}

/// DDS/protobuf type name for catalog object state messages.
#[must_use]
pub const fn type_object_state() -> &'static str {
    "ObjectState"
}

/// Current time in whole milliseconds since the Unix epoch.
///
/// Message structs use millisecond stamps for readable logs, recordings, and
/// coarse timing comparisons across processes. If the millisecond value ever
/// exceeds `u64`, it saturates to `u64::MAX`.
#[must_use]
pub fn now_ms() -> u64 {
    unix_time().as_millis().try_into().unwrap_or(u64::MAX)
}
/// Current time in nanoseconds since the Unix epoch.
///
/// This is useful for systems that need a finer timestamp than milliseconds.
/// If the nanosecond value ever exceeds `u64`, it saturates to `u64::MAX`.
#[must_use]
pub fn now_ns() -> u64 {
    unix_time().as_nanos().try_into().unwrap_or(u64::MAX)
}
/// Time elapsed since the Unix epoch.
///
/// If system time is earlier than the Unix epoch, this returns zero instead of
/// panicking. That keeps timestamp generation safe on machines with bad clocks.
fn unix_time() -> std::time::Duration {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
}
