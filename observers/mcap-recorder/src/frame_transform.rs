//! Foxglove frame-transform support for MCAP recordings.
//!
//! Agent state is recorded as native protobuf, but Foxglove can also display pose
//! through a JSON `FrameTransform` channel. This module defines that JSON schema
//! and converts each agent-state sample into a transform from `map` to `agent`.

use core_msgs::proto::AgentStateMsg;
use glam::Quat;
use serde::Serialize;
use std::time::Duration;

/// MCAP topic used for Foxglove transform messages.
pub const TOPIC_AGENT_TF: &str = "agent_tf";

/// Schema name recognized by Foxglove for frame transforms.
pub const SCHEMA_FRAME_TRANSFORM: &str = "foxglove.FrameTransform";

/// Parent frame for the agent pose transform: the canonical bus `map` frame
/// (`AgentStateMsg.position` is always `map`-framed; see `frame_id`).
const PARENT_FRAME_ID: &str = "map";

/// Child frame for the agent body.
const CHILD_FRAME_ID: &str = "agent";

/// JSON schema registered with MCAP for the Foxglove transform channel.
pub const FRAME_TRANSFORM_JSON_SCHEMA: &str = r#"{
  "title": "foxglove.FrameTransform",
  "description": "A transform between two reference frames in 3D space.",
  "type": "object",
  "properties": {
    "timestamp": {
      "type": "object",
      "title": "time",
      "properties": {
        "sec": { "type": "integer", "minimum": 0 },
        "nsec": { "type": "integer", "minimum": 0, "maximum": 999999999 }
      },
      "description": "Timestamp of transform"
    },
    "parent_frame_id": { "type": "string", "description": "Name of the parent frame" },
    "child_frame_id": { "type": "string", "description": "Name of the child frame" },
    "translation": {
      "title": "foxglove.Vector3",
      "description": "Position of the child frame's origin in the parent frame.",
      "type": "object",
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" },
        "z": { "type": "number" }
      },
      "required": ["x", "y", "z"]
    },
    "rotation": {
      "title": "foxglove.Quaternion",
      "description": "Orientation of the child frame in the parent frame.",
      "type": "object",
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" },
        "z": { "type": "number" },
        "w": { "type": "number" }
      },
      "required": ["x", "y", "z", "w"]
    }
  },
  "required": ["timestamp", "parent_frame_id", "child_frame_id", "translation", "rotation"]
}"#;

/// Foxglove timestamp split into seconds and nanoseconds.
#[derive(Serialize)]
struct Timestamp {
    /// Whole seconds since Unix epoch.
    sec: u64,

    /// Nanosecond remainder within the second.
    nsec: u32,
}

/// Serializable 3D translation vector.
#[derive(Serialize)]
struct Vector3 {
    /// X translation in the shared wire frame.
    x: f32,

    /// Y translation in the shared wire frame.
    y: f32,

    /// Z translation in the shared wire frame.
    z: f32,
}

/// Serializable unit quaternion.
#[derive(Serialize)]
struct Quaternion {
    /// Quaternion X component.
    x: f32,

    /// Quaternion Y component.
    y: f32,

    /// Quaternion Z component.
    z: f32,

    /// Quaternion W component.
    w: f32,
}

/// Foxglove-compatible transform from the world frame to the agent frame.
#[derive(Serialize)]
pub struct FrameTransform {
    /// Timestamp copied from the source agent-state message.
    timestamp: Timestamp,

    /// Parent frame name.
    parent_frame_id: &'static str,

    /// Child frame name.
    child_frame_id: &'static str,

    /// Agent position in the parent frame.
    translation: Vector3,

    /// Agent orientation: `Rz(heading_yaw_rad)`, observation-only.
    rotation: Quaternion,
}

/// Converts a protobuf agent-state sample into a Foxglove transform.
///
/// `AgentStateMsg.position` is always in the canonical `map` frame: ENU,
/// `+X` right/east, `+Y` forward/north, `+Z` up. Because the transform's
/// parent frame is also `map`, the translation preserves canonical map
/// coordinates directly and does not apply a display-frame basis change.
/// The rotation is `Rz(heading_yaw_rad)`: CCW about map `+Z`, `0` = `+X`
/// (east), matching the observation-only heading contract.
pub fn from_agent_state(state: &AgentStateMsg) -> FrameTransform {
    let core_msgs::proto::Vec3 { x, y, z } = state.position.unwrap_or_default();
    let [rx, ry, rz, rw] = Quat::from_rotation_z(state.heading_yaw_rad).to_array();
    let stamp = Duration::from_millis(state.stamp_ms);

    FrameTransform {
        timestamp: Timestamp {
            sec: stamp.as_secs(),
            nsec: stamp.subsec_nanos(),
        },
        parent_frame_id: PARENT_FRAME_ID,
        child_frame_id: CHILD_FRAME_ID,
        translation: Vector3 { x, y, z },
        rotation: Quaternion {
            x: rx,
            y: ry,
            z: rz,
            w: rw,
        },
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/mcap_recorder/frame_transform.rs"]
mod tests;
