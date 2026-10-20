//! Conversion layer between internal Rust messages and generated protobuf messages.
//!
//! The application uses plain Rust structs from `messages` internally, while DDS
//! publishing/subscribing uses generated protobuf structs from `proto`. This file
//! defines the `From` conversions in both directions so the rest of the codebase
//! can cross that boundary with `.into()` / `::from(...)` instead of manually
//! copying fields at every call site.
//!
//! A few fields need small representation changes at the boundary:
//! - Internal enum-like codes are stored as `u8`.
//! - Protobuf enum/int fields are generated as `i32`.
//! - Internal vectors are `[f32; 3]`.
//! - Protobuf vectors are nested `Option<proto::Vec3>` messages.

use super::{messages, proto};

/// Narrows a protobuf enum/int field to the shared `u8` wire-code space.
///
/// Every code family in `core_msgs::enums` fits in one byte by contract, so
/// this takes the value's low byte — bit-identical to the `as u8` narrowing
/// it replaces, including for any out-of-contract value a non-conforming
/// peer might send.
const fn wire_code(value: i32) -> u8 {
    value.to_le_bytes()[0]
}

/// Converts an internal `[x, y, z]` position array into a protobuf `Vec3`.
///
/// Internally, positions are stored as fixed-size arrays because they are small,
/// copyable, and easy to use in simulation code. Protobuf represents the same
/// value as a named message with `x`, `y`, and `z` fields.
impl From<[f32; 3]> for proto::Vec3 {
    fn from(value: [f32; 3]) -> Self {
        Self {
            x: value[0],
            y: value[1],
            z: value[2],
        }
    }
}

/// Converts a protobuf `Vec3` into the internal `[x, y, z]` array format.
impl From<proto::Vec3> for [f32; 3] {
    fn from(value: proto::Vec3) -> Self {
        [value.x, value.y, value.z]
    }
}

/// Converts an internal authority snapshot into its protobuf representation.
///
/// The internal message stores authority/mode values as compact `u8` codes.
/// The generated protobuf struct stores those numeric fields as `i32`, so this
/// conversion widens each code before publishing or encoding.
impl From<messages::AuthorityState> for proto::AuthorityState {
    fn from(value: messages::AuthorityState) -> Self {
        Self {
            agent_id: value.agent_id,
            command_authority: i32::from(value.command_authority),
            state_authority: i32::from(value.state_authority),
            physics_authority: i32::from(value.physics_authority),
            collision_authority: i32::from(value.collision_authority),
            mode: i32::from(value.mode),
            reason: value.reason,
            stamp_ms: value.stamp_ms,
        }
    }
}

/// Converts a protobuf authority snapshot into the internal Rust message.
///
/// Protobuf numeric codes are narrowed back to `u8` because the shared enum-code
/// vocabulary in `core_msgs` is represented as byte-sized values internally.
impl From<proto::AuthorityState> for messages::AuthorityState {
    fn from(value: proto::AuthorityState) -> Self {
        Self {
            agent_id: value.agent_id,
            command_authority: wire_code(value.command_authority),
            state_authority: wire_code(value.state_authority),
            physics_authority: wire_code(value.physics_authority),
            collision_authority: wire_code(value.collision_authority),
            mode: wire_code(value.mode),
            reason: value.reason,
            stamp_ms: value.stamp_ms,
        }
    }
}

/// Converts an internal constraint/safety event into protobuf form.
impl From<messages::ConstraintEvent> for proto::ConstraintEvent {
    fn from(value: messages::ConstraintEvent) -> Self {
        Self {
            agent_id: value.agent_id,
            source: i32::from(value.source),
            collision_authority: i32::from(value.collision_authority),
            constraint_type: i32::from(value.constraint_type),
            severity: i32::from(value.severity),
            object_id: value.object_id,
            stamp_ms: value.stamp_ms,
            episode_id: value.episode_id,
            detail: value.detail,
        }
    }
}

/// Converts a protobuf constraint/safety event into the internal Rust message.
impl From<proto::ConstraintEvent> for messages::ConstraintEvent {
    fn from(value: proto::ConstraintEvent) -> Self {
        Self {
            agent_id: value.agent_id,
            source: wire_code(value.source),
            collision_authority: wire_code(value.collision_authority),
            constraint_type: wire_code(value.constraint_type),
            severity: wire_code(value.severity),
            object_id: value.object_id,
            stamp_ms: value.stamp_ms,
            episode_id: value.episode_id,
            detail: value.detail,
        }
    }
}

/// Converts an internal waypoint intent into protobuf form.
///
/// `point` is wrapped in `Some(...)` because protobuf represents nested messages
/// as optional fields in the generated Rust type.
impl From<messages::WaypointIntent> for proto::WaypointIntent {
    fn from(value: messages::WaypointIntent) -> Self {
        Self {
            agent_id: value.agent_id,
            source: i32::from(value.source),
            behavior: i32::from(value.behavior),
            point: Some(value.point.into()),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
        }
    }
}

/// Converts a protobuf waypoint intent into the internal Rust message.
///
/// If the protobuf message is missing its `point`, this falls back to the
/// default `Vec3`, which becomes `[0.0, 0.0, 0.0]`. That keeps malformed or
/// partially populated wire messages from crashing the receiver.
impl From<proto::WaypointIntent> for messages::WaypointIntent {
    fn from(value: proto::WaypointIntent) -> Self {
        Self {
            agent_id: value.agent_id,
            source: wire_code(value.source),
            behavior: wire_code(value.behavior),
            point: value.point.unwrap_or_default().into(),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
        }
    }
}

/// Converts an internal SE2 velocity setpoint into protobuf form.
impl From<messages::VelocitySetpoint> for proto::Se2VelocitySetpoint {
    fn from(value: messages::VelocitySetpoint) -> Self {
        Self {
            forward_m_s: value.forward_m_s,
            left_m_s: value.left_m_s,
            yaw_rate_rad_s: value.yaw_rate_rad_s,
        }
    }
}

/// Converts a protobuf SE2 velocity setpoint into the internal Rust message.
impl From<proto::Se2VelocitySetpoint> for messages::VelocitySetpoint {
    fn from(value: proto::Se2VelocitySetpoint) -> Self {
        Self {
            forward_m_s: value.forward_m_s,
            left_m_s: value.left_m_s,
            yaw_rate_rad_s: value.yaw_rate_rad_s,
        }
    }
}

/// Converts an approved internal platform command into protobuf form.
///
/// This is the command message sent to platform adapters or visualizers after a
/// higher-level intent has passed authority and safety checks.
impl From<messages::PlatformCommand> for proto::PlatformCommand {
    fn from(value: messages::PlatformCommand) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            command_type: i32::from(value.command_type),
            source: i32::from(value.source),
            approved_by: value.approved_by,
            target_position: Some(value.target_position.into()),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
            episode_id: value.episode_id,
            command_authority: i32::from(value.command_authority),
            velocity: value.velocity.map(Into::into),
        }
    }
}

/// Converts a protobuf platform command into the internal Rust message.
///
/// Missing `target_position` values are treated as the default zero vector so
/// receivers can still build a complete internal message.
impl From<proto::PlatformCommand> for messages::PlatformCommand {
    fn from(value: proto::PlatformCommand) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            command_type: wire_code(value.command_type),
            source: wire_code(value.source),
            approved_by: value.approved_by,
            target_position: value.target_position.unwrap_or_default().into(),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
            episode_id: value.episode_id,
            command_authority: wire_code(value.command_authority),
            velocity: value.velocity.map(Into::into),
        }
    }
}

/// Converts an internal command status update into protobuf form.
impl From<messages::CommandStatus> for proto::CommandStatus {
    fn from(value: messages::CommandStatus) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            status: i32::from(value.status),
            stamp_ms: value.stamp_ms,
        }
    }
}

/// Converts a protobuf command status update into the internal Rust message.
impl From<proto::CommandStatus> for messages::CommandStatus {
    fn from(value: proto::CommandStatus) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            status: wire_code(value.status),
            stamp_ms: value.stamp_ms,
        }
    }
}

/// Converts an internal agent state sample into protobuf form.
///
/// Position is wrapped as an optional nested protobuf message because that is
/// how `prost` generates message fields.
impl From<messages::AgentStateMsg> for proto::AgentStateMsg {
    fn from(value: messages::AgentStateMsg) -> Self {
        Self {
            agent_id: value.agent_id,
            source: i32::from(value.source),
            state_authority: i32::from(value.state_authority),
            position: Some(value.position.into()),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
            heading_yaw_rad: value.heading_yaw_rad,
            velocity: Some(value.velocity.into()),
            yaw_rate_rad_s: value.yaw_rate_rad_s,
        }
    }
}

/// Converts a protobuf agent state sample into the internal Rust message.
///
/// If the wire message omits `position`, the receiver uses `[0.0, 0.0, 0.0]` as
/// a safe default instead of panicking.
impl From<proto::AgentStateMsg> for messages::AgentStateMsg {
    fn from(value: proto::AgentStateMsg) -> Self {
        Self {
            agent_id: value.agent_id,
            source: wire_code(value.source),
            state_authority: wire_code(value.state_authority),
            position: value.position.unwrap_or_default().into(),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
            heading_yaw_rad: value.heading_yaw_rad,
            velocity: value.velocity.unwrap_or_default().into(),
            yaw_rate_rad_s: value.yaw_rate_rad_s,
        }
    }
}

/// Converts an internal command veto into protobuf form.
///
/// Veto messages explain why a command was rejected before execution.
impl From<messages::CommandVeto> for proto::CommandVeto {
    fn from(value: messages::CommandVeto) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            source: i32::from(value.source),
            reason: i32::from(value.reason),
            detail: value.detail,
            stamp_ms: value.stamp_ms,
        }
    }
}

/// Converts a protobuf command veto into the internal Rust message.
impl From<proto::CommandVeto> for messages::CommandVeto {
    fn from(value: proto::CommandVeto) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            source: wire_code(value.source),
            reason: wire_code(value.reason),
            detail: value.detail,
            stamp_ms: value.stamp_ms,
        }
    }
}

/// Converts an internal yaw-only pose into protobuf form.
impl From<messages::PoseXYZYaw> for proto::PoseXyzYaw {
    fn from(value: messages::PoseXYZYaw) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            yaw_rad: value.yaw_rad,
        }
    }
}

/// Converts a protobuf yaw-only pose into the internal Rust message.
impl From<proto::PoseXyzYaw> for messages::PoseXYZYaw {
    fn from(value: proto::PoseXyzYaw) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            yaw_rad: value.yaw_rad,
        }
    }
}

/// Converts an internal manipulation intent into protobuf form.
impl From<messages::ManipulationIntent> for proto::ManipulationIntent {
    fn from(value: messages::ManipulationIntent) -> Self {
        Self {
            agent_id: value.agent_id,
            object_id: value.object_id,
            verb: i32::from(value.verb),
            source: i32::from(value.source),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
        }
    }
}

/// Converts a protobuf manipulation intent into the internal Rust message.
impl From<proto::ManipulationIntent> for messages::ManipulationIntent {
    fn from(value: proto::ManipulationIntent) -> Self {
        Self {
            agent_id: value.agent_id,
            object_id: value.object_id,
            verb: wire_code(value.verb),
            source: wire_code(value.source),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
        }
    }
}

/// Converts an internal manipulation command into protobuf form.
impl From<messages::ManipulationCommand> for proto::ManipulationCommand {
    fn from(value: messages::ManipulationCommand) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            object_id: value.object_id,
            verb: i32::from(value.verb),
            approved_by: value.approved_by,
            target_pose: Some(value.target_pose.into()),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
            grasp_profile: i32::from(value.grasp_profile),
        }
    }
}

/// Converts a protobuf manipulation command into the internal Rust message.
impl From<proto::ManipulationCommand> for messages::ManipulationCommand {
    fn from(value: proto::ManipulationCommand) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            object_id: value.object_id,
            verb: wire_code(value.verb),
            approved_by: value.approved_by,
            target_pose: value.target_pose.unwrap_or_default().into(),
            stamp_ms: value.stamp_ms,
            frame_id: value.frame_id,
            grasp_profile: wire_code(value.grasp_profile),
        }
    }
}

/// Converts an internal manipulation status into protobuf form.
impl From<messages::ManipulationStatus> for proto::ManipulationStatus {
    fn from(value: messages::ManipulationStatus) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            object_id: value.object_id,
            phase: i32::from(value.phase),
            failure_reason: i32::from(value.failure_reason),
            stamp_ms: value.stamp_ms,
        }
    }
}

/// Converts a protobuf manipulation status into the internal Rust message.
impl From<proto::ManipulationStatus> for messages::ManipulationStatus {
    fn from(value: proto::ManipulationStatus) -> Self {
        Self {
            agent_id: value.agent_id,
            command_id: value.command_id,
            object_id: value.object_id,
            phase: wire_code(value.phase),
            failure_reason: wire_code(value.failure_reason),
            stamp_ms: value.stamp_ms,
        }
    }
}

/// Converts an internal quaternion into protobuf form.
impl From<messages::Quat> for proto::Quat {
    fn from(value: messages::Quat) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            w: value.w,
        }
    }
}

/// Converts a protobuf quaternion into the internal Rust message.
impl From<proto::Quat> for messages::Quat {
    fn from(value: proto::Quat) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            w: value.w,
        }
    }
}

/// Converts an internal object state into protobuf form.
impl From<messages::ObjectState> for proto::ObjectState {
    fn from(value: messages::ObjectState) -> Self {
        Self {
            object_id: value.object_id,
            lifecycle: i32::from(value.lifecycle),
            holder_agent_id: value.holder_agent_id,
            pose: Some(value.pose.into()),
            pose_valid: value.pose_valid,
            stamp_ms: value.stamp_ms,
            orientation_map: value.orientation_map.map(Into::into),
        }
    }
}

/// Converts a protobuf object state into the internal Rust message.
impl From<proto::ObjectState> for messages::ObjectState {
    fn from(value: proto::ObjectState) -> Self {
        Self {
            object_id: value.object_id,
            lifecycle: wire_code(value.lifecycle),
            holder_agent_id: value.holder_agent_id,
            pose: value.pose.unwrap_or_default().into(),
            pose_valid: value.pose_valid,
            stamp_ms: value.stamp_ms,
            orientation_map: value.orientation_map.map(Into::into),
        }
    }
}

impl From<messages::ContactState> for proto::ContactState {
    fn from(value: messages::ContactState) -> Self {
        Self {
            agent_id: value.agent_id,
            source: i32::from(value.source),
            collision_authority: i32::from(value.collision_authority),
            other_entity_id: value.other_entity_id,
            contact_normal: Some(proto::Vec3 {
                x: value.contact_normal[0],
                y: value.contact_normal[1],
                z: value.contact_normal[2],
            }),
            stamp_ms: value.stamp_ms,
        }
    }
}
impl From<proto::ContactState> for messages::ContactState {
    fn from(value: proto::ContactState) -> Self {
        Self {
            agent_id: value.agent_id,
            source: wire_code(value.source),
            collision_authority: wire_code(value.collision_authority),
            other_entity_id: value.other_entity_id,
            contact_normal: value.contact_normal.map_or([0.0; 3], |v| [v.x, v.y, v.z]),
            stamp_ms: value.stamp_ms,
        }
    }
}

impl From<messages::InteractionEvent> for proto::InteractionEvent {
    fn from(value: messages::InteractionEvent) -> Self {
        Self {
            agent_id: value.agent_id,
            source: i32::from(value.source),
            interaction_authority: i32::from(value.interaction_authority),
            object_id: value.object_id,
            event_type: value.event_type,
            reason: value.reason,
            stamp_ms: value.stamp_ms,
        }
    }
}
impl From<proto::InteractionEvent> for messages::InteractionEvent {
    fn from(value: proto::InteractionEvent) -> Self {
        Self {
            agent_id: value.agent_id,
            source: wire_code(value.source),
            interaction_authority: wire_code(value.interaction_authority),
            object_id: value.object_id,
            event_type: value.event_type,
            reason: value.reason,
            stamp_ms: value.stamp_ms,
        }
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/core_msgs/convert.rs"]
mod convert_tests;
