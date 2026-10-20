//! Typed behavior requests, lifecycle feedback, and approved control actions.

use serde::{Deserialize, Serialize};

use crate::{CapabilityRequirement, PoseXYZYaw};

/// Identity class of a behavior-request producer.
///
/// Authority is assigned by the receiving agent; this value is not a
/// caller-controlled priority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BehaviorSource {
    Autonomy,
    Operator,
}

/// Origin retained on an agent-approved control.
///
/// Safety is agent-generated and therefore cannot be represented as a
/// behavior-request producer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlSource {
    Autonomy,
    Operator,
    Safety,
}

/// Position target whose dimensionality is explicit.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum PositionTarget {
    Planar([f32; 2]),
    Spatial([f32; 3]),
}

impl PositionTarget {
    /// Returns whether every coordinate is finite.
    pub fn is_finite(self) -> bool {
        match self {
            Self::Planar(position) => position.into_iter().all(f32::is_finite),
            Self::Spatial(position) => position.into_iter().all(f32::is_finite),
        }
    }

    /// Capability required to interpret this target without discarding axes.
    #[must_use]
    pub const fn required_navigation(self) -> CapabilityRequirement {
        match self {
            Self::Planar(_) => CapabilityRequirement::PlanarNavigation,
            Self::Spatial(_) => CapabilityRequirement::SpatialNavigation,
        }
    }
}

/// High-level primitive requested from one agent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BehaviorPrimitive {
    NavigateTo {
        target: PositionTarget,
        heading_yaw_rad: Option<f32>,
    },
    /// Preserves the current behavior: move away from a supplied point using
    /// the receiving agent's configured standoff distance.
    MoveAway {
        from: PositionTarget,
    },
    Hold,
    Pick {
        object_id: String,
    },
    Place {
        object_id: String,
    },
}

impl BehaviorPrimitive {
    /// Capability required before guidance may execute this primitive.
    #[must_use]
    pub const fn required_capability(&self) -> CapabilityRequirement {
        match self {
            Self::NavigateTo { target, .. } => target.required_navigation(),
            Self::MoveAway { from } => from.required_navigation(),
            Self::Hold => CapabilityRequirement::Hold,
            Self::Pick { .. } => CapabilityRequirement::Pick,
            Self::Place { .. } => CapabilityRequirement::Place,
        }
    }
}

/// One explicitly targeted behavior request.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BehaviorRequest {
    pub request_id: String,
    pub agent_id: String,
    pub source: BehaviorSource,
    pub behavior: BehaviorPrimitive,
    pub frame_id: String,
    pub stamp_ms: u64,
}

impl crate::AgentScoped for BehaviorRequest {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

impl BehaviorRequest {
    /// Validates fields that do not require platform state.
    ///
    /// # Errors
    ///
    /// Returns the matching [`BehaviorRequestError`] when an identity or frame
    /// is empty, or when a target contains a non-finite value.
    pub fn validate(&self) -> Result<(), BehaviorRequestError> {
        if self.request_id.trim().is_empty() {
            return Err(BehaviorRequestError::MissingRequestId);
        }
        if self.agent_id.trim().is_empty() {
            return Err(BehaviorRequestError::MissingAgentId);
        }
        if self.frame_id.trim().is_empty() {
            return Err(BehaviorRequestError::MissingFrame);
        }

        match &self.behavior {
            BehaviorPrimitive::NavigateTo {
                target,
                heading_yaw_rad,
            } => {
                if !target.is_finite()
                    || heading_yaw_rad.is_some_and(|heading| !heading.is_finite())
                {
                    return Err(BehaviorRequestError::NonFiniteTarget);
                }
            }
            BehaviorPrimitive::MoveAway { from } if !from.is_finite() => {
                return Err(BehaviorRequestError::NonFiniteTarget);
            }
            BehaviorPrimitive::Pick { object_id } | BehaviorPrimitive::Place { object_id }
                if object_id.trim().is_empty() =>
            {
                return Err(BehaviorRequestError::MissingObjectId);
            }
            BehaviorPrimitive::MoveAway { .. }
            | BehaviorPrimitive::Hold
            | BehaviorPrimitive::Pick { .. }
            | BehaviorPrimitive::Place { .. } => {}
        }

        Ok(())
    }
}

/// Invalid request detected before authority or capability checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BehaviorRequestError {
    MissingRequestId,
    MissingAgentId,
    MissingFrame,
    MissingObjectId,
    NonFiniteTarget,
}

/// Observable lifecycle phase of one behavior request.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BehaviorPhase {
    Accepted,
    Active,
    Succeeded,
    Failed,
    Rejected,
    Cancelled,
    Preempted,
}

/// Structured terminal or denial reason for a behavior.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BehaviorStatusReason {
    UnsupportedCapability,
    PlatformUnavailable,
    UnmetPrecondition,
    Unauthorized,
    AutonomyPaused,
    SafetyStop,
    InvalidRequest,
    ExecutionFailed,
    PreemptedByOperator,
}

/// Status correlated to the originating behavior request.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BehaviorStatus {
    pub request_id: String,
    pub agent_id: String,
    pub phase: BehaviorPhase,
    pub reason: Option<BehaviorStatusReason>,
    pub detail: String,
    pub stamp_ms: u64,
}

impl crate::AgentScoped for BehaviorStatus {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Operator command that re-enables autonomy after an operator override.
///
/// Operator behavior requests pause autonomy immediately. It remains paused
/// until this explicit command is received; completion of the operator
/// behavior never resumes autonomy implicitly.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResumeAutonomy {
    pub request_id: String,
    pub agent_id: String,
    pub stamp_ms: u64,
}

impl crate::AgentScoped for ResumeAutonomy {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Platform-agnostic action approved by the local agent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ControlAction {
    Hold {
        target: PositionTarget,
        heading_yaw_rad: Option<f32>,
        frame_id: String,
    },
    /// Immediate safety stop at the supplied anchor.
    Stop {
        target: PositionTarget,
        frame_id: String,
    },
    PlanarVelocity {
        target: PositionTarget,
        forward_m_s: f32,
        left_m_s: f32,
        yaw_rate_rad_s: f32,
        frame_id: String,
    },
    SpatialVelocity {
        velocity_m_s: [f32; 3],
        yaw_rate_rad_s: f32,
    },
    Pick {
        object_id: String,
        target_pose: PoseXYZYaw,
        frame_id: String,
    },
    Place {
        object_id: String,
        target_pose: PoseXYZYaw,
        frame_id: String,
    },
}

/// Generic control request crossing from the agent to a platform adapter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ControlRequest {
    pub request_id: String,
    pub agent_id: String,
    pub source: ControlSource,
    pub approved_by: String,
    pub action: ControlAction,
    pub safety_episode_id: Option<String>,
    pub stamp_ms: u64,
}

impl crate::AgentScoped for ControlRequest {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

/// Platform execution phase for an approved generic control.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlPhase {
    Active,
    Succeeded,
    Failed,
}

/// Execution feedback from a platform adapter to its local agent.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ControlStatus {
    pub request_id: String,
    pub agent_id: String,
    pub phase: ControlPhase,
    pub detail: String,
    pub stamp_ms: u64,
}

impl crate::AgentScoped for ControlStatus {
    fn agent_id(&self) -> &str {
        &self.agent_id
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/core_msgs/behavior.rs"]
mod behavior_tests;
