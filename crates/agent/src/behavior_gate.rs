//! Pure admission policy for high-level behavior requests.

use core_msgs::{
    BehaviorPrimitive, BehaviorRequest, BehaviorSource, BehaviorStatusReason, CapabilitySet,
    NavigationKind, frames::MAP_FRAME,
};

/// Rejection returned before a behavior reaches execution logic.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rejection {
    pub(super) reason: BehaviorStatusReason,
    pub(super) detail: String,
}

/// Mutable operator/autonomy authority state for one agent.
#[derive(Clone, Debug)]
pub struct BehaviorGate {
    autonomy_enabled: bool,
    autonomy_paused: bool,
}

impl BehaviorGate {
    pub(super) const fn new(autonomy_enabled: bool) -> Self {
        Self {
            autonomy_enabled,
            autonomy_paused: false,
        }
    }

    /// Validates and authorizes one request.
    ///
    /// A structurally valid operator request pauses autonomy before capability
    /// or availability checks. This makes the operator an actual override:
    /// autonomy cannot continue merely because the requested platform action
    /// is unavailable.
    pub(super) fn admit(
        &mut self,
        request: &BehaviorRequest,
        agent_id: &str,
        capabilities: &CapabilitySet,
        platform_alive: Option<bool>,
    ) -> Result<(), Rejection> {
        request.validate().map_err(|error| Rejection {
            reason: BehaviorStatusReason::InvalidRequest,
            detail: format!("invalid behavior request: {error:?}"),
        })?;

        if request.agent_id != agent_id {
            return Err(Rejection {
                reason: BehaviorStatusReason::InvalidRequest,
                detail: format!(
                    "request targets agent '{}', but this agent is '{agent_id}'",
                    request.agent_id
                ),
            });
        }

        if request.frame_id != MAP_FRAME {
            return Err(Rejection {
                reason: BehaviorStatusReason::InvalidRequest,
                detail: format!(
                    "request frame '{}' is unsupported; expected '{MAP_FRAME}'",
                    request.frame_id
                ),
            });
        }

        match request.source {
            BehaviorSource::Operator => self.autonomy_paused = true,
            BehaviorSource::Autonomy if !self.autonomy_enabled => {
                return Err(Rejection {
                    reason: BehaviorStatusReason::Unauthorized,
                    detail: "autonomy is disabled for this agent".to_owned(),
                });
            }
            BehaviorSource::Autonomy if self.autonomy_paused => {
                return Err(Rejection {
                    reason: BehaviorStatusReason::AutonomyPaused,
                    detail: "operator override is active; explicit resume is required".to_owned(),
                });
            }
            BehaviorSource::Autonomy => {}
        }

        let requirement = request.behavior.required_capability();
        if !capabilities.supports(requirement) {
            return Err(Rejection {
                reason: BehaviorStatusReason::UnsupportedCapability,
                detail: format!("platform does not support {requirement:?}"),
            });
        }

        if let BehaviorPrimitive::NavigateTo {
            target,
            heading_yaw_rad: Some(_),
        } = request.behavior
        {
            let kind = match target {
                core_msgs::PositionTarget::Planar(_) => NavigationKind::Planar,
                core_msgs::PositionTarget::Spatial(_) => NavigationKind::Spatial,
            };
            let supports_heading = capabilities
                .navigation
                .iter()
                .find(|capability| capability.kind == kind)
                .is_some_and(|capability| capability.supports_target_heading);
            if !supports_heading {
                return Err(Rejection {
                    reason: BehaviorStatusReason::UnsupportedCapability,
                    detail: "platform navigation does not support a target heading".to_owned(),
                });
            }
        }

        if platform_alive != Some(true) {
            return Err(Rejection {
                reason: BehaviorStatusReason::PlatformUnavailable,
                detail: match platform_alive {
                    Some(false) => "platform adapter is disconnected".to_owned(),
                    None => "platform adapter availability is not known yet".to_owned(),
                    Some(true) => unreachable!(),
                },
            });
        }

        Ok(())
    }

    /// Releases an operator override. Autonomy still remains unavailable when
    /// it is disabled in the deployment manifest.
    pub(super) const fn resume_autonomy(&mut self) {
        self.autonomy_paused = false;
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/agent/behavior_gate.rs"]
mod behavior_gate_tests;
