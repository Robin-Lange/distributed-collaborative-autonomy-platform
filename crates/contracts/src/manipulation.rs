//! Deterministic, agent-agnostic manipulation preconditions.
//!
//! This module is the manipulation analogue of `agent::decision`'s
//! safety gate: given a snapshot of agent state, object state, and runtime
//! latches, it decides whether a `ManipulationIntent` may become a
//! `ManipulationCommand`. It has no DDS/threading/Bevy dependency, so the
//! same logic can be reused by any scheduler that composes activities into a
//! graph: this is one precondition-gated node expressed as a pure function over
//! a plain data snapshot, with no assumption about what calls it or which agent
//! it runs on.
//!
//! Reach envelope: Spot's arm reaches 984 mm at full extension (Boston
//! Dynamics spec). Deployment-specific bounds come from the `AppConfig`
//! object catalog section. The target's planar bearing must fall within the
//! arm's forward sector relative to the platform heading. The object's own
//! orientation is not an admission constraint; execution uses its yaw to
//! orient the gripper.

use crate::{
    manipulation_verb_drop_off, manipulation_verb_pick_up, messages::PoseXYZYaw,
    object_lifecycle_held, project_planar, veto_reason_object_not_free,
    veto_reason_object_not_held, veto_reason_task_active, veto_reason_trajectory_active,
    veto_reason_unknown_object, veto_reason_unreachable, wrap_to_pi,
};
use glam::Vec3;
use serde::Deserialize;

/// Geometric reach envelope for a deterministic-known-pose grasp approach.
///
/// This is a footprint gate, not an inverse-kinematics solve: it decides
/// whether attempting the arm sequence is worth trying at all, before any
/// SDK/sim call. Tunable via the `AppConfig` object catalog section, the same
/// way `containment.trip_margin_m` is tunable in the environment section.
#[derive(serde::Serialize, Deserialize, Clone, Copy, Debug)]
pub struct ReachEnvelope {
    pub min_m: f32,
    pub max_m: f32,
    pub front_half_angle_deg: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum ReachViolation {
    InvalidGeometry,
    TooClose { distance_m: f32 },
    TooFar { distance_m: f32 },
    OutsideArmSector { heading_offset_deg: f32 },
}

impl Default for ReachEnvelope {
    fn default() -> Self {
        Self {
            min_m: 0.30,
            max_m: 0.80,
            front_half_angle_deg: 90.0,
        }
    }
}

impl ReachEnvelope {
    #[must_use]
    pub const fn min_m(&self) -> f32 {
        self.min_m
    }
    #[must_use]
    pub const fn max_m(&self) -> f32 {
        self.max_m
    }

    /// True when `target` is within reach of an agent standing at
    /// `agent_position` facing `agent_heading_yaw_rad`.
    ///
    /// The target's position is evaluated relative to the platform heading.
    /// The target object's own orientation does not enter this calculation.
    #[must_use]
    pub fn contains(&self, agent_position: Vec3, agent_heading_yaw_rad: f32, target: Vec3) -> bool {
        self.check(agent_position, agent_heading_yaw_rad, target)
            .is_ok()
    }

    fn check(
        &self,
        agent_position: Vec3,
        agent_heading_yaw_rad: f32,
        target: Vec3,
    ) -> Result<(), ReachViolation> {
        if !agent_position.is_finite()
            || !target.is_finite()
            || !agent_heading_yaw_rad.is_finite()
            || !self.min_m.is_finite()
            || !self.max_m.is_finite()
            || !self.front_half_angle_deg.is_finite()
        {
            return Err(ReachViolation::InvalidGeometry);
        }

        let delta = project_planar(target - agent_position);
        let distance = delta.length();

        if distance < self.min_m {
            return Err(ReachViolation::TooClose {
                distance_m: distance,
            });
        }
        if distance > self.max_m {
            return Err(ReachViolation::TooFar {
                distance_m: distance,
            });
        }

        // A target exactly at the reference point has no defined bearing;
        // distance alone (which already failed above unless min_m is zero)
        // decides it.
        if distance < 1e-6 {
            return Ok(());
        }

        let bearing = delta.y.atan2(delta.x);
        let error = wrap_to_pi(bearing - agent_heading_yaw_rad);

        if error.abs() > self.front_half_angle_deg.to_radians() {
            return Err(ReachViolation::OutsideArmSector {
                heading_offset_deg: error.abs().to_degrees(),
            });
        }

        Ok(())
    }
}

/// Snapshot consumed by one manipulation precondition evaluation.
pub struct ManipulationContext<'a> {
    pub agent_id: &'a str,
    pub agent_position_map: [f32; 3],
    pub agent_heading_yaw_rad: f32,
    /// (`manipulation_verb_pick_up()` / `manipulation_verb_drop_off()`).
    pub verb: u8,

    /// Catalog entry for the requested object, if known.
    pub object: Option<&'a crate::catalog::ObjectSpec>,

    /// Latest observed lifecycle for the object, if any `ObjectState` has
    /// been received yet.
    pub object_lifecycle: Option<u8>,

    /// Latest observed holder for the object, if currently held.
    pub object_holder_agent_id: Option<&'a str>,

    /// Latest observed pose for the object, if any `ObjectState` has been
    /// received yet. `PickUp` targets this (falling back to the catalog's
    /// starting pose only if no `ObjectState` has been observed) rather than
    /// always targeting the catalog pose — an object that has already been
    /// picked up and dropped elsewhere must be re-approached at its current
    /// location, not its original spawn point.
    pub object_pose: Option<PoseXYZYaw>,

    /// True while the base has an active trajectory (not stopped).
    pub trajectory_active: bool,

    /// True while a manipulation task for this agent is already in flight.
    pub task_active: bool,
}

/// One failed precondition: the shared veto reason plus a human detail.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ManipulationRejection {
    pub reason: u8,
    pub detail: String,
}

/// Evaluates preconditions and returns the target pose for the approved command.
///
/// # Errors
///
/// Returns a [`ManipulationRejection`] when another task is active, the object
/// is missing or unavailable, or the requested object is outside the reach
/// envelope.
pub fn evaluate_preconditions(
    ctx: &ManipulationContext<'_>,
    reach: &ReachEnvelope,
) -> Result<PoseXYZYaw, ManipulationRejection> {
    if ctx.trajectory_active {
        return Err(ManipulationRejection {
            reason: veto_reason_trajectory_active(),
            detail: "base must be stopped before manipulating".to_owned(),
        });
    }

    if ctx.task_active {
        return Err(ManipulationRejection {
            reason: veto_reason_task_active(),
            detail: "a manipulation task is already in progress".to_owned(),
        });
    }

    let Some(object) = ctx.object else {
        return Err(ManipulationRejection {
            reason: veto_reason_unknown_object(),
            detail: "object_id not in catalog".to_owned(),
        });
    };

    if ctx.verb == manipulation_verb_pick_up() {
        if ctx.object_lifecycle == Some(object_lifecycle_held()) {
            return Err(ManipulationRejection {
                reason: veto_reason_object_not_free(),
                detail: format!("{} is already held", object.object_id()),
            });
        }
    } else if ctx.verb == manipulation_verb_drop_off() {
        let held_by_me = ctx.object_lifecycle == Some(object_lifecycle_held())
            && ctx.object_holder_agent_id == Some(ctx.agent_id);

        if !held_by_me {
            return Err(ManipulationRejection {
                reason: veto_reason_object_not_held(),
                detail: format!("{} is not held by {}", object.object_id(), ctx.agent_id),
            });
        }
    }

    let agent_position = Vec3::from_array(ctx.agent_position_map);

    let target = if ctx.verb == manipulation_verb_drop_off() {
        default_drop_pose(agent_position, ctx.agent_heading_yaw_rad, reach, object)
    } else {
        let Some(target) = ctx
            .object_pose
            .or_else(|| object.virtual_initial_pose_map())
        else {
            // A real, MOCAP-tracked object (no `virtual_initial_pose_map`)
            // that `object_tracker` has not observed yet: there is no known
            // pose to target at all, not just an unreachable one.
            return Err(ManipulationRejection {
                reason: veto_reason_unreachable(),
                detail: format!("{} has no known pose yet", object.object_id()),
            });
        };
        target
    };
    let target_position = Vec3::new(target.x, target.y, target.z);

    let reach_result = if target.yaw_rad.is_finite() {
        reach.check(agent_position, ctx.agent_heading_yaw_rad, target_position)
    } else {
        Err(ReachViolation::InvalidGeometry)
    };
    if let Err(violation) = reach_result {
        let detail = match violation {
            ReachViolation::InvalidGeometry => format!(
                "{} cannot be evaluated for manipulation because the platform pose, target pose, \
                 or reach configuration is not finite.",
                object.object_id()
            ),
            ReachViolation::TooClose { distance_m } => format!(
                "{} is too close for manipulation: {distance_m:.2} m from the platform; \
                 minimum reach is {:.2} m.",
                object.object_id(),
                reach.min_m()
            ),
            ReachViolation::TooFar { distance_m } => format!(
                "{} is too far for manipulation: {distance_m:.2} m from the platform; \
                 maximum reach is {:.2} m.",
                object.object_id(),
                reach.max_m()
            ),
            ReachViolation::OutsideArmSector { heading_offset_deg } => format!(
                "{} is outside the arm's forward sector: {heading_offset_deg:.1}° from the \
                 platform's heading; allowed offset is ±{:.1}°.",
                object.object_id(),
                reach.front_half_angle_deg
            ),
        };
        return Err(ManipulationRejection {
            reason: veto_reason_unreachable(),
            detail,
        });
    }

    Ok(target)
}

/// Default placement pose for a `DropOff` with no operator-supplied override.
fn default_drop_pose(
    agent_position: Vec3,
    heading_yaw_rad: f32,
    reach: &ReachEnvelope,
    object: &crate::catalog::ObjectSpec,
) -> PoseXYZYaw {
    let distance = (reach.min_m + reach.max_m) * 0.5;
    PoseXYZYaw {
        x: agent_position.x + distance * heading_yaw_rad.cos(),
        y: agent_position.y + distance * heading_yaw_rad.sin(),
        z: object.resting_height_m(),
        yaw_rad: heading_yaw_rad,
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/core_msgs/manipulation.rs"]
mod manipulation_tests;
