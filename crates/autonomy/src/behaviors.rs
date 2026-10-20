//! Platform- and planner-agnostic behavior→goal derivation (`GoTo` / `MoveAway`).
//!
//! Takes plain map-frame primitives (behavior code, target point, current
//! position, standoff distance) rather than the operator-specific
//! `WaypointIntent` wire type or the agent's `DecisionConfig`, so any caller —
//! the current operator-driven path or a future planner bridge — can derive a
//! navigation goal from the same inputs.

use core_msgs::{Behavior, behavior_from_code, project_planar};
use glam::Vec3;

/// Minimum squared-distance threshold before a flee direction is considered valid.
const FLEE_DIRECTION_EPSILON: f32 = 1e-6;

/// Fallback flee direction when the agent and threat positions coincide.
const FALLBACK_FLEE_DIR: Vec3 = Vec3::X;

/// Navigation goal derived from an intent.
pub enum Goal {
    /// Track this target position.
    Track(Vec3),

    /// The requested behavior is already complete or unsupported.
    Done,
}

/// Converts a high-level behavior code into a concrete navigation goal.
///
/// `point` is the behavior's map-frame target, `position` the agent's current
/// map-frame position, and `flee_distance` the `MoveAway` standoff (meters).
#[must_use]
pub fn goal_for_intent(behavior_code: u8, point: Vec3, position: Vec3, flee_distance: f32) -> Goal {
    match behavior_from_code(behavior_code) {
        Some(Behavior::GoTo) => Goal::Track(goto_target(point, position)),
        Some(Behavior::MoveAway) => flee_goal(point, position, flee_distance),
        _ => Goal::Done,
    }
}

/// Builds a planar `GoTo` target while preserving current height (map ENU Z=up).
const fn goto_target(point: Vec3, position: Vec3) -> Vec3 {
    Vec3::new(point.x, point.y, position.z)
}

/// Builds a `MoveAway` target or marks the behavior done when already far enough.
fn flee_goal(threat: Vec3, position: Vec3, flee_distance: f32) -> Goal {
    if flee_distance <= 0.0 {
        return Goal::Done;
    }

    let delta = project_planar(position - threat);

    let distance_squared = delta.length_squared();
    let flee_distance_squared = flee_distance * flee_distance;

    if distance_squared >= flee_distance_squared {
        return Goal::Done;
    }

    // If the agent is exactly on top of the threat point, choose a deterministic
    // forward direction instead of dividing by a near-zero vector.
    let dir = if distance_squared < FLEE_DIRECTION_EPSILON * FLEE_DIRECTION_EPSILON {
        FALLBACK_FLEE_DIR
    } else {
        delta / distance_squared.sqrt()
    };

    let target = threat + dir * flee_distance;

    Goal::Track(Vec3::new(target.x, target.y, position.z))
}
