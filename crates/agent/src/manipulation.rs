//! Manipulation decision logic for the onboard agent.
//!
//! This is the manipulation analogue of `decision.rs`: given a snapshot of
//! agent state, the object catalog, latest observed object lifecycle, and
//! the runtime's mutual-exclusion latches (trajectory active, task active),
//! it decides whether a `ManipulationIntent` becomes an approved
//! `ManipulationCommand` or a `CommandVeto`. A manipulation request is
//! one-shot (unlike a `WaypointIntent`, which is tracked toward a goal every
//! tick): each intent is evaluated exactly once and either starts a task or
//! is rejected, so there is no analogue of `decision.rs`'s continuous
//! trajectory re-evaluation here.
//!
//! Active hard-stop constraints are checked by the caller (`runtime.rs`)
//! *before* this module is ever invoked, exactly like `decision.rs`'s own
//! safety gate runs ahead of intent-specific logic — this keeps one shared
//! hard-stop check instead of two.

use core_msgs::{
    APPROVED_BY_ONBOARD_AGENT, AgentStateMsg, CommandVeto, ManipulationCommand, ManipulationIntent,
    ObjectState,
    catalog::ObjectCatalog,
    frames::MAP_FRAME,
    manipulation::{ManipulationContext, evaluate_preconditions},
    manipulation_grasp_profile_top_down, veto_reason_no_state,
};
use std::collections::BTreeMap;

/// Manipulation task latched by the runtime while a `ManipulationCommand`'s
/// execution is in flight (mirrors `decision::HoldAnchor`'s role for
/// navigation: state carried forward across ticks until a terminal status
/// clears it).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ActiveManipulationTask {
    pub(super) command_id: String,
    pub(super) object_id: String,
    pub(super) verb: u8,
    pub(super) started_ms: u64,
}

/// Result of evaluating one manipulation intent.
pub struct ManipulationOutcome {
    /// Approved command to publish, if the intent passed every precondition.
    pub(super) command: Option<ManipulationCommand>,

    /// Rejection to publish, if any precondition failed.
    pub(super) veto: Option<CommandVeto>,

    /// Task latch to install when a command was approved.
    pub(super) started_task: Option<ActiveManipulationTask>,
}

impl ManipulationOutcome {
    /// Builds an outcome directly, for callers that bypass the precondition
    /// ladder entirely (the runtime's active-hard-stop short-circuit).
    pub(super) const fn new(
        command: Option<ManipulationCommand>,
        veto: Option<CommandVeto>,
        started_task: Option<ActiveManipulationTask>,
    ) -> Self {
        Self {
            command,
            veto,
            started_task,
        }
    }
}

/// Inputs for one manipulation decision tick.
#[derive(Copy, Clone)]
pub struct ManipulationInputs<'a> {
    pub intent: Option<&'a ManipulationIntent>,
    pub request_id: &'a str,
    pub agent_state: Option<&'a AgentStateMsg>,
    pub object_states: &'a BTreeMap<String, ObjectState>,
    pub trajectory_active: bool,
    pub task_active: bool,
}

/// Evaluates the newest manipulation intent, if any, against the current
/// snapshot. Returns `None` when there is nothing to evaluate this tick.
pub fn next_manipulation_decision(
    agent_id: &str,
    catalog: &ObjectCatalog,
    now_ms: u64,
    inputs: ManipulationInputs<'_>,
) -> Option<ManipulationOutcome> {
    let intent = inputs.intent?;

    let Some(state) = inputs.agent_state else {
        return Some(reject(
            agent_id,
            intent,
            inputs.request_id,
            veto_reason_no_state(),
            "No platform state available".to_owned(),
            now_ms,
        ));
    };

    let object = catalog.get(&intent.object_id);
    let object_state = inputs.object_states.get(&intent.object_id);

    let context = ManipulationContext {
        agent_id,
        agent_position_map: state.position,
        agent_heading_yaw_rad: state.heading_yaw_rad,
        verb: intent.verb,
        object,
        object_lifecycle: object_state.map(|object_state| object_state.lifecycle),
        // `.pose` defaults to (0,0,0,0) once `object_tracker` has published
        // at all, even before/after it has ever actually seen the object
        // (`pose_valid=false`) — without this filter that stale/default
        // pose would shadow `evaluate_preconditions`'s catalog fallback and
        // silently target the origin instead of vetoing "no known pose yet".
        object_pose: object_state
            .filter(|object_state| object_state.pose_valid)
            .map(|object_state| object_state.pose),
        object_holder_agent_id: object_state
            .map(|object_state| object_state.holder_agent_id.as_str()),
        trajectory_active: inputs.trajectory_active,
        task_active: inputs.task_active,
    };

    match evaluate_preconditions(&context, &catalog.reach_envelope()) {
        Ok(target_pose) => {
            let command = ManipulationCommand {
                agent_id: agent_id.to_owned(),
                command_id: inputs.request_id.to_owned(),
                object_id: intent.object_id.clone(),
                verb: intent.verb,
                approved_by: APPROVED_BY_ONBOARD_AGENT.to_owned(),
                target_pose,
                stamp_ms: now_ms,
                frame_id: MAP_FRAME.to_owned(),
                // TOP_DOWN is the only grasp profile the executor implements.
                grasp_profile: manipulation_grasp_profile_top_down(),
            };

            Some(ManipulationOutcome {
                command: Some(command),
                veto: None,
                started_task: Some(ActiveManipulationTask {
                    command_id: inputs.request_id.to_owned(),
                    object_id: intent.object_id.clone(),
                    verb: intent.verb,
                    started_ms: now_ms,
                }),
            })
        }
        Err(rejection) => Some(reject(
            agent_id,
            intent,
            inputs.request_id,
            rejection.reason,
            rejection.detail,
            now_ms,
        )),
    }
}

fn reject(
    agent_id: &str,
    intent: &ManipulationIntent,
    request_id: &str,
    reason: u8,
    detail: String,
    stamp_ms: u64,
) -> ManipulationOutcome {
    ManipulationOutcome {
        command: None,
        veto: Some(CommandVeto {
            agent_id: agent_id.to_owned(),
            command_id: request_id.to_owned(),
            source: intent.source,
            reason,
            detail,
            stamp_ms,
        }),
        started_task: None,
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/agent/manipulation.rs"]
mod manipulation_tests;
