//! Pure decision logic for the onboard agent.
//!
//! This module is intentionally free of DDS/threading concerns. Given a snapshot
//! of config, authority, time, intent, state, constraint, status, and the
//! runtime's hold latch, it returns a
//! `DecisionOutcome`: a platform command (velocity setpoint or hold), veto,
//! self-reported completion status, and latch-clear instructions (plus the
//! hold anchor to carry forward via [`DecisionOutcome::next_hold_anchor`]).
//!
//! Overseer mental model: this is the safety gate between operator intent and
//! platform motion. It validates authority, frame, state freshness, and active
//! constraints before allowing any requested movement to become a platform
//! command.

use super::{authority, config::AgentConfig};
use autonomy::{behaviors, guidance};

use core_msgs::{
    APPROVED_BY_ONBOARD_AGENT, AgentStateMsg, AuthorityState, CommandStatus, CommandVeto,
    PlatformCommand, VelocitySetpoint, WaypointIntent, command_source_autonomy,
    command_source_none, command_status_succeeded, command_type_hold, command_type_stop,
    command_type_velocity, constraint_type_robot_fault, constraints::ActiveConstraints,
    frames::MAP_FRAME, veto_reason_bad_frame, veto_reason_collision_stop, veto_reason_no_state,
    veto_reason_none, veto_reason_platform_dead, veto_reason_robot_fault, veto_reason_stale_state,
    veto_reason_superseded, veto_reason_unauthorized_source, waypoint_command_id,
};

use glam::Vec3;

/// Static decision parameters derived from runtime config and manifest policy.
#[derive(Clone, Debug)]
pub struct DecisionConfig {
    pub agent_id: String,

    /// `MoveAway` standoff distance, in meters (the field name carries no unit suffix).
    pub flee_distance: f32,

    pub state_timeout_ms: u64,

    pub guidance_limits: guidance::PlanarGuidanceLimits,

    /// Mirrors `AgentManifest::autonomy_enabled`: when true, `commands_allowed`
    /// admits both the human operator and the autonomy planner instead of only
    /// the manifest's single `command_authority` source.
    pub autonomy_enabled: bool,
}

impl DecisionConfig {
    /// Builds decision config from runtime config and manifest policy.
    pub(super) fn from_agent_config(
        config: &AgentConfig,
        state_timeout_ms: u64,
        autonomy_enabled: bool,
        max_speed: core_msgs::SpeedPreset,
        capabilities: &core_msgs::CapabilitySet,
    ) -> Self {
        let navigation = capabilities
            .navigation
            .iter()
            .find(|capability| capability.kind == core_msgs::NavigationKind::Planar)
            .unwrap_or_else(|| {
                panic!(
                    "agent '{}' platform profile does not support planar navigation",
                    config.agent_id
                )
            });
        let speed_limits = navigation.speed_presets.selected(max_speed);

        Self {
            agent_id: config.agent_id.as_str().to_owned(),
            flee_distance: config.flee_distance,
            state_timeout_ms,
            guidance_limits: guidance::PlanarGuidanceLimits {
                max_linear_speed_m_s: speed_limits.max_linear_speed_m_s,
                max_yaw_rate_rad_s: speed_limits.max_yaw_rate_rad_s,
                position_tolerance_m: navigation.position_tolerance_m,
            },
            autonomy_enabled,
        }
    }
}

/// Inputs for one decision tick.
#[derive(Clone)]
pub struct DecisionInputs<'a> {
    pub intent: Option<&'a WaypointIntent>,
    pub state: Option<&'a AgentStateMsg>,
    pub constraints: &'a ActiveConstraints,
    pub status: Option<&'a CommandStatus>,
    pub hold_anchor: Option<HoldAnchor>,

    /// Latest platform liveliness known to the runtime: `Some(false)` means the
    /// bridge process was confirmed dead via Zenoh liveliness (instant hold),
    /// `Some(true)` alive, `None` no liveliness info yet (never holds).
    pub platform_alive: Option<bool>,
}

/// Inputs for evaluating one active intent.
struct IntentInputs<'a> {
    intent: &'a WaypointIntent,
    state: Option<&'a AgentStateMsg>,
    status: Option<&'a CommandStatus>,
    hold_position: Option<[f32; 3]>,
    hold_episode_id: Option<String>,
}

/// Result of one decision tick.
#[derive(Clone, Debug)]
pub struct DecisionOutcome {
    /// Optional command to send to the platform.
    command: Option<PlatformCommand>,

    /// Optional veto to publish for operator feedback.
    veto: Option<CommandVeto>,

    /// Authority snapshot to publish every tick.
    authority: AuthorityState,

    /// Whether the runtime should clear the active intent latch.
    ///
    /// Constraints are deliberately not clearable from a decision outcome:
    /// each producer owns its conditions' lifecycle (explicit clear events or
    /// TTL expiry in the runtime's `ActiveConstraints`).
    clear_intent: bool,

    /// Self-reported command completion, published to the bus for observers.
    ///
    /// Real-mode velocity tracking has no platform-side goal-status feedback
    /// (`SE2VelocityCommand` carries none), so the onboard agent itself is
    /// the source of truth for "this waypoint succeeded". `None` on every
    /// outcome that isn't a velocity-tracked goal completion.
    status: Option<CommandStatus>,
}

impl DecisionOutcome {
    /// Returns whether the runtime should clear the active intent.
    pub(super) const fn clears_intent(&self) -> bool {
        self.clear_intent
    }

    /// Consumes the outcome into the messages adapters need to publish.
    pub(super) fn into_outputs(
        self,
    ) -> (
        Option<PlatformCommand>,
        AuthorityState,
        Option<CommandVeto>,
        Option<CommandStatus>,
    ) {
        (self.command, self.authority, self.veto, self.status)
    }

    /// Returns the hold anchor the runtime must carry into the next tick.
    ///
    /// A hold command latches its own target (the exact position the episode
    /// started from, e.g. the state at the moment of a collision) and the
    /// currently active safety episode, if any. A velocity command ends the
    /// hold and releases the anchor. A tick that emits no platform command
    /// leaves the anchor untouched, so a hold survives input gaps without
    /// re-anchoring on a drifted position.
    pub(super) fn next_hold_anchor(&self, current: Option<HoldAnchor>) -> Option<HoldAnchor> {
        match &self.command {
            Some(command)
                if command.command_type == command_type_hold()
                    || command.command_type == command_type_stop() =>
            {
                Some(HoldAnchor {
                    position: command.target_position,
                    episode_id: command.episode_id.clone(),
                })
            }
            Some(_) => None,
            None => current,
        }
    }
}

/// Safety-hold reason that should produce a hold command and veto.
#[derive(Clone, Debug)]
struct SafetyHold {
    /// Shared veto reason code.
    reason: u8,

    /// Human-readable veto detail.
    detail: String,

    /// Episode id of the triggering constraint, for `collision_stop`/
    /// `robot_fault` reasons; empty for reasons with no single triggering
    /// constraint (stale state, bad frame, unauthorized source).
    episode_id: String,
}

/// Hold target and active safety episode latched for one hold.
///
/// A hold command latches its own target (see [`DecisionOutcome::next_hold_anchor`]);
/// the safety episode remains attached only while its constraint is active.
/// Once the constraint clears, one ordinary hold clears the executor's
/// level-triggered safety state while preserving the anchor position.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HoldAnchor {
    pub(super) position: [f32; 3],
    pub(super) episode_id: String,
}

/// Intent plus deterministic command ID derived from its timestamp.
struct ActiveIntent<'a> {
    /// Original intent message.
    message: &'a WaypointIntent,

    /// Command ID used for commands, statuses, and vetoes related to this intent.
    command_id: String,
}

impl<'a> ActiveIntent<'a> {
    /// Creates an active intent and its deterministic command ID.
    fn new(agent_id: &str, message: &'a WaypointIntent) -> Self {
        Self {
            message,
            command_id: command_id_for_intent(agent_id, message),
        }
    }

    /// Returns the source code from the original intent.
    const fn source(&self) -> u8 {
        self.message.source
    }
}

/// Computes the next command/hold/veto decision from a runtime snapshot.
pub fn next_decision(
    config: &DecisionConfig,
    runtime_authority: &AuthorityState,
    now_ms: u64,
    inputs: &DecisionInputs<'_>,
) -> DecisionOutcome {
    let authority = runtime_authority.clone();

    // Decision order is part of the safety contract: choose the hold anchor,
    // handle safety holds before operator behavior, and only plan movement once
    // the current state and constraints are trusted.
    // A hold target must never chase the platform's drifting position: the
    // first hold of an episode locks the target (for a collision stop, the
    // exact state at the moment of impact) and every later hold reuses that
    // anchor. Re-deriving it from live state each tick would ratchet the hold
    // point along with post-impact drift — pulling the platform through the
    // very obstacle that triggered the stop.
    let hold_position = inputs
        .hold_anchor
        .as_ref()
        .map(|anchor| anchor.position)
        .or_else(|| hold_position_from_state(inputs.state));
    // A non-empty episode id marks an active safety hold. An ordinary hold
    // keeps the same position with an empty id so a later safety condition can
    // establish its own episode without re-anchoring.
    let hold_episode_id = inputs
        .hold_anchor
        .as_ref()
        .map(|anchor| anchor.episode_id.clone());

    // Safety holds outrank all intents. When a hard stop or stale state exists,
    // the agent publishes a hold command before considering requested behavior.
    if let Some(safety_hold) = safety_hold_for_available_state(
        config,
        runtime_authority,
        inputs.state,
        inputs.constraints,
        inputs.platform_alive,
        now_ms,
    ) {
        if let Some(intent_msg) = inputs.intent {
            return hold_with_veto(
                config,
                &ActiveIntent::new(config.agent_id.as_str(), intent_msg),
                safety_hold,
                authority,
                hold_position,
                hold_episode_id,
                now_ms,
            );
        }

        // No active intent means no command_id to attach a veto to (vetoes are
        // intent-scoped), so an idle hold is silent regardless of which safety
        // reason triggered it (bad frame, stale state, or collision): the
        // platform still holds/refuses to move, it just has nothing to veto.
        let episode_id = hold_episode_id
            .filter(|episode_id| !episode_id.is_empty())
            .unwrap_or_else(|| safety_hold.episode_id.clone());
        let already_stopped_for_episode = !episode_id.is_empty()
            && inputs
                .hold_anchor
                .as_ref()
                .is_some_and(|anchor| anchor.episode_id == episode_id);
        let command = if already_stopped_for_episode {
            None
        } else {
            hold_position.map(|position| {
                hold_command(
                    config.agent_id.as_str(),
                    waypoint_command_id(config.agent_id.as_str(), now_ms),
                    command_type_hold(),
                    command_source_none(),
                    position,
                    episode_id,
                    now_ms,
                )
            })
        };
        return idle_hold_outcome(authority, command);
    }

    if let Some(intent_msg) = inputs.intent {
        decision_for_intent(
            config,
            runtime_authority,
            authority,
            now_ms,
            IntentInputs {
                intent: intent_msg,
                state: inputs.state,
                status: inputs.status,
                hold_position,
                hold_episode_id,
            },
        )
    } else {
        let needs_initial_hold = inputs.hold_anchor.is_none();
        let needs_safety_clear = inputs
            .hold_anchor
            .as_ref()
            .is_some_and(|anchor| !anchor.episode_id.is_empty());
        let command = if needs_initial_hold || needs_safety_clear {
            hold_position.map(|position| {
                hold_command(
                    config.agent_id.as_str(),
                    waypoint_command_id(config.agent_id.as_str(), now_ms),
                    command_type_hold(),
                    command_source_none(),
                    position,
                    String::new(),
                    now_ms,
                )
            })
        } else {
            None
        };
        idle_hold_outcome(authority, command)
    }
}

/// Returns the current platform position, or `None` if no trustworthy state
/// is available.
///
/// The agent must never fabricate or trust an unlabeled position: when state
/// is missing or its `frame_id` is not `"map"`, the caller omits the platform
/// command entirely (authority/veto still publish) rather than commanding a
/// hold at an invented or unvalidated coordinate.
fn hold_position_from_state(state: Option<&AgentStateMsg>) -> Option<[f32; 3]> {
    state
        .filter(|state| state.frame_id == MAP_FRAME)
        .map(|state| state.position)
}

/// Evaluates one active intent after global safety checks have passed.
///
/// `hold_position` is the episode-anchored hold target computed by
/// `next_decision`; every hold emitted here must use it rather than
/// re-reading the live state.
fn decision_for_intent(
    config: &DecisionConfig,
    runtime_authority: &AuthorityState,
    authority: AuthorityState,
    now_ms: u64,
    inputs: IntentInputs<'_>,
) -> DecisionOutcome {
    let active_intent = ActiveIntent::new(config.agent_id.as_str(), inputs.intent);

    // Command-source authorization is checked before state availability so an
    // unauthorized request is reported as policy failure, not a state failure.
    if !authority::commands_allowed(
        runtime_authority,
        active_intent.source(),
        config.autonomy_enabled,
    ) {
        let hold = SafetyHold {
            reason: veto_reason_unauthorized_source(),
            detail: "Source not authorized".to_owned(),
            episode_id: String::new(),
        };

        return hold_with_veto(
            config,
            &active_intent,
            hold,
            authority,
            inputs.hold_position,
            inputs.hold_episode_id,
            now_ms,
        );
    }

    // The agent must not act on a spatial quantity outside the canonical `map`
    // frame: a missing or non-"map" frame_id is treated as ambiguous input.
    if inputs.intent.frame_id != MAP_FRAME {
        let hold = SafetyHold {
            reason: veto_reason_bad_frame(),
            detail: "Intent frame_id is not \"map\"".to_owned(),
            episode_id: String::new(),
        };

        return hold_with_veto(
            config,
            &active_intent,
            hold,
            authority,
            inputs.hold_position,
            inputs.hold_episode_id,
            now_ms,
        );
    }

    if intent_completed(inputs.status, &active_intent.command_id) {
        return ended_outcome(
            config,
            &active_intent,
            authority,
            inputs.hold_position,
            inputs.hold_episode_id,
            now_ms,
        );
    }

    let Some(state) = inputs.state else {
        let hold = SafetyHold {
            reason: veto_reason_no_state(),
            detail: "No platform state available".to_owned(),
            episode_id: String::new(),
        };

        return hold_with_veto(
            config,
            &active_intent,
            hold,
            authority,
            inputs.hold_position,
            inputs.hold_episode_id,
            now_ms,
        );
    };

    let position = Vec3::from_array(state.position);

    match behaviors::goal_for_intent(
        inputs.intent.behavior,
        Vec3::from_array(inputs.intent.point),
        position,
        config.flee_distance,
    ) {
        // Every mode tracks via the onboard PD controller: sim executes the
        // resulting velocity setpoint in avian physics, real via the sidecar's
        // SE2VelocityCommand. Neither executor does server-side goal tracking,
        // so completion is decided here (see
        // velocity_decision).
        behaviors::Goal::Track(target) => velocity_decision(
            config,
            &active_intent,
            authority,
            state,
            target,
            &inputs,
            now_ms,
        ),
        behaviors::Goal::Done => ended_outcome(
            config,
            &active_intent,
            authority,
            inputs.hold_position,
            inputs.hold_episode_id,
            now_ms,
        ),
    }
}

/// Evaluates one velocity-tracking tick for any mode.
///
/// No executor — the real platform's sidecar or the sim runtime — does
/// server-side goal tracking or goal-status feedback for
/// `COMMAND_TYPE_VELOCITY` commands, so completion is decided here from
/// live state, every tick.
///
/// The hold anchor in `inputs` is the same episode-anchored value that
/// `decision_for_intent`'s other branches use, so a hold emitted here (bad
/// state) is indistinguishable from any other safety hold.
fn velocity_decision(
    config: &DecisionConfig,
    active_intent: &ActiveIntent<'_>,
    authority: AuthorityState,
    state: &AgentStateMsg,
    target: Vec3,
    inputs: &IntentInputs<'_>,
    now_ms: u64,
) -> DecisionOutcome {
    let position = Vec3::from_array(state.position);
    // Waypoint velocity — WaypointIntent is position-only, so the desired
    // velocity at the goal is zero.
    let velocity = Vec3::new(state.velocity[0], state.velocity[1], state.yaw_rate_rad_s);

    let Some(track_outcome) = guidance::track(
        position,
        state.heading_yaw_rad,
        velocity,
        target,
        Vec3::ZERO,
        config.guidance_limits,
    ) else {
        // Deviation from a hard failure: a non-finite state or target sample
        // (a bad peer message) must degrade to "hold, don't command" rather
        // than propagate NaN onto the platform. Same shape as the existing
        // no-state safety hold.
        let hold = SafetyHold {
            reason: veto_reason_no_state(),
            detail: "Non-finite state or target".to_owned(),
            episode_id: String::new(),
        };

        return hold_with_veto(
            config,
            active_intent,
            hold,
            authority,
            inputs.hold_position,
            inputs.hold_episode_id.clone(),
            now_ms,
        );
    };

    match track_outcome {
        guidance::TrackOutcome::Reached => {
            let mut outcome = ended_outcome(
                config,
                active_intent,
                authority,
                inputs.hold_position,
                inputs.hold_episode_id.clone(),
                now_ms,
            );
            outcome.status = Some(CommandStatus {
                agent_id: config.agent_id.clone(),
                command_id: active_intent.command_id.clone(),
                status: command_status_succeeded(),
                stamp_ms: now_ms,
            });
            outcome
        }
        guidance::TrackOutcome::Command(setpoint) => command_outcome(
            authority,
            velocity_command(
                config.agent_id.as_str(),
                active_intent.command_id.clone(),
                target.to_array(),
                setpoint,
                active_intent.source(),
                now_ms,
            ),
        ),
    }
}

/// Returns true when latest status reports this command succeeded.
fn intent_completed(status: Option<&CommandStatus>, command_id: &str) -> bool {
    status.is_some_and(|s| s.command_id == command_id && s.status == command_status_succeeded())
}

/// Checks frame validity, stale state, and trusted hard-stop constraints.
fn safety_hold_reason(
    config: &DecisionConfig,
    authority: &AuthorityState,
    state: &AgentStateMsg,
    constraints: &ActiveConstraints,
    platform_alive: Option<bool>,
    now_ms: u64,
) -> Option<SafetyHold> {
    // The agent must not plan from a position outside the canonical `map`
    // frame: a missing or non-"map" frame_id is treated as ambiguous input.
    if state.frame_id != MAP_FRAME {
        return Some(SafetyHold {
            reason: veto_reason_bad_frame(),
            detail: "State frame_id is not \"map\"".to_owned(),
            episode_id: String::new(),
        });
    }

    // Liveliness reports the bridge process itself gone (session/process death),
    // so hold instantly instead of waiting out the stale-state window below.
    // `None` (no liveliness info yet) must not hold — same conservative default
    // as the rest of this gate.
    if platform_alive == Some(false) {
        return Some(SafetyHold {
            reason: veto_reason_platform_dead(),
            detail: "Platform bridge process not running".to_owned(),
            episode_id: String::new(),
        });
    }

    if now_ms.saturating_sub(state.stamp_ms) > config.state_timeout_ms {
        return Some(SafetyHold {
            reason: veto_reason_stale_state(),
            detail: "State too old".to_owned(),
            episode_id: String::new(),
        });
    }

    // Constraint severity is acted on only for sources the current collision
    // authority trusts. Robot faults outrank wall stops for reporting.
    let hard_stop = constraints
        .active_hard_stop(|source| authority::trusts_constraint_source(authority, source))?;

    let is_fault = hard_stop.constraint_type == constraint_type_robot_fault();
    let base = if is_fault {
        "Robot fault: hard stop active"
    } else {
        "Collision: hard stop active"
    };

    // With several conditions active, the veto explains the whole set instead
    // of only the reported one.
    let detail = if constraints.len() > 1 {
        format!("{base}; active: {}", constraints.summary())
    } else {
        base.to_owned()
    };

    Some(SafetyHold {
        reason: if is_fault {
            veto_reason_robot_fault()
        } else {
            veto_reason_collision_stop()
        },
        detail,
        episode_id: hard_stop.episode_id.clone(),
    })
}

/// Runs safety checks only when state is available.
fn safety_hold_for_available_state(
    config: &DecisionConfig,
    authority: &AuthorityState,
    state: Option<&AgentStateMsg>,
    constraints: &ActiveConstraints,
    platform_alive: Option<bool>,
    now_ms: u64,
) -> Option<SafetyHold> {
    safety_hold_reason(
        config,
        authority,
        state?,
        constraints,
        platform_alive,
        now_ms,
    )
}

/// Builds the hold command used when an intent has completed.
fn ended_outcome(
    config: &DecisionConfig,
    intent: &ActiveIntent<'_>,
    authority: AuthorityState,
    hold_position: Option<[f32; 3]>,
    hold_episode_id: Option<String>,
    stamp_ms: u64,
) -> DecisionOutcome {
    DecisionOutcome {
        command: hold_position.map(|position| {
            hold_command(
                config.agent_id.as_str(),
                intent.command_id.clone(),
                command_type_hold(),
                intent.source(),
                position,
                hold_episode_id.unwrap_or_default(),
                stamp_ms,
            )
        }),
        veto: None,
        authority,
        clear_intent: true,
        status: None,
    }
}

/// Builds a hold command and veto for a safety or authority failure.
fn hold_with_veto(
    config: &DecisionConfig,
    intent: &ActiveIntent<'_>,
    safety_hold: SafetyHold,
    authority: AuthorityState,
    hold_position: Option<[f32; 3]>,
    hold_episode_id: Option<String>,
    stamp_ms: u64,
) -> DecisionOutcome {
    // Hard stops (collision or robot fault) consume the intent so recovery
    // needs a fresh operator command — a latched intent must not auto-resume
    // motion the moment the producer clears its constraint. Other vetoes keep
    // the intent latched so they can resolve when missing/stale inputs recover.
    let hard_stop = safety_hold.reason == veto_reason_collision_stop()
        || safety_hold.reason == veto_reason_robot_fault();
    // Reuse only an active safety episode. An empty id belongs to an ordinary
    // hold and must not mask a newly triggered safety condition.
    let episode_id = hold_episode_id
        .filter(|episode_id| !episode_id.is_empty())
        .unwrap_or_else(|| safety_hold.episode_id.clone());

    let command_type = if safety_hold.reason == veto_reason_collision_stop() {
        command_type_stop()
    } else {
        command_type_hold()
    };

    DecisionOutcome {
        command: hold_position.map(|position| {
            hold_command(
                config.agent_id.as_str(),
                intent.command_id.clone(),
                command_type,
                intent.source(),
                position,
                episode_id,
                stamp_ms,
            )
        }),
        veto: Some(command_veto(
            config.agent_id.as_str(),
            intent.command_id.clone(),
            intent.source(),
            safety_hold.reason,
            safety_hold.detail,
            stamp_ms,
        )),
        authority,
        clear_intent: hard_stop,
        status: None,
    }
}

/// Builds the idle hold outcome used when no active intent exists.
const fn idle_hold_outcome(
    authority: AuthorityState,
    command: Option<PlatformCommand>,
) -> DecisionOutcome {
    DecisionOutcome {
        command,
        veto: None,
        authority,
        clear_intent: false,
        status: None,
    }
}

/// Builds an outcome for an approved platform command.
const fn command_outcome(authority: AuthorityState, command: PlatformCommand) -> DecisionOutcome {
    DecisionOutcome {
        command: Some(command),
        veto: None,
        authority,
        clear_intent: false,
        status: None,
    }
}

/// Returns a supersede veto when `previous` was an unfinished autonomy-sourced
/// intent that a different `current` intent has just replaced in the inbox
/// latch — so the planner awaiting `previous`'s completion is told it will
/// never receive one, rather than hang.
///
/// A planner may await the `CommandStatus`/`CommandVeto` for the `command_id` it
/// issued. When an operator's (or another) intent silently overwrites that
/// still-latched autonomy intent, no terminal feedback would otherwise be
/// emitted for the old `command_id`. This emits the missing terminal signal: a
/// veto against the *previous* `command_id` with reason
/// `veto_reason_superseded`, addressed to the autonomy source that owns it.
///
/// The still-latched state of `previous` is itself the "unfinished" proof: a
/// completed intent is cleared from the latch (see
/// `RuntimeInbox::clear_decision_latches`), so an autonomy intent that is both
/// still latched and now replaced was genuinely preempted mid-flight.
///
/// Returns `None` unless `previous` exists, is autonomy-sourced, and carries a
/// genuinely different `command_id` than `current`. Identical command ids are a
/// duplicate/retransmit of the same command, not a supersede.
pub fn superseded_intent_veto(
    agent_id: &str,
    previous: Option<&WaypointIntent>,
    current: &WaypointIntent,
    stamp_ms: u64,
) -> Option<CommandVeto> {
    let previous = previous?;
    if previous.source != command_source_autonomy() {
        return None;
    }

    let previous_command_id = waypoint_command_id(agent_id, previous.stamp_ms);
    if previous_command_id == waypoint_command_id(agent_id, current.stamp_ms) {
        return None;
    }

    Some(command_veto(
        agent_id,
        previous_command_id,
        previous.source,
        veto_reason_superseded(),
        "Superseded by a newer waypoint intent before completion".to_owned(),
        stamp_ms,
    ))
}

/// Builds a zero-reason veto message that clears a previously active veto.
pub fn cleared_veto(agent_id: &str, stamp_ms: u64) -> CommandVeto {
    command_veto(
        agent_id,
        waypoint_command_id(agent_id, stamp_ms),
        command_source_none(),
        veto_reason_none(),
        String::new(),
        stamp_ms,
    )
}

/// Constructs an approved velocity command with the setpoint attached.
///
/// `target_position` remains the map-frame goal (not the instantaneous
/// setpoint) purely for observability: UI/MCAP consumers can see where the
/// robot is headed even though motion is driven by `velocity`. A velocity
/// command always ends any prior hold episode (`next_hold_anchor` releases the
/// anchor), so it never carries an episode id.
fn velocity_command(
    agent_id: &str,
    command_id: String,
    target_position: [f32; 3],
    setpoint: VelocitySetpoint,
    source: u8,
    stamp_ms: u64,
) -> PlatformCommand {
    PlatformCommand {
        agent_id: agent_id.to_owned(),
        command_id,
        command_type: command_type_velocity(),
        source,
        approved_by: APPROVED_BY_ONBOARD_AGENT.to_owned(),
        target_position,
        stamp_ms,
        frame_id: MAP_FRAME.to_owned(),
        episode_id: String::new(),
        command_authority: 0,
        velocity: Some(setpoint),
    }
}

/// Constructs an approved hold command.
fn hold_command(
    agent_id: &str,
    command_id: String,
    command_type: u8,
    source: u8,
    hold_position: [f32; 3],
    episode_id: String,
    stamp_ms: u64,
) -> PlatformCommand {
    PlatformCommand {
        agent_id: agent_id.to_owned(),
        command_id,
        command_type,
        source,
        approved_by: APPROVED_BY_ONBOARD_AGENT.to_owned(),
        target_position: hold_position,
        stamp_ms,
        frame_id: MAP_FRAME.to_owned(),
        episode_id,
        command_authority: 0,
        velocity: None,
    }
}

/// Constructs a command veto message.
fn command_veto(
    agent_id: &str,
    command_id: String,
    source: u8,
    reason: u8,
    detail: String,
    stamp_ms: u64,
) -> CommandVeto {
    CommandVeto {
        agent_id: agent_id.to_owned(),
        command_id,
        source,
        reason,
        detail,
        stamp_ms,
    }
}

/// Builds the deterministic command ID associated with an intent.
fn command_id_for_intent(agent_id: &str, intent: &WaypointIntent) -> String {
    waypoint_command_id(agent_id, intent.stamp_ms)
}

#[cfg(test)]
#[path = "../../../tests/unit/agent/decision.rs"]
mod decision_tests;
