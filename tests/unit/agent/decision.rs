use super::*;
use crate::config::AgentManifest;
use core_msgs::config::AgentBackend;
use core_msgs::{
    Behavior, CapabilitySet, ConstraintEvent, NavigationCapability, NavigationKind, SpeedLimits,
    SpeedPreset, SpeedPresets, behavior_code, command_source_autonomy,
    command_source_human_operator, constraint_type_virtual_wall, severity_hard_stop,
    source_spot_adapter,
};

const AGENT_ID: &str = "agent_01";

fn assert_f32_near(actual: f32, expected: f32) {
    assert!((actual - expected).abs() < f32::EPSILON);
}

fn assert_vec3_near(actual: [f32; 3], expected: [f32; 3]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert_f32_near(actual, expected);
    }
}

/// Builds a manifest for the given backend; the default `command_authority`
/// (`human_operator`) matches `build_intent`'s source.
fn build_manifest(backend: AgentBackend) -> AgentManifest {
    AgentManifest {
        backend,
        ..Default::default()
    }
}

fn build_config(state_timeout_ms: u64) -> DecisionConfig {
    DecisionConfig {
        agent_id: AGENT_ID.to_owned(),
        flee_distance: 0.8,
        state_timeout_ms,
        guidance_limits: guidance::PlanarGuidanceLimits {
            max_linear_speed_m_s: 1.5,
            max_yaw_rate_rad_s: 1.5,
            position_tolerance_m: 0.08,
        },
        autonomy_enabled: false,
    }
}

fn build_state(position: [f32; 3], heading_yaw_rad: f32, stamp_ms: u64) -> AgentStateMsg {
    AgentStateMsg {
        agent_id: AGENT_ID.to_owned(),
        frame_id: MAP_FRAME.to_owned(),
        position,
        heading_yaw_rad,
        stamp_ms,
        ..Default::default()
    }
}

fn build_intent(point: [f32; 3], stamp_ms: u64) -> WaypointIntent {
    WaypointIntent {
        agent_id: AGENT_ID.to_owned(),
        source: command_source_human_operator(),
        behavior: behavior_code(Behavior::GoTo),
        point,
        stamp_ms,
        frame_id: MAP_FRAME.to_owned(),
    }
}

#[test]
fn selected_platform_speed_preset_sets_guidance_limits() {
    let capabilities = CapabilitySet {
        hold: true,
        navigation: vec![NavigationCapability {
            kind: NavigationKind::Planar,
            speed_presets: SpeedPresets {
                slow: SpeedLimits {
                    max_linear_speed_m_s: 0.3,
                    max_yaw_rate_rad_s: 0.4,
                },
                medium: SpeedLimits {
                    max_linear_speed_m_s: 0.8,
                    max_yaw_rate_rad_s: 0.9,
                },
                fast: SpeedLimits {
                    max_linear_speed_m_s: 1.4,
                    max_yaw_rate_rad_s: 1.5,
                },
            },
            position_tolerance_m: 0.08,
            supports_target_heading: false,
        }],
        manipulation: None,
    };

    let config = DecisionConfig::from_agent_config(
        &crate::config::AgentConfig::default(),
        1_000,
        false,
        SpeedPreset::Slow,
        &capabilities,
    );

    assert_f32_near(config.guidance_limits.max_linear_speed_m_s, 0.3);
    assert_f32_near(config.guidance_limits.max_yaw_rate_rad_s, 0.4);
}

#[test]
fn real_mode_track_emits_velocity_across_two_ticks() {
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let authority = authority::authority_from_manifest(&manifest, 10_000);
    let intent = build_intent([5.0, 0.0, 0.0], 10_000);
    let no_constraints = ActiveConstraints::default();

    let state1 = build_state([0.0, 0.0, 0.0], 0.0, 10_000);
    let outcome1 = next_decision(
        &config,
        &authority,
        10_000,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state1),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );
    let (command1, _, veto1, status1) = outcome1.into_outputs();
    let command1 = command1.expect("far from target must emit a command");

    assert_eq!(command1.command_type, command_type_velocity());
    assert!(command1.velocity.is_some());
    assert_vec3_near(command1.target_position, [5.0, 0.0, 0.0]);
    assert!(veto1.is_none());
    assert!(status1.is_none());

    // Second tick, same intent, still far from target: still a velocity
    // command (a non-empty command stream, not a one-shot).
    let state2 = build_state([0.1, 0.0, 0.0], 0.0, 10_025);
    let outcome2 = next_decision(
        &config,
        &authority,
        10_025,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state2),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );
    let (command2, _, _, _) = outcome2.into_outputs();
    let command2 = command2.expect("still far from target must emit a command");

    assert_eq!(command2.command_type, command_type_velocity());
    assert!(command2.velocity.is_some());
}

#[test]
fn real_mode_track_reaching_tolerance_completes_with_hold_and_status() {
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 20_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);
    let intent = build_intent([5.0, 0.0, 0.0], now_ms);
    let no_constraints = ActiveConstraints::default();
    // 0.03 m from the target: inside POSITION_TOLERANCE_M.
    let state = build_state([4.97, 0.0, 0.0], 0.0, now_ms);

    let outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );

    assert!(outcome.clears_intent());
    let (command, _, _, status) = outcome.into_outputs();
    let command = command.expect("a reached goal must still command a hold");
    assert_eq!(command.command_type, command_type_hold());

    let status = status.expect("reaching the goal must self-report success");
    assert_eq!(status.status, command_status_succeeded());
    assert_eq!(status.command_id, waypoint_command_id(AGENT_ID, now_ms));
}

#[test]
fn sim_mode_track_now_emits_velocity_command() {
    // Sim mode now shares the onboard PD velocity path with real/hybrid: it
    // emits a velocity setpoint (executed in avian physics), not a trajectory.
    let manifest = build_manifest(AgentBackend::Simulated);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 10_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);
    let intent = build_intent([5.0, 0.0, 0.0], now_ms);
    let no_constraints = ActiveConstraints::default();
    let state = build_state([0.0, 0.0, 0.0], 0.0, now_ms);

    let outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );

    let (command, _, _, status) = outcome.into_outputs();
    let command = command.expect("sim mode must command a velocity setpoint");

    assert_eq!(command.command_type, command_type_velocity());
    assert!(command.velocity.is_some());
    assert_vec3_near(command.target_position, [5.0, 0.0, 0.0]);
    assert!(status.is_none());
}

#[test]
fn stale_state_in_real_mode_holds_instead_of_commanding_velocity() {
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 100_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);
    let intent = build_intent([5.0, 0.0, 0.0], now_ms);
    let no_constraints = ActiveConstraints::default();
    // Stamped long before `now_ms`: older than config.state_timeout_ms.
    let stale_state = build_state([0.0, 0.0, 0.0], 0.0, 0);

    let outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&stale_state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );

    let (command, _, veto, _) = outcome.into_outputs();
    let veto = veto.expect("stale state must veto");
    assert_eq!(veto.reason, veto_reason_stale_state());
    // The safety-hold gate in `next_decision` runs before the
    // real-vs-sim physics_authority branch, so a stale state can never
    // reach velocity_decision and never produces a velocity command.
    if let Some(command) = command {
        assert_ne!(command.command_type, command_type_velocity());
    }
}

#[test]
fn platform_confirmed_dead_holds_instead_of_commanding_velocity() {
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 30_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);
    let intent = build_intent([5.0, 0.0, 0.0], now_ms);
    let no_constraints = ActiveConstraints::default();
    // Fresh state in the map frame: isolates the platform-dead branch from the
    // bad-frame and stale-state checks that run beside it.
    let state = build_state([0.0, 0.0, 0.0], 0.0, now_ms);

    let outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: Some(false),
        },
    );

    let (command, _, veto, _) = outcome.into_outputs();
    let veto = veto.expect("a confirmed-dead platform must veto");
    assert_eq!(veto.reason, veto_reason_platform_dead());
    if let Some(command) = command {
        assert_ne!(command.command_type, command_type_velocity());
    }
}

#[test]
fn platform_alive_unknown_does_not_hold() {
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 30_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);
    let intent = build_intent([5.0, 0.0, 0.0], now_ms);
    let no_constraints = ActiveConstraints::default();
    let state = build_state([0.0, 0.0, 0.0], 0.0, now_ms);

    let outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            // No liveliness info yet: must not hold, normal motion proceeds.
            platform_alive: None,
        },
    );

    let (command, _, veto, _) = outcome.into_outputs();
    assert!(veto.is_none(), "unknown liveliness must not veto");
    let command = command.expect("unknown liveliness must still command motion");
    assert_eq!(command.command_type, command_type_velocity());
}

#[test]
fn hard_stop_holds_autonomy_intent_by_safety_before_source_gate() {
    // Pins existing behavior: a trusted hard-stop safety hold outranks the
    // command-source gate for an autonomy-sourced intent exactly as it does for
    // a human one. This passes at the default autonomy_enabled = false, because
    // the safety branch in next_decision returns before commands_allowed is ever
    // consulted — so collision/fault abort is source-independent.
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 40_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);

    let mut intent = build_intent([5.0, 0.0, 0.0], now_ms);
    intent.source = command_source_autonomy();

    // Real mode's collision authority trusts the Spot adapter as the source.
    let mut constraints = ActiveConstraints::default();
    constraints.apply(
        ConstraintEvent {
            agent_id: AGENT_ID.to_owned(),
            source: source_spot_adapter(),
            constraint_type: constraint_type_robot_fault(),
            severity: severity_hard_stop(),
            object_id: "imu".to_owned(),
            stamp_ms: now_ms,
            ..Default::default()
        },
        now_ms,
    );

    let state = build_state([0.0, 0.0, 0.0], 0.0, now_ms);
    let outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state),
            constraints: &constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );

    let (command, _, veto, _) = outcome.into_outputs();
    let veto = veto.expect("an active hard stop must veto");
    // The veto is the safety reason, NOT unauthorized_source: safety outranks
    // the command-source gate regardless of which source sent the intent.
    assert_eq!(veto.reason, veto_reason_robot_fault());
    assert_ne!(veto.reason, veto_reason_unauthorized_source());
    if let Some(command) = command {
        assert_ne!(command.command_type, command_type_velocity());
    }
}

#[test]
fn active_collision_stop_is_not_republished_after_it_is_anchored() {
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 50_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);
    let intent = build_intent([5.0, 0.0, 0.0], now_ms);
    let state = build_state([0.5, 0.25, 0.0], 0.0, now_ms);
    let mut constraints = ActiveConstraints::default();
    constraints.apply(
        ConstraintEvent {
            agent_id: AGENT_ID.to_owned(),
            source: source_spot_adapter(),
            constraint_type: constraint_type_virtual_wall(),
            severity: severity_hard_stop(),
            object_id: "obstacle:test".to_owned(),
            stamp_ms: now_ms,
            episode_id: "collision-1".to_owned(),
            ..Default::default()
        },
        now_ms,
    );

    let collision_outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state),
            constraints: &constraints,
            status: None,
            hold_anchor: Some(HoldAnchor {
                position: state.position,
                episode_id: String::new(),
            }),
            platform_alive: None,
        },
    );
    let collision_anchor = collision_outcome
        .next_hold_anchor(None)
        .expect("collision stop must latch a hold anchor");
    let (collision_command, _, _, _) = collision_outcome.into_outputs();
    let collision_command = collision_command.expect("collision must command a stop");
    assert_eq!(collision_command.command_type, command_type_stop());
    assert_eq!(collision_command.episode_id, "collision-1");

    let repeated_outcome = next_decision(
        &config,
        &authority,
        now_ms + 1,
        &DecisionInputs {
            intent: None,
            state: Some(&state),
            constraints: &constraints,
            status: None,
            hold_anchor: Some(collision_anchor),
            platform_alive: None,
        },
    );
    let (repeated_command, _, _, _) = repeated_outcome.into_outputs();
    assert!(
        repeated_command.is_none(),
        "an already-commanded collision stop must not flood the bus"
    );
}

#[test]
fn cleared_collision_releases_safety_episode_then_remains_quiet() {
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 50_002;
    let authority = authority::authority_from_manifest(&manifest, now_ms);
    let state = build_state([0.5, 0.25, 0.0], 0.0, now_ms);
    let constraints = ActiveConstraints::default();
    let collision_anchor = HoldAnchor {
        position: state.position,
        episode_id: "collision-1".to_owned(),
    };

    let clear_outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: None,
            state: Some(&state),
            constraints: &constraints,
            status: None,
            hold_anchor: Some(collision_anchor),
            platform_alive: None,
        },
    );
    let cleared_anchor = clear_outcome
        .next_hold_anchor(None)
        .expect("collision clear must preserve the hold position");
    let (clear_command, _, _, _) = clear_outcome.into_outputs();
    let clear_command = clear_command.expect("collision clear must reach the executor");
    assert_eq!(clear_command.command_type, command_type_hold());
    assert!(clear_command.episode_id.is_empty());

    let idle_outcome = next_decision(
        &config,
        &authority,
        now_ms + 1,
        &DecisionInputs {
            intent: None,
            state: Some(&state),
            constraints: &constraints,
            status: None,
            hold_anchor: Some(cleared_anchor),
            platform_alive: None,
        },
    );
    let (idle_command, _, _, _) = idle_outcome.into_outputs();
    assert!(
        idle_command.is_none(),
        "a settled ordinary hold must remain quiet"
    );
}

#[test]
fn autonomy_intent_rejected_when_autonomy_disabled() {
    // Default autonomy_enabled = false keeps the exclusive gate: with the
    // manifest's default human_operator authority, an autonomy-sourced intent is
    // vetoed as an unauthorized source and never becomes a velocity command.
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = build_config(manifest.state_timeout_ms);
    let now_ms = 10_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);

    let mut intent = build_intent([5.0, 0.0, 0.0], now_ms);
    intent.source = command_source_autonomy();
    let no_constraints = ActiveConstraints::default();
    let state = build_state([0.0, 0.0, 0.0], 0.0, now_ms);

    let outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );

    let (command, _, veto, _) = outcome.into_outputs();
    let veto = veto.expect("an unauthorized source must veto");
    assert_eq!(veto.reason, veto_reason_unauthorized_source());
    if let Some(command) = command {
        assert_ne!(command.command_type, command_type_velocity());
    }
}

#[test]
fn autonomy_intent_admitted_when_autonomy_enabled() {
    // With autonomy_enabled = true the autonomy planner is admissible even
    // though the manifest's nominal command_authority is human_operator: the
    // intent produces a velocity command like any authorized source.
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = DecisionConfig {
        autonomy_enabled: true,
        ..build_config(manifest.state_timeout_ms)
    };
    let now_ms = 10_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);

    let mut intent = build_intent([5.0, 0.0, 0.0], now_ms);
    intent.source = command_source_autonomy();
    let no_constraints = ActiveConstraints::default();
    let state = build_state([0.0, 0.0, 0.0], 0.0, now_ms);

    let outcome = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&intent),
            state: Some(&state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );

    let (command, _, veto, _) = outcome.into_outputs();
    assert!(veto.is_none(), "an admitted autonomy intent must not veto");
    let command = command.expect("an admitted autonomy intent must command motion");
    assert_eq!(command.command_type, command_type_velocity());
    assert!(command.velocity.is_some());
}

#[test]
fn operator_preempts_autonomy_when_autonomy_enabled() {
    // With autonomy_enabled = true, the decision layer drives motion toward
    // whichever intent is currently latched. Tick 1: an autonomy intent drives
    // toward its target. Tick 2: a newer human-operator intent has replaced it
    // and drives toward the OPERATOR's target instead. The overwrite-on-arrival
    // itself is the runtime inbox's source-blind latest-wins latch (runtime.rs),
    // out of scope for this pure-decision-logic test; here we pass the operator
    // intent directly as inputs.intent. This proves the decision layer honors
    // whatever the latch currently holds, which combined with that inbox latch
    // delivers "operator always overrides autonomy".
    let manifest = build_manifest(AgentBackend::Hardware);
    let config = DecisionConfig {
        autonomy_enabled: true,
        ..build_config(manifest.state_timeout_ms)
    };
    let now_ms = 10_000;
    let authority = authority::authority_from_manifest(&manifest, now_ms);
    let no_constraints = ActiveConstraints::default();
    let state = build_state([0.0, 0.0, 0.0], 0.0, now_ms);

    let mut autonomy_intent = build_intent([5.0, 0.0, 0.0], now_ms);
    autonomy_intent.source = command_source_autonomy();

    let outcome_autonomy = next_decision(
        &config,
        &authority,
        now_ms,
        &DecisionInputs {
            intent: Some(&autonomy_intent),
            state: Some(&state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );
    let (command_autonomy, _, veto_autonomy, _) = outcome_autonomy.into_outputs();
    assert!(veto_autonomy.is_none(), "autonomy intent must be admitted");
    let command_autonomy = command_autonomy.expect("autonomy intent must drive motion");
    assert_eq!(command_autonomy.command_type, command_type_velocity());
    assert_vec3_near(command_autonomy.target_position, [5.0, 0.0, 0.0]);

    // Operator preempts: a newer human-operator intent toward a different target
    // is now the latched intent.
    let operator_intent = build_intent([-5.0, 0.0, 0.0], now_ms + 1);
    let outcome_operator = next_decision(
        &config,
        &authority,
        now_ms + 1,
        &DecisionInputs {
            intent: Some(&operator_intent),
            state: Some(&state),
            constraints: &no_constraints,
            status: None,
            hold_anchor: None,
            platform_alive: None,
        },
    );
    let (command_operator, _, veto_operator, _) = outcome_operator.into_outputs();
    assert!(veto_operator.is_none(), "operator intent must be admitted");
    let command_operator = command_operator.expect("operator intent must drive motion");
    assert_eq!(command_operator.command_type, command_type_velocity());
    // Drives toward the OPERATOR's target, not the autonomy one.
    assert_vec3_near(command_operator.target_position, [-5.0, 0.0, 0.0]);
}

#[test]
fn supersede_vetoes_autonomy_intent_replaced_by_different_intent() {
    // Latched autonomy intent A is replaced by a different operator intent B:
    // the planner awaiting A's completion must be told A was preempted.
    let mut autonomy_previous = build_intent([1.0, 0.0, 0.0], 5_000);
    autonomy_previous.source = command_source_autonomy();
    let operator_current = build_intent([-1.0, 0.0, 0.0], 6_000);

    let veto = superseded_intent_veto(AGENT_ID, Some(&autonomy_previous), &operator_current, 6_000)
        .expect("a replaced unfinished autonomy intent must be vetoed as superseded");

    assert_eq!(veto.reason, veto_reason_superseded());
    // The veto targets the PREVIOUS (superseded) command, not the new one.
    assert_eq!(veto.command_id, waypoint_command_id(AGENT_ID, 5_000));
    assert_eq!(veto.source, command_source_autonomy());
}

#[test]
fn supersede_no_veto_when_no_previous_intent() {
    let current = build_intent([1.0, 0.0, 0.0], 5_000);
    assert!(superseded_intent_veto(AGENT_ID, None, &current, 5_000).is_none());
}

#[test]
fn supersede_no_veto_when_previous_is_human_operator() {
    // An operator intent replaced by a newer operator intent is ordinary
    // command replacement, not a preemption anyone is waiting to hear about.
    let operator_previous = build_intent([1.0, 0.0, 0.0], 5_000);
    assert_eq!(operator_previous.source, command_source_human_operator());
    let operator_current = build_intent([-1.0, 0.0, 0.0], 6_000);

    assert!(
        superseded_intent_veto(AGENT_ID, Some(&operator_previous), &operator_current, 6_000)
            .is_none()
    );
}

#[test]
fn supersede_no_veto_on_same_command_id_retransmit() {
    // Same stamp_ms means the same deterministic command_id: a retransmit of the
    // same autonomy command, not a genuinely new one to supersede.
    let mut autonomy_previous = build_intent([1.0, 0.0, 0.0], 5_000);
    autonomy_previous.source = command_source_autonomy();
    let mut autonomy_current = build_intent([1.0, 0.0, 0.0], 5_000);
    autonomy_current.source = command_source_autonomy();

    assert!(
        superseded_intent_veto(AGENT_ID, Some(&autonomy_previous), &autonomy_current, 5_000)
            .is_none()
    );
}
