//! Authority policy for deciding which sources the onboard agent trusts.
//!
//! Each agent selects a simulated, hardware, or hybrid backend. This module
//! converts manifest policy into a runtime authority snapshot and exposes
//! helper checks for command authorization and constraint-source trust.

use core_msgs::config::AgentBackend;
use core_msgs::{
    AuthorityState, collision_authority_avian_contacts, collision_authority_fused,
    collision_authority_none, collision_authority_real_sensors, command_source_autonomy,
    command_source_human_operator, command_source_none, physics_authority_avian,
    physics_authority_real_world, runtime_mode_hybrid, runtime_mode_real, runtime_mode_sim,
    source_environment_runtime, source_spot_adapter, state_authority_environment_runtime,
    state_authority_real,
};

use super::config::AgentManifest;

/// Manifest value meaning autonomy may command the platform.
const COMMAND_AUTHORITY_AUTONOMY: &str = "autonomy";

/// Manifest value meaning human operator intents may command the platform.
const COMMAND_AUTHORITY_HUMAN_OPERATOR: &str = "human_operator";

/// Builds the runtime authority snapshot implied by the configured backend.
pub fn authority_from_manifest(manifest: &AgentManifest, now_ms: u64) -> AuthorityState {
    match manifest.backend {
        AgentBackend::Simulated => sim_as_truth_authority(manifest, now_ms),
        AgentBackend::Hardware => real_as_truth_authority(manifest, now_ms),
        AgentBackend::Hybrid => hybrid_as_truth_authority(manifest, now_ms),
    }
}

/// Returns the state-authority code the runtime should accept for this backend.
pub const fn expected_state_authority(manifest: &AgentManifest) -> u8 {
    match manifest.backend {
        AgentBackend::Simulated => state_authority_environment_runtime(),
        AgentBackend::Hardware | AgentBackend::Hybrid => state_authority_real(),
    }
}

/// Returns true when the intent source may command the platform.
///
/// The manifest's single `command_authority` always admits its matching source
/// (the exclusive gate used when autonomy is off). When `autonomy_enabled` is
/// set, both the human operator and the autonomy planner become admissible;
/// the runtime inbox's source-blind latest-wins latch then decides which most
/// recently latched intent actually drives motion, so the operator preempts
/// autonomy simply by commanding again — not via any priority comparison here.
/// `command_source_none` is never admitted.
pub const fn commands_allowed(
    authority: &AuthorityState,
    source: u8,
    autonomy_enabled: bool,
) -> bool {
    if authority.command_authority == source {
        return true;
    }

    autonomy_enabled
        && (source == command_source_autonomy() || source == command_source_human_operator())
}

/// Checks whether a constraint source is trusted under current collision authority.
///
/// Constraint events are ignored unless the current backend explicitly trusts the
/// component that produced them (sim: environment; real: Spot adapter; hybrid:
/// both, fused).
pub const fn trusts_constraint_source(authority: &AuthorityState, source: u8) -> bool {
    let from_environment = source == source_environment_runtime();
    let from_spot_adapter = source == source_spot_adapter();

    match authority.collision_authority {
        collision if collision == collision_authority_avian_contacts() => from_environment,
        collision if collision == collision_authority_real_sensors() => from_spot_adapter,
        collision if collision == collision_authority_fused() => {
            from_environment || from_spot_adapter
        }
        _ => false,
    }
}

/// Builds authority policy for a simulated backend.
fn sim_as_truth_authority(manifest: &AgentManifest, now_ms: u64) -> AuthorityState {
    AuthorityState {
        agent_id: manifest.agent_id.as_str().to_owned(),
        command_authority: command_source_from_manifest_value(manifest.command_authority.as_str()),
        state_authority: state_authority_environment_runtime(),
        physics_authority: physics_authority_avian(),
        collision_authority: environment_collision_authority(manifest),
        mode: runtime_mode_sim(),
        reason: sim_reason(manifest),
        stamp_ms: now_ms,
    }
}

/// Returns environment collision authority when the manifest allows it.
const fn environment_collision_authority(manifest: &AgentManifest) -> u8 {
    if manifest.allow_environment_constraints {
        collision_authority_avian_contacts()
    } else {
        collision_authority_none()
    }
}

/// Builds the human-readable authority reason for a simulated backend.
fn sim_reason(manifest: &AgentManifest) -> String {
    format!(
        "sim-as-truth; {} command authority; state timeout {}ms; environment constraints {}",
        manifest.command_authority.as_str(),
        manifest.state_timeout_ms,
        manifest.allow_environment_constraints
    )
}

/// Builds authority policy for a hardware backend.
fn real_as_truth_authority(manifest: &AgentManifest, now_ms: u64) -> AuthorityState {
    AuthorityState {
        agent_id: manifest.agent_id.as_str().to_owned(),
        command_authority: command_source_from_manifest_value(manifest.command_authority.as_str()),
        state_authority: state_authority_real(),
        physics_authority: physics_authority_real_world(),
        collision_authority: collision_authority_real_sensors(),
        mode: runtime_mode_real(),
        reason: real_reason(manifest),
        stamp_ms: now_ms,
    }
}

/// Builds the human-readable authority reason for a hardware backend.
fn real_reason(manifest: &AgentManifest) -> String {
    format!(
        "real-is-truth; {} command authority; platform {}; state timeout {}ms",
        manifest.command_authority.as_str(),
        manifest.platform.as_str(),
        manifest.state_timeout_ms
    )
}

/// Builds authority policy for a hybrid backend.
fn hybrid_as_truth_authority(manifest: &AgentManifest, now_ms: u64) -> AuthorityState {
    AuthorityState {
        agent_id: manifest.agent_id.as_str().to_owned(),
        command_authority: command_source_from_manifest_value(manifest.command_authority.as_str()),
        state_authority: state_authority_real(),
        physics_authority: physics_authority_real_world(),
        collision_authority: collision_authority_fused(),
        mode: runtime_mode_hybrid(),
        reason: hybrid_reason(manifest),
        stamp_ms: now_ms,
    }
}

/// Builds the human-readable authority reason for a hybrid backend.
fn hybrid_reason(manifest: &AgentManifest) -> String {
    format!(
        "hybrid; {} command authority; platform {}; fused real + virtual constraints; state timeout {}ms",
        manifest.command_authority.as_str(),
        manifest.platform.as_str(),
        manifest.state_timeout_ms
    )
}

/// Maps a manifest command-authority string to a shared source code.
fn command_source_from_manifest_value(value: &str) -> u8 {
    match value {
        COMMAND_AUTHORITY_AUTONOMY => command_source_autonomy(),
        COMMAND_AUTHORITY_HUMAN_OPERATOR => command_source_human_operator(),
        _ => command_source_none(),
    }
}
