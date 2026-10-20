//! Entry point for the onboard autonomy agent.
//!
//! The agent is the decision-making process in the stack. It reads operator
//! intents, state, constraints, and command status from adapters; applies
//! authority and safety policy; and publishes approved platform commands,
//! authority snapshots, and veto messages.

mod authority;
mod behavior_gate;
mod bus_adapter;
mod config;
mod decision;
mod manipulation;
mod runtime;

use std::env;

/// Environment variable naming which configured agent this process controls.
const AGENT_ID_ENV: &str = "DCA_AGENT_ID";

/// Loads configs, validates identity, starts adapters, and enters the runtime loop.
///
/// # Panics
///
/// Panics when the application config has no agents, the configured
/// `DCA_AGENT_ID` does not exist, or the runtime and manifest identities differ.
pub fn run() {
    let app_config = core_msgs::config::load_app_config();
    let agent = match env::var(AGENT_ID_ENV) {
        Ok(agent_id) => app_config
            .agents
            .iter()
            .find(|agent| agent.runtime.agent_id == agent_id)
            .unwrap_or_else(|| {
                panic!("{AGENT_ID_ENV} '{agent_id}' does not match any configured agent")
            }),
        Err(_) => app_config
            .agents
            .first()
            .unwrap_or_else(|| panic!("app config contains no agents")),
    };

    let config = config::AgentConfig::from(&agent.runtime);
    let manifest = config::AgentManifest::new(&agent.manifest);
    let capabilities = config::load_capabilities(manifest.platform.as_str());

    validate_agent_identity(&config, &manifest);

    // Built from the launcher's already-loaded AppConfig, not a second file
    // read, so this always matches whatever the launcher actually shipped
    // (including any objects the operator added interactively).
    let catalog = core_msgs::catalog::ObjectCatalog::from_config(app_config.objects.clone());

    let (bus, inputs) = bus_adapter::spawn(config.agent_id.as_str());

    runtime::run(&config, &manifest, &capabilities, &catalog, &bus, &inputs);
}

/// Ensures runtime config and manifest describe the same physical/logical agent.
fn validate_agent_identity(config: &config::AgentConfig, manifest: &config::AgentManifest) {
    assert!(
        config.agent_id == manifest.agent_id,
        "Agent ID mismatch: config has '{}', manifest has '{}'",
        config.agent_id,
        manifest.agent_id
    );
}
