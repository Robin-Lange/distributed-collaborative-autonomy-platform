//! Entry point for the Bevy/Avian environment simulator.
//!
//! For simulated agents this process owns physics and publishes agent state and
//! wall constraints. For hybrid agents it mirrors real-platform state and
//! evaluates virtual-world constraints.

mod app;
mod config;
mod dds;
mod startup;
mod systems;
mod world;

/// Loads configuration, starts DDS I/O, and runs the selected simulation role.
pub fn run() {
    let app_config = core_msgs::config::load_app_config();
    // Built from the launcher's already-loaded AppConfig, not a second file
    // read, so these always match whatever the launcher actually shipped
    // (including any objects the operator added interactively).
    let frames = core_msgs::frames::FramesConfig::from_config(&app_config.frames);
    let catalog = core_msgs::catalog::ObjectCatalog::from_config(app_config.objects.clone());

    let config = config::EnvironmentConfig::new(app_config);
    let simulated_agent_ids = config
        .app
        .agents
        .iter()
        .filter(|agent| agent.manifest.backend == core_msgs::config::AgentBackend::Simulated)
        .map(|agent| agent.runtime.agent_id.clone())
        .collect();
    let dds = dds::spawn_environment_dds_io(simulated_agent_ids);

    app::build_app(config, frames.t_map_bevy, dds, catalog).run();
}
