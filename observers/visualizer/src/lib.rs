//! Entry point for the read-only Bevy visualizer.
//!
//! The visualizer subscribes to the shared bus, converts incoming platform
//! coordinates into Bevy's render frame, and displays the current agent pose plus
//! lightweight operator/command status. It never publishes commands or state, so
//! it can be run as an observer alongside sim, real, or hybrid stacks.

mod app;
mod config;
mod dds;
mod setup;
mod systems;
mod world;

/// Loads configuration, starts the DDS observation thread, and runs the Bevy app.
pub fn run() {
    // Must run before any other threads start.
    #[cfg(target_os = "linux")]
    force_software_vulkan_on_wsl2();

    let app_config = core_msgs::config::load_app_config();
    // Built from the launcher's already-loaded AppConfig, not a second file
    // read, so these always match whatever the launcher actually shipped
    // (including any objects the operator added interactively).
    let frames = core_msgs::frames::FramesConfig::from_config(&app_config.frames);
    let catalog = core_msgs::catalog::ObjectCatalog::from_config(app_config.objects.clone());
    let config = config::VisualizerConfig::new(app_config);
    let dds = dds::spawn_visualizer_dds_io();

    app::build_app(config, &frames.t_map_bevy, dds, catalog).run();
}

/// On WSL2, force wgpu to use the conformant lavapipe (llvmpipe) software Vulkan adapter.
///
/// The default high-performance Vulkan adapter on WSL2 is Mesa's Dozen driver
/// (Vulkan-over-D3D12), which is explicitly non-conformant ("not a conformant Vulkan
/// implementation, testing use only") and segfaults during rendering on the NVIDIA
/// RTX 5080. lavapipe is CPU-based but stable; the visualizer's scene is light enough
/// that software rendering is perfectly adequate.
///
/// Gated to WSL2 (and skipped if the user already set `VK_ICD_FILENAMES`) so that native
/// Ubuntu, Windows, and macOS keep their hardware-accelerated GPU path untouched.
#[cfg(target_os = "linux")]
fn force_software_vulkan_on_wsl2() {
    const LAVAPIPE_ICD: &str = "/usr/share/vulkan/icd.d/lvp_icd.x86_64.json";

    let is_wsl2 = std::env::var_os("WSL_DISTRO_NAME").is_some();
    if is_wsl2
        && std::env::var_os("VK_ICD_FILENAMES").is_none()
        && std::path::Path::new(LAVAPIPE_ICD).exists()
    {
        // SAFETY: called at program start, before any other threads are spawned.
        unsafe { std::env::set_var("VK_ICD_FILENAMES", LAVAPIPE_ICD) };
    }
}
