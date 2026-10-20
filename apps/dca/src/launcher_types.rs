//! Shared launcher data shapes.
//!
//! Shared by launcher process management and the operator TUI.

use secrecy::SecretString;
use std::time::Duration;

pub const CHILD_SHUTDOWN_GRACE: Duration = Duration::from_secs(5);
pub const CHILD_SHUTDOWN_POLL: Duration = Duration::from_millis(50);
pub const SPOT_RUNTIME_SHUTDOWN_GRACE: Duration = Duration::from_secs(25);
pub const SPOT_COMPONENT_SHUTDOWN_GRACE: Duration = Duration::from_secs(30);

#[derive(Clone)]
pub struct SidecarAutoFlags {
    pub power_on: bool,
    pub stand: bool,
    pub calibrate: bool,
}

// Not `Clone`: the credential must not be silently duplicated. `SecretString`
// is deliberately non-`Clone`, and nothing in the launcher clones options.
pub struct LauncherOptions {
    pub visualizer: bool,
    pub mcap_recorder: bool,
    pub sidecar_ip: String,
    pub sidecar_auto: SidecarAutoFlags,
    pub sidecar_mocap_body_id: u32,
    /// Translation from Spot's Motive rigid-body origin to Spot's internal
    /// BODY origin, expressed in BODY `[forward, left, up]` axes, in meters.
    pub sidecar_mocap_to_body_translation_m: [f64; 3],
    pub mocap_ip: String,
    pub mocap_port: u16,
    pub mocap_wsl_relay: bool,
    /// Spot credential loaded from the environment or entered through the
    /// masked Ratatui prompt. Lives only in process memory and the sidecar
    /// children's environment — never on disk, never in source. Held in a
    /// `SecretString` so it is zeroized on drop and redacted from any `Debug`
    /// output; reading it back requires an explicit `expose_secret()`.
    pub bosdyn_password: Option<SecretString>,
}

/// One managed runtime component. `main.rs` adds a second `impl Component`
/// block for `from_os_string`, the `--component <name>` re-entry parser —
/// that half is CLI-argument handling the doc-build library target has no
/// need for, so it stays out of this shared file.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub enum Component {
    EnvironmentSim,
    Agent,
    Visualizer,
    McapRecorder,
    MocapZenohBridge,
    ObjectTracker,
    SpotSidecar,
}

impl Component {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::EnvironmentSim => "environment_sim",
            Self::Agent => "agent",
            Self::Visualizer => "visualizer",
            Self::McapRecorder => "mcap_recorder",
            Self::MocapZenohBridge => "mocap_zenoh_bridge",
            Self::ObjectTracker => "object_tracker",
            Self::SpotSidecar => "spot_sidecar",
        }
    }

    pub const fn is_required_runtime(self) -> bool {
        matches!(
            self,
            Self::EnvironmentSim | Self::Agent | Self::ObjectTracker | Self::SpotSidecar
        )
    }

    pub const fn shutdown_grace(self) -> Duration {
        if matches!(self, Self::SpotSidecar) {
            SPOT_COMPONENT_SHUTDOWN_GRACE
        } else {
            CHILD_SHUTDOWN_GRACE
        }
    }
}
