//! Unified launcher for the autonomy stack.
//!
//! Running this binary normally starts the operator UI and the components
//! required by each configured agent backend. The same binary can also
//! re-enter itself with `--component <name>` so the launcher can manage child
//! processes while keeping one Cargo target for every Rust runtime component.

pub use core_msgs;

pub use dds_support;

mod operator_bus;

pub use environment_sim;

pub use agent;

pub use visualizer;

pub use mcap_recorder;

pub use sensor_mocap::bridge as mocap_zenoh_bridge;
pub use sensor_mocap::tracker as object_tracker;

mod operator_ui;
mod platform_profiles;
mod runtime_paths;

mod launcher_types;
pub(crate) use launcher_types::{
    CHILD_SHUTDOWN_GRACE, CHILD_SHUTDOWN_POLL, Component, LauncherOptions,
    SPOT_RUNTIME_SHUTDOWN_GRACE, SidecarAutoFlags,
};

use crossbeam_channel::Sender;
use operator_ui::{EventKind, RuntimeEvent};
use runtime_paths::{RUN_ID_ENV, RunPaths, SIDECAR_LOG_DIR_ENV};
use secrecy::zeroize::Zeroizing;
use secrecy::{ExposeSecret, SecretString};
use std::env;
use std::{
    ffi::OsString,
    io::{self, BufRead, BufReader, Read, Write},
    process::{Child, Command, ExitCode, ExitStatus, Stdio},
    thread,
    time::Instant,
};

const COMPONENT_ARG: &str = "--component";
const SIDECAR_CONTROL_STDIN_ENV: &str = "DCA_SPOT_CONTROL_STDIN";

fn main() -> ExitCode {
    match Invocation::from_env() {
        Ok(Invocation::Component(component, args)) => run_component(component, args),
        Ok(Invocation::Help) => {
            println!("{}", usage());
            ExitCode::SUCCESS
        }
        Ok(Invocation::Interactive(mut options)) => {
            resolve_spot_password(&mut options);
            match run_interactive_launcher(options) {
                Ok(code) => code,
                Err(error) => {
                    eprintln!("launcher: {error}");
                    ExitCode::from(1)
                }
            }
        }
        Err(message) => {
            eprintln!("{message}");
            eprintln!("{}", usage());
            ExitCode::from(2)
        }
    }
}

fn run_sidecar(args: Vec<OsString>) -> ExitCode {
    let invocation = match SidecarInvocation::from_args(args) {
        Ok(invocation) => invocation,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::from(2);
        }
    };

    let runtime = match platform_spot::locate_runtime(report_spot_runtime_progress) {
        Ok(runtime) => runtime,
        Err(error) => {
            eprintln!("spot-runtime: unavailable: {error}");
            return ExitCode::FAILURE;
        }
    };

    let mut child = match spawn_sidecar_runtime(&runtime, invocation) {
        Ok(child) => child,
        Err(error) => {
            eprintln!(
                "failed to start Spot runtime '{}': {error}",
                runtime.program().display()
            );
            return ExitCode::FAILURE;
        }
    };

    wait_for_sidecar(&mut child)
}

struct SidecarInvocation {
    args: Vec<OsString>,
    username: String,
    password: Zeroizing<String>,
}

impl SidecarInvocation {
    fn from_args(args: Vec<OsString>) -> Result<Self, String> {
        let mut filtered_args = Vec::new();
        let mut iter = args.into_iter();
        let mut username = env::var("BOSDYN_CLIENT_USERNAME").ok();

        while let Some(arg) = iter.next() {
            if arg == "--username" {
                let Some(value) = iter.next() else {
                    return Err("--username requires a value".to_owned());
                };
                if value.to_string_lossy().starts_with("--") {
                    return Err("--username requires a value".to_owned());
                }
                username = Some(value.to_string_lossy().into_owned());
            } else if arg == "--password" {
                return Err("--password on the command line is not supported; set \
                     BOSDYN_CLIENT_PASSWORD instead"
                    .to_owned());
            } else {
                filtered_args.push(arg);
            }
        }

        let password = env::var("BOSDYN_CLIENT_PASSWORD")
            .map(Zeroizing::new)
            .map_err(|_| "missing Spot password: set BOSDYN_CLIENT_PASSWORD".to_owned())?;

        Ok(Self {
            args: filtered_args,
            username: username.unwrap_or_else(|| "admin".to_owned()),
            password,
        })
    }
}

fn report_spot_runtime_progress(message: &str) {
    println!("spot-runtime: {message}");
    if let Err(error) = io::stdout().flush() {
        eprintln!("spot-runtime: failed to flush progress output: {error}");
    }
}

fn spawn_sidecar_runtime(
    runtime: &platform_spot::SpotRuntimeCommand,
    invocation: SidecarInvocation,
) -> io::Result<Child> {
    let mut command = Command::new(runtime.program());
    command.args(runtime.arguments());
    command.env("BOSDYN_CLIENT_USERNAME", invocation.username);
    command.env("BOSDYN_CLIENT_PASSWORD", invocation.password.as_str());
    // Relayed from this process's own AppConfig (DCA_APP_CONFIG_JSON, set by
    // spawn_sidecar) rather than hardcoded defaults, so the sidecar's object
    // catalog and frame registration always match whatever the launcher
    // actually shipped — including any objects the operator added
    // interactively — instead of silently ignoring it.
    let app_config = crate::core_msgs::config::load_app_config();
    if let Ok(json) = serde_json::to_string(&app_config.objects) {
        command.env("DCA_CONFIG_CONFIG_OBJECTS_JSON", json);
    }
    if let Ok(json) = serde_json::to_string(&app_config.frames) {
        command.env("DCA_CONFIG_CONFIG_FRAMES_JSON", json);
    }
    command
        .env(SIDECAR_CONTROL_STDIN_ENV, "1")
        .args(invocation.args)
        .stdin(Stdio::piped())
        .spawn()
}

fn wait_for_sidecar(child: &mut Child) -> ExitCode {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    if let Err(e) = ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }) {
        eprintln!("failed to set sidecar signal handler: {e}");
    }
    listen_for_sidecar_shutdown(Arc::clone(&running));

    loop {
        match child.try_wait() {
            Ok(Some(status)) => {
                return if status.success() {
                    ExitCode::SUCCESS
                } else {
                    ExitCode::FAILURE
                };
            }
            Ok(None) => {
                if !running.load(Ordering::SeqCst) {
                    stop_sidecar_child(child);
                    return ExitCode::SUCCESS;
                }
            }
            Err(e) => {
                eprintln!("error waiting for sidecar child process: {e}");
                let _ = child.kill();
                return ExitCode::FAILURE;
            }
        }
        thread::sleep(CHILD_SHUTDOWN_POLL);
    }
}

fn listen_for_sidecar_shutdown(running: std::sync::Arc<std::sync::atomic::AtomicBool>) {
    thread::spawn(move || {
        let stdin = io::stdin();
        for line in stdin.lock().lines() {
            if line.is_err() {
                break;
            }
        }
        running.store(false, std::sync::atomic::Ordering::SeqCst);
    });
}

fn stop_sidecar_child(child: &mut Child) {
    if child.stdin.take().is_none() {
        request_sidecar_runtime_shutdown(child);
    }

    let grace_deadline = Instant::now() + SPOT_RUNTIME_SHUTDOWN_GRACE;
    while Instant::now() < grace_deadline {
        if child.try_wait().is_ok_and(|status| status.is_some()) {
            return;
        }
        thread::sleep(CHILD_SHUTDOWN_POLL);
    }
    let _ = child.kill();
    let _ = child.wait();
}

#[cfg(unix)]
fn request_sidecar_runtime_shutdown(child: &Child) {
    let _ = Command::new("kill")
        .arg("-INT")
        .arg(child.id().to_string())
        .status();
}

#[cfg(not(unix))]
fn request_sidecar_runtime_shutdown(child: &mut Child) {
    let _ = child.kill();
}

enum Invocation {
    Interactive(LauncherOptions),
    Component(Component, Vec<OsString>),
    Help,
}

impl Invocation {
    fn from_env() -> Result<Self, String> {
        Self::from_args(env::args_os().skip(1).collect())
    }

    /// Parses launcher arguments. Only component re-entry (`--component <name>`)
    /// and `-h`/`--help` are accepted; any other argument is rejected so that all
    /// run configuration goes through the interactive TUI setup screen, never the
    /// command line.
    fn from_args(mut args: Vec<OsString>) -> Result<Self, String> {
        if args.first().is_some_and(|arg| arg == COMPONENT_ARG) {
            if args.len() < 2 {
                return Err(format!("missing component name after {COMPONENT_ARG}"));
            }

            let component = Component::from_os_string(&args[1])?;
            let component_args = args.split_off(2);
            return Ok(Self::Component(component, component_args));
        }

        if args
            .first()
            .is_some_and(|arg| arg == "-h" || arg == "--help")
        {
            return Ok(Self::Help);
        }

        if !args.is_empty() {
            return Err("passing mode or flags directly is not supported. All settings must be configured through the interactive TUI Setup screen.".to_owned());
        }

        let options = LauncherOptions {
            visualizer: false,
            mcap_recorder: false,
            sidecar_ip: "192.168.55.21".to_owned(),
            sidecar_auto: SidecarAutoFlags {
                power_on: true,
                stand: true,
                calibrate: true,
            },
            sidecar_mocap_body_id: 1,
            sidecar_mocap_to_body_translation_m: [0.0; 3],
            mocap_ip: "239.255.42.99".to_owned(),
            mocap_port: 1511,
            mocap_wsl_relay: true,
            bosdyn_password: None,
        };

        Ok(Self::Interactive(options))
    }
}

impl Component {
    fn from_os_string(value: &OsString) -> Result<Self, String> {
        let Some(value) = value.to_str() else {
            return Err("component name must be valid UTF-8".to_owned());
        };

        match value {
            "environment_sim" => Ok(Self::EnvironmentSim),
            "agent" => Ok(Self::Agent),
            "visualizer" => Ok(Self::Visualizer),
            "mcap_recorder" => Ok(Self::McapRecorder),
            "mocap_zenoh_bridge" => Ok(Self::MocapZenohBridge),
            "object_tracker" => Ok(Self::ObjectTracker),
            "spot_sidecar" => Ok(Self::SpotSidecar),
            _ => Err(format!("unknown component `{value}`")),
        }
    }
}

pub(crate) fn spawn_runtime_children(
    options: &LauncherOptions,
    children: &mut ManagedChildren,
    event_tx: &Sender<RuntimeEvent>,
    app_config: Option<&crate::core_msgs::config::AppConfig>,
    run_paths: &RunPaths,
) -> io::Result<()> {
    let configured_agents = app_config.map_or(&[][..], |config| config.agents.as_slice());
    let starts_environment = configured_agents.is_empty()
        || configured_agents
            .iter()
            .any(|agent| agent.manifest.backend.starts_environment());
    if starts_environment {
        children.spawn_background(
            Component::EnvironmentSim,
            event_tx,
            app_config,
            None,
            run_paths,
        )?;
    }
    match app_config {
        Some(cfg) => {
            for agent in &cfg.agents {
                children.spawn_background(
                    Component::Agent,
                    event_tx,
                    app_config,
                    Some(&agent.runtime.agent_id),
                    run_paths,
                )?;
            }
        }
        None => children.spawn_background(Component::Agent, event_tx, None, None, run_paths)?,
    }

    let starts_hardware = configured_agents
        .iter()
        .any(|agent| agent.manifest.backend.requires_platform());
    if starts_hardware {
        children.spawn_background(
            Component::ObjectTracker,
            event_tx,
            app_config,
            None,
            run_paths,
        )?;

        if let Some(cfg) = app_config {
            for agent in &cfg.agents {
                if agent.manifest.backend.requires_platform()
                    && agent.manifest.platform == platform_spot::PROFILE
                {
                    children.spawn_sidecar(
                        options,
                        &agent.runtime.agent_id,
                        event_tx,
                        app_config,
                        run_paths,
                    )?;
                }
            }
        }
    }

    if options.visualizer {
        children.spawn_background(Component::Visualizer, event_tx, app_config, None, run_paths)?;
    }

    if options.mcap_recorder {
        children.spawn_background(
            Component::McapRecorder,
            event_tx,
            app_config,
            None,
            run_paths,
        )?;
    }

    Ok(())
}

fn run_interactive_launcher(options: LauncherOptions) -> io::Result<ExitCode> {
    let run_paths = RunPaths::discover()?;
    run_paths.prepare()?;
    let (event_tx, event_rx) = crossbeam_channel::unbounded();
    operator_ui::run_interactive(options, &event_rx, &event_tx, run_paths)
}

/// Consume an optionally exported Spot password before any threads are spawned.
/// Normal interactive launches obtain the credential through Ratatui when the
/// operator configures a hardware-backed Spot agent; this environment path is
/// retained for automation and preconfigured development shells.
///
/// The `BOSDYN_CLIENT_PASSWORD` variable is removed once captured so the
/// plaintext does not linger in the launcher's environment or get inherited by
/// child processes; sidecar children have it re-injected explicitly.
fn resolve_spot_password(options: &mut LauncherOptions) {
    let Ok(password) = env::var("BOSDYN_CLIENT_PASSWORD") else {
        return;
    };

    let raw = Zeroizing::new(password);
    if !raw.is_empty() {
        options.bosdyn_password = Some(SecretString::from(raw.as_str()));
    }

    // SAFETY: main calls this before run_interactive_launcher starts any
    // threads, so there is no concurrent environment access.
    unsafe { env::remove_var("BOSDYN_CLIENT_PASSWORD") };
}

fn run_component(component: Component, args: Vec<OsString>) -> ExitCode {
    match component {
        Component::EnvironmentSim => {
            crate::environment_sim::run();
            ExitCode::SUCCESS
        }
        Component::Agent => {
            crate::agent::run();
            ExitCode::SUCCESS
        }
        Component::Visualizer => {
            crate::visualizer::run();
            ExitCode::SUCCESS
        }
        Component::McapRecorder => match crate::mcap_recorder::run_with_args(args) {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!("mcap_recorder: {error}");
                ExitCode::from(2)
            }
        },
        Component::MocapZenohBridge => match crate::mocap_zenoh_bridge::run_with_args(args) {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!("mocap_zenoh_bridge: {error}");
                ExitCode::from(1)
            }
        },
        Component::ObjectTracker => {
            crate::object_tracker::run_with_args(args);
            ExitCode::SUCCESS
        }
        Component::SpotSidecar => run_sidecar(args),
    }
}

#[derive(Default)]
struct ManagedChildren {
    children: Vec<ManagedChild>,
}

impl ManagedChildren {
    fn spawn_sidecar(
        &mut self,
        options: &LauncherOptions,
        agent_id: &str,
        event_tx: &Sender<RuntimeEvent>,
        app_config: Option<&crate::core_msgs::config::AppConfig>,
        run_paths: &RunPaths,
    ) -> io::Result<()> {
        let mut command = Command::new(env::current_exe()?);
        command
            .arg(COMPONENT_ARG)
            .arg(Component::SpotSidecar.as_str());
        command.arg(&options.sidecar_ip);
        command.arg("--agent-id").arg(agent_id);
        if options.sidecar_auto.power_on {
            command.arg("--auto-power-on");
        }
        if options.sidecar_auto.stand {
            command.arg("--auto-stand");
        }
        if options.sidecar_auto.calibrate {
            command.arg("--auto-calibrate");
        }
        command
            .arg("--mocap-body-id")
            .arg(options.sidecar_mocap_body_id.to_string());
        command.arg("--mocap-body-to-spot-body").args(
            options
                .sidecar_mocap_to_body_translation_m
                .map(|value| value.to_string()),
        );
        if sidecar_owns_virtual_object_state(app_config, agent_id) {
            command.arg("--owns-object-state");
        }

        if let Some(password) = &options.bosdyn_password {
            // The child needs the value in the clear in its own env block; the
            // launcher-side copy stays wrapped and is zeroized when options drop.
            command.env("BOSDYN_CLIENT_PASSWORD", password.expose_secret());
        }
        if let Some(json) = app_config.and_then(|cfg| serde_json::to_string(cfg).ok()) {
            command.env("DCA_APP_CONFIG_JSON", json);
        }
        command.env(SIDECAR_LOG_DIR_ENV, run_paths.log_directory());
        command.env(RUN_ID_ENV, run_paths.run_id());

        command
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());

        let mut child = command.spawn()?;
        let pid = child.id();

        pipe_child_output(Component::SpotSidecar, &mut child, event_tx);
        let _ = event_tx.send(RuntimeEvent::new(
            EventKind::Component,
            Component::SpotSidecar.as_str(),
            format!("started sidecar pid {pid} for {agent_id}"),
        ));

        self.children.push(ManagedChild {
            component: Component::SpotSidecar,
            child,
            exit_status: None,
        });

        Ok(())
    }

    fn spawn_background(
        &mut self,
        component: Component,
        event_tx: &Sender<RuntimeEvent>,
        app_config: Option<&crate::core_msgs::config::AppConfig>,
        agent_id: Option<&str>,
        run_paths: &RunPaths,
    ) -> io::Result<()> {
        let mut child = spawn_component_process(component, app_config, agent_id, run_paths)?;
        let pid = child.id();

        pipe_child_output(component, &mut child, event_tx);
        let _ = event_tx.send(RuntimeEvent::new(
            EventKind::Component,
            component.as_str(),
            format!("started pid {pid}"),
        ));

        self.children.push(ManagedChild {
            component,
            child,
            exit_status: None,
        });

        Ok(())
    }

    fn refresh_statuses(&mut self, event_tx: &Sender<RuntimeEvent>) -> io::Result<()> {
        for managed in &mut self.children {
            if managed.exit_status.is_some() {
                continue;
            }

            if let Some(status) = managed.child.try_wait()? {
                managed.exit_status = Some(status);
                let kind = if status.success() {
                    EventKind::Component
                } else {
                    EventKind::Error
                };
                let _ = event_tx.send(RuntimeEvent::new(
                    kind,
                    managed.component.as_str(),
                    format!("exited with {}", status_label(status)),
                ));
            }
        }

        Ok(())
    }

    fn startup_failure(&self) -> Option<(Component, String)> {
        self.children.iter().find_map(|managed| {
            let status = managed
                .exit_status
                .filter(|_| managed.component.is_required_runtime())?;
            Some((
                managed.component,
                format!(
                    "{} exited unexpectedly with {}",
                    managed.component.as_str(),
                    status_label(status)
                ),
            ))
        })
    }

    /// Stops every still-running child using its platform-appropriate graceful
    /// request, then hard-kills any process that exceeds its shutdown grace.
    fn shutdown(&mut self, event_tx: &Sender<RuntimeEvent>) {
        for managed in &mut self.children {
            if managed.exit_status.is_some() {
                continue;
            }

            if let Ok(Some(status)) = managed.child.try_wait() {
                managed.exit_status = Some(status);
                continue;
            }

            let _ = event_tx.send(RuntimeEvent::new(
                EventKind::Component,
                managed.component.as_str(),
                "shutdown requested".to_owned(),
            ));
            let child = &mut managed.child;
            if managed.component == Component::SpotSidecar {
                if child.stdin.take().is_none() {
                    request_process_shutdown(child);
                }
            } else {
                request_process_shutdown(child);
            }

            let deadline = Instant::now() + managed.component.shutdown_grace();
            while Instant::now() < deadline {
                if let Ok(Some(status)) = managed.child.try_wait() {
                    managed.exit_status = Some(status);
                    break;
                }
                thread::sleep(CHILD_SHUTDOWN_POLL);
            }

            if managed.exit_status.is_none() {
                let _ = managed.child.kill();
                if let Ok(status) = managed.child.wait() {
                    managed.exit_status = Some(status);
                }
            }
        }
    }
}

fn sidecar_owns_virtual_object_state(
    app_config: Option<&crate::core_msgs::config::AppConfig>,
    agent_id: &str,
) -> bool {
    app_config.is_some_and(|config| {
        config.agents.iter().any(|agent| {
            agent.runtime.agent_id == agent_id
                && agent.manifest.backend == crate::core_msgs::config::AgentBackend::Hardware
        })
    })
}

impl Drop for ManagedChildren {
    fn drop(&mut self) {
        for managed in &mut self.children {
            if managed.exit_status.is_some() {
                continue;
            }

            if let Ok(Some(status)) = managed.child.try_wait() {
                managed.exit_status = Some(status);
                continue;
            }

            let _ = managed.child.kill();
            let _ = managed.child.wait();
        }
    }
}

struct ManagedChild {
    component: Component,
    child: Child,
    exit_status: Option<ExitStatus>,
}

fn spawn_component_process(
    component: Component,
    app_config: Option<&crate::core_msgs::config::AppConfig>,
    agent_id: Option<&str>,
    run_paths: &RunPaths,
) -> io::Result<Child> {
    let mut command = Command::new(env::current_exe()?);
    command.arg(COMPONENT_ARG).arg(component.as_str());
    command.env(SIDECAR_LOG_DIR_ENV, run_paths.log_directory());
    command.env(RUN_ID_ENV, run_paths.run_id());
    if component == Component::McapRecorder {
        command.arg("--out").arg(run_paths.mcap_path());
    }
    if let Some(json) = app_config.and_then(|cfg| serde_json::to_string(cfg).ok()) {
        command.env("DCA_APP_CONFIG_JSON", json);
    }
    if let Some(agent_id) = agent_id {
        command.env("DCA_AGENT_ID", agent_id);
    }
    if component == Component::Agent {
        let profile = configured_platform_profile(app_config, agent_id)?;
        let capabilities = platform_profiles::capabilities_for(profile).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("unknown platform profile '{profile}'"),
            )
        })?;
        let json = serde_json::to_string(&capabilities).map_err(io::Error::other)?;
        command.env(core_msgs::AGENT_CAPABILITIES_ENV, json);
    }
    // Spot credentials go to sidecar children only (spawn_sidecar sets them
    // explicitly); every other child must not inherit an exported value.
    command.env_remove("BOSDYN_CLIENT_USERNAME");
    command.env_remove("BOSDYN_CLIENT_PASSWORD");
    command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    command.spawn()
}

fn configured_platform_profile<'a>(
    app_config: Option<&'a core_msgs::config::AppConfig>,
    agent_id: Option<&str>,
) -> io::Result<&'a str> {
    let Some(app_config) = app_config else {
        return Ok(platform_spot::PROFILE);
    };

    let agent = match agent_id {
        Some(agent_id) => app_config
            .agents
            .iter()
            .find(|agent| agent.runtime.agent_id == agent_id)
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("agent '{agent_id}' is not present in the app config"),
                )
            })?,
        None => app_config.agents.first().ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "app config contains no agents")
        })?,
    };

    Ok(agent.manifest.platform.as_str())
}

fn pipe_child_output(component: Component, child: &mut Child, event_tx: &Sender<RuntimeEvent>) {
    if let Some(stdout) = child.stdout.take() {
        spawn_output_reader(component, "stdout", stdout, event_tx.clone());
    }

    if let Some(stderr) = child.stderr.take() {
        spawn_output_reader(component, "stderr", stderr, event_tx.clone());
    }
}

fn spawn_output_reader<R>(
    component: Component,
    stream: &'static str,
    reader: R,
    event_tx: Sender<RuntimeEvent>,
) where
    R: Read + Send + 'static,
{
    thread::spawn(move || {
        let source = format!("{}:{stream}", component.as_str());
        for line in BufReader::new(reader).lines() {
            let Ok(line) = line else {
                break;
            };
            let kind = child_output_event_kind(stream, &line);
            let _ = event_tx.send(RuntimeEvent::new(kind, &source, line));
        }
    });
}

fn child_output_event_kind(stream: &str, line: &str) -> EventKind {
    structured_output_event_kind(line).unwrap_or(if stream == "stderr" {
        EventKind::Error
    } else {
        EventKind::Info
    })
}

fn structured_output_event_kind(line: &str) -> Option<EventKind> {
    let visible = text_without_ansi_csi(line);
    let mut fields = visible.split_ascii_whitespace();
    let first = fields.next()?;

    if let Some(kind) = event_kind_for_level(first) {
        return Some(kind);
    }
    if !(first.contains('T') && first.contains('-') && first.contains(':')) {
        return None;
    }

    fields.next().and_then(event_kind_for_level)
}

fn event_kind_for_level(level: &str) -> Option<EventKind> {
    match level {
        "TRACE" | "DEBUG" | "INFO" => Some(EventKind::Info),
        "WARN" | "WARNING" => Some(EventKind::Warning),
        "ERROR" | "FATAL" => Some(EventKind::Error),
        _ => None,
    }
}

fn text_without_ansi_csi(text: &str) -> String {
    let mut visible = String::with_capacity(text.len());
    let mut characters = text.chars().peekable();

    while let Some(character) = characters.next() {
        if character != '\u{1b}' || characters.peek() != Some(&'[') {
            visible.push(character);
            continue;
        }

        characters.next();
        for control_character in characters.by_ref() {
            if ('@'..='~').contains(&control_character) {
                break;
            }
        }
    }

    visible
}

#[cfg(unix)]
fn request_process_shutdown(child: &Child) {
    let _ = Command::new("kill")
        .arg("-TERM")
        .arg(child.id().to_string())
        .status();
}

#[cfg(not(unix))]
fn request_process_shutdown(child: &mut Child) {
    let _ = child.kill();
}

fn status_label(status: ExitStatus) -> String {
    status
        .code()
        .map_or_else(|| "signal".to_owned(), |code| format!("code {code}"))
}

fn usage() -> String {
    "usage: dca\n\n\
     Starts the unified Ratatui operator interface.\n\
     Opens an interactive setup screen to configure agents and observers.\n\
     Commands: `g X Y`, `m X Y`, `h`, `p [OBJECT]`, `d [OBJECT]`, and `r`.\n\
     Prefix a command with an agent ID or press Tab to select another agent.\n\
     Press Enter to send a command; press q, Esc, or Ctrl-C to shut down."
        .to_owned()
}

#[cfg(test)]
#[path = "../../../tests/unit/launcher.rs"]
mod tests;
