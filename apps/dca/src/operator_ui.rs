//! Terminal operator UI for launching commands and watching runtime feedback.
//!
//! This Ratatui screen is the human-facing control surface for the stack. It
//! reads operator commands, publishes behavior requests, watches the Zenoh bus
//! for lifecycle status, safety constraints, vetoes, and robot position, then
//! renders that information in plain language for the selected agent. When a
//! hardware or hybrid backend is configured it also owns the MOCAP bridge
//! process and shows that bridge's latest output line.

use super::{LauncherOptions, ManagedChildren, runtime_paths::RunPaths};
use crate::{
    core_msgs::{
        AgentStateMsg, BehaviorPhase, BehaviorStatus, CapabilityReport, CapabilityRequirement,
        CommandStatus, CommandVeto, ConstraintEvent, ManipulationStatus, ObjectState,
        PlatformCommand, ResumeAutonomy, SpeedPreset,
        catalog::ObjectCatalog,
        command_status_aborted, command_status_name, command_status_succeeded, command_type_hold,
        command_type_stop, constraint_type_goal_blocked, constraint_type_name,
        constraint_type_robot_fault, constraint_type_virtual_wall, constraint_type_wall,
        constraints::{ActiveConstraints, ApplyOutcome},
        frames::{FramesConfig, RigidTransform},
        manipulation_failure_reason_name, manipulation_failure_reason_no_object_detected,
        manipulation_failure_reason_unsafe_grip_height, manipulation_failure_reason_unspecified,
        manipulation_phase_failed, manipulation_phase_name, now_ms, object_lifecycle_free,
        object_lifecycle_held, object_lifecycle_name, source_environment_runtime,
        source_spot_adapter, state_authority_environment_runtime, state_authority_real,
        topic_behavior_request, topic_resume_autonomy, veto_reason_name, veto_reason_none,
    },
    dds_support::{drain_latest_per_agent_with_status, send_or_log_drop},
    operator_bus::{command, dds::OperatorBusIo},
};
use chrono::{DateTime, Local, TimeZone};
use crossbeam_channel::{Receiver, Sender};
use crossterm::{
    cursor::Show,
    event::{
        self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEvent, KeyEventKind,
        KeyModifiers, MouseButton, MouseEvent, MouseEventKind,
    },
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use ratatui::{
    Frame, Terminal,
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{
        Block, Borders, Cell, Clear, Paragraph, Row, Scrollbar, ScrollbarOrientation,
        ScrollbarState, Table, Wrap,
    },
};
use secrecy::{SecretString, zeroize::Zeroizing};
use std::{
    collections::VecDeque,
    env,
    fs::File,
    io::{self, BufRead, BufReader, BufWriter, Read, Stdout, Write},
    path::Path,
    process::{Child, Command, ExitCode, Stdio},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

#[derive(Clone)]
pub enum Screen {
    Setup {
        pane: SetupPane,
        tab_cursor: usize,
        field_cursor: usize,
    },
    /// Modal password gate for hardware-backed Spot agents.
    SpotPassword {
        return_pane: SetupPane,
        return_tab_cursor: usize,
        return_field_cursor: usize,
    },
    Connecting {
        started: Instant,
        mocap_ok: bool,
        spot_ok: bool,
        failed: bool,
    },
    Operator,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SetupPane {
    Tabs,
    Fields,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SetupTab {
    Global,
    Agent(usize),
    VirtualObject(usize),
    MocapObject(usize),
    AddAgent,
    AddVirtualObject,
    AddMocapObject,
    SpotSidecar,
    Start,
}

pub const CONNECT_TIMEOUT: Duration = Duration::from_mins(1);

#[derive(Debug, PartialEq, Eq)]
pub enum ConnectPhase {
    Waiting,
    Connected,
    Failed,
}

pub fn connect_phase(
    started: Instant,
    mocap_ok: bool,
    spot_ok: bool,
    now: Instant,
) -> ConnectPhase {
    if mocap_ok && spot_ok {
        ConnectPhase::Connected
    } else if now.saturating_duration_since(started) >= CONNECT_TIMEOUT {
        ConnectPhase::Failed
    } else {
        ConnectPhase::Waiting
    }
}

const MAX_DIAGNOSTIC_EVENTS: usize = 50;
const EVENT_LOG_SCROLL_STEP: usize = 6;
const EVENT_LOG_WHEEL_STEP: usize = 3;
const MAX_COMMAND_HISTORY: usize = 2;
const DIAGNOSTIC_VISIBLE_DURATION: Duration = Duration::from_secs(5);
const UI_TICK: Duration = Duration::from_millis(150);
const CURRENT_POSITION_GOAL: &str = "the current position";
const MOCAP_BRIDGE_COMPONENT: &str = "mocap_zenoh_bridge";

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
    Info,
    Component,
    Command,
    Warning,
    Error,
}

#[derive(Clone)]
pub struct RuntimeEvent {
    kind: EventKind,
    source: String,
    message: String,
    created_at: Instant,
    created_wall_time: DateTime<Local>,
    command_epoch: Option<u64>,
}

impl RuntimeEvent {
    pub(super) fn new(
        kind: EventKind,
        source: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self {
            kind,
            source: source.into(),
            message: message.into(),
            created_at: Instant::now(),
            created_wall_time: Local::now(),
            command_epoch: None,
        }
    }

    fn with_stamp_ms(mut self, stamp_ms: u64) -> Self {
        self.created_wall_time = wall_time_from_stamp_ms(stamp_ms);
        self
    }

    const fn with_command_epoch(mut self, command_epoch: u64) -> Self {
        self.command_epoch = Some(command_epoch);
        self
    }

    fn same_content(&self, other: &Self) -> bool {
        self.kind == other.kind
            && self.source == other.source
            && self.message == other.message
            && self.command_epoch == other.command_epoch
    }
}

struct OperatorActivityLog {
    writer: Option<BufWriter<File>>,
}

impl OperatorActivityLog {
    fn create(path: &Path) -> io::Result<Self> {
        let mut writer = BufWriter::new(File::create(path)?);
        writer.write_all(b"# Operator UI Activity Log\n\n```\n")?;
        writer.flush()?;
        Ok(Self {
            writer: Some(writer),
        })
    }

    fn record(&mut self, event: &RuntimeEvent) -> io::Result<()> {
        let Some(writer) = self.writer.as_mut() else {
            return Ok(());
        };
        let timestamp = event.created_wall_time.format("%Y-%m-%d %H:%M:%S%.3f");
        let severity = match event.kind {
            EventKind::Warning => "WARN",
            EventKind::Error => "ERROR",
            EventKind::Info | EventKind::Component | EventKind::Command => "INFO",
        };
        let result = writeln!(writer, "[{}] [{}] {}", timestamp, severity, event.message)
            .and_then(|()| writer.flush());
        if result.is_err() {
            self.writer = None;
        }
        result
    }

    fn finish(&mut self) -> io::Result<()> {
        let Some(mut writer) = self.writer.take() else {
            return Ok(());
        };
        writer.write_all(b"```\n")?;
        writer.flush()
    }
}

pub fn run_interactive(
    mut options: LauncherOptions,
    event_rx: &Receiver<RuntimeEvent>,
    event_tx: &Sender<RuntimeEvent>,
    run_paths: RunPaths,
) -> io::Result<ExitCode> {
    options.visualizer = true;
    let app_config = crate::core_msgs::config::load_app_config();
    let frames = FramesConfig::from_config(&app_config.frames);
    let t_bevy_map = frames.t_map_bevy.inverse();
    let catalog = ObjectCatalog::from_config(app_config.objects.clone());
    let operator_io = crate::operator_bus::dds::spawn_operator_bus_io();
    let shutdown_requested = install_shutdown_handler()?;
    let mut terminal = TerminalSession::start()?;
    let children = ManagedChildren::default();

    let app = OperatorApp::new(
        options,
        operator_io,
        t_bevy_map,
        catalog,
        app_config,
        run_paths,
        Screen::Setup {
            pane: SetupPane::Tabs,
            tab_cursor: 0,
            field_cursor: 0,
        },
    )?;

    run_event_loop(
        app,
        children,
        event_rx,
        event_tx,
        &shutdown_requested,
        &mut terminal,
    )
}

fn run_event_loop(
    mut app: OperatorApp,
    mut children: ManagedChildren,
    event_rx: &Receiver<RuntimeEvent>,
    event_tx: &Sender<RuntimeEvent>,
    shutdown_requested: &Arc<AtomicBool>,
    terminal: &mut TerminalSession,
) -> io::Result<ExitCode> {
    loop {
        children.refresh_statuses(event_tx)?;
        app.drain_runtime_events(event_rx);
        app.drain_mocap_bridge();
        app.drain_bus();

        if app.connecting_failure.is_none()
            && let Some((component, fallback_reason)) = children.startup_failure()
            && matches!(app.screen, Screen::Connecting { .. })
        {
            let reason = app.startup_failure_reason(component, fallback_reason);
            app.push_event(RuntimeEvent::new(
                EventKind::Error,
                "launcher",
                reason.clone(),
            ));
            app.connecting_failure = Some(reason);
        }

        if let Screen::Connecting {
            started,
            mocap_ok,
            spot_ok,
            failed,
        } = app.screen
        {
            if app.connecting_failure.is_some() {
                app.screen = Screen::Connecting {
                    started,
                    mocap_ok,
                    spot_ok,
                    failed: true,
                };
            } else if !failed {
                match connect_phase(started, mocap_ok, spot_ok, Instant::now()) {
                    ConnectPhase::Connected => app.screen = Screen::Operator,
                    ConnectPhase::Failed => {
                        app.screen = Screen::Connecting {
                            started,
                            mocap_ok,
                            spot_ok,
                            failed: true,
                        };
                    }
                    ConnectPhase::Waiting => {}
                }
            }
        }

        terminal.terminal.draw(|frame| app.draw(frame, &children))?;

        if shutdown_requested.load(Ordering::SeqCst) {
            app.request_shutdown(event_tx, "shutdown requested from signal");
        }

        if app.should_quit {
            break;
        }

        if event::poll(UI_TICK)? {
            match event::read()? {
                Event::Key(key) => app.handle_key(key, event_tx, &mut children),
                Event::Mouse(mouse) => app.handle_mouse(mouse, event_tx, &mut children),
                _ => {}
            }
        }
    }

    children.shutdown(event_tx);
    app.drain_runtime_events(event_rx);
    app.activity_log.finish()?;
    Ok(ExitCode::SUCCESS)
}

fn install_shutdown_handler() -> io::Result<Arc<AtomicBool>> {
    let shutdown_requested = Arc::new(AtomicBool::new(false));
    let handler_flag = Arc::clone(&shutdown_requested);
    ctrlc::set_handler(move || {
        handler_flag.store(true, Ordering::SeqCst);
    })
    .map_err(io::Error::other)?;

    Ok(shutdown_requested)
}

struct TerminalSession {
    terminal: Terminal<CrosstermBackend<Stdout>>,
}

impl TerminalSession {
    fn start() -> io::Result<Self> {
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        if let Err(error) = execute!(stdout, EnterAlternateScreen, EnableMouseCapture) {
            restore_terminal_output(&mut stdout);
            return Err(error);
        }
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = match Terminal::new(backend) {
            Ok(terminal) => terminal,
            Err(error) => {
                restore_terminal_output(&mut io::stdout());
                return Err(error);
            }
        };
        if let Err(error) = terminal.clear() {
            restore_terminal_output(terminal.backend_mut());
            return Err(error);
        }

        Ok(Self { terminal })
    }
}

impl Drop for TerminalSession {
    fn drop(&mut self) {
        restore_terminal_output(self.terminal.backend_mut());
    }
}

fn restore_terminal_output(output: &mut impl Write) {
    let _ = execute!(output, DisableMouseCapture);
    let _ = output.flush();
    drain_pending_terminal_events();
    let _ = execute!(output, LeaveAlternateScreen, Show);
    let _ = output.flush();
    let _ = disable_raw_mode();
}

fn drain_pending_terminal_events() {
    const MAX_PENDING_EVENTS: usize = 256;

    for _ in 0..MAX_PENDING_EVENTS {
        match event::poll(Duration::ZERO) {
            Ok(true) => {
                if event::read().is_err() {
                    break;
                }
            }
            Ok(false) | Err(_) => break,
        }
    }
}

struct MocapBridgeRuntime {
    child: Option<Child>,
    line_rx: Receiver<MocapBridgeUpdate>,
    latest_line: String,
    failure_summary: Option<String>,
    reported_failure: Option<String>,
}

enum MocapBridgeUpdate {
    Line(String),
}

impl MocapBridgeRuntime {
    fn start(options: &LauncherOptions) -> Self {
        let (line_tx, line_rx) = crossbeam_channel::unbounded();
        let mut command = match env::current_exe() {
            Ok(executable) => Command::new(executable),
            Err(error) => {
                return Self {
                    child: None,
                    line_rx,
                    latest_line: format!("failed to locate mocap bridge executable: {error}"),
                    failure_summary: Some("failed to locate mocap bridge executable".to_owned()),
                    reported_failure: None,
                };
            }
        };
        command
            .arg("--component")
            .arg(MOCAP_BRIDGE_COMPONENT)
            .arg("--mocap-ip")
            .arg(&options.mocap_ip)
            .arg("--mocap-port")
            .arg(options.mocap_port.to_string());

        if options.mocap_wsl_relay {
            command.arg("--mocap-wsl-relay");
        }

        command
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());

        match command.spawn() {
            Ok(mut child) => {
                if let Some(stdout) = child.stdout.take() {
                    spawn_mocap_output_reader(stdout, line_tx.clone());
                }
                if let Some(stderr) = child.stderr.take() {
                    spawn_mocap_output_reader(stderr, line_tx);
                }

                Self {
                    child: Some(child),
                    line_rx,
                    latest_line: "waiting for mocap bridge output...".to_owned(),
                    failure_summary: None,
                    reported_failure: None,
                }
            }
            Err(error) => {
                let summary = "failed to start mocap bridge".to_owned();
                Self {
                    child: None,
                    line_rx,
                    latest_line: format!("{summary}: {error}"),
                    failure_summary: Some(summary),
                    reported_failure: None,
                }
            }
        }
    }

    fn drain(&mut self) {
        while let Ok(update) = self.line_rx.try_recv() {
            match update {
                MocapBridgeUpdate::Line(line) => {
                    self.latest_line = self
                        .failure_summary
                        .as_deref()
                        .map_or_else(|| line.clone(), |summary| format!("{summary}: {line}"));
                }
            }
        }

        let status = self
            .child
            .as_mut()
            .and_then(|child| match child.try_wait() {
                Ok(status) => status,
                Err(error) => {
                    let summary = "mocap bridge status check failed".to_owned();
                    self.latest_line = format!("{summary}: {error}");
                    self.failure_summary = Some(summary);
                    None
                }
            });

        if let Some(status) = status {
            let summary = format!("mocap bridge exited with {}", super::status_label(status));
            self.latest_line = mocap_bridge_failure_detail(&summary, &self.latest_line);
            self.failure_summary = Some(summary);
            self.child = None;
        }
    }

    fn latest_line(&self) -> &str {
        &self.latest_line
    }

    const fn failed(&self) -> bool {
        self.failure_summary.is_some()
    }

    fn take_unreported_failure(&mut self) -> Option<String> {
        if !self.failed() || self.reported_failure.as_deref() == Some(self.latest_line.as_str()) {
            return None;
        }
        self.reported_failure = Some(self.latest_line.clone());
        self.reported_failure.clone()
    }
}

fn mocap_bridge_failure_detail(summary: &str, latest_line: &str) -> String {
    if latest_line == "waiting for mocap bridge output..." {
        summary.to_owned()
    } else {
        format!("{summary}: {latest_line}")
    }
}

impl Drop for MocapBridgeRuntime {
    fn drop(&mut self) {
        let Some(child) = &mut self.child else {
            return;
        };

        if child.try_wait().ok().flatten().is_some() {
            return;
        }

        super::request_process_shutdown(child);
        let deadline = Instant::now() + super::CHILD_SHUTDOWN_GRACE;
        while Instant::now() < deadline {
            if child.try_wait().ok().flatten().is_some() {
                return;
            }
            thread::sleep(super::CHILD_SHUTDOWN_POLL);
        }

        let _ = child.kill();
        let _ = child.wait();
    }
}

fn spawn_mocap_output_reader<R>(reader: R, line_tx: Sender<MocapBridgeUpdate>)
where
    R: Read + Send + 'static,
{
    thread::spawn(move || {
        for line in BufReader::new(reader).lines() {
            let Ok(line) = line else {
                break;
            };
            let line = line.trim().to_owned();
            if !line.is_empty() {
                let _ = line_tx.send(MocapBridgeUpdate::Line(line));
            }
        }
    });
}

struct ActiveCommandContext {
    epoch: u64,
    stamp_ms: u64,
    ids: Vec<String>,
    goals: Vec<(String, String)>,
}

struct OperatorApp {
    focused_agent_id: String,
    options: LauncherOptions,
    operator_io: OperatorBusIo,
    input: String,
    previous_commands: VecDeque<String>,
    events: VecDeque<RuntimeEvent>,
    diagnostics: VecDeque<RuntimeEvent>,
    mocap_bridge: Option<MocapBridgeRuntime>,
    event_log_scroll: usize,
    event_log_area: Option<Rect>,
    event_log_scrollbar: Option<EventLogScrollbar>,
    event_log_dragging_scrollbar: bool,
    next_command_epoch: u64,
    active_command_context: Option<ActiveCommandContext>,
    latest_state: std::collections::HashMap<String, AgentStateMsg>,
    t_bevy_map: RigidTransform,
    active_constraints: ActiveConstraints,
    latest_veto: Option<CommandVeto>,
    last_platform_log: Option<(u8, String)>,
    should_quit: bool,
    dds_disconnected: bool,
    held_object_id: std::collections::HashMap<String, String>,
    object_states: std::collections::HashMap<String, ObjectState>,
    catalog: ObjectCatalog,
    capability_reports: std::collections::HashMap<String, CapabilityReport>,
    behavior_statuses: std::collections::HashMap<String, BehaviorStatus>,
    autonomy_paused: std::collections::HashSet<String>,
    /// Plaintext Spot password captured keystroke-by-keystroke on the
    /// `SpotPassword` screen. `Zeroizing` wipes the buffer on drop and on every
    /// `clear()`, so the secret never lingers in the host heap after it is
    /// promoted into the `SecretString` in `options.bosdyn_password`.
    spot_password_input: Zeroizing<String>,
    spot_password_error: Option<String>,
    spot_runtime_status: Option<String>,
    latest_spot_sidecar_error: Option<String>,
    spot_sidecar_error_priority: u8,
    connecting_failure: Option<String>,

    pub(crate) screen: Screen,
    pub(crate) retry_rect: Option<Rect>,
    pub(crate) app_config: crate::core_msgs::config::AppConfig,
    run_paths: RunPaths,

    running_components: std::collections::HashSet<super::Component>,
    activity_log: OperatorActivityLog,
}

impl OperatorApp {
    fn new(
        options: LauncherOptions,
        operator_io: OperatorBusIo,
        t_bevy_map: RigidTransform,
        catalog: ObjectCatalog,
        app_config: crate::core_msgs::config::AppConfig,
        run_paths: RunPaths,
        screen: Screen,
    ) -> io::Result<Self> {
        let activity_log = OperatorActivityLog::create(&run_paths.operator_log_path())?;
        let focused_agent_id = app_config
            .agents
            .first()
            .map_or_else(String::new, |agent| agent.runtime.agent_id.clone());
        let mut app = Self {
            focused_agent_id,
            options,
            operator_io,
            input: String::new(),
            previous_commands: VecDeque::new(),
            events: VecDeque::new(),
            diagnostics: VecDeque::new(),
            mocap_bridge: None,
            event_log_scroll: 0,
            event_log_area: None,
            event_log_scrollbar: None,
            event_log_dragging_scrollbar: false,
            next_command_epoch: 1,
            active_command_context: None,
            latest_state: std::collections::HashMap::new(),
            t_bevy_map,
            active_constraints: ActiveConstraints::default(),
            latest_veto: None,
            last_platform_log: None,
            should_quit: false,
            dds_disconnected: false,
            held_object_id: std::collections::HashMap::new(),
            object_states: std::collections::HashMap::new(),
            catalog,
            capability_reports: std::collections::HashMap::new(),
            behavior_statuses: std::collections::HashMap::new(),
            autonomy_paused: std::collections::HashSet::new(),
            spot_password_input: Zeroizing::new(String::new()),
            spot_password_error: None,
            spot_runtime_status: None,
            latest_spot_sidecar_error: None,
            spot_sidecar_error_priority: 0,
            connecting_failure: None,
            screen,
            retry_rect: None,
            app_config,
            run_paths,
            running_components: std::collections::HashSet::new(),
            activity_log,
        };

        app.push_event(RuntimeEvent::new(
            EventKind::Info,
            "operator",
            "Commands: g X Y: navigate. m X Y: move away. h: hold. p: pick. d: place. r: resume autonomy.",
        ));
        Ok(app)
    }

    fn start_runtime(&mut self, children: &mut ManagedChildren, event_tx: &Sender<RuntimeEvent>) {
        if has_spot_hardware_agents(&self.app_config) && self.options.bosdyn_password.is_none() {
            let start_tab_cursor = self.setup_tabs().len().saturating_sub(1);
            self.open_spot_password_prompt(SetupPane::Tabs, start_tab_cursor, 0);
            return;
        }

        if mocap_bridge_enabled(&self.app_config) {
            self.mocap_bridge = Some(MocapBridgeRuntime::start(&self.options));
        }
        self.spot_runtime_status = None;
        self.latest_spot_sidecar_error = None;
        self.spot_sidecar_error_priority = 0;
        self.connecting_failure = None;
        if let Err(error) = super::spawn_runtime_children(
            &self.options,
            children,
            event_tx,
            Some(&self.app_config),
            &self.run_paths,
        ) {
            let reason = format!("failed to launch runtime components: {error}");
            self.push_event(RuntimeEvent::new(
                EventKind::Error,
                "launcher",
                reason.clone(),
            ));
            self.connecting_failure = Some(reason);
            self.screen = Screen::Connecting {
                started: Instant::now(),
                mocap_ok: false,
                spot_ok: false,
                failed: true,
            };
            return;
        }
        self.screen = if mocap_bridge_enabled(&self.app_config) {
            Screen::Connecting {
                started: Instant::now(),
                mocap_ok: false,
                spot_ok: false,
                failed: false,
            }
        } else {
            Screen::Operator
        };
    }

    fn drain_runtime_events(&mut self, event_rx: &Receiver<RuntimeEvent>) {
        while let Ok(event) = event_rx.try_recv() {
            self.push_event(event);
        }
    }

    fn drain_mocap_bridge(&mut self) {
        let failure = self.mocap_bridge.as_mut().and_then(|bridge| {
            bridge.drain();
            bridge.take_unreported_failure()
        });
        let Some(failure) = failure else {
            return;
        };

        let reason = format!("MOCAP bridge failed: {failure}");
        self.push_event(RuntimeEvent::new(
            EventKind::Error,
            "mocap_bridge",
            reason.clone(),
        ));
        if matches!(self.screen, Screen::Connecting { .. }) {
            self.connecting_failure = Some(reason);
        }
    }

    fn begin_command_scope(&mut self, stamp_ms: u64) -> u64 {
        let command_epoch = self.next_command_epoch;
        self.next_command_epoch = self.next_command_epoch.saturating_add(1);
        self.active_command_context = Some(ActiveCommandContext {
            epoch: command_epoch,
            stamp_ms,
            ids: Vec::new(),
            goals: Vec::new(),
        });
        self.event_log_scroll = 0;
        self.last_platform_log = None;
        command_epoch
    }

    fn register_latest_command_id(&mut self, command_id: impl Into<String>) {
        let command_id = command_id.into();
        if command_id.is_empty() {
            return;
        }
        if let Some(ctx) = &mut self.active_command_context
            && !ctx.ids.iter().any(|id| id == &command_id)
        {
            ctx.ids.push(command_id);
        }
    }

    fn register_latest_command_goal(&mut self, command_id: &str, goal: String) {
        self.register_latest_command_id(command_id.to_owned());
        if let Some(ctx) = &mut self.active_command_context {
            if let Some((_, existing_goal)) = ctx.goals.iter_mut().find(|(id, _)| id == command_id)
            {
                *existing_goal = goal;
            } else {
                ctx.goals.push((command_id.to_owned(), goal));
            }
        }
    }

    fn latest_command_goal_for_id(&self, command_id: &str) -> Option<&str> {
        self.active_command_context.as_ref().and_then(|ctx| {
            ctx.goals
                .iter()
                .find_map(|(id, goal)| (id == command_id).then_some(goal.as_str()))
        })
    }

    fn latest_command_epoch_for_id(&self, command_id: &str) -> Option<u64> {
        if command_id.is_empty() {
            return None;
        }
        self.active_command_context.as_ref().and_then(|ctx| {
            ctx.ids
                .iter()
                .any(|id| id == command_id)
                .then_some(ctx.epoch)
        })
    }

    const fn latest_command_epoch_for_stamp(&self, stamp_ms: u64) -> Option<u64> {
        if let Some(ctx) = &self.active_command_context
            && stamp_ms >= ctx.stamp_ms
        {
            return Some(ctx.epoch);
        }
        None
    }

    fn event_for_latest_command_id(
        &self,
        event: RuntimeEvent,
        command_id: &str,
        stamp_ms: u64,
    ) -> RuntimeEvent {
        Self::with_latest_command_epoch(
            event.with_stamp_ms(stamp_ms),
            self.latest_command_epoch_for_id(command_id),
        )
    }

    fn event_for_latest_command_stamp(&self, event: RuntimeEvent, stamp_ms: u64) -> RuntimeEvent {
        Self::with_latest_command_epoch(
            event.with_stamp_ms(stamp_ms),
            self.latest_command_epoch_for_stamp(stamp_ms),
        )
    }

    fn event_for_latest_command_id_or_stamp(
        &self,
        event: RuntimeEvent,
        command_id: &str,
        stamp_ms: u64,
    ) -> RuntimeEvent {
        let command_epoch = self
            .latest_command_epoch_for_id(command_id)
            .or_else(|| self.latest_command_epoch_for_stamp(stamp_ms));
        Self::with_latest_command_epoch(event.with_stamp_ms(stamp_ms), command_epoch)
    }

    const fn with_latest_command_epoch(
        event: RuntimeEvent,
        command_epoch: Option<u64>,
    ) -> RuntimeEvent {
        if let Some(command_epoch) = command_epoch {
            event.with_command_epoch(command_epoch)
        } else {
            event
        }
    }

    fn command_status_event_message(&self, status: &CommandStatus) -> String {
        let Some(goal) = self.latest_command_goal_for_id(&status.command_id) else {
            return command_status_event_message(status);
        };

        if status.status == command_status_succeeded() {
            if goal == CURRENT_POSITION_GOAL {
                "Holding at the current position.".to_owned()
            } else {
                format!("Reached {goal}.")
            }
        } else if status.status == command_status_aborted() {
            format!("Command failed before reaching {goal}.")
        } else {
            command_status_event_message(status)
        }
    }

    /// Drains every incoming bus channel once per UI tick.
    ///
    /// Beyond forwarding samples, this advances the `Connecting` handshake from
    /// observed traffic rather than any explicit ready signal: the first focused
    /// agent state flips `spot_ok` and the first `mocap` frame flips `mocap_ok`.
    fn drain_bus(&mut self) {
        let (latest_states, state_disconnected) =
            drain_latest_per_agent_with_status(&self.operator_io.state_rx);
        if !latest_states.is_empty() {
            let focused_seen = latest_states.contains_key(&self.focused_agent_id);
            for state in latest_states.values() {
                if state.state_authority == state_authority_environment_runtime() {
                    self.running_components
                        .insert(super::Component::EnvironmentSim);
                } else if state.state_authority == state_authority_real() {
                    self.running_components
                        .insert(super::Component::SpotSidecar);
                }
            }
            self.latest_state.extend(latest_states);

            if focused_seen
                && let Screen::Connecting {
                    ref mut spot_ok, ..
                } = self.screen
            {
                *spot_ok = true;
            }
        }

        if state_disconnected && !self.dds_disconnected {
            self.dds_disconnected = true;
            self.push_event(RuntimeEvent::new(
                EventKind::Error,
                "dds",
                "operator state channel disconnected",
            ));
        }

        while self.operator_io.mocap_rx.try_recv().is_ok() {
            self.running_components
                .insert(super::Component::MocapZenohBridge);
            if let Screen::Connecting {
                ref mut mocap_ok, ..
            } = self.screen
            {
                *mocap_ok = true;
            }
        }

        self.drain_platform_commands();
        self.drain_behavior_statuses();
        self.drain_capability_reports();
        self.drain_command_statuses();
        self.drain_constraints();
        self.drain_vetoes();
        self.drain_manipulation_statuses();
        self.drain_object_states();
        self.expire_constraints();
    }

    /// Drops environment constraints whose TTL backstop has elapsed.
    ///
    /// Environment-sourced conditions are level-published and cleared primarily
    /// by a matching `severity_none` event; if that clear is lost, only the TTL
    /// backstop releases them. The UI must run this every drain or a dropped
    /// clear would pin a safety condition in the display indefinitely. It stays
    /// consistent with the onboard agent because both share the same
    /// `ActiveConstraints` expiry, not because two implementations agree.
    fn expire_constraints(&mut self) {
        for expired in self.active_constraints.expire(now_ms()) {
            let stamp_ms = now_ms();
            let event = self.event_for_latest_command_stamp(
                RuntimeEvent::new(
                    EventKind::Info,
                    "constraint",
                    format!(
                        "Safety condition expired: {}",
                        cleared_constraint_text(&expired)
                    ),
                ),
                stamp_ms,
            );
            self.push_event(event);
        }
    }

    fn drain_platform_commands(&mut self) {
        while let Ok(command) = self.operator_io.command_rx.try_recv() {
            self.running_components.insert(super::Component::Agent);
            if command.agent_id.as_str() == self.focused_agent_id {
                self.handle_platform_command(&command);
            }
        }
    }

    fn is_known_agent(&self, agent_id: &str) -> bool {
        self.app_config
            .agents
            .iter()
            .any(|agent| agent.runtime.agent_id == agent_id)
    }

    fn drain_behavior_statuses(&mut self) {
        while let Ok(status) = self.operator_io.behavior_status_rx.try_recv() {
            self.running_components.insert(super::Component::Agent);
            if !self.is_known_agent(status.agent_id.as_str()) {
                continue;
            }

            let kind = match status.phase {
                BehaviorPhase::Rejected | BehaviorPhase::Failed | BehaviorPhase::Preempted => {
                    EventKind::Warning
                }
                BehaviorPhase::Accepted
                | BehaviorPhase::Active
                | BehaviorPhase::Succeeded
                | BehaviorPhase::Cancelled => EventKind::Command,
            };
            if status.agent_id == self.focused_agent_id {
                let detail = if status.detail.is_empty() {
                    status.reason.map_or_else(
                        || format!("Behavior {:?}.", status.phase),
                        |reason| format!("Behavior {:?}: {reason:?}.", status.phase),
                    )
                } else {
                    format!("Behavior {:?}: {}", status.phase, status.detail)
                };
                let event = self.event_for_latest_command_id(
                    RuntimeEvent::new(kind, "behavior", detail),
                    &status.request_id,
                    status.stamp_ms,
                );
                self.push_event(event);
            }
            self.behavior_statuses
                .insert(status.agent_id.clone(), status);
        }
    }

    fn drain_capability_reports(&mut self) {
        while let Ok(report) = self.operator_io.capability_report_rx.try_recv() {
            if self.is_known_agent(report.agent_id.as_str()) {
                self.running_components.insert(super::Component::Agent);
                if report.connected {
                    match self.backend_for_agent(report.agent_id.as_str()) {
                        Some(crate::core_msgs::config::AgentBackend::Simulated) => {
                            self.running_components
                                .insert(super::Component::EnvironmentSim);
                        }
                        Some(
                            crate::core_msgs::config::AgentBackend::Hardware
                            | crate::core_msgs::config::AgentBackend::Hybrid,
                        ) => {
                            self.running_components
                                .insert(super::Component::SpotSidecar);
                        }
                        None => {}
                    }
                }
                self.capability_reports
                    .insert(report.agent_id.clone(), report);
            }
        }
    }

    fn drain_command_statuses(&mut self) {
        while let Ok(status) = self.operator_io.status_rx.try_recv() {
            if self.is_known_agent(status.agent_id.as_str()) {
                self.handle_command_status(&status);
            }
        }
    }

    fn drain_constraints(&mut self) {
        while let Ok(constraint) = self.operator_io.constraint_rx.try_recv() {
            if constraint.source == source_environment_runtime() {
                self.running_components
                    .insert(super::Component::EnvironmentSim);
            } else if constraint.source == source_spot_adapter() {
                self.running_components
                    .insert(super::Component::SpotSidecar);
            }
            if constraint.agent_id.as_str() == self.focused_agent_id {
                self.handle_constraint(constraint);
            }
        }
    }

    fn drain_vetoes(&mut self) {
        while let Ok(veto) = self.operator_io.veto_rx.try_recv() {
            self.running_components.insert(super::Component::Agent);
            if self.is_known_agent(veto.agent_id.as_str()) {
                self.handle_veto(veto);
            }
        }
    }

    fn drain_manipulation_statuses(&mut self) {
        while let Ok(status) = self.operator_io.manipulation_status_rx.try_recv() {
            if status.agent_id.as_str() == self.focused_agent_id {
                self.handle_manipulation_status(&status);
            }
        }
    }

    fn drain_object_states(&mut self) {
        while let Ok(state) = self.operator_io.object_state_rx.try_recv() {
            if self
                .catalog
                .get(&state.object_id)
                .is_some_and(|obj| obj.virtual_initial_pose_map().is_some())
            {
                self.running_components
                    .insert(super::Component::EnvironmentSim);
            } else {
                self.running_components
                    .insert(super::Component::ObjectTracker);
            }
            self.handle_object_state(state);
        }
    }

    fn handle_platform_command(&mut self, command: &PlatformCommand) {
        let is_hold = command.command_type == command_type_hold();
        let is_stop = command.command_type == command_type_stop();
        let operator_requested_hold = is_hold
            && self
                .latest_command_goal_for_id(&command.command_id)
                .is_some_and(|goal| goal == CURRENT_POSITION_GOAL);

        let episode_key = (
            command.command_type,
            if is_hold || is_stop {
                String::new()
            } else {
                command.command_id.clone()
            },
        );
        if self.last_platform_log.as_ref() == Some(&episode_key) {
            return;
        }
        self.last_platform_log = Some(episode_key);

        let Some((kind, message)) = platform_command_event(command, operator_requested_hold) else {
            return;
        };
        let event = self.event_for_latest_command_id_or_stamp(
            RuntimeEvent::new(kind, "platform_command", message),
            &command.command_id,
            command.stamp_ms,
        );
        self.push_event(event);
    }

    fn handle_command_status(&mut self, status: &CommandStatus) {
        if status.agent_id.as_str() == self.focused_agent_id {
            let status_event = self.event_for_latest_command_id(
                RuntimeEvent::new(
                    command_status_event_kind(status),
                    "command_status",
                    self.command_status_event_message(status),
                ),
                &status.command_id,
                status.stamp_ms,
            );
            self.push_event(status_event);
        }
    }

    fn handle_constraint(&mut self, constraint: ConstraintEvent) {
        let stamp_ms = constraint.stamp_ms;
        let raised = raised_constraint_message(&constraint);
        let cleared = format!("Safety clear: {}", cleared_constraint_text(&constraint));

        match self.active_constraints.apply(constraint, now_ms()) {
            ApplyOutcome::Activated => {
                let event = self.event_for_latest_command_stamp(
                    RuntimeEvent::new(EventKind::Warning, "constraint", raised),
                    stamp_ms,
                );
                self.push_event(event);
            }
            ApplyOutcome::Cleared => {
                let event = self.event_for_latest_command_stamp(
                    RuntimeEvent::new(EventKind::Command, "constraint", cleared),
                    stamp_ms,
                );
                self.push_event(event);
            }
            ApplyOutcome::Refreshed | ApplyOutcome::Ignored => {}
        }
    }

    fn handle_veto(&mut self, veto: CommandVeto) {
        let stamp_ms = veto.stamp_ms;
        let command_id = veto.command_id.clone();
        let is_focused = veto.agent_id.as_str() == self.focused_agent_id;

        if veto.reason == veto_reason_none() {
            if is_focused && self.latest_veto.take().is_some() {
                let event = self.event_for_latest_command_id_or_stamp(
                    RuntimeEvent::new(EventKind::Info, "veto", "Command veto cleared."),
                    &command_id,
                    stamp_ms,
                );
                self.push_event(event);
            }
            return;
        }

        if is_focused {
            let event = self.event_for_latest_command_id_or_stamp(
                RuntimeEvent::new(EventKind::Warning, "veto", veto_message(&veto)),
                &command_id,
                stamp_ms,
            );
            self.push_event(event);
            self.latest_veto = Some(veto);
        }
    }

    fn handle_manipulation_status(&mut self, status: &ManipulationStatus) {
        let kind = if status.phase == manipulation_phase_failed() {
            EventKind::Warning
        } else {
            EventKind::Command
        };
        let event = self.event_for_latest_command_id(
            RuntimeEvent::new(kind, "manipulation", manipulation_status_message(status)),
            &status.command_id,
            status.stamp_ms,
        );
        self.push_event(event);
    }

    fn handle_object_state(&mut self, state: ObjectState) {
        let previous = self.object_states.get(&state.object_id).cloned();
        if self.object_state_is_from_tracker(&state.object_id) {
            for event in object_tracker_events(previous.as_ref(), &state) {
                self.push_event(event);
            }
        }

        self.held_object_id.retain(|_, obj| obj != &state.object_id);
        if state.lifecycle == object_lifecycle_held() {
            self.held_object_id
                .insert(state.holder_agent_id.clone(), state.object_id.clone());
        }

        self.object_states.insert(state.object_id.clone(), state);
    }

    fn object_state_is_from_tracker(&self, object_id: &str) -> bool {
        has_hardware_agents(&self.app_config)
            && self
                .catalog
                .get(object_id)
                .and_then(super::core_msgs::catalog::ObjectSpec::mocap_rigid_body_id)
                .is_some()
    }

    fn backend_for_agent(&self, agent_id: &str) -> Option<crate::core_msgs::config::AgentBackend> {
        self.app_config
            .agents
            .iter()
            .find(|agent| agent.runtime.agent_id == agent_id)
            .map(|agent| agent.manifest.backend)
    }

    fn handle_key(
        &mut self,
        key: KeyEvent,
        event_tx: &Sender<RuntimeEvent>,
        children: &mut ManagedChildren,
    ) {
        // On Windows, crossterm emits Press, Repeat, and Release events for
        // every key stroke.  Only react to Press so that toggles don't
        // rapid-fire on a single tap.
        if key.kind != KeyEventKind::Press {
            return;
        }

        if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
            self.request_shutdown(event_tx, "shutdown requested from keyboard");
            return;
        }

        if let Screen::Setup {
            pane,
            tab_cursor,
            field_cursor,
        } = self.screen
        {
            self.handle_setup_key(pane, tab_cursor, field_cursor, key.code, event_tx, children);
            return;
        }

        if let Screen::SpotPassword {
            return_pane,
            return_tab_cursor,
            return_field_cursor,
        } = self.screen
        {
            self.handle_spot_password_key(return_pane, return_tab_cursor, return_field_cursor, key);
            return;
        }

        if let Screen::Connecting { failed, .. } = self.screen {
            self.handle_connecting_key(failed, key.code, event_tx);
            return;
        }

        match key.code {
            KeyCode::Esc => self.request_shutdown(event_tx, "shutdown requested from keyboard"),
            KeyCode::Char('q') if self.input.is_empty() => {
                self.request_shutdown(event_tx, "shutdown requested from keyboard");
            }
            KeyCode::Enter => self.submit_input(),
            KeyCode::Tab => self.cycle_focused_agent(),
            KeyCode::Up => self.scroll_event_log(1),
            KeyCode::Down => self.scroll_event_log_down(1),
            KeyCode::PageUp => self.scroll_event_log(EVENT_LOG_SCROLL_STEP),
            KeyCode::PageDown => self.scroll_event_log_down(EVENT_LOG_SCROLL_STEP),
            KeyCode::Home => self.event_log_scroll = usize::MAX,
            KeyCode::End => self.event_log_scroll = 0,
            KeyCode::Backspace => {
                self.input.pop();
            }
            KeyCode::Delete => {
                self.input.clear();
            }
            KeyCode::Char(ch) => self.input.push(ch),
            _ => {}
        }
    }

    fn handle_connecting_key(
        &mut self,
        failed: bool,
        code: KeyCode,
        event_tx: &Sender<RuntimeEvent>,
    ) {
        match code {
            KeyCode::Esc | KeyCode::Char('q') => {
                self.request_shutdown(event_tx, "operator hit q");
            }
            KeyCode::Enter if failed && self.connecting_failure.is_none() => {
                if let Some(bridge) = self.mocap_bridge.as_mut()
                    && bridge.child.is_none()
                {
                    self.mocap_bridge = Some(MocapBridgeRuntime::start(&self.options));
                }
                self.screen = Screen::Connecting {
                    started: Instant::now(),
                    mocap_ok: false,
                    spot_ok: false,
                    failed: false,
                };
            }
            _ => {}
        }
    }

    fn handle_setup_arrow_key(
        &mut self,
        pane: SetupPane,
        tab_cursor: usize,
        field_cursor: usize,
        code: KeyCode,
        tabs: &[SetupTab],
    ) {
        match code {
            KeyCode::Left => {
                if pane == SetupPane::Fields {
                    self.screen = Screen::Setup {
                        pane,
                        tab_cursor,
                        field_cursor,
                    };
                    let tab = tabs.get(tab_cursor).copied().unwrap_or(SetupTab::Global);
                    self.handle_setup_toggle(tab, tab_cursor, field_cursor, false);
                }
            }
            KeyCode::Right => {
                if pane == SetupPane::Tabs {
                    self.screen = Screen::Setup {
                        pane: SetupPane::Fields,
                        tab_cursor,
                        field_cursor,
                    };
                } else {
                    self.screen = Screen::Setup {
                        pane,
                        tab_cursor,
                        field_cursor,
                    };
                    let tab = tabs.get(tab_cursor).copied().unwrap_or(SetupTab::Global);
                    self.handle_setup_toggle(tab, tab_cursor, field_cursor, true);
                }
            }
            KeyCode::Up => {
                if pane == SetupPane::Tabs {
                    self.screen = Screen::Setup {
                        pane,
                        tab_cursor: tab_cursor.saturating_sub(1),
                        field_cursor: 0,
                    };
                } else {
                    self.screen = Screen::Setup {
                        pane,
                        tab_cursor,
                        field_cursor: field_cursor.saturating_sub(1),
                    };
                }
            }
            KeyCode::Down => {
                if pane == SetupPane::Tabs {
                    self.screen = Screen::Setup {
                        pane,
                        tab_cursor: (tab_cursor + 1).min(tabs.len().saturating_sub(1)),
                        field_cursor: 0,
                    };
                } else {
                    let tab = tabs.get(tab_cursor).copied().unwrap_or(SetupTab::Global);
                    let field_count = self.setup_field_labels(tab).len();
                    self.screen = Screen::Setup {
                        pane,
                        tab_cursor,
                        field_cursor: next_setup_field_cursor(field_cursor, field_count),
                    };
                }
            }
            _ => {}
        }
    }

    fn handle_setup_enter(
        &mut self,
        tab: SetupTab,
        children: &mut ManagedChildren,
        event_tx: &Sender<RuntimeEvent>,
    ) {
        match tab {
            SetupTab::Start => {
                self.start_runtime(children, event_tx);
            }
            SetupTab::AddAgent => {
                let mut cfg = crate::core_msgs::config::AgentConfig::default();
                let new_id = format!("agent_{:02}", self.app_config.agents.len() + 1);
                cfg.runtime.agent_id.clone_from(&new_id);
                cfg.manifest.agent_id = new_id;
                self.app_config.agents.push(cfg);
            }
            SetupTab::AddVirtualObject => {
                self.app_config
                    .objects
                    .objects
                    .push(crate::core_msgs::config::ObjectSpec {
                        object_id: format!("virt_obj_{}", self.app_config.objects.objects.len()),
                        shape: "cylinder".to_owned(),
                        length_m: 0.5,
                        radius_m: 0.15,
                        width_m: 0.0,
                        height_m: 0.0,
                        color: "pink".to_owned(),
                        virtual_initial_pose_map: Some(crate::core_msgs::config::ObjectPose {
                            x: 0.0,
                            y: 0.0,
                            z: 0.15,
                            yaw_rad: 0.0,
                        }),
                        mocap_rigid_body_id: None,
                    });
            }
            SetupTab::AddMocapObject => {
                let next_mocap_id = self
                    .app_config
                    .objects
                    .objects
                    .iter()
                    .filter_map(|obj| obj.mocap_rigid_body_id)
                    .max()
                    .unwrap_or(0)
                    + 1;
                self.app_config
                    .objects
                    .objects
                    .push(crate::core_msgs::config::ObjectSpec {
                        object_id: format!("mocap_obj_{}", self.app_config.objects.objects.len()),
                        shape: "box".to_owned(),
                        length_m: 0.2,
                        radius_m: 0.0,
                        width_m: 0.2,
                        height_m: 0.2,
                        color: "orange".to_owned(),
                        virtual_initial_pose_map: None,
                        mocap_rigid_body_id: Some(next_mocap_id),
                    });
            }
            _ => {}
        }
    }

    fn handle_setup_key(
        &mut self,
        pane: SetupPane,
        tab_cursor: usize,
        field_cursor: usize,
        code: KeyCode,
        event_tx: &Sender<RuntimeEvent>,
        children: &mut ManagedChildren,
    ) {
        let tabs = self.setup_tabs();
        match code {
            KeyCode::Esc => {
                if pane == SetupPane::Fields {
                    self.screen = Screen::Setup {
                        pane: SetupPane::Tabs,
                        tab_cursor,
                        field_cursor: 0,
                    };
                } else {
                    self.request_shutdown(event_tx, "operator hit Esc");
                }
            }
            KeyCode::Char(c) => {
                if pane == SetupPane::Fields {
                    let tab = tabs.get(tab_cursor).copied().unwrap_or(SetupTab::Global);
                    if tab == SetupTab::Global && field_cursor == 3 {
                        self.options.mocap_ip.push(c);
                    }
                } else if c == 'q' {
                    self.request_shutdown(event_tx, "operator hit q");
                }
            }
            KeyCode::Backspace => {
                if pane == SetupPane::Fields {
                    let tab = tabs.get(tab_cursor).copied().unwrap_or(SetupTab::Global);
                    if tab == SetupTab::Global && field_cursor == 3 {
                        self.options.mocap_ip.pop();
                    }
                }
            }
            KeyCode::Tab => {
                self.screen = Screen::Setup {
                    pane: if pane == SetupPane::Tabs {
                        SetupPane::Fields
                    } else {
                        SetupPane::Tabs
                    },
                    tab_cursor,
                    field_cursor,
                };
            }
            KeyCode::Left | KeyCode::Right | KeyCode::Up | KeyCode::Down => {
                self.handle_setup_arrow_key(pane, tab_cursor, field_cursor, code, &tabs);
            }
            KeyCode::Enter => {
                let tab = tabs.get(tab_cursor).copied().unwrap_or(SetupTab::Global);
                if pane == SetupPane::Fields && tab == SetupTab::SpotSidecar && field_cursor == 1 {
                    self.open_spot_password_prompt(pane, tab_cursor, field_cursor);
                } else {
                    self.handle_setup_enter(tab, children, event_tx);
                }
            }
            _ => {}
        }
    }

    fn open_spot_password_prompt(
        &mut self,
        return_pane: SetupPane,
        return_tab_cursor: usize,
        return_field_cursor: usize,
    ) {
        self.spot_password_input.clear();
        self.spot_password_error = None;
        self.screen = Screen::SpotPassword {
            return_pane,
            return_tab_cursor,
            return_field_cursor,
        };
    }

    fn close_spot_password_prompt(
        &mut self,
        return_pane: SetupPane,
        return_tab_cursor: usize,
        return_field_cursor: usize,
    ) {
        self.spot_password_input.clear();
        self.spot_password_error = None;
        self.screen = Screen::Setup {
            pane: return_pane,
            tab_cursor: return_tab_cursor,
            field_cursor: return_field_cursor,
        };
    }

    /// Drives the Spot password prompt.
    ///
    /// On Enter with a non-empty buffer, promotes the captured input into a
    /// `SecretString` in `options.bosdyn_password` (later handed to the Spot
    /// child process); an empty buffer is rejected. The `Zeroizing` input
    /// buffer is wiped whenever the prompt closes.
    fn handle_spot_password_key(
        &mut self,
        return_pane: SetupPane,
        return_tab_cursor: usize,
        return_field_cursor: usize,
        key: KeyEvent,
    ) {
        match key.code {
            KeyCode::Esc => {
                self.close_spot_password_prompt(
                    return_pane,
                    return_tab_cursor,
                    return_field_cursor,
                );
            }
            KeyCode::Enter => {
                if self.spot_password_input.is_empty() {
                    self.spot_password_error = Some("Password cannot be empty.".to_owned());
                    return;
                }

                self.options.bosdyn_password =
                    Some(SecretString::from(self.spot_password_input.as_str()));
                self.close_spot_password_prompt(
                    return_pane,
                    return_tab_cursor,
                    return_field_cursor,
                );
            }
            KeyCode::Backspace => {
                self.spot_password_input.pop();
                self.spot_password_error = None;
            }
            KeyCode::Delete => {
                self.spot_password_input.clear();
                self.spot_password_error = None;
            }
            KeyCode::Char(ch) if !key.modifiers.contains(KeyModifiers::CONTROL) => {
                self.spot_password_input.push(ch);
                self.spot_password_error = None;
            }
            _ => {}
        }
    }

    fn handle_mouse(
        &mut self,
        mouse: MouseEvent,
        _event_tx: &Sender<RuntimeEvent>,
        _children: &mut ManagedChildren,
    ) {
        if matches!(self.screen, Screen::SpotPassword { .. }) {
            return;
        }

        if mouse.kind == MouseEventKind::Down(MouseButton::Left) {
            if let Screen::Setup { .. } = self.screen {
                return;
            }

            if let Screen::Connecting { failed, .. } = self.screen {
                if failed
                    && self.connecting_failure.is_none()
                    && let Some(rect) = self.retry_rect
                    && rect_contains(rect, mouse.column, mouse.row)
                {
                    if let Some(bridge) = self.mocap_bridge.as_mut()
                        && bridge.child.is_none()
                    {
                        self.mocap_bridge = Some(MocapBridgeRuntime::start(&self.options));
                    }
                    self.screen = Screen::Connecting {
                        started: Instant::now(),
                        mocap_ok: false,
                        spot_ok: false,
                        failed: false,
                    };
                }
                return;
            }
        }

        match mouse.kind {
            MouseEventKind::ScrollUp => {
                if self.mouse_is_over_event_log(mouse) {
                    self.scroll_event_log(EVENT_LOG_WHEEL_STEP);
                }
            }
            MouseEventKind::ScrollDown => {
                if self.mouse_is_over_event_log(mouse) {
                    self.scroll_event_log_down(EVENT_LOG_WHEEL_STEP);
                }
            }
            MouseEventKind::Down(MouseButton::Left) => {
                self.event_log_dragging_scrollbar = self.scroll_event_log_to_mouse(mouse);
            }
            MouseEventKind::Drag(MouseButton::Left) => {
                if self.event_log_dragging_scrollbar {
                    self.scroll_event_log_to_mouse(mouse);
                }
            }
            MouseEventKind::Up(MouseButton::Left) => {
                self.event_log_dragging_scrollbar = false;
            }
            _ => {}
        }
    }

    fn mouse_is_over_event_log(&self, mouse: MouseEvent) -> bool {
        self.event_log_area
            .is_some_and(|area| rect_contains(area, mouse.column, mouse.row))
    }

    fn scroll_event_log_to_mouse(&mut self, mouse: MouseEvent) -> bool {
        let Some(scrollbar) = self.event_log_scrollbar else {
            return false;
        };
        if !rect_contains(scrollbar.area, mouse.column, mouse.row) {
            return false;
        }

        self.event_log_scroll = event_log_scroll_from_scrollbar_row(scrollbar, mouse.row);
        true
    }

    const fn scroll_event_log(&mut self, amount: usize) {
        self.event_log_scroll = self.event_log_scroll.saturating_add(amount);
    }

    const fn scroll_event_log_down(&mut self, amount: usize) {
        self.event_log_scroll = self.event_log_scroll.saturating_sub(amount);
    }

    fn request_shutdown(&mut self, event_tx: &Sender<RuntimeEvent>, reason: &'static str) {
        if self.should_quit {
            return;
        }

        self.should_quit = true;
        let _ = event_tx.send(RuntimeEvent::new(EventKind::Component, "operator", reason));
    }

    fn submit_input(&mut self) {
        let line = self.input.trim().to_owned();
        self.input.clear();

        let (target_agent_id, rest) = self.resolve_command_target(&line);

        match command::parse_line(rest) {
            Ok(None) => {}
            Ok(Some(command)) => {
                let command = command::resolve_object_ids(command, &self.catalog);
                self.dispatch_command(&command, &target_agent_id);
            }
            Err(error) => self.push_event(RuntimeEvent::new(
                EventKind::Error,
                "input",
                error.message().to_owned(),
            )),
        }
    }

    fn resolve_command_target<'a>(&self, line: &'a str) -> (String, &'a str) {
        if let Some((first, rest)) = line.split_once(char::is_whitespace)
            && self
                .app_config
                .agents
                .iter()
                .any(|agent| agent.runtime.agent_id == first)
        {
            return (first.to_owned(), rest);
        }
        (self.focused_agent_id.clone(), line)
    }

    fn cycle_focused_agent(&mut self) {
        let agents = &self.app_config.agents;
        if agents.len() <= 1 {
            return;
        }
        let next = agents
            .iter()
            .position(|agent| agent.runtime.agent_id == self.focused_agent_id)
            .map_or(0, |current| (current + 1) % agents.len());
        self.focused_agent_id = agents[next].runtime.agent_id.clone();
    }

    /// Sends one operator command to the target agent.
    ///
    /// One operator action always becomes one behavior request. In particular,
    /// pickup never inserts an implicit navigation step; the receiving agent
    /// reports an unmet reach precondition when the object is out of reach.
    fn dispatch_command(&mut self, command: &command::OperatorCommand, target_agent_id: &str) {
        let now = now_ms();
        let command_epoch = self.begin_command_scope(now);

        if let Some(reason) = self.command_disabled_reason(command, target_agent_id) {
            self.push_event(
                RuntimeEvent::new(
                    EventKind::Warning,
                    "operator",
                    format!("Command disabled for {target_agent_id}: {reason}"),
                )
                .with_stamp_ms(now)
                .with_command_epoch(command_epoch),
            );
            return;
        }

        let event_label = command_sent_event_label(command);
        let history_label = command_history_label(command);
        if matches!(command, command::OperatorCommand::ResumeAutonomy) {
            let request_id = format!("{target_agent_id}-resume-{now}");
            self.register_latest_command_id(request_id.clone());
            let sent = send_or_log_drop(
                &self.operator_io.resume_autonomy_tx,
                topic_resume_autonomy(),
                ResumeAutonomy {
                    request_id,
                    agent_id: target_agent_id.to_owned(),
                    stamp_ms: now,
                },
            );
            if sent {
                self.autonomy_paused.remove(target_agent_id);
            }
            self.send_or_report(sent, &event_label, &history_label, command_epoch, now);
            return;
        }

        let Some(request) = command::behavior_request_for_command(
            command,
            target_agent_id,
            self.held_object_id.get(target_agent_id).map(String::as_str),
            now,
        ) else {
            self.push_event(
                RuntimeEvent::new(
                    EventKind::Error,
                    "operator",
                    "DropOff has no object to resolve: nothing is currently held and none was named",
                )
                .with_stamp_ms(now)
                .with_command_epoch(command_epoch),
            );
            return;
        };

        if let Some(goal) = command_goal_label(command) {
            self.register_latest_command_goal(&request.request_id, goal);
        } else {
            self.register_latest_command_goal(&request.request_id, history_label.clone());
        }
        let sent = send_or_log_drop(
            &self.operator_io.behavior_request_tx,
            topic_behavior_request(),
            request,
        );
        if sent {
            self.autonomy_paused.insert(target_agent_id.to_owned());
        }
        self.send_or_report(sent, &event_label, &history_label, command_epoch, now);
    }

    fn command_disabled_reason(
        &self,
        command: &command::OperatorCommand,
        target_agent_id: &str,
    ) -> Option<String> {
        if matches!(command, command::OperatorCommand::ResumeAutonomy) {
            let autonomy_enabled = self
                .app_config
                .agents
                .iter()
                .find(|agent| agent.runtime.agent_id == target_agent_id)
                .is_some_and(|agent| agent.manifest.autonomy_enabled);
            return (!autonomy_enabled).then(|| "autonomy is disabled in the manifest".to_owned());
        }

        let requirement = match command {
            command::OperatorCommand::GoTo(..) | command::OperatorCommand::MoveAway(..) => {
                CapabilityRequirement::PlanarNavigation
            }
            command::OperatorCommand::Hold => CapabilityRequirement::Hold,
            command::OperatorCommand::PickUp(..) => CapabilityRequirement::Pick,
            command::OperatorCommand::DropOff(..) => CapabilityRequirement::Place,
            command::OperatorCommand::ResumeAutonomy => unreachable!(),
        };
        capability_disabled_reason(self.capability_reports.get(target_agent_id), requirement)
    }

    fn send_or_report(
        &mut self,
        sent: bool,
        event_label: &str,
        history_label: &str,
        command_epoch: u64,
        stamp_ms: u64,
    ) {
        if sent {
            self.push_previous_command(history_label.to_owned());
            self.push_event(
                RuntimeEvent::new(EventKind::Command, "operator", event_label.to_owned())
                    .with_stamp_ms(stamp_ms)
                    .with_command_epoch(command_epoch),
            );
        } else {
            self.push_event(
                RuntimeEvent::new(
                    EventKind::Error,
                    "operator",
                    "failed to send command; DDS bridge is closed",
                )
                .with_stamp_ms(stamp_ms)
                .with_command_epoch(command_epoch),
            );
        }
    }

    fn push_previous_command(&mut self, command: String) {
        self.previous_commands.push_back(command);
        while self.previous_commands.len() > MAX_COMMAND_HISTORY {
            self.previous_commands.pop_front();
        }
    }

    fn push_event(&mut self, event: RuntimeEvent) {
        if let Some(status) = spot_runtime_status(&event) {
            self.spot_runtime_status = Some(status.to_owned());
        }
        if let Some((priority, detail)) = spot_sidecar_failure_detail(&event)
            && priority >= self.spot_sidecar_error_priority
        {
            self.latest_spot_sidecar_error = Some(detail);
            self.spot_sidecar_error_priority = priority;
            if self
                .connecting_failure
                .as_deref()
                .is_some_and(|reason| reason.starts_with("spot_sidecar exited unexpectedly"))
            {
                self.connecting_failure = self
                    .latest_spot_sidecar_error
                    .as_ref()
                    .map(|error| format!("Spot sidecar failed: {error}"));
            }
        }

        if self.events.front().is_some_and(|latest| {
            latest.same_content(&event) && duplicate_event_should_coalesce(latest)
        }) {
            return;
        }

        if activity_event_should_be_recorded(&event)
            && let Err(error) = self.activity_log.record(&event)
        {
            let log_event = RuntimeEvent::new(
                EventKind::Error,
                "operator_log",
                format!("operator activity log write failed: {error}"),
            );
            self.diagnostics.push_front(log_event.clone());
            trim_diagnostics(&mut self.diagnostics);
            push_recent_event(&mut self.events, log_event);
        }

        if event.kind == EventKind::Error {
            self.diagnostics.push_front(event.clone());
            trim_diagnostics(&mut self.diagnostics);
        }

        push_recent_event(&mut self.events, event);
    }

    fn draw(&mut self, frame: &mut Frame<'_>, children: &ManagedChildren) {
        match self.screen.clone() {
            Screen::Setup {
                pane,
                tab_cursor,
                field_cursor,
            } => self.draw_setup(frame, pane, tab_cursor, field_cursor),
            Screen::SpotPassword {
                return_pane,
                return_tab_cursor,
                return_field_cursor,
            } => self.draw_spot_password_prompt(
                frame,
                return_pane,
                return_tab_cursor,
                return_field_cursor,
            ),
            Screen::Connecting {
                started,
                mocap_ok,
                spot_ok,
                failed,
            } => self.draw_connecting(frame, started, mocap_ok, spot_ok, failed, children),
            Screen::Operator => self.draw_operator(frame, children),
        }
    }

    fn draw_spot_password_prompt(
        &self,
        frame: &mut Frame<'_>,
        return_pane: SetupPane,
        return_tab_cursor: usize,
        return_field_cursor: usize,
    ) {
        self.draw_setup(frame, return_pane, return_tab_cursor, return_field_cursor);

        let terminal_area = frame.area();
        let width = terminal_area.width.saturating_sub(4).clamp(1, 72);
        let height = terminal_area.height.saturating_sub(2).clamp(1, 9);
        let area = Rect {
            x: terminal_area.x + terminal_area.width.saturating_sub(width) / 2,
            y: terminal_area.y + terminal_area.height.saturating_sub(height) / 2,
            width,
            height,
        };

        let masked = "•".repeat(self.spot_password_input.chars().count());
        let password_line = if masked.is_empty() {
            "Password: ".to_owned()
        } else {
            format!("Password: {masked}")
        };
        let status_line = self
            .spot_password_error
            .as_deref()
            .unwrap_or("The password is kept in memory and passed only to Spot sidecar processes.");
        let status_style = if self.spot_password_error.is_some() {
            Style::default().fg(ratatui::style::Color::Red)
        } else {
            Style::default()
        };

        let prompt_message = format!(
            "A hardware-backed Spot agent requires the password for {}.",
            self.options.sidecar_ip
        );
        let prompt = Paragraph::new(vec![
            Line::from(prompt_message),
            Line::from(""),
            Line::styled(password_line, Style::default().add_modifier(Modifier::BOLD)),
            Line::from(""),
            Line::styled(status_line, status_style),
            Line::from("Enter: save    Esc: cancel    Backspace/Delete: edit"),
        ])
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Spot Credential "),
        )
        .wrap(Wrap { trim: true });

        frame.render_widget(Clear, area);
        frame.render_widget(prompt, area);
    }

    fn setup_tabs(&self) -> Vec<SetupTab> {
        let mut tabs = vec![SetupTab::Global];
        for i in 0..self.app_config.agents.len() {
            tabs.push(SetupTab::Agent(i));
        }
        tabs.push(SetupTab::AddAgent);

        let mut virt_idx = 0;
        let mut mocap_idx = 0;
        for obj in &self.app_config.objects.objects {
            if obj.mocap_rigid_body_id.is_none() {
                tabs.push(SetupTab::VirtualObject(virt_idx));
                virt_idx += 1;
            } else {
                tabs.push(SetupTab::MocapObject(mocap_idx));
                mocap_idx += 1;
            }
        }
        tabs.push(SetupTab::AddVirtualObject);
        tabs.push(SetupTab::AddMocapObject);
        tabs.push(SetupTab::SpotSidecar);
        tabs.push(SetupTab::Start);
        tabs
    }

    fn handle_setup_toggle(
        &mut self,
        tab: SetupTab,
        tab_cursor: usize,
        field_cursor: usize,
        forward: bool,
    ) {
        match tab {
            SetupTab::Global => {
                self.handle_setup_toggle_global(tab_cursor, field_cursor, forward);
            }
            SetupTab::Agent(idx) => self.handle_setup_toggle_agent(idx, field_cursor, forward),
            SetupTab::VirtualObject(idx) => {
                self.handle_setup_toggle_virtual_object(idx, field_cursor, forward);
            }
            SetupTab::MocapObject(idx) => {
                self.handle_setup_toggle_mocap_object(idx, field_cursor, forward);
            }
            SetupTab::SpotSidecar => self.handle_setup_toggle_spot_sidecar(field_cursor, forward),
            _ => {}
        }
    }

    fn handle_setup_toggle_global(
        &mut self,
        _tab_cursor: usize,
        field_cursor: usize,
        forward: bool,
    ) {
        match field_cursor {
            0 => self.options.visualizer = !self.options.visualizer,
            1 => self.options.mcap_recorder = !self.options.mcap_recorder,
            2 => self.options.mocap_wsl_relay = !self.options.mocap_wsl_relay,
            4 => {
                self.options.mocap_port = if forward {
                    self.options.mocap_port.saturating_add(1)
                } else {
                    self.options.mocap_port.saturating_sub(1)
                };
            }
            5 => {
                let step = if forward { 0.5 } else { -0.5 };
                self.app_config.environment.world_size[0] =
                    (self.app_config.environment.world_size[0] + step).max(1.0);
            }
            6 => {
                let step = if forward { 0.5 } else { -0.5 };
                self.app_config.environment.world_size[1] =
                    (self.app_config.environment.world_size[1] + step).max(1.0);
            }
            7 => {
                let step = if forward { 0.5 } else { -0.5 };
                self.app_config.environment.world_size[2] =
                    (self.app_config.environment.world_size[2] + step).max(1.0);
            }
            _ => {}
        }
    }

    fn handle_setup_toggle_agent(&mut self, idx: usize, field_cursor: usize, forward: bool) {
        if idx < self.app_config.agents.len() {
            let agent = &mut self.app_config.agents[idx];
            match field_cursor {
                2 => {
                    let backends = [
                        crate::core_msgs::config::AgentBackend::Simulated,
                        crate::core_msgs::config::AgentBackend::Hardware,
                        crate::core_msgs::config::AgentBackend::Hybrid,
                    ];
                    let current = backends
                        .iter()
                        .position(|backend| *backend == agent.manifest.backend)
                        .unwrap_or(0);
                    let next = if forward {
                        (current + 1) % backends.len()
                    } else {
                        (current + backends.len() - 1) % backends.len()
                    };
                    agent.manifest.backend = backends[next];
                }
                3 => {
                    let presets = [SpeedPreset::Slow, SpeedPreset::Medium, SpeedPreset::Fast];
                    let current = presets
                        .iter()
                        .position(|preset| *preset == agent.manifest.max_speed)
                        .unwrap_or(1);
                    let next = if forward {
                        (current + 1) % presets.len()
                    } else {
                        (current + presets.len() - 1) % presets.len()
                    };
                    agent.manifest.max_speed = presets[next];
                }
                4 => {
                    agent.size[0] = (agent.size[0] + if forward { 0.1 } else { -0.1 }).max(0.1);
                }
                5 => {
                    agent.size[1] = (agent.size[1] + if forward { 0.1 } else { -0.1 }).max(0.1);
                }
                6 => {
                    agent.size[2] = (agent.size[2] + if forward { 0.1 } else { -0.1 }).max(0.1);
                }
                7 => agent.initial_position[0] += if forward { 0.1 } else { -0.1 },
                8 => agent.initial_position[1] += if forward { 0.1 } else { -0.1 },
                9 => agent.initial_position[2] += if forward { 0.1 } else { -0.1 },
                _ => {}
            }
        }
    }

    fn handle_setup_toggle_virtual_object(
        &mut self,
        idx: usize,
        field_cursor: usize,
        forward: bool,
    ) {
        let Some(obj) = self
            .app_config
            .objects
            .objects
            .iter_mut()
            .filter(|obj| obj.mocap_rigid_body_id.is_none())
            .nth(idx)
        else {
            return;
        };
        match field_cursor {
            2 => obj.length_m = (obj.length_m + if forward { 0.1 } else { -0.1 }).max(0.1),
            3 => obj.radius_m = (obj.radius_m + if forward { 0.1 } else { -0.1 }).max(0.1),
            _ => {}
        }
    }

    fn handle_setup_toggle_mocap_object(&mut self, idx: usize, field_cursor: usize, forward: bool) {
        let Some(obj) = self
            .app_config
            .objects
            .objects
            .iter_mut()
            .filter(|obj| obj.mocap_rigid_body_id.is_some())
            .nth(idx)
        else {
            return;
        };
        if field_cursor == 2 {
            let val = obj.mocap_rigid_body_id.unwrap_or(0);
            obj.mocap_rigid_body_id = Some(if forward {
                val + 1
            } else {
                val.saturating_sub(1).max(1)
            });
        }
    }

    fn handle_setup_toggle_spot_sidecar(&mut self, field_cursor: usize, forward: bool) {
        match field_cursor {
            2 => self.options.sidecar_auto.power_on = !self.options.sidecar_auto.power_on,
            3 => self.options.sidecar_auto.stand = !self.options.sidecar_auto.stand,
            4 => self.options.sidecar_auto.calibrate = !self.options.sidecar_auto.calibrate,
            5 => {
                self.options.sidecar_mocap_body_id = if forward {
                    self.options.sidecar_mocap_body_id + 1
                } else {
                    self.options.sidecar_mocap_body_id.saturating_sub(1).max(1)
                };
            }
            6..=8 => {
                let axis = field_cursor - 6;
                self.options.sidecar_mocap_to_body_translation_m[axis] +=
                    if forward { 0.005 } else { -0.005 };
            }
            _ => {}
        }
    }

    fn draw_setup(
        &self,
        frame: &mut Frame<'_>,
        pane: SetupPane,
        tab_cursor: usize,
        field_cursor: usize,
    ) {
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(30), Constraint::Percentage(70)])
            .split(frame.area());

        let tabs = self.setup_tabs();
        let current_tab = tabs.get(tab_cursor).copied().unwrap_or(SetupTab::Global);

        Self::draw_setup_tabs(frame, chunks[0], pane, tab_cursor, &tabs);
        self.draw_setup_fields(frame, chunks[1], pane, field_cursor, current_tab);
    }

    fn draw_setup_tabs(
        frame: &mut Frame<'_>,
        area: ratatui::layout::Rect,
        pane: SetupPane,
        tab_cursor: usize,
        tabs: &[SetupTab],
    ) {
        let mut tab_items = Vec::new();
        for (i, tab) in tabs.iter().enumerate() {
            let prefix = if pane == SetupPane::Tabs && i == tab_cursor {
                "> "
            } else {
                "  "
            };
            let label = match tab {
                SetupTab::Global => "Global Settings".to_owned(),
                SetupTab::Agent(i) => format!("Agent {}", i + 1),
                SetupTab::VirtualObject(i) => format!("Virtual Object {}", i + 1),
                SetupTab::MocapObject(i) => format!("Mocap Object {}", i + 1),
                SetupTab::AddAgent => "+ Add Agent".to_owned(),
                SetupTab::AddVirtualObject => "+ Add Virtual Object".to_owned(),
                SetupTab::AddMocapObject => "+ Add Mocap Object".to_owned(),
                SetupTab::SpotSidecar => "Spot Sidecar".to_owned(),
                SetupTab::Start => "[ START ]".to_owned(),
            };
            let style = if pane == SetupPane::Tabs && i == tab_cursor {
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(ratatui::style::Color::Yellow)
            } else {
                Style::default()
            };
            tab_items.push(Line::styled(format!("{prefix}{label}"), style));
        }
        let tabs_block = Paragraph::new(tab_items)
            .block(Block::default().borders(Borders::ALL).title(" Setup Menu "));
        frame.render_widget(tabs_block, area);
    }

    fn setup_field_labels_global(&self) -> Vec<String> {
        vec![
            format!("  Visualizer: {}", on_off(self.options.visualizer)),
            format!("  Logging: {}", on_off(self.options.mcap_recorder)),
            format!(
                "  Mocap Source: {}",
                if self.options.mocap_wsl_relay {
                    "WSL Relay (TCP)"
                } else {
                    "Direct (UDP)"
                }
            ),
            format!("  Mocap IP: {}", self.options.mocap_ip),
            format!("  Mocap Port: {}", self.options.mocap_port),
            format!(
                "  World Size X: {:.2} m",
                self.app_config.environment.world_size[0]
            ),
            format!(
                "  World Size Y: {:.2} m",
                self.app_config.environment.world_size[1]
            ),
            format!(
                "  World Size Z: {:.2} m",
                self.app_config.environment.world_size[2]
            ),
        ]
    }

    fn setup_field_labels_agent(&self, idx: usize) -> Vec<String> {
        if idx < self.app_config.agents.len() {
            let agent = &self.app_config.agents[idx];
            vec![
                format!("  Agent ID: {}", agent.manifest.agent_id),
                format!("  Platform: {}", agent.manifest.platform),
                format!("  Backend: {}", agent.manifest.backend.as_str()),
                format!("  Max Speed: {}", agent_speed_label(agent)),
                format!("  Agent Size X: {:.2} m", agent.size[0]),
                format!("  Agent Size Y: {:.2} m", agent.size[1]),
                format!("  Agent Size Z: {:.2} m", agent.size[2]),
                format!("  Init Pos X: {:.2} m", agent.initial_position[0]),
                format!("  Init Pos Y: {:.2} m", agent.initial_position[1]),
                format!("  Init Pos Z: {:.2} m", agent.initial_position[2]),
            ]
        } else {
            vec![]
        }
    }

    fn setup_field_labels_virtual_object(&self, idx: usize) -> Vec<String> {
        self.app_config
            .objects
            .objects
            .iter()
            .filter(|obj| obj.mocap_rigid_body_id.is_none())
            .nth(idx)
            .map_or_else(Vec::new, |obj| {
                vec![
                    format!("  Object ID: {}", obj.object_id),
                    format!("  Shape: {}", obj.shape),
                    format!("  Length: {:.2} m", obj.length_m),
                    format!("  Radius: {:.2} m", obj.radius_m),
                ]
            })
    }

    fn setup_field_labels_mocap_object(&self, idx: usize) -> Vec<String> {
        self.app_config
            .objects
            .objects
            .iter()
            .filter(|obj| obj.mocap_rigid_body_id.is_some())
            .nth(idx)
            .map_or_else(Vec::new, |obj| {
                vec![
                    format!("  Object ID: {}", obj.object_id),
                    format!("  Shape: {}", obj.shape),
                    format!("  Mocap ID: {}", obj.mocap_rigid_body_id.unwrap_or(0)),
                ]
            })
    }

    fn setup_field_labels_spot_sidecar(&self) -> Vec<String> {
        vec![
            format!("  IP: {}", self.options.sidecar_ip),
            format!(
                "  Password: {}",
                if self.options.bosdyn_password.is_some() {
                    "configured (ENTER to replace)"
                } else if has_spot_hardware_agents(&self.app_config) {
                    "required (ENTER to enter)"
                } else {
                    "not required (ENTER to preconfigure)"
                }
            ),
            format!(
                "  Auto Power On: {}",
                on_off(self.options.sidecar_auto.power_on)
            ),
            format!("  Auto Stand: {}", on_off(self.options.sidecar_auto.stand)),
            format!(
                "  Auto Calibrate: {}",
                on_off(self.options.sidecar_auto.calibrate)
            ),
            format!("  Mocap Body ID: {}", self.options.sidecar_mocap_body_id),
            format!(
                "  Mocap→Body Forward: {:.3} m",
                self.options.sidecar_mocap_to_body_translation_m[0]
            ),
            format!(
                "  Mocap→Body Left: {:.3} m",
                self.options.sidecar_mocap_to_body_translation_m[1]
            ),
            format!(
                "  Mocap→Body Up: {:.3} m",
                self.options.sidecar_mocap_to_body_translation_m[2]
            ),
        ]
    }

    fn setup_field_labels(&self, current_tab: SetupTab) -> Vec<String> {
        match current_tab {
            SetupTab::Global => self.setup_field_labels_global(),
            SetupTab::Agent(idx) => self.setup_field_labels_agent(idx),
            SetupTab::VirtualObject(idx) => self.setup_field_labels_virtual_object(idx),
            SetupTab::MocapObject(idx) => self.setup_field_labels_mocap_object(idx),
            SetupTab::SpotSidecar => self.setup_field_labels_spot_sidecar(),
            _ => vec!["  Press ENTER to execute".to_owned()],
        }
    }

    fn draw_setup_fields(
        &self,
        frame: &mut Frame<'_>,
        area: ratatui::layout::Rect,
        pane: SetupPane,
        field_cursor: usize,
        current_tab: SetupTab,
    ) {
        let mut field_items = Vec::new();
        let field_labels = self.setup_field_labels(current_tab);

        for (i, label) in field_labels.iter().enumerate() {
            let prefix = if pane == SetupPane::Fields && i == field_cursor {
                "> "
            } else {
                "  "
            };
            let style = if pane == SetupPane::Fields && i == field_cursor {
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(ratatui::style::Color::Yellow)
            } else {
                Style::default()
            };
            field_items.push(Line::styled(format!("{prefix}{label}"), style));
        }
        let fields_block = Paragraph::new(field_items)
            .block(Block::default().borders(Borders::ALL).title(" Edit "));
        frame.render_widget(fields_block, area);
    }
    fn draw_connecting(
        &mut self,
        frame: &mut Frame<'_>,
        started: Instant,
        mocap_ok: bool,
        spot_ok: bool,
        failed: bool,
        children: &ManagedChildren,
    ) {
        let area = frame.area();
        let title = if failed {
            " Connection Failed "
        } else {
            " Connecting... "
        };
        let block = Block::default().title(title).borders(Borders::ALL);
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let layout = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(1),
                Constraint::Length(1),
                Constraint::Length(1),
                Constraint::Length(1),
                Constraint::Length(1),
                Constraint::Length(2),
                Constraint::Min(0),
            ])
            .split(inner);

        let elapsed = started.elapsed().as_secs();
        frame.render_widget(
            Paragraph::new(format!(
                "Elapsed: {}s / {}s",
                elapsed,
                CONNECT_TIMEOUT.as_secs()
            )),
            layout[0],
        );
        frame.render_widget(
            Paragraph::new(format!(
                "MOCAP: {}",
                if mocap_ok {
                    "OK"
                } else {
                    "Waiting for mocap_rigid_body..."
                }
            )),
            layout[1],
        );
        frame.render_widget(
            Paragraph::new(format!(
                "Agent: {}",
                if spot_ok {
                    "OK"
                } else {
                    "Waiting for agent_state..."
                }
            )),
            layout[2],
        );

        let bridge_line = self
            .mocap_bridge
            .as_ref()
            .map_or("not running", MocapBridgeRuntime::latest_line);
        frame.render_widget(Paragraph::new(format!("Bridge: {bridge_line}")), layout[3]);

        frame.render_widget(
            Paragraph::new(format!("Children: {}", self.system_status_label(children))),
            layout[4],
        );

        let spot_runtime = if has_spot_hardware_agents(&self.app_config) {
            self.latest_spot_runtime_status()
                .unwrap_or("waiting for runtime check...")
        } else {
            "not required"
        };
        frame.render_widget(
            Paragraph::new(format!("Spot runtime: {spot_runtime}")),
            layout[5],
        );

        if failed {
            if let Some(reason) = self.connecting_failure.as_deref() {
                frame.render_widget(
                    Paragraph::new(format!(
                        "Startup failed: {reason}. Exit and relaunch after correcting the error."
                    ))
                    .wrap(Wrap { trim: true }),
                    layout[6],
                );
                self.retry_rect = None;
            } else {
                let retry_text =
                    "Timeout! Missing signal shown. (press Enter to retry, or click [ Retry ])";
                frame.render_widget(Paragraph::new(retry_text), layout[6]);
                self.retry_rect = Some(layout[6]);
            }
        } else {
            frame.render_widget(
                Paragraph::new(
                    "The launcher will continue when the configured state sources are ready.",
                ),
                layout[6],
            );
            self.retry_rect = None;
        }
    }

    fn latest_spot_runtime_status(&self) -> Option<&str> {
        self.spot_runtime_status.as_deref()
    }

    fn startup_failure_reason(
        &self,
        component: super::Component,
        fallback_reason: String,
    ) -> String {
        if component != super::Component::SpotSidecar {
            return fallback_reason;
        }

        self.latest_spot_sidecar_error
            .as_ref()
            .map_or(fallback_reason, |error| {
                format!("Spot sidecar failed: {error}")
            })
    }

    fn draw_operator(&mut self, frame: &mut Frame<'_>, children: &ManagedChildren) {
        let root = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),
                Constraint::Length(7),
                Constraint::Length(7),
                Constraint::Length(6),
                Constraint::Min(6),
                Constraint::Length(10),
                Constraint::Length(4),
                Constraint::Length(3),
            ])
            .split(frame.area());

        self.render_header(frame, root[0], children);

        let top = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(root[1]);
        self.render_components(frame, top[0], children);
        self.render_errors(frame, top[1]);

        if self.mocap_bridge.is_some() {
            let position = Layout::default()
                .direction(Direction::Horizontal)
                .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
                .split(root[2]);
            self.render_agent_position(frame, position[0]);
            self.render_mocap_bridge(frame, position[1]);
        } else {
            self.render_agent_position(frame, root[2]);
        }
        self.render_objects(frame, root[3]);
        self.render_runtime_events(frame, root[4]);
        self.render_help(frame, root[5]);
        self.render_previous_commands(frame, root[6]);
        self.render_input(frame, root[7]);
    }

    fn render_header(&self, frame: &mut Frame<'_>, area: Rect, children: &ManagedChildren) {
        let report = self.capability_reports.get(&self.focused_agent_id);
        let profile = report.map_or("awaiting profile", |report| {
            report.platform_profile.as_str()
        });
        let adapter = report.map_or("adapter unknown", |report| {
            if report.connected {
                "adapter connected"
            } else {
                "adapter unavailable"
            }
        });
        let autonomy = self
            .app_config
            .agents
            .iter()
            .find(|agent| agent.runtime.agent_id == self.focused_agent_id)
            .map_or("autonomy unknown", |agent| {
                if !agent.manifest.autonomy_enabled {
                    "autonomy disabled"
                } else if self.autonomy_paused.contains(&self.focused_agent_id) {
                    "autonomy paused"
                } else {
                    "autonomy enabled"
                }
            });
        let behavior = self
            .behavior_statuses
            .get(&self.focused_agent_id)
            .map_or_else(|| "idle".to_owned(), |status| format!("{:?}", status.phase));
        let title = format!(
            "{} | {profile} | {adapter} | {autonomy} | behavior {behavior}",
            agent_display_label(&self.focused_agent_id),
        );
        let status = if self.dds_disconnected {
            "System: Degraded"
        } else {
            self.system_status_label(children)
        };
        let inner_width = area.width.saturating_sub(2) as usize;
        let status_width = status.len().min(inner_width);
        let title_width = inner_width.saturating_sub(status_width + 1);
        let title: String = title.chars().take(title_width).collect();
        let status: String = status.chars().take(status_width).collect();
        let gap = inner_width.saturating_sub(title.chars().count() + status.chars().count());
        let status_style = Style::default();
        let header = Paragraph::new(Line::from(vec![
            Span::styled(title, Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(" ".repeat(gap)),
            Span::styled(status, status_style),
        ]))
        .block(Block::default().borders(Borders::ALL));
        frame.render_widget(header, area);
    }

    fn render_components(&self, frame: &mut Frame<'_>, area: Rect, children: &ManagedChildren) {
        let block = Block::default().borders(Borders::ALL).title("Components");
        let inner = block.inner(area);
        let width = inner.width as usize;
        let height = inner.height as usize;
        let lines = if children.children.is_empty() {
            vec![Line::from("no managed components")]
        } else {
            children
                .children
                .iter()
                .take(height)
                .map(|managed| {
                    let is_up = match managed.component {
                        super::Component::EnvironmentSim
                        | super::Component::Agent
                        | super::Component::ObjectTracker
                        | super::Component::SpotSidecar
                        | super::Component::MocapZenohBridge => {
                            self.running_components.contains(&managed.component)
                        }
                        super::Component::Visualizer | super::Component::McapRecorder => {
                            let started = match self.screen {
                                Screen::Connecting { started, .. } => Some(started),
                                _ => None,
                            };
                            started.is_none_or(|t| t.elapsed() >= Duration::from_millis(1500))
                        }
                    };
                    let status = if managed.exit_status.is_some() {
                        "exited".to_owned()
                    } else if is_up {
                        "running".to_owned()
                    } else {
                        "starting...".to_owned()
                    };
                    let status_style = Style::default();
                    let status_width = 11usize.min(width);
                    let name_width = width.saturating_sub(status_width + 1).max(1);

                    Line::from(vec![
                        Span::raw(fit_column(managed.component.as_str(), name_width)),
                        Span::raw(" "),
                        Span::styled(fit_column(&status, status_width), status_style),
                    ])
                })
                .collect()
        };

        frame.render_widget(block, area);
        frame.render_widget(Paragraph::new(lines), inner);
    }

    fn render_agent_position(&self, frame: &mut Frame<'_>, area: Rect) {
        let outer = Block::default()
            .borders(Borders::ALL)
            .title("Agent Position");
        let inner = outer.inner(area);
        frame.render_widget(outer, area);

        if inner.width == 0 || inner.height == 0 {
            return;
        }

        let agent_block = Block::default()
            .borders(Borders::ALL)
            .title(agent_display_label(&self.focused_agent_id));
        let agent_inner = agent_block.inner(inner);
        frame.render_widget(agent_block, inner);

        let (source, map_position, bevy_position) = self.agent_position_display();
        let rows = vec![
            Row::new(vec![
                Cell::from("map"),
                Cell::from(source.clone()),
                Cell::from(map_position[0].clone()),
                Cell::from(map_position[1].clone()),
                Cell::from(map_position[2].clone()),
            ]),
            Row::new(vec![
                Cell::from("bevy_world"),
                Cell::from(source),
                Cell::from(bevy_position[0].clone()),
                Cell::from(bevy_position[1].clone()),
                Cell::from(bevy_position[2].clone()),
            ]),
        ];

        let table = Table::new(
            rows,
            [
                Constraint::Length(12),
                Constraint::Length(22),
                Constraint::Length(10),
                Constraint::Length(10),
                Constraint::Length(10),
            ],
        )
        .header(compact_header_row(["Frame", "Source", "X", "Y", "Z"]));
        frame.render_widget(table, agent_inner);
    }

    fn render_mocap_bridge(&self, frame: &mut Frame<'_>, area: Rect) {
        let block = Block::default().borders(Borders::ALL).title("Mocap Bridge");
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let line = self.mocap_bridge.as_ref().map_or(
            "mocap bridge disabled in simulation mode",
            MocapBridgeRuntime::latest_line,
        );
        let paragraph = Paragraph::new(line.to_owned()).wrap(Wrap { trim: true });
        frame.render_widget(paragraph, inner);
    }

    fn render_objects(&self, frame: &mut Frame<'_>, area: Rect) {
        let block = Block::default().borders(Borders::ALL).title("Objects");
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let mut ids: Vec<&str> = self
            .catalog
            .iter()
            .map(super::core_msgs::catalog::ObjectSpec::object_id)
            .collect();
        ids.sort_unstable();

        let rows: Vec<Row<'_>> = ids
            .into_iter()
            .map(|object_id| {
                let source = self
                    .catalog
                    .get(object_id)
                    .and_then(super::core_msgs::catalog::ObjectSpec::mocap_rigid_body_id)
                    .map_or_else(
                        || "virtual".to_owned(),
                        |mocap_id| format!("mocap #{mocap_id}"),
                    );
                let state = self.object_states.get(object_id);
                let lifecycle = state.map_or_else(
                    || "UNKNOWN".to_owned(),
                    |s| object_lifecycle_name(s.lifecycle).to_owned(),
                );
                let holder = state
                    .map(|s| s.holder_agent_id.as_str())
                    .filter(|holder| !holder.is_empty())
                    .unwrap_or("-")
                    .to_owned();
                let valid = match state {
                    Some(s) if s.pose_valid => "yes",
                    Some(_) => "no",
                    None => "-",
                };
                let pose = state.map_or_else(
                    || "-".to_owned(),
                    |s| format!("{:.2}, {:.2}, {:.2}", s.pose.x, s.pose.y, s.pose.z),
                );

                Row::new(vec![
                    Cell::from(object_id.to_owned()),
                    Cell::from(source),
                    Cell::from(lifecycle),
                    Cell::from(holder),
                    Cell::from(valid),
                    Cell::from(pose),
                ])
            })
            .collect();

        let table = Table::new(
            rows,
            [
                Constraint::Length(16),
                Constraint::Length(12),
                Constraint::Length(11),
                Constraint::Length(10),
                Constraint::Length(6),
                Constraint::Length(22),
            ],
        )
        .header(compact_header_row([
            "Object ID",
            "Source",
            "Lifecycle",
            "Holder",
            "Valid",
            "Pose (x,y,z)",
        ]));
        frame.render_widget(table, inner);
    }

    fn agent_position_display(&self) -> (String, [String; 3], [String; 3]) {
        let Some(state) = self.latest_state.get(&self.focused_agent_id) else {
            return ("waiting".to_owned(), dash_position(), dash_position());
        };

        let map_position = [
            f64::from(state.position[0]),
            f64::from(state.position[1]),
            f64::from(state.position[2]),
        ];
        let bevy_position = self.t_bevy_map.transform_point(map_position);

        (
            position_source_label(state).to_owned(),
            format_position(map_position),
            format_position(bevy_position),
        )
    }

    fn render_runtime_events(&mut self, frame: &mut Frame<'_>, area: Rect) {
        self.event_log_area = Some(area);
        let provisional_outer = Block::default().borders(Borders::ALL);
        let inner = provisional_outer.inner(area);

        if inner.width == 0 || inner.height == 0 {
            self.event_log_scrollbar = None;
            self.event_log_dragging_scrollbar = false;
            frame.render_widget(
                Block::default().borders(Borders::ALL).title("Event Log"),
                area,
            );
            return;
        }

        let agent_block = Block::default()
            .borders(Borders::ALL)
            .title(agent_display_label(&self.focused_agent_id));
        let agent_inner = agent_block.inner(inner);
        let event_text_area = event_log_text_area(agent_inner);
        let events = event_log_events(&self.events);
        let body_line_count =
            runtime_event_body_line_count(events.iter().copied(), event_text_area.width as usize);
        let max_scroll =
            runtime_event_max_scroll_for_body(body_line_count, event_text_area.height as usize);
        self.event_log_scroll = self.event_log_scroll.min(max_scroll);
        self.event_log_scrollbar = (max_scroll > 0)
            .then(|| event_log_scrollbar(agent_inner, body_line_count, event_text_area.height));
        if self.event_log_scrollbar.is_none() {
            self.event_log_dragging_scrollbar = false;
        }

        let outer = Block::default().borders(Borders::ALL).title("Event Log");
        frame.render_widget(outer, area);
        frame.render_widget(agent_block, inner);

        let lines = runtime_event_lines_with_scroll(
            events.iter().copied(),
            event_text_area.width as usize,
            event_text_area.height as usize,
            self.event_log_scroll,
        );
        frame.render_widget(Paragraph::new(lines), event_text_area);

        if max_scroll > 0 && agent_inner.width > 0 && agent_inner.height > 0 {
            let position = runtime_event_scroll_position(
                body_line_count,
                event_text_area.height as usize,
                self.event_log_scroll,
            );
            let mut scrollbar = ScrollbarState::new(body_line_count)
                .position(position)
                .viewport_content_length(event_text_area.height.saturating_sub(1) as usize);
            frame.render_stateful_widget(
                Scrollbar::new(ScrollbarOrientation::VerticalRight),
                agent_inner,
                &mut scrollbar,
            );
        }
    }

    fn render_errors(&self, frame: &mut Frame<'_>, area: Rect) {
        let block = Block::default()
            .borders(Borders::ALL)
            .title("Errors & Warnings");
        let inner = block.inner(area);
        let lines = diagnostic_lines(
            self.diagnostics
                .iter()
                .filter(|event| diagnostic_event_is_visible(event)),
            inner.width,
            inner.height,
        );

        frame.render_widget(Clear, area);
        frame.render_widget(block, area);
        frame.render_widget(Clear, inner);
        frame.render_widget(Paragraph::new(lines), inner);
    }

    fn render_help(&self, frame: &mut Frame<'_>, area: Rect) {
        let navigation = self.primitive_status(CapabilityRequirement::PlanarNavigation);
        let hold = self.primitive_status(CapabilityRequirement::Hold);
        let pick = self.primitive_status(CapabilityRequirement::Pick);
        let place = self.primitive_status(CapabilityRequirement::Place);
        let resume = self
            .app_config
            .agents
            .iter()
            .find(|agent| agent.runtime.agent_id == self.focused_agent_id)
            .map_or_else(
                || "[disabled: agent unknown]".to_owned(),
                |agent| {
                    if agent.manifest.autonomy_enabled {
                        "[enabled]".to_owned()
                    } else {
                        "[disabled: autonomy disabled]".to_owned()
                    }
                },
            );
        let max_speed = self
            .app_config
            .agents
            .iter()
            .find(|agent| agent.runtime.agent_id == self.focused_agent_id)
            .map_or_else(|| "unknown".to_owned(), agent_speed_label);
        let text = Paragraph::new(vec![
            Line::from(format!(
                "Focused: {}",
                agent_display_label(&self.focused_agent_id)
            )),
            Line::from(format!("Max speed: {max_speed}")),
            Line::from(format!("g X Y: navigate {navigation}")),
            Line::from(format!("m X Y: move away {navigation}")),
            Line::from(format!("h: hold {hold}")),
            Line::from(format!("p [OBJECT]: pick {pick}")),
            Line::from(format!("d [OBJECT]: place {place}")),
            Line::from(format!("r: resume autonomy {resume}")),
            Line::from("AGENT_ID ...: target | Tab: focus next"),
            Line::from("Enter: send"),
            Line::from("q/Esc: exit"),
        ])
        .block(Block::default().borders(Borders::ALL).title("Help"))
        .wrap(Wrap { trim: true });
        frame.render_widget(text, area);
    }

    fn primitive_status(&self, requirement: CapabilityRequirement) -> String {
        capability_disabled_reason(
            self.capability_reports.get(&self.focused_agent_id),
            requirement,
        )
        .map_or_else(
            || "[enabled]".to_owned(),
            |reason| format!("[disabled: {reason}]"),
        )
    }

    fn render_previous_commands(&self, frame: &mut Frame<'_>, area: Rect) {
        let lines: Vec<_> = if self.previous_commands.is_empty() {
            vec![Line::from("No previous commands")]
        } else {
            self.previous_commands
                .iter()
                .enumerate()
                .map(|(index, command)| Line::from(format!("{}. {command}", index + 1)))
                .collect()
        };

        let previous = Paragraph::new(lines)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title("Previous Commands"),
            )
            .wrap(Wrap { trim: true });
        frame.render_widget(previous, area);
    }

    fn render_input(&self, frame: &mut Frame<'_>, area: Rect) {
        let input = Paragraph::new(format!("> {}", self.input)).block(
            Block::default()
                .borders(Borders::ALL)
                .title("Command Input"),
        );
        frame.render_widget(input, area);
    }

    fn system_status_label(&self, children: &ManagedChildren) -> &'static str {
        if self
            .mocap_bridge
            .as_ref()
            .is_some_and(MocapBridgeRuntime::failed)
            || children
                .children
                .iter()
                .any(|child| child.exit_status.is_some())
        {
            "System: Degraded"
        } else {
            let is_running = |component| self.running_components.contains(&component);
            let mut all_up = is_running(super::Component::Agent);
            if self
                .app_config
                .agents
                .iter()
                .any(|agent| agent.manifest.backend.starts_environment())
            {
                all_up &= is_running(super::Component::EnvironmentSim);
            }
            if mocap_bridge_enabled(&self.app_config) {
                all_up &= is_running(super::Component::MocapZenohBridge);
            }
            if has_hardware_agents(&self.app_config) {
                all_up &= is_running(super::Component::ObjectTracker);
                all_up &= is_running(super::Component::SpotSidecar);
            }

            if all_up {
                "System: Running"
            } else {
                "System: Starting"
            }
        }
    }
}

const fn on_off(enabled: bool) -> &'static str {
    if enabled { "ON" } else { "OFF" }
}

fn capability_disabled_reason(
    report: Option<&CapabilityReport>,
    requirement: CapabilityRequirement,
) -> Option<String> {
    let Some(report) = report else {
        return Some("awaiting capabilities".to_owned());
    };
    if !report.capabilities.supports(requirement) {
        return Some(format!(
            "{} does not support {requirement:?}",
            report.platform_profile
        ));
    }
    if !report.connected {
        return Some(
            report
                .unavailable_reason
                .clone()
                .unwrap_or_else(|| "platform adapter is unavailable".to_owned()),
        );
    }
    None
}

fn has_hardware_agents(config: &crate::core_msgs::config::AppConfig) -> bool {
    config
        .agents
        .iter()
        .any(|agent| agent.manifest.backend.requires_platform())
}

fn has_spot_hardware_agents(config: &crate::core_msgs::config::AppConfig) -> bool {
    config.agents.iter().any(|agent| {
        agent.manifest.backend.requires_platform()
            && agent.manifest.platform == platform_spot::PROFILE
    })
}

fn mocap_bridge_enabled(config: &crate::core_msgs::config::AppConfig) -> bool {
    has_hardware_agents(config)
}

fn next_setup_field_cursor(current: usize, field_count: usize) -> usize {
    current.saturating_add(1).min(field_count.saturating_sub(1))
}

fn agent_speed_label(agent: &crate::core_msgs::config::AgentConfig) -> String {
    let preset = agent.manifest.max_speed;
    let Some(navigation) = crate::platform_profiles::capabilities_for(&agent.manifest.platform)
        .and_then(|capabilities| {
            capabilities
                .navigation
                .into_iter()
                .find(|capability| capability.kind == crate::core_msgs::NavigationKind::Planar)
        })
    else {
        return format!("{} (unavailable)", preset.as_str());
    };
    let limits = navigation.speed_presets.selected(preset);
    format!(
        "{} ({:.2} m/s, {:.2} rad/s)",
        preset.as_str(),
        limits.max_linear_speed_m_s,
        limits.max_yaw_rate_rad_s
    )
}

fn command_status_event_message(status: &CommandStatus) -> String {
    if status.status == command_status_succeeded() {
        "Reached the requested target.".to_owned()
    } else if status.status == command_status_aborted() {
        "Command failed before reaching the requested target.".to_owned()
    } else {
        format!(
            "Command status changed: {}.",
            command_status_label(status.status)
        )
    }
}

fn command_status_label(status: u8) -> String {
    if status == command_status_succeeded() {
        "succeeded".to_owned()
    } else if status == command_status_aborted() {
        "failed".to_owned()
    } else {
        humanize_code(command_status_name(status))
    }
}

const fn command_status_event_kind(status: &CommandStatus) -> EventKind {
    if status.status == command_status_succeeded() {
        EventKind::Command
    } else if status.status == command_status_aborted() {
        EventKind::Error
    } else {
        EventKind::Warning
    }
}

fn platform_command_event(
    command: &PlatformCommand,
    operator_requested_hold: bool,
) -> Option<(EventKind, String)> {
    let target = command.target_position;

    if command.command_type == command_type_stop() {
        return Some((
            EventKind::Warning,
            format!(
                "Safety stop: robot stopped near {:.2} {:.2}.",
                target[0], target[1]
            ),
        ));
    }

    if operator_requested_hold {
        return Some((
            EventKind::Command,
            format!("Holding position at {:.2} {:.2}.", target[0], target[1]),
        ));
    }

    if command.command_type == command_type_hold() {
        return None;
    }

    Some((
        EventKind::Info,
        format!("Driving to {:.2} {:.2}.", target[0], target[1]),
    ))
}

fn raised_constraint_message(constraint: &ConstraintEvent) -> String {
    format!("Safety stop: {}", active_constraint_text(constraint))
}

fn cleared_constraint_text(constraint: &ConstraintEvent) -> String {
    let subject = constraint_subject(constraint);
    if constraint.constraint_type == constraint_type_robot_fault() {
        "robot fault is no longer active.".to_owned()
    } else if constraint.constraint_type == constraint_type_goal_blocked() {
        format!("{subject} is no longer blocking the goal.")
    } else {
        format!("{subject} is no longer blocking motion.")
    }
}

fn active_constraint_text(constraint: &ConstraintEvent) -> String {
    active_constraint_entry_text(constraint.constraint_type, &constraint.object_id)
}

fn active_constraint_entry_text(constraint_type: u8, object_id: &str) -> String {
    let subject = constraint_subject_from_parts(constraint_type, object_id);
    if constraint_type == constraint_type_robot_fault() {
        "robot fault is active.".to_owned()
    } else if constraint_type == constraint_type_goal_blocked() {
        format!("{subject} blocked the goal.")
    } else {
        format!("{subject} is blocking the path.")
    }
}

fn constraint_subject(constraint: &ConstraintEvent) -> String {
    constraint_subject_from_parts(constraint.constraint_type, &constraint.object_id)
}

fn constraint_subject_from_parts(constraint_type: u8, object_id: &str) -> String {
    if !object_id.is_empty() && object_id != "-" {
        object_id.to_owned()
    } else if constraint_type == constraint_type_robot_fault() {
        "robot fault".to_owned()
    } else if constraint_type == constraint_type_goal_blocked() {
        "the goal".to_owned()
    } else if constraint_type == constraint_type_wall() {
        "a wall".to_owned()
    } else if constraint_type == constraint_type_virtual_wall() {
        "a virtual wall".to_owned()
    } else {
        humanize_code(constraint_type_name(constraint_type))
    }
}

fn veto_reason_label(reason: u8) -> String {
    humanize_code(veto_reason_name(reason))
}

fn veto_message(veto: &CommandVeto) -> String {
    let reason = veto_reason_label(veto.reason);
    let detail = readable_veto_detail(&veto.detail);
    if detail.is_empty() {
        format!("Command vetoed: {reason}.")
    } else {
        format!("Command vetoed: {reason}. {detail}")
    }
}

fn readable_veto_detail(detail: &str) -> String {
    let detail = detail.trim();
    if detail.is_empty() {
        return String::new();
    }

    if let Some(active) = detail.strip_prefix("Collision: hard stop active; active: ") {
        return format!(
            "Robot stopped before completing the command because {}",
            readable_active_constraint_summary(active)
        );
    }

    if let Some(active) = detail.strip_prefix("Robot fault: hard stop active; active: ") {
        return format!(
            "Robot stopped before completing the command because {}",
            readable_active_constraint_summary(active)
        );
    }

    match detail {
        "Collision: hard stop active" => "Robot stopped before completing the command.".to_owned(),
        "Robot fault: hard stop active" => {
            "Robot stopped before completing the command because robot fault is active.".to_owned()
        }
        "State too old" => "The latest platform state is stale.".to_owned(),
        "No platform state available" => "No platform state is available.".to_owned(),
        "Source not authorized" => "The command source is not authorized.".to_owned(),
        "Intent frame_id is not \"map\"" => {
            "The command target is not expressed in the map frame.".to_owned()
        }
        "State frame_id is not \"map\"" => {
            "The platform state is not expressed in the map frame.".to_owned()
        }
        other => other.to_owned(),
    }
}

fn readable_active_constraint_summary(raw: &str) -> String {
    let entries: Vec<_> = raw
        .split(',')
        .map(str::trim)
        .filter(|entry| !entry.is_empty())
        .map(readable_active_constraint_entry)
        .collect();

    if entries.is_empty() {
        "unknown constraint is active".to_owned()
    } else {
        entries.join(", ")
    }
}

fn readable_active_constraint_entry(entry: &str) -> String {
    let without_source = entry.split_once('/').map_or(entry, |(_, rest)| rest);
    let Some((kind, object)) = without_source.split_once('[') else {
        return format!("{} is active", humanize_code(without_source));
    };
    let object = object.trim_end_matches(']');
    let constraint_type = constraint_type_code_from_name(kind);

    active_constraint_entry_text(constraint_type, object)
}

fn constraint_type_code_from_name(name: &str) -> u8 {
    match name {
        "WALL" => constraint_type_wall(),
        "ROBOT_FAULT" => constraint_type_robot_fault(),
        "VIRTUAL_WALL" => constraint_type_virtual_wall(),
        "GOAL_BLOCKED" => constraint_type_goal_blocked(),
        _ => u8::MAX,
    }
}

fn manipulation_status_message(status: &ManipulationStatus) -> String {
    let phase = humanize_code(manipulation_phase_name(status.phase));
    if status.phase == manipulation_phase_failed() {
        let reason = manipulation_failure_text(status.failure_reason);
        format!(
            "Manipulation failed: {}. Reason: {}.",
            status.object_id, reason
        )
    } else {
        format!("Manipulation {phase}: {}.", status.object_id)
    }
}

fn manipulation_failure_text(failure_reason: u8) -> String {
    let failure_name = manipulation_failure_reason_name(failure_reason);
    if failure_reason == manipulation_failure_reason_no_object_detected() {
        "gripper reached fully closed without detecting object contact".to_owned()
    } else if failure_reason == manipulation_failure_reason_unsafe_grip_height() {
        "requested grip point is below the platform's safe floor clearance".to_owned()
    } else if failure_reason == manipulation_failure_reason_unspecified()
        || failure_name == "UNKNOWN"
    {
        "platform execution failure; inspect the platform runtime log".to_owned()
    } else {
        humanize_code(failure_name)
    }
}

fn object_tracker_events(
    previous: Option<&ObjectState>,
    current: &ObjectState,
) -> Vec<RuntimeEvent> {
    let mut events = Vec::new();

    let Some(previous) = previous else {
        events.push(object_tracker_pose_event(current, true));
        return events;
    };
    if previous.pose_valid != current.pose_valid {
        events.push(object_tracker_pose_event(current, false));
    }

    if previous.lifecycle != current.lifecycle {
        events.push(object_tracker_lifecycle_event(current));
    } else if current.lifecycle == object_lifecycle_held()
        && previous.holder_agent_id != current.holder_agent_id
    {
        events.push(object_tracker_holder_event(current));
    }

    events
}

fn object_tracker_pose_event(state: &ObjectState, first_sample: bool) -> RuntimeEvent {
    let (kind, message) = if state.pose_valid {
        let verb = if first_sample {
            "tracking"
        } else {
            "tracking reacquired"
        };
        (
            EventKind::Info,
            format!(
                "Object {} {verb} at {:.2} {:.2} {:.2}.",
                state.object_id, state.pose.x, state.pose.y, state.pose.z
            ),
        )
    } else {
        (
            EventKind::Warning,
            format!("Object {} tracking has no valid pose.", state.object_id),
        )
    };

    RuntimeEvent::new(kind, "object_tracker", message).with_stamp_ms(state.stamp_ms)
}

fn object_tracker_lifecycle_event(state: &ObjectState) -> RuntimeEvent {
    let message = if state.lifecycle == object_lifecycle_held() {
        format!(
            "Object {} picked up by {}.",
            state.object_id,
            object_holder_label(&state.holder_agent_id)
        )
    } else if state.lifecycle == object_lifecycle_free() {
        format!("Object {} released.", state.object_id)
    } else {
        format!(
            "Object {} lifecycle changed to {}.",
            state.object_id,
            humanize_code(object_lifecycle_name(state.lifecycle))
        )
    };

    RuntimeEvent::new(EventKind::Command, "object_tracker", message).with_stamp_ms(state.stamp_ms)
}

fn object_tracker_holder_event(state: &ObjectState) -> RuntimeEvent {
    RuntimeEvent::new(
        EventKind::Command,
        "object_tracker",
        format!(
            "Object {} holder changed to {}.",
            state.object_id,
            object_holder_label(&state.holder_agent_id)
        ),
    )
    .with_stamp_ms(state.stamp_ms)
}

fn object_holder_label(holder_agent_id: &str) -> String {
    if holder_agent_id.is_empty() {
        "no holder".to_owned()
    } else {
        agent_display_label(holder_agent_id)
    }
}

fn humanize_code(value: &str) -> String {
    value.replace('_', " ").to_ascii_lowercase()
}

fn command_sent_event_label(command: &command::OperatorCommand) -> String {
    match command {
        command::OperatorCommand::GoTo(x, y) => format!("Commanded to move to {x:.2} {y:.2}."),
        command::OperatorCommand::MoveAway(x, y) => {
            format!("Commanded to move away from {x:.2} {y:.2}.")
        }
        command::OperatorCommand::Hold => "Commanded to hold position.".to_owned(),
        command::OperatorCommand::PickUp(object_id) => {
            pickup_sent_event_label(object_id.as_deref().unwrap_or(command::DEFAULT_OBJECT_ID))
        }
        command::OperatorCommand::DropOff(object_id) => object_id.as_ref().map_or_else(
            || "Commanded to drop off the held object.".to_owned(),
            |object_id| format!("Commanded to drop off {object_id}."),
        ),
        command::OperatorCommand::ResumeAutonomy => "Commanded autonomy to resume.".to_owned(),
    }
}

fn command_goal_label(command: &command::OperatorCommand) -> Option<String> {
    match command {
        command::OperatorCommand::GoTo(x, y) | command::OperatorCommand::MoveAway(x, y) => {
            Some(format!("{x:.2} {y:.2}"))
        }
        command::OperatorCommand::Hold => Some(CURRENT_POSITION_GOAL.to_owned()),
        command::OperatorCommand::PickUp(..)
        | command::OperatorCommand::DropOff(..)
        | command::OperatorCommand::ResumeAutonomy => None,
    }
}

fn pickup_sent_event_label(object_id: &str) -> String {
    if object_id == command::DEFAULT_OBJECT_ID {
        "Commanded to pick up.".to_owned()
    } else {
        format!("Commanded to pick up {object_id}.")
    }
}

fn pickup_history_label(object_id: &str) -> String {
    if object_id == command::DEFAULT_OBJECT_ID {
        "commanded to pick up".to_owned()
    } else {
        format!("commanded to pick up {object_id}")
    }
}

fn command_history_label(command: &command::OperatorCommand) -> String {
    match command {
        command::OperatorCommand::GoTo(x, y) => format!("commanded to move to {x:.2} {y:.2}"),
        command::OperatorCommand::MoveAway(x, y) => {
            format!("commanded to move away from {x:.2} {y:.2}")
        }
        command::OperatorCommand::Hold => "commanded to hold position".to_owned(),
        command::OperatorCommand::PickUp(object_id) => {
            pickup_history_label(object_id.as_deref().unwrap_or(command::DEFAULT_OBJECT_ID))
        }
        command::OperatorCommand::DropOff(object_id) => object_id.as_ref().map_or_else(
            || "commanded to drop off held object".to_owned(),
            |object_id| format!("commanded to drop off {object_id}"),
        ),
        command::OperatorCommand::ResumeAutonomy => "commanded autonomy to resume".to_owned(),
    }
}

fn agent_display_label(agent_id: &str) -> String {
    if let Some(suffix) = agent_id.strip_prefix("agent_")
        && suffix.parse::<u32>().is_ok()
    {
        return format!("Agent {suffix}");
    }

    agent_id.to_owned()
}

fn compact_header_row<const N: usize>(cells: [&'static str; N]) -> Row<'static> {
    Row::new(cells).style(Style::default().add_modifier(Modifier::BOLD))
}

fn event_age_label(event: &RuntimeEvent) -> String {
    format!("{}ms", event.created_at.elapsed().as_millis())
}

#[derive(Clone, Copy)]
enum RuntimeTimestampFormat {
    TimeOnly,
    Compact,
}

struct RuntimeEventColumns {
    timestamp: usize,
    event: usize,
    timestamp_format: RuntimeTimestampFormat,
}

fn runtime_event_timestamp(event: &RuntimeEvent, format: RuntimeTimestampFormat) -> String {
    match format {
        RuntimeTimestampFormat::TimeOnly => {
            event.created_wall_time.format("%H:%M:%S%.3f").to_string()
        }
        RuntimeTimestampFormat::Compact => event.created_wall_time.format("%M:%S%.3f").to_string(),
    }
}

fn wall_time_from_stamp_ms(stamp_ms: u64) -> DateTime<Local> {
    Local
        .timestamp_millis_opt(stamp_ms.cast_signed())
        .single()
        .unwrap_or_else(Local::now)
}

fn push_recent_event(events: &mut VecDeque<RuntimeEvent>, event: RuntimeEvent) {
    if process_output_event(&event) {
        return;
    }

    events.push_front(event);
}

fn event_log_events(events: &VecDeque<RuntimeEvent>) -> Vec<&RuntimeEvent> {
    let mut selected = events
        .iter()
        .filter(|event| !process_output_event(event))
        .collect::<Vec<_>>();
    selected.reverse();
    selected.sort_by_key(|event| event.created_wall_time);
    selected
}

#[derive(Clone, Copy)]
struct EventLogScrollbar {
    area: Rect,
    body_line_count: usize,
    viewport_height: usize,
}

const fn event_log_text_area(area: Rect) -> Rect {
    if area.width > 1 {
        Rect {
            width: area.width - 1,
            ..area
        }
    } else {
        area
    }
}

fn event_log_scrollbar(
    area: Rect,
    body_line_count: usize,
    viewport_height: u16,
) -> EventLogScrollbar {
    EventLogScrollbar {
        area: event_log_scrollbar_area(area),
        body_line_count,
        viewport_height: viewport_height as usize,
    }
}

fn event_log_scrollbar_area(area: Rect) -> Rect {
    Rect {
        x: area.x.saturating_add(area.width.saturating_sub(1)),
        y: area.y,
        width: area.width.min(1),
        height: area.height,
    }
}

fn event_log_scroll_from_scrollbar_row(scrollbar: EventLogScrollbar, row: u16) -> usize {
    let max_scroll =
        runtime_event_max_scroll_for_body(scrollbar.body_line_count, scrollbar.viewport_height);
    if max_scroll == 0 || scrollbar.area.height <= 1 {
        return 0;
    }

    let row_offset = row.saturating_sub(scrollbar.area.y) as usize;
    let track_last = scrollbar.area.height.saturating_sub(1) as usize;
    let position_from_top = row_offset.min(track_last) * max_scroll / track_last.max(1);
    max_scroll.saturating_sub(position_from_top)
}

const fn rect_contains(area: Rect, column: u16, row: u16) -> bool {
    column >= area.x
        && column < area.x.saturating_add(area.width)
        && row >= area.y
        && row < area.y.saturating_add(area.height)
}

fn process_output_event(event: &RuntimeEvent) -> bool {
    event.source.ends_with(":stdout") || event.source.ends_with(":stderr")
}

fn activity_event_should_be_recorded(event: &RuntimeEvent) -> bool {
    !process_output_event(event)
        || matches!(event.kind, EventKind::Warning | EventKind::Error)
        || spot_sidecar_failure_detail(event).is_some()
}

fn spot_runtime_status(event: &RuntimeEvent) -> Option<&str> {
    event
        .source
        .starts_with("spot_sidecar:")
        .then(|| event.message.strip_prefix("spot-runtime: "))
        .flatten()
}

const SPOT_LEASE_CONFLICT_GUIDANCE: &str = "Spot body lease is owned by another controller. \
Keep the controller connected for E-stop, but set Robot Body Control to No Control before \
retrying. DCA did not take the lease.";
const SPOT_KEEPALIVE_MOTORS_OFF_GUIDANCE: &str = "Spot motor power is blocked by an active Keepalive safety policy. \
Stop or clear the owning tablet, mission, AutoReturn, or SDK client, then retry. \
DCA did not remove the policy.";

fn operator_facing_spot_failure(detail: &str) -> String {
    if detail.contains("ResourceAlreadyClaimedError") {
        SPOT_LEASE_CONFLICT_GUIDANCE.to_owned()
    } else if detail.contains("KeepaliveMotorsOffError") {
        SPOT_KEEPALIVE_MOTORS_OFF_GUIDANCE.to_owned()
    } else {
        detail.to_owned()
    }
}

fn spot_sidecar_failure_detail(event: &RuntimeEvent) -> Option<(u8, String)> {
    if !event.source.starts_with("spot_sidecar:") {
        return None;
    }
    if let Some(detail) = event.message.strip_prefix("FATAL ERROR: ") {
        return Some((3, operator_facing_spot_failure(detail)));
    }
    if event.source != "spot_sidecar:stderr" {
        return None;
    }

    let priority = if event.message.starts_with("[PYI-") {
        0
    } else if event.message.contains("Error:")
        || event.message.contains("Exception:")
        || event.message.starts_with("spot-runtime: unavailable:")
    {
        2
    } else {
        1
    };
    Some((priority, operator_facing_spot_failure(&event.message)))
}

fn duplicate_event_should_coalesce(event: &RuntimeEvent) -> bool {
    event.kind != EventKind::Error || diagnostic_event_is_visible(event)
}

const fn position_source_label(state: &AgentStateMsg) -> &'static str {
    if state.source == source_environment_runtime()
        || state.state_authority == state_authority_environment_runtime()
    {
        "environment runtime"
    } else if state.source == source_spot_adapter()
        || state.state_authority == state_authority_real()
    {
        "Spot ODM frame"
    } else {
        "unknown source"
    }
}

fn format_position(position: [f64; 3]) -> [String; 3] {
    position.map(format_coord)
}

fn format_coord(value: f64) -> String {
    format!("{value:.2}")
}

fn dash_position() -> [String; 3] {
    ["--"; 3].map(str::to_owned)
}

fn runtime_event_lines_with_scroll<'a>(
    events: impl Iterator<Item = &'a RuntimeEvent>,
    width: usize,
    height: usize,
    scroll_from_bottom: usize,
) -> Vec<Line<'static>> {
    if width == 0 || height == 0 {
        return Vec::new();
    }

    let columns = runtime_event_columns(width);
    let header_style = Style::default().add_modifier(Modifier::BOLD);
    let mut lines = vec![Line::from(vec![
        Span::styled(fit_column("Timestamp", columns.timestamp), header_style),
        Span::raw(" "),
        Span::styled("Event", header_style),
    ])];

    if height == 1 {
        return lines;
    }

    let mut body = Vec::new();
    for event in events {
        let style = Style::default();
        let timestamp = runtime_event_timestamp(event, columns.timestamp_format);
        let wrapped = wrap_text(&event.message, columns.event);
        for (index, message) in wrapped.into_iter().enumerate() {
            let timestamp = if index == 0 {
                timestamp.clone()
            } else {
                String::new()
            };
            body.push(Line::from(vec![
                Span::styled(fit_column(&timestamp, columns.timestamp), style),
                Span::styled(" ", style),
                Span::styled(message, style),
            ]));
        }
    }

    if body.is_empty() {
        body.push(Line::from("No events yet"));
    }

    let body_height = height.saturating_sub(1);
    let max_start = body.len().saturating_sub(body_height);
    let offset = scroll_from_bottom.min(max_start);
    let start = max_start.saturating_sub(offset);

    lines.extend(body.into_iter().skip(start).take(body_height));
    lines.truncate(height);
    lines
}

fn runtime_event_body_line_count<'a>(
    events: impl Iterator<Item = &'a RuntimeEvent>,
    width: usize,
) -> usize {
    if width == 0 {
        return 0;
    }

    let columns = runtime_event_columns(width);
    events
        .map(|event| wrap_text(&event.message, columns.event).len())
        .sum::<usize>()
        .max(1)
}

const fn runtime_event_max_scroll_for_body(body_line_count: usize, height: usize) -> usize {
    if height <= 1 {
        return 0;
    }

    body_line_count.saturating_sub(height.saturating_sub(1))
}

fn runtime_event_scroll_position(
    body_line_count: usize,
    height: usize,
    scroll_from_bottom: usize,
) -> usize {
    let max_scroll = runtime_event_max_scroll_for_body(body_line_count, height);
    max_scroll.saturating_sub(scroll_from_bottom.min(max_scroll))
}

fn runtime_event_columns(width: usize) -> RuntimeEventColumns {
    if width >= 48 {
        let timestamp = 12;
        return RuntimeEventColumns {
            timestamp,
            event: width.saturating_sub(timestamp + 1).max(1),
            timestamp_format: RuntimeTimestampFormat::TimeOnly,
        };
    }

    if width <= 10 {
        return RuntimeEventColumns {
            timestamp: width.saturating_sub(1),
            event: 1,
            timestamp_format: RuntimeTimestampFormat::Compact,
        };
    }

    let timestamp = 9.min(width.saturating_sub(2));
    RuntimeEventColumns {
        timestamp,
        event: width.saturating_sub(timestamp + 1).max(1),
        timestamp_format: RuntimeTimestampFormat::Compact,
    }
}

fn trim_diagnostics(events: &mut VecDeque<RuntimeEvent>) {
    events.retain(diagnostic_event_is_visible);

    events.truncate(MAX_DIAGNOSTIC_EVENTS);
}

fn diagnostic_event_is_visible(event: &RuntimeEvent) -> bool {
    event.created_at.elapsed() < DIAGNOSTIC_VISIBLE_DURATION
}

fn diagnostic_lines<'a>(
    events: impl Iterator<Item = &'a RuntimeEvent>,
    width: u16,
    height: u16,
) -> Vec<Line<'static>> {
    if width == 0 || height == 0 {
        return Vec::new();
    }

    let width = width as usize;
    let visible_events: Vec<_> = events.collect();
    if visible_events.is_empty() {
        return vec![Line::from(fit_column("no active errors", width))];
    }

    let mut lines = Vec::new();
    lines.push(diagnostic_header_line(width));

    for event in visible_events {
        for line in diagnostic_event_lines(event, width) {
            if lines.len() >= height as usize {
                return lines;
            }
            lines.push(line);
        }
    }

    lines.truncate(height as usize);
    lines
}

fn diagnostic_header_line(width: usize) -> Line<'static> {
    let style = Style::default().add_modifier(Modifier::BOLD);

    if let Some(columns) = diagnostic_columns(width) {
        return Line::from(Span::styled(
            format!(
                "{} {} {}",
                fit_column("Age", columns.age),
                fit_column("Source", columns.source),
                fit_column("Message", columns.message)
            ),
            style,
        ));
    }

    Line::from(Span::styled(fit_column("Age Source Message", width), style))
}

fn diagnostic_event_lines(event: &RuntimeEvent, width: usize) -> Vec<Line<'static>> {
    let style = Style::default();

    if let Some(columns) = diagnostic_columns(width) {
        return diagnostic_column_lines(event, columns, style);
    }

    let mut lines = Vec::new();
    let label = format!("{} {}", event_age_label(event), event.source);
    lines.push(Line::from(Span::styled(fit_column(&label, width), style)));

    for message_line in wrap_text(&event.message, width) {
        lines.push(Line::from(Span::styled(message_line, style)));
    }

    lines
}

fn diagnostic_column_lines(
    event: &RuntimeEvent,
    columns: DiagnosticColumns,
    style: Style,
) -> Vec<Line<'static>> {
    wrap_text(&event.message, columns.message)
        .into_iter()
        .enumerate()
        .map(|(index, message)| {
            let age = if index == 0 {
                event_age_label(event)
            } else {
                String::new()
            };
            let source = if index == 0 {
                event.source.clone()
            } else {
                String::new()
            };

            Line::from(Span::styled(
                format!(
                    "{} {} {}",
                    fit_column(&age, columns.age),
                    fit_column(&source, columns.source),
                    fit_column(&message, columns.message)
                ),
                style,
            ))
        })
        .collect()
}

#[derive(Clone, Copy)]
struct DiagnosticColumns {
    age: usize,
    source: usize,
    message: usize,
}

fn diagnostic_columns(width: usize) -> Option<DiagnosticColumns> {
    if width < 36 {
        return None;
    }

    let age = 8;
    let source = if width >= 64 {
        20
    } else if width >= 48 {
        16
    } else {
        12
    };
    let separators = 2;
    let message = width.saturating_sub(age + source + separators);

    Some(DiagnosticColumns {
        age,
        source,
        message: message.max(1),
    })
}

fn wrap_text(text: &str, width: usize) -> Vec<String> {
    if width == 0 {
        return Vec::new();
    }

    let mut lines = Vec::new();
    let mut current = String::new();

    for word in text.split_whitespace() {
        if word.chars().count() > width {
            if !current.is_empty() {
                lines.push(current);
                current = String::new();
            }
            lines.extend(split_long_word(word, width));
        } else if current.is_empty() {
            current.push_str(word);
        } else if current.chars().count() + 1 + word.chars().count() <= width {
            current.push(' ');
            current.push_str(word);
        } else {
            lines.push(std::mem::take(&mut current));
            word.clone_into(&mut current);
        }
    }

    if !current.is_empty() {
        lines.push(current);
    }

    if lines.is_empty() {
        lines.push(String::new());
    }

    lines
}

fn split_long_word(word: &str, width: usize) -> Vec<String> {
    word.chars()
        .collect::<Vec<_>>()
        .chunks(width)
        .map(String::from_iter)
        .collect()
}

fn fit_column(value: &str, width: usize) -> String {
    let fitted: String = value.chars().take(width).collect();
    format!("{fitted:<width$}")
}

#[cfg(test)]
#[path = "../../../tests/unit/operator_ui.rs"]
mod tests;
