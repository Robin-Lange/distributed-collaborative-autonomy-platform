//! Platform adapter for the onboard agent.
//!
//! The runtime consumes inputs from the shared-bus adapter and delivers
//! decision outputs back through it, keeping decision logic free of
//! transport-specific I/O.

use core_msgs::{
    AgentStateMsg, AuthorityState, BehaviorRequest, BehaviorStatus, CapabilityReport,
    CommandStatus, CommandVeto, ConstraintEvent, ControlRequest, ManipulationCommand,
    ManipulationIntent, ManipulationStatus, ObjectState, PlatformCommand, ResumeAutonomy,
    WaypointIntent, proto, topic_agent_state, topic_authority_state, topic_behavior_request,
    topic_behavior_status, topic_capability_report, topic_command_status, topic_command_veto,
    topic_constraint_event, topic_control_request, topic_manipulation_command,
    topic_manipulation_intent, topic_manipulation_status, topic_object_state,
    topic_platform_command, topic_resume_autonomy, topic_waypoint_intent,
};
use crossbeam_channel::{Receiver, Select, Sender, bounded};
use dds_support::{
    DDS_CHANNEL_CAPACITY, Session, declare_publisher, declare_safety_publisher, open_session,
    publish_available, publish_json_available, send_drop_oldest, send_or_log_drop,
    subscribe_forward, subscribe_forward_latest, subscribe_json_forward, subscribe_liveliness,
};
use std::thread;

/// A level-triggered output needs only its newest value while the bus thread
/// is occupied. Event outputs use the normal bounded event capacity below.
const LEVEL_OUTPUT_CAPACITY: usize = 1;

/// Output bundle produced by one decision tick.
pub struct PlatformOutput {
    /// Optional platform command to publish.
    command: Option<PlatformCommand>,

    /// Authority snapshot published on the runtime's heartbeat cadence.
    authority: AuthorityState,

    /// Optional veto message explaining a rejected command.
    veto: Option<CommandVeto>,

    /// Optional self-reported command completion (real-mode velocity
    /// tracking has no platform-side goal status, so the onboard agent
    /// publishes this itself).
    status: Option<CommandStatus>,
}

impl PlatformOutput {
    /// Creates a decision-output bundle.
    pub(super) const fn new(
        command: Option<PlatformCommand>,
        authority: AuthorityState,
        veto: Option<CommandVeto>,
        status: Option<CommandStatus>,
    ) -> Self {
        Self {
            command,
            authority,
            veto,
            status,
        }
    }

    /// Returns the optional platform command.
    pub(super) const fn command(&self) -> Option<&PlatformCommand> {
        self.command.as_ref()
    }

    /// Returns the authority snapshot.
    pub(super) const fn authority(&self) -> &AuthorityState {
        &self.authority
    }

    /// Returns the optional veto message.
    pub(super) const fn veto(&self) -> Option<&CommandVeto> {
        self.veto.as_ref()
    }

    /// Returns the optional self-reported command status.
    pub(super) const fn status(&self) -> Option<&CommandStatus> {
        self.status.as_ref()
    }
}

/// Error returned when an adapter can no longer accept output.
#[derive(Debug)]
pub struct AdapterDown {
    /// Human-readable adapter name for diagnostics.
    adapter: &'static str,
}

impl AdapterDown {
    /// Creates an adapter-disconnected error.
    pub(super) const fn new(adapter: &'static str) -> Self {
        Self { adapter }
    }

    /// Returns the disconnected adapter name.
    pub(super) const fn adapter_name(&self) -> &'static str {
        self.adapter
    }
}

/// Output bundle produced by one manipulation decision tick.
///
/// Kept separate from [`PlatformOutput`] rather than adding fields to it:
/// navigation's `veto` field is governed by `runtime.rs`'s persistent
/// active/clear tracking (a safety hold stays vetoed across ticks until it
/// resolves, then emits one explicit clear), while a manipulation
/// intent's accept/reject is a one-shot event with no equivalent "clear"
/// state to track.
pub struct ManipulationOutput {
    /// Optional approved manipulation command to publish.
    command: Option<ManipulationCommand>,

    /// Optional veto explaining a rejected manipulation intent.
    veto: Option<CommandVeto>,
}

impl ManipulationOutput {
    /// Creates a manipulation-output bundle.
    pub(super) const fn new(
        command: Option<ManipulationCommand>,
        veto: Option<CommandVeto>,
    ) -> Self {
        Self { command, veto }
    }

    /// Returns the optional approved command.
    pub(super) const fn command(&self) -> Option<&ManipulationCommand> {
        self.command.as_ref()
    }

    /// Returns the optional veto.
    pub(super) const fn veto(&self) -> Option<&CommandVeto> {
        self.veto.as_ref()
    }
}

/// Runtime input channels populated by the bus adapter.
pub struct PlatformInputs {
    /// Typed operator/autonomy behavior requests.
    behavior_requests: Receiver<BehaviorRequest>,

    /// Explicit operator commands releasing the autonomy pause latch.
    resume_autonomy: Receiver<ResumeAutonomy>,

    /// Operator/autonomy waypoint intents.
    intents: Receiver<WaypointIntent>,

    /// Agent state samples.
    states: Receiver<AgentStateMsg>,

    /// Constraint/safety events.
    constraints: Receiver<ConstraintEvent>,

    /// Platform command status events.
    statuses: Receiver<CommandStatus>,

    /// Operator/autonomy manipulation intents.
    manipulation_intents: Receiver<ManipulationIntent>,

    /// Manipulation execution status events from the executing platform.
    manipulation_statuses: Receiver<ManipulationStatus>,

    /// Catalog object lifecycle/pose state.
    object_states: Receiver<ObjectState>,

    /// Platform bridge liveliness: `true` when the token is alive, `false`
    /// when it was withdrawn (process/session death).
    platform_alive: Receiver<bool>,
}

impl PlatformInputs {
    pub(super) const fn behavior_request_rx(&self) -> &Receiver<BehaviorRequest> {
        &self.behavior_requests
    }

    pub(super) const fn resume_autonomy_rx(&self) -> &Receiver<ResumeAutonomy> {
        &self.resume_autonomy
    }

    /// Returns the intent receiver.
    pub(super) const fn intent_rx(&self) -> &Receiver<WaypointIntent> {
        &self.intents
    }

    /// Returns the state receiver.
    pub(super) const fn state_rx(&self) -> &Receiver<AgentStateMsg> {
        &self.states
    }

    /// Returns the constraint receiver.
    pub(super) const fn constraint_rx(&self) -> &Receiver<ConstraintEvent> {
        &self.constraints
    }

    /// Returns the command-status receiver.
    pub(super) const fn status_rx(&self) -> &Receiver<CommandStatus> {
        &self.statuses
    }

    /// Returns the manipulation-intent receiver.
    pub(super) const fn manipulation_intent_rx(&self) -> &Receiver<ManipulationIntent> {
        &self.manipulation_intents
    }

    /// Returns the manipulation-status receiver.
    pub(super) const fn manipulation_status_rx(&self) -> &Receiver<ManipulationStatus> {
        &self.manipulation_statuses
    }

    /// Returns the object-state receiver.
    pub(super) const fn object_state_rx(&self) -> &Receiver<ObjectState> {
        &self.object_states
    }

    /// Returns the platform-liveliness receiver.
    pub(super) const fn platform_alive_rx(&self) -> &Receiver<bool> {
        &self.platform_alive
    }
}

/// Spawns the bus adapter and returns it plus the runtime input channels.
///
/// `agent_id` identifies which platform bridge's liveliness this agent watches;
/// it is embedded in the liveliness key expression (tokens carry no payload).
pub fn spawn(agent_id: &str) -> (BusAdapter, PlatformInputs) {
    let (behavior_request_tx, behavior_request_rx) =
        bounded::<BehaviorRequest>(DDS_CHANNEL_CAPACITY);
    let (resume_autonomy_tx, resume_autonomy_rx) = bounded::<ResumeAutonomy>(DDS_CHANNEL_CAPACITY);
    let (intent_tx, intent_rx) = bounded::<WaypointIntent>(DDS_CHANNEL_CAPACITY);
    let (state_tx, state_rx) = bounded::<AgentStateMsg>(DDS_CHANNEL_CAPACITY);
    let (constraint_tx, constraint_rx) = bounded::<ConstraintEvent>(DDS_CHANNEL_CAPACITY);
    let (status_tx, status_rx) = bounded::<CommandStatus>(DDS_CHANNEL_CAPACITY);
    let (manipulation_intent_tx, manipulation_intent_rx) =
        bounded::<ManipulationIntent>(DDS_CHANNEL_CAPACITY);
    let (manipulation_status_tx, manipulation_status_rx) =
        bounded::<ManipulationStatus>(DDS_CHANNEL_CAPACITY);
    let (object_state_tx, object_state_rx) = bounded::<ObjectState>(DDS_CHANNEL_CAPACITY);
    let (platform_alive_tx, platform_alive_rx) = bounded::<bool>(DDS_CHANNEL_CAPACITY);
    // `state_tx` is a latest-wins stream: the bus thread evicts through this
    // clone instead of blocking or dropping the newest sample when full.
    let state_evict_rx = state_rx.clone();

    let bus = spawn_bus_adapter(
        RuntimeForwarding {
            behavior_request_tx,
            resume_autonomy_tx,
            intent_tx,
            state_tx,
            state_evict_rx,
            constraint_tx,
            status_tx,
            manipulation_intent_tx,
            manipulation_status_tx,
            object_state_tx,
            platform_alive_tx,
        },
        agent_id.to_owned(),
    );

    (
        bus,
        PlatformInputs {
            behavior_requests: behavior_request_rx,
            resume_autonomy: resume_autonomy_rx,
            intents: intent_rx,
            states: state_rx,
            constraints: constraint_rx,
            statuses: status_rx,
            manipulation_intents: manipulation_intent_rx,
            manipulation_statuses: manipulation_status_rx,
            object_states: object_state_rx,
            platform_alive: platform_alive_rx,
        },
    )
}

/// Adapter name used in disconnect diagnostics.
const BUS_ADAPTER_NAME: &str = "bus";

/// Foreground adapter handle used by the runtime to queue outgoing messages.
pub struct BusAdapter {
    /// Ordered navigation controls approved for execution by the platform.
    navigation_control_requests: Sender<ControlRequest>,

    /// One-shot manipulation controls approved for platform execution.
    manipulation_control_requests: Sender<ControlRequest>,

    /// Typed behavior lifecycle feedback.
    behavior_statuses: Sender<BehaviorStatus>,

    /// Static capabilities combined with current adapter availability.
    capability_reports: Sender<CapabilityReport>,

    /// Outgoing platform commands queued for bus publication.
    platform_commands: Sender<PlatformCommand>,

    /// Outgoing authority snapshots queued for bus publication.
    authority_snapshots: Sender<AuthorityState>,

    /// Eviction handle for the latest-wins authority-snapshot slot.
    authority_evict_rx: Receiver<AuthorityState>,

    /// Outgoing veto messages queued for bus publication. Shared by
    /// navigation and manipulation vetoes: both are `CommandVeto` on the
    /// same `command_veto` topic, distinguished downstream by `command_id`.
    vetoes: Sender<CommandVeto>,

    /// Outgoing manipulation commands queued for bus publication.
    manipulation_commands: Sender<ManipulationCommand>,

    /// Outgoing self-reported command statuses queued for bus publication
    /// (real-mode velocity-tracking completion; see [`RuntimeForwarding::status_tx`]
    /// for the corresponding subscribe side).
    statuses: Sender<CommandStatus>,
}

/// Channels through which the bus thread forwards incoming bus messages to
/// the runtime, plus the eviction handle for the latest-wins state stream.
pub struct RuntimeForwarding {
    /// Incoming typed behavior requests.
    pub(super) behavior_request_tx: Sender<BehaviorRequest>,

    /// Incoming explicit autonomy-resume commands.
    pub(super) resume_autonomy_tx: Sender<ResumeAutonomy>,

    /// Incoming intents forwarded from the bus to the runtime.
    pub(super) intent_tx: Sender<WaypointIntent>,

    /// Incoming state samples forwarded from the bus to the runtime.
    pub(super) state_tx: Sender<AgentStateMsg>,

    /// Clone of the runtime's state receiver, used only to evict the oldest
    /// queued sample when `state_tx` is full (latest-wins stream).
    pub(super) state_evict_rx: Receiver<AgentStateMsg>,

    /// Incoming constraints forwarded from the bus to the runtime.
    pub(super) constraint_tx: Sender<ConstraintEvent>,

    /// Incoming command statuses forwarded from the bus to the runtime.
    pub(super) status_tx: Sender<CommandStatus>,

    /// Incoming manipulation intents forwarded from the bus to the runtime.
    pub(super) manipulation_intent_tx: Sender<ManipulationIntent>,

    /// Incoming manipulation status updates forwarded to the runtime.
    pub(super) manipulation_status_tx: Sender<ManipulationStatus>,

    /// Incoming object lifecycle/pose state forwarded to the runtime.
    pub(super) object_state_tx: Sender<ObjectState>,

    /// Platform-bridge liveliness changes forwarded to the runtime (`true`
    /// alive, `false` withdrawn/dead).
    pub(super) platform_alive_tx: Sender<bool>,
}

/// Channel bundle moved into the background bus thread.
struct BusThreadChannels {
    /// Bus-to-runtime forwarding endpoints.
    forwarding: RuntimeForwarding,

    /// Agent whose platform-bridge liveliness this thread subscribes to.
    agent_id: String,

    /// Typed behavior lifecycle updates waiting for publication.
    behavior_status_rx: Receiver<BehaviorStatus>,

    /// Capability and availability reports waiting for publication.
    capability_report_rx: Receiver<CapabilityReport>,

    /// Ordered navigation controls waiting for publication.
    navigation_control_request_rx: Receiver<ControlRequest>,

    /// One-shot manipulation controls waiting for publication.
    manipulation_control_request_rx: Receiver<ControlRequest>,

    /// Runtime commands waiting to be published on the bus.
    command_rx: Receiver<PlatformCommand>,

    /// Runtime authority snapshots waiting to be published on the bus.
    authority_rx: Receiver<AuthorityState>,

    /// Runtime vetoes waiting to be published on the bus.
    veto_rx: Receiver<CommandVeto>,

    /// Runtime manipulation commands waiting to be published on the bus.
    manipulation_command_rx: Receiver<ManipulationCommand>,

    /// Runtime self-reported command statuses waiting to be published on the bus.
    status_rx: Receiver<CommandStatus>,
}

impl BusAdapter {
    /// Publishes one behavior lifecycle transition.
    pub(super) fn deliver_behavior_status(
        &self,
        status: BehaviorStatus,
    ) -> Result<(), AdapterDown> {
        publish(&self.behavior_statuses, topic_behavior_status(), status)
    }

    /// Publishes one capability and availability snapshot.
    pub(super) fn deliver_capability_report(
        &self,
        report: CapabilityReport,
    ) -> Result<(), AdapterDown> {
        publish(&self.capability_reports, topic_capability_report(), report)
    }

    /// Publishes one navigation control approved by the local agent.
    pub(super) fn deliver_navigation_control_request(
        &self,
        request: ControlRequest,
    ) -> Result<(), AdapterDown> {
        publish(
            &self.navigation_control_requests,
            topic_control_request(),
            request,
        )
    }

    /// Publishes one manipulation control approved by the local agent.
    pub(super) fn deliver_manipulation_control_request(
        &self,
        request: ControlRequest,
    ) -> Result<(), AdapterDown> {
        publish(
            &self.manipulation_control_requests,
            topic_control_request(),
            request,
        )
    }

    /// Delivers one decision-output bundle onto the bus.
    pub(super) fn deliver(
        &self,
        output: &PlatformOutput,
        publish_authority: bool,
    ) -> Result<(), AdapterDown> {
        if publish_authority {
            publish_latest(
                &self.authority_snapshots,
                &self.authority_evict_rx,
                output.authority().clone(),
            )?;
        }

        if let Some(command) = output.command() {
            publish(
                &self.platform_commands,
                topic_platform_command(),
                command.clone(),
            )?;
        }

        if let Some(veto) = output.veto() {
            publish(&self.vetoes, topic_command_veto(), veto.clone())?;
        }

        if let Some(status) = output.status() {
            publish(&self.statuses, topic_command_status(), status.clone())?;
        }

        Ok(())
    }

    /// Delivers one standalone veto onto the shared `command_veto` topic.
    ///
    /// Used for the supersede signal: the per-tick [`PlatformOutput`] carries a
    /// single veto slot already owned by the current intent's decision, so a
    /// veto for a *superseded* prior command needs its own emission. It is a
    /// one-shot event (like a manipulation veto) with no active/clear tracking.
    pub(super) fn deliver_veto(&self, veto: &CommandVeto) -> Result<(), AdapterDown> {
        publish(&self.vetoes, topic_command_veto(), veto.clone())
    }

    /// Delivers one manipulation-output bundle onto the bus.
    pub(super) fn deliver_manipulation(
        &self,
        output: &ManipulationOutput,
    ) -> Result<(), AdapterDown> {
        if let Some(command) = output.command() {
            publish(
                &self.manipulation_commands,
                topic_manipulation_command(),
                command.clone(),
            )?;
        }

        if let Some(veto) = output.veto() {
            publish(&self.vetoes, topic_command_veto(), veto.clone())?;
        }

        Ok(())
    }
}

/// Queues one outgoing message or reports that the bus thread has stopped.
fn publish<T>(tx: &Sender<T>, topic: &'static str, message: T) -> Result<(), AdapterDown> {
    if send_or_log_drop(tx, topic, message) {
        Ok(())
    } else {
        Err(AdapterDown::new(BUS_ADAPTER_NAME))
    }
}

/// Replaces a queued level-triggered value or reports that the bus thread stopped.
fn publish_latest<T>(
    tx: &Sender<T>,
    evict_rx: &Receiver<T>,
    message: T,
) -> Result<(), AdapterDown> {
    if send_drop_oldest(tx, evict_rx, message) {
        Ok(())
    } else {
        Err(AdapterDown::new(BUS_ADAPTER_NAME))
    }
}

/// Spawns the bus thread and returns the adapter handle used by the runtime.
fn spawn_bus_adapter(forwarding: RuntimeForwarding, agent_id: String) -> BusAdapter {
    let (behavior_status_tx, behavior_status_rx) = bounded::<BehaviorStatus>(DDS_CHANNEL_CAPACITY);
    let (capability_report_tx, capability_report_rx) =
        bounded::<CapabilityReport>(DDS_CHANNEL_CAPACITY);
    let (navigation_control_request_tx, navigation_control_request_rx) =
        bounded::<ControlRequest>(DDS_CHANNEL_CAPACITY);
    let (manipulation_control_request_tx, manipulation_control_request_rx) =
        bounded::<ControlRequest>(DDS_CHANNEL_CAPACITY);
    let (command_tx, command_rx) = bounded::<PlatformCommand>(DDS_CHANNEL_CAPACITY);
    let (authority_tx, authority_rx) = bounded::<AuthorityState>(LEVEL_OUTPUT_CAPACITY);
    let authority_evict_rx = authority_rx.clone();
    let (veto_tx, veto_rx) = bounded::<CommandVeto>(DDS_CHANNEL_CAPACITY);
    let (manipulation_command_tx, manipulation_command_rx) =
        bounded::<ManipulationCommand>(DDS_CHANNEL_CAPACITY);
    let (status_tx, status_rx) = bounded::<CommandStatus>(DDS_CHANNEL_CAPACITY);

    thread::spawn(move || {
        run_bus_thread(&BusThreadChannels {
            forwarding,
            agent_id,
            behavior_status_rx,
            capability_report_rx,
            navigation_control_request_rx,
            manipulation_control_request_rx,
            command_rx,
            authority_rx,
            veto_rx,
            manipulation_command_rx,
            status_rx,
        });
    });

    BusAdapter {
        navigation_control_requests: navigation_control_request_tx,
        manipulation_control_requests: manipulation_control_request_tx,
        behavior_statuses: behavior_status_tx,
        capability_reports: capability_report_tx,
        platform_commands: command_tx,
        authority_snapshots: authority_tx,
        authority_evict_rx,
        vetoes: veto_tx,
        manipulation_commands: manipulation_command_tx,
        statuses: status_tx,
    }
}

/// Runs the blocking bus polling/publishing loop.
fn run_bus_thread(channels: &BusThreadChannels) {
    let session = open_session();

    let _behavior_request_sub = subscribe_json_forward(
        &session,
        topic_behavior_request(),
        &channels.forwarding.behavior_request_tx,
    );
    let _resume_autonomy_sub = subscribe_json_forward(
        &session,
        topic_resume_autonomy(),
        &channels.forwarding.resume_autonomy_tx,
    );
    let _intent_sub = subscribe_forward(
        &session,
        topic_waypoint_intent(),
        &channels.forwarding.intent_tx,
        |message: proto::WaypointIntent| message.into(),
    );
    let _state_sub = subscribe_forward_latest(
        &session,
        topic_agent_state(),
        &channels.forwarding.state_tx,
        &channels.forwarding.state_evict_rx,
        |message: proto::AgentStateMsg| message.into(),
    );
    let _constraint_sub = subscribe_forward(
        &session,
        topic_constraint_event(),
        &channels.forwarding.constraint_tx,
        |message: proto::ConstraintEvent| message.into(),
    );
    let _status_sub = subscribe_forward(
        &session,
        topic_command_status(),
        &channels.forwarding.status_tx,
        |message: proto::CommandStatus| message.into(),
    );
    let _manipulation_intent_sub = subscribe_forward(
        &session,
        topic_manipulation_intent(),
        &channels.forwarding.manipulation_intent_tx,
        |message: proto::ManipulationIntent| message.into(),
    );
    let _manipulation_status_sub = subscribe_forward(
        &session,
        topic_manipulation_status(),
        &channels.forwarding.manipulation_status_tx,
        |message: proto::ManipulationStatus| message.into(),
    );
    let _object_state_sub = subscribe_forward(
        &session,
        topic_object_state(),
        &channels.forwarding.object_state_tx,
        |message: proto::ObjectState| message.into(),
    );
    // Event-based death detection for the platform bridge: a `Delete` (token
    // withdrawn on process/session death) forwards `false` instantly, ahead of
    // the state-staleness fallback.
    let platform_alive_tx = channels.forwarding.platform_alive_tx.clone();
    let _platform_alive_sub = subscribe_liveliness(
        &session,
        &format!("liveliness/agent_state/{}", channels.agent_id),
        move |alive| {
            let _ = platform_alive_tx.try_send(alive);
        },
    );

    publish_bus_outputs(&session, channels);
}

/// Blocks on outbound channels and publishes every queued agent output.
fn publish_bus_outputs(session: &Session, channels: &BusThreadChannels) -> ! {
    let command_publisher = declare_safety_publisher(session, topic_platform_command());
    let behavior_status_publisher = declare_publisher(session, topic_behavior_status());
    let capability_report_publisher = declare_publisher(session, topic_capability_report());
    let control_request_publisher = declare_safety_publisher(session, topic_control_request());
    let authority_publisher = declare_publisher(session, topic_authority_state());
    let veto_publisher = declare_publisher(session, topic_command_veto());
    let manipulation_command_publisher = declare_publisher(session, topic_manipulation_command());
    let status_publisher = declare_publisher(session, topic_command_status());

    // Event-driven outbound publish: block until any outgoing channel has a
    // message instead of sleep-polling 5 ms latency onto every command. The
    // senders live for the process lifetime (the runtime exits the process on
    // adapter failure), so a disconnected channel can't busy-wake this loop.
    let mut outgoing_ready = Select::new();
    outgoing_ready.recv(&channels.behavior_status_rx);
    outgoing_ready.recv(&channels.capability_report_rx);
    outgoing_ready.recv(&channels.navigation_control_request_rx);
    outgoing_ready.recv(&channels.manipulation_control_request_rx);
    outgoing_ready.recv(&channels.command_rx);
    outgoing_ready.recv(&channels.authority_rx);
    outgoing_ready.recv(&channels.veto_rx);
    outgoing_ready.recv(&channels.manipulation_command_rx);
    outgoing_ready.recv(&channels.status_rx);

    loop {
        let _ = outgoing_ready.ready();
        publish_json_available(
            &behavior_status_publisher,
            topic_behavior_status(),
            &channels.behavior_status_rx,
        );
        publish_json_available(
            &capability_report_publisher,
            topic_capability_report(),
            &channels.capability_report_rx,
        );
        publish_json_available(
            &control_request_publisher,
            topic_control_request(),
            &channels.navigation_control_request_rx,
        );
        // Navigation controls are drained first so a safety-clear hold queued
        // by the navigation tick reaches the executor before a manipulation
        // request admitted later in that same runtime tick.
        publish_available(
            &manipulation_command_publisher,
            topic_manipulation_command(),
            &channels.manipulation_command_rx,
            proto::ManipulationCommand::from,
        );
        publish_json_available(
            &control_request_publisher,
            topic_control_request(),
            &channels.manipulation_control_request_rx,
        );
        // Drain crossbeam outgoing channels and put onto Zenoh
        publish_available(
            &command_publisher,
            topic_platform_command(),
            &channels.command_rx,
            proto::PlatformCommand::from,
        );
        publish_available(
            &authority_publisher,
            topic_authority_state(),
            &channels.authority_rx,
            proto::AuthorityState::from,
        );
        publish_available(
            &veto_publisher,
            topic_command_veto(),
            &channels.veto_rx,
            proto::CommandVeto::from,
        );
        publish_available(
            &status_publisher,
            topic_command_status(),
            &channels.status_rx,
            proto::CommandStatus::from,
        );
    }
}
