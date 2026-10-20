//! Shared operator bus transport.
//!
//! This module is intentionally UI-neutral: it owns the Zenoh topic setup and
//! exposes simple channels for any operator surface that needs to publish
//! waypoint intents or observe autonomy feedback. Operator surfaces depend on
//! this layer instead of depending on each other.
//!
//! Direction of data flow:
//! - An operator surface sends typed behavior or autonomy-resume requests.
//! - The background bus thread publishes them onto the shared topics.
//! - The background bus thread subscribes to state, command, constraint, and
//!   veto topics.
//! - The operator surface receives incoming messages through the exposed
//!   receivers.

use crate::core_msgs::{
    AgentStateMsg, BehaviorRequest, BehaviorStatus, CapabilityReport, CommandStatus, CommandVeto,
    ConstraintEvent, ManipulationStatus, ObjectState, PlatformCommand, ResumeAutonomy,
    dds_poll_interval_ms, proto, topic_agent_state, topic_behavior_request, topic_behavior_status,
    topic_capability_report, topic_command_status, topic_command_veto, topic_constraint_event,
    topic_manipulation_status, topic_mocap_rigid_body, topic_object_state, topic_platform_command,
    topic_resume_autonomy,
};
use crate::dds_support::{
    DDS_CHANNEL_CAPACITY, declare_publisher, open_session, publish_json_available,
    subscribe_forward, subscribe_forward_latest, subscribe_json_forward,
};
use crossbeam_channel::{Receiver, Sender, bounded};
use std::{thread, time::Duration};

/// Channel bundle used by operator surfaces to communicate with the bus thread.
///
/// Operator surfaces send outgoing requests through the two sender fields.
/// Everything else is received from the bus and forwarded through read-only
/// receivers.
pub struct OperatorBusIo {
    pub(crate) behavior_request_tx: Sender<BehaviorRequest>,
    pub(crate) resume_autonomy_tx: Sender<ResumeAutonomy>,
    pub(crate) behavior_status_rx: Receiver<BehaviorStatus>,
    pub(crate) capability_report_rx: Receiver<CapabilityReport>,
    pub(crate) state_rx: Receiver<AgentStateMsg>,
    pub(crate) command_rx: Receiver<PlatformCommand>,
    pub(crate) status_rx: Receiver<CommandStatus>,
    pub(crate) constraint_rx: Receiver<ConstraintEvent>,
    pub(crate) veto_rx: Receiver<CommandVeto>,
    pub(crate) manipulation_status_rx: Receiver<ManipulationStatus>,
    pub(crate) object_state_rx: Receiver<ObjectState>,
    pub(crate) mocap_rx: Receiver<proto::MocapRigidBodyFrame>,
}

/// Starts the operator bus bridge and returns channel endpoints for a UI.
///
/// This function creates one channel per message stream, then spawns a dedicated
/// background thread that owns the DDS session. The foreground CLI can therefore
/// remain focused on operator interaction and feedback, while the
/// background thread continuously polls DDS and publishes queued intents.
///
/// The bus thread is detached and loops for the lifetime of the process:
/// dropping the returned `OperatorBusIo` closes the channels but does not stop
/// the thread.
pub fn spawn_operator_bus_io() -> OperatorBusIo {
    let (io, txs, rxs) = create_channels();

    // The DDS session and topic handles stay inside this thread. That keeps DDS
    // lifetime/threading complexity contained to a single non-blocking stream.
    thread::spawn(move || run_bus_thread(&txs, &rxs));

    io
}

struct BusThreadSenders {
    behavior_status: Sender<BehaviorStatus>,
    capability_report: Sender<CapabilityReport>,
    state: Sender<AgentStateMsg>,
    command: Sender<PlatformCommand>,
    status: Sender<CommandStatus>,
    constraint: Sender<ConstraintEvent>,
    veto: Sender<CommandVeto>,
    manipulation_status: Sender<ManipulationStatus>,
    object_state: Sender<ObjectState>,
    mocap: Sender<proto::MocapRigidBodyFrame>,
}

struct BusThreadReceivers {
    behavior_request: Receiver<BehaviorRequest>,
    resume_autonomy: Receiver<ResumeAutonomy>,
    state_evict: Receiver<AgentStateMsg>,
    command_evict: Receiver<PlatformCommand>,
    mocap_evict: Receiver<proto::MocapRigidBodyFrame>,
}

fn create_channels() -> (OperatorBusIo, BusThreadSenders, BusThreadReceivers) {
    // Outgoing operator command stream: foreground UI -> background bus publisher.
    let (behavior_request_tx, behavior_request_rx) =
        bounded::<BehaviorRequest>(DDS_CHANNEL_CAPACITY);
    let (resume_autonomy_tx, resume_autonomy_rx) = bounded::<ResumeAutonomy>(DDS_CHANNEL_CAPACITY);

    // Incoming bus streams: background subscribers -> foreground UI.
    let (behavior_status_tx, behavior_status_rx) = bounded::<BehaviorStatus>(DDS_CHANNEL_CAPACITY);
    let (capability_report_tx, capability_report_rx) =
        bounded::<CapabilityReport>(DDS_CHANNEL_CAPACITY);
    let (state_tx, state_rx) = bounded::<AgentStateMsg>(DDS_CHANNEL_CAPACITY);
    let (command_tx, command_rx) = bounded::<PlatformCommand>(DDS_CHANNEL_CAPACITY);
    let (status_tx, status_rx) = bounded::<CommandStatus>(DDS_CHANNEL_CAPACITY);
    let (constraint_tx, constraint_rx) = bounded::<ConstraintEvent>(DDS_CHANNEL_CAPACITY);
    let (veto_tx, veto_rx) = bounded::<CommandVeto>(DDS_CHANNEL_CAPACITY);
    let (manipulation_status_tx, manipulation_status_rx) =
        bounded::<ManipulationStatus>(DDS_CHANNEL_CAPACITY);
    let (object_state_tx, object_state_rx) = bounded::<ObjectState>(DDS_CHANNEL_CAPACITY);
    let (mocap_tx, mocap_rx) = bounded::<proto::MocapRigidBodyFrame>(DDS_CHANNEL_CAPACITY);

    // `state_tx` is a latest-wins stream: evict the oldest queued sample
    // through this clone when full instead of falling behind.
    let state_evict_rx = state_rx.clone();
    let command_evict_rx = command_rx.clone();
    let mocap_evict_rx = mocap_rx.clone();

    let io = OperatorBusIo {
        behavior_request_tx,
        resume_autonomy_tx,
        behavior_status_rx,
        capability_report_rx,
        state_rx,
        command_rx,
        status_rx,
        constraint_rx,
        veto_rx,
        manipulation_status_rx,
        object_state_rx,
        mocap_rx,
    };

    let txs = BusThreadSenders {
        behavior_status: behavior_status_tx,
        capability_report: capability_report_tx,
        state: state_tx,
        command: command_tx,
        status: status_tx,
        constraint: constraint_tx,
        veto: veto_tx,
        manipulation_status: manipulation_status_tx,
        object_state: object_state_tx,
        mocap: mocap_tx,
    };

    let rxs = BusThreadReceivers {
        behavior_request: behavior_request_rx,
        resume_autonomy: resume_autonomy_rx,
        state_evict: state_evict_rx,
        command_evict: command_evict_rx,
        mocap_evict: mocap_evict_rx,
    };

    (io, txs, rxs)
}

fn run_bus_thread(txs: &BusThreadSenders, rxs: &BusThreadReceivers) {
    let session = open_session();

    // Publisher used for operator waypoint intents from any UI surface.
    let behavior_request_publisher = declare_publisher(&session, topic_behavior_request());
    let resume_autonomy_publisher = declare_publisher(&session, topic_resume_autonomy());

    // Subscribers for the message streams the CLI displays or reacts to.
    let _behavior_status_sub =
        subscribe_json_forward(&session, topic_behavior_status(), &txs.behavior_status);
    let _capability_report_sub =
        subscribe_json_forward(&session, topic_capability_report(), &txs.capability_report);
    let _state_sub = subscribe_forward_latest(
        &session,
        topic_agent_state(),
        &txs.state,
        &rxs.state_evict,
        |message: proto::AgentStateMsg| message.into(),
    );
    let _command_sub = subscribe_forward_latest(
        &session,
        topic_platform_command(),
        &txs.command,
        &rxs.command_evict,
        |message: proto::PlatformCommand| message.into(),
    );
    let _status_sub = subscribe_forward(
        &session,
        topic_command_status(),
        &txs.status,
        |message: proto::CommandStatus| message.into(),
    );
    let _constraint_sub = subscribe_forward(
        &session,
        topic_constraint_event(),
        &txs.constraint,
        |message: proto::ConstraintEvent| message.into(),
    );
    let _veto_sub = subscribe_forward(
        &session,
        topic_command_veto(),
        &txs.veto,
        |message: proto::CommandVeto| message.into(),
    );
    let _manipulation_status_sub = subscribe_forward(
        &session,
        topic_manipulation_status(),
        &txs.manipulation_status,
        |message: proto::ManipulationStatus| message.into(),
    );
    let _object_state_sub = subscribe_forward(
        &session,
        topic_object_state(),
        &txs.object_state,
        |message: proto::ObjectState| message.into(),
    );
    let _mocap_sub = subscribe_forward_latest(
        &session,
        topic_mocap_rigid_body(),
        &txs.mocap,
        &rxs.mocap_evict,
        |message: proto::MocapRigidBodyFrame| message,
    );

    loop {
        publish_json_available(
            &behavior_request_publisher,
            topic_behavior_request(),
            &rxs.behavior_request,
        );
        publish_json_available(
            &resume_autonomy_publisher,
            topic_resume_autonomy(),
            &rxs.resume_autonomy,
        );

        // Avoid a tight spin loop while still polling often enough for
        // responsive operator feedback.
        thread::sleep(Duration::from_millis(dds_poll_interval_ms()));
    }
}
