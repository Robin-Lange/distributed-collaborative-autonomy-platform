//! DDS/Zenoh bridge for the environment simulator.
//!
//! The shared bus frame is the canonical `map` frame (ENU); Bevy's internal
//! physics/render frame is Y-up. This thread only moves protobuf bytes and
//! internal message structs across channels; the `map` <-> Bevy coordinate
//! conversion happens at the Bevy-system boundary (`world::FrameConversion`),
//! not here.

use super::world::EnvironmentChannels;
use core_msgs::{
    AgentStateMsg, ConstraintEvent, ControlAction, ControlRequest, ControlSource,
    ManipulationCommand, ManipulationStatus, ObjectState, PlatformCommand, PositionTarget,
    VelocitySetpoint, command_source_autonomy, command_source_human_operator, command_source_none,
    command_type_hold, command_type_stop, command_type_velocity, dds_poll_interval_ms,
    manipulation_grasp_profile_top_down, manipulation_verb_drop_off, manipulation_verb_pick_up,
    proto, state_authority_real, topic_agent_state, topic_constraint_event, topic_control_request,
    topic_manipulation_status, topic_object_state,
};
use crossbeam_channel::{Receiver, Sender, bounded};
use dds_support::{
    DDS_CHANNEL_CAPACITY, declare_liveliness_token, declare_publisher, declare_safety_publisher,
    open_session, publish_available, send_drop_oldest, send_or_log_drop, subscribe_for_each,
    subscribe_forward, subscribe_json_for_each,
};
use std::{thread, time::Duration};

/// Channel bundle used to connect Bevy systems with the bus thread.
pub struct EnvironmentDdsIo {
    /// Platform commands received from the autonomy stack.
    command_rx: Receiver<PlatformCommand>,

    /// Simulated state samples produced by `sim` role systems.
    state_tx: Sender<AgentStateMsg>,

    /// Clone of the DDS thread's state receiver, used only to evict the
    /// oldest queued sample when `state_tx` is full (latest-wins stream).
    state_evict_rx: Receiver<AgentStateMsg>,

    /// Constraint events produced by wall/obstacle systems.
    constraint_tx: Sender<ConstraintEvent>,

    /// Real-authority state samples consumed by `hybrid` role systems.
    hybrid_state_rx: Receiver<AgentStateMsg>,

    /// Approved manipulation commands received from the onboard agent.
    manipulation_command_rx: Receiver<ManipulationCommand>,

    /// Manipulation execution status messages produced by `sim` role.
    manipulation_status_tx: Sender<ManipulationStatus>,

    /// Manipulation execution status messages published externally by the
    /// real-Spot sidecar, consumed by `hybrid` role (the sidecar, not this
    /// process, executes the arm sequence in hybrid; hybrid role only owns
    /// object-state truth, applying attach/detach once the sidecar reports a
    /// terminal status for a command this process latched).
    manipulation_status_rx: Receiver<ManipulationStatus>,

    /// Catalog object lifecycle/pose state produced by `sim` role.
    object_state_tx: Sender<ObjectState>,
}

impl EnvironmentDdsIo {
    /// Converts the DDS handle bundle into the ECS channel resource.
    pub(super) fn into_channels(self) -> EnvironmentChannels {
        EnvironmentChannels {
            command_rx: self.command_rx,
            state_tx: self.state_tx,
            state_evict_rx: self.state_evict_rx,
            constraint_tx: self.constraint_tx,
            hybrid_state_rx: self.hybrid_state_rx,
            manipulation_command_rx: self.manipulation_command_rx,
            manipulation_status_tx: self.manipulation_status_tx,
            manipulation_status_rx: self.manipulation_status_rx,
            object_state_tx: self.object_state_tx,
        }
    }
}

/// Channel endpoints moved into the background bus thread.
struct BusThreadChannels {
    /// Simulated agents whose platform availability this process owns.
    agent_ids: Vec<String>,

    /// Approved platform commands forwarded to Bevy systems.
    command_tx: Sender<PlatformCommand>,

    /// Approved manipulation commands forwarded to Bevy systems.
    manipulation_command_tx: Sender<ManipulationCommand>,

    /// Externally published manipulation statuses forwarded to `hybrid` role.
    manipulation_status_sub_tx: Sender<ManipulationStatus>,

    /// Real-authority state samples forwarded to `hybrid` role (latest-wins).
    hybrid_state_tx: Sender<AgentStateMsg>,

    /// Eviction handle for the latest-wins hybrid-state stream.
    hybrid_evict_rx: Receiver<AgentStateMsg>,

    /// Simulated state samples queued by `sim` role for publication.
    state_rx: Receiver<AgentStateMsg>,

    /// Constraint events queued by contact systems for publication.
    constraint_rx: Receiver<ConstraintEvent>,

    /// Manipulation statuses queued by `sim` role for publication.
    manipulation_status_publish_rx: Receiver<ManipulationStatus>,

    /// Object lifecycle/pose states queued by `sim` role for publication.
    object_state_rx: Receiver<ObjectState>,
}

/// Starts the simulator's background DDS bus thread.
///
/// The thread owns all Zenoh publishers/subscribers and communicates with Bevy
/// systems through channels. This keeps middleware polling out of the fixed
/// physics schedule.
pub fn spawn_environment_dds_io(agent_ids: Vec<String>) -> EnvironmentDdsIo {
    let (command_tx, command_rx) = bounded::<PlatformCommand>(DDS_CHANNEL_CAPACITY);
    let (state_tx, state_rx) = bounded::<AgentStateMsg>(DDS_CHANNEL_CAPACITY);
    let (constraint_tx, constraint_rx) = bounded::<ConstraintEvent>(DDS_CHANNEL_CAPACITY);
    let (hybrid_state_tx, hybrid_state_rx) = bounded::<AgentStateMsg>(DDS_CHANNEL_CAPACITY);
    let (manipulation_command_tx, manipulation_command_rx) =
        bounded::<ManipulationCommand>(DDS_CHANNEL_CAPACITY);
    let (manipulation_status_tx, manipulation_status_publish_rx) =
        bounded::<ManipulationStatus>(DDS_CHANNEL_CAPACITY);
    let (manipulation_status_sub_tx, manipulation_status_rx) =
        bounded::<ManipulationStatus>(DDS_CHANNEL_CAPACITY);
    let (object_state_tx, object_state_rx) = bounded::<ObjectState>(DDS_CHANNEL_CAPACITY);
    // Both `state_tx` and `hybrid_state_tx` are latest-wins streams; these
    // clones let their senders evict the oldest queued sample when full
    // instead of blocking this thread or falling behind by a full backlog.
    let state_evict_rx = state_rx.clone();
    let hybrid_evict_rx = hybrid_state_rx.clone();

    let thread_channels = BusThreadChannels {
        agent_ids,
        command_tx,
        manipulation_command_tx,
        manipulation_status_sub_tx,
        hybrid_state_tx,
        hybrid_evict_rx,
        state_rx,
        constraint_rx,
        manipulation_status_publish_rx,
        object_state_rx,
    };
    thread::spawn(move || run_bus_thread(&thread_channels));

    EnvironmentDdsIo {
        command_rx,
        state_tx,
        state_evict_rx,
        constraint_tx,
        hybrid_state_rx,
        manipulation_command_rx,
        manipulation_status_tx,
        manipulation_status_rx,
        object_state_tx,
    }
}

/// Runs the blocking bus polling/publishing loop.
fn run_bus_thread(channels: &BusThreadChannels) {
    let session = open_session();
    let _liveliness_tokens: Vec<_> = channels
        .agent_ids
        .iter()
        .map(|agent_id| {
            declare_liveliness_token(&session, &format!("liveliness/agent_state/{agent_id}"))
        })
        .collect();

    let command_tx = channels.command_tx.clone();
    let manipulation_command_tx = channels.manipulation_command_tx.clone();
    let _control_request_sub = subscribe_json_for_each(
        &session,
        topic_control_request(),
        move |request: ControlRequest| match route_control_request(&request) {
            SimControl::Drive(command) => {
                send_or_log_drop(&command_tx, topic_control_request(), command);
            }
            SimControl::Manipulate(command) => {
                send_or_log_drop(&manipulation_command_tx, topic_control_request(), command);
            }
            SimControl::Unsupported(detail) => {
                eprintln!("environment_sim: dropped unsupported control: {detail}");
            }
        },
    );
    // `hybrid` role does not execute the arm sequence itself (the
    // real-Spot sidecar does); it needs the sidecar's own published
    // status to know when to attach/detach the object. `sim` role never
    // receives anything on this subscription (nothing else publishes
    // `manipulation_status` in sim mode), so declaring it unconditionally
    // is harmless.
    let _manipulation_status_sub = subscribe_forward(
        &session,
        topic_manipulation_status(),
        &channels.manipulation_status_sub_tx,
        |message: proto::ManipulationStatus| message.into(),
    );

    let tx_clone = channels.hybrid_state_tx.clone();
    let evict_rx_clone = channels.hybrid_evict_rx.clone();
    let _state_sub = subscribe_for_each(
        &session,
        topic_agent_state(),
        move |message: proto::AgentStateMsg| {
            if message.state_authority == i32::from(state_authority_real()) {
                send_drop_oldest(&tx_clone, &evict_rx_clone, message.into());
            }
        },
    );

    let state_publisher = declare_publisher(&session, topic_agent_state());
    let constraint_publisher = declare_safety_publisher(&session, topic_constraint_event());
    let manipulation_status_publisher = declare_publisher(&session, topic_manipulation_status());
    let object_state_publisher = declare_publisher(&session, topic_object_state());

    loop {
        publish_available(
            &state_publisher,
            topic_agent_state(),
            &channels.state_rx,
            |state: AgentStateMsg| proto::AgentStateMsg::from(state),
        );
        publish_available(
            &constraint_publisher,
            topic_constraint_event(),
            &channels.constraint_rx,
            |message: ConstraintEvent| proto::ConstraintEvent::from(message),
        );
        publish_available(
            &manipulation_status_publisher,
            topic_manipulation_status(),
            &channels.manipulation_status_publish_rx,
            |message: ManipulationStatus| proto::ManipulationStatus::from(message),
        );
        publish_available(
            &object_state_publisher,
            topic_object_state(),
            &channels.object_state_rx,
            |message: ObjectState| proto::ObjectState::from(message),
        );
        thread::sleep(Duration::from_millis(dds_poll_interval_ms()));
    }
}

enum SimControl {
    Drive(PlatformCommand),
    Manipulate(ManipulationCommand),
    Unsupported(&'static str),
}

fn route_control_request(request: &ControlRequest) -> SimControl {
    let source = match request.source {
        ControlSource::Autonomy => command_source_autonomy(),
        ControlSource::Operator => command_source_human_operator(),
        ControlSource::Safety => command_source_none(),
    };

    match &request.action {
        ControlAction::Hold {
            target, frame_id, ..
        } => SimControl::Drive(drive_command(
            request,
            source,
            command_type_hold(),
            *target,
            frame_id.clone(),
            None,
        )),
        ControlAction::Stop { target, frame_id } => SimControl::Drive(drive_command(
            request,
            source,
            command_type_stop(),
            *target,
            frame_id.clone(),
            None,
        )),
        ControlAction::PlanarVelocity {
            target,
            forward_m_s,
            left_m_s,
            yaw_rate_rad_s,
            frame_id,
        } => SimControl::Drive(drive_command(
            request,
            source,
            command_type_velocity(),
            *target,
            frame_id.clone(),
            Some(VelocitySetpoint {
                forward_m_s: *forward_m_s,
                left_m_s: *left_m_s,
                yaw_rate_rad_s: *yaw_rate_rad_s,
            }),
        )),
        ControlAction::SpatialVelocity { .. } => {
            SimControl::Unsupported("spatial velocity is not implemented by the planar simulator")
        }
        ControlAction::Pick {
            object_id,
            target_pose,
            frame_id,
        } => SimControl::Manipulate(ManipulationCommand {
            agent_id: request.agent_id.clone(),
            command_id: request.request_id.clone(),
            object_id: object_id.clone(),
            verb: manipulation_verb_pick_up(),
            approved_by: request.approved_by.clone(),
            target_pose: *target_pose,
            stamp_ms: request.stamp_ms,
            frame_id: frame_id.clone(),
            grasp_profile: manipulation_grasp_profile_top_down(),
        }),
        ControlAction::Place {
            object_id,
            target_pose,
            frame_id,
        } => SimControl::Manipulate(ManipulationCommand {
            agent_id: request.agent_id.clone(),
            command_id: request.request_id.clone(),
            object_id: object_id.clone(),
            verb: manipulation_verb_drop_off(),
            approved_by: request.approved_by.clone(),
            target_pose: *target_pose,
            stamp_ms: request.stamp_ms,
            frame_id: frame_id.clone(),
            grasp_profile: manipulation_grasp_profile_top_down(),
        }),
    }
}

fn drive_command(
    request: &ControlRequest,
    source: u8,
    command_type: u8,
    target: PositionTarget,
    frame_id: String,
    velocity: Option<VelocitySetpoint>,
) -> PlatformCommand {
    PlatformCommand {
        agent_id: request.agent_id.clone(),
        command_id: request.request_id.clone(),
        command_type,
        source,
        approved_by: request.approved_by.clone(),
        target_position: position_target_array(target),
        stamp_ms: request.stamp_ms,
        frame_id,
        episode_id: request.safety_episode_id.clone().unwrap_or_default(),
        command_authority: source,
        velocity,
    }
}

const fn position_target_array(target: PositionTarget) -> [f32; 3] {
    match target {
        PositionTarget::Planar([x, y]) => [x, y, 0.0],
        PositionTarget::Spatial(position) => position,
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/environment_sim/dds.rs"]
mod tests;
