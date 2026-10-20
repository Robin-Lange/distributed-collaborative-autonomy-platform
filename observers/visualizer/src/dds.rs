//! DDS/Zenoh observation bridge for the visualizer.
//!
//! The shared bus frame is the canonical `map` frame (ENU). This thread only
//! moves protobuf bytes and internal message structs across channels; the
//! `map` -> Bevy render-frame conversion happens at the render boundary
//! (`systems::poll_agent_state_system`, via `world::FrameConversion`).

use core_msgs::{
    AgentStateMsg, ObjectState, PlatformCommand, WaypointIntent, dds_observer_poll_interval_ms,
    proto, topic_agent_state, topic_object_state, topic_platform_command, topic_waypoint_intent,
};
use crossbeam_channel::{Receiver, bounded};
use dds_support::{
    DDS_CHANNEL_CAPACITY, open_session, subscribe_forward, subscribe_forward_latest,
};
use std::{thread, time::Duration};

/// Channel bundle returned to Bevy after the DDS thread is spawned.
pub struct VisualizerDdsIo {
    /// Agent state samples.
    agent_states: Receiver<AgentStateMsg>,

    /// Platform commands.
    platform_commands: Receiver<PlatformCommand>,

    /// Operator waypoint intents for HUD text.
    operator_intents: Receiver<WaypointIntent>,

    /// Catalog object lifecycle/pose state.
    object_states: Receiver<ObjectState>,
}

impl VisualizerDdsIo {
    /// Consumes the bundle and exposes the receivers as Bevy resources.
    pub(super) fn into_channels(
        self,
    ) -> (
        Receiver<AgentStateMsg>,
        Receiver<PlatformCommand>,
        Receiver<WaypointIntent>,
        Receiver<ObjectState>,
    ) {
        (
            self.agent_states,
            self.platform_commands,
            self.operator_intents,
            self.object_states,
        )
    }
}

/// Starts the visualizer's background bus subscriber thread.
pub fn spawn_visualizer_dds_io() -> VisualizerDdsIo {
    let (state_tx, state_rx) = bounded::<AgentStateMsg>(DDS_CHANNEL_CAPACITY);
    let (platform_command_tx, platform_command_rx) =
        bounded::<PlatformCommand>(DDS_CHANNEL_CAPACITY);
    let (intent_tx, intent_rx) = bounded::<WaypointIntent>(DDS_CHANNEL_CAPACITY);
    let (object_state_tx, object_state_rx) = bounded::<ObjectState>(DDS_CHANNEL_CAPACITY);
    // `state_tx` is a latest-wins stream: evict the oldest queued sample
    // through this clone when full instead of falling behind.
    let state_evict_rx = state_rx.clone();
    let platform_command_evict_rx = platform_command_rx.clone();

    thread::spawn(move || {
        let session = open_session();

        let _state_sub = subscribe_forward_latest(
            &session,
            topic_agent_state(),
            &state_tx,
            &state_evict_rx,
            |message: proto::AgentStateMsg| message.into(),
        );
        let _command_sub = subscribe_forward_latest(
            &session,
            topic_platform_command(),
            &platform_command_tx,
            &platform_command_evict_rx,
            |message: proto::PlatformCommand| message.into(),
        );
        let _intent_sub = subscribe_forward(
            &session,
            topic_waypoint_intent(),
            &intent_tx,
            |message: proto::WaypointIntent| message.into(),
        );
        let _object_state_sub = subscribe_forward(
            &session,
            topic_object_state(),
            &object_state_tx,
            |message: proto::ObjectState| message.into(),
        );

        loop {
            // This thread only observes 20-40 Hz streams for HUD/render use;
            // a 1 kHz control-path wakeup here would be wasted cycles.
            thread::sleep(Duration::from_millis(dds_observer_poll_interval_ms()));
        }
    });

    VisualizerDdsIo {
        agent_states: state_rx,
        platform_commands: platform_command_rx,
        operator_intents: intent_rx,
        object_states: object_state_rx,
    }
}
