//! DDS/Zenoh subscriber thread for the MCAP recorder.
//!
//! The recorder keeps protobuf messages as protobuf all the way to disk. Generic
//! topics use `record_available`; agent state additionally emits a derived
//! Foxglove transform sample so recordings are easier to inspect visually.

use super::{
    frame_transform,
    record::{RecordedSample, recorded_json_sample, recorded_proto_sample},
};
use core_msgs::{
    dds_observer_poll_interval_ms, proto, topic_agent_state, topic_authority_state,
    topic_command_status, topic_command_veto, topic_constraint_event, topic_frame_calibration,
    topic_manipulation_command, topic_manipulation_intent, topic_manipulation_status,
    topic_mocap_rigid_body, topic_object_state, topic_platform_command, topic_waypoint_intent,
    type_agent_state, type_authority_state, type_command_status, type_command_veto,
    type_constraint_event, type_frame_calibration, type_manipulation_command,
    type_manipulation_intent, type_manipulation_status, type_mocap_rigid_body, type_object_state,
    type_platform_command, type_waypoint_intent,
};
use crossbeam_channel::Sender;
use dds_support::{open_session, send_or_log_drop, subscribe_for_each};
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::Duration,
};
use zenoh::{Session, pubsub::Subscriber};

/// Starts the recorder bus thread and forwards encoded samples to the writer.
pub fn spawn_recorder_dds_io(tx: Sender<RecordedSample>, running: Arc<AtomicBool>) {
    thread::spawn(move || {
        let session = open_session();

        let _intent = record_available::<proto::WaypointIntent>(
            &session,
            &tx,
            topic_waypoint_intent(),
            type_waypoint_intent(),
        );
        let _state = record_agent_state(&session, &tx);
        let _constraint = record_available::<proto::ConstraintEvent>(
            &session,
            &tx,
            topic_constraint_event(),
            type_constraint_event(),
        );
        let _authority = record_available::<proto::AuthorityState>(
            &session,
            &tx,
            topic_authority_state(),
            type_authority_state(),
        );
        let _command = record_available::<proto::PlatformCommand>(
            &session,
            &tx,
            topic_platform_command(),
            type_platform_command(),
        );
        let _veto = record_available::<proto::CommandVeto>(
            &session,
            &tx,
            topic_command_veto(),
            type_command_veto(),
        );
        let _status = record_available::<proto::CommandStatus>(
            &session,
            &tx,
            topic_command_status(),
            type_command_status(),
        );
        let _mocap = record_available::<proto::MocapRigidBodyFrame>(
            &session,
            &tx,
            topic_mocap_rigid_body(),
            type_mocap_rigid_body(),
        );
        let _frame_cal = record_available::<proto::FrameCalibration>(
            &session,
            &tx,
            topic_frame_calibration(),
            type_frame_calibration(),
        );
        let _manip_intent = record_available::<proto::ManipulationIntent>(
            &session,
            &tx,
            topic_manipulation_intent(),
            type_manipulation_intent(),
        );
        let _manip_cmd = record_available::<proto::ManipulationCommand>(
            &session,
            &tx,
            topic_manipulation_command(),
            type_manipulation_command(),
        );
        let _manip_status = record_available::<proto::ManipulationStatus>(
            &session,
            &tx,
            topic_manipulation_status(),
            type_manipulation_status(),
        );
        let _object_state = record_available::<proto::ObjectState>(
            &session,
            &tx,
            topic_object_state(),
            type_object_state(),
        );

        while running.load(Ordering::SeqCst) {
            // The recorder only observes 20-120 Hz streams for later playback;
            // a 1 kHz control-path wakeup here would be wasted cycles.
            thread::sleep(Duration::from_millis(dds_observer_poll_interval_ms()));
        }
    });
}

/// Records agent state plus a derived Foxglove frame-transform sample.
fn record_agent_state(session: &Session, tx: &Sender<RecordedSample>) -> Subscriber<()> {
    let tx = tx.clone();
    subscribe_for_each(
        session,
        topic_agent_state(),
        move |message: proto::AgentStateMsg| {
            // Capture one timestamp and reuse it for both the native state sample
            // and derived transform so the channels line up in Foxglove.
            let time_ns = core_msgs::now_ns();
            send_or_log_drop(
                &tx,
                topic_agent_state(),
                recorded_proto_sample(topic_agent_state(), type_agent_state(), &message, time_ns),
            );

            let transform = frame_transform::from_agent_state(&message);
            if let Ok(sample) = recorded_json_sample(
                frame_transform::TOPIC_AGENT_TF,
                frame_transform::SCHEMA_FRAME_TRANSFORM,
                &transform,
                time_ns,
            ) {
                send_or_log_drop(&tx, frame_transform::TOPIC_AGENT_TF, sample);
            }
        },
    )
}

/// Records all available samples for one protobuf-backed topic.
fn record_available<T: prost::Message + Default + Clone + Send + 'static>(
    session: &Session,
    tx: &Sender<RecordedSample>,
    topic: &'static str,
    schema_name: &'static str,
) -> Subscriber<()> {
    let tx = tx.clone();
    subscribe_for_each(session, topic, move |message: T| {
        send_or_log_drop(
            &tx,
            topic,
            recorded_proto_sample(topic, schema_name, &message, core_msgs::now_ns()),
        );
    })
}
