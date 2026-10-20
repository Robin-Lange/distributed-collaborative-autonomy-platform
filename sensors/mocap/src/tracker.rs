//! Real-mode `ObjectState` publisher for MOCAP-tracked objects.
//!
//! Matches `mocap_rigid_body` frames against catalog IDs, converts them into `map`
//! frame, and republishes the live pose as `ObjectState`.

use core_msgs::{
    APPROVED_BY_ONBOARD_AGENT, ManipulationCommand, ManipulationStatus, ObjectState, PoseXYZYaw,
    Quat,
    catalog::ObjectCatalog,
    frames::{FramesConfig, RigidTransform, mat3_determinant, mat3_to_quat, quat_is_valid},
    manipulation_phase_failed, manipulation_phase_succeeded, manipulation_verb_pick_up, now_ms,
    object_lifecycle_free, object_lifecycle_held,
    proto::{self, MocapRigidBodyFrame},
    topic_manipulation_command, topic_manipulation_status, topic_mocap_rigid_body,
    topic_object_state,
};
use glam::DQuat;
use prost::Message;
use std::{
    collections::HashMap,
    ffi::OsString,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::Duration,
};
use zenoh::Wait;

/// A tracked object is stale (and reported `pose_valid=false`) once no
/// matching `mocap_rigid_body` frame has updated it within this window.
/// `NatNet` publishes at ~120 Hz; this tolerates gripper occlusion of a few
/// frames without flapping, matching `ActiveConstraints`'
/// `ENV_CONSTRAINT_TTL_MS` staleness pattern.
const STALE_MS: u64 = 300;
/// `ObjectState` republish cadence — a state stream, not a control-path
/// message, so this is far below `dds_poll_interval_ms()`'s 1 ms sweep.
const PUBLISH_INTERVAL_MS: u64 = 50;

/// Runtime state for one MOCAP-tracked catalog object.
struct TrackedObject {
    mocap_body_id: i32,
    lifecycle: u8,
    holder_agent_id: String,
    pose: Option<PoseXYZYaw>,
    /// Full orientation in `map`, `(x, y, z, w)` — set together with
    /// `pose` from the same observed frame, so the two never disagree.
    orientation_map: Option<(f32, f32, f32, f32)>,
    last_seen_ms: u64,
}

/// A manipulation command latched for a tracked object, awaiting its terminal status.
struct PendingTask {
    command_id: String,
    verb: u8,
    agent_id: String,
}

/// Seeds one tracked entry per catalog object carrying a `mocap_rigid_body_id`.
fn seed_tracked_objects(catalog: &ObjectCatalog) -> HashMap<String, TrackedObject> {
    catalog
        .iter()
        .filter_map(|spec| {
            let mocap_body_id = spec.mocap_rigid_body_id()?;
            Some((
                spec.object_id().to_owned(),
                TrackedObject {
                    mocap_body_id,
                    lifecycle: object_lifecycle_free(),
                    holder_agent_id: String::new(),
                    pose: None,
                    orientation_map: None,
                    last_seen_ms: 0,
                },
            ))
        })
        .collect()
}

/// Latches an incoming `ManipulationCommand` for a tracked object.
fn latch_manipulation_command(
    pending: &mut HashMap<String, PendingTask>,
    tracked: &HashMap<String, TrackedObject>,
    command: &ManipulationCommand,
) {
    if command.approved_by != APPROVED_BY_ONBOARD_AGENT {
        return;
    }
    if !tracked.contains_key(&command.object_id) {
        return;
    }

    pending.insert(
        command.object_id.clone(),
        PendingTask {
            command_id: command.command_id.clone(),
            verb: command.verb,
            agent_id: command.agent_id.clone(),
        },
    );
}

/// Applies a terminal `ManipulationStatus` to update the tracked object's lifecycle.
fn apply_manipulation_status(
    pending: &mut HashMap<String, PendingTask>,
    tracked: &mut HashMap<String, TrackedObject>,
    status: &ManipulationStatus,
) {
    let Some(task) = pending.get(&status.object_id) else {
        return;
    };
    if task.command_id != status.command_id {
        return;
    }

    let terminal = status.phase == manipulation_phase_succeeded()
        || status.phase == manipulation_phase_failed();
    if !terminal {
        return;
    }

    if status.phase == manipulation_phase_succeeded()
        && let Some(object) = tracked.get_mut(&status.object_id)
    {
        if task.verb == manipulation_verb_pick_up() {
            object.lifecycle = object_lifecycle_held();
            object.holder_agent_id.clone_from(&task.agent_id);
        } else {
            object.lifecycle = object_lifecycle_free();
            object.holder_agent_id = String::new();
        }
    }

    pending.remove(&status.object_id);
}

/// Updates tracked objects' poses and orientations from a `mocap_rigid_body` frame.
///
/// `yaw_rad` is the heading of the body's forward (`+X`) axis after rotation into
/// `map`, projected onto the `map` XY-plane. A body missing from the frame, or with
/// a non-finite position or invalid quaternion, is skipped, so it retains its last
/// pose until `STALE_MS` ages it out in `object_state_for`.
fn apply_mocap_frame(
    tracked: &mut HashMap<String, TrackedObject>,
    t_map_mocap: &RigidTransform,
    q_map_mocap: DQuat,
    frame: &MocapRigidBodyFrame,
) {
    for object in tracked.values_mut() {
        let Some(body) = frame.bodies.iter().find(|b| b.id == object.mocap_body_id) else {
            continue;
        };
        let Some(pos) = body.position.as_ref() else {
            continue;
        };
        if !(pos.x.is_finite() && pos.y.is_finite() && pos.z.is_finite()) {
            continue;
        }

        let orientation = match body.orientation.as_ref() {
            Some(ori) => {
                let (qx, qy, qz, qw) = (
                    f64::from(ori.x),
                    f64::from(ori.y),
                    f64::from(ori.z),
                    f64::from(ori.w),
                );
                if !quat_is_valid(qx, qy, qz, qw) {
                    continue;
                }
                let q_mocap_body = DQuat::from_xyzw(qx, qy, qz, qw).normalize();
                Some((q_map_mocap * q_mocap_body).normalize())
            }
            None => None,
        };

        let yaw_rad = orientation.map_or(0.0, |q| {
            let fwd = (q * glam::DVec3::X).as_vec3();
            fwd.y.atan2(fwd.x)
        });

        let position = glam::DVec3::from_array(t_map_mocap.transform_point([
            f64::from(pos.x),
            f64::from(pos.y),
            f64::from(pos.z),
        ]))
        .as_vec3();

        object.pose = Some(PoseXYZYaw {
            x: position.x,
            y: position.y,
            z: position.z,
            yaw_rad,
        });
        object.orientation_map = orientation.map(|q| {
            let q = q.as_quat();
            (q.x, q.y, q.z, q.w)
        });
        object.last_seen_ms = frame.stamp_ms;
    }
}

/// Builds the `ObjectState` to publish for one tracked object.
fn object_state_for(object_id: &str, object: &TrackedObject, now_ms: u64) -> ObjectState {
    let pose_valid =
        object.pose.is_some() && now_ms.saturating_sub(object.last_seen_ms) <= STALE_MS;
    ObjectState {
        object_id: object_id.to_owned(),
        lifecycle: object.lifecycle,
        holder_agent_id: object.holder_agent_id.clone(),
        pose: object.pose.unwrap_or_default(),
        pose_valid,
        stamp_ms: now_ms,
        orientation_map: object
            .orientation_map
            .filter(|_| pose_valid)
            .map(|(x, y, z, w)| Quat { x, y, z, w }),
    }
}

/// Tracks configured objects and publishes their lifecycle and pose.
///
/// # Panics
///
/// Panics when the process cannot install its Ctrl-C handler or initialize a
/// required middleware resource.
pub fn run_with_args(_args: impl IntoIterator<Item = OsString>) {
    enum TrackerEvent {
        Mocap(MocapRigidBodyFrame),
        Command(proto::ManipulationCommand),
        Status(proto::ManipulationStatus),
    }

    // Built from the launcher's DCA_APP_CONFIG_JSON, not a standalone
    // `--object-catalog`/config-file path, so this always matches whatever
    // the launcher actually shipped (including any objects the operator
    // added interactively).
    let app_config = core_msgs::config::load_app_config();
    let catalog = ObjectCatalog::from_config(app_config.objects);
    let mut tracked = seed_tracked_objects(&catalog);

    if tracked.is_empty() {
        println!(
            "object_tracker: no mocap_rigid_body_id entries in the catalog — nothing to track"
        );
    } else {
        let ids: Vec<&str> = tracked.keys().map(String::as_str).collect();
        println!(
            "object_tracker: tracking {} object(s): {}",
            tracked.len(),
            ids.join(", ")
        );
    }

    let t_map_mocap = FramesConfig::from_config(&app_config.frames).t_map_mocap;
    let t_map_mocap_det = mat3_determinant(&t_map_mocap.r);
    println!(
        "object_tracker: T_map_mocap det={t_map_mocap_det:.4} (expect ~1.0 for a proper rotation)"
    );
    let (qx, qy, qz, qw) = mat3_to_quat(&t_map_mocap.r);
    let q_map_mocap = DQuat::from_xyzw(qx, qy, qz, qw).normalize();

    let session = dds_support::open_session();

    let (tx, rx) = crossbeam_channel::bounded(1000);

    let tx_mocap = tx.clone();
    let _mocap_sub = dds_support::subscribe_for_each(
        &session,
        topic_mocap_rigid_body(),
        move |frame: MocapRigidBodyFrame| {
            let _ = tx_mocap.try_send(TrackerEvent::Mocap(frame));
        },
    );
    let tx_command = tx.clone();
    let _command_sub = dds_support::subscribe_for_each(
        &session,
        topic_manipulation_command(),
        move |cmd: proto::ManipulationCommand| {
            let _ = tx_command.try_send(TrackerEvent::Command(cmd));
        },
    );
    let tx_status = tx;
    let _status_sub = dds_support::subscribe_for_each(
        &session,
        topic_manipulation_status(),
        move |status: proto::ManipulationStatus| {
            let _ = tx_status.try_send(TrackerEvent::Status(status));
        },
    );

    let state_pub = dds_support::declare_publisher(&session, topic_object_state());

    let running = Arc::new(AtomicBool::new(true));
    {
        let r = Arc::clone(&running);
        ctrlc::set_handler(move || r.store(false, Ordering::SeqCst))
            .unwrap_or_else(|error| panic!("failed to install Ctrl-C handler: {error}"));
    }

    let mut pending: HashMap<String, PendingTask> = HashMap::new();
    let mut last_publish_ms: u64 = 0;

    while running.load(Ordering::SeqCst) {
        while let Ok(event) = rx.try_recv() {
            match event {
                TrackerEvent::Mocap(frame) => {
                    apply_mocap_frame(&mut tracked, &t_map_mocap, q_map_mocap, &frame);
                }
                TrackerEvent::Command(command) => {
                    latch_manipulation_command(&mut pending, &tracked, &command.into());
                }
                TrackerEvent::Status(status) => {
                    apply_manipulation_status(&mut pending, &mut tracked, &status.into());
                }
            }
        }

        let now = now_ms();
        if now.saturating_sub(last_publish_ms) >= PUBLISH_INTERVAL_MS {
            for (object_id, object) in &tracked {
                let state = object_state_for(object_id, object, now);
                let _ = state_pub
                    .put(proto::ObjectState::from(state).encode_to_vec())
                    .wait();
            }
            last_publish_ms = now;
        }

        thread::sleep(Duration::from_millis(core_msgs::dds_poll_interval_ms()));
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/mocap_tracker.rs"]
mod tests;
