//! Fixed-update systems for simulated control, state publication, and constraints.
//!
//! The key control invariant is: the simulator reports a hard stop only when the
//! agent is in contact with an obstacle or wall and the active command continues
//! to push into that contact. It publishes `severity_none` once that condition
//! clears so the onboard agent can release collision latches.
//!
//! Overseer mental model: this module is the simulated platform executor. It
//! accepts only approved platform commands, moves or freezes the Bevy body, then
//! reports state, command status, and contact-derived constraints through the
//! same message path used by the rest of the stack.

use avian3d::prelude::*;
use bevy::{
    ecs::{query::QueryData, system::SystemParam},
    prelude::*,
};
use core_msgs::{
    APPROVED_BY_ONBOARD_AGENT, AgentStateMsg, ConstraintEvent, ManipulationCommand,
    ManipulationStatus, ObjectState, PlatformCommand, PoseXYZYaw,
    collision_authority_avian_contacts, command_type_velocity, constraint_type_virtual_wall,
    constraint_type_wall, frames::MAP_FRAME, manipulation_failure_reason_safety_stop,
    manipulation_failure_reason_unspecified, manipulation_phase_failed,
    manipulation_phase_in_progress, manipulation_phase_succeeded, manipulation_verb_pick_up,
    object_lifecycle_free, object_lifecycle_held, severity_hard_stop, severity_none,
    source_environment_runtime, state_authority_environment_runtime, topic_constraint_event,
    topic_manipulation_status, topic_object_state,
};
use dds_support::{send_drop_oldest, send_or_log_drop};
use std::collections::HashMap;

use super::{
    config::EnvironmentConfig,
    world::{
        ActiveManipulationTask, Agent, AgentDriveCommand, AgentId, BlockingDirection,
        BlockingObjectId, EnvironmentChannels, FrameConversion, HardStopActive, HybridAgent,
        HybridDriveTarget, HybridStateStamp, ManipulationTaskState, NonBlocking, ObjectCatalogRes,
        ObjectStateStore, SimulatedAgent, VirtualObjectCollider, virtual_object_transform,
    },
};

/// Deterministic execution time for the sim-only manipulation gesture.
const MANIPULATION_TASK_DURATION_MS: u64 = 800;

/// Query bundle used to classify each current agent contact.
#[derive(SystemParam)]
pub struct ContactClassification<'w, 's> {
    /// Structural surfaces (floor, ceiling) never become a hard-stop constraint.
    non_blocking: Query<'w, 's, (), With<NonBlocking>>,

    /// Boundary walls block along their fixed outward normal.
    walls: Query<'w, 's, &'static BlockingDirection>,

    /// Stable object IDs for blocking colliders.
    object_ids: Query<'w, 's, &'static BlockingObjectId>,

    /// Any other collider blocks along the direction from the agent to its center.
    transforms: Query<'w, 's, &'static Transform>,
}

/// Resource bundle the constraint systems consume to publish severity on the
/// bus: channel endpoints, the configured agent identity, and the newest
/// hybrid real-state stamp attached to constraint events.
#[derive(SystemParam)]
pub struct BusReporting<'w> {
    channels: Res<'w, EnvironmentChannels>,
    config: Res<'w, EnvironmentConfig>,
}

/// Mutable motion state read and written by one simulated drive tick.
#[derive(QueryData)]
#[query_data(mutable)]
pub struct SimulatedDrive {
    command: &'static AgentDriveCommand,
    hard_stop: &'static HardStopActive,
    transform: &'static Transform,
    velocity: &'static mut LinearVelocity,
    angular_velocity: &'static mut AngularVelocity,
    locked_axes: &'static mut LockedAxes,
}

/// Mutable contact state needed to stop one simulated agent.
#[derive(QueryData)]
#[query_data(mutable)]
pub struct SimulatedContact {
    command: &'static AgentDriveCommand,
    transform: &'static Transform,
    colliding: &'static CollidingEntities,
    velocity: &'static mut LinearVelocity,
    angular_velocity: &'static mut AngularVelocity,
    hybrid_target: &'static HybridDriveTarget,
}

/// Contact state used to publish one agent's collision constraint.
#[derive(QueryData)]
#[query_data(mutable)]
pub struct AgentContact {
    agent_id: &'static AgentId,
    command: &'static AgentDriveCommand,
    transform: &'static Transform,
    colliding: &'static CollidingEntities,
    hard_stop: &'static mut HardStopActive,
    hybrid_target: &'static HybridDriveTarget,
    is_hybrid: Has<HybridAgent>,
}

/// Drains the newest accepted platform command and updates the drive command.
///
/// Executor ingress contract (see `PlatformCommand` in `core_msgs.proto`):
/// commands for another agent, or without the onboard agent's approval label,
/// are dropped here exactly as the Spot sidecar drops them — both executors
/// enforce the same targeting rule.
pub fn read_platform_command_channel_system(
    channels: Res<'_, EnvironmentChannels>,
    frames: Res<'_, FrameConversion>,
    mut agents: Query<
        '_,
        '_,
        (&AgentId, &mut AgentDriveCommand, &mut HybridDriveTarget),
        With<Agent>,
    >,
    mut warned_unapproved: Local<'_, bool>,
) {
    let (channels, frames) = (channels.into_inner(), frames.into_inner());

    let mut latest: HashMap<String, PlatformCommand> = HashMap::new();
    let mut unapproved_agents: Vec<String> = Vec::new();
    for command in channels.command_rx.try_iter() {
        if command.approved_by == APPROVED_BY_ONBOARD_AGENT {
            latest.insert(command.agent_id.clone(), command);
        } else {
            unapproved_agents.push(command.agent_id);
        }
    }

    let mut rejected_unapproved = false;
    for (agent_id, mut drive_command, mut hybrid_target) in &mut agents {
        if let Some(command) = latest.get(&agent_id.0) {
            *drive_command = drive_command_from_platform_command(command, frames);
            hybrid_target.0 = hybrid_drive_target_from_platform_command(command, frames);
        }
        if unapproved_agents.contains(&agent_id.0) {
            rejected_unapproved = true;
        }
    }

    if rejected_unapproved && !*warned_unapproved {
        *warned_unapproved = true;
        eprintln!(
            "environment_sim: dropping platform command(s) without approved_by=\"{APPROVED_BY_ONBOARD_AGENT}\" \
             (raw-topic publisher or foreign stack on the bus?)"
        );
    }
}

/// First-order velocity-response time constant emulating Spot's gait
/// dynamics — commanded velocity is approached, never jumped to. Without
/// this lag the onboard PD's derivative term (KD > 1) oscillates against a
/// zero-lag plant.
// ponytail: one shared time constant stands in for Spot's real per-axis gait
// dynamics; split per axis only if sim/real divergence ever matters.
const PLANT_RESPONSE_TAU_S: f32 = 0.25;

/// One fixed-step first-order approach of `current` velocity toward the
/// clamped `commanded` setpoint (time constant `PLANT_RESPONSE_TAU_S`). Kept
/// pure so the emulated plant response is unit-testable without a Bevy world.
fn plant_velocity_step(current: Vec3, commanded: Vec3, dt: f32) -> Vec3 {
    let alpha = dt / (PLANT_RESPONSE_TAU_S + dt);
    current + (commanded - current) * alpha
}

/// Integrates the commanded `flat_body` velocity setpoint on the dynamic
/// simulated agent, or freezes it in place while holding.
pub fn drive_agent_system(
    time: Res<'_, Time>,
    frames: Res<'_, FrameConversion>,
    mut agents: Query<'_, '_, SimulatedDrive, With<SimulatedAgent>>,
) {
    let frames = frames.into_inner();
    let dt = time.into_inner().delta_secs();
    for mut agent in &mut agents {
        if agent.hard_stop.0 {
            agent.velocity.0 = Vec3::ZERO;
            agent.angular_velocity.0 = Vec3::ZERO;
            *agent.locked_axes = LockedAxes::ALL_LOCKED;
            continue;
        }

        match agent.command {
            AgentDriveCommand::Drive { setpoint, .. } => {
                // Restore the spawn freedom for the new setpoint: translation
                // stays free and yaw is driven directly; roll/pitch stay locked
                // so contacts can never tip the body over.
                *agent.locked_axes = LockedAxes::new().lock_rotation_x().lock_rotation_z();

                // The onboard controller (autonomy/guidance.rs) already
                // clamps every setpoint to the robot-side SE2VelocityLimit, so the
                // plant only needs its own first-order response lag, not a limit
                // of its own.
                let forward = setpoint.forward_m_s;
                let left = setpoint.left_m_s;
                let yaw_rate = setpoint.yaw_rate_rad_s;

                // flat_body (forward, left) -> map via R(+heading), then map -> bevy.
                let heading = bevy_rotation_to_map_heading(frames, agent.transform.rotation);
                let (sin_h, cos_h) = heading.sin_cos();
                let velocity_map = Vec3::new(
                    sin_h.mul_add(-left, cos_h * forward),
                    cos_h.mul_add(left, sin_h * forward),
                    0.0,
                );
                let commanded_bevy = frames.map_to_bevy_vector(velocity_map);

                // Yaw rate is the exact inverse of publish_agent_state_system's
                // readback (`bevy_to_map_vector(angular).z`), so the commanded
                // and reported map yaw rate can never desync.
                let commanded_angular_bevy =
                    frames.map_to_bevy_vector(Vec3::new(0.0, 0.0, yaw_rate));

                // Approach the commanded velocity through the plant's first-order
                // gait lag instead of snapping to it, so the onboard PD's
                // derivative term (KD > 1) damps against real lag rather than
                // diverging against a zero-lag plant.
                agent.velocity.0 = plant_velocity_step(agent.velocity.0, commanded_bevy, dt);
                agent.angular_velocity.0 =
                    plant_velocity_step(agent.angular_velocity.0, commanded_angular_bevy, dt);
            }
            // Hold means stop now and stay: zero velocity immediately and lock
            // translation too, so the body is pinned exactly where it is and no
            // residual velocity, damping decay, or contact impulse can move it
            // before a new setpoint arrives. Hold never re-dispatches or walks
            // back toward any target.
            AgentDriveCommand::Hold => {
                agent.velocity.0 = Vec3::ZERO;
                agent.angular_velocity.0 = Vec3::ZERO;
                *agent.locked_axes = LockedAxes::ALL_LOCKED;
            }
        }
    }
}

/// Freezes the agent the moment it is driven into a blocking contact.
///
/// Restitution is zero, but the solver's contact response can still leave a
/// residual outward velocity after an impact. Clearing linear and angular
/// velocity while a pushed-into contact exists turns a wall or obstacle hit
/// into a dead stop: the body remains exactly at the point of collision. A
/// command that points away from the contact skips this, so the agent can
/// still be driven back off the surface.
pub fn halt_agent_on_blocking_contact_system(
    contacts: ContactClassification<'_, '_>,
    config: Res<'_, EnvironmentConfig>,
    mut agents: Query<'_, '_, SimulatedContact, With<SimulatedAgent>>,
) {
    let config = config.into_inner();
    for mut agent in &mut agents {
        if contacts
            .pushed_blocking_object_id(
                agent.colliding,
                agent.command,
                agent.transform.translation,
                agent.hybrid_target.0,
                false,
                config,
            )
            .is_some()
        {
            agent.velocity.0 = Vec3::ZERO;
            agent.angular_velocity.0 = Vec3::ZERO;
        }
    }
    let _ = std::hint::black_box(contacts);
}

/// Moves the kinematic hybrid body to the newest real-platform state.
///
/// Rotation mirrors the real platform's reported `heading_yaw_rad`, not just
/// translation. The trip-containment sweep
/// (`swept_trip_hit`) reads this same rotation, so the mirrored footprint
/// stays representative of the real robot's actual orientation.
pub fn hybrid_real_pose_system(
    channels: Res<'_, EnvironmentChannels>,
    frames: Res<'_, FrameConversion>,
    mut hybrid_stamp: ResMut<'_, HybridStateStamp>,
    mut agents: Query<'_, '_, (&AgentId, &mut Transform), With<HybridAgent>>,
) {
    let (channels, frames) = (channels.into_inner(), frames.into_inner());
    let mut latest: HashMap<String, AgentStateMsg> = HashMap::new();
    for state in channels.hybrid_state_rx.try_iter() {
        latest.insert(state.agent_id.clone(), state);
    }
    for (agent_id, mut transform) in &mut agents {
        let Some(state) = latest.get(&agent_id.0) else {
            continue;
        };
        transform.translation = frames.map_to_bevy(Vec3::from_array(state.position));
        let fwd_map = Vec3::new(
            state.heading_yaw_rad.cos(),
            state.heading_yaw_rad.sin(),
            0.0,
        );
        transform.look_to(frames.map_to_bevy_vector(fwd_map), Vec3::Y);
        hybrid_stamp.set(state.stamp_ms);
    }
}

/// Publishes the simulated agent position after the physics step.
pub fn publish_agent_state_system(
    channels: Res<'_, EnvironmentChannels>,
    frames: Res<'_, FrameConversion>,
    agents: Query<
        '_,
        '_,
        (&AgentId, &Transform, &LinearVelocity, &AngularVelocity),
        With<SimulatedAgent>,
    >,
) {
    let (channels, frames) = (channels.into_inner(), frames.into_inner());
    for (agent_id, transform, linear, angular) in &agents {
        let position = frames.bevy_to_map(transform.translation);
        let heading_yaw_rad = bevy_rotation_to_map_heading(frames, transform.rotation);
        // Linear velocity is a free vector: rotate it into map (no
        // translation). Yaw rate is the map +Z component of the angular
        // velocity carried through the same rotation, so neither can desync
        // from the frame registration.
        let velocity = frames.bevy_to_map_vector(linear.0);
        let yaw_rate_rad_s = frames.bevy_to_map_vector(angular.0).z;
        let message = agent_state_message(
            &agent_id.0,
            position,
            heading_yaw_rad,
            velocity,
            yaw_rate_rad_s,
            core_msgs::now_ms(),
        );

        send_drop_oldest(&channels.state_tx, &channels.state_evict_rx, message);
    }
}

/// Publishes wall/obstacle hard-stop and clear events after physics contact.
///
/// A contact is a contact: any non-floor collider the agent touches becomes a
/// hard stop while the active command keeps pushing into it, and clears once the
/// command points away (so the agent can be driven off) or contact ends. In sim
/// mode the walls are solid (`constraint_type_wall`); in hybrid mode they are
/// sensors the kinematic body overlaps (`constraint_type_virtual_wall`).
pub fn publish_contact_constraint_system(
    mut agents: Query<'_, '_, AgentContact, With<Agent>>,
    contacts: ContactClassification<'_, '_>,
    reporting: BusReporting<'_>,
    mut active_episode: Local<'_, HashMap<String, Option<ActiveEpisode>>>,
) {
    let now_ms = core_msgs::now_ms();

    for mut agent in &mut agents {
        let kind = if agent.is_hybrid {
            constraint_type_virtual_wall()
        } else {
            constraint_type_wall()
        };
        let pushing_object_id = contacts.pushed_blocking_object_id(
            agent.colliding,
            agent.command,
            agent.transform.translation,
            agent.hybrid_target.0,
            agent.is_hybrid,
            &reporting.config,
        );
        agent.hard_stop.0 = pushing_object_id.is_some();

        report_push_severity(
            &reporting,
            &agent.agent_id.0,
            pushing_object_id,
            active_episode.entry(agent.agent_id.0.clone()).or_default(),
            kind,
            "overlap",
            now_ms,
        );
    }
    let _ = std::hint::black_box(contacts);
    let _ = std::hint::black_box(reporting);
}

/// Onset episode id and object id tracked across ticks for one active
/// hard-stop condition, so re-assertions and the eventual clear reuse the
/// same `episode_id` for stop-latency correlation.
#[derive(Clone, Debug)]
pub struct ActiveEpisode {
    object_id: String,
    episode_id: String,
}

/// Converts an approved platform command into the simulator drive resource.
fn drive_command_from_platform_command(
    command: &PlatformCommand,
    frames: &FrameConversion,
) -> AgentDriveCommand {
    match command.velocity {
        Some(setpoint) if command.command_type == command_type_velocity() => {
            AgentDriveCommand::Drive {
                setpoint,
                target: frames.map_to_bevy(Vec3::from_array(command.target_position)),
            }
        }
        // Hold/stop (and any command without a velocity setpoint) freeze the body.
        _ => AgentDriveCommand::Hold,
    }
}

/// Builds a simulator-authority agent state message.
fn agent_state_message(
    agent_id: &str,
    position: Vec3,
    heading_yaw_rad: f32,
    velocity: Vec3,
    yaw_rate_rad_s: f32,
    stamp_ms: u64,
) -> AgentStateMsg {
    AgentStateMsg {
        agent_id: agent_id.to_owned(),
        source: source_environment_runtime(),
        state_authority: state_authority_environment_runtime(),
        position: position.to_array(),
        stamp_ms,
        frame_id: MAP_FRAME.to_owned(),
        heading_yaw_rad,
        velocity: velocity.to_array(),
        yaw_rate_rad_s,
    }
}

/// Converts a Bevy body rotation into a `map`-frame heading (CCW about map
/// `+Z`, `0` = `+X` east): rotates the declared body-forward axis (Bevy
/// `-Z`) by the body's current orientation, then converts that direction
/// into `map` through the same `FrameConversion` as every other vector — so
/// it tracks the body's live rotation and can never desync from the frame
/// registration.
fn bevy_rotation_to_map_heading(frames: &FrameConversion, rotation: Quat) -> f32 {
    let fwd_bevy = rotation * Vec3::NEG_Z;
    let fwd_map = frames.bevy_to_map_vector(fwd_bevy);
    fwd_map.y.atan2(fwd_map.x)
}

/// Publishes hard-stop or clear severity based on the current pushing state.
///
/// Onset mints `"<agent_id>-<stamp_ms>"` as the episode id; every re-assertion
/// of the same `object_id` reuses it, and the eventual clear carries it one
/// last time so offline analysis can tie constraint onset to clear.
fn report_push_severity(
    reporting: &BusReporting<'_>,
    agent_id: &str,
    pushing_object_id: Option<String>,
    active_episode: &mut Option<ActiveEpisode>,
    kind: u8,
    trigger: &str,
    stamp_ms: u64,
) {
    let BusReporting { channels, .. } = reporting;
    // Either one hard-stop assertion for the currently pushed contact, or one
    // exact clear for the episode that just ended; otherwise nothing to send.
    let (severity, object_id, episode_id, detail) = if let Some(object_id) = pushing_object_id {
        let episode_id = match active_episode.as_ref() {
            Some(episode) if episode.object_id == object_id => episode.episode_id.clone(),
            _ => format!("{agent_id}-{stamp_ms}"),
        };
        *active_episode = Some(ActiveEpisode {
            object_id: object_id.clone(),
            episode_id: episode_id.clone(),
        });
        (
            severity_hard_stop(),
            object_id,
            episode_id,
            trigger.to_owned(),
        )
    } else if let Some(episode) = active_episode.take() {
        (
            severity_none(),
            episode.object_id,
            episode.episode_id,
            String::new(),
        )
    } else {
        return;
    };

    let message = ConstraintEvent {
        agent_id: agent_id.to_owned(),
        source: source_environment_runtime(),
        collision_authority: collision_authority_avian_contacts(),
        constraint_type: kind,
        severity,
        object_id,
        stamp_ms,
        episode_id,
        detail,
    };
    send_or_log_drop(&channels.constraint_tx, topic_constraint_event(), message);
}

impl ContactClassification<'_, '_> {
    /// Detects whether any current blocking contact is being actively pushed
    /// into, consuming this tick's classification queries.
    ///
    /// Floor/ceiling contacts are skipped. A wall blocks along its stored
    /// outward normal; every other collider blocks along the direction from
    /// the agent to its center. The contact counts as a hard stop only while
    /// the active command still pushes along that direction.
    fn pushed_blocking_object_id(
        &self,
        colliding: &CollidingEntities,
        drive_command: &AgentDriveCommand,
        agent_position: Vec3,
        hybrid_target: Option<Vec3>,
        is_hybrid: bool,
        config: &EnvironmentConfig,
    ) -> Option<String> {
        colliding
            .iter()
            .filter(|entity| !self.non_blocking.contains(**entity))
            .find_map(|entity| {
                let blocking_direction = if let Ok(wall) = self.walls.get(*entity) {
                    wall.0
                } else if let Ok(transform) = self.transforms.get(*entity) {
                    transform.translation - agent_position
                } else {
                    // A contact with no transform cannot be classified: fail closed.
                    return Some("blocking:unclassified".to_owned());
                };

                if !command_pushes_along(
                    drive_command,
                    agent_position,
                    blocking_direction,
                    hybrid_target,
                    is_hybrid,
                    config,
                ) {
                    return None;
                }

                Some(self.object_ids.get(*entity).map_or_else(
                    |_| fallback_blocking_object_id(blocking_direction),
                    |id| id.0.clone(),
                ))
            })
    }
}

fn fallback_blocking_object_id(blocking_direction: Vec3) -> String {
    let planar = blocking_direction.xz();
    if planar.x.abs() >= planar.y.abs() {
        if planar.x >= 0.0 {
            "boundary:+x".to_owned()
        } else {
            "boundary:-x".to_owned()
        }
    } else if planar.y <= 0.0 {
        "boundary:+y".to_owned()
    } else {
        "boundary:-y".to_owned()
    }
}

/// Returns true when the active command drives the body farther along a
/// blocking direction (a wall's outward normal, or the vector toward an
/// obstacle's center). A non-positive dot product means the command points away
/// from the contact, so the body can move off it and no hard stop is needed.
///
/// A small positive threshold (`PUSH_DOT_THRESHOLD`) absorbs floating-point
/// imprecision and near-tangent motion so that the very first escape command
/// after a collision is never mistakenly classified as "pushing in".
///
/// The commanded target comes from `AgentDriveCommand::Drive` for a
/// locally-driven (sim) agent, or from `hybrid_target` for a hybrid agent —
/// see [`HybridDriveTarget`]. Either way, no commanded target (`Hold`, or
/// `None`) means "not pushing": this is the escape hatch that lets a hard
/// stop clear once the agent is told to hold, rather than latching forever.
fn command_pushes_along(
    drive_command: &AgentDriveCommand,
    position: Vec3,
    blocking_direction: Vec3,
    hybrid_target: Option<Vec3>,
    is_hybrid: bool,
    config: &EnvironmentConfig,
) -> bool {
    let target = if is_hybrid {
        let Some(target) = hybrid_target else {
            return false;
        };
        target
    } else {
        let AgentDriveCommand::Drive { target, .. } = drive_command else {
            return false;
        };
        *target
    };

    let motion = (target - position).xz();
    let motion_length = motion.length();
    if motion_length < 1e-3 {
        return false;
    }

    let normal = blocking_direction.xz();
    let normal_length = normal.length();
    if normal_length < 1e-3 {
        return false;
    }

    let deadband_rad = config.push_angular_deadband_rad();
    let sin_deadband = deadband_rad.sin();

    motion.dot(normal) > motion_length * normal_length * sin_deadband
}

/// Derives the hybrid collision-gating target from an approved platform
/// command: `Some` for a velocity command (its `target_position` is the
/// map-frame waypoint goal, not the instantaneous setpoint — see
/// `velocity_command` in `agent::decision`), `None` for hold/stop.
fn hybrid_drive_target_from_platform_command(
    command: &PlatformCommand,
    frames: &FrameConversion,
) -> Option<Vec3> {
    (command.command_type == command_type_velocity())
        .then(|| frames.map_to_bevy(Vec3::from_array(command.target_position)))
}

/// Publishes each catalog object's starting lifecycle/pose at boot, and
/// seeds the local `ObjectStateStore` so the manipulation gate has an
/// immediate answer instead of waiting for a first observed sample.
///
/// Skips a real, MOCAP-tracked object (no `virtual_initial_pose_map`):
/// `object_tracker` is its sole `ObjectState` authority in real mode, and
/// sim/hybrid have no sane placeholder pose to seed it at — it simply has
/// no simulated presence there.
pub fn seed_object_states_system(
    catalog: Res<'_, ObjectCatalogRes>,
    mut store: ResMut<'_, ObjectStateStore>,
    channels: Res<'_, EnvironmentChannels>,
) {
    let (catalog, channels) = (catalog.into_inner(), channels.into_inner());
    for object in catalog.0.iter() {
        let Some(pose) = object.virtual_initial_pose_map() else {
            continue;
        };
        let state = ObjectState {
            object_id: object.object_id().to_owned(),
            lifecycle: object_lifecycle_free(),
            holder_agent_id: String::new(),
            pose,
            pose_valid: true,
            stamp_ms: core_msgs::now_ms(),
            orientation_map: None,
        };
        store.0.insert(object.object_id().to_owned(), state.clone());
        send_or_log_drop(&channels.object_state_tx, topic_object_state(), state);
    }
}

/// Keeps virtual-object collision geometry aligned with object lifecycle state.
///
/// A free object blocks the platform at its published pose. A held or invalid
/// object has no independent collision volume until it is released again.
pub fn sync_virtual_object_colliders_system(
    mut commands: Commands<'_, '_>,
    frames: Res<'_, FrameConversion>,
    store: Res<'_, ObjectStateStore>,
    mut colliders: Query<
        '_,
        '_,
        (
            Entity,
            &VirtualObjectCollider,
            &mut Transform,
            Has<ColliderDisabled>,
        ),
    >,
) {
    let (frames, store) = (frames.into_inner(), store.into_inner());
    for (entity, collider, mut transform, is_disabled) in &mut colliders {
        let Some(state) = store.0.get(&collider.object_id) else {
            continue;
        };

        if state.pose_valid {
            *transform = virtual_object_transform(frames, state.pose, collider.orientation);
        }

        let should_be_enabled = state.pose_valid && state.lifecycle == object_lifecycle_free();
        if should_be_enabled && is_disabled {
            commands.entity(entity).remove::<ColliderDisabled>();
        } else if !should_be_enabled && !is_disabled {
            commands.entity(entity).insert(ColliderDisabled);
        }
    }
}

/// Drains the newest accepted manipulation command and starts a task.
///
/// Executor ingress contract mirrors `read_platform_command_channel_system`:
/// commands for another agent, or without the onboard agent's approval
/// label, are dropped. A command is also ignored (defensively) if a task is
/// already active — the onboard agent's own "single active task"
/// precondition should prevent this in normal operation; the `_pending`-slot
/// style guard here is a one-line backstop, not the primary enforcement.
pub fn read_manipulation_command_channel_system(
    channels: Res<'_, EnvironmentChannels>,
    mut agents: Query<'_, '_, (&AgentId, &mut ManipulationTaskState), With<Agent>>,
) {
    let channels = channels.into_inner();
    let mut latest: HashMap<String, ManipulationCommand> = HashMap::new();
    for command in channels.manipulation_command_rx.try_iter() {
        if command.approved_by == APPROVED_BY_ONBOARD_AGENT {
            latest.insert(command.agent_id.clone(), command);
        }
    }

    let now_ms = core_msgs::now_ms();
    for (agent_id, mut task_state) in &mut agents {
        let Some(command) = latest.remove(&agent_id.0) else {
            continue;
        };
        if task_state.active.is_some() {
            continue;
        }

        task_state.active = Some(ActiveManipulationTask {
            command_id: command.command_id.clone(),
            agent_id: command.agent_id.clone(),
            object_id: command.object_id.clone(),
            verb: command.verb,
            target_pose: command.target_pose,
            deadline_ms: now_ms + MANIPULATION_TASK_DURATION_MS,
        });

        send_or_log_drop(
            &channels.manipulation_status_tx,
            topic_manipulation_status(),
            ManipulationStatus {
                agent_id: command.agent_id,
                command_id: command.command_id,
                object_id: command.object_id,
                phase: manipulation_phase_in_progress(),
                failure_reason: manipulation_failure_reason_unspecified(),
                stamp_ms: now_ms,
            },
        );
    }
}

/// Advances the in-flight manipulation task: aborts it the instant a hard
/// stop becomes active, completes it (attach/detach + terminal status) once
/// its deterministic delay elapses, and republishes the pose of any
/// currently-held object every tick so it visibly follows the platform.
pub fn tick_manipulation_task_system(
    channels: Res<'_, EnvironmentChannels>,
    config: Res<'_, EnvironmentConfig>,
    frames: Res<'_, FrameConversion>,
    mut store: ResMut<'_, ObjectStateStore>,
    mut agents: Query<
        '_,
        '_,
        (
            &AgentId,
            &mut ManipulationTaskState,
            &HardStopActive,
            &Transform,
        ),
        With<SimulatedAgent>,
    >,
) {
    let (channels, config, frames) = (
        channels.into_inner(),
        config.into_inner(),
        frames.into_inner(),
    );
    let now_ms = core_msgs::now_ms();

    for (agent_id, mut task_state, hard_stop_active, transform) in &mut agents {
        let agent_position_map = frames.bevy_to_map(transform.translation);
        let agent_heading_yaw_rad = bevy_rotation_to_map_heading(frames, transform.rotation);

        if let Some(task) = task_state.active.take() {
            if hard_stop_active.0 {
                abort_manipulation_task(channels, &task, now_ms);
            } else if now_ms >= task.deadline_ms {
                complete_manipulation_task(
                    channels,
                    config,
                    &mut store,
                    &task,
                    agent_position_map,
                    agent_heading_yaw_rad,
                    now_ms,
                );
            } else {
                task_state.active = Some(task);
            }
        }

        republish_held_object_poses(
            channels,
            config,
            &mut store,
            &agent_id.0,
            agent_position_map,
            agent_heading_yaw_rad,
            now_ms,
        );
    }
}

/// Publishes a `FAILED`/`SAFETY_STOP` status without touching object state:
/// `PickUp` never attached (the object stays as it was), and `DropOff` never
/// released, so a failed task never opens the gripper automatically.
fn abort_manipulation_task(
    channels: &EnvironmentChannels,
    task: &ActiveManipulationTask,
    now_ms: u64,
) {
    send_or_log_drop(
        &channels.manipulation_status_tx,
        topic_manipulation_status(),
        ManipulationStatus {
            agent_id: task.agent_id.clone(),
            command_id: task.command_id.clone(),
            object_id: task.object_id.clone(),
            phase: manipulation_phase_failed(),
            failure_reason: manipulation_failure_reason_safety_stop(),
            stamp_ms: now_ms,
        },
    );
}

/// Applies a completed task's effect (attach for `PickUp`, release for
/// `DropOff`) and publishes the terminal `SUCCEEDED` status.
fn complete_manipulation_task(
    channels: &EnvironmentChannels,
    config: &EnvironmentConfig,
    store: &mut ObjectStateStore,
    task: &ActiveManipulationTask,
    agent_position_map: Vec3,
    agent_heading_yaw_rad: f32,
    now_ms: u64,
) {
    let (lifecycle, holder_agent_id, pose) =
        manipulation_effect(config, task, agent_position_map, agent_heading_yaw_rad);

    let state = ObjectState {
        object_id: task.object_id.clone(),
        lifecycle,
        holder_agent_id,
        pose,
        pose_valid: true,
        stamp_ms: now_ms,
        orientation_map: None,
    };
    store.0.insert(task.object_id.clone(), state.clone());
    send_or_log_drop(&channels.object_state_tx, topic_object_state(), state);

    send_or_log_drop(
        &channels.manipulation_status_tx,
        topic_manipulation_status(),
        ManipulationStatus {
            agent_id: task.agent_id.clone(),
            command_id: task.command_id.clone(),
            object_id: task.object_id.clone(),
            phase: manipulation_phase_succeeded(),
            failure_reason: manipulation_failure_reason_unspecified(),
            stamp_ms: now_ms,
        },
    );
}

/// Republishes the pose of every object currently held by this agent so it
/// visibly tracks the platform between task completion and any future
/// `DropOff`.
fn republish_held_object_poses(
    channels: &EnvironmentChannels,
    config: &EnvironmentConfig,
    store: &mut ObjectStateStore,
    agent_id: &str,
    agent_position_map: Vec3,
    agent_heading_yaw_rad: f32,
    now_ms: u64,
) {
    for state in update_held_object_poses(
        config,
        store,
        agent_id,
        agent_position_map,
        agent_heading_yaw_rad,
        now_ms,
    ) {
        send_or_log_drop(&channels.object_state_tx, topic_object_state(), state);
    }
}

fn update_held_object_poses(
    config: &EnvironmentConfig,
    store: &mut ObjectStateStore,
    agent_id: &str,
    agent_position_map: Vec3,
    agent_heading_yaw_rad: f32,
    now_ms: u64,
) -> Vec<ObjectState> {
    let agent_size = config.agent_size(agent_id);
    let mut updates = Vec::new();
    for state in store.0.values_mut() {
        if state.lifecycle == object_lifecycle_held() && state.holder_agent_id == agent_id {
            state.pose = carry_pose_map(agent_size, agent_position_map, agent_heading_yaw_rad);
            state.stamp_ms = now_ms;
            updates.push(state.clone());
        }
    }
    updates
}

/// Object-state effect of a completed manipulation task (attach for `PickUp`,
/// release for `DropOff`) — shared by `sim` role, which applies this the
/// instant its own deterministic gesture timer elapses, and `hybrid` role
/// (`apply_manipulation_status_system`), which applies the identical effect
/// only once the real-Spot sidecar (the actual executor in hybrid mode)
/// reports success for the same command.
fn manipulation_effect(
    config: &EnvironmentConfig,
    task: &ActiveManipulationTask,
    agent_position_map: Vec3,
    agent_heading_yaw_rad: f32,
) -> (u8, String, PoseXYZYaw) {
    if task.verb == manipulation_verb_pick_up() {
        (
            object_lifecycle_held(),
            task.agent_id.clone(),
            carry_pose_map(
                config.agent_size(&task.agent_id),
                agent_position_map,
                agent_heading_yaw_rad,
            ),
        )
    } else {
        (object_lifecycle_free(), String::new(), task.target_pose)
    }
}

/// Hybrid-role counterpart of `tick_manipulation_task_system`: the real-Spot
/// sidecar (not this process) executes the arm sequence in hybrid mode, so
/// this system only applies the object-state effect once the sidecar
/// reports a *terminal* `ManipulationStatus` for the command
/// `read_manipulation_command_channel_system` latched. It never re-publishes
/// `ManipulationStatus` itself — the sidecar's own publication is the
/// authoritative record on the bus/MCAP — and it does not consult
/// `HardStopActive`: the sidecar already aborts its in-flight arm sequence on
/// any hard stop (virtual-wall or Spot fault alike) via its own reflex-stop
/// preemption, so a FAILED status here is observed, not independently
/// re-derived. Matches `abort_manipulation_task`'s invariant: a failure never
/// touches object state, so there is no false HELD/PLACED.
pub fn apply_manipulation_status_system(
    channels: Res<'_, EnvironmentChannels>,
    config: Res<'_, EnvironmentConfig>,
    frames: Res<'_, FrameConversion>,
    mut store: ResMut<'_, ObjectStateStore>,
    mut agents: Query<
        '_,
        '_,
        (&AgentId, &mut ManipulationTaskState, &Transform),
        With<HybridAgent>,
    >,
) {
    let (channels, config, frames) = (
        channels.into_inner(),
        config.into_inner(),
        frames.into_inner(),
    );
    let mut latest: HashMap<String, ManipulationStatus> = HashMap::new();
    for status in channels.manipulation_status_rx.try_iter() {
        latest.insert(status.agent_id.clone(), status);
    }

    for (agent_id, mut task_state, transform) in &mut agents {
        let Some(task) = task_state.active.take() else {
            continue;
        };

        let Some(status) = latest.get(&agent_id.0) else {
            task_state.active = Some(task);
            continue;
        };

        let terminal = status.phase == manipulation_phase_succeeded()
            || status.phase == manipulation_phase_failed();
        if status.command_id != task.command_id || !terminal {
            // Not (yet) a terminal status for the task this process latched:
            // keep waiting. A stale/foreign command id must never clear or
            // re-key a newer latch.
            task_state.active = Some(task);
            continue;
        }

        if status.phase == manipulation_phase_succeeded() {
            let now_ms = core_msgs::now_ms();
            let agent_position_map = frames.bevy_to_map(transform.translation);
            let agent_heading_yaw_rad = bevy_rotation_to_map_heading(frames, transform.rotation);
            let (lifecycle, holder_agent_id, pose) =
                manipulation_effect(config, &task, agent_position_map, agent_heading_yaw_rad);

            let state = ObjectState {
                object_id: task.object_id.clone(),
                lifecycle,
                holder_agent_id,
                pose,
                pose_valid: true,
                stamp_ms: now_ms,
                orientation_map: None,
            };
            store.0.insert(task.object_id, state.clone());
            send_or_log_drop(&channels.object_state_tx, topic_object_state(), state);
        }
    }
}

/// Publishes hybrid-held object poses from the newest real-platform pose.
///
/// Hybrid manipulation status owns attach/detach transitions, while this
/// system owns the continuous carry transform between those transitions.
pub fn publish_hybrid_held_object_pose_system(
    channels: Res<'_, EnvironmentChannels>,
    config: Res<'_, EnvironmentConfig>,
    frames: Res<'_, FrameConversion>,
    mut store: ResMut<'_, ObjectStateStore>,
    agents: Query<'_, '_, (&AgentId, &Transform), With<HybridAgent>>,
) {
    let (channels, config, frames) = (
        channels.into_inner(),
        config.into_inner(),
        frames.into_inner(),
    );
    let now_ms = core_msgs::now_ms();

    for (agent_id, transform) in &agents {
        republish_held_object_poses(
            channels,
            config,
            &mut store,
            &agent_id.0,
            frames.bevy_to_map(transform.translation),
            bevy_rotation_to_map_heading(frames, transform.rotation),
            now_ms,
        );
    }
}

/// Carry pose: flush with the body's front face, at the body's top height —
/// matching the visualizer's static "black arm" marker geometry
/// (`setup_visual_agent`) without real forward kinematics. The simulator does
/// not model grasp contact physics, and the arm marker is a fixed decorative
/// box rather than an articulated model.
fn carry_pose_map(
    agent_size: [f32; 3],
    agent_position_map: Vec3,
    agent_heading_yaw_rad: f32,
) -> PoseXYZYaw {
    let forward = agent_size[0] * 0.5;
    PoseXYZYaw {
        x: agent_position_map.x + forward * agent_heading_yaw_rad.cos(),
        y: agent_position_map.y + forward * agent_heading_yaw_rad.sin(),
        z: agent_size[2],
        yaw_rad: agent_heading_yaw_rad,
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/environment_sim/systems.rs"]
mod systems_tests;
