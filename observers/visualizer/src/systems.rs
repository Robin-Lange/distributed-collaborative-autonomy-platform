//! Runtime systems for the read-only visualizer.
//!
//! Each update drains local channels populated by the DDS thread. The visualizer
//! deliberately keeps only the newest message for the configured agent, because
//! rendering stale intermediate samples is less useful than showing the most
//! current observed state.

use bevy::{ecs::system::SystemParam, prelude::*};
use core_msgs::{
    AgentStateMsg, PlatformCommand, WaypointIntent, behavior_from_code, behavior_label,
    command_type_name, object_lifecycle_free, object_lifecycle_held,
};
use dds_support::drain_latest_for_agent;
use std::collections::HashMap;

use super::{
    config::VisualizerConfig,
    setup::spawn_visual_object,
    world::{
        AgentId, FrameConversion, HudText, IDLE_HUD_TEXT, LatestOperatorIntent,
        LatestPlatformCommand, ObjectCatalogRes, ObjectMotion, VisualAgent, VisualObject,
        VisualObjects, VisualizerObservationChannels,
    },
};

/// Moves the rendered agent to the latest observed bus state and orients it
/// to the reported heading. Rendering is observation-only and never feeds
/// back into navigation or collider rotation.
pub fn poll_agent_state_system(
    channels: Res<'_, VisualizerObservationChannels>,
    frames: Res<'_, FrameConversion>,
    mut agents: Query<'_, '_, (&AgentId, &mut Transform), With<VisualAgent>>,
) {
    let (channels, frames) = (channels.into_inner(), frames.into_inner());
    let mut latest: HashMap<String, AgentStateMsg> = HashMap::new();
    for state in channels.state_rx().try_iter() {
        latest.insert(state.agent_id.clone(), state);
    }
    for (agent_id, mut transform) in &mut agents {
        let Some(message) = latest.get(&agent_id.0) else {
            continue;
        };
        transform.translation = frames.map_to_bevy(Vec3::from_array(message.position));
        transform.look_to(
            heading_to_bevy_forward(frames, message.heading_yaw_rad),
            Vec3::Y,
        );
    }
}

/// `map`-frame unit direction for a heading, converted into Bevy through
/// `FrameConversion` — the rendering counterpart of
/// `environment_sim::systems::bevy_rotation_to_map_heading`.
fn heading_to_bevy_forward(frames: &FrameConversion, heading_yaw_rad: f32) -> Vec3 {
    let dir_map = Vec3::new(heading_yaw_rad.cos(), heading_yaw_rad.sin(), 0.0);
    frames.map_to_bevy_vector(dir_map)
}

/// Mutable ECS state used while spawning or updating visual objects.
#[derive(SystemParam)]
pub struct ObjectRenderParams<'w, 's> {
    commands: Commands<'w, 's>,
    meshes: ResMut<'w, Assets<Mesh>>,
    materials: ResMut<'w, Assets<StandardMaterial>>,
    visual_objects: ResMut<'w, VisualObjects>,
    objects: Query<'w, 's, (&'static mut Transform, &'static mut ObjectMotion), With<VisualObject>>,
}

const MANIPULATION_TRANSITION_DURATION_S: f32 = 0.8;

/// Moves every rendered catalog object to its latest observed pose —
/// covering a free object sitting still, a held object tracking the
/// carrying agent (`environment_sim` republishes its pose every tick while
/// held), and a placed object at its drop point. Not agent-scoped: an
/// object belongs to the shared catalog, not to the configured agent's
/// filter, so every sample is applied regardless of which agent is holding
/// it.
///
/// A real, MOCAP-tracked object with no catalog `virtual_initial_pose_map` is
/// not spawned at `Startup` because `setup_visual_objects` has
/// no sane placeholder pose for it — so its *first* `ObjectState` spawns it
/// here instead, at wherever it actually is.
pub fn poll_object_state_system(
    channels: Res<'_, VisualizerObservationChannels>,
    frames: Res<'_, FrameConversion>,
    catalog: Res<'_, ObjectCatalogRes>,
    render: ObjectRenderParams<'_, '_>,
) {
    let (channels, frames, catalog) = (
        channels.into_inner(),
        frames.into_inner(),
        catalog.into_inner(),
    );
    let ObjectRenderParams {
        mut commands,
        mut meshes,
        mut materials,
        mut visual_objects,
        mut objects,
    } = render;

    while let Ok(state) = channels.object_state_rx().try_recv() {
        let entity = if let Some(entity) = visual_objects.0.get(&state.object_id).copied() {
            entity
        } else {
            let Some(object) = catalog.0.get(&state.object_id) else {
                continue;
            };
            let entity = spawn_visual_object(
                &mut commands,
                &mut meshes,
                &mut materials,
                frames,
                object,
                state.pose,
            );
            visual_objects.0.insert(state.object_id.clone(), entity);
            entity
        };

        let Ok((mut transform, mut motion)) = objects.get_mut(entity) else {
            continue;
        };

        let target_translation =
            frames.map_to_bevy(Vec3::new(state.pose.x, state.pose.y, state.pose.z));
        let target_rotation = match state.orientation_map.filter(|_| state.pose_valid) {
            Some(q) => frames.map_orientation_to_bevy((q.x, q.y, q.z, q.w)),
            None => object_rotation(frames, state.pose.yaw_rad),
        };
        observe_object_pose(
            &mut transform,
            &mut motion,
            state.lifecycle,
            target_translation,
            target_rotation,
        );
    }
}

fn observe_object_pose(
    transform: &mut Transform,
    motion: &mut ObjectMotion,
    lifecycle: u8,
    target_translation: Vec3,
    target_rotation: Quat,
) {
    let lifecycle_changed = motion.lifecycle.is_some_and(|previous| {
        (previous == object_lifecycle_free() && lifecycle == object_lifecycle_held())
            || (previous == object_lifecycle_held() && lifecycle == object_lifecycle_free())
    });
    motion.lifecycle = Some(lifecycle);
    motion.target_translation = target_translation;
    motion.target_rotation = target_rotation;

    if lifecycle_changed {
        motion.start_translation = transform.translation;
        motion.start_rotation = transform.rotation;
        motion.elapsed_s = 0.0;
        motion.active = true;
    } else if !motion.active {
        transform.translation = target_translation;
        transform.rotation = target_rotation;
    }
}

/// Advances active pick/place render transitions without changing bus state.
pub fn animate_object_motion_system(
    time: Res<'_, Time>,
    mut objects: Query<'_, '_, (&mut Transform, &mut ObjectMotion), With<VisualObject>>,
) {
    let delta_s = time.into_inner().delta_secs();
    for (mut transform, mut motion) in &mut objects {
        advance_object_motion(&mut transform, &mut motion, delta_s);
    }
}

fn advance_object_motion(transform: &mut Transform, motion: &mut ObjectMotion, delta_s: f32) {
    if !motion.active {
        return;
    }

    motion.elapsed_s += delta_s;
    let linear = (motion.elapsed_s / MANIPULATION_TRANSITION_DURATION_S).clamp(0.0, 1.0);
    let eased = linear * linear * 2.0f32.mul_add(-linear, 3.0);
    transform.translation = motion
        .start_translation
        .lerp(motion.target_translation, eased);
    transform.rotation = motion.start_rotation.slerp(motion.target_rotation, eased);

    if linear >= 1.0 {
        motion.active = false;
    }
}

/// Rotation that points a mesh's default vertical axis (Bevy `+Y`, e.g. a
/// `Cylinder` primitive's rotational axis) along the horizontal direction
/// corresponding to a `map`-frame yaw — the rendering counterpart of
/// `heading_to_bevy_forward`, but aligning an axis of symmetry rather than a
/// declared forward face. The yaw-only fallback for any object with no
/// full-orientation observation (every virtual object; a MOCAP object
/// before its first observation).
pub fn object_rotation(frames: &FrameConversion, yaw_rad: f32) -> Quat {
    let dir_bevy = heading_to_bevy_forward(frames, yaw_rad);
    Quat::from_rotation_arc(Vec3::Y, dir_bevy.normalize())
}

/// Yaw-only rotation for a non-rotationally-symmetric mesh (e.g. a box):
/// faces the mesh's default forward (Bevy `-Z`) toward the `map`-frame yaw,
/// with `+Y` up — the same convention `poll_agent_state_system` uses for
/// the agent body, unlike `object_rotation`'s cylinder-specific "align the
/// symmetry axis."
pub fn forward_facing_rotation(frames: &FrameConversion, yaw_rad: f32) -> Quat {
    let dir_bevy = heading_to_bevy_forward(frames, yaw_rad);
    Transform::default().looking_to(dir_bevy, Vec3::Y).rotation
}

/// Stores the newest platform command for HUD display.
pub fn poll_platform_command_system(
    channels: Res<'_, VisualizerObservationChannels>,
    config: Res<'_, VisualizerConfig>,
    mut latest: ResMut<'_, LatestPlatformCommand>,
) {
    let (channels, config) = (channels.into_inner(), config.into_inner());
    if let Some(command) = drain_latest_for_agent(channels.platform_command_rx(), config.agent_id())
    {
        latest.replace(command);
    }
}

/// Stores the newest operator intent for HUD display.
pub fn poll_operator_intent_system(
    channels: Res<'_, VisualizerObservationChannels>,
    config: Res<'_, VisualizerConfig>,
    mut latest: ResMut<'_, LatestOperatorIntent>,
) {
    let (channels, config) = (channels.into_inner(), config.into_inner());
    if let Some(intent) = drain_latest_for_agent(channels.intent_rx(), config.agent_id()) {
        latest.replace(intent);
    }
}

/// Rebuilds the HUD text from the latest observed intent and command.
pub fn sync_hud_system(
    intent: Res<'_, LatestOperatorIntent>,
    command: Res<'_, LatestPlatformCommand>,
    mut hud: Single<'_, '_, &mut Text, With<HudText>>,
) {
    let changed = intent.is_changed() || command.is_changed();
    let (intent, command) = (intent.into_inner(), command.into_inner());
    if changed {
        **hud = Text::new(hud_text(intent.intent(), command.command()));
    }
}

/// Formats a concise status line for the visualizer HUD.
fn hud_text(intent: Option<&WaypointIntent>, command: Option<&PlatformCommand>) -> String {
    let Some(intent) = intent else {
        return IDLE_HUD_TEXT.to_string();
    };

    // Unknown behavior codes are displayed as UNKNOWN so malformed or newer bus
    // messages do not crash the visualizer.
    let behavior = behavior_from_code(intent.behavior).map_or("UNKNOWN", behavior_label);
    let [x, y, z] = intent.point;
    let command = command.map_or("none", |command| command_type_name(command.command_type));

    format!("{behavior} [{x:.2}, {y:.2}, {z:.2}] | cmd: {command}")
}

#[cfg(test)]
#[path = "../../../tests/unit/visualizer/systems.rs"]
mod systems_tests;
