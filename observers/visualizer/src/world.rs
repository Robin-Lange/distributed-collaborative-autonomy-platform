//! Bevy components and resources owned by the visualizer.
//!
//! These types are intentionally small markers or channel/state containers. They
//! let setup and runtime systems communicate through Bevy's ECS without coupling
//! rendering code directly to the DDS thread implementation.

use bevy::prelude::*;
use core_msgs::{
    AgentStateMsg, ObjectState, PlatformCommand, WaypointIntent,
    catalog::ObjectCatalog,
    frames::{RigidTransform, mat3_to_quat},
};
use crossbeam_channel::Receiver;
use std::collections::HashMap;

/// `map` (ENU) -> Bevy render-frame conversion applied at the render boundary.
#[derive(Resource)]
pub struct FrameConversion {
    t_bevy_map: RigidTransform,
    /// `t_bevy_map`'s rotation as a quaternion, precomputed once — the
    /// primitive `map_orientation_to_bevy` needs to compose a full
    /// orientation (not just a single heading vector) into Bevy.
    bevy_map_rotation: Quat,
}

impl FrameConversion {
    /// Precomputes the inverse of the static `T_map_bevy` config entry.
    pub(super) fn new(t_map_bevy: &RigidTransform) -> Self {
        let t_bevy_map = t_map_bevy.inverse();
        let (qx, qy, qz, qw) = mat3_to_quat(&t_bevy_map.r);
        Self {
            t_bevy_map,
            bevy_map_rotation: bevy::math::DQuat::from_xyzw(qx, qy, qz, qw)
                .as_quat()
                .normalize(),
        }
    }

    /// Converts a `map`-frame point received from the bus into Bevy coordinates.
    pub(super) fn map_to_bevy(&self, p: Vec3) -> Vec3 {
        let out = self
            .t_bevy_map
            .transform_point([f64::from(p.x), f64::from(p.y), f64::from(p.z)]);
        glam::DVec3::from_array(out).as_vec3().to_array().into()
    }

    /// Rotates a `map`-frame free vector (e.g. a heading) into Bevy
    /// coordinates, ignoring translation — the correct primitive for
    /// directions, unlike `map_to_bevy` which is only valid for points.
    pub(super) fn map_to_bevy_vector(&self, v: Vec3) -> Vec3 {
        let out =
            self.t_bevy_map
                .transform_vector([f64::from(v.x), f64::from(v.y), f64::from(v.z)]);
        glam::DVec3::from_array(out).as_vec3().to_array().into()
    }

    /// Composes a full `map`-frame orientation (e.g. a MOCAP-tracked
    /// object's `ObjectState.orientation_map`) into a Bevy
    /// rotation. `q_map` need not be normalized; the result always is.
    pub(super) fn map_orientation_to_bevy(&self, q_map: (f32, f32, f32, f32)) -> Quat {
        let q = Quat::from_xyzw(q_map.0, q_map.1, q_map.2, q_map.3).normalize();
        (self.bevy_map_rotation * q).normalize()
    }
}

/// HUD text shown before any operator intent has been observed.
pub const IDLE_HUD_TEXT: &str = "OBSERVING - no active command";

/// Marker component for the rendered agent body.
#[derive(Component)]
pub struct VisualAgent;

#[derive(Component)]
pub struct AgentId(pub(super) String);

/// Marker component for the single HUD text entity.
#[derive(Component)]
pub struct HudText;

/// Marker component for a rendered catalog object.
#[derive(Component)]
pub struct VisualObject;

/// Render-only interpolation state for a pick/place lifecycle transition.
///
/// Bus state remains authoritative. This component only prevents a discrete
/// FREE/HELD state update from appearing as a teleport in the visualizer.
#[derive(Component)]
pub struct ObjectMotion {
    pub(super) lifecycle: Option<u8>,
    pub(super) start_translation: Vec3,
    pub(super) start_rotation: Quat,
    pub(super) target_translation: Vec3,
    pub(super) target_rotation: Quat,
    pub(super) elapsed_s: f32,
    pub(super) active: bool,
}

impl ObjectMotion {
    pub(super) const fn new(transform: &Transform) -> Self {
        Self {
            lifecycle: None,
            start_translation: transform.translation,
            start_rotation: transform.rotation,
            target_translation: transform.translation,
            target_rotation: transform.rotation,
            elapsed_s: 0.0,
            active: false,
        }
    }
}

/// Wraps the shared, non-Bevy `ObjectCatalog` as a resource.
#[derive(Resource)]
pub struct ObjectCatalogRes(pub(super) ObjectCatalog);

/// Entity spawned for each catalog object, keyed by `object_id`, so the
/// per-tick pose poll knows which entity to move without a linear scan.
#[derive(Resource, Default)]
pub struct VisualObjects(pub(super) HashMap<String, Entity>);

/// Receivers used by systems to observe bus messages from the DDS thread.
#[derive(Resource)]
pub struct VisualizerObservationChannels {
    /// Agent state samples for all agents on the bus.
    state: Receiver<AgentStateMsg>,

    /// Platform commands observed on the bus.
    platform_command: Receiver<PlatformCommand>,

    /// Operator waypoint intents observed on the bus.
    intent: Receiver<WaypointIntent>,

    /// Catalog object lifecycle/pose state observed on the bus.
    object_state: Receiver<ObjectState>,
}

impl VisualizerObservationChannels {
    /// Creates a channel resource from the DDS thread's receivers.
    pub(super) const fn new(
        state: Receiver<AgentStateMsg>,
        platform_command: Receiver<PlatformCommand>,
        intent: Receiver<WaypointIntent>,
        object_state: Receiver<ObjectState>,
    ) -> Self {
        Self {
            state,
            platform_command,
            intent,
            object_state,
        }
    }

    /// Returns the agent-state receiver.
    pub(super) const fn state_rx(&self) -> &Receiver<AgentStateMsg> {
        &self.state
    }

    /// Returns the platform-command receiver.
    pub(super) const fn platform_command_rx(&self) -> &Receiver<PlatformCommand> {
        &self.platform_command
    }

    /// Returns the operator-intent receiver.
    pub(super) const fn intent_rx(&self) -> &Receiver<WaypointIntent> {
        &self.intent
    }

    /// Returns the object-state receiver.
    pub(super) const fn object_state_rx(&self) -> &Receiver<ObjectState> {
        &self.object_state
    }
}

/// Resource holding the latest platform command displayed in the HUD.
#[derive(Resource, Default)]
pub struct LatestPlatformCommand {
    /// Most recent command for the configured agent, if one has been observed.
    command: Option<PlatformCommand>,
}

impl LatestPlatformCommand {
    /// Returns the stored command without consuming it.
    pub(super) const fn command(&self) -> Option<&PlatformCommand> {
        self.command.as_ref()
    }

    /// Replaces the stored command with a newer bus sample.
    pub(super) fn replace(&mut self, command: PlatformCommand) {
        self.command = Some(command);
    }
}

/// Resource holding the latest operator intent displayed in the HUD.
#[derive(Resource, Default)]
pub struct LatestOperatorIntent {
    /// Most recent intent for the configured agent, if one has been observed.
    intent: Option<WaypointIntent>,
}

impl LatestOperatorIntent {
    /// Returns the stored intent without consuming it.
    pub(super) const fn intent(&self) -> Option<&WaypointIntent> {
        self.intent.as_ref()
    }

    /// Replaces the stored intent with a newer bus sample.
    pub(super) fn replace(&mut self, intent: WaypointIntent) {
        self.intent = Some(intent);
    }
}
