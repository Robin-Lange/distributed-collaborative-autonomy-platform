//! ECS components and resources shared by environment simulator systems.
//!
//! The simulator communicates with the DDS thread through channel resources and
//! stores the current drive command as ECS state. Marker components distinguish
//! agents, walls, and floors for physics/contact systems.

use bevy::prelude::{Component, Quat, Resource, Transform, Vec3};
use core_msgs::{
    AgentStateMsg, ConstraintEvent, ManipulationCommand, ManipulationStatus, ObjectState,
    PlatformCommand, PoseXYZYaw, VelocitySetpoint, catalog::ObjectCatalog, frames::RigidTransform,
};
use crossbeam_channel::{Receiver, Sender};
use std::collections::HashMap;

/// Boundary conversion between Bevy's internal Y-up frame and the canonical
/// `map` (ENU) frame carried on the bus.
#[derive(Resource)]
pub struct FrameConversion {
    t_map_bevy: RigidTransform,
    t_bevy_map: RigidTransform,
}

impl FrameConversion {
    /// Precomputes both directions from the static `T_map_bevy` config entry.
    pub(super) fn new(t_map_bevy: RigidTransform) -> Self {
        let t_bevy_map = t_map_bevy.inverse();
        Self {
            t_map_bevy,
            t_bevy_map,
        }
    }

    /// Converts a Bevy-frame point into the `map` frame for bus publication.
    pub(super) fn bevy_to_map(&self, p: Vec3) -> Vec3 {
        transform_vec3(&self.t_map_bevy, p)
    }

    /// Converts a `map`-frame point received from the bus into Bevy coordinates.
    pub(super) fn map_to_bevy(&self, p: Vec3) -> Vec3 {
        transform_vec3(&self.t_bevy_map, p)
    }

    /// Rotates a Bevy-frame free vector (e.g. a facing direction) into `map`,
    /// ignoring translation — the correct primitive for headings, unlike
    /// `bevy_to_map` which is only valid for points.
    pub(super) fn bevy_to_map_vector(&self, v: Vec3) -> Vec3 {
        transform_vec3_direction(&self.t_map_bevy, v)
    }

    /// Rotates a `map`-frame free vector (e.g. a commanded heading) into
    /// Bevy coordinates, ignoring translation.
    pub(super) fn map_to_bevy_vector(&self, v: Vec3) -> Vec3 {
        transform_vec3_direction(&self.t_bevy_map, v)
    }
}

fn transform_vec3(t: &RigidTransform, p: Vec3) -> Vec3 {
    let out = t.transform_point([f64::from(p.x), f64::from(p.y), f64::from(p.z)]);
    glam::DVec3::from_array(out).as_vec3().to_array().into()
}

fn transform_vec3_direction(t: &RigidTransform, v: Vec3) -> Vec3 {
    let out = t.transform_vector([f64::from(v.x), f64::from(v.y), f64::from(v.z)]);
    glam::DVec3::from_array(out).as_vec3().to_array().into()
}

/// Marker component for the simulated or hybrid-tracked agent body.
#[derive(Component)]
pub struct Agent;

/// Marker for an Avian-owned simulated platform body.
#[derive(Component)]
pub struct SimulatedAgent;

/// Marker for a kinematic body mirroring a hardware platform.
#[derive(Component)]
pub struct HybridAgent;

/// Per-entity agent identity, set from `agent_config.runtime.agent_id` at
/// spawn. Every per-agent system routes bus messages and per-agent state by
/// matching this against the `agent_id` field messages already carry.
#[derive(Component)]
pub struct AgentId(pub(super) String);

/// Stamp of the newest real-platform state applied to the hybrid body.
///
/// Lets containment/constraint events report how stale the state driving the
/// hybrid body was at the moment a hard stop was raised.
/// Stays at zero in `sim` role, where there is no hybrid-tracked real state.
#[derive(Resource, Default, Clone, Copy)]
pub struct HybridStateStamp {
    stamp_ms: u64,
}

impl HybridStateStamp {
    /// Records the stamp of a newly applied hybrid state sample.
    pub(super) const fn set(&mut self, stamp_ms: u64) {
        self.stamp_ms = stamp_ms;
    }
}

/// Outward normal of a boundary wall — the direction that points out of the
/// arena. Contact detection reports only *that* the agent and wall touch, and
/// this stored normal decides which way "into the wall" points, so push-gating
/// reads it directly instead of recomputing arena limits from world/agent sizes.
#[derive(Component)]
pub struct BlockingDirection(pub(super) Vec3);

/// Stable identity for a blocking collider reported in constraint events.
#[derive(Component, Clone, Debug)]
pub struct BlockingObjectId(pub(super) String);

/// Catalog identity and orientation convention for a virtual-object collider.
#[derive(Component)]
pub struct VirtualObjectCollider {
    pub(super) object_id: String,
    pub(super) orientation: VirtualObjectOrientation,
}

/// How a catalog shape's local axes correspond to its map-frame yaw.
#[derive(Clone, Copy)]
pub enum VirtualObjectOrientation {
    /// Cylinder local `+Y` is its long axis.
    AxisAlongHeading,

    /// Box local `-Z` is its forward axis.
    ForwardAlongHeading,

    /// Rotation does not change the collision shape.
    Invariant,
}

/// Converts a catalog pose into the matching Bevy collider transform.
pub fn virtual_object_transform(
    frames: &FrameConversion,
    pose: PoseXYZYaw,
    orientation: VirtualObjectOrientation,
) -> Transform {
    let heading_map = Vec3::new(pose.yaw_rad.cos(), pose.yaw_rad.sin(), 0.0);
    let heading_bevy = frames.map_to_bevy_vector(heading_map).normalize();
    let rotation = match orientation {
        VirtualObjectOrientation::AxisAlongHeading => {
            Quat::from_rotation_arc(Vec3::Y, heading_bevy)
        }
        VirtualObjectOrientation::ForwardAlongHeading => {
            Transform::default()
                .looking_to(heading_bevy, Vec3::Y)
                .rotation
        }
        VirtualObjectOrientation::Invariant => Quat::IDENTITY,
    };

    Transform::from_translation(frames.map_to_bevy(Vec3::new(pose.x, pose.y, pose.z)))
        .with_rotation(rotation)
}

/// Marker for structural surfaces (floor, ceiling) that must never become a
/// hard-stop constraint: resting on the floor is normal and the ceiling is out
/// of a ground robot's reach. Every other collider contact is blocking.
#[derive(Component)]
pub struct NonBlocking;

/// Channel endpoints used by simulation systems to talk to the DDS thread.
#[derive(Resource)]
pub struct EnvironmentChannels {
    /// Commands received from the onboard agent.
    pub(super) command_rx: Receiver<PlatformCommand>,

    /// State samples published by pure simulation mode.
    pub(super) state_tx: Sender<AgentStateMsg>,

    /// Clone of the DDS thread's state receiver, used only to evict the
    /// oldest queued sample when `state_tx` is full (latest-wins stream).
    pub(super) state_evict_rx: Receiver<AgentStateMsg>,

    /// Safety/constraint events published by contact systems.
    pub(super) constraint_tx: Sender<ConstraintEvent>,

    /// Real-authority state samples consumed by hybrid mode.
    pub(super) hybrid_state_rx: Receiver<AgentStateMsg>,

    /// Approved manipulation commands received from the onboard agent.
    pub(super) manipulation_command_rx: Receiver<ManipulationCommand>,

    /// Manipulation execution status messages published by `sim` role.
    pub(super) manipulation_status_tx: Sender<ManipulationStatus>,

    /// Manipulation execution status messages published externally by the
    /// real-Spot sidecar, consumed by `hybrid` role only.
    pub(super) manipulation_status_rx: Receiver<ManipulationStatus>,

    /// Catalog object lifecycle/pose state published by `sim` role.
    pub(super) object_state_tx: Sender<ObjectState>,
}

/// Wraps the shared, non-Bevy `ObjectCatalog` as a resource.
#[derive(Resource)]
pub struct ObjectCatalogRes(pub(super) ObjectCatalog);

/// Current lifecycle/pose per catalog object, keyed by `object_id`.
///
/// `sim` role owns object truth directly, executing the deterministic
/// gesture itself. `hybrid` role also owns object truth for the
/// *virtual* object catalog — seeded FREE at startup the same way — but
/// applies attach/detach only once the real-Spot sidecar (which executes the
/// actual arm sequence) reports a terminal success, rather than on its own
/// timer. Real mode does not run `environment_sim`: the sidecar can own
/// virtual object state with `--owns-object-state`, while `object_tracker`
/// owns state for physical MOCAP-tracked objects.
#[derive(Resource, Default)]
pub struct ObjectStateStore(pub(super) HashMap<String, ObjectState>);

/// Manipulation task in flight, if any. `sim` role completes it after a
/// deterministic delay; `hybrid` role completes it only once the real-Spot
/// sidecar reports a terminal `ManipulationStatus` for the same command id.
#[derive(Component, Default)]
pub struct ManipulationTaskState {
    pub(super) active: Option<ActiveManipulationTask>,
}

/// One in-flight manipulation task's deterministic execution state.
pub struct ActiveManipulationTask {
    pub(super) command_id: String,
    pub(super) agent_id: String,
    pub(super) object_id: String,
    pub(super) verb: u8,
    pub(super) target_pose: core_msgs::PoseXYZYaw,
    pub(super) deadline_ms: u64,
}

/// True while the sim-role agent is currently reporting an active hard-stop
/// wall/obstacle contact. Set by `publish_contact_constraint_system`; read by
/// `sim` role's manipulation task system to abort an in-flight task in the
/// same tick that a hard stop becomes active. `hybrid` role does not consult
/// this resource for
/// manipulation: the real-Spot sidecar executes the arm sequence and already
/// aborts it on its own reflex-stop preemption (any hard stop, virtual-wall
/// or Spot fault alike), so hybrid only needs to observe the resulting
/// terminal `ManipulationStatus` rather than re-deriving the abort itself.
///
/// Per-agent Component (not a global Resource): each agent's manipulation task
/// aborts on that agent's own hard stop, so the flag must be scoped to its
/// entity rather than shared across all simulated agents.
#[derive(Component, Default)]
pub struct HardStopActive(pub(super) bool);

/// Current drive command consumed by the physics controller.
#[derive(Component, Default, Debug)]
pub enum AgentDriveCommand {
    /// Execute this velocity setpoint (`flat_body` frame) until replaced.
    /// `target` (bevy coords) is the command's map goal, kept ONLY for
    /// contact push-gating (`command_pushes_along`), not for control.
    Drive {
        setpoint: VelocitySetpoint,
        target: Vec3,
    },

    /// Stop immediately and stay exactly at the current position; never
    /// re-dispatches or walks back toward a target.
    #[default]
    Hold,
}

/// Target of the newest approved velocity command for a hybrid agent, in
/// Bevy coordinates; `None` while holding.
///
/// A hybrid body's `AgentDriveCommand` is always `Hold` (it is a kinematic
/// follower of the real platform, not locally driven — see
/// `drive_command_from_platform_command`), so collision gating has no
/// direction to read from it the way sim's `Drive` branch does. This
/// component restores that missing signal without feeding into
/// `drive_agent_system`'s physics: `command_pushes_along` reads it only to
/// decide whether the current command still pushes into a contact, exactly
/// mirroring sim's "Hold never counts as pushing" escape hatch — otherwise a
/// hybrid collision latches forever, since a hard stop blocks every command
/// including the retreat that would clear it.
#[derive(Component, Default)]
pub struct HybridDriveTarget(pub(super) Option<Vec3>);
