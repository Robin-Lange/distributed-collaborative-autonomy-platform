//! Startup systems that construct the simulator physics world.
//!
//! `sim` role uses solid colliders; `hybrid` role uses sensor colliders so the
//! kinematic hybrid body can overlap geometry for constraint detection without
//! being physically blocked.

use avian3d::prelude::*;
use bevy::prelude::*;
use core_msgs::config::AgentBackend;

use super::{
    config::EnvironmentConfig,
    world::{
        Agent, AgentDriveCommand, AgentId, BlockingDirection, BlockingObjectId, FrameConversion,
        HardStopActive, HybridAgent, HybridDriveTarget, ManipulationTaskState, NonBlocking,
        ObjectCatalogRes, SimulatedAgent, VirtualObjectCollider, VirtualObjectOrientation,
        virtual_object_transform,
    },
};

/// Geometry description used before creating an Avian cuboid collider.
#[derive(Clone, Copy)]
struct BoundaryCollider {
    /// Full collider dimensions.
    size: Vec3,

    /// Collider center in Bevy coordinates.
    center: Vec3,
}

/// Size of the fixed obstacle used to exercise collision and containment behavior.
const DIAGNOSTIC_OBSTACLE_RADIUS: f32 = 0.2;
const DIAGNOSTIC_OBSTACLE_HEIGHT: f32 = 0.8;

/// Center of the fixed diagnostic obstacle in the canonical `map` frame
/// (ENU): 1 m east, 1 m north, resting on the floor. Scene geometry is
/// authored in operator-facing `map` terms and converted to Bevy at spawn,
/// so the obstacle can never sit at a different point than the one every
/// operator-facing surface names.
const DIAGNOSTIC_OBSTACLE_POSITION_MAP: Vec3 =
    Vec3::new(1.0, 1.0, DIAGNOSTIC_OBSTACLE_HEIGHT * 0.5);
const DIAGNOSTIC_OBSTACLE_ID: &str = "obstacle:test_obstacle";
const MIN_OBJECT_COLLIDER_DIMENSION_M: f32 = 0.02;

const SIMULATED_AGENT_LAYER: u32 = 1 << 0;
const HYBRID_AGENT_LAYER: u32 = 1 << 1;
const SOLID_ENVIRONMENT_LAYER: u32 = 1 << 2;
const VIRTUAL_ENVIRONMENT_LAYER: u32 = 1 << 3;

/// Creates solid simulated boundaries, virtual hybrid boundaries, and the
/// simulated floor and ceiling.
///
/// Each side wall is a separate entity carrying its outward `BlockingDirection`,
/// so the stop logic reads the contact normal directly instead of recomputing
/// arena limits from world/agent sizes. Every collider carries an explicit zero
/// restitution so impacts are perfectly inelastic (dead stop, no rebound),
/// matching a real platform. Hybrid boundaries are separate sensor entities so
/// simulated and hybrid agents can share one environment process.
pub fn setup_walls(mut commands: Commands<'_, '_>, config: Res<'_, EnvironmentConfig>) {
    let config = config.into_inner();
    let walls = side_wall_colliders(
        config.world_size(),
        config.wall_thickness(),
        0.0,
        "boundary",
    );
    let has_simulated_agent = config
        .app
        .agents
        .iter()
        .any(|agent| agent.manifest.backend == AgentBackend::Simulated);
    let has_hybrid_agent = config
        .app
        .agents
        .iter()
        .any(|agent| agent.manifest.backend == AgentBackend::Hybrid);

    if has_simulated_agent {
        for (wall, outward_normal, object_id) in &walls {
            commands.spawn((
                Name::new("SolidBoundaryWall"),
                RigidBody::Static,
                boundary_collider(*wall),
                CollisionLayers::new(SOLID_ENVIRONMENT_LAYER, SIMULATED_AGENT_LAYER),
                Restitution::PERFECTLY_INELASTIC,
                Transform::from_translation(wall.center),
                BlockingDirection(*outward_normal),
                BlockingObjectId(object_id.clone()),
            ));
        }
    }

    if has_hybrid_agent {
        for (wall, outward_normal, object_id) in &walls {
            commands.spawn((
                Name::new("VirtualBoundaryWall"),
                RigidBody::Static,
                boundary_collider(*wall),
                Sensor,
                CollisionEventsEnabled,
                CollisionLayers::new(VIRTUAL_ENVIRONMENT_LAYER, HYBRID_AGENT_LAYER),
                Transform::from_translation(wall.center),
                BlockingDirection(*outward_normal),
                BlockingObjectId(object_id.clone()),
            ));
        }
    }

    if has_simulated_agent {
        let (floor, ceiling) =
            floor_and_ceiling_colliders(config.world_size(), config.wall_thickness());

        commands.spawn((
            Name::new("Floor"),
            NonBlocking,
            RigidBody::Static,
            boundary_collider(floor),
            CollisionLayers::new(SOLID_ENVIRONMENT_LAYER, SIMULATED_AGENT_LAYER),
            Restitution::PERFECTLY_INELASTIC,
            Transform::from_translation(floor.center),
        ));
        commands.spawn((
            Name::new("Ceiling"),
            NonBlocking,
            RigidBody::Static,
            boundary_collider(ceiling),
            CollisionLayers::new(SOLID_ENVIRONMENT_LAYER, SIMULATED_AGENT_LAYER),
            Restitution::PERFECTLY_INELASTIC,
            Transform::from_translation(ceiling.center),
        ));
    }
}

/// Creates a solid simulated obstacle and a matching hybrid sensor obstacle.
pub fn setup_obstacle(
    mut commands: Commands<'_, '_>,
    config: Res<'_, EnvironmentConfig>,
    frames: Res<'_, FrameConversion>,
) {
    let (config, frames) = (config.into_inner(), frames.into_inner());
    let position = frames.map_to_bevy(DIAGNOSTIC_OBSTACLE_POSITION_MAP);
    if config
        .app
        .agents
        .iter()
        .any(|agent| agent.manifest.backend == AgentBackend::Simulated)
    {
        commands.spawn((
            Name::new("SolidDiagnosticObstacle"),
            RigidBody::Static,
            Collider::cylinder(DIAGNOSTIC_OBSTACLE_RADIUS, DIAGNOSTIC_OBSTACLE_HEIGHT),
            CollisionLayers::new(SOLID_ENVIRONMENT_LAYER, SIMULATED_AGENT_LAYER),
            Restitution::PERFECTLY_INELASTIC,
            Transform::from_translation(position),
            BlockingObjectId(DIAGNOSTIC_OBSTACLE_ID.to_owned()),
        ));
    }
    if config
        .app
        .agents
        .iter()
        .any(|agent| agent.manifest.backend == AgentBackend::Hybrid)
    {
        commands.spawn((
            Name::new("VirtualDiagnosticObstacle"),
            RigidBody::Static,
            Collider::cylinder(DIAGNOSTIC_OBSTACLE_RADIUS, DIAGNOSTIC_OBSTACLE_HEIGHT),
            Sensor,
            CollisionEventsEnabled,
            CollisionLayers::new(VIRTUAL_ENVIRONMENT_LAYER, HYBRID_AGENT_LAYER),
            Transform::from_translation(position),
            BlockingObjectId(DIAGNOSTIC_OBSTACLE_ID.to_owned()),
        ));
    }
}

/// Creates collision geometry for every virtual catalog object.
///
/// Simulated agents receive solid static objects. Hybrid agents receive
/// sensors at the same poses so overlap produces a virtual collision veto
/// without applying simulation forces to the real platform.
pub fn setup_virtual_object_colliders(
    mut commands: Commands<'_, '_>,
    config: Res<'_, EnvironmentConfig>,
    catalog: Res<'_, ObjectCatalogRes>,
    frames: Res<'_, FrameConversion>,
) {
    let (config, catalog, frames) = (
        config.into_inner(),
        catalog.into_inner(),
        frames.into_inner(),
    );
    let has_simulated_agent = config
        .app
        .agents
        .iter()
        .any(|agent| agent.manifest.backend == AgentBackend::Simulated);
    let has_hybrid_agent = config
        .app
        .agents
        .iter()
        .any(|agent| agent.manifest.backend == AgentBackend::Hybrid);

    for object in catalog.0.iter() {
        let Some(pose) = object.virtual_initial_pose_map() else {
            continue;
        };
        let orientation = virtual_object_orientation(object.shape());
        let transform = virtual_object_transform(frames, pose, orientation);

        if has_simulated_agent {
            commands.spawn((
                Name::new(format!("SolidVirtualObject_{}", object.object_id())),
                RigidBody::Static,
                virtual_object_collider(object),
                CollisionLayers::new(SOLID_ENVIRONMENT_LAYER, SIMULATED_AGENT_LAYER),
                Restitution::PERFECTLY_INELASTIC,
                transform,
                BlockingObjectId(object.object_id().to_owned()),
                VirtualObjectCollider {
                    object_id: object.object_id().to_owned(),
                    orientation,
                },
            ));
        }

        if has_hybrid_agent {
            commands.spawn((
                Name::new(format!("HybridVirtualObject_{}", object.object_id())),
                RigidBody::Static,
                virtual_object_collider(object),
                Sensor,
                CollisionEventsEnabled,
                CollisionLayers::new(VIRTUAL_ENVIRONMENT_LAYER, HYBRID_AGENT_LAYER),
                transform,
                BlockingObjectId(object.object_id().to_owned()),
                VirtualObjectCollider {
                    object_id: object.object_id().to_owned(),
                    orientation,
                },
            ));
        }
    }
}

fn virtual_object_collider(object: &core_msgs::catalog::ObjectSpec) -> Collider {
    match object.shape() {
        "cylinder" => Collider::cylinder(
            object.radius_m().max(MIN_OBJECT_COLLIDER_DIMENSION_M),
            object.length_m().max(MIN_OBJECT_COLLIDER_DIMENSION_M),
        ),
        "box" => Collider::cuboid(
            object.width_m().max(MIN_OBJECT_COLLIDER_DIMENSION_M),
            object.height_m().max(MIN_OBJECT_COLLIDER_DIMENSION_M),
            object.length_m().max(MIN_OBJECT_COLLIDER_DIMENSION_M),
        ),
        _ => Collider::sphere(object.radius_m().max(MIN_OBJECT_COLLIDER_DIMENSION_M)),
    }
}

fn virtual_object_orientation(shape: &str) -> VirtualObjectOrientation {
    match shape {
        "cylinder" => VirtualObjectOrientation::AxisAlongHeading,
        "box" => VirtualObjectOrientation::ForwardAlongHeading,
        _ => VirtualObjectOrientation::Invariant,
    }
}

/// Creates the dynamic simulated agent body controlled by `drive_agent_system`.
///
/// The spawn transform's rotation is an initial condition only (identity —
/// facing Bevy `-Z`, the declared body-forward axis); `drive_agent_system`
/// then turns the body to face the direction of travel as it seeks (holonomic
/// — rotate and translate together).
pub fn setup_physics_agent(mut commands: Commands<'_, '_>, config: Res<'_, EnvironmentConfig>) {
    let config = config.into_inner();
    for agent_config in config
        .app
        .agents
        .iter()
        .filter(|agent| agent.manifest.backend == AgentBackend::Simulated)
    {
        commands.spawn((
            Name::new(format!("Agent_{}", agent_config.runtime.agent_id)),
            Agent,
            SimulatedAgent,
            (
                AgentId(agent_config.runtime.agent_id.clone()),
                AgentDriveCommand::Hold,
                ManipulationTaskState::default(),
                HardStopActive::default(),
                HybridDriveTarget::default(),
            ),
            Transform::from_translation(Vec3::from_array(agent_config.initial_position)),
            RigidBody::Dynamic,
            Collider::cuboid(
                agent_config.size[1],
                agent_config.size[2],
                agent_config.size[0],
            ),
            CollisionLayers::new(SIMULATED_AGENT_LAYER, SOLID_ENVIRONMENT_LAYER),
            (
                Mass(39.7),
                CollisionEventsEnabled,
                CollidingEntities::default(),
            ),
            LinearVelocity::ZERO,
            AngularVelocity::ZERO,
            LockedAxes::new().lock_rotation_x().lock_rotation_z(),
            SweptCcd::LINEAR,
            Friction::new(0.0).with_combine_rule(CoefficientCombine::Min),
            Restitution::PERFECTLY_INELASTIC.with_combine_rule(CoefficientCombine::Min),
        ));
    }
}

/// Creates the kinematic hybrid agent body that mirrors the real platform.
///
/// The spawn transform is only an initial placeholder: `hybrid_real_pose_system`
/// overwrites it with the real platform's pose every tick a real-authority state
/// sample is available, and leaves it in place until the first sample arrives.
pub fn setup_hybrid_agent(mut commands: Commands<'_, '_>, config: Res<'_, EnvironmentConfig>) {
    let config = config.into_inner();
    for agent_config in config
        .app
        .agents
        .iter()
        .filter(|agent| agent.manifest.backend == AgentBackend::Hybrid)
    {
        commands.spawn((
            Name::new(format!("HybridAgent_{}", agent_config.runtime.agent_id)),
            Agent,
            HybridAgent,
            (
                AgentId(agent_config.runtime.agent_id.clone()),
                AgentDriveCommand::Hold,
                ManipulationTaskState::default(),
                HardStopActive::default(),
                HybridDriveTarget::default(),
            ),
            Transform::from_translation(Vec3::from_array(agent_config.initial_position)),
            RigidBody::Kinematic,
            Collider::cuboid(
                agent_config.size[1],
                agent_config.size[2],
                agent_config.size[0],
            ),
            CollisionLayers::new(HYBRID_AGENT_LAYER, VIRTUAL_ENVIRONMENT_LAYER),
            Mass(39.7),
            CollisionEventsEnabled,
            CollidingEntities::default(),
        ));
    }
}

/// Converts a boundary geometry record into an Avian cuboid collider.
fn boundary_collider(boundary: BoundaryCollider) -> Collider {
    Collider::cuboid(boundary.size.x, boundary.size.y, boundary.size.z)
}

/// Computes the four side-wall colliders around the X/Z world bounds, each
/// paired with its outward normal (the direction that points out of the arena,
/// i.e. the direction a command pushes when driving into that wall).
///
/// `inset` shifts every wall inward by that distance along its outward
/// normal (zero for the permitted boundary itself), and `object_id_prefix` names the resulting
/// object ids (`"boundary"`).
fn side_wall_colliders(
    world_size: [f32; 3],
    thickness: f32,
    inset: f32,
    object_id_prefix: &str,
) -> [(BoundaryCollider, Vec3, String); 4] {
    let [world_x, world_y, world_z] = world_size;
    let half_thickness = thickness * 0.5;
    let half_x = world_x.mul_add(0.5, -inset);
    let half_z = world_z.mul_add(0.5, -inset);
    let center_y = world_y * 0.5;

    [
        (
            BoundaryCollider {
                size: Vec3::new(thickness, world_y, world_z),
                center: Vec3::new(-(half_x + half_thickness), center_y, 0.0),
            },
            Vec3::NEG_X,
            format!("{object_id_prefix}:-x"),
        ),
        (
            BoundaryCollider {
                size: Vec3::new(thickness, world_y, world_z),
                center: Vec3::new(half_x + half_thickness, center_y, 0.0),
            },
            Vec3::X,
            format!("{object_id_prefix}:+x"),
        ),
        (
            BoundaryCollider {
                size: Vec3::new(world_x, world_y, thickness),
                center: Vec3::new(0.0, center_y, -(half_z + half_thickness)),
            },
            Vec3::NEG_Z,
            format!("{object_id_prefix}:+y"),
        ),
        (
            BoundaryCollider {
                size: Vec3::new(world_x, world_y, thickness),
                center: Vec3::new(0.0, center_y, half_z + half_thickness),
            },
            Vec3::Z,
            format!("{object_id_prefix}:-y"),
        ),
    ]
}

/// Computes floor and ceiling colliders from world dimensions and wall thickness.
fn floor_and_ceiling_colliders(
    world_size: [f32; 3],
    thickness: f32,
) -> (BoundaryCollider, BoundaryCollider) {
    let [world_x, world_y, world_z] = world_size;
    let half_thickness = thickness * 0.5;

    let floor = BoundaryCollider {
        size: Vec3::new(world_x, thickness, world_z),
        center: Vec3::new(0.0, -half_thickness, 0.0),
    };
    let ceiling = BoundaryCollider {
        size: Vec3::new(world_x, thickness, world_z),
        center: Vec3::new(0.0, world_y + half_thickness, 0.0),
    };

    (floor, ceiling)
}

#[cfg(test)]
#[path = "../../../tests/unit/environment_sim/startup.rs"]
mod startup_tests;
