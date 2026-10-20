//! Bevy app wiring for the environment simulator.
//!
//! The schedule differs by role. `sim` mode creates a dynamic physics agent that
//! publishes state and completion status. `hybrid` mode creates a kinematic body
//! driven by real-platform state and uses sensor walls to report virtual-wall
//! constraints.

use avian3d::prelude::*;
use bevy::{app::ScheduleRunnerPlugin, prelude::*};

use core_msgs::{catalog::ObjectCatalog, frames::RigidTransform};

use super::{
    config::EnvironmentConfig,
    dds::EnvironmentDdsIo,
    startup, systems,
    world::{FrameConversion, HybridStateStamp, ObjectCatalogRes, ObjectStateStore},
};

/// Builds the simulator app and registers systems for the configured role.
pub fn build_app(
    config: EnvironmentConfig,
    t_map_bevy: RigidTransform,
    dds: EnvironmentDdsIo,
    catalog: ObjectCatalog,
) -> App {
    let mut app = App::new();
    let channels = dds.into_channels();

    // MinimalPlugins plus ScheduleRunnerPlugin keep this process headless while
    // Avian handles physics stepping.
    app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(
        Time::<Fixed>::default().timestep(),
    )))
    .add_plugins(TransformPlugin)
    .add_plugins(PhysicsPlugins::default())
    .insert_resource(config)
    .insert_resource(channels)
    .insert_resource(FrameConversion::new(t_map_bevy))
    .insert_resource(HybridStateStamp::default())
    .insert_resource(ObjectCatalogRes(catalog))
    .insert_resource(ObjectStateStore::default());

    app.add_systems(
        Startup,
        (
            startup::setup_walls,
            startup::setup_obstacle,
            startup::setup_virtual_object_colliders,
            startup::setup_physics_agent,
            startup::setup_hybrid_agent,
            systems::seed_object_states_system,
        ),
    )
    .add_systems(
        FixedUpdate,
        (
            systems::hybrid_real_pose_system,
            systems::read_platform_command_channel_system,
            systems::read_manipulation_command_channel_system,
            systems::drive_agent_system,
            systems::apply_manipulation_status_system,
            systems::publish_hybrid_held_object_pose_system,
            systems::sync_virtual_object_colliders_system,
        )
            .chain(),
    )
    .add_systems(
        FixedPostUpdate,
        (
            systems::halt_agent_on_blocking_contact_system,
            systems::publish_agent_state_system,
            systems::publish_contact_constraint_system,
            systems::tick_manipulation_task_system,
        )
            .chain()
            .after(PhysicsSystems::StepSimulation),
    );

    app
}
