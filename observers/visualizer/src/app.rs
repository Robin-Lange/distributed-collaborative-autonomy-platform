//! Bevy app construction for the visualizer.
//!
//! This module wires together resources and schedules. Startup systems create
//! the static scene, while update systems poll the bus-backed channels and keep
//! the rendered agent/HUD synchronized with the newest messages.

use bevy::prelude::*;

use core_msgs::{catalog::ObjectCatalog, frames::RigidTransform};

use super::{
    config::VisualizerConfig,
    dds::VisualizerDdsIo,
    setup, systems,
    world::{
        FrameConversion, LatestOperatorIntent, LatestPlatformCommand, ObjectCatalogRes,
        VisualObjects, VisualizerObservationChannels,
    },
};

pub fn build_app(
    config: VisualizerConfig,
    t_map_bevy: &RigidTransform,
    dds: VisualizerDdsIo,
    catalog: ObjectCatalog,
) -> App {
    let mut app = App::new();
    let (state_rx, platform_command_rx, intent_rx, object_state_rx) = dds.into_channels();

    app.add_plugins(DefaultPlugins)
        .insert_resource(config)
        .insert_resource(FrameConversion::new(t_map_bevy))
        .insert_resource(VisualizerObservationChannels::new(
            state_rx,
            platform_command_rx,
            intent_rx,
            object_state_rx,
        ))
        .insert_resource(LatestPlatformCommand::default())
        .insert_resource(LatestOperatorIntent::default())
        .insert_resource(ObjectCatalogRes(catalog))
        .insert_resource(VisualObjects::default())
        .add_systems(
            Startup,
            (
                setup::setup_scene,
                setup::setup_camera,
                setup::setup_hud,
                setup::setup_visual_agent,
                setup::setup_diagnostic_obstacle,
                setup::setup_visual_objects,
                setup::draw_datum_axes,
            ),
        )
        .add_systems(
            Update,
            (
                systems::poll_agent_state_system,
                systems::poll_platform_command_system,
                systems::poll_operator_intent_system,
                systems::poll_object_state_system,
                systems::animate_object_motion_system,
                systems::sync_hud_system,
            )
                .chain(),
        );

    app
}
