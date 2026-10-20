use super::*;
use bevy::prelude::{App, Name, Startup, With};
use core_msgs::config::{AgentBackend, AgentConfig, AppConfig};

fn app_with_agents(backends: &[AgentBackend]) -> App {
    let config = AppConfig {
        agents: backends
            .iter()
            .enumerate()
            .map(|(index, backend)| {
                let mut agent = AgentConfig::default();
                agent.manifest.backend = *backend;
                agent.runtime.agent_id = format!("agent_{index:02}");
                agent.manifest.agent_id.clone_from(&agent.runtime.agent_id);
                agent
            })
            .collect(),
        ..AppConfig::default()
    };

    let mut app = App::new();
    app.insert_resource(EnvironmentConfig::new(config));
    app.add_systems(Startup, setup_walls);
    app.update();
    app
}

fn app_with_virtual_objects(backends: &[AgentBackend]) -> App {
    let app_config = AppConfig {
        agents: backends
            .iter()
            .enumerate()
            .map(|(index, backend)| {
                let mut agent = AgentConfig::default();
                agent.manifest.backend = *backend;
                agent.runtime.agent_id = format!("agent_{index:02}");
                agent.manifest.agent_id.clone_from(&agent.runtime.agent_id);
                agent
            })
            .collect(),
        ..AppConfig::default()
    };
    let frames = core_msgs::frames::FramesConfig::from_config(&app_config.frames);
    let catalog = core_msgs::catalog::ObjectCatalog::from_config(app_config.objects.clone());

    let mut app = App::new();
    app.insert_resource(EnvironmentConfig::new(app_config));
    app.insert_resource(FrameConversion::new(frames.t_map_bevy));
    app.insert_resource(ObjectCatalogRes(catalog));
    app.add_systems(Startup, setup_virtual_object_colliders);
    app.update();
    app
}

fn named_entity_count(app: &mut App, expected_name: &str) -> usize {
    let world = app.world_mut();
    let mut query = world.query::<&Name>();
    query
        .iter(world)
        .filter(|name| name.as_str() == expected_name)
        .count()
}

#[test]
fn hybrid_boundaries_are_sensors_without_solid_duplicates() {
    let mut app = app_with_agents(&[AgentBackend::Hybrid]);

    assert_eq!(named_entity_count(&mut app, "VirtualBoundaryWall"), 4);
    assert_eq!(named_entity_count(&mut app, "SolidBoundaryWall"), 0);

    let world = app.world_mut();
    let mut sensors = world.query_filtered::<&Name, With<Sensor>>();
    assert_eq!(
        sensors
            .iter(world)
            .filter(|name| name.as_str() == "VirtualBoundaryWall")
            .count(),
        4
    );
}

#[test]
fn mixed_sim_and_hybrid_agents_receive_separate_boundary_geometry() {
    let mut app = app_with_agents(&[AgentBackend::Simulated, AgentBackend::Hybrid]);

    assert_eq!(named_entity_count(&mut app, "SolidBoundaryWall"), 4);
    assert_eq!(named_entity_count(&mut app, "VirtualBoundaryWall"), 4);
}

#[test]
fn hybrid_virtual_objects_are_blocking_sensors() {
    let mut app = app_with_virtual_objects(&[AgentBackend::Hybrid]);
    let world = app.world_mut();
    let mut objects = world.query::<(
        &VirtualObjectCollider,
        &BlockingObjectId,
        Has<Sensor>,
        &Collider,
    )>();
    let objects = objects.iter(world).collect::<Vec<_>>();

    assert_eq!(objects.len(), 1);
    assert_eq!(objects[0].0.object_id, "cylinder_pink_01");
    assert_eq!(objects[0].1.0, "cylinder_pink_01");
    assert!(objects[0].2);
}

#[test]
fn mixed_backends_receive_separate_virtual_object_colliders() {
    let mut app = app_with_virtual_objects(&[AgentBackend::Simulated, AgentBackend::Hybrid]);
    let world = app.world_mut();
    let mut objects = world.query::<(&VirtualObjectCollider, Has<Sensor>)>();
    let objects = objects.iter(world).collect::<Vec<_>>();

    assert_eq!(objects.len(), 2);
    assert_eq!(objects.iter().filter(|(_, sensor)| *sensor).count(), 1);
}
