use super::*;
use crate::world::VirtualObjectOrientation;
use core_msgs::config::AppConfig;

fn sim_config() -> EnvironmentConfig {
    EnvironmentConfig::new(AppConfig::default())
}

fn hybrid_config() -> EnvironmentConfig {
    sim_config()
}

const AGENT_POS: Vec3 = Vec3::new(0.0, 0.0, -3.5);
// Outward normal of a wall the agent is touching to its south (-z).
const SOUTH_WALL_NORMAL: Vec3 = Vec3::new(0.0, 0.0, -1.0);

/// Regression test for the hybrid collision deadlock: a hybrid agent that
/// contacts a wall must be able to clear the hard stop once commanded to
/// hold, exactly like the sim path already does — otherwise the safety hold
/// that follows a collision blocks every future command (including the
/// retreat that would clear it) forever.
#[test]
fn hybrid_hold_is_never_pushing() {
    let config = hybrid_config();
    assert!(!command_pushes_along(
        &AgentDriveCommand::Hold,
        AGENT_POS,
        SOUTH_WALL_NORMAL,
        None,
        true,
        &config,
    ));
}

#[test]
fn hybrid_target_away_from_wall_is_not_pushing() {
    let config = hybrid_config();
    let retreat_target = Vec3::new(0.0, 0.0, 0.0);
    assert!(!command_pushes_along(
        &AgentDriveCommand::Hold,
        AGENT_POS,
        SOUTH_WALL_NORMAL,
        Some(retreat_target),
        true,
        &config,
    ));
}

#[test]
fn hybrid_target_into_wall_is_pushing() {
    let config = hybrid_config();
    let deeper_target = Vec3::new(0.0, 0.0, -5.0);
    assert!(command_pushes_along(
        &AgentDriveCommand::Hold,
        AGENT_POS,
        SOUTH_WALL_NORMAL,
        Some(deeper_target),
        true,
        &config,
    ));
}

#[test]
fn sim_drive_into_wall_pushes_but_hold_does_not() {
    let config = sim_config();
    let drive_into_wall = AgentDriveCommand::Drive {
        setpoint: core_msgs::VelocitySetpoint::default(),
        target: Vec3::new(0.0, 0.0, -5.0),
    };

    assert!(command_pushes_along(
        &drive_into_wall,
        AGENT_POS,
        SOUTH_WALL_NORMAL,
        None,
        false,
        &config,
    ));
    assert!(!command_pushes_along(
        &AgentDriveCommand::Hold,
        AGENT_POS,
        SOUTH_WALL_NORMAL,
        None,
        false,
        &config,
    ));
}

#[test]
fn active_sim_hard_stop_prevents_velocity_from_being_reapplied() {
    let mut app = App::new();
    app.insert_resource(Time::<()>::default());
    app.insert_resource(FrameConversion::new(
        core_msgs::frames::RigidTransform::from_rotation_translation(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0; 3],
        ),
    ));
    app.add_systems(Update, drive_agent_system);
    let entity = app
        .world_mut()
        .spawn((
            SimulatedAgent,
            AgentDriveCommand::Drive {
                setpoint: core_msgs::VelocitySetpoint {
                    forward_m_s: 1.0,
                    left_m_s: 0.5,
                    yaw_rate_rad_s: 0.2,
                },
                target: Vec3::new(5.0, 0.0, 0.0),
            },
            HardStopActive(true),
            Transform::default(),
            LinearVelocity(Vec3::splat(0.5)),
            AngularVelocity(Vec3::splat(0.5)),
            LockedAxes::new(),
        ))
        .id();

    app.update();

    let world = app.world();
    assert_eq!(world.get::<LinearVelocity>(entity).unwrap().0, Vec3::ZERO);
    assert_eq!(world.get::<AngularVelocity>(entity).unwrap().0, Vec3::ZERO);
    assert_eq!(
        world.get::<LockedAxes>(entity).unwrap().to_bits(),
        LockedAxes::ALL_LOCKED.to_bits()
    );
}

#[test]
fn held_object_pose_updates_on_every_hybrid_platform_pose() {
    let config = hybrid_config();
    let mut store = ObjectStateStore::default();
    store.0.insert(
        "held".to_owned(),
        ObjectState {
            object_id: "held".to_owned(),
            lifecycle: object_lifecycle_held(),
            holder_agent_id: "agent_01".to_owned(),
            pose_valid: true,
            ..Default::default()
        },
    );
    store.0.insert(
        "free".to_owned(),
        ObjectState {
            object_id: "free".to_owned(),
            lifecycle: object_lifecycle_free(),
            pose: PoseXYZYaw {
                x: 8.0,
                ..Default::default()
            },
            pose_valid: true,
            ..Default::default()
        },
    );

    let first = update_held_object_poses(
        &config,
        &mut store,
        "agent_01",
        Vec3::new(1.0, 2.0, 0.0),
        0.0,
        20,
    );
    let second = update_held_object_poses(
        &config,
        &mut store,
        "agent_01",
        Vec3::new(2.0, 3.0, 0.0),
        std::f32::consts::FRAC_PI_2,
        21,
    );

    assert_eq!(first.len(), 1);
    assert_eq!(second.len(), 1);
    assert!((first[0].pose.x - 1.55).abs() < 1e-5);
    assert!((first[0].pose.y - 2.0).abs() < 1e-5);
    assert!((second[0].pose.x - 2.0).abs() < 1e-5);
    assert!((second[0].pose.y - 3.55).abs() < 1e-5);
    assert_eq!(second[0].stamp_ms, 21);
    assert!((store.0["free"].pose.x - 8.0).abs() < f32::EPSILON);
}

#[test]
fn successful_pick_effect_attaches_the_object_to_the_executing_agent() {
    let config = hybrid_config();
    let task = ActiveManipulationTask {
        command_id: "pick-1".to_owned(),
        agent_id: "agent_01".to_owned(),
        object_id: "virtual".to_owned(),
        verb: manipulation_verb_pick_up(),
        target_pose: PoseXYZYaw::default(),
        deadline_ms: 0,
    };

    let (lifecycle, holder_agent_id, pose) =
        manipulation_effect(&config, &task, Vec3::new(1.0, 2.0, 0.0), 0.0);

    assert_eq!(lifecycle, object_lifecycle_held());
    assert_eq!(holder_agent_id, "agent_01");
    assert!((pose.x - 1.55).abs() < 1e-5);
    assert!((pose.y - 2.0).abs() < 1e-5);
}

#[test]
fn successful_place_effect_releases_the_object_at_the_requested_pose() {
    let config = hybrid_config();
    let target_pose = PoseXYZYaw {
        x: 3.0,
        y: 4.0,
        z: 0.2,
        yaw_rad: 0.5,
    };
    let task = ActiveManipulationTask {
        command_id: "place-1".to_owned(),
        agent_id: "agent_01".to_owned(),
        object_id: "virtual".to_owned(),
        verb: core_msgs::manipulation_verb_drop_off(),
        target_pose,
        deadline_ms: 0,
    };

    let (lifecycle, holder_agent_id, pose) =
        manipulation_effect(&config, &task, Vec3::new(1.0, 2.0, 0.0), 0.0);

    assert_eq!(lifecycle, object_lifecycle_free());
    assert!(holder_agent_id.is_empty());
    assert_eq!(pose, target_pose);
}

#[test]
fn held_virtual_object_disables_its_collider_until_release() {
    let mut store = ObjectStateStore::default();
    store.0.insert(
        "virtual".to_owned(),
        ObjectState {
            object_id: "virtual".to_owned(),
            lifecycle: object_lifecycle_held(),
            holder_agent_id: "agent_01".to_owned(),
            pose: PoseXYZYaw {
                x: 1.0,
                y: 2.0,
                z: 0.3,
                yaw_rad: 0.0,
            },
            pose_valid: true,
            ..Default::default()
        },
    );

    let mut app = App::new();
    app.insert_resource(FrameConversion::new(
        core_msgs::frames::RigidTransform::from_rotation_translation(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0; 3],
        ),
    ));
    app.insert_resource(store);
    app.add_systems(Update, sync_virtual_object_colliders_system);
    let entity = app
        .world_mut()
        .spawn((
            VirtualObjectCollider {
                object_id: "virtual".to_owned(),
                orientation: VirtualObjectOrientation::Invariant,
            },
            Transform::default(),
        ))
        .id();

    app.update();
    assert!(app.world().get::<ColliderDisabled>(entity).is_some());

    {
        let mut store = app.world_mut().resource_mut::<ObjectStateStore>();
        let state = store.0.get_mut("virtual").unwrap();
        state.lifecycle = object_lifecycle_free();
        state.holder_agent_id.clear();
        state.pose.x = 4.0;
        state.pose.y = 5.0;
    }
    app.update();

    assert!(app.world().get::<ColliderDisabled>(entity).is_none());
    assert_eq!(
        app.world().get::<Transform>(entity).unwrap().translation,
        Vec3::new(4.0, 5.0, 0.3)
    );
}

/// The sim executor emulates Spot's gait lag, not its velocity limit — the
/// onboard controller (`autonomy/guidance.rs`) already clamps
/// every setpoint to the SDK's `SE2VelocityLimit` before it reaches either
/// executor. Given such an already-bounded target, the plant's first-order
/// response must approach it monotonically — never overshoot, oscillate, or
/// diverge (the unbounded-teleport bug this replaced). Pure math on the
/// extracted step, no Bevy world.
#[test]
fn plant_response_approaches_target_monotonically() {
    const DT: f32 = 1.0 / 60.0;
    const EPS: f32 = 1e-4;
    const TARGET_SPEED: f32 = 1.5; // the controller's own velocity bound

    let target = Vec3::new(TARGET_SPEED, 0.0, 0.0);

    let mut velocity = Vec3::ZERO;
    let mut prev_speed = 0.0_f32;
    for _ in 0..500 {
        velocity = plant_velocity_step(velocity, target, DT);
        let speed = velocity.length();
        assert!(
            speed <= TARGET_SPEED + EPS,
            "speed {speed} overshot the target"
        );
        assert!(
            speed >= prev_speed - EPS,
            "speed regressed {speed} < {prev_speed}: response is not a monotonic approach"
        );
        prev_speed = speed;
    }
    assert!(
        (prev_speed - TARGET_SPEED).abs() < 1e-2,
        "response did not converge toward the target: {prev_speed}"
    );
}
