use super::*;
use crate::{
    catalog::{ObjectCatalog, ObjectSpec},
    object_lifecycle_free, veto_reason_object_not_free, veto_reason_object_not_held,
    veto_reason_task_active, veto_reason_trajectory_active, veto_reason_unknown_object,
    veto_reason_unreachable,
};
use std::f32::consts::{FRAC_PI_2, PI};

fn catalog() -> ObjectCatalog {
    ObjectCatalog::from_config(crate::config::ObjectCatalogConfig::default())
}

fn reach() -> ReachEnvelope {
    ReachEnvelope {
        min_m: 0.30,
        max_m: 0.80,
        front_half_angle_deg: 90.0,
    }
}

fn context(object: Option<&ObjectSpec>) -> ManipulationContext<'_> {
    ManipulationContext {
        agent_id: "spot-1",
        agent_position_map: [0.5, -2.0, 0.5],
        agent_heading_yaw_rad: 0.0,
        verb: manipulation_verb_pick_up(),
        object,
        object_lifecycle: Some(object_lifecycle_free()),
        object_holder_agent_id: None,
        object_pose: None,
        trajectory_active: false,
        task_active: false,
    }
}

fn assert_close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 1.0e-5,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn reach_envelope_includes_distance_and_angle_boundaries_and_ignores_height() {
    let reach = reach();
    let agent = Vec3::new(0.0, 0.0, 20.0);

    assert!(reach.contains(agent, 0.0, Vec3::new(0.30, 0.0, -20.0)));
    assert!(reach.contains(agent, 0.0, Vec3::new(0.80, 0.0, 100.0)));
    assert!(!reach.contains(agent, 0.0, Vec3::new(0.299, 0.0, 20.0)));
    assert!(!reach.contains(agent, 0.0, Vec3::new(0.801, 0.0, 20.0)));

    assert!(reach.contains(agent, 0.0, Vec3::new(0.0, 0.50, 20.0)));
    assert!(reach.contains(agent, 0.0, Vec3::new(0.0, -0.50, 20.0)));
    assert!(!reach.contains(agent, 0.0, Vec3::new(-0.50, 0.0, 20.0)));
}

#[test]
fn reach_envelope_wraps_heading_and_handles_a_zero_distance_target() {
    let narrow = ReachEnvelope {
        min_m: 0.0,
        max_m: 1.0,
        front_half_angle_deg: 15.0,
    };
    let agent = Vec3::ZERO;

    assert!(narrow.contains(agent, 0.0, agent));

    let bearing = -PI + 0.10;
    let target = Vec3::new(0.5 * bearing.cos(), 0.5 * bearing.sin(), 4.0);
    assert!(narrow.contains(agent, PI - 0.10, target));
}

#[test]
fn reach_envelope_rejects_non_finite_geometry() {
    let reach = reach();
    let agent = Vec3::ZERO;
    let target = Vec3::new(0.5, 0.0, 0.1);

    assert!(!reach.contains(Vec3::splat(f32::NAN), 0.0, target));
    assert!(!reach.contains(agent, f32::NAN, target));
    assert!(!reach.contains(agent, 0.0, Vec3::new(f32::NAN, 0.0, 0.1)));
    assert!(!reach.contains(agent, 0.0, Vec3::new(0.5, 0.0, f32::NAN)));

    let invalid_reach = ReachEnvelope {
        front_half_angle_deg: f32::NAN,
        ..reach
    };
    assert!(!invalid_reach.contains(agent, 0.0, target));
}

#[test]
fn precondition_failures_follow_the_documented_precedence() {
    let mut ctx = context(None);
    ctx.trajectory_active = true;
    ctx.task_active = true;

    assert_eq!(
        evaluate_preconditions(&ctx, &reach()).unwrap_err().reason,
        veto_reason_trajectory_active()
    );

    ctx.trajectory_active = false;
    assert_eq!(
        evaluate_preconditions(&ctx, &reach()).unwrap_err().reason,
        veto_reason_task_active()
    );

    ctx.task_active = false;
    assert_eq!(
        evaluate_preconditions(&ctx, &reach()).unwrap_err().reason,
        veto_reason_unknown_object()
    );
}

#[test]
fn pickup_rejects_held_objects_and_uses_the_latest_observed_pose() {
    let catalog = catalog();
    let object = catalog.get("cylinder_pink_01").unwrap();
    let mut ctx = context(Some(object));
    ctx.object_lifecycle = Some(object_lifecycle_held());

    let rejection = evaluate_preconditions(&ctx, &reach()).unwrap_err();
    assert_eq!(rejection.reason, veto_reason_object_not_free());
    assert!(rejection.detail.contains("already held"));

    let observed = PoseXYZYaw {
        x: 4.5,
        y: 5.0,
        z: 7.0,
        yaw_rad: -0.75,
    };
    ctx.object_lifecycle = Some(object_lifecycle_free());
    ctx.agent_position_map = [4.0, 5.0, -100.0];
    ctx.object_pose = Some(observed);

    assert_eq!(evaluate_preconditions(&ctx, &reach()).unwrap(), observed);
}

#[test]
fn pickup_falls_back_to_virtual_pose_but_requires_a_pose_for_tracked_objects() {
    let catalog = catalog();
    let virtual_object = catalog.get("cylinder_pink_01").unwrap();
    let mut ctx = context(Some(virtual_object));
    // cylinder_pink_01's default virtual pose is [1.0, -1.0, 0.15]; place the
    // agent within the reach envelope of it (the shared `context()` default
    // of [0.5, -2.0, 0.5] is ~1.12 m away, outside the 0.30-0.80 m envelope).
    ctx.agent_position_map = [0.5, -1.0, 0.5];
    assert_eq!(
        evaluate_preconditions(&ctx, &reach()).unwrap(),
        virtual_object.virtual_initial_pose_map().unwrap()
    );

    let tracked_object = catalog.get("block_mocap_01").unwrap();
    let mut ctx = context(Some(tracked_object));
    ctx.agent_position_map = [0.0, 0.0, 0.0];
    let rejection = evaluate_preconditions(&ctx, &reach()).unwrap_err();
    assert_eq!(rejection.reason, veto_reason_unreachable());
    assert!(rejection.detail.contains("no known pose yet"));
}

#[test]
fn unreachable_pickup_reports_the_specific_geometric_failure() {
    let catalog = catalog();
    let object = catalog.get("cylinder_pink_01").unwrap();
    let mut ctx = context(Some(object));
    ctx.agent_position_map = [0.0, 0.0, 0.0];

    ctx.object_pose = Some(PoseXYZYaw {
        x: 0.1,
        ..Default::default()
    });
    let too_close = evaluate_preconditions(&ctx, &reach()).unwrap_err();
    assert_eq!(too_close.reason, veto_reason_unreachable());
    assert_eq!(
        too_close.detail,
        "cylinder_pink_01 is too close for manipulation: 0.10 m from the platform; \
         minimum reach is 0.30 m."
    );

    ctx.object_pose = Some(PoseXYZYaw {
        x: 2.0,
        ..Default::default()
    });
    let too_far = evaluate_preconditions(&ctx, &reach()).unwrap_err();
    assert_eq!(too_far.reason, veto_reason_unreachable());
    assert_eq!(
        too_far.detail,
        "cylinder_pink_01 is too far for manipulation: 2.00 m from the platform; \
         maximum reach is 0.80 m."
    );

    ctx.object_pose = Some(PoseXYZYaw {
        x: -0.5,
        ..Default::default()
    });
    let outside_sector = evaluate_preconditions(&ctx, &reach()).unwrap_err();
    assert_eq!(outside_sector.reason, veto_reason_unreachable());
    assert_eq!(
        outside_sector.detail,
        "cylinder_pink_01 is outside the arm's forward sector: 180.0° from the \
         platform's heading; allowed offset is ±90.0°."
    );

    ctx.object_pose = Some(PoseXYZYaw {
        x: f32::NAN,
        ..Default::default()
    });
    let invalid = evaluate_preconditions(&ctx, &reach()).unwrap_err();
    assert_eq!(invalid.reason, veto_reason_unreachable());
    assert_eq!(
        invalid.detail,
        "cylinder_pink_01 cannot be evaluated for manipulation because the platform pose, target \
         pose, or reach configuration is not finite."
    );

    ctx.object_pose = Some(PoseXYZYaw {
        x: 0.5,
        yaw_rad: f32::NAN,
        ..Default::default()
    });
    assert_eq!(
        evaluate_preconditions(&ctx, &reach()).unwrap_err().detail,
        invalid.detail
    );
}

#[test]
fn drop_requires_same_agent_ownership_and_places_at_mid_reach_on_the_floor() {
    let catalog = catalog();
    let object = catalog.get("cylinder_pink_01").unwrap();
    let mut ctx = context(Some(object));
    ctx.verb = manipulation_verb_drop_off();
    ctx.object_lifecycle = Some(object_lifecycle_held());
    ctx.object_holder_agent_id = Some("spot-2");

    assert_eq!(
        evaluate_preconditions(&ctx, &reach()).unwrap_err().reason,
        veto_reason_object_not_held()
    );

    ctx.object_holder_agent_id = Some("spot-1");
    ctx.agent_position_map = [1.0, 2.0, 0.6];
    ctx.agent_heading_yaw_rad = FRAC_PI_2;
    let target = evaluate_preconditions(&ctx, &reach()).unwrap();

    assert_close(target.x, 1.0);
    assert_close(target.y, 2.55);
    assert_close(target.z, object.resting_height_m());
    assert_close(target.yaw_rad, FRAC_PI_2);
}
