use super::*;
use std::collections::BTreeSet;

fn default_catalog() -> ObjectCatalog {
    ObjectCatalog::from_config(crate::config::ObjectCatalogConfig::default())
}

fn assert_close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 1.0e-6,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn loads_default_catalog_and_reach_envelope() {
    let catalog = default_catalog();

    let reach = catalog.reach_envelope();
    assert_close(reach.min_m(), 0.30);
    assert_close(reach.max_m(), 0.80);

    let ids = catalog
        .iter()
        .map(ObjectSpec::object_id)
        .collect::<BTreeSet<_>>();
    assert_eq!(ids, BTreeSet::from(["block_mocap_01", "cylinder_pink_01"]));
    assert!(catalog.get("missing").is_none());
}

#[test]
fn virtual_and_mocap_entries_expose_their_distinct_pose_contracts() {
    let catalog = default_catalog();

    let virtual_object = catalog.get("cylinder_pink_01").unwrap();
    assert_eq!(virtual_object.shape(), "cylinder");
    assert_eq!(virtual_object.color(), "pink");
    assert_eq!(virtual_object.mocap_rigid_body_id(), None);
    assert_close(virtual_object.length_m(), 0.5);
    assert_close(virtual_object.radius_m(), 0.15);
    let pose = virtual_object.virtual_initial_pose_map().unwrap();
    assert_close(pose.x, 1.0);
    assert_close(pose.y, -1.0);
    assert_close(pose.z, 0.15);
    assert_close(virtual_object.resting_height_m(), 0.15);

    let tracked_object = catalog.get("block_mocap_01").unwrap();
    assert_eq!(tracked_object.shape(), "box");
    assert_eq!(tracked_object.color(), "orange");
    assert_eq!(tracked_object.mocap_rigid_body_id(), Some(2));
    assert_eq!(tracked_object.virtual_initial_pose_map(), None);
    assert_close(tracked_object.width_m(), 0.09);
    assert_close(tracked_object.height_m(), 0.09);
    assert_close(tracked_object.resting_height_m(), 0.045);
}

#[test]
fn resting_height_prefers_an_explicit_pose_over_shape_dimensions() {
    let spec = ObjectSpec {
        object_id: "fixture".to_owned(),
        shape: "box".to_owned(),
        length_m: 0.4,
        radius_m: 0.0,
        color: "blue".to_owned(),
        width_m: 0.2,
        height_m: 0.2,
        virtual_initial_pose_map: Some(PoseXYZYaw {
            z: 0.123,
            ..Default::default()
        }),
        mocap_rigid_body_id: None,
    };

    assert_close(spec.resting_height_m(), 0.123);
}
