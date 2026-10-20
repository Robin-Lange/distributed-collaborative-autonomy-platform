use super::*;
use core_msgs::proto::Vec3;

fn close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 1.0e-6,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn conversion_preserves_map_translation_and_splits_timestamp() {
    let state = AgentStateMsg {
        position: Some(Vec3 {
            x: 1.25,
            y: -2.5,
            z: 0.75,
        }),
        stamp_ms: 3_004,
        heading_yaw_rad: 0.0,
        ..Default::default()
    };

    let transform = from_agent_state(&state);

    assert_eq!(transform.timestamp.sec, 3);
    assert_eq!(transform.timestamp.nsec, 4_000_000);
    assert_eq!(transform.parent_frame_id, "map");
    assert_eq!(transform.child_frame_id, "agent");
    close(transform.translation.x, 1.25);
    close(transform.translation.y, -2.5);
    close(transform.translation.z, 0.75);
}

#[test]
fn conversion_encodes_the_reported_heading_as_a_unit_yaw_quaternion() {
    for heading in [0.0, std::f32::consts::FRAC_PI_2, -std::f32::consts::PI] {
        let state = AgentStateMsg {
            heading_yaw_rad: heading,
            ..Default::default()
        };
        let transform = from_agent_state(&state);
        let expected = Quat::from_rotation_z(heading).to_array();

        close(transform.rotation.x, expected[0]);
        close(transform.rotation.y, expected[1]);
        close(transform.rotation.z, expected[2]);
        close(transform.rotation.w, expected[3]);
        close(
            transform.rotation.w.mul_add(
                transform.rotation.w,
                transform.rotation.z.mul_add(
                    transform.rotation.z,
                    transform.rotation.y.mul_add(
                        transform.rotation.y,
                        transform.rotation.x * transform.rotation.x,
                    ),
                ),
            ),
            1.0,
        );
    }
}

#[test]
fn missing_position_has_an_explicit_zero_translation() {
    let transform = from_agent_state(&AgentStateMsg {
        position: None,
        ..Default::default()
    });

    close(transform.translation.x, 0.0);
    close(transform.translation.y, 0.0);
    close(transform.translation.z, 0.0);
}

#[test]
fn serialized_transform_matches_the_registered_schema_shape() {
    let Ok(value) = serde_json::to_value(from_agent_state(&AgentStateMsg::default())) else {
        panic!("frame transform should serialize as JSON");
    };

    assert_eq!(value["parent_frame_id"], "map");
    assert_eq!(value["child_frame_id"], "agent");
    assert!(value["timestamp"]["sec"].is_u64());
    assert!(value["translation"]["x"].is_number());
    assert!(value["rotation"]["w"].is_number());

    let Ok(schema) = serde_json::from_str::<serde_json::Value>(FRAME_TRANSFORM_JSON_SCHEMA) else {
        panic!("the advertised JSON schema must itself be valid JSON");
    };
    let Some(required) = schema["required"].as_array() else {
        panic!("schema should declare required top-level fields");
    };
    for field in [
        "timestamp",
        "parent_frame_id",
        "child_frame_id",
        "translation",
        "rotation",
    ] {
        assert!(required.iter().any(|entry| entry == field));
        assert!(value.get(field).is_some());
    }
}
