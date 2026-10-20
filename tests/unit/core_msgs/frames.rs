use super::*;

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1.0e-9,
        "expected {expected}, got {actual}"
    );
}

fn assert_vec_close(actual: [f64; 3], expected: [f64; 3]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert_close(actual, expected);
    }
}

fn test_yaw(theta: f64) -> [[f64; 3]; 3] {
    let c = theta.cos();
    let s = theta.sin();
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

#[test]
fn inverse_round_trips_points_and_vectors() {
    let transform = RigidTransform::from_rotation_translation(test_yaw(0.73), [1.2, -3.4, 5.6]);
    let inverse = transform.inverse();
    let point = [4.0, -2.5, 0.25];
    let vector = [-0.5, 8.0, 2.0];

    assert_vec_close(
        inverse.transform_point(transform.transform_point(point)),
        point,
    );
    assert_vec_close(
        inverse.transform_vector(transform.transform_vector(vector)),
        vector,
    );
}

#[test]
fn quaternion_validation_rejects_invalid_values() {
    assert!(quat_is_valid(0.0, 0.0, 0.0, 1.0));
    assert!(quat_is_valid(0.0, 0.0, 0.0, 2.0));
    assert!(!quat_is_valid(0.0, 0.0, 0.0, 0.0));
    assert!(!quat_is_valid(5.0e-7, 0.0, 0.0, 0.0));
    assert!(quat_is_valid(2.0e-6, 0.0, 0.0, 0.0));
    assert!(!quat_is_valid(f64::NAN, 0.0, 0.0, 1.0));
    assert!(!quat_is_valid(f64::INFINITY, 0.0, 0.0, 1.0));
}

#[test]
fn determinant_distinguishes_proper_rotations_from_reflections() {
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let rotation = test_yaw(1.2);
    let reflection = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    assert_close(mat3_determinant(&identity), 1.0);
    assert_close(mat3_determinant(&rotation), 1.0);
    assert_close(mat3_determinant(&reflection), -1.0);
}

#[test]
fn matrix_to_quaternion_preserves_the_rotation() {
    let rotation = test_yaw(-1.1);
    let (qx, qy, qz, qw) = mat3_to_quat(&rotation);
    let quaternion = glam::DQuat::from_xyzw(qx, qy, qz, qw);

    assert_close(quaternion.length(), 1.0);
    let expected = RigidTransform::from_rotation_translation(rotation, [0.0; 3])
        .transform_vector([0.3, -0.4, 0.5]);
    let actual = quaternion * glam::DVec3::new(0.3, -0.4, 0.5);
    assert_vec_close(actual.to_array(), expected);
}

#[test]
fn default_frame_config_maps_native_axes_into_map() {
    let config = FramesConfig::from_config(&crate::config::FramesConfig::default());

    assert_vec_close(
        config.t_map_bevy.transform_point([1.0, 2.0, 3.0]),
        [1.0, -3.0, 2.0],
    );
    assert_vec_close(
        config.t_map_mocap.transform_point([1.0, 2.0, 3.0]),
        [-1.0, 3.0, 2.0],
    );
}
