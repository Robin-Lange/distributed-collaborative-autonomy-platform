from __future__ import annotations

import json
import math
import types
import unittest
from unittest.mock import patch

from fake_sidecar_runtime import (
    ArmCartesianFeedback,
    FakeRobot,
    FakeTransform,
    GripperFeedback,
    IDENTITY_4X4,
    feedback_response,
    load_production_modules,
)


class ArmManipulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        loaded = load_production_modules()
        cls.pb = loaded.pb
        cls.sidecar = loaded.spot_sidecar

    def test_grip_target_preserves_published_xyz_above_the_floor_guard(self):
        pose, requested_height = self.sidecar._grip_pose_in_vision(
            (0.6, -0.2, 0.18),
            0.4,
            IDENTITY_4X4,
            FakeTransform(),
        )

        self.assertAlmostEqual(pose.x, 0.6)
        self.assertAlmostEqual(pose.y, -0.2)
        self.assertAlmostEqual(pose.z, 0.18)
        self.assertAlmostEqual(requested_height, 0.18)

    def test_map_registration_preserves_mocap_body_height(self):
        body_position_map = (3.0, 4.0, 0.12)
        body_pose_vision = FakeTransform(x=0.5, y=-0.2, z=0.43)

        t_map_body, _, t_map_vision, t_vision_map = (
            self.sidecar._map_vision_registration(
                body_position_map,
                0.0,
                body_pose_vision,
            )
        )
        body_in_vision = self.sidecar._mat4_apply_point(
            t_vision_map,
            body_position_map,
        )
        body_in_map = self.sidecar._mat4_apply_point(
            t_map_vision,
            (body_pose_vision.x, body_pose_vision.y, body_pose_vision.z),
        )

        self.assertAlmostEqual(t_map_body[11], body_position_map[2])
        for actual, expected in zip(body_in_vision, (0.5, -0.2, 0.43)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(body_in_map, body_position_map):
            self.assertAlmostEqual(actual, expected)

    def test_unsafe_grip_target_is_rejected_in_spots_ground_plane_frame(self):
        with self.assertRaisesRegex(
            self.sidecar.UnsafeGripHeight,
            r"0\.020 m above ground.*at least 0\.050 m",
        ):
            self.sidecar._grip_pose_in_vision(
                (0.6, -0.2, 0.32),
                0.0,
                IDENTITY_4X4,
                FakeTransform(z=0.30),
            )

    def test_non_finite_grip_pose_is_rejected_before_transforming(self):
        for target, yaw_rad in (
            ((math.nan, 0.0, 0.2), 0.0),
            ((0.5, 0.0, 0.2), math.nan),
        ):
            with self.subTest(target=target, yaw_rad=yaw_rad):
                with self.assertRaisesRegex(
                    ValueError,
                    "requested grip pose contains non-finite values",
                ):
                    self.sidecar._grip_pose_in_vision(
                        target,
                        yaw_rad,
                        IDENTITY_4X4,
                        FakeTransform(),
                    )

    def test_arm_command_targets_the_gripper_center_with_height_assist(self):
        command, requested_height = self.sidecar._grip_pose_command(
            (0.7, 0.1, 0.25),
            0.0,
            IDENTITY_4X4,
            FakeTransform(),
        )

        cartesian = command.synchronized_command.arm_command.arm_cartesian_command
        target = cartesian.pose_trajectory_in_task.points[0].pose
        tool = cartesian.wrist_tform_tool
        assist = command.mobility_params.body_control.body_assist_for_manipulation

        self.assertAlmostEqual(target.position.x, 0.7)
        self.assertAlmostEqual(target.position.y, 0.1)
        self.assertAlmostEqual(target.position.z, 0.25)
        self.assertAlmostEqual(tool.position.x, 0.23589)
        self.assertAlmostEqual(tool.position.y, 0.0)
        self.assertAlmostEqual(tool.position.z, 0.0)
        self.assertTrue(assist.enable_hip_height_assist)
        self.assertFalse(assist.enable_body_yaw_assist)
        self.assertAlmostEqual(requested_height, 0.25)

    def test_mocap_to_body_translation_rotates_in_spot_body_axes(self):
        position = self.sidecar._spot_body_position_map(
            (1.0, 2.0, 3.0),
            math.pi / 2.0,
            (0.4, 0.2, -0.1),
        )

        self.assertAlmostEqual(position[0], 0.8)
        self.assertAlmostEqual(position[1], 2.4)
        self.assertAlmostEqual(position[2], 2.9)

    def test_zero_mocap_to_body_translation_preserves_registration(self):
        marker_position = (1.2, -0.4, 0.6)

        self.assertEqual(
            self.sidecar._spot_body_position_map(
                marker_position,
                1.1,
                (0.0, 0.0, 0.0),
            ),
            marker_position,
        )

    def test_auto_calibration_publishes_the_corrected_spot_body_pose(self):
        robot = FakeRobot()
        sidecar = self.sidecar.SpotSidecar(
            robot,
            "agent_01",
            t_map_mocap=object(),
            mocap_to_body_translation_m=(0.4, 0.2, -0.1),
        )
        for index in range(21):
            sidecar._mocap_samples.append(
                (98.0 + index * 0.1, 1.0, 2.0, 3.0, math.pi / 2.0)
            )
        state = types.SimpleNamespace()

        with (
            patch.object(self.sidecar.time, "time", return_value=100.0),
            patch.object(sidecar, "_log_floor_diagnostic"),
            patch.object(sidecar, "_publish_calibration") as publish,
            patch("builtins.print") as output,
        ):
            sidecar._try_auto_calibrate(FakeTransform(), state)

        publish.assert_called_once()
        t_map_body, _ = publish.call_args.args
        self.assertAlmostEqual(t_map_body[3], 0.8)
        self.assertAlmostEqual(t_map_body[7], 2.4)
        self.assertAlmostEqual(t_map_body[11], 2.9)
        self.assertTrue(
            any(
                "Motive origin at map (1.000, 2.000, 3.000); "
                "Spot BODY at map (0.800, 2.400, 2.900)"
                in str(call.args[0])
                for call in output.call_args_list
            )
        )

    def test_stalled_cartesian_motion_is_a_failure(self):
        stalled = feedback_response(
            cartesian=ArmCartesianFeedback.STATUS_TRAJECTORY_STALLED
        )
        complete = feedback_response(
            cartesian=ArmCartesianFeedback.STATUS_TRAJECTORY_COMPLETE
        )

        self.assertEqual(
            self.sidecar._arm_cartesian_status(stalled),
            "FAILED: Cartesian trajectory stalled",
        )
        self.assertEqual(self.sidecar._arm_cartesian_status(complete), "COMPLETE")

    def test_control_request_preserves_the_published_grasp_pose(self):
        sidecar = self.sidecar.SpotSidecar(FakeRobot(), "agent_01")
        accepted = []
        sidecar._accept_manipulation_command = accepted.append
        request = {
            "agent_id": "agent_01",
            "request_id": "pick-1",
            "source": "Operator",
            "approved_by": "onboard_agent",
            "stamp_ms": 123,
            "action": {
                "Pick": {
                    "object_id": "block_mocap_01",
                    "target_pose": {
                        "x": 0.51,
                        "y": -0.24,
                        "z": 0.13,
                        "yaw_rad": 0.72,
                    },
                    "frame_id": "map",
                }
            },
        }

        sidecar.on_control_request(
            types.SimpleNamespace(payload=json.dumps(request).encode())
        )

        self.assertEqual(len(accepted), 1)
        target = accepted[0].target_pose
        self.assertAlmostEqual(target.x, 0.51)
        self.assertAlmostEqual(target.y, -0.24)
        self.assertAlmostEqual(target.z, 0.13)
        self.assertAlmostEqual(target.yaw_rad, 0.72)

    def test_real_grasp_requires_contact_feedback(self):
        robot = FakeRobot()
        robot.command_client.feedback_results.extend(
            [feedback_response() for _ in range(4)]
            + [feedback_response(gripper=GripperFeedback.STATUS_APPLYING_FORCE)]
        )
        sidecar = self.sidecar.SpotSidecar(robot, "agent_01")
        command = self._pick_command("real-object", z=0.20)

        with patch.object(self.sidecar.time, "sleep"):
            sidecar._arm_worker(command, IDENTITY_4X4, is_virtual=False)

        status, _ = sidecar._manipulation_results.get_nowait()
        self.assertEqual(status.phase, self.pb.MANIPULATION_PHASE_SUCCEEDED)
        self.assertEqual(
            status.failure_reason,
            self.pb.MANIPULATION_FAILURE_REASON_UNSPECIFIED,
        )
        self.assertIn("carry", [entry[0].kind for entry in robot.command_client.commands])

    def test_real_fully_closed_gripper_reports_no_object_and_recovers(self):
        robot = FakeRobot()
        sidecar = self.sidecar.SpotSidecar(robot, "agent_01")
        command = self._pick_command("real-object", z=0.20)

        sidecar._arm_worker(command, IDENTITY_4X4, is_virtual=False)

        status, _ = sidecar._manipulation_results.get_nowait()
        self.assertEqual(status.phase, self.pb.MANIPULATION_PHASE_FAILED)
        self.assertEqual(
            status.failure_reason,
            self.pb.MANIPULATION_FAILURE_REASON_NO_OBJECT_DETECTED,
        )
        command_kinds = [entry[0].kind for entry in robot.command_client.commands]
        self.assertEqual(command_kinds[-2:], ["open", "stow"])
        self.assertNotIn("carry", command_kinds)

    def test_virtual_fully_closed_gripper_remains_a_success(self):
        robot = FakeRobot()
        sidecar = self.sidecar.SpotSidecar(robot, "agent_01")
        command = self._pick_command("virtual-object", z=0.20)

        with patch.object(self.sidecar.time, "sleep"):
            sidecar._arm_worker(command, IDENTITY_4X4, is_virtual=True)

        status, _ = sidecar._manipulation_results.get_nowait()
        self.assertEqual(status.phase, self.pb.MANIPULATION_PHASE_SUCCEEDED)
        self.assertIn(
            "carry",
            [entry[0].kind for entry in robot.command_client.commands],
        )

    def test_unsafe_pick_and_place_fail_before_arm_motion(self):
        for verb in (
            self.pb.MANIPULATION_VERB_PICK_UP,
            self.pb.MANIPULATION_VERB_DROP_OFF,
        ):
            with self.subTest(verb=verb):
                robot = FakeRobot()
                sidecar = self.sidecar.SpotSidecar(robot, "agent_01")
                command = self._pick_command("object", z=0.02)
                command.verb = verb

                sidecar._arm_worker(command, IDENTITY_4X4, is_virtual=False)

                status, _ = sidecar._manipulation_results.get_nowait()
                self.assertEqual(status.phase, self.pb.MANIPULATION_PHASE_FAILED)
                self.assertEqual(
                    status.failure_reason,
                    self.pb.MANIPULATION_FAILURE_REASON_UNSAFE_GRIP_HEIGHT,
                )
                self.assertEqual(robot.command_client.commands, [])

    def test_safety_stop_suppresses_recovery_commands(self):
        robot = FakeRobot()
        sidecar = self.sidecar.SpotSidecar(robot, "agent_01")
        sidecar._arm_abort_requested.set()

        sidecar._arm_worker(
            self._pick_command("real-object", z=0.20),
            IDENTITY_4X4,
            is_virtual=False,
        )

        status, _ = sidecar._manipulation_results.get_nowait()
        self.assertEqual(status.phase, self.pb.MANIPULATION_PHASE_FAILED)
        self.assertEqual(
            status.failure_reason,
            self.pb.MANIPULATION_FAILURE_REASON_SAFETY_STOP,
        )
        self.assertEqual(robot.command_client.commands, [])

    def _pick_command(self, object_id, z):
        return self.pb.ManipulationCommand(
            agent_id="agent_01",
            command_id="pick-1",
            object_id=object_id,
            verb=self.pb.MANIPULATION_VERB_PICK_UP,
            approved_by="onboard_agent",
            target_pose=self.pb.PoseXYZYaw(x=0.5, y=0.0, z=z, yaw_rad=0.0),
            stamp_ms=123,
            frame_id="map",
            grasp_profile=self.pb.MANIPULATION_GRASP_PROFILE_TOP_DOWN,
        )


if __name__ == "__main__":
    unittest.main()
