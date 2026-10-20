from __future__ import annotations

import json
import os
import time
import types
import unittest
from unittest.mock import patch

from fake_sidecar_runtime import (
    FakeCommandClient,
    FakeFuture,
    FakeRobot,
    load_production_modules,
)


class VelocityDispatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        loaded = load_production_modules()
        cls.pb = loaded.pb
        cls.sidecar_module = loaded.spot_sidecar

    def make_sidecar(self):
        client = FakeCommandClient()
        robot = FakeRobot(command_client=client)
        sidecar = self.sidecar_module.SpotSidecar(robot, "agent_01")
        return sidecar, client

    def velocity_command(self, forward=1.0, left=0.5, yaw_rate=0.2, **overrides):
        values = dict(
            agent_id="agent_01",
            command_id="cmd-1",
            command_type=self.pb.COMMAND_TYPE_VELOCITY,
            approved_by="onboard_agent",
            frame_id="map",
            stamp_ms=0,
            velocity=self.pb.SE2VelocitySetpoint(
                forward_m_s=forward, left_m_s=left, yaw_rate_rad_s=yaw_rate
            ),
        )
        values.update(overrides)
        return self.pb.PlatformCommand(**values)

    def platform_command(self, command_type, stamp_ms):
        return self.pb.PlatformCommand(command_type=command_type, stamp_ms=stamp_ms)

    def test_dispatch_velocity_uses_flat_body_frame_and_setpoint_values(self):
        sidecar, client = self.make_sidecar()
        cmd = self.velocity_command(forward=1.0, left=0.5, yaw_rate=0.2)

        sidecar._handle_command(cmd, None)

        self.assertEqual(len(client.command_attempts), 1)
        command, _kwargs = client.command_attempts[0]
        velocity_request = command.synchronized_command.mobility_command.se2_velocity_request
        self.assertEqual(velocity_request.se2_frame_name, "flat_body")
        self.assertEqual(velocity_request.velocity.linear.x, 1.0)
        self.assertEqual(velocity_request.velocity.linear.y, 0.5)
        self.assertEqual(velocity_request.velocity.angular, 0.2)

    def test_dispatch_velocity_applies_robot_side_velocity_limit(self):
        sidecar, client = self.make_sidecar()
        cmd = self.velocity_command()

        sidecar._dispatch_velocity(cmd)

        command, _kwargs = client.command_attempts[0]
        vel_limit = command.mobility_params.vel_limit
        self.assertEqual(vel_limit.max_vel.linear.x, 1.5)
        self.assertEqual(vel_limit.max_vel.linear.y, 1.5)
        self.assertEqual(vel_limit.max_vel.angular, 1.5)
        self.assertEqual(vel_limit.min_vel.linear.x, -1.5)
        self.assertEqual(vel_limit.min_vel.linear.y, -1.5)
        self.assertEqual(vel_limit.min_vel.angular, -1.5)

        # Regression: params must come from RobotCommandBuilder.mobility_params(), so
        # locomotion_hint is HINT_AUTO — a bare MobilityParams leaves it 0 (HINT_UNKNOWN)
        # and Spot never produces a gait for the velocity command.
        self.assertEqual(
            command.mobility_params.locomotion_hint,
            self.sidecar_module.spot_command_pb2.HINT_AUTO,
        )
        self.assertNotEqual(command.mobility_params.locomotion_hint, 0)

    def test_dispatch_velocity_refreshes_end_time_on_each_send(self):
        sidecar, client = self.make_sidecar()
        cmd = self.velocity_command()

        t0 = time.time()
        sidecar._dispatch_velocity(cmd)
        self.assertEqual(len(client.command_attempts), 1)
        first_end = client.command_attempts[0][1]["end_time_secs"]
        self.assertAlmostEqual(
            first_end, t0 + self.sidecar_module.VELOCITY_CMD_EXPIRATION_S, delta=0.2
        )

        t1 = time.time()
        sidecar._dispatch_velocity(cmd)
        self.assertEqual(len(client.command_attempts), 2)
        second_end = client.command_attempts[1][1]["end_time_secs"]
        self.assertAlmostEqual(
            second_end, t1 + self.sidecar_module.VELOCITY_CMD_EXPIRATION_S, delta=0.2
        )
        self.assertGreater(second_end, first_end)

    def test_may_replace_pending_hold_precedence(self):
        may_replace = self.sidecar_module._may_replace_pending
        pb = self.pb
        hold = self.platform_command(pb.COMMAND_TYPE_HOLD, stamp_ms=100)

        older_or_equal = self.platform_command(pb.COMMAND_TYPE_VELOCITY, stamp_ms=100)
        newer = self.platform_command(pb.COMMAND_TYPE_VELOCITY, stamp_ms=101)
        self.assertFalse(may_replace(hold, older_or_equal))
        self.assertTrue(may_replace(hold, newer))

        stop_over_hold = self.platform_command(pb.COMMAND_TYPE_STOP, stamp_ms=0)
        hold_over_hold = self.platform_command(pb.COMMAND_TYPE_HOLD, stamp_ms=0)
        self.assertTrue(may_replace(hold, stop_over_hold))
        self.assertTrue(may_replace(hold, hold_over_hold))

    def test_zenoh_multicast_defaults_to_auto_and_accepts_disabled_override(self):
        with patch.dict(os.environ, {}, clear=True):
            automatic = self.sidecar_module._zenoh_config()
        self.assertEqual(
            automatic.values["scouting/multicast/interface"], '"auto"'
        )

        disabled = self.sidecar_module._zenoh_config("disabled")
        self.assertEqual(
            disabled.values["scouting/multicast/enabled"], "false"
        )
        self.assertNotIn("scouting/multicast/interface", disabled.values)

    def test_zenoh_open_retries_once_without_multicast(self):
        attempts = []
        opened = object()

        def open_session(config):
            attempts.append(config.values)
            if len(attempts) == 1:
                raise RuntimeError("multicast unavailable")
            return opened

        with patch.dict(os.environ, {}, clear=True):
            original_open = self.sidecar_module.zenoh.open
            self.sidecar_module.zenoh.open = open_session
            try:
                result = self.sidecar_module._open_zenoh_session()
            finally:
                self.sidecar_module.zenoh.open = original_open

        self.assertIs(result, opened)
        self.assertEqual(len(attempts), 2)
        self.assertEqual(
            attempts[1]["scouting/multicast/enabled"], "false"
        )

    def test_velocity_dropped_while_arm_busy(self):
        sidecar, client = self.make_sidecar()
        sidecar._arm_thread = types.SimpleNamespace(is_alive=lambda: True)
        cmd = self.velocity_command()

        sidecar._handle_command(cmd, None)

        self.assertEqual(len(client.command_attempts), 0)

    def test_trajectory_command_produces_no_rpc(self):
        sidecar, client = self.make_sidecar()
        cmd = self.velocity_command(command_type=self.pb.COMMAND_TYPE_TRAJECTORY)

        sidecar._handle_command(cmd, None)

        self.assertEqual(len(client.command_attempts), 0)

    def test_dispatch_failure_does_not_raise_and_frees_the_gate(self):
        sidecar, client = self.make_sidecar()
        cmd = self.velocity_command()

        client.dispatch_results.append(RuntimeError("dispatch unavailable"))
        sidecar._dispatch_velocity(cmd)
        self.assertEqual(len(client.command_attempts), 1)
        self.assertFalse(sidecar._actuation_inflight)

        # The gate wasn't left stuck: a later dispatch still goes out.
        sidecar._dispatch_velocity(cmd)
        self.assertEqual(len(client.command_attempts), 2)

    def test_not_powered_failure_is_latched_until_a_new_command(self):
        class NotPoweredOnError(RuntimeError):
            pass

        sidecar, client = self.make_sidecar()
        first = self.velocity_command(command_id="first")
        client.dispatch_results.append(NotPoweredOnError("motors are off"))

        sidecar._dispatch_velocity(first)
        for _ in range(10):
            sidecar._dispatch_velocity(first)

        self.assertEqual(len(client.command_attempts), 1)
        self.assertEqual(sidecar._unavailable_command_id, "first")
        self.assertIsNone(sidecar._actuation_pending)
        self.assertFalse(sidecar._motion_active)

        second = self.velocity_command(command_id="second", stamp_ms=1)
        sidecar._dispatch_velocity(second)

        self.assertEqual(len(client.command_attempts), 2)
        self.assertIsNone(sidecar._unavailable_command_id)

    def test_at_most_one_command_client_rpc_in_flight(self):
        # Regression: velocity dispatch used to fire a new async RPC for
        # every incoming sample regardless of whether the previous one had
        # resolved, so overlapping in-flight requests could reach bosdyn out
        # of order and get rejected with LeaseUseError.
        sidecar, client = self.make_sidecar()
        inflight = FakeFuture(done=False)
        client.robot_command_async = lambda command, **kwargs: (
            client.command_attempts.append((command, kwargs)),
            inflight,
        )[1]

        sidecar._dispatch_velocity(self.velocity_command(command_id="a"))
        self.assertEqual(len(client.command_attempts), 1)

        # A second tick toward the same goal must not open a second RPC
        # while the first is still outstanding.
        sidecar._dispatch_velocity(self.velocity_command(command_id="a"))
        self.assertEqual(len(client.command_attempts), 1)

        inflight.is_done = True
        for callback in list(inflight.callbacks):
            callback(inflight)

        # Once the in-flight RPC resolves, the queued (latest) setpoint goes
        # out on its own, self-triggered by the completion — no timer.
        self.assertEqual(len(client.command_attempts), 2)

    def test_stop_preempts_pending_velocity_and_is_never_skipped(self):
        # Regression: velocity and stop RPCs used to race independently on
        # the same lease. If a stop landed at bosdyn "behind" a later
        # velocity, it was rejected as stale — silently dropping a safety
        # stop while the robot kept moving.
        sidecar, client = self.make_sidecar()
        inflight = FakeFuture(done=False)
        client.robot_command_async = lambda command, **kwargs: (
            client.command_attempts.append((command, kwargs)),
            inflight,
        )[1]

        sidecar._dispatch_velocity(self.velocity_command(command_id="a"))
        self.assertEqual(len(client.command_attempts), 1)

        # Queued behind the in-flight velocity RPC while it's still moving.
        sidecar._reflex_stop("")
        self.assertEqual(len(client.command_attempts), 1)

        # More velocity ticks for the same goal arrive before the in-flight
        # RPC resolves; none of them may bump the queued stop out.
        sidecar._dispatch_velocity(self.velocity_command(command_id="a"))
        self.assertEqual(len(client.command_attempts), 1)
        self.assertEqual(sidecar._actuation_pending, ("stop", None))

        inflight.is_done = True
        for callback in list(inflight.callbacks):
            callback(inflight)

        self.assertEqual(len(client.command_attempts), 2)
        stop_command, _kwargs = client.command_attempts[1]
        self.assertEqual(stop_command.kind, "stop")

    def test_reflex_stop_is_quiet_while_idle(self):
        # Duplicate ordinary HOLDs can still arrive from retries or another
        # compatible producer. They must not become repeated stop RPCs while
        # the platform is already idle.
        sidecar, client = self.make_sidecar()

        for _ in range(5):
            sidecar._reflex_stop("")

        self.assertEqual(len(client.command_attempts), 0)

    def test_reflex_stop_goes_quiet_again_once_a_stop_confirms(self):
        sidecar, client = self.make_sidecar()

        sidecar._dispatch_velocity(self.velocity_command())
        self.assertEqual(len(client.command_attempts), 1)

        sidecar._reflex_stop("")  # velocity's FakeFuture resolves synchronously
        self.assertEqual(len(client.command_attempts), 2)

        # The stop confirmed (synchronously, per FakeFuture's default), so
        # further HOLD ticks are quiet again.
        sidecar._reflex_stop("")
        sidecar._reflex_stop("")
        self.assertEqual(len(client.command_attempts), 2)

    def test_non_safety_hold_clears_previous_arm_abort(self):
        sidecar, _client = self.make_sidecar()

        sidecar._reflex_stop("collision-1")
        self.assertTrue(sidecar._arm_abort_requested.is_set())

        sidecar._reflex_stop("")
        self.assertFalse(sidecar._arm_abort_requested.is_set())

    def _calibrate(self, sidecar):
        identity = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]
        sidecar._t_map_vision = identity
        sidecar._t_vision_map = identity

    def _sample(self, cmd):
        return types.SimpleNamespace(payload=cmd.SerializeToString())

    def test_on_command_dispatches_velocity_without_step(self):
        sidecar, client = self.make_sidecar()
        self._calibrate(sidecar)

        sidecar.on_command(self._sample(self.velocity_command()))

        self.assertEqual(len(client.command_attempts), 1)
        self.assertIsNone(sidecar._pending)

    def test_generic_control_request_dispatches_velocity(self):
        sidecar, client = self.make_sidecar()
        self._calibrate(sidecar)
        accepted = []
        accept = sidecar._accept_platform_command

        def capture_and_accept(command):
            accepted.append(command)
            accept(command)

        sidecar._accept_platform_command = capture_and_accept
        request = {
            "request_id": "request-1",
            "agent_id": "agent_01",
            "source": "Operator",
            "approved_by": "onboard_agent",
            "action": {
                "PlanarVelocity": {
                    "target": {"Planar": [4.0, 5.0]},
                    "forward_m_s": 1.0,
                    "left_m_s": 0.5,
                    "yaw_rate_rad_s": 0.2,
                    "frame_id": "map",
                }
            },
            "safety_episode_id": None,
            "stamp_ms": 42,
        }

        sidecar.on_control_request(
            types.SimpleNamespace(payload=json.dumps(request).encode())
        )

        self.assertEqual(len(client.command_attempts), 1)
        command, _kwargs = client.command_attempts[0]
        velocity = command.synchronized_command.mobility_command.se2_velocity_request.velocity
        self.assertEqual(velocity.linear.x, 1.0)
        self.assertEqual(velocity.linear.y, 0.5)
        self.assertEqual(velocity.angular, 0.2)
        self.assertEqual(len(accepted), 1)
        self.assertEqual(accepted[0].target_position.x, 4.0)
        self.assertEqual(accepted[0].target_position.y, 5.0)

    def test_on_command_velocity_does_not_supersede_equal_stamp_hold(self):
        sidecar, client = self.make_sidecar()
        self._calibrate(sidecar)
        sidecar._pending = self.platform_command(self.pb.COMMAND_TYPE_HOLD, stamp_ms=100)

        sidecar.on_command(self._sample(self.velocity_command(stamp_ms=100)))
        self.assertEqual(len(client.command_attempts), 0)
        self.assertEqual(sidecar._pending.command_type, self.pb.COMMAND_TYPE_HOLD)

        sidecar.on_command(self._sample(self.velocity_command(stamp_ms=101)))
        self.assertEqual(len(client.command_attempts), 1)
        self.assertIsNone(sidecar._pending)

    def test_on_command_velocity_held_before_calibration(self):
        sidecar, client = self.make_sidecar()
        # _t_map_vision left None: pre-calibration, the sidecar dispatches nothing.

        sidecar.on_command(self._sample(self.velocity_command()))

        self.assertEqual(len(client.command_attempts), 0)
        self.assertIsNone(sidecar._pending)

    def test_vision_tform_body_pipelines_state_rpc(self):
        sidecar, _client = self.make_sidecar()
        state_client = sidecar.state_client
        sidecar._vision_tform_body()
        self.assertEqual(len(state_client.async_futures), 1)
        self.assertIs(sidecar._state_future, state_client.async_futures[0])
        sidecar._vision_tform_body()
        self.assertEqual(len(state_client.async_futures), 2)
        self.assertIs(sidecar._state_future, state_client.async_futures[1])


if __name__ == "__main__":
    unittest.main()
