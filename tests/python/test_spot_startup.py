from __future__ import annotations

import io
import sys
import threading
import types
import unittest
from unittest.mock import patch

from fake_sidecar_runtime import load_production_modules


class RecordingLease:
    def __init__(self, events):
        self.events = events

    def __enter__(self):
        self.events.append("lease entered")
        return self

    def __exit__(self, exception_type, _exception, _traceback):
        label = exception_type.__name__ if exception_type else "none"
        self.events.append(f"lease exited: {label}")
        return False


class FakeRobot:
    def __init__(self, events, power_error=None, powered_on=False):
        self.events = events
        self.power_error = power_error
        self.powered_on = powered_on
        self.time_sync = types.SimpleNamespace(wait_for_sync=lambda: None)
        self.lease_client = object()

    def is_estopped(self):
        return False

    def ensure_client(self, service_name):
        if service_name != "lease":
            raise AssertionError(f"unexpected service {service_name}")
        return self.lease_client

    def power_on(self, timeout_sec):
        self.events.append(f"power on: {timeout_sec}")
        if self.power_error is not None:
            raise self.power_error
        self.powered_on = True

    def is_powered_on(self):
        return self.powered_on

    def power_off(self, cut_immediately, timeout_sec):
        self.events.append(f"power off: {cut_immediately}, {timeout_sec}")
        self.powered_on = False


class FakeSession:
    def __enter__(self):
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        return False

    def liveliness(self):
        return types.SimpleNamespace(declare_token=lambda _topic: object())

    def declare_publisher(self, _topic, **_options):
        return object()

    def declare_subscriber(self, _topic, _callback):
        return object()


class InterruptingSidecar:
    agent_id = "agent_01"
    command_client = object()
    on_control_request = None
    on_calibration = None

    @staticmethod
    def step(*_publishers):
        raise KeyboardInterrupt


class SpotStartupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        loaded = load_production_modules()
        cls.sidecar = loaded.spot_sidecar

    def run_main(self, robot, lease, arguments):
        sdk = types.SimpleNamespace(create_robot=lambda _hostname: robot)
        with (
            patch.object(sys, "argv", ["spot_sidecar.py", *arguments]),
            patch.object(self.sidecar, "_start_console_logging", return_value="test.log"),
            patch.object(self.sidecar, "_load_object_catalog_seeds", return_value=[]),
            patch.object(self.sidecar.bosdyn.client, "create_standard_sdk", return_value=sdk),
            patch.object(self.sidecar, "LeaseKeepAlive", return_value=lease) as constructor,
            patch.object(self.sidecar, "SpotSidecar", return_value=InterruptingSidecar()),
            patch.object(self.sidecar, "_open_zenoh_session", return_value=FakeSession()),
            patch("builtins.print"),
        ):
            self.sidecar.main()
        return constructor

    def test_shutdown_powers_off_before_returning_the_lease(self):
        events = []
        robot = FakeRobot(events, powered_on=True)
        lease = RecordingLease(events)

        constructor = self.run_main(robot, lease, ["192.168.55.21"])

        constructor.assert_called_once_with(
            robot.lease_client,
            must_acquire=True,
            return_at_exit=True,
            on_failure_callback=self.sidecar._report_lease_keepalive_failure,
            warnings=False,
        )
        self.assertEqual(
            events,
            [
                "lease entered",
                "power off: False, 20",
                "lease exited: none",
            ],
        )

    def test_closing_control_input_sets_the_shutdown_event(self):
        shutdown_requested = threading.Event()

        self.sidecar._wait_for_control_input_close(
            io.StringIO("ignored\n"),
            shutdown_requested,
        )

        self.assertTrue(shutdown_requested.is_set())

    def test_power_startup_failure_still_returns_the_lease(self):
        events = []
        power_error = self.sidecar.KeepaliveMotorsOffError()
        robot = FakeRobot(events, power_error=power_error)
        lease = RecordingLease(events)

        with self.assertRaisesRegex(RuntimeError, "Keepalive safety policy"):
            self.run_main(
                robot,
                lease,
                ["192.168.55.21", "--auto-power-on"],
            )

        self.assertEqual(
            events,
            [
                "lease entered",
                "power on: 20",
                "lease exited: RuntimeError",
            ],
        )

    def test_lease_conflict_reports_the_current_owner_without_taking_it(self):
        owner = types.SimpleNamespace(
            client_name="tablet-controller",
            user_name="operator",
        )
        response = types.SimpleNamespace(lease_owner=owner)
        conflict = self.sidecar.ResourceAlreadyClaimedError(response)

        with (
            patch.object(self.sidecar, "LeaseKeepAlive", side_effect=conflict),
            self.assertRaisesRegex(
                RuntimeError,
                r"Lease owner: tablet-controller \(user operator\).*did not take",
            ),
        ):
            self.sidecar._acquire_body_lease(object())

    def test_keepalive_failure_reports_status_and_current_owner(self):
        owner = types.SimpleNamespace(
            client_name="tablet-controller",
            user_name="operator",
        )
        result = types.SimpleNamespace(status=3, owner=owner)
        response = types.SimpleNamespace(lease_use_result=result)
        error = RuntimeError("retain rejected")
        error.response = response

        with patch("builtins.print") as print_mock:
            self.sidecar._report_lease_keepalive_failure(error)

        message = print_mock.call_args.args[0]
        self.assertIn("SDK will retry", message)
        self.assertIn("status=3", message)
        self.assertIn("tablet-controller (user operator)", message)


if __name__ == "__main__":
    unittest.main()
