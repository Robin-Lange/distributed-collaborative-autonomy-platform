from __future__ import annotations

import unittest

from fake_sidecar_runtime import (
    CapturePublisher,
    FakeRobot,
    load_production_modules,
    robot_state,
)


class SpotFaultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        loaded = load_production_modules()
        cls.pb = loaded.pb
        cls.sidecar_module = loaded.spot_sidecar

    def test_motor_power_loss_publishes_and_clears_a_hard_stop(self):
        sidecar = self.sidecar_module.SpotSidecar(FakeRobot(), "agent_01")
        publisher = CapturePublisher()

        sidecar._publish_fault(publisher, robot_state(motors_on=False))
        sidecar._publish_fault(publisher, robot_state(motors_on=True))

        events = publisher.decoded(self.pb.ConstraintEvent)
        self.assertEqual(len(events), 2)
        self.assertEqual(events[0].object_id, "motors_off")
        self.assertEqual(events[0].severity, self.pb.SEVERITY_HARD_STOP)
        self.assertEqual(events[1].object_id, "motors_off")
        self.assertEqual(events[1].severity, self.pb.SEVERITY_NONE)


if __name__ == "__main__":
    unittest.main()
