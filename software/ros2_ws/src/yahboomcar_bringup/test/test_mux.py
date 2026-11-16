"""Pure unit tests for VelocityMux's MuxCore. No ROS graph is required."""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from yahboomcar_bringup.velocity_mux import MuxCore, SOURCE_NAMES


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class MuxCoreTest(unittest.TestCase):

    def setUp(self):
        self.clock = FakeClock()
        self.core = MuxCore(clock=self.clock, source_timeout_s=0.3)

    def test_no_sources_yields_zero(self):
        cmd = self.core.compose()
        self.assertEqual((cmd.linear.x, cmd.linear.y, cmd.angular.z), (0.0, 0.0, 0.0))

    def test_priority_teleop_over_navigation(self):
        self.core.on_command('navigation', 0.1, 0.0, 0.0)
        self.core.on_command('teleop', 0.15, 0.0, 0.0)
        cmd = self.core.compose()
        self.assertAlmostEqual(cmd.linear.x, 0.15)

    def test_idle_teleop_does_not_starve_navigation(self):
        self.core.on_command('teleop', 0.0, 0.0, 0.0)
        self.core.on_command('navigation', 0.1, 0.0, 0.0)
        cmd = self.core.compose()
        self.assertAlmostEqual(cmd.linear.x, 0.1)

    def test_freshness_drops_stale(self):
        self.core.on_command('navigation', 0.1, 0.0, 0.0)
        self.clock.advance(0.5)
        cmd = self.core.compose()
        self.assertEqual((cmd.linear.x, cmd.linear.y, cmd.angular.z), (0.0, 0.0, 0.0))

    def test_joy_restricts_teleop(self):
        self.core.on_joy(True)
        self.core.on_command('teleop', 0.15, 0.0, 0.0)
        self.core.on_command('laser', 0.05, 0.0, 0.0)
        cmd = self.core.compose()
        self.assertAlmostEqual(cmd.linear.x, 0.05)

    def test_fault_clears_and_does_not_replay(self):
        self.core.on_command('navigation', 0.1, 0.0, 0.0)
        self.core.on_fault()
        cmd = self.core.compose()
        self.assertEqual((cmd.linear.x, cmd.linear.y, cmd.angular.z), (0.0, 0.0, 0.0))
        # Driver reset lifts the latch, but old commands must NOT come back.
        self.core.on_reset()
        self.clock.advance(0.01)
        cmd = self.core.compose()
        self.assertEqual((cmd.linear.x, cmd.linear.y, cmd.angular.z), (0.0, 0.0, 0.0))

    def test_invalid_axes_dropped(self):
        self.core.on_command('teleop', float('nan'), 0.0, 0.0)
        cmd = self.core.compose()
        self.assertEqual((cmd.linear.x, cmd.linear.y, cmd.angular.z), (0.0, 0.0, 0.0))

    def test_ellipse_bound(self):
        self.core.on_command('navigation', 1.0, 1.0, 5.0)
        cmd = self.core.compose()
        self.assertLessEqual(cmd.linear.x / 0.2 ** 0 + 0.0, 1e9)  # placeholder sanity
        ex = cmd.linear.x / 0.2
        ey = cmd.linear.y / 0.2
        self.assertLessEqual(ex * ex + ey * ey, 1.0 + 1e-9)
        self.assertLessEqual(abs(cmd.angular.z), 0.6 + 1e-9)

    def test_priority_vector_length(self):
        self.assertEqual(len(self.core.priority), len(SOURCE_NAMES))


if __name__ == '__main__':
    unittest.main()
