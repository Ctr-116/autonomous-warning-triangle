import math
import unittest
from x3plus_base.control import CommandGuard, Limits, PlanarOdometry


class ControlTests(unittest.TestCase):
    def test_no_first_sample_epoch_jump(self):
        o = PlanarOdometry()
        o.update(900000.0, 1.0, 0.0, 0.0)
        self.assertEqual(o.x, 0.0)
        o.update(900000.1, 1.0, 0.0, 0.0)
        self.assertAlmostEqual(o.x, 0.1)

    def test_stale_gap_and_duplicate_samples_not_integrated(self):
        o = PlanarOdometry()
        o.update(1.0, 1.0, 0.0, 0.0)
        self.assertFalse(o.update(1.0, 1.0, 0.0, 0.0))
        o.update(10.0, 1.0, 0.0, 0.0)
        self.assertEqual(o.x, 0.0)

    def test_holonomic_turn_uses_body_frame(self):
        o = PlanarOdometry(max_dt=2.0)
        o.update(0.0, 1.0, 0.0, math.pi / 2)
        o.update(1.0, 1.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(o.x, 2 / math.pi)
        self.assertAlmostEqual(o.y, 2 / math.pi)

    def test_ellipse_limit_and_deadman(self):
        g = CommandGuard(Limits())
        g.receive(10.0, 10.0, 5.0, 1.0)
        cmd, state = g.output(1.1, 1.1)
        self.assertEqual(state, 'ACTIVE')
        self.assertLessEqual(math.hypot(cmd[0], cmd[1]), 0.200001)
        self.assertEqual(cmd[2], 0.6)
        self.assertEqual(g.output(1.5, 1.5)[0], (0.0, 0.0, 0.0))

    def test_feedback_failure_is_latched(self):
        g = CommandGuard(Limits())
        g.output(1.0, 1.0)
        g.receive(0.1, 0.0, 0.0, 1.0)
        self.assertEqual(g.output(2.0, 1.0)[0], (0.0, 0.0, 0.0))
        self.assertFalse(g.receive(0.1, 0.0, 0.0, 2.0))
        self.assertFalse(g.reset(2.0, 1.0))
        self.assertTrue(g.reset(2.0, 2.0))
        self.assertEqual(g.output(2.0, 2.0)[0], (0.0, 0.0, 0.0))

    def test_nonfinite_command_and_transport_error(self):
        g = CommandGuard(Limits())
        self.assertFalse(g.receive(float('nan'), 0.0, 0.0, 0.0))
        self.assertIn('INVALID', g.fault)
        self.assertEqual(g.output(1.0, 1.0, 'serial lost')[0], (0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
