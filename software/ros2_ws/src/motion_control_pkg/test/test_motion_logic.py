import math
import unittest

from motion_control_pkg.motion_logic import (
    MotionController, TestConfig as MotionConfig, parse_command, wrap_angle, yaw_from_quaternion)


class MotionSafetyTests(unittest.TestCase):
    def make_controller(self, **kwargs):
        ctl = MotionController(MotionConfig(**kwargs))
        ctl.update_odom(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, stamp=1.0)
        return ctl

    def test_invalid_commands_do_not_change_existing_test(self):
        ctl = self.make_controller()
        ctl.start(parse_command('forward_1m', ctl.config), 1.0)
        for text in ('fly_1m', 'forward_nan', 'turn_inf', 'forward_-1',
                     'forward_0', 'turn_0', 'forward_11m', 'turn_361',
                     'turn_90m', 'forward_90deg', 'back_0.01m'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_command(text, ctl.config)
            self.assertEqual(ctl.state, 'RUNNING')
            self.assertEqual(ctl.command.target, 1.0)

    def test_rejects_start_before_valid_odometry(self):
        ctl = MotionController()
        with self.assertRaisesRegex(ValueError, 'waiting'):
            ctl.start(parse_command('forward_1m', ctl.config), 1.0)
        self.assertEqual(ctl.state, 'IDLE')

    def test_rejects_start_with_stale_odometry(self):
        ctl = self.make_controller()
        with self.assertRaisesRegex(ValueError, 'stale'):
            ctl.start(parse_command('forward_1m', ctl.config), 2.0)
        self.assertEqual(ctl.state, 'IDLE')

    def test_no_idle_velocity_stream(self):
        ctl = self.make_controller()
        self.assertIsNone(ctl.tick(1.1))
        ctl.state = 'FINISHED'
        self.assertIsNone(ctl.tick(1.2))
        ctl.state = 'FAULT'
        self.assertIsNone(ctl.tick(1.3))

    def test_odom_timeout_stops_and_latches_fault(self):
        ctl = self.make_controller()
        command = parse_command('forward_1m', ctl.config)
        ctl.start(command, 1.0)
        self.assertGreater(ctl.tick(1.1)[0], 0.0)
        self.assertEqual(ctl.tick(1.6), (0.0, 0.0))
        self.assertEqual(ctl.state, 'FAULT')
        ctl.update_odom(0.1, 0.0, 0.0, 0.0, 0.0, 1.7, stamp=1.7)
        with self.assertRaisesRegex(ValueError, 'fault latched'):
            ctl.start(command, 1.7)
        ctl.stop()
        ctl.update_odom(0.1, 0.0, 0.0, 0.0, 0.0, 1.8, stamp=1.8)
        ctl.start(command, 1.8)
        self.assertEqual(ctl.state, 'RUNNING')

    def test_replayed_header_does_not_refresh_watchdog(self):
        ctl = self.make_controller()
        ctl.start(parse_command('forward_1m', ctl.config), 1.0)
        self.assertFalse(ctl.update_odom(0.1, 0, 0, 0, 0, 1.4, stamp=1.0))
        self.assertEqual(ctl.tick(1.6), (0.0, 0.0))
        self.assertEqual(ctl.state, 'FAULT')

    def test_fresh_but_frozen_pose_faults(self):
        ctl = self.make_controller(progress_timeout=1.0)
        ctl.start(parse_command('forward_1m', ctl.config), 1.0)
        for timestamp in (1.2, 1.4, 1.6, 1.8, 2.1):
            ctl.update_odom(0, 0, 0, 0, 0, timestamp, stamp=timestamp)
            command = ctl.tick(timestamp)
        self.assertEqual(command, (0.0, 0.0))
        self.assertEqual(ctl.state, 'FAULT')
        self.assertIn('progress', ctl.reason)

    def test_lateral_displacement_does_not_complete_forward_test(self):
        ctl = self.make_controller()
        ctl.start(parse_command('forward_1m', ctl.config), 1.0)
        ctl.update_odom(0, 1.5, 0, 0, 0, 1.1, stamp=1.1)
        self.assertGreater(ctl.tick(1.1)[0], 0)
        self.assertAlmostEqual(ctl.progress, 0.0)
        self.assertEqual(ctl.state, 'RUNNING')

    def test_backward_progress_and_terminal_zero(self):
        ctl = self.make_controller()
        ctl.start(parse_command('back_0.2m', ctl.config), 1.0)
        self.assertLess(ctl.tick(1.01)[0], 0.0)
        ctl.update_odom(-0.19, 0, 0, -0.1, 0, 1.2, stamp=1.2)
        self.assertEqual(ctl.tick(1.2), (0.0, 0.0))
        self.assertEqual(ctl.state, 'FINISHED')
        self.assertIsNone(ctl.tick(1.3))

    def test_rotated_start_frame_projects_distance_correctly(self):
        ctl = self.make_controller()
        ctl.update_odom(1, 2, math.pi/2, 0, 0, 1.1, stamp=1.1)
        ctl.start(parse_command('forward_1m', ctl.config), 1.1)
        ctl.update_odom(1, 3, math.pi/2, 0, 0, 1.2, stamp=1.2)
        self.assertEqual(ctl.tick(1.2), (0.0, 0.0))
        self.assertEqual(ctl.state, 'FINISHED')

    def test_clockwise_alias(self):
        ctl = self.make_controller()
        ctl.start(parse_command('turnr_90', ctl.config), 1.0)
        self.assertLess(ctl.tick(1.1)[1], 0.0)

    def test_unwrapped_360_degree_turn_both_directions(self):
        for sign, command in ((1, 'turn_left_360'), (-1, 'turn_right_360')):
            with self.subTest(command=command):
                ctl = self.make_controller()
                ctl.update_odom(0, 0, math.radians(170), 0, 0, 1.1, stamp=1.1)
                ctl.start(parse_command(command, ctl.config), 1.1)
                for index in range(1, 37):
                    timestamp = 1.1 + index*0.1
                    yaw = wrap_angle(math.radians(170 + sign*index*10))
                    ctl.update_odom(0, 0, yaw, 0, sign*0.5, timestamp, stamp=timestamp)
                    velocity = ctl.tick(timestamp)
                    if index < 36:
                        self.assertEqual(ctl.state, 'RUNNING')
                self.assertAlmostEqual(ctl.accumulated_yaw, sign*2*math.pi)
                self.assertEqual(ctl.state, 'FINISHED')
                self.assertEqual(velocity, (0.0, 0.0))

    def test_total_runtime_deadline_despite_progress(self):
        ctl = self.make_controller(max_test_duration=1.0)
        ctl.start(parse_command('forward_10m', ctl.config), 1.0)
        for timestamp in (1.2, 1.4, 1.6, 1.8, 2.0):
            ctl.update_odom((timestamp-1)*0.1, 0, 0, .1, 0, timestamp, stamp=timestamp)
            command = ctl.tick(timestamp)
        self.assertEqual(command, (0.0, 0.0))
        self.assertIn('duration', ctl.reason)

    def test_active_test_cannot_be_replaced(self):
        ctl = self.make_controller()
        ctl.start(parse_command('forward_1m', ctl.config), 1.0)
        with self.assertRaisesRegex(ValueError, 'already running'):
            ctl.start(parse_command('turn_90', ctl.config), 1.1)
        self.assertEqual(ctl.command.action, 'forward')

    def test_odometry_and_quaternion_validation(self):
        ctl = self.make_controller()
        for invalid in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                ctl.update_odom(invalid, 0, 0, 0, 0, 1.2)
        for quaternion in ((0, 0, 0, 0), (0, 0, float('nan'), 1)):
            with self.assertRaises(ValueError):
                yaw_from_quaternion(*quaternion)
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, 2, 2), math.pi/2)

    def test_parameter_validation(self):
        for kwargs in ({'test_speed': 0}, {'turn_speed': float('nan')},
                       {'odom_timeout': -1}, {'minimum_speed': 0.2}, {'kd_pos': -1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                MotionConfig(**kwargs)

    def test_explicit_stop_allows_new_timestamp_baseline(self):
        ctl = self.make_controller()
        ctl.stop()
        with self.assertRaisesRegex(ValueError, 'waiting'):
            ctl.start(parse_command('forward_1m', ctl.config), 1.1)
        self.assertTrue(ctl.update_odom(0, 0, 0, 0, 0, 1.2, stamp=0.1))
        ctl.start(parse_command('forward_1m', ctl.config), 1.2)
        self.assertGreater(ctl.tick(1.21)[0], 0.0)


if __name__ == '__main__':
    unittest.main()
