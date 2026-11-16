#!/usr/bin/env python3
"""Real ROS 2 node + real SDK + pyserial + Linux PTY firmware checks.

Never connects a robot: the subprocess receives only FirmwareEmulator.port.
Run after sourcing ROS Humble and the workspace install. Run sequentially with
the other ROS graph tests, preferably in an isolated ROS_DOMAIN_ID.
"""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

from serial_accessory_emulator import AccessoryFirmware as FirmwareEmulator


def main():
    root = Path(__file__).resolve().parents[1]
    log_dir = Path(os.environ.get('VALIDATION_LOG_DIR', '/tmp/x3plus-validation'))
    log_dir.mkdir(parents=True, exist_ok=True)
    output_path = log_dir / 'ros_hardware_checks.json'
    report = {
        'passed': False,
        'fixture': 'ROS 2 hardware backend / vendor SDK 3.3.9 / pyserial / Linux PTY; no physical robot',
        'results': [],
    }
    results = report['results']
    os.environ.setdefault('ROS_DOMAIN_ID', '92')
    env = os.environ.copy()
    env['PYTHONPATH'] = str(root / 'vendor' / 'py_install') + os.pathsep + env.get('PYTHONPATH', '')
    firmware = FirmwareEmulator().start()
    process = None
    node = None
    log = (log_dir / 'ros_hardware_node.log').open('w')
    rclpy.init()
    state = {'odom': None, 'odom_count': 0, 'imu': None, 'imu_count': 0,
             'velocity': None, 'base': {}}

    def store_odom(msg):
        state['odom'], state['odom_count'] = msg, state['odom_count'] + 1

    def store_imu(msg):
        state['imu'], state['imu_count'] = msg, state['imu_count'] + 1

    try:
        process = subprocess.Popen([
            'ros2', 'run', 'x3plus_base', 'base_node', '--ros-args',
            '-p', 'use_mock:=false', '-p', 'port:=' + firmware.port,
        ], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = Node('x3plus_hardware_pty_probe')
        node.create_subscription(Odometry, '/odom_raw', store_odom, 10)
        node.create_subscription(Imu, '/imu/data_raw', store_imu, qos_profile_sensor_data)
        node.create_subscription(Twist, '/chassis/velocity_raw',
                                 lambda msg: state.update(velocity=msg), 10)
        node.create_subscription(String, '/base/status',
                                 lambda msg: state.update(base=json.loads(msg.data)), 10)
        commands = node.create_publisher(Twist, '/vel_raw', 1)
        next_command = [0.0]

        def publish(velocity):
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = map(float, velocity)
            commands.publish(msg)

        def pump(velocity=None):
            if process.poll() is not None:
                raise AssertionError('hardware ROS node exited; see ros_hardware_node.log')
            if firmware.error is not None:
                raise AssertionError('fake firmware: ' + firmware.error)
            now = time.monotonic()
            if velocity is not None and now >= next_command[0]:
                publish(velocity)
                next_command[0] = now + 0.04
            rclpy.spin_once(node, timeout_sec=0.02)

        def wait_for(predicate, timeout, label, velocity=None):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                pump(velocity)
                if predicate():
                    return
            raise AssertionError(label + ': base=' + repr(state['base']))

        def drive_for(duration, velocity=None):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                pump(velocity)

        def service(name):
            client = node.create_client(Trigger, name)
            try:
                assert client.wait_for_service(timeout_sec=3), name + ' unavailable'
                future = client.call_async(Trigger.Request())
                wait_for(future.done, 3, name)
                response = future.result()
                assert response is not None and response.success, name + ': ' + repr(response)
            finally:
                node.destroy_client(client)

        def feedback_matches(vx, vy, wz):
            msg = state['velocity']
            return msg is not None and all(abs(actual - expected) < 0.002 for actual, expected in (
                (msg.linear.x, vx), (msg.linear.y, vy), (msg.angular.z, wz)))

        def yaw(msg):
            q = msg.pose.pose.orientation
            return math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))

        wait_for(lambda: state['odom'] is not None and state['imu'] is not None and
                 state['base'].get('feedback_fresh') and commands.get_subscription_count() == 1,
                 20, 'hardware backend startup')
        assert state['base']['mode'] == 'hardware'
        assert state['imu'].orientation_covariance[0] == -1.0
        assert abs(state['imu'].linear_acceleration.z - 9.807) < 1e-6
        assert [command[0] for command in firmware.commands[:2]] == [0x12, 0x01]
        assert firmware.motion_commands[0] == (0, 0, 0)
        # Startup also polls measured arm/version feedback. These two exact
        # read-only queries are permitted; servo/flash/torque writes are not.
        assert all(command[0] in (0x12, 0x01) or
                   (command[0] == 0x50 and command[1] in (b'\x23\x01', b'\x51\x00'))
                   for command in firmware.commands)
        results.append({'check': 'actual_hardware_node_sdk_startup_and_sensor_qos', 'passed': True})

        start_x = state['odom'].pose.pose.position.x
        wait_for(lambda: feedback_matches(0.1, 0, 0), 3, 'forward velocity feedback', (0.1, 0, 0))
        wait_for(lambda: state['odom'].pose.pose.position.x - start_x > 0.04,
                 2, 'forward odometry response', (0.1, 0, 0))
        forward_delta = state['odom'].pose.pose.position.x - start_x
        wait_for(lambda: feedback_matches(0, 0, 0), 2, 'forward stop', (0, 0, 0))
        results.append({'check': 'real_node_forward_velocity_and_odometry',
                        'passed': True, 'observed_delta_x_m': forward_delta})

        start_y = state['odom'].pose.pose.position.y
        wait_for(lambda: feedback_matches(0, 0.1, 0), 3, 'sideways velocity feedback', (0, 0.1, 0))
        wait_for(lambda: state['odom'].pose.pose.position.y - start_y > 0.04,
                 2, 'mecanum sideways odometry response', (0, 0.1, 0))
        sideways_delta = state['odom'].pose.pose.position.y - start_y
        wait_for(lambda: feedback_matches(0, 0, 0), 2, 'sideways stop', (0, 0, 0))
        results.append({'check': 'real_node_sideways_velocity_and_odometry',
                        'passed': True, 'observed_delta_y_m': sideways_delta})

        start_yaw = yaw(state['odom'])
        wait_for(lambda: feedback_matches(0, 0, 0.3) and
                 abs(state['imu'].angular_velocity.z - 0.3) < 0.002,
                 3, 'yaw feedback with ICM gyro', (0, 0, 0.3))
        wait_for(lambda: yaw(state['odom']) - start_yaw > 0.08,
                 2, 'yaw odometry', (0, 0, 0.3))
        results.append({'check': 'real_node_yaw_and_imu_response', 'passed': True})
        wait_for(lambda: feedback_matches(0, 0, 0), 2, 'yaw stop', (0, 0, 0))

        # Stop publishing while valid firmware feedback continues.
        wait_for(lambda: feedback_matches(0.1, 0, 0), 3, 'deadman start', (0.1, 0, 0))
        wait_for(lambda: state['base'].get('state') == 'IDLE' and feedback_matches(0, 0, 0),
                 2, 'ROS command timeout with live serial feedback')
        results.append({'check': 'real_node_command_timeout_zero', 'passed': True})

        wait_for(lambda: feedback_matches(0.1, 0, 0), 3, 'feedback-loss start', (0.1, 0, 0))
        firmware.pause_reports(True)
        wait_for(lambda: 'FEEDBACK_TIMEOUT' in state['base'].get('state', ''),
                 2, 'hardware feedback timeout latch', (0.1, 0, 0))
        wait_for(lambda: firmware.motion_commands[-1] == (0, 0, 0),
                 1, 'zero command transmitted after silence', (0.1, 0, 0))
        drive_for(0.1)
        old_count = state['odom_count']
        drive_for(0.3)
        assert state['odom_count'] == old_count, 'stale cached feedback republished as new odometry'
        results.append({'check': 'silent_firmware_latches_fault_and_transmits_zero', 'passed': True})

        restored_at = len(firmware.motion_commands)
        firmware.pause_reports(False)
        wait_for(lambda: state['base'].get('feedback_fresh'), 3, 'reports restored', (0.1, 0, 0))
        drive_for(0.25, (0.1, 0, 0))
        assert 'FEEDBACK_TIMEOUT' in state['base']['state']
        assert all(command == (0, 0, 0) for command in firmware.motion_commands[restored_at:])
        service('/base/reset_stop')
        wait_for(lambda: state['base'].get('state') == 'IDLE', 2, 'explicit fault reset')
        wait_for(lambda: feedback_matches(0, 0.1, 0), 3, 'new command after reset', (0, 0.1, 0))
        wait_for(lambda: feedback_matches(0, 0, 0), 2, 'final zero', (0, 0, 0))
        results.append({'check': 'fresh_reports_require_explicit_reset_then_new_command', 'passed': True})
        report['passed'] = True
    except Exception as exc:
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
        raise
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
        if report['passed']:
            time.sleep(0.05)
            report['shutdown_zero_observed'] = bool(
                firmware.motion_commands and firmware.motion_commands[-1] == (0, 0, 0))
            report['passed'] = report['shutdown_zero_observed']
        report['firmware_error'] = firmware.error
        report['captured_motion_commands'] = len(firmware.motion_commands)
        firmware.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        output_path.write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    assert report['passed'], 'hardware backend graph checks failed'


if __name__ == '__main__':
    main()
