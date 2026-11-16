#!/usr/bin/env python3
"""Real ROS 2 graph tests using the mock backend; never opens a robot port."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


def main():
    log_dir = Path(os.environ.get('VALIDATION_LOG_DIR', '/tmp/x3plus-validation'))
    log_dir.mkdir(parents=True, exist_ok=True)
    log = (log_dir / 'ros_launch.log').open('w')
    launch = subprocess.Popen([
        'ros2', 'launch', 'x3plus_base', 'bringup.launch.py',
        'use_mock:=true', 'with_motion_test:=true'], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    rclpy.init()
    node = Node('x3plus_integration_probe')
    state = {'odom': None, 'motion': '', 'base': {}, 'velocity': None}
    node.create_subscription(Odometry, '/odom_raw', lambda msg: state.update(odom=msg), 10)
    node.create_subscription(String, '/test_state', lambda msg: state.update(motion=msg.data), 10)
    node.create_subscription(String, '/base/status', lambda msg: state.update(base=json.loads(msg.data)), 10)
    node.create_subscription(Twist, '/chassis/velocity_raw', lambda msg: state.update(velocity=msg), 10)
    commands = node.create_publisher(String, '/test_cmd', 10)
    results = []

    def wait_for(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise AssertionError('Launch exited; inspect ros_launch.log')
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError(label + ': ' + repr(state))

    def service(name, kind, request):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=5), name + ' unavailable'
        future = client.call_async(request)
        wait_for(future.done, 5, name)
        result = future.result()
        node.destroy_client(client)
        assert result.success, name + ': ' + result.message

    def command(text, through_cli=False):
        if through_cli:
            sent = subprocess.run(['ros2', 'run', 'motion_control_pkg', 'test_cli', text],
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, timeout=12)
            assert sent.returncode == 0, sent.stdout
            assert 'Sent ' + text in sent.stdout, sent.stdout
        else:
            commands.publish(String(data=text))

    def yaw(odom):
        q = odom.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

    try:
        wait_for(lambda: state['odom'] is not None and state['motion'].startswith('IDLE') and commands.get_subscription_count() > 0, 20, 'startup')
        for text, axis, expected in (
            ('forward_0.2m', 'distance', 0.2), ('back_0.2m', 'distance', 0.2),
            ('turn_90', 'yaw', math.pi/2), ('turnr_90', 'yaw', -math.pi/2),
        ):
            before = state['odom']
            start_x, start_y, start_yaw = before.pose.pose.position.x, before.pose.pose.position.y, yaw(before)
            command(text, through_cli=(text == 'forward_0.2m'))
            wait_for(lambda: state['motion'].startswith('RUNNING'), 5, text + ' start')
            wait_for(lambda: state['motion'].startswith('FINISHED'), 35, text + ' completion')
            after = state['odom']
            value = math.hypot(after.pose.pose.position.x-start_x, after.pose.pose.position.y-start_y) if axis == 'distance' else math.atan2(math.sin(yaw(after)-start_yaw), math.cos(yaw(after)-start_yaw))
            assert abs(value-expected) < (0.05 if axis == 'distance' else 0.12), (text, value)
            wait_for(lambda: state['velocity'] is not None and abs(state['velocity'].linear.x) < 1e-5 and abs(state['velocity'].angular.z) < 1e-5, 2, text + ' zero')
            results.append({'check': text, 'passed': True, 'observed': value})

        command('forward_1m')
        wait_for(lambda: state['motion'].startswith('RUNNING'), 5, 'freeze test start')
        service('/base/mock_freeze_feedback', SetBool, SetBool.Request(data=True))
        wait_for(lambda: 'FEEDBACK_TIMEOUT' in state['base'].get('state', ''), 3, 'base watchdog')
        wait_for(lambda: state['motion'].startswith('FAULT'), 3, 'test odom timeout')
        results.append({'check': 'feedback_timeout_latched', 'passed': True})
        service('/base/mock_freeze_feedback', SetBool, SetBool.Request(data=False))
        wait_for(lambda: state['base'].get('feedback_fresh', False), 3, 'feedback restored')
        service('/base/reset_stop', Trigger, Trigger.Request())
        command('stop')
        wait_for(lambda: state['motion'].startswith('IDLE'), 3, 'motion reset')

        service('/base/emergency_stop', Trigger, Trigger.Request())
        wait_for(lambda: state['base'].get('state') == 'EMERGENCY_STOP', 3, 'emergency stop latch')
        service('/base/reset_stop', Trigger, Trigger.Request())
        wait_for(lambda: state['base'].get('state') == 'IDLE', 3, 'emergency reset')
        results.append({'check': 'emergency_stop_and_explicit_reset', 'passed': True})

        results.append({'check': 'documented_ros2_run_cli_entrypoint', 'passed': True})
        print(json.dumps({'passed': True, 'results': results}, indent=2))
        (log_dir / 'ros_integration.json').write_text(json.dumps({'passed': True, 'results': results}, indent=2))
    finally:
        command('stop')
        rclpy.spin_once(node, timeout_sec=0.05)
        # ros2 launch forwards SIGINT to its children. Signalling the entire
        # group as well can interrupt child cleanup twice.
        launch.send_signal(signal.SIGINT) if launch.poll() is None else None
        try:
            launch.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGTERM)
            launch.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == '__main__':
    main()
