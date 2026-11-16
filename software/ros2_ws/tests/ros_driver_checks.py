#!/usr/bin/env python3
"""ROS API checks of limits, command deadman, stop latch and name collision."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    log_dir = Path(os.environ.get('VALIDATION_LOG_DIR', '/tmp/x3plus-validation'))
    log_dir.mkdir(parents=True, exist_ok=True)
    log = (log_dir / 'ros_driver.log').open('w')
    launch = subprocess.Popen(['ros2', 'run', 'x3plus_base', 'base_node'],
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('driver_api_probe')
    feedback = {'vel': None, 'status': {}, 'odom': None}
    node.create_subscription(Twist, '/chassis/velocity_raw', lambda m: feedback.update(vel=m), 10)
    node.create_subscription(String, '/base/status', lambda m: feedback.update(status=json.loads(m.data)), 10)
    node.create_subscription(Odometry, '/odom_raw', lambda m: feedback.update(odom=m), 10)
    pub = node.create_publisher(Twist, '/vel_raw', 1)
    results = []

    def wait(predicate, timeout=5, publish=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if publish is not None:
                pub.publish(publish)
            rclpy.spin_once(node, timeout_sec=0.03)
            if predicate():
                return
        raise AssertionError(repr(feedback))

    def moving():
        v = feedback['vel']
        return v is not None and (abs(v.linear.x) + abs(v.linear.y) + abs(v.angular.z)) > 0.001

    def reset():
        client = node.create_client(Trigger, '/base/reset_stop')
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success, future.result().message
        node.destroy_client(client)

    try:
        wait(lambda: pub.get_subscription_count() == 1 and feedback['status'].get('feedback_fresh', False))
        command = Twist()
        command.linear.x = command.linear.y = 10.0
        command.angular.z = 10.0
        wait(moving, publish=command)
        v = feedback['vel']
        assert math.hypot(v.linear.x, v.linear.y) <= 0.200001 and abs(v.angular.z) <= 0.600001
        results.append({'check': 'ROS_command_limits', 'passed': True})
        # Stop sending: feedback should return to zero without a stop command.
        wait(lambda: not moving(), timeout=2)
        results.append({'check': 'ROS_command_deadman', 'passed': True})
        invalid = Twist()
        invalid.linear.x = float('nan')
        pub.publish(invalid)
        wait(lambda: 'INVALID_COMMAND' in feedback['status'].get('state', ''))
        wait(lambda: not moving())
        reset()
        results.append({'check': 'ROS_nonfinite_command_latch', 'passed': True})
        second = Node('duplicate_controller_probe')
        second_pub = second.create_publisher(Twist, '/vel_raw', 1)
        wait(lambda: feedback['status'].get('state') == 'MULTIPLE_COMMAND_PUBLISHERS')
        second.destroy_publisher(second_pub)
        second.destroy_node()
        wait(lambda: feedback['status'].get('command_publishers') == 1)
        reset()
        results.append({'check': 'ROS_duplicate_controller_latch', 'passed': True})

        collision = subprocess.run(['ros2', 'run', 'x3plus_base', 'base_node', '--ros-args',
                                    '-r', '/chassis/velocity_raw:=/vel_raw'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        assert collision.returncode != 0 and 'distinct' in collision.stdout, collision.stdout
        results.append({'check': 'ROS_remap_collision_rejected', 'passed': True})
        result = {'passed': True, 'results': results}
        print(json.dumps(result, indent=2))
        (log_dir / 'ros_driver_checks.json').write_text(json.dumps(result, indent=2))
    finally:
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
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
