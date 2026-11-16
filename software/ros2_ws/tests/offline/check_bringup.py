"""Real bringup + robot_localization + mux with synthetic base feedback."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = rclpy.create_node('bringup_validation')
    try:
        for index, (ekf, mux, prefix) in enumerate(((False, False, ''), (True, False, ''), (True, True, 'r1/'))):
            ns = '/bringup_check_' + str(index)
            log = open(str(ARTIFACT_DIR) + '/bringup_case_%d.log' % index, 'w')
            launch_args = [
                'ros2', 'launch', 'yahboomcar_bringup', 'bringup.launch.py',
                'namespace:=' + ns, 'use_mock:=true', 'publish_model:=true',
                'rviz:=false', 'use_ekf:=' + str(ekf).lower(),
                'use_mux:=' + str(mux).lower()]
            if prefix:
                launch_args.append('frame_prefix:=' + prefix)
            child = subprocess.Popen(launch_args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            odom, statuses, owners = [], [], set()
            def on_tf(msg):
                for transform in msg.transforms:
                    if (transform.header.frame_id == prefix + 'odom' and
                            transform.child_frame_id == prefix + 'base_footprint'):
                        owners.add((transform.header.frame_id, transform.child_frame_id))
            subscriptions = [
                node.create_subscription(TFMessage, ns + '/tf', on_tf, 100),
                node.create_subscription(Odometry, ns + ('/odom' if ekf else '/odom_raw'), lambda msg: odom.append(msg), 100),
                node.create_subscription(String, ns + '/base/status', lambda msg: statuses.append(json.loads(msg.data)), 100),
            ]
            command = node.create_publisher(Twist, ns + ('/control/navigation' if mux else '/vel_raw'), 10)
            try:
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.03)
                    assert child.poll() is None, 'bringup launch exited'
                    if odom and owners and statuses:
                        break
                assert odom and owners and statuses, ('missing odom/TF/status', index)
                parameters = node.create_client(GetParameters, ns + '/x3plus_base/get_parameters')
                end = time.monotonic() + 2
                while not parameters.service_is_ready() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert parameters.service_is_ready()
                future = parameters.call_async(GetParameters.Request(names=['publish_odom_tf']))
                end = time.monotonic() + 2
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert future.done() and future.result().values[0].bool_value == (not ekf)
                assert odom[-1].header.frame_id == prefix + 'odom'
                assert odom[-1].child_frame_id == prefix + 'base_footprint'
                x_start = odom[-1].pose.pose.position.x
                end = time.monotonic() + 1.2
                next_command = 0.0
                while time.monotonic() < end:
                    if time.monotonic() >= next_command:
                        msg = Twist()
                        msg.linear.x = 0.1
                        command.publish(msg)
                        next_command = time.monotonic() + 0.05
                    rclpy.spin_once(node, timeout_sec=0.01)
                assert odom[-1].pose.pose.position.x - x_start > 0.05, (index, 'base/EKF did not progress')
                assert len(owners) == 1, (index, 'multiple odom TF owners', len(owners))
                tf_nodes = {info.node_name for info in node.get_publishers_info_by_topic(ns + '/tf')}
                assert tf_nodes.intersection({'x3plus_base', 'ekf_filter_node'}) == ({'ekf_filter_node'} if ekf else {'x3plus_base'}), tf_nodes
                if mux:
                    assert node.count_publishers(ns + '/vel_raw') == 1
                assert not any('FAULT' in item.get('state', '') or 'STOP' in item.get('state', '') for item in statuses), statuses[-3:]
                print('PASS bringup ekf=%s mux=%s prefix=%r: real odom progression, correct parameter, namespace and sole odom TF owner' % (ekf, mux, prefix), flush=True)
            finally:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGTERM)
                        child.wait(timeout=3)
                log.close()
                for subscription in subscriptions:
                    node.destroy_subscription(subscription)
                node.destroy_publisher(command)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
