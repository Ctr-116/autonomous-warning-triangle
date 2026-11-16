#!/usr/bin/env python3
"""Standalone local integration test for actual ROS 2 Humble Nav2 bringup with X3plus.

Two modes:
  * default (navigation_only): localization:=false, synthetic transient map
    publisher plus a static map->odom fixture owned by this test.
  * --amcl: localization:=true with a temporary valid PGM/YAML map file.
    Only the ACTUAL map_server and ACTUAL AMCL own /map and map->odom.

The physical fixture is an idealized kinematic stand-in, not a calibrated
vehicle.
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from x3plus_base.base_node import BaseNode


LOG_DIR = os.environ.get('NAV2_TEST_LOG_DIR', str(Path.home() / '.ros' / 'x3plus_validation'))
LOG_PATH_DEFAULT = os.path.join(LOG_DIR, "nav2_runtime_launch.log")
LOG_PATH_AMCL = os.path.join(LOG_DIR, "nav2_amcl_runtime_launch.log")


def dump_launch_log(log_path, lines=60):
    print(f"\n--- [Nav2 Launch Log Tail (last {lines} lines)] ---")
    if os.path.exists(log_path):
        try:
            with open(log_path, "r", encoding="utf-8", errors="replace") as f:
                content = f.readlines()
                print("".join(content[-lines:]))
        except Exception as e:
            print(f"Failed to read launch log: {e}")
    else:
        print(f"Log file not found at {log_path}")
    print("--- [End Launch Log] ---\n")


def _write_test_map(directory):
    """Write a valid 100x100 P5 binary PGM + YAML pair. Returns yaml path."""
    width = 100
    height = 100
    # PGM convention used by map_server negate=0: 0 = black = occupied,
    # 255 = white = free.
    border = bytes([0]) * width
    interior = bytes([255]) * width
    pixels = bytearray()
    for r in range(height):
        if r == 0 or r == height - 1:
            pixels += border
        else:
            pixels += b"\x00" + interior[1:-1] + b"\x00"

    pgm_path = os.path.join(directory, "nav2_test_map.pgm")
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n")
        f.write(f"{width} {height}\n".encode("ascii"))
        f.write(b"255\n")
        f.write(bytes(pixels))

    yaml_path = os.path.join(directory, "nav2_test_map.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("image: nav2_test_map.pgm\n")
        f.write("resolution: 0.1\n")
        f.write("origin: [-5.0, -5.0, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")
    return yaml_path


class Nav2OfflineTester(Node):
    def __init__(self, amcl=False):
        super().__init__("nav2_offline_tester")

        self.amcl = bool(amcl)

        # Odometry tracking
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_yaw = 0.0
        self.odom_received = False
        self.odom_count = 0

        # Command / relay tracking
        self.cmd_vel_count = 0
        self.cmd_vel_nonzero_count = 0
        self.vel_raw_count = 0
        self.vel_raw_nonzero_count = 0
        self.last_cmd_vel = Twist()
        self.last_vel_raw = Twist()

        # TF tooling used only by the AMCL mode checks.
        self.static_tf_broadcaster = None
        self.tf_buffer = None
        self.tf_listener = None

        if not self.amcl:
            # Navigation-only mode: synthetic map + static map->odom fixture.
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            self.broadcast_map_to_odom_tf()

            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.map_pub = self.create_publisher(OccupancyGrid, "map", map_qos)
            self.map_msg = self._build_map_msg()
            self.map_timer = self.create_timer(1.0, self.on_map_timer)
        else:
            # AMCL mode: real map_server + AMCL own /map and map->odom.
            self.map_pub = None
            self.map_msg = None
            self.map_timer = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.initial_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, "initialpose", 10
            )

        # Scan publisher: BEST_EFFORT, 10Hz, frame base_footprint
        self.scan_pub = self.create_publisher(
            LaserScan, "scan", qos_profile_sensor_data
        )

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, "odom_raw", self.on_odom, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.on_cmd_vel, 10
        )
        self.vel_raw_sub = self.create_subscription(
            Twist, "vel_raw", self.on_vel_raw, 10
        )

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.lifecycle_client = self.create_client(GetState, 'bt_navigator/get_state')

        # Timers
        self.scan_timer = self.create_timer(0.1, self.on_scan_timer)

    def broadcast_map_to_odom_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf)

    def _build_map_msg(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = 0.1
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        grid = [0] * (100 * 100)
        for r in range(100):
            for c in range(100):
                if r == 0 or r == 99 or c == 0 or c == 99:
                    grid[r * 100 + c] = 100
                else:
                    grid[r * 100 + c] = 0
        msg.data = grid
        return msg

    def on_map_timer(self):
        if self.map_pub is None:
            return
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def publish_initial_pose(self, executor):
        if not self.amcl:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = 0.0001    # x
        cov[7] = 0.0001    # y
        cov[35] = 0.0001   # yaw
        msg.pose.covariance = cov
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            spin_for(executor, 0.1)
        print("[OFFLINE_CHECK] Published initialpose map(0,0,yaw=0) with tight covariance (AMCL mode).")

    def on_odom(self, msg: Odometry):
        self.odom_received = True
        self.odom_count += 1
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.last_odom_yaw = 2.0 * math.atan2(qz, qw)

        assert math.isfinite(msg.pose.pose.position.x), "Non-finite odom x"
        assert math.isfinite(msg.pose.pose.position.y), "Non-finite odom y"
        assert math.isfinite(msg.twist.twist.linear.x), "Non-finite odom linear x"
        assert math.isfinite(msg.twist.twist.linear.y), "Non-finite odom linear y"
        assert math.isfinite(msg.twist.twist.angular.z), "Non-finite odom angular z"

    def on_cmd_vel(self, msg: Twist):
        self.cmd_vel_count += 1
        assert math.isfinite(msg.linear.x), "Non-finite cmd_vel linear.x"
        assert math.isfinite(msg.linear.y), "Non-finite cmd_vel linear.y"
        assert math.isfinite(msg.angular.z), "Non-finite cmd_vel angular.z"
        assert abs(msg.linear.x) <= 0.20001, f"cmd_vel linear.x exceeded safety limit: {msg.linear.x}"
        assert abs(msg.linear.y) <= 0.15001, f"cmd_vel linear.y exceeded safety limit: {msg.linear.y}"
        assert abs(msg.angular.z) <= 0.60001, f"cmd_vel angular.z exceeded safety limit: {msg.angular.z}"

        if abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.cmd_vel_nonzero_count += 1
        self.last_cmd_vel = msg

    def on_vel_raw(self, msg: Twist):
        self.vel_raw_count += 1
        assert math.isfinite(msg.linear.x), "Non-finite vel_raw linear.x"
        assert math.isfinite(msg.linear.y), "Non-finite vel_raw linear.y"
        assert math.isfinite(msg.angular.z), "Non-finite vel_raw angular.z"
        assert abs(msg.linear.x) <= 0.20001, f"vel_raw linear.x exceeded safety limit: {msg.linear.x}"
        assert abs(msg.linear.y) <= 0.15001, f"vel_raw linear.y exceeded safety limit: {msg.linear.y}"
        assert abs(msg.angular.z) <= 0.60001, f"vel_raw angular.z exceeded safety limit: {msg.angular.z}"

        if abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.vel_raw_nonzero_count += 1
        self.last_vel_raw = msg

    def on_scan_timer(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_footprint"
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 8.0

        rx = self.last_odom_x
        ry = self.last_odom_y
        ryaw = self.last_odom_yaw

        ranges = []
        for i in range(360):
            phi = scan.angle_min + i * scan.angle_increment
            theta = ryaw + phi
            dx = math.cos(theta)
            dy = math.sin(theta)

            t_min = scan.range_max
            # Raycast against square wall [-4.9, 4.9]
            if dx > 1e-6:
                t = (4.9 - rx) / dx
                if t > 0:
                    y_hit = ry + t * dy
                    if -4.9 <= y_hit <= 4.9:
                        t_min = min(t_min, t)
            elif dx < -1e-6:
                t = (-4.9 - rx) / dx
                if t > 0:
                    y_hit = ry + t * dy
                    if -4.9 <= y_hit <= 4.9:
                        t_min = min(t_min, t)

            if dy > 1e-6:
                t = (4.9 - ry) / dy
                if t > 0:
                    x_hit = rx + t * dx
                    if -4.9 <= x_hit <= 4.9:
                        t_min = min(t_min, t)
            elif dy < -1e-6:
                t = (-4.9 - ry) / dy
                if t > 0:
                    x_hit = rx + t * dx
                    if -4.9 <= x_hit <= 4.9:
                        t_min = min(t_min, t)

            dist = max(scan.range_min, min(scan.range_max, t_min))
            ranges.append(dist)

        scan.ranges = ranges
        self.scan_pub.publish(scan)


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)


def spin_until(executor, predicate, deadline, description):
    while time.monotonic() < deadline:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.05)
    raise TimeoutError(f"Timed out waiting for {description}")


def wait_lifecycle_active(executor, node, service_name, deadline, launch_process, log_path):
    client = node.create_client(GetState, service_name)
    try:
        while time.monotonic() < deadline:
            if launch_process is not None and launch_process.poll() is not None:
                raise RuntimeError(f"Nav2 launch exited while waiting for {service_name}")
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                inner_deadline = min(deadline, time.monotonic() + 2.0)
                while not future.done() and time.monotonic() < inner_deadline:
                    executor.spin_once(timeout_sec=0.05)
                if future.done() and future.result() is not None:
                    if future.result().current_state.id == 3:
                        return True
            executor.spin_once(timeout_sec=0.05)
        return False
    finally:
        node.destroy_client(client)


def main():
    parser = argparse.ArgumentParser(
        description="Nav2 offline integration check (navigation_only or --amcl)"
    )
    parser.add_argument(
        "--amcl", action="store_true",
        help="Run actual map_server + AMCL via localization:=true",
    )
    parser.add_argument('--namespace', default='', help='Optional robot namespace, e.g. nav_robot')
    args = parser.parse_args()

    amcl_mode = bool(args.amcl)
    namespace = args.namespace.strip('/')
    if namespace and not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace):
        parser.error('namespace must contain valid ROS name tokens')
    log_path = LOG_PATH_AMCL if amcl_mode else LOG_PATH_DEFAULT
    if namespace:
        stem, suffix = os.path.splitext(log_path)
        log_path = stem + '_' + namespace.replace('/', '_') + suffix
    scenario = "AMCL (actual map_server + actual AMCL ownership)" if amcl_mode \
        else "navigation_only (synthetic map + static map->odom fixture)"
    print(f"[OFFLINE_CHECK] Scenario: {scenario}")
    print("[OFFLINE_CHECK] Artifact note: idealized physical fixture, not a real vehicle calibration.")

    os.makedirs(LOG_DIR, exist_ok=True)

    temp_dir = None
    map_yaml = None
    if amcl_mode:
        temp_dir = tempfile.TemporaryDirectory(prefix="nav2_amcl_fixture_")
        map_yaml = _write_test_map(temp_dir.name)
        print(f"[OFFLINE_CHECK] Wrote temporary map fixture: {map_yaml}")

    launch_log_file = open(log_path, "w", buffering=1, encoding="utf-8")

    launch_cmd = [
        "ros2",
        "launch",
        "yahboomcar_nav",
        "yahboomcar_navigation.launch.py",
        f"localization:={'true' if amcl_mode else 'false'}",
        "rviz:=false",
        "use_composition:=false",
        'use_namespace:=' + ('true' if namespace else 'false'),
    ]
    if namespace:
        launch_cmd.append('namespace:=' + namespace)
    if amcl_mode:
        launch_cmd.append(f"map:={map_yaml}")

    print("[OFFLINE_CHECK] Starting Nav2 launch subprocess...")
    process = subprocess.Popen(
        launch_cmd,
        stdout=launch_log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    ros_args = [
        "--ros-args",
        "-p", "use_mock:=true",
        "-p", "publish_rate:=30.0",
        "-p", "max_linear_x:=0.2",
        "-p", "max_linear_y:=0.15",
        "-p", "max_angular_z:=0.6",
    ]
    if namespace:
        ros_args.extend(['-r', '__ns:=/' + namespace,
                         '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])

    rclpy.init(args=ros_args)
    executor = SingleThreadedExecutor()

    base_node = None
    test_node = None

    start_time = time.monotonic()
    overall_timeout = 80.0

    try:
        base_node = BaseNode()
        test_node = Nav2OfflineTester(amcl=amcl_mode)

        executor.add_node(base_node)
        executor.add_node(test_node)

        # Spin to warm up mock odometry and (default mode) initial static TF.
        spin_for(executor, 2.0)

        assert test_node.odom_received, "BaseNode odom_raw was not received during startup"
        print("[PASS GROUP 1] Mock BaseNode initialized, odom_raw received, TF online.")

        if amcl_mode:
            # Localization must be initialized before Nav2 can activate its
            # global costmap; waiting for bt_navigator first creates a cycle.
            amcl_deadline = time.monotonic() + 15.0
            assert wait_lifecycle_active(
                executor, test_node, 'map_server/get_state',
                amcl_deadline, process, log_path), 'map_server did not reach ACTIVE'
            assert wait_lifecycle_active(
                executor, test_node, 'amcl/get_state',
                amcl_deadline, process, log_path), 'amcl did not reach ACTIVE'
            test_node.publish_initial_pose(executor)
            tf_deadline = time.monotonic() + 15.0
            tf_ok = False
            while time.monotonic() < tf_deadline:
                try:
                    test_node.tf_buffer.lookup_transform('map', 'odom', Time())
                    tf_ok = True
                    break
                except Exception:
                    executor.spin_once(timeout_sec=0.05)
            assert tf_ok, 'Actual map->odom transform from AMCL was not observed'
            assert test_node.count_publishers('map') == 1, 'map_server must be the sole map publisher'
            print('[PASS GROUP 6] Actual AMCL/map_server ACTIVE, initialpose accepted, map->odom observed, sole map publisher.')

        # Wait for the real navigate_to_pose action server with a deadline.
        print("[OFFLINE_CHECK] Waiting for navigate_to_pose action server...")
        action_wait_deadline = time.monotonic() + 30.0
        server_ready = False
        while time.monotonic() < action_wait_deadline:
            if process.poll() is not None:
                raise RuntimeError('Nav2 launch exited during startup')
            if test_node.nav_client.wait_for_server(timeout_sec=0.1):
                server_ready = True
                break
            executor.spin_once(timeout_sec=0.05)

        if not server_ready:
            raise RuntimeError("Timeout (30s) waiting for navigate_to_pose action server.")

        print("[PASS GROUP 2] Real Nav2 navigate_to_pose action server online.")

        # Confirm bt_navigator lifecycle activation.
        lifecycle_active = wait_lifecycle_active(
            executor, test_node, 'bt_navigator/get_state',
            action_wait_deadline, process, log_path)
        assert lifecycle_active, 'bt_navigator lifecycle did not reach ACTIVE'

        # Verify single velocity publisher on vel_raw
        publishers_count = base_node.count_publishers("vel_raw")
        assert publishers_count == 1, f"Expected exactly 1 publisher on vel_raw, found {publishers_count}"
        print(f"[PASS GROUP 3] Verified single velocity publisher on vel_raw (count={publishers_count}).")
        if namespace:
            for topic in ('/vel_raw', '/cmd_vel', '/odom_raw', '/tf', '/tf_static', '/map'):
                assert test_node.count_publishers(topic) == 0, f'Namespaced launch leaked publisher to {topic}'
            print('[PASS GROUP 7] Command, odometry, TF and map publishers remain inside the robot namespace.')

        # TEST GOAL 1: Send goal map(x=0.45, y=0.0, yaw=0.0) -> SUCCEEDED (4)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = test_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 0.45
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        print("[OFFLINE_CHECK] Sending Goal 1: x=0.45, y=0.0...")
        initial_x = test_node.last_odom_x
        test_node.cmd_vel_nonzero_count = 0
        test_node.vel_raw_nonzero_count = 0

        goal_future = test_node.nav_client.send_goal_async(goal_msg)
        while not goal_future.done():
            executor.spin_once(timeout_sec=0.05)
            if time.monotonic() - start_time > overall_timeout:
                raise TimeoutError("Overall timeout reached waiting for Goal 1 acceptance.")

        goal_handle = goal_future.result()
        assert goal_handle is not None and goal_handle.accepted, "Goal 1 was rejected by Nav2 bt_navigator."

        result_future = goal_handle.get_result_async()
        goal_deadline = time.monotonic() + 25.0
        while not result_future.done():
            executor.spin_once(timeout_sec=0.05)
            if time.monotonic() > goal_deadline:
                raise TimeoutError("Goal 1 did not reach result within 25.0s deadline.")
            if time.monotonic() - start_time > overall_timeout:
                raise TimeoutError("Overall test timeout reached during Goal 1 execution.")

        action_result = result_future.result()
        status = action_result.status

        delta_x = test_node.last_odom_x - initial_x
        final_dist = math.hypot(test_node.last_odom_x - 0.45, test_node.last_odom_y - 0.0)

        print(
            f"[OFFLINE_CHECK] Goal 1 Completed: status={status}, delta_x={delta_x:.3f}m, "
            f"dist_to_goal={final_dist:.3f}m, cmd_nonzero={test_node.cmd_vel_nonzero_count}, "
            f"vel_raw_nonzero={test_node.vel_raw_nonzero_count}"
        )

        assert status == GoalStatus.STATUS_SUCCEEDED, f"Goal 1 failed: status={status} (expected SUCCEEDED=4)"
        assert delta_x > 0.25, f"Robot x advancement {delta_x:.3f}m <= 0.25m threshold"
        assert final_dist < 0.20, f"Final position distance {final_dist:.3f}m >= 0.20m tolerance"
        assert test_node.cmd_vel_nonzero_count > 0, "No non-zero cmd_vel commands observed"
        assert test_node.vel_raw_nonzero_count > 0, "No non-zero vel_raw relay outputs observed"

        print("[PASS GROUP 4] Goal 1 SUCCEEDED: accurate motion closure, relays active, metrics verified.")

        # TEST GOAL 2: Send goal map(x=2.0, y=0.0), wait for non-zero output and 0.4s, then cancel
        print("[OFFLINE_CHECK] Sending Goal 2: x=2.0, y=0.0 for cancellation test...")
        goal2_msg = NavigateToPose.Goal()
        goal2_msg.pose.header.frame_id = "map"
        goal2_msg.pose.header.stamp = test_node.get_clock().now().to_msg()
        goal2_msg.pose.pose.position.x = 2.0
        goal2_msg.pose.pose.position.y = 0.0
        goal2_msg.pose.pose.orientation.w = 1.0

        test_node.cmd_vel_nonzero_count = 0
        test_node.vel_raw_nonzero_count = 0

        goal2_future = test_node.nav_client.send_goal_async(goal2_msg)
        while not goal2_future.done():
            executor.spin_once(timeout_sec=0.05)
            if time.monotonic() - start_time > overall_timeout:
                raise TimeoutError("Overall timeout reached waiting for Goal 2 acceptance.")

        goal2_handle = goal2_future.result()
        assert goal2_handle is not None and goal2_handle.accepted, "Goal 2 was rejected by Nav2 bt_navigator."

        goal2_result_future = goal2_handle.get_result_async()

        # Wait until non-zero velocity output is observed
        nonzero_seen = False
        poll_deadline = time.monotonic() + 10.0
        while time.monotonic() < poll_deadline:
            executor.spin_once(timeout_sec=0.05)
            if test_node.vel_raw_nonzero_count > 0 or test_node.cmd_vel_nonzero_count > 0:
                nonzero_seen = True
                break

        assert nonzero_seen, "Did not observe motion initiation on Goal 2 before cancel"

        # Let it run for 0.4s
        spin_for(executor, 0.4)

        print("[OFFLINE_CHECK] Cancelling Goal 2...")
        cancel_future = goal2_handle.cancel_goal_async()
        cancel_deadline = time.monotonic() + 5.0
        while not cancel_future.done():
            executor.spin_once(timeout_sec=0.05)
            if time.monotonic() > cancel_deadline:
                raise TimeoutError("Timeout waiting for cancel_goal_async response.")

        cancel_response = cancel_future.result()
        assert cancel_response.return_code == 0, f"Unexpected cancel return code: {cancel_response.return_code}"

        # Wait for result to confirm CANCELED (5)
        wait_cancel_result = time.monotonic() + 5.0
        while not goal2_result_future.done():
            executor.spin_once(timeout_sec=0.05)
            if time.monotonic() > wait_cancel_result:
                raise TimeoutError("Timeout waiting for goal 2 canceled result.")

        g2_status = goal2_result_future.result().status
        assert g2_status == GoalStatus.STATUS_CANCELED, f"Expected STATUS_CANCELED (5), got {g2_status}"

        # Require velocities to settle to bounded zero within 1.0 second
        spin_for(executor, 1.0)

        final_vx = abs(test_node.last_vel_raw.linear.x)
        final_vy = abs(test_node.last_vel_raw.linear.y)
        final_wz = abs(test_node.last_vel_raw.angular.z)
        assert final_vx <= 1e-6 and final_vy <= 1e-6 and final_wz <= 1e-6, (
            f"Velocities did not drop to zero within 1s after cancel: vx={final_vx}, vy={final_vy}, wz={final_wz}"
        )

        print(f"[PASS GROUP 5] Goal 2 CANCELED: status={g2_status}, velocities settled to zero within 1s.")
        print(f"\nALL INTEGRATION CHECKS PASSED SUCCESSFULLY in {time.monotonic() - start_time:.2f}s.\n")

    except Exception as exc:
        print(f"\n[FAILURE] Exception during offline check: {exc}", file=sys.stderr)
        dump_launch_log(log_path, lines=80)
        raise

    finally:
        print("[OFFLINE_CHECK] Cleaning up child processes and nodes...")

        # Terminate Nav2 launch cleanly (signal the direct child only).
        if process and process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5.0)
            except Exception:
                try:
                    process.terminate()
                    process.wait(timeout=2.0)
                except Exception:
                    pass

        try:
            launch_log_file.close()
        except Exception:
            pass

        if base_node is not None:
            try:
                base_node.close()
            except Exception:
                pass

        if test_node is not None:
            try:
                executor.remove_node(test_node)
            except Exception:
                pass
            try:
                test_node.destroy_node()
            except Exception:
                pass

        if base_node is not None:
            try:
                executor.remove_node(base_node)
            except Exception:
                pass
            try:
                base_node.destroy_node()
            except Exception:
                pass

        try:
            executor.shutdown()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

        # Temporary map fixture is removed only after the launch has stopped.
        if temp_dir is not None:
            try:
                temp_dir.cleanup()
            except Exception:
                pass


if __name__ == "__main__":
    main()
