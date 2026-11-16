#!/usr/bin/env python3
"""Standalone ROS 2 Humble local test script for nav2 map saver and RRT save map."""

import glob
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
import yaml

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.srv import SaveMap

LAUNCH_LOG_PATH = str(Path(os.environ.get(
    'NAV2_TEST_LOG_DIR', str(Path.home() / '.ros' / 'x3plus_validation'))) / 'nav2_map_saver_launch.log')
RRT_SCRIPT_DIR = str(Path(__file__).resolve().parent)
if RRT_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, RRT_SCRIPT_DIR)

try:
    import rrt_save_map as rrt_mod
except ImportError:
    rrt_mod = None


def print_log_tail(path, num_lines=30):
    if not os.path.exists(path):
        print(f"[ERROR] Log file not found: {path}", file=sys.stderr)
        return
    print(f"\n--- Tail of {path} ---", file=sys.stderr)
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            lines = f.readlines()
            for line in lines[-num_lines:]:
                sys.stderr.write(line)
    except Exception as exc:
        print(f"Error reading log: {exc}", file=sys.stderr)
    print("--------------------------------\n", file=sys.stderr)


class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_map_publisher_node')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos_profile)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.click_pub = self.create_publisher(PointStamped, 'clicked_point', 10)

        self._map_msg = OccupancyGrid()
        self._map_msg.header.frame_id = 'map'
        self._map_msg.info.resolution = 0.1
        self._map_msg.info.width = 10
        self._map_msg.info.height = 10
        self._map_msg.info.origin.position.x = -0.5
        self._map_msg.info.origin.position.y = -0.5
        self._map_msg.info.origin.position.z = 0.0
        self._map_msg.info.origin.orientation.w = 1.0

        grid = [0] * 100
        for r in range(10):
            for c in range(10):
                if r == 0 or r == 9 or c == 0 or c == 9:
                    grid[r * 10 + c] = 100
        self._map_msg.data = grid

        self.last_map_time = 0.0
        self.last_odom_time = 0.0
        self.publish_odom = False

    def publish_map(self):
        now_time = time.monotonic()
        if now_time - self.last_map_time >= 0.2:
            self._map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self._map_msg)
            self.last_map_time = now_time

    def publish_zero_odom(self):
        now_time = time.monotonic()
        if self.publish_odom and (now_time - self.last_odom_time >= 0.05):
            msg = Odometry()
            msg.header.frame_id = 'odom'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.angular.z = 0.0
            self.odom_pub.publish(msg)
            self.last_odom_time = now_time

    def publish_click(self, x=1.0, y=1.0):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        self.click_pub.publish(msg)


def spin_for_duration(executor, pub_node, duration_sec):
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline and rclpy.ok():
        pub_node.publish_map()
        pub_node.publish_zero_odom()
        executor.spin_once(timeout_sec=0.02)


def main():
    os.makedirs(os.path.dirname(LAUNCH_LOG_PATH), exist_ok=True)
    launch_log_file = open(LAUNCH_LOG_PATH, 'w', encoding='utf-8')

    launch_cmd = [
        'ros2', 'launch', 'yahboomcar_nav', 'map_saver.launch.py',
        'start_trigger:=false'
    ]
    launch_proc = subprocess.Popen(
        launch_cmd,
        stdout=launch_log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid if hasattr(os, 'setsid') else None
    )

    rclpy.init(args=None)
    executor = SingleThreadedExecutor()
    test_node = TestPublisherNode()
    executor.add_node(test_node)

    get_state_client = test_node.create_client(GetState, 'map_saver/get_state')
    ready_check_client = test_node.create_client(SaveMap, 'map_saver/save_map')

    active = False
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and rclpy.ok():
        test_node.publish_map()
        executor.spin_once(timeout_sec=0.1)

        if get_state_client.service_is_ready():
            req = GetState.Request()
            future = get_state_client.call_async(req)
            sub_deadline = time.monotonic() + 1.0
            while time.monotonic() < sub_deadline and not future.done() and rclpy.ok():
                test_node.publish_map()
                executor.spin_once(timeout_sec=0.05)
            if future.done() and future.result() is not None:
                if future.result().current_state.id == 3:  # PRIMARY_STATE_ACTIVE
                    active = True
                    break

    if not active:
        print_log_tail(LAUNCH_LOG_PATH)
        cleanup(launch_proc, launch_log_file, test_node, executor)
        raise AssertionError("map_saver lifecycle node did not reach ACTIVE state (id 3) within 15 seconds")

    print("[PASS] real server active: nav2 map_saver lifecycle is ACTIVE(3)")

    with tempfile.TemporaryDirectory() as tempdir:
        cli_cmd = [
            'python3',
            str(Path(RRT_SCRIPT_DIR) / 'save_map_once.py'),
            'cli_map',
            '--output-directory', tempdir,
            '--timeout', '10'
        ]
        cli_proc = subprocess.Popen(cli_cmd)
        cli_deadline = time.monotonic() + 12.0
        while cli_proc.poll() is None and time.monotonic() < cli_deadline:
            test_node.publish_map()
            executor.spin_once(timeout_sec=0.05)

        if cli_proc.poll() is None:
            cli_proc.terminate()
            cli_proc.wait(timeout=2.0)
            print_log_tail(LAUNCH_LOG_PATH)
            cleanup(launch_proc, launch_log_file, test_node, executor)
            raise AssertionError("save_map_once CLI timed out")

        if cli_proc.returncode != 0:
            print_log_tail(LAUNCH_LOG_PATH)
            cleanup(launch_proc, launch_log_file, test_node, executor)
            raise AssertionError(f"save_map_once CLI exited with code {cli_proc.returncode}")

        yaml_path = os.path.join(tempdir, 'cli_map.yaml')
        if not os.path.isfile(yaml_path) or os.path.getsize(yaml_path) == 0:
            cleanup(launch_proc, launch_log_file, test_node, executor)
            raise AssertionError(f"cli_map.yaml does not exist or is empty: {yaml_path}")

        with open(yaml_path, 'r', encoding='utf-8') as yf:
            yaml_data = yaml.safe_load(yf)

        image_path = yaml_data.get('image', '')
        if not os.path.isabs(image_path):
            image_path = os.path.join(tempdir, image_path)

        if not os.path.isfile(image_path) or os.path.getsize(image_path) == 0:
            cleanup(launch_proc, launch_log_file, test_node, executor)
            raise AssertionError(f"Referenced PGM file does not exist or is empty: {image_path}")

        res = yaml_data.get('resolution')
        origin = yaml_data.get('origin')
        if abs(float(res) - 0.1) > 1e-4:
            cleanup(launch_proc, launch_log_file, test_node, executor)
            raise AssertionError(f"Resolution mismatch: expected 0.1, got {res}")
        if abs(float(origin[0]) - (-0.5)) > 1e-3 or abs(float(origin[1]) - (-0.5)) > 1e-3:
            cleanup(launch_proc, launch_log_file, test_node, executor)
            raise AssertionError(f"Origin mismatch: expected [-0.5, -0.5, ...], got {origin}")

        print("[PASS] CLI YAML/PGM: save_map_once.py generated verified non-empty map artifacts")

        if rrt_mod is None:
            cleanup(launch_proc, launch_log_file, test_node, executor)
            raise AssertionError(f"Failed to import rrt_save_map from {RRT_SCRIPT_DIR}")

        test_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

        rrt_args = [
            '--ros-args',
            '-p', 'rrt_map_name:=rrt_map',
            '-p', f'output_directory:={tempdir}',
            '-p', 'waiting_time:=3'
        ]
        rclpy.init(args=rrt_args)
        executor = SingleThreadedExecutor()

        test_node = TestPublisherNode()
        executor.add_node(test_node)

        rrt_node = rrt_mod.RRTSaveMap()
        executor.add_node(rrt_node)

        spin_for_duration(executor, test_node, 0.5)

        for _ in range(5):
            test_node.publish_click()
            spin_for_duration(executor, test_node, 0.05)

        test_node.publish_odom = True
        rrt_yaml = os.path.join(tempdir, 'rrt_map.yaml')

        rrt_deadline = time.monotonic() + 6.0
        while time.monotonic() < rrt_deadline and rclpy.ok():
            test_node.publish_map()
            test_node.publish_zero_odom()
            executor.spin_once(timeout_sec=0.02)
            if os.path.isfile(rrt_yaml) and os.path.getsize(rrt_yaml) > 0:
                break

        if not (os.path.isfile(rrt_yaml) and os.path.getsize(rrt_yaml) > 0):
            print_log_tail(LAUNCH_LOG_PATH)
            cleanup(launch_proc, launch_log_file, test_node, executor, rrt_node)
            raise AssertionError("rrt_save_map did not trigger and write rrt_map.yaml within 6 seconds")

        with open(rrt_yaml, 'r', encoding='utf-8') as f:
            rrt_data = yaml.safe_load(f)
        rrt_pgm = rrt_data.get('image', '')
        if not os.path.isabs(rrt_pgm):
            rrt_pgm = os.path.join(tempdir, rrt_pgm)

        if not (os.path.isfile(rrt_pgm) and os.path.getsize(rrt_pgm) > 0):
            cleanup(launch_proc, launch_log_file, test_node, executor, rrt_node)
            raise AssertionError(f"rrt_map PGM file does not exist or is empty: {rrt_pgm}")

        if hasattr(rrt_node, '_save_pending'):
            callback_deadline = time.monotonic() + 1.0
            while rrt_node._save_pending and time.monotonic() < callback_deadline:
                test_node.publish_map()
                executor.spin_once(timeout_sec=0.02)
            if rrt_node._save_pending:
                cleanup(launch_proc, launch_log_file, test_node, executor, rrt_node)
                raise AssertionError("rrt_node._save_pending is still True after save completed")

        if hasattr(rrt_node, '_save_future') and rrt_node._save_future is not None:
            if not rrt_node._save_future.done():
                cleanup(launch_proc, launch_log_file, test_node, executor, rrt_node)
                raise AssertionError("rrt_node._save_future is not done")

        if hasattr(rrt_node, '_save_triggered'):
            if not rrt_node._save_triggered:
                cleanup(launch_proc, launch_log_file, test_node, executor, rrt_node)
                raise AssertionError("rrt_node._save_triggered is False despite map being written")

        initial_mtime = os.path.getmtime(rrt_yaml)
        spin_for_duration(executor, test_node, 2.0)
        after_mtime = os.path.getmtime(rrt_yaml)

        if initial_mtime != after_mtime:
            cleanup(launch_proc, launch_log_file, test_node, executor, rrt_node)
            raise AssertionError("rrt_map.yaml mtime changed; repeated save command was incorrectly issued")

        print("[PASS] real RRT timer one-shot actual disk proof: rrt_map.yaml/.pgm verified without re-saves")

        cleanup(launch_proc, launch_log_file, test_node, executor, rrt_node)


def cleanup(launch_proc, launch_log_file, test_node=None, executor=None, rrt_node=None):
    if launch_proc is not None and launch_proc.poll() is None:
        try:
            launch_proc.send_signal(signal.SIGINT)
            launch_proc.wait(timeout=5.0)
        except (subprocess.TimeoutExpired, Exception):
            try:
                launch_proc.terminate()
                launch_proc.wait(timeout=2.0)
            except Exception:
                pass

    if launch_log_file is not None and not launch_log_file.closed:
        try:
            launch_log_file.close()
        except Exception:
            pass

    try:
        if test_node is not None:
            test_node.destroy_node()
        if rrt_node is not None:
            rrt_node.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
