#!/usr/bin/env python3
"""Real offline smoke test for slam_toolbox mapping on ROS 2 Humble.

Launches yahboomcar_nav/yahboomcar_map.launch.py with use_sim_time:=false and
feeds it a synthetic 360-beam LaserScan of a stationary square room through a
live rclpy Node.  The fixture publishes only odom->base_footprint (dynamic) and
base_footprint->laser (static); map->odom is never published by the fixture and
must come from the actual slam_toolbox process.

Two PASS groups are recorded:
  1. Mapping: /map OccupancyGrid with dimensions > 20x20, resolution ~0.05,
     finite origin, actual free (0) and occupied (100) cells.
  2. TF: an actual map->odom transform exists, is finite, and its quaternion is
     a reasonable (near-identity) rotation.

Exit code 0 on success, 1 on any explicit failure.  No silent skip.
"""

import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


DEADLINE_SECONDS = 30.0
ARTIFACT_DIR_ENV = 'X3PLUS_TEST_ARTIFACTS'


def _artifact_dir():
    env_dir = os.environ.get(ARTIFACT_DIR_ENV)
    if env_dir:
        try:
            os.makedirs(env_dir, exist_ok=True)
            return env_dir
        except OSError:
            pass
    return tempfile.mkdtemp(prefix='slam_offline_')


class ScanFixture(Node):
    NUM_BEAMS = 360

    def __init__(self):
        super().__init__('slam_offline_scan_fixture')
        self._angle_min = -math.pi
        self._increment = (2.0 * math.pi) / self.NUM_BEAMS
        self._angle_max = self._angle_min + self._increment * (self.NUM_BEAMS - 1)
        self._range_min = 0.05
        self._range_max = 8.0

        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._static_broadcaster.sendTransform(self._make_transform(
            'base_footprint', 'laser', self.get_clock().now().to_msg()))
        self._scan_timer = self.create_timer(0.15, self._publish_scan)  # ~6.7 Hz
        self._tf_timer = self.create_timer(0.05, self._publish_odom_tf)

    def _range_for(self, angle):
        c = abs(math.cos(angle))
        s = abs(math.sin(angle))
        return 2.0 / max(c, s)

    def _publish_scan(self):
        now = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser'
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = [
            self._range_for(self._angle_min + i * self._increment)
            for i in range(self.NUM_BEAMS)
        ]
        self._scan_pub.publish(scan)

    def _make_transform(self, parent, child, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        return t

    def _publish_odom_tf(self):
        stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(
            self._make_transform('odom', 'base_footprint', stamp))


class MapChecker(Node):
    def __init__(self):
        super().__init__('slam_offline_map_checker')
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_msg = None
        self.map_odom = None
        self.tf_pairs = set()
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_tf(self, msg):
        for t in msg.transforms:
            self.tf_pairs.add((t.header.frame_id, t.child_frame_id))
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                self.map_odom = t


def _finite(v):
    return isinstance(v, float) and math.isfinite(v)


def _check_map(msg):
    if msg is None:
        return None
    info = msg.info
    if info.width <= 20 or info.height <= 20:
        return None
    if not _finite(info.resolution) or abs(info.resolution - 0.05) > 0.005:
        return None
    origin = info.origin
    if not all(_finite(v) for v in (
            origin.position.x, origin.position.y, origin.position.z,
            origin.orientation.x, origin.orientation.y,
            origin.orientation.z, origin.orientation.w)):
        return None
    free = occupied = 0
    for cell in msg.data:
        if cell == 0:
            free += 1
        elif cell == 100:
            occupied += 1
    if len(msg.data) != info.width * info.height or free <= 0 or occupied <= 0:
        return None
    return {'free': free, 'occupied': occupied,
            'known': free + occupied, 'width': info.width,
            'height': info.height, 'resolution': info.resolution}


def _check_tf(t):
    if t is None:
        return None
    tr = t.transform
    vals = [tr.translation.x, tr.translation.y, tr.translation.z,
            tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
    if not all(_finite(v) for v in vals):
        return None
    norm = math.sqrt(tr.rotation.x ** 2 + tr.rotation.y ** 2 +
                     tr.rotation.z ** 2 + tr.rotation.w ** 2)
    if abs(norm - 1.0) > 1e-3 or abs(tr.rotation.z) > 0.1:
        return None
    return {'x': tr.translation.x, 'y': tr.translation.y, 'norm': norm,
            'trans_mag': math.sqrt(tr.translation.x ** 2 + tr.translation.y ** 2)}


def _tail(path, n=40):
    try:
        with open(path, 'r', errors='replace') as fh:
            lines = fh.readlines()
        sys.stderr.write(''.join(lines[-n:]))
    except OSError:
        pass


def _cleanup(proc, node):
    if proc is not None and proc.poll() is None:
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                proc.kill()
    if node is not None:
        try:
            node.destroy_node()
        except Exception:
            pass


def main():
    artifacts = _artifact_dir()
    log_path = os.path.join(artifacts, 'launch.log')
    log_file = open(log_path, 'w')
    proc = None
    checker = None
    fixture = None
    executor = None
    try:
        rclpy.init(args=None)
        proc = subprocess.Popen(
            ['ros2', 'launch', 'yahboomcar_nav', 'yahboomcar_map.launch.py',
             'use_sim_time:=false'],
            stdout=log_file, stderr=subprocess.STDOUT)
        checker = MapChecker()
        fixture = ScanFixture()
        executor = SingleThreadedExecutor()
        executor.add_node(checker)
        executor.add_node(fixture)

        start = time.monotonic()
        map_result = None
        tf_result = None
        while time.monotonic() - start < DEADLINE_SECONDS:
            if proc.poll() is not None:
                sys.stderr.write('Launcher exited early (rc=%s)\n' % proc.returncode)
                break
            executor.spin_once(timeout_sec=0.05)
            if map_result is None:
                map_result = _check_map(checker.map_msg)
            if tf_result is None:
                tf_result = _check_tf(checker.map_odom)
            if map_result is not None and tf_result is not None:
                break

        groups = []
        if map_result is not None:
            groups.append(('mapping', map_result))
        if tf_result is not None:
            groups.append(('tf', tf_result))

        if map_result is not None and tf_result is not None:
            print('PASS groups=2 mapping=%s tf=%s' % (map_result, tf_result))
            return 0

        sys.stderr.write('FAIL groups=%d\n' % len(groups))
        if map_result is None:
            sys.stderr.write('No valid /map produced.\n')
            if checker.map_msg is not None:
                m = checker.map_msg
                sys.stderr.write('Observed map: %sx%s res=%s cells=%s\n' % (
                    m.info.width, m.info.height, m.info.resolution, sorted(set(m.data))))
        if tf_result is None:
            sys.stderr.write('No valid actual map->odom transform.\n')
            sys.stderr.write('Observed TF pairs: %s\n' % sorted(checker.tf_pairs))
        log_file.flush()
        _tail(log_path)
        return 1
    except Exception as exc:  # noqa: BLE001 - explicit failure required
        sys.stderr.write('FAIL exception: %s\n' % exc)
        log_file.flush()
        _tail(log_path)
        return 1
    finally:
        if executor is not None:
            executor.shutdown()
        _cleanup(proc, checker)
        if fixture is not None:
            try:
                fixture.destroy_node()
            except Exception:
                pass
        if not log_file.closed:
            log_file.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
