#!/usr/bin/env python3
"""
Integration fixture and validation check for yahboomcar_slam pointcloud_mapping.
Evaluates the native ROS2 Humble pointcloud_mapping executable against physical
projection and coordinate contract specifications.
"""

import os
import sys
import time
import json
import struct
import signal
import subprocess
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from ament_index_python.packages import get_package_prefix


def parse_pointcloud2(cloud_msg: PointCloud2) -> List[Dict[str, Any]]:
    points = []
    field_offsets = {}
    field_datatypes = {}
    for f in cloud_msg.fields:
        field_offsets[f.name] = f.offset
        field_datatypes[f.name] = f.datatype

    if 'x' not in field_offsets or 'y' not in field_offsets or 'z' not in field_offsets:
        return points

    x_off = field_offsets['x']
    y_off = field_offsets['y']
    z_off = field_offsets['z']
    
    has_rgba = 'rgba' in field_offsets
    has_rgb = 'rgb' in field_offsets
    rgb_off = field_offsets['rgba'] if has_rgba else (field_offsets['rgb'] if has_rgb else None)

    point_step = cloud_msg.point_step
    raw_data = cloud_msg.data

    total_points = cloud_msg.width * cloud_msg.height
    for i in range(total_points):
        offset = i * point_step
        if offset + 12 > len(raw_data):
            break
        x, y, z = struct.unpack_from('fff', raw_data, offset + x_off)
        r, g, b, a = 0, 0, 0, 255
        if rgb_off is not None and offset + rgb_off + 4 <= len(raw_data):
            # In PCL/ROS PointCloud2, rgb/rgba is stored as BGRA or RGBA packed bytes / float
            b_val, g_val, r_val, a_val = struct.unpack_from('BBBB', raw_data, offset + rgb_off)
            r, g, b, a = r_val, g_val, b_val, a_val

        points.append({
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'r': int(r),
            'g': int(g),
            'b': int(b),
            'a': int(a)
        })
    return points


class PerceptionMapperTestHarness(Node):
    def __init__(self, namespace: str = '/perception_mapper'):
        super().__init__('perception_mapper_check_fixture')
        self.ns = namespace

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        global_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        local_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.pub_color = self.create_publisher(Image, f'{self.ns}/color', sensor_qos)
        self.pub_depth = self.create_publisher(Image, f'{self.ns}/depth', sensor_qos)
        self.pub_pose = self.create_publisher(PoseStamped, f'{self.ns}/pose', sensor_qos)
        self.pub_info = self.create_publisher(CameraInfo, f'{self.ns}/info', sensor_qos)

        # Subscriptions
        self.sub_global = self.create_subscription(
            PointCloud2, f'{self.ns}/Global/PointCloudOutput', self._global_cb, global_sub_qos)
        self.sub_local = self.create_subscription(
            PointCloud2, f'{self.ns}/Local/PointCloudOutput', self._local_cb, local_sub_qos)

        # Client for clear_map
        self.cli_clear = self.create_client(Empty, f'{self.ns}/clear_map')

        self.last_global_cloud: Optional[PointCloud2] = None
        self.last_local_cloud: Optional[PointCloud2] = None
        self.global_received_count = 0
        self.local_received_count = 0

    def _global_cb(self, msg: PointCloud2):
        self.last_global_cloud = msg
        self.global_received_count += 1

    def _local_cb(self, msg: PointCloud2):
        self.last_local_cloud = msg
        self.local_received_count += 1

    def reset_reception(self):
        self.last_global_cloud = None
        self.last_local_cloud = None
        self.global_received_count = 0
        self.local_received_count = 0

    def publish_frame(
        self,
        stamp_sec: int,
        stamp_nanosec: int,
        depth_type: str = '16UC1',
        depth_val: float = 2.0,
        pose_trans: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        pose_quat: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        frame_id: str = 'map',
        color_bgr: Tuple[int, int, int] = (10, 20, 30),
        nan_depth: bool = False
    ):
        stamp = self.get_clock().now().to_msg()
        stamp.sec = stamp_sec
        stamp.nanosec = stamp_nanosec

        # 1. Color: 3x3 bgr8
        c_msg = Image()
        c_msg.header.stamp = stamp
        c_msg.header.frame_id = 'camera'
        c_msg.height = 3
        c_msg.width = 3
        c_msg.encoding = 'bgr8'
        c_msg.is_bigendian = 0
        c_msg.step = 3 * 3
        b, g, r = color_bgr
        c_data = bytearray()
        for _ in range(9):
            c_data.extend([b, g, r])
        c_msg.data = bytes(c_data)

        # 2. Depth: 3x3 16UC1 or 32FC1
        d_msg = Image()
        d_msg.header.stamp = stamp
        d_msg.header.frame_id = 'camera'
        d_msg.height = 3
        d_msg.width = 3
        d_msg.is_bigendian = 0

        if depth_type == '16UC1':
            d_msg.encoding = '16UC1'
            d_msg.step = 3 * 2
            val_u16 = int(round(depth_val * 1000.0))  # 2.0m -> 2000mm
            d_data = bytearray()
            for _ in range(9):
                d_data.extend(struct.pack('<H', val_u16))
            d_msg.data = bytes(d_data)
        else:
            d_msg.encoding = '32FC1'
            d_msg.step = 3 * 4
            val_f32 = float('nan') if nan_depth else float(depth_val)
            d_data = bytearray()
            for _ in range(9):
                d_data.extend(struct.pack('<f', val_f32))
            d_msg.data = bytes(d_data)

        # 3. CameraInfo: K=[100, 0, 1; 0, 100, 1; 0, 0, 1]
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = 'camera'
        info_msg.height = 3
        info_msg.width = 3
        info_msg.k = [100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]

        # 4. PoseStamped
        p_msg = PoseStamped()
        p_msg.header.stamp = stamp
        p_msg.header.frame_id = frame_id
        p_msg.pose.position.x = float(pose_trans[0])
        p_msg.pose.position.y = float(pose_trans[1])
        p_msg.pose.position.z = float(pose_trans[2])
        p_msg.pose.orientation.x = float(pose_quat[0])
        p_msg.pose.orientation.y = float(pose_quat[1])
        p_msg.pose.orientation.z = float(pose_quat[2])
        p_msg.pose.orientation.w = float(pose_quat[3])

        # Publish in synchrony
        self.pub_color.publish(c_msg)
        self.pub_depth.publish(d_msg)
        self.pub_info.publish(info_msg)
        self.pub_pose.publish(p_msg)


def wait_for_cloud(
    fixture: PerceptionMapperTestHarness,
    executor: SingleThreadedExecutor,
    timeout_sec: float = 5.0,
    min_count: int = 1
) -> bool:
    start_t = time.time()
    while time.time() - start_t < timeout_sec:
        executor.spin_once(timeout_sec=0.05)
        if fixture.global_received_count >= min_count:
            return True
    return False


def start_native_process(
    input_pose_is_twc: bool,
    log_file_handle
) -> subprocess.Popen:
    try:
        pkg_prefix = get_package_prefix('yahboomcar_slam')
    except Exception:
        pkg_prefix = '/opt/ros/humble'

    exec_path = Path(pkg_prefix) / 'lib' / 'yahboomcar_slam' / 'pointcloud_mapping'
    if not exec_path.exists():
        # Fallback check standard install or local ws
        cand = Path('/work/x3plus_ros2_full_ws/install/yahboomcar_slam/lib/yahboomcar_slam/pointcloud_mapping')
        if cand.exists():
            exec_path = cand

    cmd = [
        str(exec_path),
        '--ros-args',
        '-r', '__ns:=/perception_mapper',
        '-p', 'topicColor:=color',
        '-p', 'topicDepth:=depth',
        '-p', 'topicTcw:=pose',
        '-p', 'topicCameraInfo:=info',
        '-p', 'pixel_step:=1',
        '-p', 'resolution:=0.001',
        '-p', 'fx:=100.0',
        '-p', 'fy:=100.0',
        '-p', 'cx:=1.0',
        '-p', 'cy:=1.0',
        '-p', f'input_pose_is_twc:={"true" if input_pose_is_twc else "false"}'
    ]

    proc = subprocess.Popen(
        cmd,
        stdout=log_file_handle,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid if hasattr(os, 'setsid') else None
    )
    return proc


def stop_process(proc: subprocess.Popen):
    if proc.poll() is None:
        try:
            if hasattr(os, 'killpg') and hasattr(os, 'getpgid'):
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            else:
                proc.terminate()
            proc.wait(timeout=3.0)
        except Exception:
            try:
                if hasattr(os, 'killpg') and hasattr(os, 'getpgid'):
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                else:
                    proc.kill()
                proc.wait(timeout=2.0)
            except Exception:
                pass


def run_checks() -> int:
    log_dir = Path('/full-validation-control')
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        log_dir = Path('./full-validation-control')
        log_dir.mkdir(parents=True, exist_ok=True)
    log_file_path = log_dir / 'perception_mapper_native.log'

    log_fh = open(log_file_path, 'w', buffering=1)

    rclpy.init()
    results: List[Dict[str, Any]] = []

    def record_check(name: str, passed: bool, evidence: Dict[str, Any], message: str):
        record = {
            'check': name,
            'passed': passed,
            'message': message,
            'evidence': evidence
        }
        results.append(record)
        status_str = "PASS" if passed else "FAIL"
        print(f"[{status_str}] {name}: {message}", file=sys.stderr)

    # -------------------------------------------------------------------------
    # Phase 1: Test input_pose_is_twc = False (Default Tcw: world->camera)
    # Physical convention:
    #   p_cam = Tcw * p_world => p_world = Tcw^-1 * p_cam.
    #   Given Tcw translation = (+1, 0, 0) and Identity rotation:
    #   p_cam = p_world + [1, 0, 0]^T => p_world = p_cam - [1, 0, 0]^T.
    #   Projected camera points:
    #     depth Z=2.0, fx=fy=100, cx=cy=1.
    #     n in [0, 1, 2], m in [0, 1, 2]:
    #     n=0: x_cam = (0 - 1)*2/100 = -0.02 -> x_world = -1.02
    #     n=1: x_cam = (1 - 1)*2/100 = 0.00  -> x_world = -1.00
    #     n=2: x_cam = (2 - 1)*2/100 = +0.02 -> x_world = -0.98
    #   Therefore, worldpoint.x MUST be in range [-1.02, -0.98].
    #   If an implementation suffers from double inversion, x_world would be +1.00.
    # -------------------------------------------------------------------------
    proc_tcw = start_native_process(input_pose_is_twc=False, log_file_handle=log_fh)
    time.sleep(1.5)  # Let node initialize publishers/subscribers

    fixture = PerceptionMapperTestHarness(namespace='/perception_mapper')
    executor = SingleThreadedExecutor()
    executor.add_node(fixture)

    coords_16u: List[Tuple[float, float, float]] = []

    try:
        # Check 1: 16UC1 depth projection, color BGR(10, 20, 30) -> RGB(30, 20, 10), 9 points, geometry
        fixture.reset_reception()
        # Non-zero timestamp
        for i in range(3):
            fixture.publish_frame(
                stamp_sec=100, stamp_nanosec=100000000 + i * 20000000,
                depth_type='16UC1', depth_val=2.0,
                pose_trans=(1.0, 0.0, 0.0), pose_quat=(0.0, 0.0, 0.0, 1.0),
                color_bgr=(10, 20, 30)
            )
            time.sleep(0.05)

        received = wait_for_cloud(fixture, executor, timeout_sec=5.0)
        if not received or fixture.last_global_cloud is None:
            record_check('tcw_16uc1_reception', False, {}, 'Timeout waiting for Global PointCloud2 with 16UC1 input')
        else:
            pts = parse_pointcloud2(fixture.last_global_cloud)
            raw_sample = pts[:5] if pts else []
            num_pts = len(pts)
            all_finite = all(np.isfinite(p['x']) and np.isfinite(p['y']) and np.isfinite(p['z']) for p in pts)
            
            # Color assertion: input BGR(10, 20, 30) -> Red=30, Green=20, Blue=10
            correct_colors = [p for p in pts if p['r'] == 30 and p['g'] == 20 and p['b'] == 10]
            
            # Check Z ~ 2.0m (with identity rotation and z_trans=0)
            z_valid = all(abs(p['z'] - 2.0) < 0.05 for p in pts)

            # Check unique 9 points (resolution=0.001 keeps all 9 pixels at 0.02m spacing)
            unique_pts = len(set((round(p['x'], 3), round(p['y'], 3), round(p['z'], 3)) for p in pts))

            # Check physical translation contract: Tcw x=+1 implies worldpoint.x = x_cam - 1
            # Expected center x around -1.0, NOT +1.0
            x_vals = [p['x'] for p in pts]
            mean_x = float(np.mean(x_vals)) if x_vals else 0.0
            expected_x_center = -1.0
            double_inversion_detected = (mean_x > 0.5)

            pass_geom = (
                num_pts >= 9 and unique_pts >= 9 and all_finite and z_valid and
                len(correct_colors) == num_pts and not double_inversion_detected and
                abs(mean_x - expected_x_center) < 0.05
            )

            coords_16u = [(p['x'], p['y'], p['z']) for p in pts]

            record_check(
                'tcw_16uc1_geometry_and_color',
                pass_geom,
                {
                    'num_points': num_pts,
                    'unique_points': unique_pts,
                    'mean_x': mean_x,
                    'expected_mean_x': expected_x_center,
                    'double_inversion_detected': double_inversion_detected,
                    'z_valid': z_valid,
                    'colors_matched': len(correct_colors),
                    'sample_observed_pts': raw_sample
                },
                'Double inversion detected: world x positive instead of negative' if double_inversion_detected
                else ('Geometry, colors and coordinate contract verified' if pass_geom else 'Failed geometry/color verification')
            )

        # Check 2: 32FC1 depth = 2.0m produces identical coordinates
        # Reset map first to isolate or compare directly
        fixture.reset_reception()
        for i in range(3):
            fixture.publish_frame(
                stamp_sec=101, stamp_nanosec=100000000 + i * 20000000,
                depth_type='32FC1', depth_val=2.0,
                pose_trans=(1.0, 0.0, 0.0), pose_quat=(0.0, 0.0, 0.0, 1.0),
                color_bgr=(10, 20, 30)
            )
            time.sleep(0.05)

        received_32f = wait_for_cloud(fixture, executor, timeout_sec=5.0)
        if not received_32f or fixture.last_global_cloud is None:
            record_check('tcw_32fc1_comparison', False, {}, 'Timeout waiting for Global PointCloud2 with 32FC1')
        else:
            pts_32f = parse_pointcloud2(fixture.last_global_cloud)
            # Find points with center x close to 16UC1 coords
            coords_32f = [(p['x'], p['y'], p['z']) for p in pts_32f]
            # Since global map accumulates without clear, points should match grid exactly
            coords_16u_set = set((round(x, 3), round(y, 3), round(z, 3)) for x, y, z in coords_16u)
            coords_32f_set = set((round(x, 3), round(y, 3), round(z, 3)) for x, y, z in coords_32f)
            matching = coords_16u_set.issubset(coords_32f_set)
            record_check(
                'tcw_16uc1_vs_32fc1_equivalence',
                matching,
                {
                    'set_16u_size': len(coords_16u_set),
                    'set_32f_size': len(coords_32f_set),
                    'coords_16u_sample': list(coords_16u_set)[:3],
                    'coords_32f_sample': list(coords_32f_set)[:3]
                },
                '16UC1 and 32FC1 yield identical metric pointcloud coordinates' if matching else 'Mismatch between 16UC1 and 32FC1 coordinates'
            )

        # Check 3: Clear map service (std_srvs/Empty clear_map)
        if not fixture.cli_clear.wait_for_service(timeout_sec=3.0):
            record_check('clear_map_service_available', False, {}, 'Service clear_map not available')
        else:
            fixture.reset_reception()
            req = Empty.Request()
            future = fixture.cli_clear.call_async(req)
            call_start = time.time()
            while time.time() - call_start < 3.0 and not future.done():
                executor.spin_once(timeout_sec=0.05)

            if not future.done():
                record_check('clear_map_service_call', False, {}, 'Service clear_map call timed out')
            else:
                # After clearing, verify observable global map clearing or fresh replacement
                # In current PCLMapper implementation: resetMap() clears global_map_ and resets last_keyframe_size_,
                # but does NOT publish an empty PointCloud2 message upon service request.
                # Per task requirements: "expect sourcemayneedfix, failifnoobservableclear"
                time.sleep(0.5)
                executor.spin_once(timeout_sec=0.1)

                observable_clear = (
                    fixture.last_global_cloud is not None and
                    (fixture.last_global_cloud.width * fixture.last_global_cloud.height == 0)
                )

                record_check(
                    'clear_map_observable_empty_publish',
                    observable_clear,
                    {
                        'global_received_after_clear': fixture.global_received_count,
                        'observed_cloud': bool(fixture.last_global_cloud)
                    },
                    'Observable clear verified: empty pointcloud published on clear_map' if observable_clear
                    else 'FAIL: No observable clear published immediately after clear_map service call'
                )

                # Now send a fresh pointcloud and verify it starts clean (9 points instead of accumulated)
                fixture.reset_reception()
                for i in range(1):
                    fixture.publish_frame(
                        stamp_sec=105, stamp_nanosec=100000000 + i * 20000000,
                        depth_type='32FC1', depth_val=2.0,
                        pose_trans=(1.0, 0.0, 0.0), pose_quat=(0.0, 0.0, 0.0, 1.0),
                        color_bgr=(10, 20, 30)
                    )
                    time.sleep(0.05)

                wait_for_cloud(fixture, executor, timeout_sec=5.0)
                fresh_pts = parse_pointcloud2(fixture.last_global_cloud) if fixture.last_global_cloud else []
                fresh_clean = (len(fresh_pts) == 9)
                record_check(
                    'fresh_pointcloud_after_clear',
                    fresh_clean,
                    {'fresh_points_count': len(fresh_pts)},
                    'Fresh pointcloud has exactly 9 points after reset' if fresh_clean
                    else f'Accumulation dirty: expected 9 points, found {len(fresh_pts)}'
                )

        # Check 4: Robustness - all-NaN depth does not add points
        count_before = len(parse_pointcloud2(fixture.last_global_cloud)) if fixture.last_global_cloud else 0
        fixture.reset_reception()
        for i in range(3):
            fixture.publish_frame(
                stamp_sec=110, stamp_nanosec=100000000 + i * 20000000,
                depth_type='32FC1', depth_val=0.0, nan_depth=True,
                pose_trans=(1.0, 0.0, 0.0), pose_quat=(0.0, 0.0, 0.0, 1.0)
            )
            time.sleep(0.05)
        # Spin to see if anything gets processed
        time.sleep(0.5)
        executor.spin_once(timeout_sec=0.1)
        count_after = len(parse_pointcloud2(fixture.last_global_cloud)) if fixture.last_global_cloud else count_before
        nan_rejected = (count_after == count_before)
        record_check(
            'all_nan_depth_rejection',
            nan_rejected,
            {'count_before': count_before, 'count_after': count_after},
            'All-NaN depth rejected without corrupting map' if nan_rejected else 'All-NaN depth improperly updated map'
        )

        # Check 5: Robustness - zero quaternion or non-finite pose no accepted map update
        count_before_pose = count_after
        fixture.reset_reception()
        for i in range(3):
            # Zero quaternion: (0, 0, 0, 0)
            fixture.publish_frame(
                stamp_sec=115, stamp_nanosec=100000000 + i * 20000000,
                depth_type='32FC1', depth_val=2.0,
                pose_trans=(1.0, 0.0, 0.0), pose_quat=(0.0, 0.0, 0.0, 0.0)
            )
            # Non-finite pose: NaN position
            fixture.publish_frame(
                stamp_sec=116, stamp_nanosec=100000000 + i * 20000000,
                depth_type='32FC1', depth_val=2.0,
                pose_trans=(float('nan'), 0.0, 0.0), pose_quat=(0.0, 0.0, 0.0, 1.0)
            )
            time.sleep(0.05)
        time.sleep(0.5)
        executor.spin_once(timeout_sec=0.1)
        count_after_pose = len(parse_pointcloud2(fixture.last_global_cloud)) if fixture.last_global_cloud else count_before_pose
        invalid_pose_rejected = (count_after_pose == count_before_pose)
        record_check(
            'invalid_pose_rejection',
            invalid_pose_rejected,
            {'count_before': count_before_pose, 'count_after': count_after_pose},
            'Zero quaternion and non-finite poses rejected' if invalid_pose_rejected else 'Invalid pose accepted into map'
        )

    finally:
        stop_process(proc_tcw)
        executor.remove_node(fixture)

    # -------------------------------------------------------------------------
    # Phase 2: Test input_pose_is_twc = True (Pose is Twc: camera->world)
    # Physical convention:
    #   p_world = Twc * p_cam.
    #   Given Twc translation = (+1, 0, 0):
    #   p_world = p_cam + [1, 0, 0]^T.
    #   Projected camera points: x_cam around 0.
    #   Therefore, worldpoint.x MUST be in range [+0.98, +1.02].
    # -------------------------------------------------------------------------
    proc_twc = start_native_process(input_pose_is_twc=True, log_file_handle=log_fh)
    time.sleep(1.5)

    executor.add_node(fixture)
    try:
        fixture.reset_reception()
        for i in range(3):
            fixture.publish_frame(
                stamp_sec=200, stamp_nanosec=100000000 + i * 20000000,
                depth_type='16UC1', depth_val=2.0,
                pose_trans=(1.0, 0.0, 0.0), pose_quat=(0.0, 0.0, 0.0, 1.0),
                color_bgr=(10, 20, 30)
            )
            time.sleep(0.05)

        received_twc = wait_for_cloud(fixture, executor, timeout_sec=5.0)
        if not received_twc or fixture.last_global_cloud is None:
            record_check('twc_mode_reception', False, {}, 'Timeout waiting for Global PointCloud2 under input_pose_is_twc=True')
        else:
            pts = parse_pointcloud2(fixture.last_global_cloud)
            raw_sample = pts[:5] if pts else []
            x_vals = [p['x'] for p in pts]
            mean_x = float(np.mean(x_vals)) if x_vals else 0.0
            expected_x_center = +1.0
            
            # Under true Twc +x translation, world +x should be centered at +1.0
            pass_twc = abs(mean_x - expected_x_center) < 0.05
            record_check(
                'twc_mode_translation_direction',
                pass_twc,
                {
                    'mean_x': mean_x,
                    'expected_mean_x': expected_x_center,
                    'sample_pts': raw_sample
                },
                'Twc mode correctly transforms +x translation to world +x (+1.0)' if pass_twc
                else f'Twc mode translation inverted or erroneous: observed mean_x={mean_x}'
            )

    finally:
        stop_process(proc_twc)
        executor.remove_node(fixture)
        fixture.destroy_node()
        rclpy.shutdown()
        log_fh.close()

    # Summary and Exit
    summary_report = {
        'total_checks': len(results),
        'passed_checks': sum(1 for r in results if r['passed']),
        'failed_checks': sum(1 for r in results if not r['passed']),
        'checks': results
    }

    print("\n--- FINAL TEST RESULTS JSON ---")
    print(json.dumps(summary_report, indent=2))

    failures = [r for r in results if not r['passed']]
    if failures:
        print(f"\nCompleted with {len(failures)} failures.", file=sys.stderr)
        return 1
    print("\nAll checks passed successfully.", file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(run_checks())
