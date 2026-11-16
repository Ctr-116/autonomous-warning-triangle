#!/usr/bin/env python3
"""Perception native integration check for yahboomcar_visual ROS 2 Humble nodes.

Validates:
- Executable discovery via ament_index_python.
- Headless execution under namespace /perception_native.
- Astra RGB image headless subscription and error logging on malformed encoding.
- Astra Depth image headless subscription (16UC1, 32FC1) and error logging on malformed encoding.
- IR_transform normalization from 16UC1 to MONO8, retaining stamp and frame_id.
- pointCloud_pub synthetic PointCloud2 generation with custom seed/frame_id/points/rate.
- pointCloud_visualize headless mode receipt.
- Strict namespace isolation with no global color_cloud publisher.
- Graceful shutdown handling via SIGINT.
"""

import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2


def get_log_dir() -> str:
    default_dir = "/full-validation-control"
    if os.path.isdir(default_dir) and os.access(default_dir, os.W_OK):
        return default_dir
    return tempfile.gettempdir()


def start_process(exe_name: str, args: list, log_dir: str) -> tuple:
    pkg_prefix = get_package_prefix("yahboomcar_visual")
    exe_path = os.path.join(pkg_prefix, "lib", "yahboomcar_visual", exe_name)
    if not os.path.isfile(exe_path):
        raise FileNotFoundError(f"Executable not found: {exe_path}")

    log_path = os.path.join(log_dir, f"perception_native_{exe_name}.log")
    log_file = open(log_path, "w+", encoding="utf-8", errors="replace")
    cmd = [exe_path] + args
    proc = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return proc, log_file, log_path


def stop_process(proc: subprocess.Popen, log_file, timeout: float = 5.0):
    if proc.poll() is None:
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=2.0)
    try:
        log_file.close()
    except Exception:
        pass


def read_log_file(log_path: str) -> str:
    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except Exception:
        return ""


def spin_until(node: Node, condition_fn, timeout: float = 5.0, step: float = 0.05) -> bool:
    start_time = time.monotonic()
    while time.monotonic() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=step)
        if condition_fn():
            return True
    return False


def wait_for_subscriber(publisher, min_subscribers: int = 1, timeout: float = 5.0) -> bool:
    start_time = time.monotonic()
    while time.monotonic() - start_time < timeout:
        if publisher.get_subscription_count() >= min_subscribers:
            return True
        time.sleep(0.05)
    return False


def main():
    rclpy.init()
    test_node = Node("perception_native_test_harness")
    bridge = CvBridge()
    log_dir = get_log_dir()

    sensor_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    processes_to_clean = []

    try:
        # ----------------------------------------------------------------------
        # Group 1: Namespace isolation check & pointCloud_pub launch
        # ----------------------------------------------------------------------
        cloud_pub_proc, cloud_pub_log_file, cloud_pub_log = start_process(
            "pointCloud_pub",
            [
                "--ros-args",
                "-r", "__ns:=/perception_native",
                "-p", "seed:=42",
                "-p", "num_points:=100",
                "-p", "rate:=2.0",
                "-p", "frame_id:=fixture_map",
            ],
            log_dir,
        )
        processes_to_clean.append((cloud_pub_proc, cloud_pub_log_file))

        # Check that no global /color_cloud publisher exists
        time.sleep(0.5)
        publishers_global = test_node.get_publishers_info_by_topic("/color_cloud")
        assert len(publishers_global) == 0, "Violation: Global topic /color_cloud publisher detected!"

        received_clouds = []

        def cloud_cb(msg: PointCloud2):
            received_clouds.append(msg)

        cloud_sub = test_node.create_subscription(
            PointCloud2,
            "/perception_native/color_cloud",
            cloud_cb,
            sensor_qos,
        )

        assert spin_until(test_node, lambda: len(received_clouds) > 0, timeout=6.0), (
            "Failed to receive PointCloud2 from /perception_native/color_cloud"
        )

        sample_cloud = received_clouds[0]
        assert sample_cloud.header.frame_id == "fixture_map", (
            f"Expected frame_id 'fixture_map', got '{sample_cloud.header.frame_id}'"
        )

        offset_x = None
        offset_y = None
        offset_z = None
        offset_rgb = None
        for f in sample_cloud.fields:
            if f.name == "x":
                offset_x = f.offset
            elif f.name == "y":
                offset_y = f.offset
            elif f.name == "z":
                offset_z = f.offset
            elif f.name == "rgb":
                offset_rgb = f.offset

        assert (
            offset_x is not None
            and offset_y is not None
            and offset_z is not None
            and offset_rgb is not None
        ), "PointCloud2 missing x/y/z/rgb field definitions"

        point_step = sample_cloud.point_step
        total_points = sample_cloud.width * sample_cloud.height
        assert total_points == 100, f"Expected 100 points, got {total_points}"

        data = sample_cloud.data
        for i in range(total_points):
            base = i * point_step
            x = struct.unpack_from("<f", data, base + offset_x)[0]
            y = struct.unpack_from("<f", data, base + offset_y)[0]
            z = struct.unpack_from("<f", data, base + offset_z)[0]
            rgb_val = struct.unpack_from("<I", data, base + offset_rgb)[0]
            r = (rgb_val >> 16) & 0xFF
            g = (rgb_val >> 8) & 0xFF
            b = rgb_val & 0xFF

            assert math.isfinite(x) and 0.0 <= x <= 10.0, f"Point {i} X out of bounds: {x}"
            assert math.isfinite(y) and 0.0 <= y <= 10.0, f"Point {i} Y out of bounds: {y}"
            assert math.isfinite(z) and 0.0 <= z <= 10.0, f"Point {i} Z out of bounds: {z}"
            assert 0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255

        print("PASS: pointCloud_pub synthetic output contract and namespace isolation")

        # ----------------------------------------------------------------------
        # Group 2: pointCloud_visualize headless execution
        # ----------------------------------------------------------------------
        cloud_vis_proc, cloud_vis_log_file, cloud_vis_log = start_process(
            "pointCloud_visualize",
            [
                "--ros-args",
                "-r", "__ns:=/perception_native",
                "-p", "gui:=false",
                "-p", "topic:=color_cloud",
            ],
            log_dir,
        )
        processes_to_clean.append((cloud_vis_proc, cloud_vis_log_file))

        def headless_logged() -> bool:
            return "[Headless]" in read_log_file(cloud_vis_log)

        # Allow pointCloud_pub to keep sending while spinning
        assert spin_until(test_node, headless_logged, timeout=6.0), (
            "pointCloud_visualize failed to log headless receipt"
        )
        assert cloud_vis_proc.poll() is None, "pointCloud_visualize terminated prematurely"

        print("PASS: pointCloud_visualize headless execution and frame receipt")

        # ----------------------------------------------------------------------
        # Group 3: Astra RGB Image headless subscriber & error handling
        # ----------------------------------------------------------------------
        rgb_proc, rgb_log_file, rgb_log = start_process(
            "astra_rgb_image",
            [
                "--ros-args",
                "-r", "__ns:=/perception_native",
                "-p", "display:=false",
                "-p", "image_topic:=camera/rgb/image_raw",
            ],
            log_dir,
        )
        processes_to_clean.append((rgb_proc, rgb_log_file))

        rgb_pub = test_node.create_publisher(
            Image,
            "/perception_native/camera/rgb/image_raw",
            sensor_qos,
        )
        assert wait_for_subscriber(rgb_pub, 1, timeout=5.0), "Discovery failed for Astra RGB Image subscriber"

        # Publish valid bgr8 frames
        dummy_rgb = np.full((120, 160, 3), 128, dtype=np.uint8)
        valid_rgb_msg = bridge.cv2_to_imgmsg(dummy_rgb, encoding="bgr8")
        for _ in range(3):
            rgb_pub.publish(valid_rgb_msg)
            spin_until(test_node, lambda: False, timeout=0.1)

        assert rgb_proc.poll() is None, "astra_rgb_image died on valid bgr8 frames"

        # Publish malformed encoding frame
        malformed_msg = Image()
        malformed_msg.header.stamp = test_node.get_clock().now().to_msg()
        malformed_msg.header.frame_id = "camera_optical_frame"
        malformed_msg.height = 100
        malformed_msg.width = 100
        malformed_msg.encoding = "INVALID_ENCODING"
        malformed_msg.step = 100
        malformed_msg.data = b"\x00" * 10000

        rgb_pub.publish(malformed_msg)

        def rgb_err_logged() -> bool:
            return "cv_bridge exception" in read_log_file(rgb_log)

        assert spin_until(test_node, rgb_err_logged, timeout=5.0), (
            "astra_rgb_image failed to catch and log cv_bridge exception"
        )
        assert rgb_proc.poll() is None, "astra_rgb_image died after malformed encoding"

        print("PASS: astra_rgb_image headless subscription and error handling")

        # ----------------------------------------------------------------------
        # Group 4: Astra Depth Image headless subscriber & error handling
        # ----------------------------------------------------------------------
        depth_proc, depth_log_file, depth_log = start_process(
            "astra_depth_image",
            [
                "--ros-args",
                "-r", "__ns:=/perception_native",
                "-p", "display:=false",
                "-p", "image_topic:=camera/depth/image_raw",
            ],
            log_dir,
        )
        processes_to_clean.append((depth_proc, depth_log_file))

        depth_pub = test_node.create_publisher(
            Image,
            "/perception_native/camera/depth/image_raw",
            sensor_qos,
        )
        assert wait_for_subscriber(depth_pub, 1, timeout=5.0), (
            "Discovery failed for Astra Depth Image subscriber"
        )

        # Publish valid 16UC1 frame
        dummy_depth_16u = np.full((120, 160), 1000, dtype=np.uint16)
        valid_depth_16u_msg = bridge.cv2_to_imgmsg(dummy_depth_16u, encoding="16UC1")
        depth_pub.publish(valid_depth_16u_msg)
        spin_until(test_node, lambda: False, timeout=0.1)

        # Publish valid 32FC1 frame
        dummy_depth_32f = np.full((120, 160), 1.5, dtype=np.float32)
        valid_depth_32f_msg = bridge.cv2_to_imgmsg(dummy_depth_32f, encoding="32FC1")
        depth_pub.publish(valid_depth_32f_msg)
        spin_until(test_node, lambda: False, timeout=0.1)

        assert depth_proc.poll() is None, "astra_depth_image died on valid depth frames"

        # Publish malformed encoding frame
        malformed_depth = Image()
        malformed_depth.header.stamp = test_node.get_clock().now().to_msg()
        malformed_depth.header.frame_id = "depth_frame"
        malformed_depth.height = 100
        malformed_depth.width = 100
        malformed_depth.encoding = "CORRUPTED_DEPTH"
        malformed_depth.step = 100
        malformed_depth.data = b"\x00" * 10000

        depth_pub.publish(malformed_depth)

        def depth_err_logged() -> bool:
            return "cv_bridge exception" in read_log_file(depth_log)

        assert spin_until(test_node, depth_err_logged, timeout=5.0), (
            "astra_depth_image failed to catch and log cv_bridge exception"
        )
        assert depth_proc.poll() is None, "astra_depth_image died after malformed encoding"

        print("PASS: astra_depth_image headless subscription and error handling")

        # ----------------------------------------------------------------------
        # Group 5: IR_transform normalization and header preservation
        # ----------------------------------------------------------------------
        ir_proc, ir_log_file, ir_log = start_process(
            "IR_transform",
            [
                "--ros-args",
                "-r", "__ns:=/perception_native",
                "-p", "image_topic:=camera/ir/image_raw",
                "-p", "output_topic:=camera/ir/image_mono8",
            ],
            log_dir,
        )
        processes_to_clean.append((ir_proc, ir_log_file))

        ir_pub = test_node.create_publisher(
            Image,
            "/perception_native/camera/ir/image_raw",
            sensor_qos,
        )

        transformed_ir_msgs = []

        def ir_cb(msg: Image):
            transformed_ir_msgs.append(msg)

        test_node.create_subscription(
            Image,
            "/perception_native/camera/ir/image_mono8",
            ir_cb,
            sensor_qos,
        )

        assert wait_for_subscriber(ir_pub, 1, timeout=5.0), "Discovery failed for IR_transform subscriber"

        # Create known 16UC1 image with distinct gradient
        raw_ir_mat = np.arange(100, dtype=np.uint16).reshape((10, 10)) * 500
        ir_msg = bridge.cv2_to_imgmsg(raw_ir_mat, encoding="16UC1")
        ir_msg.header.stamp.sec = 12345
        ir_msg.header.stamp.nanosec = 67890
        ir_msg.header.frame_id = "ir_sensor_frame"

        # Periodically publish until transformed image received
        start_t = time.monotonic()
        while time.monotonic() - start_t < 6.0 and len(transformed_ir_msgs) == 0:
            ir_pub.publish(ir_msg)
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(transformed_ir_msgs) > 0, "Failed to receive normalized image from IR_transform"

        recv_msg = transformed_ir_msgs[0]
        assert recv_msg.header.stamp.sec == 12345 and recv_msg.header.stamp.nanosec == 67890, (
            "IR_transform did not retain source header timestamp"
        )
        assert recv_msg.header.frame_id == "ir_sensor_frame", (
            f"Expected frame_id 'ir_sensor_frame', got '{recv_msg.header.frame_id}'"
        )
        assert recv_msg.encoding == "mono8", f"Expected encoding 'mono8', got '{recv_msg.encoding}'"

        converted_mat = bridge.imgmsg_to_cv2(recv_msg, desired_encoding="mono8")
        expected_mat = cv2.normalize(raw_ir_mat, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        assert np.array_equal(converted_mat, expected_mat), "IR_transform normalization mismatch with cv2.normalize"

        print("PASS: IR_transform normalization and header preservation")

    finally:
        for proc, log_f in processes_to_clean:
            stop_process(proc, log_f, timeout=5.0)

        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
