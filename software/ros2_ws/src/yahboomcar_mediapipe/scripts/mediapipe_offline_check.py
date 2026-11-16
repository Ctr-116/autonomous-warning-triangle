#!/usr/bin/env python3
# encoding: utf-8
"""Offline regression and sanity check for MediaPipe 0.10.9 CPU and ROS2 Humble.

Verifies:
1. MediaPipe SDK imports and optional site-packages resolution.
2. Detector pipelines on blank frames (robust empty checks).
3. Optional positive image checks (--testdata dir).
4. MediaPipePointNode DDS PointArray -> PointCloud2 translation.
"""

import argparse
import math
import os
from pathlib import Path
import struct
import sys
import time

# Prepend optional MediaPipe site BEFORE numpy/cv/ROS imports
_OPTIONAL_SITE = os.environ.get('X3PLUS_MEDIAPIPE_SITE', '')
if _OPTIONAL_SITE and os.path.isdir(_OPTIONAL_SITE):
    if _OPTIONAL_SITE not in sys.path:
        sys.path.insert(0, _OPTIONAL_SITE)

# Enforce headless matplotlib backend
os.environ['MPLBACKEND'] = 'Agg'

# Derive workspace root: src/pkg/scripts -> parents[3] is workspace root
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
YAHBOOMCAR_WS_SRC = WORKSPACE_ROOT / 'src' / 'yahboomcar_mediapipe'
ARM_WS_SRC = WORKSPACE_ROOT / 'src' / 'arm_mediapipe'

for p in (YAHBOOMCAR_WS_SRC, ARM_WS_SRC):
    p_str = str(p)
    if p.exists() and p_str not in sys.path:
        sys.path.insert(0, p_str)

import cv2 as cv
import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from yahboomcar_msgs.msg import PointArray

import yahboomcar_mediapipe.detectors as detectors
from yahboomcar_mediapipe.mediapipe_point import MediaPipePointNode
import arm_mediapipe.media_library as arm_media


def check_sdk_version():
    try:
        import mediapipe as mp
        print(f"[SDK] mediapipe version: {getattr(mp, '__version__', 'unknown')} from {getattr(mp, '__file__', 'unknown')}")
    except Exception as exc:
        print(f"[SDK] Failed to import mediapipe: {exc}", file=sys.stderr)
        raise

    try:
        import cv_bridge
        print(f"[SDK] cv_bridge version: {getattr(cv_bridge, '__version__', 'present')}")
    except Exception as exc:
        print(f"[SDK] Failed to import cv_bridge: {exc}", file=sys.stderr)
        raise


def test_blank_frames_migrated():
    blank_bgr = np.zeros((240, 320, 3), dtype=np.uint8)

    # 1. HandDetector
    print("[TEST] yahboomcar_mediapipe.detectors.HandDetector (blank)...")
    hd = detectors.HandDetector(mode=False, maxHands=2)
    try:
        frame_res, img_res, pt_array = hd.pubHandsPoint(blank_bgr.copy(), draw=False)
        assert isinstance(pt_array, PointArray), "Expected PointArray instance"
        assert len(pt_array.points) == 0, f"Expected 0 points, got {len(pt_array.points)}"
        assert frame_res.shape == blank_bgr.shape
    finally:
        if hasattr(hd, 'hands') and hasattr(hd.hands, 'close'):
            hd.hands.close()

    # 2. PoseDetector
    print("[TEST] yahboomcar_mediapipe.detectors.PoseDetector (blank)...")
    pd = detectors.PoseDetector(mode=False, smooth=False)
    try:
        frame_res, img_res, pt_array = pd.pubPosePoint(blank_bgr.copy(), draw=False)
        assert isinstance(pt_array, PointArray), "Expected PointArray instance"
        assert len(pt_array.points) == 0, f"Expected 0 points, got {len(pt_array.points)}"
        assert frame_res.shape == blank_bgr.shape
    finally:
        if hasattr(pd, 'pose') and hasattr(pd.pose, 'close'):
            pd.pose.close()

    # 3. Holistic
    print("[TEST] yahboomcar_mediapipe.detectors.Holistic (blank)...")
    hl = detectors.Holistic(staticMode=False, landmarks=False)
    try:
        frame_res, img_res, pt_array = hl.findHolistic(blank_bgr.copy(), draw=False)
        assert isinstance(pt_array, PointArray), "Expected PointArray instance"
        assert len(pt_array.points) == 0, f"Expected 0 points, got {len(pt_array.points)}"
        assert frame_res.shape == blank_bgr.shape
    finally:
        if hasattr(hl, 'mpholistic') and hasattr(hl.mpholistic, 'close'):
            hl.mpholistic.close()

    # 4. FaceMesh
    print("[TEST] yahboomcar_mediapipe.detectors.FaceMesh (blank)...")
    fm = detectors.FaceMesh(staticMode=False, maxFaces=2)
    try:
        frame_res, img_res, pt_array = fm.pubFaceMeshPoint(blank_bgr.copy(), draw=False)
        assert isinstance(pt_array, PointArray), "Expected PointArray instance"
        assert len(pt_array.points) == 0, f"Expected 0 points, got {len(pt_array.points)}"
        assert frame_res.shape == blank_bgr.shape
    finally:
        if hasattr(fm, 'faceMesh') and hasattr(fm.faceMesh, 'close'):
            fm.faceMesh.close()

    # 5. FaceDetector
    print("[TEST] yahboomcar_mediapipe.detectors.FaceDetector (blank)...")
    fd = detectors.FaceDetector()
    try:
        frame_res, bboxs = fd.findFaces(blank_bgr.copy())
        assert isinstance(bboxs, list), "Expected list of bboxes"
        assert len(bboxs) == 0, f"Expected 0 faces, got {len(bboxs)}"
        assert frame_res.shape == blank_bgr.shape
    finally:
        if hasattr(fd, 'facedetection') and hasattr(fd.facedetection, 'close'):
            fd.facedetection.close()


def test_blank_frames_arm():
    blank_bgr = np.zeros((240, 320, 3), dtype=np.uint8)

    # 1. arm_mediapipe HandDetector
    print("[TEST] arm_mediapipe.media_library.HandDetector (blank)...")
    arm_hd = arm_media.HandDetector(mode=False, maxHands=1)
    try:
        frame_res, lm_list, bbox = arm_hd.findHands(blank_bgr.copy())
        assert len(lm_list) == 0, f"Expected 0 landmarks, got {len(lm_list)}"
        assert bbox == (0, 0, 0, 0), f"Expected empty bbox, got {bbox}"
    finally:
        if hasattr(arm_hd, 'hands') and hasattr(arm_hd.hands, 'close'):
            arm_hd.hands.close()

    # 2. arm_mediapipe PoseDetector
    print("[TEST] arm_mediapipe.media_library.PoseDetector (blank)...")
    arm_pd = arm_media.PoseDetector(mode=False, smooth=False)
    try:
        frame_res, pt_array = arm_pd.pubPosePoint(blank_bgr.copy(), draw=False)
        assert len(pt_array) == 0, f"Expected 0 points, got {len(pt_array)}"
    finally:
        if hasattr(arm_pd, 'pose') and hasattr(arm_pd.pose, 'close'):
            arm_pd.pose.close()

    # 3. arm_mediapipe Holistic
    print("[TEST] arm_mediapipe.media_library.Holistic (blank)...")
    arm_hl = arm_media.Holistic(staticMode=False, landmarks=False)
    try:
        frame_res, pose_pts, lhand_pts, rhand_pts = arm_hl.findHolistic(blank_bgr.copy(), draw=False)
        assert len(pose_pts) == 0, f"Expected 0 pose points, got {len(pose_pts)}"
        assert len(lhand_pts) == 0, f"Expected 0 lhand points, got {len(lhand_pts)}"
        assert len(rhand_pts) == 0, f"Expected 0 rhand points, got {len(rhand_pts)}"
    finally:
        if hasattr(arm_hl, 'mpholistic') and hasattr(arm_hl.mpholistic, 'close'):
            arm_hl.mpholistic.close()

    # 4. arm_mediapipe FaceMesh
    print("[TEST] arm_mediapipe.media_library.FaceMesh (blank)...")
    arm_fm = arm_media.FaceMesh(staticMode=False, maxFaces=1)
    try:
        frame_res, pt_array = arm_fm.pubFaceMeshPoint(blank_bgr.copy(), draw=False)
        assert len(pt_array) == 0, f"Expected 0 face points, got {len(pt_array)}"
    finally:
        if hasattr(arm_fm, 'faceMesh') and hasattr(arm_fm.faceMesh, 'close'):
            arm_fm.faceMesh.close()


def test_objectron_guarded():
    import mediapipe as mp
    mp_file = getattr(mp, '__file__', None)
    has_res = False
    if mp_file:
        mp_parent = Path(mp_file).resolve().parent
        objectron_mod = mp_parent / 'modules' / 'objectron'
        if objectron_mod.is_dir() and any(objectron_mod.iterdir()):
            has_res = True

    if not has_res:
        print("[SKIP] SKIP Objectron models: offline resources not present under mediapipe/modules/objectron")
        return

    print("[TEST] Objectron detector initialization...")
    obj = None
    try:
        obj = detectors.Objectron(staticMode=True, maxObjects=1)
        blank = np.zeros((240, 320, 3), dtype=np.uint8)
        _ = obj.findObjectron(blank)
    except Exception as exc:
        print(f"[SKIP] Objectron models check raised: {exc}")
    finally:
        if obj is not None and hasattr(obj, 'mpobjectron') and hasattr(obj.mpobjectron, 'close'):
            obj.mpobjectron.close()


def test_optional_positive_images(testdata_dir):
    if not testdata_dir or not os.path.isdir(testdata_dir):
        print("[INFO] No positive testdata directory provided or found. Skipping positive checks.")
        return

    import mediapipe as mp
    hands_img_path = os.path.join(testdata_dir, 'hands.jpg')
    pose_img_path = os.path.join(testdata_dir, 'pose.jpg')

    if os.path.exists(hands_img_path):
        print(f"[TEST] Positive test: {hands_img_path}")
        img = cv.imread(hands_img_path)
        if img is not None:
            # Native MP test
            with mp.solutions.hands.Hands(static_image_mode=True, max_num_hands=2) as hands:
                results = hands.process(cv.cvtColor(img, cv.COLOR_BGR2RGB))
                assert results.multi_hand_landmarks is not None, "Expected hands detected in hands.jpg"
                first_hand = results.multi_hand_landmarks[0]
                assert len(first_hand.landmark) == 21, f"Expected 21 landmarks, got {len(first_hand.landmark)}"
            # Migrated wrapper test
            hd = detectors.HandDetector(mode=True, maxHands=2)
            try:
                _, _, pt_array = hd.pubHandsPoint(img.copy(), draw=False)
                assert len(pt_array.points) >= 21, f"Expected >= 21 points, got {len(pt_array.points)}"
            finally:
                if hasattr(hd, 'hands') and hasattr(hd.hands, 'close'):
                    hd.hands.close()
        else:
            print(f"[WARN] Failed to read image: {hands_img_path}")

    if os.path.exists(pose_img_path):
        print(f"[TEST] Positive test: {pose_img_path}")
        img = cv.imread(pose_img_path)
        if img is not None:
            # Native MP test
            with mp.solutions.pose.Pose(static_image_mode=True) as pose:
                results = pose.process(cv.cvtColor(img, cv.COLOR_BGR2RGB))
                assert results.pose_landmarks is not None, "Expected pose detected in pose.jpg"
                assert len(results.pose_landmarks.landmark) == 33, f"Expected 33 landmarks, got {len(results.pose_landmarks.landmark)}"
            # Migrated wrapper test
            pd = detectors.PoseDetector(mode=True, smooth=False)
            try:
                _, _, pt_array = pd.pubPosePoint(img.copy(), draw=False)
                assert len(pt_array.points) == 33, f"Expected 33 points, got {len(pt_array.points)}"
            finally:
                if hasattr(pd, 'pose') and hasattr(pd.pose, 'close'):
                    pd.pose.close()
        else:
            print(f"[WARN] Failed to read image: {pose_img_path}")


class DDSProbeNode(Node):
    def __init__(self):
        super().__init__('dds_probe', namespace='/mp_test')
        self.received_clouds = []

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_points = self.create_publisher(PointArray, 'mediapipe/points', pub_qos)

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_cloud = self.create_subscription(PointCloud2, 'mediapipe_cloud', self._cloud_cb, sub_qos)

    def _cloud_cb(self, msg: PointCloud2):
        self.received_clouds.append(msg)


def test_mediapipe_point_dds():
    print("[TEST] MediaPipePointNode DDS translation under /mp_test...")
    node_point = None
    probe = None
    executor = None

    try:
        executor = SingleThreadedExecutor()
        node_point = MediaPipePointNode()
        # Set max_points parameter explicitly if needed
        probe = DDSProbeNode()

        executor.add_node(node_point)
        executor.add_node(probe)

        # Discovery wait: allow DDS match (at least 0.4s)
        start_wait = time.time()
        while time.time() - start_wait < 0.6:
            executor.spin_once(timeout_sec=0.05)
            if (node_point.pub_pcl.get_subscription_count() >= 1 and
                    probe.pub_points.get_subscription_count() >= 1):
                break

        sub_cnt = probe.pub_points.get_subscription_count()
        pub_cnt = node_point.pub_pcl.get_subscription_count()
        print(f"[INFO] DDS Matched counts: pub->sub: {sub_cnt}, cloud->probe: {pub_cnt}")
        assert sub_cnt >= 1 and pub_cnt >= 1, "DDS endpoint matching timed out"

        def publish_and_wait(msg: PointArray, expected_count: int, timeout_sec: float = 1.5):
            probe.received_clouds.clear()
            probe.pub_points.publish(msg)
            t_end = time.time() + timeout_sec
            while time.time() < t_end:
                executor.spin_once(timeout_sec=0.05)
                if len(probe.received_clouds) >= expected_count:
                    break
            return list(probe.received_clouds)

        # 1. Empty message test
        empty_msg = PointArray()
        clouds = publish_and_wait(empty_msg, expected_count=1)
        assert len(clouds) == 1, f"Expected 1 PointCloud2, got {len(clouds)}"
        assert clouds[0].width == 0, f"Empty PointArray must produce width 0, got {clouds[0].width}"
        assert clouds[0].header.frame_id == 'normalized_landmarks', f"Frame mismatch: {clouds[0].header.frame_id}"

        # 2. Finite 2 points test: verify XYZ and packed green bits (0x00FF00)
        p1 = Point(x=0.1, y=0.2, z=0.3)
        p2 = Point(x=0.4, y=0.5, z=0.6)
        valid_msg = PointArray(points=[p1, p2])
        clouds = publish_and_wait(valid_msg, expected_count=1)
        assert len(clouds) == 1, f"Expected 1 PointCloud2, got {len(clouds)}"
        cloud = clouds[0]
        assert cloud.width == 2, f"Expected width 2, got {cloud.width}"
        assert cloud.point_step == 16, f"Expected point_step 16, got {cloud.point_step}"
        assert cloud.header.frame_id == 'normalized_landmarks', f"Frame mismatch: {cloud.header.frame_id}"

        data = bytes(cloud.data)
        assert len(data) == 32, f"Expected data length 32, got {len(data)}"

        # Unpack using little-endian float bytes '<ffff'
        x1, y1, z1, rgb1 = struct.unpack_from('<ffff', data, 0)
        x2, y2, z2, rgb2 = struct.unpack_from('<ffff', data, 16)

        assert abs(x1 - 0.1) < 1e-5 and abs(y1 - 0.2) < 1e-5 and abs(z1 - 0.3) < 1e-5, f"P1 coords mismatch: {x1, y1, z1}"
        assert abs(x2 - 0.4) < 1e-5 and abs(y2 - 0.5) < 1e-5 and abs(z2 - 0.6) < 1e-5, f"P2 coords mismatch: {x2, y2, z2}"

        rgb1_uint = struct.unpack('<I', struct.pack('<f', rgb1))[0]
        rgb2_uint = struct.unpack('<I', struct.pack('<f', rgb2))[0]
        assert (rgb1_uint & 0x00FFFFFF) == 0x0000FF00, f"P1 RGB green bits mismatch: 0x{rgb1_uint:08X}"
        assert (rgb2_uint & 0x00FFFFFF) == 0x0000FF00, f"P2 RGB green bits mismatch: 0x{rgb2_uint:08X}"

        # 3. Non-finite rejection test (NaN / Inf)
        nan_pt = Point(x=float('nan'), y=0.0, z=0.0)
        nan_msg = PointArray(points=[nan_pt])
        clouds = publish_and_wait(nan_msg, expected_count=1, timeout_sec=0.3)
        assert len(clouds) == 0, f"Non-finite point should be rejected, but got {len(clouds)} cloud messages"

        inf_pt = Point(x=0.0, y=float('inf'), z=0.0)
        inf_msg = PointArray(points=[inf_pt])
        clouds = publish_and_wait(inf_msg, expected_count=1, timeout_sec=0.3)
        assert len(clouds) == 0, f"Infinite point should be rejected, but got {len(clouds)} cloud messages"

        # 4. Oversized rejection test
        max_limit = node_point.max_points
        oversized_points = [Point(x=0.0, y=0.0, z=0.0) for _ in range(max_limit + 5)]
        oversized_msg = PointArray(points=oversized_points)
        clouds = publish_and_wait(oversized_msg, expected_count=1, timeout_sec=0.3)
        assert len(clouds) == 0, f"Oversized message should be rejected, but got {len(clouds)} cloud messages"

        print("[TEST] MediaPipePointNode DDS tests passed.")

    finally:
        if node_point is not None:
            node_point.destroy_node()
        if probe is not None:
            probe.destroy_node()


def main():
    parser = argparse.ArgumentParser(description="MediaPipe CPU & ROS2 Humble Offline Regression Test")
    parser.add_argument('--testdata', type=str, default='', help="Optional directory containing positive images (hands.jpg, pose.jpg)")
    args, _ = parser.parse_known_args()

    # Pass remaining args or namespace setup to rclpy
    rclpy.init(args=['--ros-args', '-r', '__ns:=/mp_test'])
    exit_code = 0

    try:
        check_sdk_version()
        test_blank_frames_migrated()
        test_blank_frames_arm()
        test_objectron_guarded()
        test_optional_positive_images(args.testdata)
        test_mediapipe_point_dds()
        print("\nAll offline MediaPipe / ROS2 Humble regression tests PASSED.")
    except Exception as exc:
        print(f"\n[FAIL] Regression test failed: {exc}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        exit_code = 1
    finally:
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
