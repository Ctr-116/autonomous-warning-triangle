#!/usr/bin/env python3
# coding: utf-8

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from garbage_identify_yolov11.detector import (
    ORIGINAL_CONFIDENCE,
    OnnxYoloBackend,
    YoloDetector,
    decode_yolo11_output,
    nms_candidates,
)
from garbage_identify_yolov11.garbage_library import (
    category_point,
    classify_label,
)


def resolve_model_path(cli_path=None):
    if cli_path and os.path.isfile(cli_path):
        return os.path.abspath(cli_path)
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = get_package_share_directory('garbage_identify_yolov11')
        cand = os.path.join(share_dir, 'models', 'best1.onnx')
        if os.path.isfile(cand):
            return os.path.abspath(cand)
    except Exception:
        pass
    script_dir = Path(__file__).resolve().parent
    fallback_cand = script_dir.parent / 'models' / 'best1.onnx'
    if fallback_cand.is_file():
        return str(fallback_cand.resolve())
    same_dir_cand = (
        script_dir.parents[0]
        / 'garbage_identify_yolov11'
        / 'models'
        / 'best1.onnx'
    )
    if same_dir_cand.is_file():
        return str(same_dir_cand.resolve())
    if cli_path:
        return os.path.abspath(cli_path)
    return str(fallback_cand.resolve())


def resolve_samples_dir(cli_dir=None):
    if cli_dir and os.path.isdir(cli_dir):
        return os.path.abspath(cli_dir)
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = get_package_share_directory('garbage_identify_yolov11')
        cand = os.path.join(share_dir, 'models', 'samples')
        if os.path.isdir(cand):
            return os.path.abspath(cand)
    except Exception:
        pass
    script_dir = Path(__file__).resolve().parent
    fallback_cand = script_dir.parents[1] / 'models' / 'samples'
    if fallback_cand.is_dir():
        return str(fallback_cand.resolve())
    same_dir_cand = (
        script_dir.parents[0]
        / 'garbage_identify_yolov11'
        / 'models'
        / 'samples'
    )
    if same_dir_cand.is_dir():
        return str(same_dir_cand.resolve())
    if cli_dir:
        return os.path.abspath(cli_dir)
    return str(fallback_cand.resolve())


def test_synthetic_postprocessing():
    # Synthetic test fixture: 16 classes -> rows = 4 + 16 = 20, 3 anchors.
    # YOLO11 has NO objectness row.
    tensor_shape = [1, 20, 3]
    raw_tensor = np.zeros(tensor_shape, dtype=np.float32)

    # Anchor 0: Valid high-confidence detection (class 2, score 0.95 >= 0.88)
    raw_tensor[0, 0, 0] = 320.0  # cx
    raw_tensor[0, 1, 0] = 320.0  # cy
    raw_tensor[0, 2, 0] = 100.0  # w
    raw_tensor[0, 3, 0] = 100.0  # h
    raw_tensor[0, 4 + 2, 0] = 0.95

    # Anchor 1: Low-confidence detection (class 5, score 0.70 < 0.88)
    raw_tensor[0, 0, 1] = 100.0
    raw_tensor[0, 1, 1] = 100.0
    raw_tensor[0, 2, 1] = 50.0
    raw_tensor[0, 3, 1] = 50.0
    raw_tensor[0, 4 + 5, 1] = 0.70

    # Anchor 2: Duplicate/overlapping box for class 2 to verify class-aware NMS
    raw_tensor[0, 0, 2] = 322.0
    raw_tensor[0, 1, 2] = 321.0
    raw_tensor[0, 2, 2] = 98.0
    raw_tensor[0, 3, 2] = 102.0
    raw_tensor[0, 4 + 2, 2] = 0.91

    candidates = decode_yolo11_output(
        raw_tensor, class_count=16, confidence=ORIGINAL_CONFIDENCE
    )
    assert (
        len(candidates) == 2
    ), f"Expected 2 candidates >= {ORIGINAL_CONFIDENCE}, got {len(candidates)}"

    for c in candidates:
        assert c["score"] >= ORIGINAL_CONFIDENCE
        assert c["class_id"] == 2

    # NMS & letterbox unpad test
    orig_w, orig_h = 1280, 720
    ratio = min(640.0 / orig_h, 640.0 / orig_w)
    pad_x = (640 - int(round(orig_w * ratio))) // 2
    pad_y = (640 - int(round(orig_h * ratio))) // 2

    dets = nms_candidates(
        candidates,
        orig_w,
        orig_h,
        ratio,
        pad_x,
        pad_y,
        score_threshold=ORIGINAL_CONFIDENCE,
    )
    assert (
        len(dets) == 1
    ), f"NMS failed to suppress duplicate box; got {len(dets)} detections"
    assert dets[0]["class_id"] == 2
    assert dets[0]["confidence"] >= 0.94

    box = dets[0]["box"]
    assert 0.0 <= box[0] <= box[2] <= orig_w
    assert 0.0 <= box[1] <= box[3] <= orig_h
    assert all(math.isfinite(coord) for coord in box)

    # Test rejection of invalid shape
    invalid_shape_tensor = np.zeros([1, 19, 3], dtype=np.float32)
    rejected_shape = False
    try:
        decode_yolo11_output(
            invalid_shape_tensor, class_count=16, confidence=ORIGINAL_CONFIDENCE
        )
    except ValueError:
        rejected_shape = True
    assert (
        rejected_shape
    ), "decode_yolo11_output accepted invalid row shape without error"

    # Test rejection of non-finite values
    nan_tensor = np.zeros([1, 20, 3], dtype=np.float32)
    nan_tensor[0, 0, 0] = np.nan
    rejected_nan = False
    try:
        decode_yolo11_output(
            nan_tensor, class_count=16, confidence=ORIGINAL_CONFIDENCE
        )
    except ValueError:
        rejected_nan = True
    assert (
        rejected_nan
    ), "decode_yolo11_output accepted NaN tensor without error"


def cv2_to_imgmsg(cv_image, encoding="bgr8"):
    if CvBridge is not None:
        return CvBridge().cv2_to_imgmsg(cv_image, encoding=encoding)
    msg = Image()
    h, w, c = cv_image.shape
    msg.height = h
    msg.width = w
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = w * c
    msg.data = cv_image.tobytes()
    return msg


def main():
    parser = argparse.ArgumentParser(
        description="ROS2 Humble YOLO inference regression check"
    )
    parser.add_argument(
        "--model", default=None, help="Path to best1.onnx model"
    )
    parser.add_argument(
        "--samples", default=None, help="Directory containing validation JPEGs"
    )
    parsed_args, _ = parser.parse_known_args()

    model_path = resolve_model_path(parsed_args.model)
    samples_dir = resolve_samples_dir(parsed_args.samples)

    results_report = {
        "model_path": model_path,
        "samples_dir": samples_dir,
        "imports": {},
        "metadata_and_library": {},
        "synthetic_postprocessing": {},
        "actual_backend_inference": {},
        "black_image_inference": {},
        "dds_disabled_node": {},
        "dds_active_detector": {},
        "summary": "FAIL",
    }

    # Group 1: Dependency cleanliness check
    assert (
        "torch" not in sys.modules
    ), "CRITICAL: torch imported in sys.modules"
    assert (
        "ultralytics" not in sys.modules
    ), "CRITICAL: ultralytics imported in sys.modules"
    results_report["imports"]["torch_absent"] = True
    results_report["imports"]["ultralytics_absent"] = True
    print("PASS: Dependency isolation (torch and ultralytics not loaded)")

    # Group 2: Synthetic postprocessing unit test
    test_synthetic_postprocessing()
    results_report["synthetic_postprocessing"]["status"] = "PASSED"
    print("PASS: Pure synthetic YOLO11 post-processing unit fixture")

    # Group 3: OnnxYoloBackend initialization and metadata validation
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"Model file not found at: {model_path}")

    backend = OnnxYoloBackend(model_path, confidence=ORIGINAL_CONFIDENCE)
    assert len(backend.names) == 16, f"Expected 16 classes, got {len(backend.names)}"

    for class_id, label_name in backend.names.items():
        cat = classify_label(label_name)
        assert (
            cat is not None
        ), f"Metadata label {label_name} (class {class_id}) rejected by classify_label"
        pt = category_point(cat)
        assert (
            pt in (1, 2, 3, 4)
        ), f"Category {cat} for {label_name} has invalid point {pt}"

    results_report["metadata_and_library"]["class_count"] = len(backend.names)
    results_report["metadata_and_library"]["all_labels_classified"] = True
    print("PASS: Model metadata and garbage_library class mapping alignment")

    # Group 4: Actual inference across validation JPEGs
    if not os.path.isdir(samples_dir):
        raise FileNotFoundError(f"Samples directory not found at: {samples_dir}")

    jpeg_files = sorted(
        [
            os.path.join(samples_dir, f)
            for f in os.listdir(samples_dir)
            if f.lower().endswith((".jpg", ".jpeg"))
        ]
    )
    assert (
        len(jpeg_files) >= 6
    ), f"Expected at least 6 validation JPEGs in {samples_dir}, found {len(jpeg_files)}"

    total_positive_detections = 0
    sample_observations = {}
    known_positive_img = None
    known_positive_dets = None

    for img_path in jpeg_files[:6]:
        fname = os.path.basename(img_path)
        img_bgr = cv2.imread(img_path)
        assert (
            img_bgr is not None
        ), f"Failed to read image at: {img_path}"
        assert (
            img_bgr.dtype == np.uint8
        ), f"Image {fname} dtype is {img_bgr.dtype}, expected uint8"
        h, w = img_bgr.shape[:2]

        dets = backend.infer(img_bgr, confidence=ORIGINAL_CONFIDENCE)
        sample_observations[fname] = dets

        for d in dets:
            total_positive_detections += 1
            assert (
                d["confidence"] >= ORIGINAL_CONFIDENCE
            ), f"Detection {d['label']} confidence {d['confidence']} < {ORIGINAL_CONFIDENCE}"
            box = d["box"]
            assert all(
                math.isfinite(v) for v in box
            ), f"Non-finite box {box} in {fname}"
            assert (
                0.0 <= box[0] <= box[2] <= float(w) + 1.0
            ), f"Box X bounds violated: {box} for w={w}"
            assert (
                0.0 <= box[1] <= box[3] <= float(h) + 1.0
            ), f"Box Y bounds violated: {box} for h={h}"

        if dets and known_positive_img is None:
            known_positive_img = img_bgr.copy()
            known_positive_dets = dets

    results_report["actual_backend_inference"]["samples_tested"] = len(
        jpeg_files[:6]
    )
    results_report["actual_backend_inference"]["total_detections"] = (
        total_positive_detections
    )
    results_report["actual_backend_inference"]["observations"] = (
        sample_observations
    )

    if total_positive_detections == 0:
        results_report["actual_backend_inference"]["failure"] = (
            f"No detections at confidence {ORIGINAL_CONFIDENCE}"
        )
        print(json.dumps(results_report, indent=2))
        sys.exit(1)

    print(
        f"PASS: Actual OnnxYoloBackend session inference ({total_positive_detections} positive detections at threshold {ORIGINAL_CONFIDENCE})"
    )

    # Group 5: Real black image inference (empty allowed, no fabricated detections)
    black_img = np.zeros((480, 640, 3), dtype=np.uint8)
    black_dets = backend.infer(black_img, confidence=ORIGINAL_CONFIDENCE)
    assert (
        len(black_dets) == 0
    ), f"Black image produced fabricated detections: {black_dets}"
    results_report["black_image_inference"]["detections_count"] = 0
    print("PASS: Black image inference yields zero fabricated detections")

    # Group 6 & 7: ROS2 DDS Node Tests
    rclpy.init()
    executor = SingleThreadedExecutor()

    try:
        # Group 6: Disabled YoloDetector when model_path is blank
        disabled_node = YoloDetector(
            parameter_overrides=[Parameter(name, value=value) for name, value in {
                "model_path": "",
                "image_topic": "test_disabled/image",
                "detect_topic": "test_disabled/DetectMsg",
            }.items()],
        )
        assert (
            not disabled_node.enabled
        ), "Node with blank model_path should have enabled=False"

        executor.add_node(disabled_node)

        disabled_helper = Node("test_disabled_helper")
        executor.add_node(disabled_helper)

        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        dis_pub = disabled_helper.create_publisher(
            Image, "test_disabled/image", best_effort_qos
        )
        dis_emissions = []
        dis_sub = disabled_helper.create_subscription(
            String,
            "test_disabled/DetectMsg",
            lambda m: dis_emissions.append(m.data),
            10,
        )

        test_img_msg = cv2_to_imgmsg(known_positive_img, encoding="bgr8")
        dis_pub.publish(test_img_msg)

        start_time = time.monotonic()
        while time.monotonic() - start_time < 0.6:
            executor.spin_once(timeout_sec=0.05)

        assert (
            len(dis_emissions) == 0
        ), f"Disabled node published messages: {dis_emissions}"
        assert (
            disabled_node._frames_received == 0
        ), "Disabled node processed image frames when enabled was False"

        executor.remove_node(disabled_node)
        executor.remove_node(disabled_helper)
        disabled_node.destroy_node()
        disabled_helper.destroy_node()

        results_report["dds_disabled_node"]["enabled"] = False
        results_report["dds_disabled_node"]["emissions_count"] = 0
        print("PASS: Default blank model_path keeps enabled=False and emits nothing")

        # Group 7: Actual DDS node test with OnnxRuntime backend and known positive image
        yolo_node = YoloDetector(
            parameter_overrides=[Parameter(name, value=value) for name, value in {
                "model_path": model_path,
                "backend": "onnxruntime",
                "image_topic": "fixture/image",
                "detect_topic": "fixture/DetectMsg",
                "publish_period_s": 0.1,
                "confidence": ORIGINAL_CONFIDENCE,
            }.items()],
        )
        assert yolo_node.enabled, "fixture_yolo_detector failed to enable"
        assert yolo_node.backend_kind == "onnxruntime"
        assert yolo_node.backend is not None

        executor.add_node(yolo_node)

        test_helper = Node("fixture_test_helper")
        executor.add_node(test_helper)

        img_pub = test_helper.create_publisher(
            Image, "fixture/image", best_effort_qos
        )
        received_labels = []
        detect_sub = test_helper.create_subscription(
            String,
            "fixture/DetectMsg",
            lambda msg: received_labels.append(msg.data),
            10,
        )

        # Wait for discovery (monotonic bounded wait 3-8s)
        discovery_start = time.monotonic()
        discovered = False
        while time.monotonic() - discovery_start < 5.0:
            executor.spin_once(timeout_sec=0.05)
            if (
                img_pub.get_subscription_count() > 0
                and yolo_node.pub.get_subscription_count() > 0
            ):
                discovered = True
                break

        assert (
            discovered
        ), "DDS topic discovery timed out between detector and test fixture"

        # Send known positive image
        pos_msg = cv2_to_imgmsg(known_positive_img, encoding="bgr8")
        img_pub.publish(pos_msg)

        listen_start = time.monotonic()
        while time.monotonic() - listen_start < 4.0:
            executor.spin_once(timeout_sec=0.05)
            if len(received_labels) > 0:
                break

        expected_labels = {d["label"] for d in known_positive_dets}
        received_set = set(received_labels)

        assert (
            len(received_set) > 0
        ), "No DetectMsg received over DDS for known positive image"
        assert (
            received_set == expected_labels
        ), f"DDS detected labels mismatch: received {received_set}, backend inferred {expected_labels}"

        # Rate-limiting burst verification
        # Send burst of frames quickly
        initial_frames = yolo_node._frames_received
        for _ in range(15):
            img_pub.publish(pos_msg)
            time.sleep(0.005)

        burst_start = time.monotonic()
        while time.monotonic() - burst_start < 0.8:
            executor.spin_once(timeout_sec=0.05)

        frames_delta = yolo_node._frames_received - initial_frames
        # With publish_period_s = 0.1 and burst across ~0.8s, processed frames must be bounded < 15
        assert (
            frames_delta < 15
        ), f"Rate limiting failed: node processed {frames_delta}/15 rapid frames"

        results_report["dds_active_detector"]["discovered"] = True
        results_report["dds_active_detector"]["received_labels"] = list(
            received_set
        )
        results_report["dds_active_detector"]["expected_labels"] = list(
            expected_labels
        )
        results_report["dds_active_detector"]["frames_processed_in_burst"] = (
            frames_delta
        )
        print(
            "PASS: Actual DDS node detects matching labels and enforces publish_period rate limiting"
        )

        executor.remove_node(yolo_node)
        executor.remove_node(test_helper)
        yolo_node.destroy_node()
        test_helper.destroy_node()

    finally:
        if rclpy.ok():
            rclpy.shutdown()

    results_report["summary"] = "PASS"
    print("\nRegression Check Results:")
    print(json.dumps(results_report, indent=2))
    sys.exit(0)


if __name__ == "__main__":
    main()
