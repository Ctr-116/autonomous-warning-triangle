#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenCV DNN actual model validation and TargetDetectionNode test script.

Validates:
  1. Real supplied models under ament share or explicit --share directory:
     - frozen_inference_graph.pb (+ ssd_mobilenet_v2_coco.txt / config)
     - graph_opt.pb (OpenPose)
     - Supplied Detection.jpg / person.jpg (or deterministic fallback)
  2. Actual inference forward pass via cv2.dnn:
     - Detection forward pass (SSD MobileNet V2 blob 300x300, mean 104,117,123, swapRB=True)
     - OpenPose forward pass (blob 368x368, mean 127.5, 19 heatmaps output)
  3. TargetDetectionAlgorithm execution (finite bounded boxes, identical dimensions)
  4. TargetDetectionNode ROS2 integration:
     - Best effort image callback, DDS message publication on processed_image and TargetArray.
     - Asserts models are loaded and actually executed.
"""

import argparse
import hashlib
import json
import os
import sys
import time

import numpy as np

try:
    import cv2 as cv
except ImportError:
    cv = None

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from ament_index_python.packages import get_package_share_directory

try:
    from yahboomcar_msgs.msg import TargetArray
except ImportError:
    TargetArray = None

try:
    from yahboomcar_visual.target_detection import (
        TargetDetectionAlgorithm,
        TargetDetectionNode,
        COCO_CLASSES,
        load_coco_names,
    )
except ImportError:
    TargetDetectionAlgorithm = None
    TargetDetectionNode = None
    COCO_CLASSES = None
    load_coco_names = None


def compute_sha256(filepath):
    """Compute sha256 checksum of a file without reading all weights into memory at once."""
    if not filepath or not os.path.isfile(filepath):
        return None
    hasher = hashlib.sha256()
    with open(filepath, 'rb') as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            hasher.update(chunk)
    return hasher.hexdigest()


def resolve_model_files(share_arg=None):
    """Locate model and image files across potential search directories."""
    search_dirs = []
    if share_arg and os.path.exists(share_arg):
        search_dirs.append(share_arg)
        search_dirs.append(os.path.join(share_arg, 'detection'))

    try:
        ament_share = get_package_share_directory('yahboomcar_visual')
        if ament_share and os.path.exists(ament_share):
            search_dirs.append(ament_share)
            search_dirs.append(os.path.join(ament_share, 'detection'))
    except Exception:
        pass

    # Common workspace source/share paths
    workspace_candidates = [
        'work/x3plus_ros2_full_ws/src/yahboomcar_visual/detection',
        'work/x3plus_ros2_full_ws/install/yahboomcar_visual/share/yahboomcar_visual/detection',
        'install/yahboomcar_visual/share/yahboomcar_visual/detection',
    ]
    for candidate in workspace_candidates:
        abs_cand = os.path.abspath(candidate)
        if os.path.exists(abs_cand):
            search_dirs.append(abs_cand)

    files_to_find = {
        'frozen_graph': ['frozen_inference_graph.pb'],
        'ssd_config': ['ssd_mobilenet_v2_coco.txt', 'ssd_mobilenet_v2_coco.pbtxt'],
        'pose_graph': ['graph_opt.pb'],
        'coco_txt': ['object_detection_coco.txt', 'coco.names'],
        'detection_img': ['Detection.jpg', 'person.jpg'],
        'person_img': ['person.jpg', 'Detection.jpg'],
    }

    resolved = {}
    for key, filenames in files_to_find.items():
        resolved[key] = None
        for base_dir in search_dirs:
            for fname in filenames:
                cand = os.path.join(base_dir, fname)
                if os.path.isfile(cand):
                    resolved[key] = os.path.abspath(cand)
                    break
            if resolved[key] is not None:
                break

    return resolved


def create_deterministic_image(width=640, height=480):
    """Generate a reproducible synthetic test image when sample image is missing."""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        img[y, :, 0] = (y % 256)
        img[y, :, 1] = ((y * 2) % 256)
        img[y, :, 2] = ((y * 3) % 256)
    cv.rectangle(img, (100, 100), (300, 400), (255, 255, 255), -1)
    cv.circle(img, (200, 150), 40, (0, 0, 255), -1)
    return img


def check_ssd_model(model_files, test_img):
    """Directly test SSD MobileNet V2 model loading and forward pass."""
    result = {
        'status': 'FAIL',
        'model_path': model_files.get('frozen_graph'),
        'config_path': model_files.get('ssd_config'),
        'sha256': compute_sha256(model_files.get('frozen_graph')),
        'loaded_net': None,
        'detections_count': 0,
        'error': None,
    }

    frozen_path = model_files.get('frozen_graph')
    config_path = model_files.get('ssd_config')

    if not frozen_path or not os.path.isfile(frozen_path):
        result['error'] = f"frozen_inference_graph.pb not found. Checked: {frozen_path}"
        return result

    net = None
    try:
        if config_path and os.path.isfile(config_path):
            net = cv.dnn.readNetFromTensorflow(frozen_path, config_path)
        else:
            net = cv.dnn.readNetFromTensorflow(frozen_path)
    except Exception as exc:
        result['error'] = f"Failed to load SSD model: {exc}"
        return result

    try:
        net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

        blob = cv.dnn.blobFromImage(
            image=test_img,
            size=(300, 300),
            mean=(104.0, 117.0, 123.0),
            swapRB=True,
            crop=False
        )
        net.setInput(blob)
        output = net.forward()

        if output is None or not isinstance(output, np.ndarray):
            result['error'] = "SSD forward output is None or invalid type"
            return result

        if not np.all(np.isfinite(output)):
            result['error'] = "SSD forward output contains NaN or Inf"
            return result

        result['loaded_net'] = net
        result['output_shape'] = list(output.shape)
        count = 0
        if len(output.shape) == 4 and output.shape[2] > 0:
            for det in output[0, 0, :, :]:
                if float(det[2]) > 0.1:
                    count += 1
        result['detections_count'] = count
        result['status'] = 'PASS'
    except Exception as exc:
        result['error'] = f"SSD forward execution failed: {exc}"

    return result


def check_openpose_model(model_files, test_img):
    """Directly test OpenPose graph_opt.pb loading and forward pass."""
    result = {
        'status': 'FAIL',
        'model_path': model_files.get('pose_graph'),
        'sha256': compute_sha256(model_files.get('pose_graph')),
        'loaded_net': None,
        'heatmaps_finite': False,
        'error': None,
    }

    pose_path = model_files.get('pose_graph')
    if not pose_path or not os.path.isfile(pose_path):
        result['error'] = f"graph_opt.pb not found. Checked: {pose_path}"
        return result

    net = None
    try:
        net = cv.dnn.readNetFromTensorflow(pose_path)
    except Exception as exc:
        result['error'] = f"Failed to load OpenPose model: {exc}"
        return result

    try:
        net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

        blob = cv.dnn.blobFromImage(
            test_img,
            scalefactor=1.0,
            size=(368, 368),
            mean=(127.5, 127.5, 127.5),
            swapRB=True,
            crop=False
        )
        net.setInput(blob)
        out = net.forward()

        if out is None or not isinstance(out, np.ndarray):
            result['error'] = "OpenPose forward output is None or invalid type"
            return result

        if out.ndim != 4 or out.shape[1] < 19:
            result['error'] = f"Expected at least 19 heatmap channels, got shape {out.shape}"
            return result

        heatmaps = out[:, :19, :, :]
        if not np.all(np.isfinite(heatmaps)):
            result['error'] = "OpenPose 19 heatmaps contain NaN or Inf values"
            return result

        result['loaded_net'] = net
        result['heatmaps_finite'] = True
        result['output_shape'] = list(out.shape)
        result['status'] = 'PASS'
    except Exception as exc:
        result['error'] = f"OpenPose forward execution failed: {exc}"

    return result


def check_target_detection_algorithm(detection_net, openpose_net, test_img, coco_path):
    """Exercise TargetDetectionAlgorithm functions with real loaded networks."""
    result = {
        'status': 'FAIL',
        'target_detection_pass': False,
        'openpose_pass': False,
        'error': None,
    }

    if TargetDetectionAlgorithm is None:
        result['error'] = "TargetDetectionAlgorithm class could not be imported"
        return result

    classes = None
    if coco_path and os.path.isfile(coco_path) and load_coco_names is not None:
        try:
            classes = load_coco_names(coco_path)
        except Exception:
            classes = list(COCO_CLASSES)
    else:
        classes = list(COCO_CLASSES) if COCO_CLASSES else []

    algo = TargetDetectionAlgorithm(
        coco_names=classes,
        detection_model=detection_net,
        openpose_net=openpose_net,
        confidence=0.1,
        pose_threshold=0.1
    )

    h, w, c = test_img.shape

    # 1. Test Target_Detection
    if detection_net is not None:
        try:
            img_copy = test_img.copy()
            annotated_img, detections = algo.Target_Detection(img_copy)
            if annotated_img.shape != (h, w, c):
                result['error'] = f"Target_Detection changed image shape: {annotated_img.shape} vs {(h, w, c)}"
                return result

            for det in detections:
                for k in ['ptx', 'pty', 'distw', 'disth', 'centerx', 'centery', 'confidence']:
                    val = det[k]
                    if not np.isfinite(val):
                        result['error'] = f"Non-finite value {val} for key {k} in detection"
                        return result
            result['target_detection_pass'] = True
        except Exception as exc:
            result['error'] = f"Target_Detection execution failed: {exc}"
            return result
    else:
        result['target_detection_pass'] = False

    # 2. Test openpose
    if openpose_net is not None:
        try:
            img_copy2 = test_img.copy()
            pose_img = algo.openpose(img_copy2)
            if pose_img.shape != (h, w, c):
                result['error'] = f"OpenPose changed image shape: {pose_img.shape} vs {(h, w, c)}"
                return result
            result['openpose_pass'] = True
        except Exception as exc:
            result['error'] = f"OpenPose execution failed: {exc}"
            return result
    else:
        result['openpose_pass'] = False

    if result['target_detection_pass'] or result['openpose_pass']:
        result['status'] = 'PASS'
    return result


def check_target_detection_node(model_files, test_img):
    """Test TargetDetectionNode instantiation, callback processing and DDS publishing."""
    result = {
        'status': 'FAIL',
        'node_instantiated': False,
        'model_loaded': False,
        'processed_image_received': False,
        'target_array_received': False,
        'error': None,
    }

    if TargetDetectionNode is None:
        result['error'] = "TargetDetectionNode could not be imported"
        return result

    if not rclpy.ok():
        rclpy.init()

    node = None
    helper_node = None
    executor = None

    try:
        node_params = [
            rclpy.parameter.Parameter('image_topic', rclpy.Parameter.Type.STRING, '/camera/image_raw'),
            rclpy.parameter.Parameter('processed_image_topic', rclpy.Parameter.Type.STRING, '/camera/processed_image'),
            rclpy.parameter.Parameter('target_array_topic', rclpy.Parameter.Type.STRING, '/camera/TargetArray'),
            rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.STRING, 'detection'),
            rclpy.parameter.Parameter('gui', rclpy.Parameter.Type.BOOL, False),
        ]
        if model_files.get('frozen_graph'):
            node_params.append(rclpy.parameter.Parameter(
                'frozen_graph_path', rclpy.Parameter.Type.STRING, model_files['frozen_graph']))
        if model_files.get('pose_graph'):
            node_params.append(rclpy.parameter.Parameter(
                'pose_graph_path', rclpy.Parameter.Type.STRING, model_files['pose_graph']))
        if model_files.get('coco_txt'):
            node_params.append(rclpy.parameter.Parameter(
                'coco_names_path', rclpy.Parameter.Type.STRING, model_files['coco_txt']))

        if model_files.get('ssd_config'):
            node_params.append(rclpy.parameter.Parameter(
                'detection_config_path', rclpy.Parameter.Type.STRING, model_files['ssd_config']))
        node = TargetDetectionNode(parameter_overrides=node_params)

        result['node_instantiated'] = True

        # Assert model is not lazily left as None if files exist
        if node.algo.detection_model is not None:
            result['model_loaded'] = True
        else:
            result['error'] = "TargetDetectionNode has no detection model loaded"
            return result

        # Setup subscriber test harness
        helper_node = rclpy.create_node('dnn_test_harness')
        bridge = CvBridge() if CvBridge else None
        if bridge is None:
            result['error'] = "CvBridge not available"
            return result

        processed_images = []
        target_arrays = []

        def proc_img_cb(msg):
            processed_images.append(msg)

        def target_arr_cb(msg):
            target_arrays.append(msg)

        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )

        helper_node.create_subscription(Image, '/camera/processed_image', proc_img_cb, 10)
        helper_node.create_subscription(TargetArray, '/camera/TargetArray', target_arr_cb, 10)

        pub_img = helper_node.create_publisher(Image, '/camera/image_raw', best_effort_qos)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(helper_node)

        # Publish test frame
        img_msg = bridge.cv2_to_imgmsg(test_img, encoding='bgr8')
        img_msg.header.stamp = helper_node.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'

        # Discover and repeatedly send a real frame until outputs arrive.
        # Publishing once before discovery is not a reliable DDS fixture.
        start_wait = time.time()
        next_publish = 0.0
        while time.time() - start_wait < 10.0:
            if time.monotonic() >= next_publish and pub_img.get_subscription_count() > 0:
                pub_img.publish(img_msg)
                next_publish = time.monotonic() + 0.3
            executor.spin_once(timeout_sec=0.1)
            if len(processed_images) > 0 and len(target_arrays) > 0:
                break

        if len(processed_images) > 0:
            result['processed_image_received'] = True
        if len(target_arrays) > 0:
            result['target_array_received'] = True

        if result['processed_image_received'] and result['target_array_received']:
            result['status'] = 'PASS'
        else:
            result['error'] = (
                f"Did not receive expected DDS messages. "
                f"processed_images={len(processed_images)}, target_arrays={len(target_arrays)}"
            )

    except Exception as exc:
        result['error'] = f"Exception in TargetDetectionNode test: {exc}"
    finally:
        if executor is not None:
            executor.shutdown()
        if helper_node is not None:
            helper_node.destroy_node()
        if node is not None:
            node.destroy_node()

    return result


def main():
    parser = argparse.ArgumentParser(description="OpenCV DNN perception check for ROS2 Humble.")
    parser.add_argument('--share', type=str, default=None, help="Explicit share path or root directory containing models.")
    parsed_args, _ = parser.parse_known_args()

    results = {
        'opencv_available': cv is not None,
        'model_files': {},
        'checks': {
            'ssd_mobilenet_v2': None,
            'openpose': None,
            'algorithm_pipeline': None,
            'node_ros2_dds': None,
        },
        'overall_status': 'FAIL'
    }

    if cv is None:
        results['error'] = "OpenCV (cv2) is not installed or available"
        print(json.dumps(results, indent=2))
        sys.exit(1)

    model_files = resolve_model_files(parsed_args.share)
    results['model_files'] = {k: v for k, v in model_files.items()}

    # Load test image
    test_img = None
    for img_key in ['detection_img', 'person_img']:
        path = model_files.get(img_key)
        if path and os.path.isfile(path):
            try:
                loaded = cv.imread(path)
                if loaded is not None and loaded.size > 0:
                    test_img = loaded
                    results['test_image_source'] = path
                    break
            except Exception:
                pass

    if test_img is None:
        test_img = create_deterministic_image(640, 480)
        results['test_image_source'] = 'synthetic_deterministic_640x480'

    # Check 1: SSD MobileNet V2
    ssd_res = check_ssd_model(model_files, test_img)
    ssd_net = ssd_res.pop('loaded_net', None)
    results['checks']['ssd_mobilenet_v2'] = ssd_res

    # Check 2: OpenPose graph_opt.pb
    pose_res = check_openpose_model(model_files, test_img)
    pose_net = pose_res.pop('loaded_net', None)
    results['checks']['openpose'] = pose_res

    # Check 3: TargetDetectionAlgorithm
    algo_res = check_target_detection_algorithm(
        ssd_net, pose_net, test_img, model_files.get('coco_txt'))
    results['checks']['algorithm_pipeline'] = algo_res

    # Check 4: TargetDetectionNode DDS
    node_res = check_target_detection_node(model_files, test_img)
    results['checks']['node_ros2_dds'] = node_res

    # Determine overall status: true actual success requires at least one real model pass
    # and no silent fake successes.
    has_real_dnn_pass = (ssd_res['status'] == 'PASS' or pose_res['status'] == 'PASS')
    algo_passed = (algo_res['status'] == 'PASS')
    node_passed = (node_res['status'] == 'PASS')

    if has_real_dnn_pass and algo_passed and node_passed:
        results['overall_status'] = 'PASS'
    else:
        results['overall_status'] = 'FAIL'

    print(json.dumps(results, indent=2))

    if rclpy.ok():
        rclpy.shutdown()

    if results['overall_status'] != 'PASS':
        sys.exit(1)
    sys.exit(0)


if __name__ == '__main__':
    main()
