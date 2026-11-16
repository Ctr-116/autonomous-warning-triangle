#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Full DNN target detection + OpenPose for ROS2 Humble.

Preserves ALL original DNN functions from target_detection.py:
  * Target_Detection  (ssd_mobilenet_v2_coco frozen_inference_graph.pb)
  * Openpose           (graph_opt.pb)

Models are loaded lazily and by explicit valid path (ament share dir or param).
Produces yahboomcar_msgs/TargetArray on processed_image.
"""

import os
import time

import numpy as np

try:
    import cv2 as cv
except Exception:  # pragma: no cover
    cv = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException

from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    CvBridge = None

from ament_index_python.packages import get_package_share_directory

try:
    from yahboomcar_msgs.msg import Target, TargetArray
except Exception:  # pragma: no cover
    Target = None
    TargetArray = None


COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'street sign',
    'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
    'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'hat', 'backpack',
    'umbrella', 'shoe', 'eye glasses', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'plate', 'wine glass',
    'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich',
    'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'mirror', 'dining table', 'window', 'desk',
    'toilet', 'door', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
    'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
    'blender', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
    'toothbrush'
]

COLORS = np.random.uniform(0, 255, size=(len(COCO_CLASSES) + 1, 3)).astype(np.float32)

BODY_PARTS = {"Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
              "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
              "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
              "LEye": 15, "REar": 16, "LEar": 17, "Background": 18}
POSE_PAIRS = [["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
              ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
              ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
              ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
              ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"]]


def _best_effort_qos(depth=1):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)


def _default_share_dir():
    try:
        return get_package_share_directory('yahboomcar_visual')
    except Exception:
        return ''


def load_coco_names(path):
    with open(path, 'r') as f:
        return [ln for ln in f.read().split('\n') if ln]


class TargetDetectionAlgorithm:
    """Pure algorithm helpers preserving all original DNN functions.

    Models are injected lazily so tests may supply their own cv.dnn networks.
    """

    def __init__(self, coco_names=None, detection_model=None, openpose_net=None,
                 confidence=0.4, pose_threshold=0.2):
        self.class_names = coco_names if coco_names is not None else list(COCO_CLASSES)
        self.detection_model = detection_model
        self.openpose_net = openpose_net
        self.confidence = confidence
        self.pose_threshold = pose_threshold

    def Target_Detection(self, image):
        """Original Target_Detection; returns annotated image and detections."""
        if self.detection_model is None or cv is None:
            return image, []
        image_height, image_width, _ = image.shape
        blob = cv.dnn.blobFromImage(image=image, size=(300, 300),
                                    mean=(104, 117, 123), swapRB=True)
        self.detection_model.setInput(blob)
        output = self.detection_model.forward()
        detections = []
        for detection in output[0, 0, :, :]:
            confidence = float(detection[2])
            if confidence <= self.confidence:
                continue
            class_id = int(detection[1])
            idx = class_id - 1
            if idx < 0 or idx >= len(self.class_names):
                continue
            class_name = self.class_names[idx]
            color = COLORS[class_id % len(COLORS)].tolist()
            box_x = float(detection[3]) * image_width
            box_y = float(detection[4]) * image_height
            box_width = float(detection[5]) * image_width
            box_height = float(detection[6]) * image_height
            x1, y1 = int(box_x), int(box_y)
            x2, y2 = int(box_width), int(box_height)
            detections.append({
                'class_name': class_name, 'class_id': class_id,
                'confidence': confidence, 'ptx': box_x, 'pty': box_y,
                'distw': box_width, 'disth': box_height,
                'centerx': (box_x + box_width) / 2.0,
                'centery': (box_y + box_height) / 2.0,
            })
            cv.rectangle(image, (x1, y1), (x2, y2), color, thickness=2)
            cv.putText(image, class_name, (x1, y1 - 5), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        return image, detections

    def openpose(self, frame):
        """Original OpenPose algorithm on graph_opt.pb."""
        if self.openpose_net is None or cv is None:
            return frame
        frameHeight, frameWidth = frame.shape[:2]
        self.openpose_net.setInput(cv.dnn.blobFromImage(
            frame, 1.0, (368, 368), (127.5, 127.5, 127.5), swapRB=True, crop=False))
        out = self.openpose_net.forward()
        out = out[:, :19, :, :]
        if out.shape[1] != len(BODY_PARTS):
            return frame
        points = []
        for i in range(len(BODY_PARTS)):
            heatMap = out[0, i, :, :]
            _, conf, _, point = cv.minMaxLoc(heatMap)
            x = (frameWidth * point[0]) / out.shape[3]
            y = (frameHeight * point[1]) / out.shape[2]
            points.append((int(x), int(y)) if conf > self.pose_threshold else None)
        for pair in POSE_PAIRS:
            idFrom = BODY_PARTS[pair[0]]
            idTo = BODY_PARTS[pair[1]]
            if points[idFrom] and points[idTo]:
                cv.line(frame, points[idFrom], points[idTo], (0, 255, 0), 3)
                cv.ellipse(frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
                cv.ellipse(frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
        return frame


def _resolve_model_path(param_value, share_rel):
    """Return an existing explicit path, else resolve under ament share dir."""
    if param_value:
        if os.path.isfile(param_value):
            return param_value
        # Also try as relative under share dir
        share = _default_share_dir()
        if share:
            candidate = os.path.join(share, param_value)
            if os.path.isfile(candidate):
                return candidate
        return None
    share = _default_share_dir()
    if share:
        candidate = os.path.join(share, share_rel)
        if os.path.isfile(candidate):
            return candidate
    return None


class TargetDetectionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('target_detection', **kwargs)
        if cv is None or CvBridge is None:
            raise RuntimeError('opencv-python and cv_bridge are required')
        if Target is None or TargetArray is None:
            raise RuntimeError('yahboomcar_msgs Target/TargetArray are required')

        self.bridge = CvBridge()
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('processed_image_topic', 'processed_image')
        self.declare_parameter('target_array_topic', 'TargetArray')
        self.declare_parameter('frozen_graph_path', '')
        self.declare_parameter('detection_config_path', '')
        self.declare_parameter('pose_graph_path', '')
        self.declare_parameter('coco_names_path', '')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('pose_threshold', 0.2)
        self.declare_parameter('gui', False)
        self.declare_parameter('mode', 'detection')
        self.declare_parameter('frame_id', '')

        image_topic = str(self.get_parameter('image_topic').value)
        processed_topic = str(self.get_parameter('processed_image_topic').value)
        targets_topic = str(self.get_parameter('target_array_topic').value)
        self.gui = bool(self.get_parameter('gui').value)
        self.mode = str(self.get_parameter('mode').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.confidence = float(self.get_parameter('confidence').value)
        pose_threshold = float(self.get_parameter('pose_threshold').value)

        valid_modes = ('detection', 'openpose')
        if self.mode not in valid_modes:
            raise RuntimeError("Invalid mode '%s'. Must be one of %s" % (self.mode, valid_modes))

        coco_path = _resolve_model_path(
            str(self.get_parameter('coco_names_path').value),
            os.path.join('detection', 'object_detection_coco.txt'))

        class_names = list(COCO_CLASSES)
        if coco_path:
            try:
                class_names = load_coco_names(coco_path)
            except Exception as exc:
                self.get_logger().warn('could not read COCO names: %s' % exc)

        detection_model = None
        openpose_net = None

        if self.mode == 'detection':
            frozen = _resolve_model_path(
                str(self.get_parameter('frozen_graph_path').value),
                os.path.join('detection', 'frozen_inference_graph.pb'))
            detection_config = _resolve_model_path(
                str(self.get_parameter('detection_config_path').value),
                os.path.join('detection', 'ssd_mobilenet_v2_coco.txt'))

            if not frozen:
                raise RuntimeError('frozen_inference_graph.pb not found for detection mode')
            try:
                if detection_config:
                    detection_model = cv.dnn.readNetFromTensorflow(frozen, detection_config)
                else:
                    detection_model = cv.dnn.readNetFromTensorflow(frozen)
            except Exception as exc:
                raise RuntimeError('failed to load detection model: %s' % exc) from exc
        elif self.mode == 'openpose':
            pose_graph = _resolve_model_path(
                str(self.get_parameter('pose_graph_path').value),
                os.path.join('detection', 'graph_opt.pb'))
            if not pose_graph:
                raise RuntimeError('graph_opt.pb not found for openpose mode')
            try:
                openpose_net = cv.dnn.readNetFromTensorflow(pose_graph)
            except Exception as exc:
                raise RuntimeError('failed to load openpose model: %s' % exc) from exc

        self.algo = TargetDetectionAlgorithm(
            coco_names=class_names, detection_model=detection_model,
            openpose_net=openpose_net, confidence=self.confidence,
            pose_threshold=pose_threshold)

        qos = _best_effort_qos(1)
        self.sub_img = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.pub_img = self.create_publisher(Image, processed_topic, 1)
        self.pub_targets = self.create_publisher(TargetArray, targets_topic, 10)
        self.get_logger().info('target_detection ready (mode=%s, image=%s, out=%s)' %
                               (self.mode, image_topic, processed_topic))

    def image_cb(self, msg):
        if not isinstance(msg, Image):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('cv_bridge failed: %s' % exc)
            return

        if frame is None or frame.size == 0:
            self.get_logger().error('received empty or invalid frame')
            return

        start = time.time()
        detections = []
        try:
            if self.mode == 'openpose':
                if self.algo.openpose_net is None:
                    self.get_logger().error('openpose_net is not initialized')
                    return
                frame = self.algo.openpose(frame)
                cv.putText(frame, "Openpose", (240, 30), cv.FONT_HERSHEY_SIMPLEX,
                           0.9, (0, 0, 255), 1)
            else:
                if self.algo.detection_model is None:
                    self.get_logger().error('detection_model is not initialized')
                    return
                frame, detections = self.algo.Target_Detection(frame)
                cv.putText(frame, "Detection", (240, 30), cv.FONT_HERSHEY_SIMPLEX,
                           0.9, (0, 0, 255), 1)
        except Exception as exc:
            self.get_logger().error('DNN forward execution failed: %s' % exc)
            return

        end = time.time()
        fps = 1.0 / max(end - start, 1e-6)
        cv.putText(frame, 'FPS : ' + str(int(fps)), (20, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (100, 200, 200), 1)

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_img.publish(out_msg)

        arr = TargetArray()
        for det in detections:
            t = Target()
            t.frame_id = msg.header.frame_id or self.frame_id
            t.stamp = TimeMsg(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec)
            t.scores = float(det['confidence'])
            t.ptx = float(det['ptx'])
            t.pty = float(det['pty'])
            t.distw = float(det['distw'])
            t.disth = float(det['disth'])
            t.centerx = float(det['centerx'])
            t.centery = float(det['centery'])
            arr.data.append(t)
        self.pub_targets.publish(arr)

        if self.gui:
            try:
                cv.imshow('target_detection', frame)
                cv.waitKey(1)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetDetectionNode()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if cv is not None and node is not None and getattr(node, 'gui', False):
            try:
                cv.destroyAllWindows()
            except Exception:
                pass


if __name__ == '__main__':
    main()
