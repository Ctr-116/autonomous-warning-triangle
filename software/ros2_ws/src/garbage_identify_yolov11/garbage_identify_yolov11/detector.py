#!/usr/bin/env python3
# coding: utf-8
# Copyright (c) 2021-2024 YAHBOOM.
# Licensed under the Apache License, Version 2.0.
#
# Native ROS2 Humble YOLO publisher.
# - Publishes std_msgs/String 'DetectMsg' of each detected class label, one
#   message per detected label per frame (original subscriber protocol).
# - Preserves ORIGINAL_CONFIDENCE = 0.88.
# - Optional independent ONNX Runtime backend (CPUExecutionProvider) for .onnx,
#   with NO dependency on torch/ultralytics for the .onnx route.
# - Legacy ultralytics YOLO route retained for .pt (and for explicit
#   backend=ultralytics); ultralytics/torch are imported lazily.
# - Old TensorRT .engine files are rejected (not portable across hosts).
# - model_path empty => inference disabled; no fake detections are emitted.
# - Camera image subscription uses BEST_EFFORT QoS.
# - No socket/roslibpy import; no subprocess; no automatic downloads.
import ast
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

ORIGINAL_CONFIDENCE = 0.88

_SUPPORTED_EXT = ('.pt', '.onnx')
_VALID_BACKENDS = ('auto', 'onnxruntime', 'ultralytics')

# Input / preprocessing constants for the ONNX Runtime backend.
_INPUT_SIZE = 640
_PAD_VALUE = 114
_IOU_THRESHOLD = 0.45
_MAX_DET = 100

# YOLO11 (16-class single-stage) native ONNX output layout: [1, 4 + nc, 8400]
# Rows 0..3 are box center-x/center-y/width/height; rows 4.. are per-class
# confidence scores. There is NO objectness row for YOLO11.
_EXPECTED_NC = 16

_FALLBACK_NAMES = [
    'Apple_core',
    'Book',
    'Cigarette_butts',
    'Disposable_chopsticks',
    'Egg_shell',
    'Expired_cosmetics',
    'Expired_tablets',
    'Fish_bone',
    'Newspaper',
    'Old_school_bag',
    'Peach_pit',
    'Syringe',
    'Toliet_paper',
    'Used_batteries',
    'Watermelon_rind',
    'Zip_top_can',
]


def letterbox_bgr(image, new_size=_INPUT_SIZE, pad_value=_PAD_VALUE):
    """Resize a BGR uint8 image to new_size x new_size, preserving aspect
    ratio and padding with pad_value. Returns (padded_bgr, ratio, pad_x, pad_y).
    Pure helper, importable/testable without ONNX Runtime.
    """
    import numpy as np
    import cv2

    if image is None:
        raise ValueError('image is None')
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('expected HxWx3 BGR image')
    h, w = int(image.shape[0]), int(image.shape[1])
    if h <= 0 or w <= 0:
        raise ValueError('image has zero dimension')
    ratio = min(float(new_size) / float(h), float(new_size) / float(w))
    new_w = max(1, int(round(w * ratio)))
    new_h = max(1, int(round(h * ratio)))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_w = new_size - new_w
    pad_h = new_size - new_h
    pad_x = pad_w // 2
    pad_y = pad_h // 2
    padded = cv2.copyMakeBorder(
        resized, pad_y, pad_h - pad_y, pad_x, pad_w - pad_x,
        cv2.BORDER_CONSTANT, value=(int(pad_value),) * 3)
    return padded, ratio, pad_x, pad_y


def bgr_to_nchw_input(padded_bgr):
    """Convert a padded BGR uint8 HxWx3 image into an RGB float32
    [1, 3, H, W] NCHW array normalized to [0, 1]. Pure helper.
    """
    import numpy as np

    if padded_bgr is None:
        raise ValueError('image is None')
    rgb = padded_bgr[:, :, ::-1]
    chw = np.transpose(rgb, (2, 0, 1)).astype(np.float32) / 255.0
    return np.expand_dims(chw, axis=0)


def unletterbox_boxes(boxes_xyxy, ratio, pad_x, pad_y, orig_w, orig_h):
    """Map xyxy boxes from letterboxed input coordinates back to original
    image coordinates and clip to image bounds. boxes_xyxy is Nx4 (any float
    array). Pure helper; returns list of [x1, y1, x2, y2] floats.
    """
    out = []
    if boxes_xyxy is None:
        return out
    for row in boxes_xyxy:
        x1 = (float(row[0]) - pad_x) / ratio
        y1 = (float(row[1]) - pad_y) / ratio
        x2 = (float(row[2]) - pad_x) / ratio
        y2 = (float(row[3]) - pad_y) / ratio
        x1 = min(max(x1, 0.0), float(orig_w))
        y1 = min(max(y1, 0.0), float(orig_h))
        x2 = min(max(x2, 0.0), float(orig_w))
        y2 = min(max(y2, 0.0), float(orig_h))
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1
        out.append([x1, y1, x2, y2])
    return out


def _validate_names(value, expected_nc):
    """Validate a names payload (dict {int: str} or list[str]). Returns a dict
    {int: str} with exactly expected_nc entries, or raises ValueError.
    """
    if isinstance(value, dict):
        if len(value) != expected_nc:
            raise ValueError('names dict has %d entries, expected %d'
                             % (len(value), expected_nc))
        normalized = {}
        for k, v in value.items():
            if not isinstance(k, int):
                raise ValueError('names dict key not int: %r' % (k,))
            if not isinstance(v, str):
                raise ValueError('names dict value not str: %r' % (v,))
            normalized[k] = v
    elif isinstance(value, (list, tuple)):
        if len(value) != expected_nc:
            raise ValueError('names list has %d entries, expected %d'
                             % (len(value), expected_nc))
        normalized = {}
        for i, v in enumerate(value):
            if not isinstance(v, str):
                raise ValueError('names list value not str: %r' % (v,))
            normalized[i] = v
    else:
        raise ValueError('names payload has unsupported type: %s'
                         % type(value).__name__)
    for i in range(expected_nc):
        if i not in normalized:
            raise ValueError('names missing class index %d' % i)
    return normalized


def parse_names_from_metadata(metadata_map, expected_nc=_EXPECTED_NC):
    """Parse ONNX custom_metadata_map['names'] using ast.literal_eval only
    (never eval). Fallback to the exact 16 source YAML class names, preserving
    the original 'Toliet_paper' misspelling. Raises ValueError if ambiguous.
    """
    names = None
    if metadata_map:
        raw = metadata_map.get('names')
        if raw is not None:
            try:
                parsed = ast.literal_eval(raw)
            except (ValueError, SyntaxError) as exc:
                raise ValueError('unparseable names metadata: %s' % exc)
            names = _validate_names(parsed, expected_nc)
    if names is None:
        if expected_nc != len(_FALLBACK_NAMES):
            raise ValueError('no names metadata and fallback count mismatch')
        names = {i: n for i, n in enumerate(_FALLBACK_NAMES)}
    if len(names) != expected_nc:
        raise ValueError('resolved names count %d != expected %d'
                         % (len(names), expected_nc))
    return names


def decode_yolo11_output(raw_output, class_count=_EXPECTED_NC,
                         confidence=ORIGINAL_CONFIDENCE):
    """Interpret a native YOLO11 ONNX output tensor of shape
    [1, 4 + class_count, num_anchors]. Returns a list of candidate dicts:
    {class_id, score, box: [x1, y1, x2, y2]} in input-image coordinates
    (before unletterboxing). Pure helper; no ORT required.
    """
    import numpy as np

    if raw_output is None:
        raise ValueError('output is None')
    arr = np.asarray(raw_output)
    if arr.ndim != 3 or arr.shape[0] != 1:
        raise ValueError('unexpected YOLO output rank/shape: %r'
                         % (arr.shape,))
    _, rows, anchors = arr.shape
    if rows != 4 + class_count:
        raise ValueError('unexpected YOLO output rows=%d (expected %d for %d '
                         'classes); refusing to fabricate detections'
                         % (rows, 4 + class_count, class_count))
    if anchors <= 0:
        raise ValueError('YOLO output has zero anchors')
    arr = arr[0]  # [rows, anchors]
    if not np.all(np.isfinite(arr)):
        raise ValueError('YOLO output contains non-finite values')

    boxes_xywh = arr[0:4, :]          # [4, anchors]
    class_scores = arr[4:4 + class_count, :]  # [class_count, anchors]

    best_scores = np.max(class_scores, axis=0)      # [anchors]
    best_cls = np.argmax(class_scores, axis=0)      # [anchors]

    mask = best_scores >= float(confidence)
    if not np.any(mask):
        return []

    cx = boxes_xywh[0, mask]
    cy = boxes_xywh[1, mask]
    bw = boxes_xywh[2, mask]
    bh = boxes_xywh[3, mask]
    cls = best_cls[mask]
    scores = best_scores[mask]

    x1 = cx - bw / 2.0
    y1 = cy - bh / 2.0
    x2 = cx + bw / 2.0
    y2 = cy + bh / 2.0

    candidates = []
    for i in range(cls.shape[0]):
        candidates.append({
            'class_id': int(cls[i]),
            'score': float(scores[i]),
            'box': [float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i])],
        })
    return candidates


def nms_candidates(candidates, orig_w, orig_h, ratio, pad_x, pad_y,
                   iou_threshold=_IOU_THRESHOLD, max_det=_MAX_DET,
                   score_threshold=ORIGINAL_CONFIDENCE):
    """Apply class-aware cv2.dnn.NMSBoxes in the letterboxed coordinate frame,
    then unletterbox and clip to the original image. Returns list of dicts:
    {label_id, confidence, box: [x1, y1, x2, y2]}. Pure helper.
    """
    import cv2

    if not candidates:
        return []

    xyxy_in = [c['box'] for c in candidates]
    unlettered = unletterbox_boxes(xyxy_in, ratio, pad_x, pad_y, orig_w, orig_h)

    cv_boxes = []
    cv_scores = []
    for box in unlettered:
        w = max(0.0, box[2] - box[0])
        h = max(0.0, box[3] - box[1])
        cv_boxes.append([float(box[0]), float(box[1]), float(w), float(h)])
    cv_scores = [float(c['score']) for c in candidates]

    leftover = list(range(len(candidates)))
    kept = []
    try:
        all_ids = sorted(set(c['class_id'] for c in candidates))
    except AttributeError:
        all_ids = []

    for class_id in all_ids:
        idxs = [i for i in leftover if candidates[i]['class_id'] == class_id]
        if not idxs:
            continue
        class_boxes = [cv_boxes[i] for i in idxs]
        class_scores = [cv_scores[i] for i in idxs]
        result = cv2.dnn.NMSBoxes(
            class_boxes, class_scores, float(score_threshold),
            float(iou_threshold))
        if result is None or len(result) == 0:
            continue
        if hasattr(result, 'flatten'):
            pick = result.flatten().tolist()
        else:
            pick = list(result)
        for local_i in pick:
            global_i = idxs[int(local_i)]
            kept.append(global_i)
            if len(kept) >= max_det:
                break
        if len(kept) >= max_det:
            break

    kept.sort(key=lambda i: -cv_scores[i])
    detections = []
    for i in kept:
        detections.append({
            'class_id': int(candidates[i]['class_id']),
            'confidence': float(candidates[i]['score']),
            'box': unlettered[i],
        })
    return detections


class OnnxYoloBackend(object):
    """Independent YOLO ONNX Runtime backend (CPUExecutionProvider).

    Requires onnxruntime. Does NOT import torch or ultralytics.
    """

    def __init__(self, model_path, confidence=ORIGINAL_CONFIDENCE):
        import onnxruntime as ort

        self.model_path = model_path
        self.confidence = float(confidence)
        self.input_name = None
        self.input_shape = None
        self.class_count = _EXPECTED_NC

        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1

        self._session = ort.InferenceSession(
            model_path,
            sess_options=options,
            providers=['CPUExecutionProvider'])

        inputs = self._session.get_inputs()
        if not inputs:
            raise ValueError('ONNX model exposes no inputs')
        if len(inputs) != 1:
            raise ValueError('ONNX model has %d inputs; expected 1'
                             % len(inputs))
        inp = inputs[0]
        self.input_name = inp.name
        self.input_shape = list(inp.shape)

        try:
            metadata = self._session.get_modelmeta().custom_metadata_map or {}
        except Exception:  # pylint: disable=broad-except
            metadata = {}

        self.names = parse_names_from_metadata(metadata, self.class_count)

        self._output_name = None
        outs = self._session.get_outputs()
        if not outs:
            raise ValueError('ONNX model exposes no outputs')
        self._output_name = outs[0].name

        self._validate_input_shape()

    def _validate_input_shape(self):
        shape = self.input_shape
        if not shape or len(shape) != 4:
            raise ValueError('unexpected ONNX input rank: %r' % (shape,))
        # Batch dimension may be dynamic.
        if shape[1] not in (3, None, 'None', 'batch', 'N'):
            if isinstance(shape[1], int) and shape[1] != 3:
                raise ValueError('ONNX input channels=%r (expected 3)'
                                 % (shape[1],))
        for dim in shape[2:]:
            if isinstance(dim, int) and dim != _INPUT_SIZE:
                raise ValueError('ONNX input spatial dim=%d (expected %d)'
                                 % (dim, _INPUT_SIZE))

    def infer(self, image, confidence=None):
        """Run inference on a BGR uint8 image. Returns a bounded, finite list
        of dicts {label, confidence, box: [x1, y1, x2, y2]} in original image
        pixel coordinates. Raises ValueError on malformed model/output.
        """
        import numpy as np

        conf = self.confidence if confidence is None else float(confidence)
        padded, ratio, pad_x, pad_y = letterbox_bgr(image, _INPUT_SIZE,
                                                    _PAD_VALUE)
        nchw = bgr_to_nchw_input(padded)
        if not np.all(np.isfinite(nchw)):
            raise ValueError('preprocessed input contains non-finite values')

        raw = self._session.run([self._output_name],
                                {self.input_name: nchw})
        if not raw:
            raise ValueError('ONNX session returned no outputs')

        candidates = decode_yolo11_output(raw[0], self.class_count, conf)
        h, w = int(image.shape[0]), int(image.shape[1])
        detections = nms_candidates(candidates, w, h, ratio, pad_x, pad_y,
                                    score_threshold=conf)

        results = []
        for d in detections:
            box = d['box']
            if not all(np.isfinite(box)):
                continue
            label = self.names.get(d['class_id'])
            if label is None:
                raise ValueError('class id %d has no label name' % d['class_id'])
            results.append({
                'label': label,
                'confidence': float(d['confidence']),
                'box': [float(box[0]), float(box[1]),
                        float(box[2]), float(box[3])],
            })
        return results


class YoloDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('garbage_yolo_detector', **kwargs)
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', 'camera/color/image_raw')
        self.declare_parameter('detect_topic', 'DetectMsg')
        self.declare_parameter('confidence', ORIGINAL_CONFIDENCE)
        self.declare_parameter('publish_period_s', 0.1)
        self.declare_parameter('backend', 'auto')

        self.image_topic = self.get_parameter('image_topic').value
        self.detect_topic = self.get_parameter('detect_topic').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.publish_period_s = float(
            self.get_parameter('publish_period_s').value)
        self.backend_name = str(self.get_parameter('backend').value).strip()

        self.model_path = self.get_parameter('model_path').value or ''
        self.model = None
        self.backend = None
        self.backend_kind = None
        self.enabled = False
        self._frames_received = 0
        self._last_publish_time = None

        self._validate_confidence()
        self._validate_publish_period()
        self._resolve_model_path()
        self._validate_backend_argument()
        self._validate_model()

        self.pub = self.create_publisher(String, self.detect_topic, 10)
        best_effort = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            Image, self.image_topic, self.on_image, best_effort)

        if self.enabled:
            self.get_logger().info(
                'YOLO detector ready: backend=%s model=%s conf=%.2f topic=%s'
                % (self.backend_kind, self.model_path, self.confidence,
                   self.detect_topic))
        else:
            self.get_logger().warn(
                'YOLO inference DISABLED (invalid/empty model_path). '
                'Publishing is disabled; no fake detections will be emitted.')

    # ----- configuration validation -------------------------------------

    def _validate_confidence(self):
        if (self.confidence != self.confidence  # NaN
                or self.confidence <= 0.0
                or self.confidence > 1.0):
            self.get_logger().error(
                'invalid confidence %.4f; falling back to %.2f'
                % (self.confidence, ORIGINAL_CONFIDENCE))
            self.confidence = ORIGINAL_CONFIDENCE

    def _validate_publish_period(self):
        if (not math.isfinite(self.publish_period_s)
                or self.publish_period_s <= 0.0):
            self.get_logger().error(
                'invalid publish_period_s %.4f; reset to 0.1'
                % self.publish_period_s)
            self.publish_period_s = 0.1

    def _validate_backend_argument(self):
        if self.backend_name not in _VALID_BACKENDS:
            self.get_logger().error(
                'invalid backend=%r; using auto' % (self.backend_name,))
            self.backend_name = 'auto'

    def _resolve_model_path(self):
        """Resolve explicit basename or package share 'models/...' under
        ament_share/garbage_identify_yolov11. Accept source paths too.
        """
        if not self.model_path:
            return
        raw = str(self.model_path).strip()
        if not raw:
            self.model_path = ''
            return

        if os.path.isfile(raw):
            self.model_path = os.path.abspath(raw)
            return

        candidates = []
        # ament share install location.
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('garbage_identify_yolov11')
        except Exception:  # pylint: disable=broad-except
            share_dir = None
        if share_dir:
            models_dir = os.path.join(share_dir, 'models')
            candidates.append(os.path.join(models_dir, raw))
            candidates.append(os.path.join(share_dir, raw))
        # Source tree fallback: same dir as this file / package root.
        here = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(here)
        candidates.append(os.path.join(pkg_root, 'models', raw))
        candidates.append(os.path.join(pkg_root, raw))
        candidates.append(os.path.join(here, raw))
        for cand in candidates:
            if os.path.isfile(cand):
                self.model_path = os.path.abspath(cand)
                return
        # Leave unresolved; _validate_model will log the error.
        self.model_path = raw

    def _validate_model(self):
        if not self.model_path:
            self.enabled = False
            return
        if not os.path.isfile(self.model_path):
            self.get_logger().error(
                'model_path not found: %s' % self.model_path)
            self.enabled = False
            return
        lower = self.model_path.lower()
        if lower.endswith('.engine'):
            self.get_logger().error(
                'Old TensorRT .engine not portable across hosts; use .pt or '
                '.onnx')
            self.enabled = False
            return
        if not lower.endswith(_SUPPORTED_EXT):
            self.get_logger().error(
                'Unsupported model extension: %s' % self.model_path)
            self.enabled = False
            return

        resolved_backend = self.backend_name
        if resolved_backend == 'auto':
            if lower.endswith('.onnx'):
                resolved_backend = 'onnxruntime'
            elif lower.endswith('.pt'):
                resolved_backend = 'ultralytics'
            else:
                self.get_logger().error(
                    'cannot auto-select backend for: %s' % self.model_path)
                self.enabled = False
                return

        if resolved_backend == 'onnxruntime':
            if not lower.endswith('.onnx'):
                self.get_logger().error(
                    'onnxruntime backend requires .onnx model: %s'
                    % self.model_path)
                self.enabled = False
                return
            self._load_onnxruntime_backend()
        elif resolved_backend == 'ultralytics':
            if not lower.endswith('.pt'):
                self.get_logger().error(
                    'ultralytics backend requires .pt model: %s'
                    % self.model_path)
                self.enabled = False
                return
            self._load_ultralytics_backend()
        else:
            self.get_logger().error(
                'unresolved backend for: %s' % self.model_path)
            self.enabled = False

    def _load_onnxruntime_backend(self):
        try:
            self.backend = OnnxYoloBackend(self.model_path, self.confidence)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(
                'ONNX Runtime backend load failed: %s' % exc)
            self.backend = None
            self.backend_kind = None
            self.enabled = False
            return
        self.backend_kind = 'onnxruntime'
        self.enabled = True

    def _load_ultralytics_backend(self):
        try:
            from ultralytics import YOLO  # lazy import, legacy .pt route
        except ImportError as exc:
            self.get_logger().error('ultralytics not available: %s' % exc)
            self.model = None
            self.enabled = False
            return
        try:
            self.model = YOLO(self.model_path)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error('YOLO load failed: %s' % exc)
            self.model = None
            self.enabled = False
            return
        self.backend_kind = 'ultralytics'
        self.enabled = True

    # ----- image callback ------------------------------------------------

    def on_image(self, msg: Image):
        if not self.enabled:
            return
        now = time.monotonic()
        if self._last_publish_time is not None and now - self._last_publish_time < self.publish_period_s:
            return
        self._last_publish_time = now
        self._frames_received += 1
        try:
            import numpy as np
            from cv_bridge import CvBridge
        except ImportError as exc:
            self.get_logger().error('cv_bridge/numpy missing: %s' % exc)
            return

        bridge = CvBridge()
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn('cv_bridge conversion failed: %s' % exc)
            return

        if not isinstance(cv_img, np.ndarray) or cv_img.size == 0:
            self.get_logger().warn('cv_bridge returned empty image')
            return

        detections = self._run_inference(cv_img)
        if detections is None:
            return

        emitted = set()
        for det in detections:
            label = det['label']
            if label in emitted:
                continue
            emitted.add(label)
            self.pub.publish(String(data=str(label)))

    def _run_inference(self, cv_img):
        if self.backend_kind == 'onnxruntime':
            if self.backend is None:
                return None
            try:
                return self.backend.infer(cv_img, self.confidence)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(
                    'ONNX inference failed: %s' % exc)
                return None
        if self.backend_kind == 'ultralytics':
            if self.model is None:
                return None
            try:
                results = self.model.predict(
                    cv_img, conf=self.confidence, verbose=False)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error('YOLO predict failed: %s' % exc)
                return None
            return self._decode_ultralytics_results(results)
        return None

    def _decode_ultralytics_results(self, results):
        if not results:
            return []
        names = getattr(self.model, 'names', None) \
            or getattr(results[0], 'names', {})
        out = []
        r = results[0]
        boxes = getattr(r, 'boxes', None)
        if boxes is None:
            return out
        for box in boxes:
            try:
                cls_id = int(box.cls[0]) if hasattr(box, 'cls') else int(box.cls)
            except Exception:  # pylint: disable=broad-except
                continue
            if isinstance(names, dict):
                label = names.get(cls_id, str(cls_id))
            else:
                label = names[cls_id] if cls_id < len(names) else str(cls_id)
            try:
                conf_val = float(box.conf[0]) if hasattr(box, 'conf') \
                    else float(box.conf)
            except Exception:  # pylint: disable=broad-except
                conf_val = self.confidence
            out.append({
                'label': str(label),
                'confidence': conf_val,
                'box': None,
            })
        return out


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
