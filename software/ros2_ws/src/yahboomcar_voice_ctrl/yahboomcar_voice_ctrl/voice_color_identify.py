#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 Humble Node for Voice-Triggered Color Identification (Image-Only).

Original ROS 1 behavior relied on an OpenCV GUI mouse callback to draw a ROI,
and used a fragile H_min/H_max heuristic directly inside the video loop:
    if H_min == 0 and H_max == 179: red
    elif 23 <= H_min <= 56: yellow
    elif H_min >= 56 and S_min < 200: green
    elif H_min >= 60 and S_min > 200: blue

Practical Adaptation:
- Reconciles the interactive UI ROI requirement by providing a validated,
  configurable parameter 'roi' = [x0, y0, x1, y1] with default [240, 160, 400, 320].
  This removes any mandatory GUI/mouse dependency for headless and embedded robotics
  while allowing future GUI overlays or dynamic reconfiguration to set the ROI.
- Replaces ambiguous min/max boundary checks with a robust dominant HSV color
  classification algorithm over valid saturated/bright pixels that handles red hue
  wrapping across 0/180 degrees.
- Exposes pure classification helper functions (`classify_hsv` and `classify_roi`).
- Evaluates incoming command 60 strictly once per fresh request, checking image
  freshness within image_timeout (0.5s).
- Publishes speech playback IDs:
    61: Red
    62: Blue
    63: Green
    64: Yellow
"""

from typing import List, Optional, Tuple
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool


# Speech command and playback ID definitions
CMD_COLOR_IDENTIFY = 60
VOICE_PLAYBACK_RED = 61
VOICE_PLAYBACK_BLUE = 62
VOICE_PLAYBACK_GREEN = 63
VOICE_PLAYBACK_YELLOW = 64

COLOR_PLAYBACK_MAP = {
    'red': VOICE_PLAYBACK_RED,
    'blue': VOICE_PLAYBACK_BLUE,
    'green': VOICE_PLAYBACK_GREEN,
    'yellow': VOICE_PLAYBACK_YELLOW,
}


def classify_hsv(
    h: float,
    s: float,
    v: float,
    min_saturation: float = 60.0,
    min_value: float = 60.0,
) -> Optional[str]:
    """
    Pure helper to classify an individual HSV pixel (OpenCV scale: H 0-180, S 0-255, V 0-255).

    Traditional calibrated HSV boundaries:
      - Red: [0, 10] U [160, 180]
      - Yellow: [11, 35]
      - Green: [36, 85]
      - Blue: [86, 130]

    Returns 'red', 'yellow', 'green', 'blue', or None if desaturated/too dark.
    """
    if s < min_saturation or v < min_value:
        return None

    # Red hue wrapping
    if (0.0 <= h <= 10.0) or (160.0 <= h <= 180.0):
        return 'red'
    elif 11.0 <= h <= 35.0:
        return 'yellow'
    elif 36.0 <= h <= 85.0:
        return 'green'
    elif 86.0 <= h <= 130.0:
        return 'blue'

    return None


def classify_roi(
    bgr_image: np.ndarray,
    roi: List[int],
    min_saturation: int = 60,
    min_value: int = 60,
) -> Optional[str]:
    """
    Pure helper to classify dominant color inside a validated ROI of a BGR image.

    Args:
        bgr_image: np.ndarray (H, W, 3) in BGR format.
        roi: [x0, y0, x1, y1] coordinates.
        min_saturation: Minimum saturation threshold.
        min_value: Minimum value threshold.

    Returns:
        Dominant color name ('red', 'blue', 'green', 'yellow') or None.
    """
    if bgr_image is None or bgr_image.size == 0 or len(bgr_image.shape) != 3:
        return None

    img_h, img_w = bgr_image.shape[:2]
    if len(roi) != 4:
        return None

    x0, y0, x1, y1 = roi
    x_min, x_max = min(x0, x1), max(x0, x1)
    y_min, y_max = min(y0, y1), max(y0, y1)

    # Clamp to image boundaries
    x_min = max(0, min(x_min, img_w))
    x_max = max(0, min(x_max, img_w))
    y_min = max(0, min(y_min, img_h))
    y_max = max(0, min(y_max, img_h))

    if (x_max - x_min) <= 0 or (y_max - y_min) <= 0:
        return None

    roi_patch = bgr_image[y_min:y_max, x_min:x_max]
    if roi_patch.size == 0:
        return None

    hsv = cv2.cvtColor(roi_patch, cv2.COLOR_BGR2HSV)
    h_channel = hsv[:, :, 0]
    s_channel = hsv[:, :, 1]
    v_channel = hsv[:, :, 2]

    # Mask out low saturation and low brightness pixels
    valid_mask = (s_channel >= min_saturation) & (v_channel >= min_value)
    total_valid = np.count_nonzero(valid_mask)
    if total_valid == 0:
        return None

    # Calibrated traditional HSV masks
    red_mask = valid_mask & ((h_channel <= 10) | (h_channel >= 160))
    yellow_mask = valid_mask & ((h_channel >= 11) & (h_channel <= 35))
    green_mask = valid_mask & ((h_channel >= 36) & (h_channel <= 85))
    blue_mask = valid_mask & ((h_channel >= 86) & (h_channel <= 130))

    counts = {
        'red': int(np.count_nonzero(red_mask)),
        'yellow': int(np.count_nonzero(yellow_mask)),
        'green': int(np.count_nonzero(green_mask)),
        'blue': int(np.count_nonzero(blue_mask)),
    }

    dominant_color, dominant_count = max(counts.items(), key=lambda item: item[1])
    # Must have a significant majority of colored pixels in the valid region
    if dominant_count == 0 or (dominant_count / float(total_valid)) < 0.25:
        return None

    return dominant_color


class VoiceColorIdentify(Node):
    """
    Native ROS 2 Node for Voice-Controlled Color Identification.
    """

    def __init__(self) -> None:
        super().__init__('voice_color_identify')

        # Declare configurable parameters
        self.declare_parameter('roi', [240, 160, 400, 320])
        self.declare_parameter('image_timeout', 0.5)
        self.declare_parameter('min_saturation', 60)
        self.declare_parameter('min_value', 60)

        self.bridge = CvBridge()
        self.enabled: bool = False
        self.latest_cv_image: Optional[np.ndarray] = None
        self.latest_image_time: Optional[rclpy.time.Time] = None

        # Best effort QoS for sensor camera streams
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Standard reliable QoS for command/status/playback
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self._image_callback,
            best_effort_qos,
        )
        self.command_sub = self.create_subscription(
            Int32,
            'voice/command',
            self._command_callback,
            reliable_qos,
        )
        self.status_sub = self.create_subscription(
            String,
            'voice/status',
            self._status_callback,
            reliable_qos,
        )

        # Publisher for speech playback results
        self.playback_pub = self.create_publisher(
            Int32,
            'voice/playback',
            reliable_qos,
        )

        # Service to enable/disable identification mode
        self.set_enabled_srv = self.create_service(
            SetBool,
            '~/set_enabled',
            self._handle_set_enabled,
        )

        self.get_logger().info('VoiceColorIdentify node initialized and ready.')

    def _handle_set_enabled(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.enabled = request.data
        if not self.enabled:
            self._clear_frame()
        response.success = True
        response.message = f'Enabled state set to {self.enabled}'
        self.get_logger().info(response.message)
        return response

    def _clear_frame(self) -> None:
        """Clear cached frame and timestamps on disable or faults."""
        self.latest_cv_image = None
        self.latest_image_time = None

    def _image_callback(self, msg: Image) -> None:
        """Receive and safely store the latest image frame."""
        if not self.enabled:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if cv_img is None or cv_img.size == 0:
                self.get_logger().warn('Received empty image frame.')
                return
            self.latest_cv_image = cv_img
            self.latest_image_time = self.get_clock().now()
        except (CvBridgeError, Exception) as ex:
            self.get_logger().error(f'Failed to convert image frame: {ex}')
            self._clear_frame()

    def _status_callback(self, msg: String) -> None:
        """Handle voice status updates."""
        status_text = msg.data.strip().lower()
        if status_text in ['error', 'fault', 'disconnect', 'disconnected']:
            self.get_logger().warn(f'Fault/disconnect status received: {status_text}. Disabling.')
            self.enabled = False
            self._clear_frame()

    def _get_validated_roi(self, img_shape: Tuple[int, ...]) -> Optional[List[int]]:
        """Validate and clamp configured ROI against image shape."""
        raw_roi = self.get_parameter('roi').value
        if not isinstance(raw_roi, (list, tuple)) or len(raw_roi) != 4:
            self.get_logger().error(f'Invalid ROI parameter format: {raw_roi}')
            return None

        for val in raw_roi:
            if not isinstance(val, (int, float)) or not np.isfinite(val):
                self.get_logger().error('ROI contains non-finite or invalid coordinates.')
                return None

        h, w = img_shape[:2]
        x0, y0, x1, y1 = [int(v) for v in raw_roi]
        x_min, x_max = min(x0, x1), max(x0, x1)
        y_min, y_max = min(y0, y1), max(y0, y1)

        x_min = max(0, min(x_min, w))
        x_max = max(0, min(x_max, w))
        y_min = max(0, min(y_min, h))
        y_max = max(0, min(y_max, h))

        if (x_max - x_min) <= 0 or (y_max - y_min) <= 0:
            self.get_logger().warn(f'Degenerate or out-of-bounds ROI: {raw_roi}')
            return None

        return [x_min, y_min, x_max, y_max]

    def _command_callback(self, msg: Int32) -> None:
        """
        Handle incoming voice command. Evaluates at most once per request.
        Only publishes if node is enabled and image is fresh.
        """
        cmd_id = msg.data
        if cmd_id != CMD_COLOR_IDENTIFY:
            return

        if not self.enabled:
            self.get_logger().warn('Received identify command 60, but node is currently disabled.')
            return

        # Check if an image is present
        if self.latest_cv_image is None or self.latest_image_time is None:
            self.get_logger().warn('Command 60 rejected: missing image frame.')
            return

        # Check image freshness
        now = self.get_clock().now()
        age_sec = (now - self.latest_image_time).nanoseconds / 1e9
        timeout = float(self.get_parameter('image_timeout').value)
        if age_sec < 0 or age_sec > timeout:
            self.get_logger().warn(f'Command 60 rejected: stale image frame (age: {age_sec:.3f}s > {timeout}s).')
            return

        # Validate ROI
        roi = self._get_validated_roi(self.latest_cv_image.shape)
        if roi is None:
            self.get_logger().warn('Command 60 rejected: invalid or empty ROI.')
            return

        # Perform color classification
        min_sat = int(self.get_parameter('min_saturation').value)
        min_val = int(self.get_parameter('min_value').value)

        color_result = classify_roi(
            self.latest_cv_image,
            roi,
            min_saturation=min_sat,
            min_value=min_val,
        )

        if color_result is None or color_result not in COLOR_PLAYBACK_MAP:
            self.get_logger().warn('Color classification produced no confident result. No response published.')
            return

        playback_id = COLOR_PLAYBACK_MAP[color_result]
        pub_msg = Int32()
        pub_msg.data = playback_id
        self.playback_pub.publish(pub_msg)
        self.get_logger().info(f'Identified color "{color_result}", published voice/playback: {playback_id}')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VoiceColorIdentify()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
