#!/usr/bin/env python3
# encoding: utf-8

import sys
import rclpy
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool

from yahboomcar_linefollw.follow_line import LineDetect

LINE_HSV_BY_ID = {
    23: [(0, 84, 131), (180, 253, 255)],     # red
    24: [(55, 105, 136), (95, 255, 255)],    # green
    25: [(55, 134, 218), (125, 253, 255)],   # blue
    26: [(18, 45, 144), (125, 253, 255)],    # yellow
}


class VoiceLine(LineDetect):
    def __init__(self):
        super().__init__()

        # Initial state setup: default disabled, init tracking state, no nonzero movement
        self.enabled = False
        self.Track_state = 'init'
        self.ros_ctrl.Joy_active = True
        self.model = "General"
        self._publish_vel(0.0, 0.0)

        # Service for enabling/disabling
        self.srv_set_enabled = self.create_service(
            SetBool, '~/set_enabled', self.set_enabled_callback
        )

        # Relative voice topic subscriptions and publisher
        self.sub_voice_cmd = self.create_subscription(
            Int32, 'voice/command', self.voice_command_callback, 10
        )
        self.sub_voice_status = self.create_subscription(
            String, 'voice/status', self.voice_status_callback, 10
        )
        self.pub_voice_playback = self.create_publisher(
            Int32, 'voice/playback', 10
        )

        self.get_logger().info("VoiceLine node initialized (enabled=False, Track_state='init').")

    def set_enabled_callback(self, request, response):
        try:
            if request.data:
                self.enabled = True
                response.success = True
                response.message = "VoiceLine enabled."
                self.get_logger().info("VoiceLine service: enabled.")
            else:
                self._clear_and_disable()
                response.success = True
                response.message = "VoiceLine disabled and stopped."
                self.get_logger().info("VoiceLine service: disabled.")
        except Exception as exc:
            self._fault_handler(f"Exception in set_enabled_callback: {exc}")
            response.success = False
            response.message = f"Error: {exc}"
        return response

    def voice_status_callback(self, msg):
        text = msg.data.lower()
        if any(word in text for word in ('disconnect', 'error', 'disabled', 'unavailable', 'failed')):
            self._fault_handler(msg.data)

    def voice_command_callback(self, msg):
        try:
            cmd = msg.data

            # Stop command (22 or 0) must stop even if disabled
            if cmd == 22 or cmd == 0:
                self.model = "Stop"
                self.Track_state = 'init'
                self.ros_ctrl.Joy_active = True
                self._publish_vel(0.0, 0.0)
                playback_msg = Int32()
                playback_msg.data = cmd
                self.pub_voice_playback.publish(playback_msg)
                self.get_logger().info(f"Stop command received: {cmd}. Robot stopped.")
                return

            # Color tracking commands (23: red, 24: green, 25: blue, 26: yellow)
            if cmd in LINE_HSV_BY_ID:
                if not self.enabled:
                    self.get_logger().warn(
                        f"Command {cmd} received but VoiceLine is disabled. Ignoring."
                    )
                    return

                hsv = LINE_HSV_BY_ID[cmd]
                self.hsv_range = (tuple(hsv[0]), tuple(hsv[1]))
                self.model = "color_follow_line"
                self.Track_state = 'tracking'
                self.ros_ctrl.Joy_active = False

                playback_msg = Int32()
                playback_msg.data = cmd
                self.pub_voice_playback.publish(playback_msg)
                self.get_logger().info(
                    f"Line tracking command {cmd} activated with HSV: {self.hsv_range}"
                )
            else:
                self.get_logger().info(f"Ignored voice command: {cmd}")

        except Exception as exc:
            self._fault_handler(f"Exception in voice_command_callback: {exc}")

    def _clear_and_disable(self):
        self.enabled = False
        self.Track_state = 'init'
        self.model = "Stop"
        self.ros_ctrl.Joy_active = True
        self._publish_vel(0.0, 0.0)

    def _fault_handler(self, reason):
        self.get_logger().error(f"Fault detected ({reason}). Latching disabled state and stopping.")
        self._clear_and_disable()


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = VoiceLine()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node._shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
