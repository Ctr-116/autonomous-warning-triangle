#!/usr/bin/env python3
# coding: utf-8
"""Serial transport and ROS 2 bridge for the Yahboom speech module.

This module intentionally does not import the legacy SDK ``Speech`` class.
It speaks the module's serial wire protocol directly:

  TX: b"$" + b"A" + <three ASCII decimal digits> + b"#"
      (exactly 6 bytes, e.g. playback code 7 -> b"$A007#")

  RX: The historical SDK parsed ``str(data)[4:7]`` which corresponds to a
      two-character prefix followed by three ASCII decimal digits. The exact
      response header is not proven, so the decoder accepts any frame of the
      form b"$" + <ASCII letter> + <3 digits> + b"#" by default. An optional
      prefix filter can restrict reception to a known header.

The parser is incremental, bounded (256-byte buffer), resynchronises on bad
bytes / partial frames / multiple frames, supports an optional 2-second frame
expiry, and returns a list of integer codes. Code 999 is an idle marker and is
ignored at the node layer.
"""

from __future__ import annotations

import functools
import time
from typing import Callable, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

try:  # pyserial is optional so the node can be inspected without hardware.
    import serial  # type: ignore
except Exception:  # pragma: no cover - exercised only when pyserial absent
    serial = None


IDLE_CODE = 999
MAX_BUFFER = 256
DEFAULT_FRAME_EXPIRY = 2.0
FRAME_LEN = 6
TX_HEADER = 0x24  # '$'
TX_ADDR = 0x41    # 'A'
TX_TAIL = 0x23    # '#'


def encode_playback(code: int) -> bytes:
    """Encode a playback code into the 6-byte TX frame.

    ``code`` must be an int in [0, 999]. The output is exactly
    b"$A" + three ASCII decimal digits + b"#".
    """
    if isinstance(code, bool) or not isinstance(code, int):
        raise TypeError("playback code must be an int in [0, 999]")
    if code < 0 or code > 999:
        raise ValueError("playback code must be in [0, 999]")
    return bytes(
        (
            TX_HEADER,
            TX_ADDR,
            ord("0") + (code // 100) % 10,
            ord("0") + (code // 10) % 10,
            ord("0") + (code % 10),
            TX_TAIL,
        )
    )


class SpeechDecoder:
    """Incremental, bounded decoder for speech module response frames.

    Accepted frame shape (default)::

        b"$" + <ASCII letter> + <digit> + <digit> + <digit> + b"#"

    ``prefix_filter`` (a ``bytes`` object) may be supplied to restrict the
    two leading bytes to an exact value, for example ``b"$A"``. If it is
    ``None`` the second byte may be any ASCII letter.

    ``feed`` returns a list of integer codes extracted from complete frames.
    Malformed bytes and partial frames are discarded with resynchronisation
    on the next ``$``. At most ``max_buffer`` bytes are retained. When
    ``frame_expiry`` is a positive number, a partial frame older than that
    many seconds is dropped before processing new data.
    """

    def __init__(
        self,
        max_buffer: int = MAX_BUFFER,
        frame_expiry: Optional[float] = DEFAULT_FRAME_EXPIRY,
        prefix_filter: Optional[bytes] = None,
    ) -> None:
        if not isinstance(max_buffer, int) or isinstance(max_buffer, bool):
            raise TypeError("max_buffer must be an int")
        if max_buffer < FRAME_LEN:
            raise ValueError("max_buffer must be at least 6")
        self.max_buffer = max_buffer
        if frame_expiry is not None:
            if frame_expiry <= 0:
                frame_expiry = None
        self.frame_expiry = frame_expiry
        if prefix_filter is not None:
            if not isinstance(prefix_filter, (bytes, bytearray)):
                raise TypeError("prefix_filter must be bytes or None")
            prefix_filter = bytes(prefix_filter)
            if len(prefix_filter) != 2:
                raise ValueError("prefix_filter must contain exactly 2 bytes")
        self.prefix_filter = prefix_filter
        self._buffer = bytearray()
        self._last_byte_time: Optional[float] = None

    def reset(self) -> None:
        """Discard any buffered partial data."""
        self._buffer.clear()
        self._last_byte_time = None

    def _now(self, now: Optional[float]) -> float:
        return time.monotonic() if now is None else float(now)

    def _expire_if_needed(self, now: float) -> None:
        if self.frame_expiry is None:
            return
        if not self._buffer:
            return
        if self._last_byte_time is not None and (now - self._last_byte_time) > self.frame_expiry:
            self._buffer.clear()
            self._last_byte_time = None

    @staticmethod
    def _is_digit(value: int) -> bool:
        return 0x30 <= value <= 0x39

    @staticmethod
    def _is_letter(value: int) -> bool:
        return 0x41 <= value <= 0x5A or 0x61 <= value <= 0x7A

    def _header_matches(self, first: int, second: int) -> bool:
        if first != TX_HEADER:
            return False
        if self.prefix_filter is not None:
            return first == self.prefix_filter[0] and second == self.prefix_filter[1]
        return self._is_letter(second)

    def _decode_once(self) -> Optional[int]:
        """Attempt to decode one frame from the head of the buffer.

        Returns the code when a frame is available, otherwise ``None``.
        Invalid leading bytes are dropped and the buffer is re-examined by
        the caller. Returns ``None`` also when more data is required, leaving
        the buffer untouched.
        """
        if not self._buffer:
            return None
        if self._buffer[0] != TX_HEADER:
            del self._buffer[0]
            return None
        if len(self._buffer) < 2:
            return None
        if self.prefix_filter is not None:
            if len(self._buffer) < 2:
                return None
            if not self._header_matches(self._buffer[0], self._buffer[1]):
                del self._buffer[0]
                return None
        else:
            if not self._is_letter(self._buffer[1]):
                del self._buffer[0]
                return None
        if len(self._buffer) < FRAME_LEN:
            return None
        frame = self._buffer[:FRAME_LEN]
        if frame[5] != TX_TAIL:
            del self._buffer[0]
            return None
        if not (
            self._is_digit(frame[2])
            and self._is_digit(frame[3])
            and self._is_digit(frame[4])
        ):
            del self._buffer[0]
            return None
        code = (frame[2] - 0x30) * 100 + (frame[3] - 0x30) * 10 + (frame[4] - 0x30)
        del self._buffer[:FRAME_LEN]
        return code

    def feed(self, data: bytes, now: Optional[float] = None) -> List[int]:
        """Feed raw bytes and return a list of decoded integer codes."""
        if data is None:
            return []
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("data must be bytes-like")
        raw = bytes(data)
        current = self._now(now)
        self._expire_if_needed(current)
        codes: List[int] = []
        if raw:
            self._buffer.extend(raw)
            self._last_byte_time = current
        if len(self._buffer) > self.max_buffer:
            excess = len(self._buffer) - self.max_buffer
            del self._buffer[:excess]
            self._last_byte_time = current
        while True:
            before = len(self._buffer)
            code = self._decode_once()
            if code is not None:
                codes.append(code)
                continue
            if len(self._buffer) == before:
                break
        return codes


class SpeechTransportNode(Node):
    """ROS 2 node bridging the speech module serial port and messages."""

    def __init__(self) -> None:
        super().__init__("speech_transport")

        self.declare_parameter("enable_serial", False)
        self.declare_parameter("port", "/dev/myspeech")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("read_period", 0.05)
        self.declare_parameter("frame_expiry", DEFAULT_FRAME_EXPIRY)
        self.declare_parameter("max_buffer", MAX_BUFFER)
        self.declare_parameter("prefix_filter", "")

        self._enable_serial = bool(self.get_parameter("enable_serial").value)
        self._port = str(self.get_parameter("port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._read_period = float(self.get_parameter("read_period").value)
        frame_expiry_param = self.get_parameter("frame_expiry").value
        self._frame_expiry = (
            None if frame_expiry_param is None else float(frame_expiry_param)
        )
        self._max_buffer = int(self.get_parameter("max_buffer").value)
        prefix_param = self.get_parameter("prefix_filter").value
        prefix_bytes: Optional[bytes]
        if prefix_param is None or prefix_param == "":
            prefix_bytes = None
        else:
            prefix_bytes = str(prefix_param).encode("latin-1", "replace")
            if len(prefix_bytes) != 2:
                self.get_logger().warn(
                    "prefix_filter must be exactly 2 bytes; ignoring value"
                )
                prefix_bytes = None

        self._decoder = SpeechDecoder(
            max_buffer=self._max_buffer,
            frame_expiry=self._frame_expiry,
            prefix_filter=prefix_bytes,
        )
        self._serial_port = None
        self._serial_open_failed = False
        self._last_error = ""

        self._voice_pub = self.create_publisher(Int32, "voice/command", 10)
        self._status_pub = self.create_publisher(String, "voice/status", 10)
        self._playback_sub = self.create_subscription(
            Int32, "voice/playback", self._on_playback, 10
        )

        self._timer = None
        if self._enable_serial:
            self._open_serial()
            period = self._read_period if self._read_period > 0 else 0.05
            self._timer = self.create_timer(period, self._on_timer)
        else:
            self._publish_status("serial disabled")

    # ------------------------------------------------------------------
    # Serial lifecycle
    # ------------------------------------------------------------------
    def _open_serial(self) -> None:
        if serial is None:
            self._serial_open_failed = True
            self._last_error = "pyserial not available"
            self._publish_status("serial open failed: pyserial not available")
            self.get_logger().error(
                "Speech serial open failed: pyserial is not installed"
            )
            return
        try:
            self._serial_port = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0,
                write_timeout=0.05,
                exclusive=True,
            )
            self._serial_open_failed = False
            self._last_error = ""
            self._publish_status("serial open ok: %s" % self._port)
            self.get_logger().info(
                "Speech serial opened on %s at %d baud"
                % (self._port, self._baudrate)
            )
        except Exception as exc:  # noqa: BLE001 - report any open failure
            self._serial_port = None
            self._serial_open_failed = True
            self._last_error = str(exc)
            self._publish_status("serial open failed: %s" % exc)
            self.get_logger().error(
                "Speech serial open failed on %s: %s" % (self._port, exc)
            )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _serial_ready(self) -> bool:
        return self._serial_port is not None

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_playback(self, msg: Int32) -> None:
        if not self._enable_serial:
            return
        if not self._serial_ready():
            self._publish_status("playback ignored: serial not open")
            return
        try:
            frame = encode_playback(int(msg.data))
        except (TypeError, ValueError) as exc:
            self._publish_status("playback invalid: %s" % exc)
            return
        try:
            self._serial_port.write(frame)
            self._serial_port.flush()
        except Exception as exc:  # noqa: BLE001 - report write errors
            self._last_error = str(exc)
            self._publish_status("serial write error: %s" % exc)
            self.get_logger().error("Speech serial write failed: %s" % exc)

    def _on_timer(self) -> None:
        if not self._serial_ready():
            if self._serial_open_failed:
                return
            self._publish_status("serial unavailable")
            return
        try:
            waiting = self._serial_port.in_waiting
            if not waiting:
                return
            raw = self._serial_port.read(waiting)
        except Exception as exc:  # noqa: BLE001 - report read errors
            self._last_error = str(exc)
            self._publish_status("serial read error: %s" % exc)
            self.get_logger().error("Speech serial read failed: %s" % exc)
            return
        if not raw:
            return
        try:
            codes = self._decoder.feed(raw)
        except Exception as exc:  # noqa: BLE001 - decoder must not kill timer
            self._publish_status("decode error: %s" % exc)
            self.get_logger().error("Speech decode error: %s" % exc)
            return
        for code in codes:
            if code == IDLE_CODE:
                continue
            out = Int32()
            out.data = int(code)
            self._voice_pub.publish(out)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def _close_serial(self) -> None:
        if self._serial_port is not None:
            try:
                self._serial_port.close()
            except Exception:  # noqa: BLE001 - shutdown best effort
                pass
            self._serial_port = None

    def destroy_node(self) -> bool:
        if self._timer is not None:
            try:
                self.destroy_timer(self._timer)
            except Exception:  # noqa: BLE001 - shutdown best effort
                pass
            self._timer = None
        self._close_serial()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[SpeechTransportNode] = None
    try:
        node = SpeechTransportNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001 - shutdown best effort
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 - shutdown best effort
            pass


if __name__ == "__main__":
    main()
