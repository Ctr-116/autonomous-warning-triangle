"""Exercise the actual supplied SDK encoder and adversarial serial input."""

import importlib.util
from pathlib import Path
import queue
import struct
import sys
import threading
import time
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'x3plus_base'))
from x3plus_base.transport import FrameDecoder, RosmasterTransport


def vendor_sdk():
    # Its import needs pyserial, but these tests inject FakeSerial and never
    # instantiate the original hardware-opening constructor.
    inserted = 'serial' not in sys.modules
    if inserted:
        sys.modules['serial'] = types.ModuleType('serial')
    try:
        path = ROOT / 'vendor' / 'py_install' / 'Rosmaster_Lib' / 'Rosmaster_Lib.py'
        spec = importlib.util.spec_from_file_location('_sdk_encoder_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.Rosmaster
    finally:
        if inserted:
            del sys.modules['serial']


def report(function, payload):
    body = bytes((len(payload) + 3, function)) + payload
    return b'\xff\xfb' + body + bytes((sum(body) & 0xFF,))


def until(predicate, timeout=0.8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError('condition not reached within bounded test timeout')


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.input = queue.Queue()
        self.writes = []
        self.closed = False
        self.fail_write = False
        self.short_write = False
        self.block_next = False
        self.entered_write = threading.Event()
        self.release_write = threading.Event()

    def read(self, count):
        try:
            value = self.input.get(timeout=0.01)
        except queue.Empty:
            return b''
        if isinstance(value, Exception):
            raise value
        return value

    def write(self, data):
        if self.block_next:
            self.block_next = False
            self.entered_write.set()
            self.release_write.wait(0.2)
        if self.fail_write:
            raise OSError('synthetic write disconnect')
        data = bytes(data)
        self.writes.append(data)
        return len(data) - 1 if self.short_write else len(data)

    def cancel_read(self):
        self.input.put(b'')

    def close(self):
        self.closed = True
        self.input.put(b'')


class DecoderTests(unittest.TestCase):
    def test_split_and_coalesced_frames(self):
        decoder = FrameDecoder()
        speed = struct.pack('<hhhB', 200, -50, 400, 123)
        frame = report(0x0A, speed)
        self.assertEqual(decoder.feed(b'noise' + frame[:6], 10), [])
        result = decoder.feed(frame[6:] + report(0x0C, bytes(6)), 10.01)
        self.assertEqual(result, [(0x0A, speed), (0x0C, bytes(6))])

    def test_corrupt_checksum_and_length_resynchronize(self):
        decoder = FrameDecoder()
        frame = report(0x0A, struct.pack('<hhhB', 100, 0, 0, 120))
        corrupt = frame[:-1] + bytes((frame[-1] ^ 0x7F,))
        data = b'\xff\xfb\x00\x0a' + b'\xff\xfb\x40\x0a' + corrupt + frame
        self.assertEqual(decoder.feed(data, 1), [(0x0A, struct.pack('<hhhB', 100, 0, 0, 120))])
        self.assertEqual(decoder.checksum_errors, 1)
        self.assertEqual(decoder.malformed_frames, 2)

    def test_payload_contains_header(self):
        payload = bytes.fromhex('aaff55fb000078')
        self.assertEqual(FrameDecoder().feed(report(0x0A, payload), 1), [(0x0A, payload)])

    def test_unknown_truncated_frame_expires(self):
        decoder = FrameDecoder(frame_timeout=0.1)
        valid = report(0x0A, bytes(7))
        self.assertEqual(decoder.feed(b'\xff\xfb\x30\x7f' + valid, 2), [])
        self.assertEqual(decoder.feed(b'', 2.2), [(0x0A, bytes(7))])
        self.assertEqual(decoder.malformed_frames, 1)

    def test_invalid_imu_payload_length_never_accepted(self):
        decoder = FrameDecoder()
        self.assertEqual(decoder.feed(report(0x0E, bytes(16)), 1), [])
        self.assertEqual(decoder.valid_frames, 0)


class TransportTests(unittest.TestCase):
    def make_transport(self, clock=None):
        fake = FakeSerial()
        transport = RosmasterTransport('/dev/fake', serial_factory=lambda **kw: (
            fake.kwargs.update(kw) or fake), sdk_class=vendor_sdk(), clock=clock)
        transport.start()
        self.addCleanup(transport.close)
        return transport, fake

    def ready(self, transport, fake):
        fake.input.put(report(0x0A, bytes(7)))
        until(lambda: transport.snapshot().motion_sequence == 1)

    def test_startup_only_zero_and_temporary_reporting_no_arm_or_flash(self):
        transport, fake = self.make_transport()
        self.assertEqual([frame[3] for frame in fake.writes], [0x12, 0x01])
        zero, reporting = fake.writes
        self.assertEqual(zero, bytes.fromhex('fffc0a12020000000000001e'))
        self.assertEqual(reporting, bytes.fromhex('fffc0501010007'))
        self.assertEqual(fake.kwargs['timeout'], 0.05)
        self.assertEqual(fake.kwargs['write_timeout'], 0.05)
        self.assertTrue(fake.kwargs['exclusive'])
        self.assertIsNone(transport.snapshot().motion_time)

    def test_real_sdk_velocity_frame(self):
        transport, fake = self.make_transport()
        self.ready(transport, fake)
        transport.send_velocity(0.2, -0.1, 0.5)
        until(lambda: len(fake.writes) == 3)
        packet = fake.writes[-1]
        self.assertEqual(packet[:5], bytes.fromhex('fffc0a1202'))
        self.assertEqual(struct.unpack('<hhh', packet[5:11]), (200, -100, 500))
        self.assertEqual(packet[-1], sum(packet[2:-1]) & 0xFF)

    def test_only_valid_received_frame_refreshes_motion(self):
        current_time = [10.0]
        transport, fake = self.make_transport(clock=lambda: current_time[0])
        speed = report(0x0A, struct.pack('<hhhB', 123, -234, 456, 125))
        fake.input.put(speed)
        until(lambda: transport.snapshot().motion_sequence == 1)
        snapshot = transport.snapshot()
        self.assertAlmostEqual(snapshot.vx, 0.123)
        self.assertAlmostEqual(snapshot.vy, -0.234)
        self.assertAlmostEqual(snapshot.wz, 0.456)
        self.assertEqual(snapshot.voltage, 12.5)
        self.assertEqual(snapshot.motion_time, 10.0)
        current_time[0] = 15.0
        fake.input.put(report(0x0C, bytes(6)))
        until(lambda: transport.snapshot().valid_frames == 2)
        self.assertEqual(transport.snapshot().motion_time, 10.0)
        self.assertEqual(snapshot.motion_sequence, 1)  # old snapshot stays immutable
        fake.input.put(speed[:-1] + bytes((speed[-1] ^ 1,)))
        until(lambda: transport.snapshot().checksum_errors == 1)
        self.assertEqual(transport.snapshot().motion_sequence, 1)

    def test_mpu_and_icm_units_follow_vendor(self):
        transport, fake = self.make_transport(clock=lambda: 20.0)
        fake.input.put(report(0x0B, struct.pack('<9h', 3755, -3755, 3755, 1672, 0, 16384, 10, 20, 30)))
        until(lambda: transport.snapshot().imu_sequence == 1)
        snapshot = transport.snapshot()
        self.assertAlmostEqual(snapshot.gyro[0], 1.0, places=3)
        self.assertAlmostEqual(snapshot.gyro[1], 1.0, places=3)
        self.assertAlmostEqual(snapshot.gyro[2], -1.0, places=3)
        self.assertAlmostEqual(snapshot.acceleration[0], 1.0, places=3)
        self.assertEqual(snapshot.magnetic, (10.0, 20.0, 30.0))
        fake.input.put(report(0x0E, struct.pack('<9h', 1000, -1000, 500, 0, 0, 9800, 100, 200, 300)))
        until(lambda: transport.snapshot().imu_sequence == 2)
        snapshot = transport.snapshot()
        self.assertEqual(snapshot.gyro, (1.0, -1.0, 0.5))
        self.assertEqual(snapshot.acceleration, (0.0, 0.0, 9.8))
        self.assertEqual(snapshot.imu_time, 20.0)
        self.assertIsNone(snapshot.motion_time)

    def test_mailbox_is_nonblocking_and_discards_queued_old_commands(self):
        transport, fake = self.make_transport()
        self.ready(transport, fake)
        fake.block_next = True
        transport.send_velocity(0.1, 0, 0)
        self.assertTrue(fake.entered_write.wait(0.3))
        before = time.monotonic()
        for value in (0.2, 0.3, 0.4, 0.5, 0.0):
            transport.send_velocity(value, 0, 0)
        self.assertLess(time.monotonic() - before, 0.03)
        fake.release_write.set()
        until(lambda: len(fake.writes) >= 4)
        self.assertEqual([struct.unpack('<h', packet[5:7])[0] for packet in fake.writes[2:]], [100, 0])

    def test_short_write_not_hidden_by_sdk_exception_handler(self):
        transport, fake = self.make_transport()
        fake.short_write = True
        transport.send_velocity(0.1, 0, 0)
        until(lambda: transport.error is not None)
        self.assertIn('short serial write', transport.error)
        with self.assertRaises(RuntimeError):
            transport.send_velocity(0.1, 0, 0)

    def test_reader_disconnect_latches_error(self):
        transport, fake = self.make_transport()
        fake.input.put(OSError('synthetic read disconnect'))
        until(lambda: transport.error is not None)
        self.assertIn('read disconnect', transport.error)
        self.assertEqual(transport.snapshot().motion_sequence, 0)

    def test_invalid_velocity_cannot_enter_mailbox(self):
        transport, fake = self.make_transport()
        for values in ((math_nan(), 0, 0), (0.71, 0, 0), (0, 0, 3.3)):
            with self.assertRaises(ValueError):
                transport.send_velocity(*values)
        self.assertEqual(len(fake.writes), 2)

    def test_shutdown_zero_and_no_live_workers(self):
        transport, fake = self.make_transport()
        transport.close()
        self.assertEqual(struct.unpack('<hhh', fake.writes[-1][5:11]), (0, 0, 0))
        self.assertTrue(fake.closed)
        self.assertFalse(transport._reader.is_alive())
        self.assertFalse(transport._writer.is_alive())
        self.assertIsNone(transport.error)
        transport.close()
        with self.assertRaises(RuntimeError):
            transport.send_velocity(0, 0, 0)


def math_nan():
    return float('nan')


if __name__ == '__main__':
    unittest.main()
