"""Hardware code path smoke: real SDK + pyserial + pseudo-terminal firmware."""

from pathlib import Path
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'x3plus_base'))
sys.path.insert(0, str(ROOT / 'vendor' / 'py_install'))
from x3plus_base.transport import RosmasterTransport
from serial_emulator import FirmwareEmulator


def wait_for(predicate, description, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('timeout: ' + description)


def main():
    results = {}
    firmware = FirmwareEmulator().start()
    transport = RosmasterTransport(firmware.port, command_timeout=0.15, feedback_timeout=0.3)
    try:
        transport.start()
        wait_for(lambda: transport.snapshot().motion_time is not None and
                 transport.snapshot().imu_time is not None, 'initial real serial feedback')
        assert [command[0] for command in firmware.commands[:2]] == [0x12, 0x01]
        assert firmware.motion_commands[0] == (0, 0, 0)
        assert firmware.commands[1][1] == bytes((1, 0))
        results['startup_zero_temporary_report_no_arm_no_flash'] = 'PASS'

        before = time.monotonic()
        transport.send_velocity(0.2, -0.1, 0.4)
        assert time.monotonic() - before < 0.02
        wait_for(lambda: (200, -100, 400) in firmware.motion_commands, 'actual SDK command bytes')
        wait_for(lambda: abs(transport.snapshot().vx - 0.2) < 1e-9 and
                 abs(transport.snapshot().gyro[2] - 0.4) < 1e-9, 'speed and ICM IMU decode')
        assert transport.snapshot().acceleration[2] == 9.807
        assert transport.snapshot().voltage == 12.4
        results['sdk_pyserial_pty_motion_and_imu_roundtrip'] = 'PASS'

        # No ROS timer or further enqueue exists: writer alone must stop it.
        wait_for(lambda: firmware.motion_commands[-1] == (0, 0, 0), 'independent writer deadman')
        assert transport.error is None
        results['sender_stall_watchdog_with_live_feedback'] = 'PASS'

        firmware.pause_reports(True)
        time.sleep(0.12)  # drain a read already in flight before pausing
        sample = transport.snapshot()
        old_time, old_sequence = sample.motion_time, sample.motion_sequence
        bad_count = sample.checksum_errors
        firmware.emit_motion(corrupt=True)
        firmware.send_raw(b'\xff\xfb\x00\x0a')
        firmware.emit_imu()
        wait_for(lambda: transport.snapshot().checksum_errors > bad_count, 'checksum rejection')
        wait_for(lambda: transport.snapshot().imu_sequence > sample.imu_sequence, 'fresh unrelated IMU')
        assert transport.snapshot().motion_time == old_time
        assert transport.snapshot().motion_sequence == old_sequence
        assert transport.snapshot().malformed_frames >= 1
        results['corruption_rejected_without_fake_motion_freshness'] = 'PASS'

        # A newly queued nonzero command cannot pass stale feedback gating.
        time.sleep(0.31)
        count = len(firmware.motion_commands)
        transport.send_velocity(0.1, 0, 0)
        wait_for(lambda: len(firmware.motion_commands) > count, 'stale-feedback zero write')
        assert firmware.motion_commands[-1] == (0, 0, 0)
        results['stale_feedback_gates_nonzero'] = 'PASS'

        transport.close()
        wait_for(lambda: firmware.motion_commands[-1] == (0, 0, 0), 'final shutdown zero')
        assert not transport._reader.is_alive() and not transport._writer.is_alive()
        assert firmware.error is None
        results['bounded_shutdown_zero_and_workers_stopped'] = 'PASS'
    finally:
        transport.close()
        firmware.close()

    # A separate connection is deliberately disconnected without transport.close.
    firmware = FirmwareEmulator().start()
    transport = RosmasterTransport(firmware.port)
    try:
        transport.start()
        wait_for(lambda: transport.snapshot().motion_time is not None, 'disconnect fixture feedback')
        firmware.close()
        wait_for(lambda: transport.error is not None, 'PTY disconnect surfaced')
        results['physical_serial_disconnect_error_latched'] = 'PASS'
    finally:
        transport.close()
        firmware.close()
    print(json.dumps({'fixture': 'real SDK 3.3.9 / pyserial / Linux PTY, no physical robot',
                      'results': results}, indent=2))


if __name__ == '__main__':
    main()
