"""Real SDK encoders + pyserial/PTY accessory requests and measured replies."""
from pathlib import Path
import struct
import sys
import time
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'x3plus_base'))
sys.path.insert(0, str(ROOT / 'vendor' / 'py_install'))
from x3plus_base.transport import RosmasterTransport
from serial_emulator import report
from serial_accessory_emulator import AccessoryFirmware
from hardware_transport_smoke import wait_for


def main():
    firmware = AccessoryFirmware().start()
    transport = RosmasterTransport(firmware.port, command_timeout=0.2, feedback_timeout=0.3)
    heartbeat_stop = threading.Event()
    heartbeat = None
    try:
        transport.start()
        wait_for(lambda: transport.snapshot().motion_time is not None, 'fixture motion feedback')
        assert [item[0] for item in firmware.commands[:2]] == [0x12, 0x01]
        assert transport.snapshot().arm_angles is None and transport.snapshot().version is None
        assert not any(item[0] in (0x22, 0xA0) for item in firmware.commands)
        print('PASS accessory path startup has zero motion, no torque/flash and no invented arm/version feedback', flush=True)

        # The real BaseNode sends a live zero command while idle. Accessories
        # deliberately require that drive heartbeat, even when not moving.
        def keep_idle_alive():
            while not heartbeat_stop.is_set():
                transport.send_velocity(0.0, 0.0, 0.0)
                heartbeat_stop.wait(0.05)
        heartbeat = threading.Thread(target=keep_idle_alive, daemon=True)
        heartbeat.start()

        before = time.monotonic()
        transport.set_lamps(255, 18, 52, 86)
        assert time.monotonic() - before < 0.02
        wait_for(lambda: firmware.lamps == (255, 18, 52, 86), 'static lamp frame')
        packet = next(packet for function, payload, packet in firmware.commands if function == 0x05)
        assert packet == b'\xff\xfc\x07\x05\xff\x12\x34\x56\xa7', packet.hex()
        transport.set_rgb(1)
        wait_for(lambda: firmware.effect == (1, 6, 1), 'effect mapping')
        transport.set_buzzer(True)
        wait_for(lambda: firmware.buzzer == 1, 'buzzer on')
        transport.set_buzzer(False)
        wait_for(lambda: firmware.buzzer == 0, 'buzzer off')
        print('PASS actual SDK static RGB bytes/checksum, raw effect and buzzer on/off through single writer', flush=True)

        transport.send_arm(joints=[90.0] * 6, run_time=500)
        wait_for(lambda: any(f == 0x23 for f, _, _ in firmware.commands), 'full arm SDK frame')
        payload = next(p for f, p, _ in firmware.commands if f == 0x23)
        assert struct.unpack('<6HH', payload) == (2000, 2000, 2000, 2000, 1486, 2000, 500)
        assert transport.snapshot().arm_angles is None, 'command target advertised as measurement'
        transport.request_arm()
        wait_for(lambda: transport.snapshot().arm_angles == (90,) * 6, 'actual arm query reply decode')
        assert any(f == 0x50 and p == b'\x23\x01' for f, p, _ in firmware.commands)
        print('PASS exact six-servo SDK pulse frame; target does not become measurement before a real reply', flush=True)

        transport.send_arm(servo_id=2, angle=180.0, run_time=250)
        wait_for(lambda: any(f == 0x20 for f, _, _ in firmware.commands), 'single-servo frame')
        payload = next(p for f, p, _ in firmware.commands if f == 0x20)
        assert struct.unpack('<BHH', payload) == (2, 900, 250)
        transport.request_servo_angle(2)
        wait_for(lambda: transport.snapshot().servo_angle == 180, 'single-servo query reply')
        assert transport.snapshot().servo_id == 2
        print('PASS exact single-servo pulse frame and independent angle-query decoding', flush=True)

        transport.request_version()
        wait_for(lambda: transport.snapshot().version == 3.3, 'version query response')
        assert any(f == 0x50 and p == b'\x51\x00' for f, p, _ in firmware.commands)
        old = transport.snapshot()
        firmware.send_raw(report(0x23, struct.pack('<6H', 0, 0, 0, 0, 0, 0)))
        bad_version = report(0x51, bytes((3, 99)))
        firmware.send_raw(bad_version[:-1] + bytes((bad_version[-1] ^ 1,)))
        time.sleep(0.1)
        new = transport.snapshot()
        assert new.arm_time == old.arm_time and new.arm_sequence == old.arm_sequence
        assert new.version_time == old.version_time and new.version_sequence == old.version_sequence
        print('PASS actual version response, malformed arm/version cannot manufacture fresh feedback', flush=True)

        count = len([f for f, _, _ in firmware.commands if f != 0x12])
        for args in ((14, 1, 2, 3), (True, 1, 2, 3), (255, -1, 0, 0), (255, 0, 0, 256)):
            try:
                transport.set_lamps(*args)
            except ValueError:
                pass
            else:
                raise AssertionError('invalid RGB accepted')
        try:
            transport.send_arm(servo_id=6, angle=0.0, run_time=500)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid gripper angle accepted')
        time.sleep(0.05)
        assert len([f for f, _, _ in firmware.commands if f != 0x12]) == count, 'invalid accessory queued serial bytes'
        assert transport.error is None and firmware.error is None
        print('PASS invalid commands rejected before serial, all accessory I/O uses the one transport owner', flush=True)
    finally:
        heartbeat_stop.set()
        if heartbeat is not None:
            heartbeat.join(timeout=1.0)
        transport.close()
        firmware.close()


if __name__ == '__main__':
    main()
