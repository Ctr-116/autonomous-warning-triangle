"""Explicit ideal arm/accessory firmware fixture on real Linux PTYs.

Reported pulses are fixture responses to queries, never proof of real motion.
Unknown commands, torque enables and flash writes remain rejected.
"""
import struct
from serial_emulator import FirmwareEmulator, report


class AccessoryFirmware(FirmwareEmulator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.arm_raw = [2000, 2000, 2000, 2000, 1486, 2000]
        self.version_bytes = bytes((3, 3))
        self.lamps = None
        self.effect = None
        self.buzzer = None
        self.answer_arm_queries = True

    def _handle_command(self, function, payload):
        if function == 0x05 and len(payload) == 4:
            self.lamps = tuple(payload)
        elif function == 0x06 and len(payload) == 3:
            self.effect = tuple(payload)
        elif function == 0x02 and len(payload) == 2:
            self.buzzer = struct.unpack('<h', payload)[0]
        elif function == 0x23 and len(payload) == 14:
            with self._lock:
                self.arm_raw = list(struct.unpack('<6H', payload[:12]))
        elif function == 0x20 and len(payload) == 5:
            servo_id, pulse, duration = struct.unpack('<BHH', payload)
            if not 1 <= servo_id <= 6 or not 0 <= duration <= 2000:
                raise ValueError('invalid single-servo fixture command')
            with self._lock:
                self.arm_raw[servo_id - 1] = pulse
        elif function == 0x50 and len(payload) == 2:
            target, param = payload
            with self._lock:
                pulses = tuple(self.arm_raw)
            if target == 0x23 and param == 1:
                if self.answer_arm_queries:
                    self.send_raw(report(0x23, struct.pack('<6H', *pulses)))
            elif target == 0x20 and 1 <= param <= 6:
                self.send_raw(report(0x20, struct.pack('<BH', param, pulses[param - 1])))
            elif target == 0x51 and param == 0:
                self.send_raw(report(0x51, self.version_bytes))
            else:
                raise ValueError('unknown accessory fixture query')
        else:
            super()._handle_command(function, payload)
