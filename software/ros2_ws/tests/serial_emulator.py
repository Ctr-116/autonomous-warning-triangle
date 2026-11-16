"""Linux PTY fake firmware for the actual Rosmaster binary serial protocol.

This is an ideal velocity echo fixture, not evidence of real motor dynamics or
firmware compatibility. It intentionally accepts only the phase-1 command set.
Run standalone to obtain a /dev/pts/N path usable by the hardware ROS driver.
"""

import errno
import os
import pty
import select
import struct
import threading
import time
import tty


def report(function, payload):
    body = bytes((len(payload) + 3, function)) + payload
    return b'\xff\xfb' + body + bytes((sum(body) & 0xFF,))


class FirmwareEmulator:
    def __init__(self, report_period=0.04):
        self.master_fd, self.slave_fd = pty.openpty()
        tty.setraw(self.slave_fd)
        self.port = os.ttyname(self.slave_fd)
        self.report_period = report_period
        self._stop = threading.Event()
        self._pause = threading.Event()
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._commands = []
        self.velocity = (0, 0, 0)
        self.reporting = False
        self.error = None
        self._thread = None
        self.closed = False

    @property
    def commands(self):
        with self._lock:
            return tuple(self._commands)

    @property
    def motion_commands(self):
        return tuple(struct.unpack('<hhh', payload[1:])
                     for function, payload, _ in self.commands
                     if function == 0x12 and len(payload) == 7)

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name='fake_stm32', daemon=True)
            self._thread.start()
        return self

    def pause_reports(self, paused=True):
        if paused:
            self._pause.set()
        else:
            self._pause.clear()

    def send_raw(self, data):
        with self._write_lock:
            view = memoryview(data)
            while view and not self.closed:
                count = os.write(self.master_fd, view)
                view = view[count:]

    def emit_motion(self, corrupt=False):
        with self._lock:
            values = self.velocity
        frame = report(0x0A, struct.pack('<hhhB', *values, 124))
        if corrupt:
            frame = frame[:-1] + bytes((frame[-1] ^ 1,))
        self.send_raw(frame)

    def emit_imu(self):
        with self._lock:
            wz = self.velocity[2]
        self.send_raw(report(0x0E, struct.pack('<9h', 0, 0, wz, 0, 0, 9807, 100, 200, 300)))

    def _handle_command(self, function, payload):
        """Validate the baseline command set; subclasses may add explicit fixtures."""
        with self._lock:
            if function == 0x12 and len(payload) == 7:
                if payload[0] != 2:
                    raise ValueError('driver did not select X3plus in motion packet')
                self.velocity = struct.unpack('<hhh', payload[1:])
            elif function == 0x01 and len(payload) == 2:
                if payload[1] != 0:
                    raise ValueError('driver requested persistent flash update')
                self.reporting = bool(payload[0])
            else:
                raise ValueError('unexpected baseline SDK command 0x%02x' % function)

    def _run(self):
        data = bytearray()
        next_report = time.monotonic()
        try:
            while not self._stop.is_set():
                readable, _, _ = select.select([self.master_fd], [], [], 0.01)
                if readable:
                    try:
                        chunk = os.read(self.master_fd, 4096)
                    except OSError as exc:
                        if exc.errno == errno.EIO:
                            chunk = b''
                        else:
                            raise
                    data.extend(chunk)
                    while data:
                        offset = data.find(b'\xff\xfc')
                        if offset < 0:
                            data[:] = b'\xff' if data[-1] == 0xFF else b''
                            break
                        if offset:
                            del data[:offset]
                        if len(data) < 4:
                            break
                        length = data[2]
                        if length < 3 or length > 64:
                            del data[0]
                            continue
                        total = length + 2
                        if len(data) < total:
                            break
                        packet = bytes(data[:total])
                        if sum(packet[2:-1]) & 0xFF != packet[-1]:
                            del data[0]
                            continue
                        del data[:total]
                        function, payload = packet[3], packet[4:-1]
                        with self._lock:
                            self._commands.append((function, payload, packet))
                        self._handle_command(function, payload)
                now = time.monotonic()
                if now >= next_report:
                    next_report = now + self.report_period
                    if self.reporting and not self._pause.is_set():
                        self.emit_motion()
                        self.emit_imu()
        except Exception as exc:
            if not self._stop.is_set():
                self.error = '%s: %s' % (type(exc).__name__, exc)

    def close(self):
        if self.closed:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.3)
        self.closed = True
        os.close(self.master_fd)
        os.close(self.slave_fd)

    def __enter__(self):
        return self.start()

    def __exit__(self, *args):
        self.close()


def main():
    with FirmwareEmulator() as firmware:
        print(firmware.port, flush=True)
        try:
            while firmware.error is None:
                time.sleep(0.2)
        except KeyboardInterrupt:
            return
        raise RuntimeError(firmware.error)


if __name__ == '__main__':
    main()
