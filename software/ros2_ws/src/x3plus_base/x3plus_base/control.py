"""ROS-independent command supervision and planar odometry."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    x: float = 0.2
    y: float = 0.2
    yaw: float = 0.6
    command_timeout: float = 0.3
    feedback_timeout: float = 0.3

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in vars(self).values()):
            raise ValueError('Limits and timeouts must be finite and positive')


class CommandGuard:
    def __init__(self, limits):
        self.limits = limits
        self.command = (0.0, 0.0, 0.0)
        self.command_time = None
        self.fault = ''
        self.ever_ready = False

    def stop(self, reason):
        self.fault = str(reason)
        self.command = (0.0, 0.0, 0.0)
        self.command_time = None

    def receive(self, vx, vy, wz, now):
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self.stop('INVALID_COMMAND')
            return False
        if self.fault:
            return False
        speed = math.hypot(vx / self.limits.x, vy / self.limits.y)
        scale = max(1.0, speed)
        self.command = (vx / scale, vy / scale,
                        max(-self.limits.yaw, min(self.limits.yaw, wz)))
        self.command_time = now
        return True

    def reset(self, now, feedback_time):
        if feedback_time is None or not 0 <= now - feedback_time <= self.limits.feedback_timeout:
            return False
        self.fault = ''
        self.command = (0.0, 0.0, 0.0)
        self.command_time = None
        return True

    def output(self, now, feedback_time, transport_error=None):
        if transport_error:
            self.stop('TRANSPORT_ERROR: ' + str(transport_error))
        fresh = feedback_time is not None and 0 <= now - feedback_time <= self.limits.feedback_timeout
        if not fresh:
            if self.ever_ready and not self.fault:
                self.stop('FEEDBACK_TIMEOUT')
            # No queued nonzero command may survive unavailable feedback.
            self.command_time = None
            return (0.0, 0.0, 0.0), self.fault or 'WAITING_FEEDBACK'
        self.ever_ready = True
        if self.fault:
            return (0.0, 0.0, 0.0), self.fault
        if self.command_time is None or now - self.command_time > self.limits.command_timeout:
            return (0.0, 0.0, 0.0), 'IDLE'
        return self.command, 'ACTIVE'


class PlanarOdometry:
    """Integrate only newly received velocity samples; never bridge a stale gap."""
    def __init__(self, max_dt=0.3, scale_x=1.0, scale_y=1.0):
        if not all(math.isfinite(v) and v > 0 for v in (max_dt, scale_x, scale_y)):
            raise ValueError('Odometry parameters must be positive')
        self.max_dt, self.scale_x, self.scale_y = max_dt, scale_x, scale_y
        self.x = self.y = self.yaw = 0.0
        self.last_time = None

    def update(self, stamp, vx, vy, wz):
        if stamp is None or not all(math.isfinite(v) for v in (stamp, vx, vy, wz)):
            return False
        if self.last_time is not None and stamp <= self.last_time:
            return False
        dt = 0.0 if self.last_time is None else stamp - self.last_time
        self.last_time = stamp
        if dt > self.max_dt:
            dt = 0.0
        vx, vy = vx * self.scale_x, vy * self.scale_y
        angle = wz * dt
        if abs(wz) > 1e-9:
            local_x = (vx * math.sin(angle) + vy * (math.cos(angle) - 1)) / wz
            local_y = (vx * (1 - math.cos(angle)) + vy * math.sin(angle)) / wz
        else:
            local_x, local_y = vx * dt, vy * dt
        self.x += math.cos(self.yaw) * local_x - math.sin(self.yaw) * local_y
        self.y += math.sin(self.yaw) * local_x + math.cos(self.yaw) * local_y
        self.yaw = math.atan2(math.sin(self.yaw + angle), math.cos(self.yaw + angle))
        return True
