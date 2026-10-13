import time


class PID:
    """位置式PID，带积分限幅、输出限幅、微分低通滤波"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0,
                 output_limit=1.0, integral_limit=100.0,
                 derivative_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.derivative_filter = derivative_filter

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.last_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.last_time = time.time()

    def update(self, error):
        now = time.time()
        dt = now - self.last_time
        if dt < 0.001:
            dt = 0.001
        self.last_time = now

        p = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i = self.ki * self.integral

        raw_derivative = (error - self.prev_error) / dt
        derivative = self.prev_derivative + \
                     (dt / (self.derivative_filter + dt)) * (raw_derivative - self.prev_derivative)
        self.prev_derivative = derivative
        d = self.kd * derivative

        self.prev_error = error

        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))