"""Reusable PID helpers for drivetrain and heading control."""
from math import pi
from typing import Optional

from constants import MAX_INTEGRAL


def wrap_angle_radians(angle: float) -> float:
    """Wrap an angle into the [-pi, pi) range."""
    while angle >= pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


class PIDConfig:
    def __init__(
        self,
        kp,
        ki=0.0,
        kd=0.0,
        output_min=None,
        output_max=None,
        integral_limit=MAX_INTEGRAL,
        continuous=False,
        derivative_on_measurement=True,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.continuous = continuous
        self.derivative_on_measurement = derivative_on_measurement


class PIDController:
    """Small PID controller with optional wraparound error support."""

    def __init__(self, config: PIDConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None  # type: Optional[float]
        self.previous_measurement = None  # type: Optional[float]

    def _normalize_error(self, error: float) -> float:
        if self.config.continuous:
            return wrap_angle_radians(error)
        return error

    def update(self, target: float, measurement: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")

        error = self._normalize_error(target - measurement)
        candidate_integral = self.integral + error * dt

        if self.config.integral_limit is not None:
            limit = abs(self.config.integral_limit)
            candidate_integral = max(-limit, min(limit, candidate_integral))

        derivative = 0.0
        if self.config.derivative_on_measurement:
            if self.previous_measurement is not None:
                derivative = -(measurement - self.previous_measurement) / dt
        elif self.previous_error is not None:
            derivative = (error - self.previous_error) / dt

        unclamped_output = (
            self.config.kp * error
            + self.config.ki * candidate_integral
            + self.config.kd * derivative
        )
        output = unclamped_output

        saturated_low = self.config.output_min is not None and output < self.config.output_min
        saturated_high = self.config.output_max is not None and output > self.config.output_max
        should_integrate = (
            not saturated_low and not saturated_high
        ) or (
            saturated_low and error > 0.0
        ) or (
            saturated_high and error < 0.0
        )

        if should_integrate:
            self.integral = candidate_integral

        self.previous_error = error
        self.previous_measurement = measurement

        if self.config.output_min is not None:
            output = max(self.config.output_min, output)
        if self.config.output_max is not None:
            output = min(self.config.output_max, output)
        return output
