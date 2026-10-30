"""Wheel slip estimation and EKF covariance inflation for snow.

slip ratio s = 1 - v_ground / v_wheel   (0 = perfect grip, 1 = spinning in place)
v_ground comes from LiDAR-inertial odometry or radar ego-velocity, whichever is healthy.
"""
from dataclasses import dataclass


@dataclass
class SlipEstimator:
    alpha: float = 0.2          # EMA smoothing
    slip: float = 0.0
    spin_threshold: float = 0.35

    def update(self, v_wheel: float, v_ground: float) -> float:
        if abs(v_wheel) < 0.05:
            return self.slip
        s = max(0.0, min(1.0, 1.0 - v_ground / v_wheel))
        self.slip = (1 - self.alpha) * self.slip + self.alpha * s
        return self.slip

    def wheel_variance(self, v_wheel: float, base_sigma=0.03) -> float:
        """Variance to report for wheel odometry: grows quadratically with slip."""
        sigma = base_sigma + abs(v_wheel) * (0.05 + 2.0 * self.slip)
        return sigma * sigma

    @property
    def spinning(self) -> bool:
        return self.slip > self.spin_threshold
