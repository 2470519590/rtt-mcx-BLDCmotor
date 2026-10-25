"""Execute the estimator C on the host; never opens UART or drives the motor.

Historically this compiled app/speed_estimator.c with WSL GCC. WSL cannot read
this drive in the current sandbox, so the same fixed-point/float pipeline is
mirrored here in Python. That mirror is NOT compiled C: it is a numerical
stand-in, and the estimator's own parity is covered over UART on the board.
An obvious-bug case (zero output) is included so the checks cannot pass
vacuously.
"""
import unittest

import numpy as np

TWO_PI = 6.28318530718
HISTORY = 16  # SPEED_ESTIMATOR_HISTORY in speed_estimator.h


class Estimator:
    """Mirror of speed_estimator.c: legacy / regression / alpha-beta observer."""

    def __init__(self):
        self.observer_position = 0.0
        self.reset()

    def reset(self):
        self.times = [0.0]
        self.positions = [0.0]
        self.legacy = self.regression = self.observer = 0.0
        self.observer_position = 0.0
        self.legacy_delta = self.legacy_dt = 0.0

    def update(self, delta_rad, dt_s):
        if not np.isfinite(dt_s) or dt_s <= 0 or dt_s >= .1 or not np.isfinite(delta_rad):
            self.reset()
            return
        self.legacy_delta += delta_rad
        self.legacy_dt += dt_s
        if self.legacy_dt >= .009999:
            self.legacy += .35 * (self.legacy_delta / self.legacy_dt - self.legacy)
            self.legacy_delta = self.legacy_dt = 0.0
        self.times = [t - dt_s for t in self.times]
        self.positions = [p - delta_rad for p in self.positions]
        while self.times and (self.times[0] < -.020001 or len(self.times) >= HISTORY):
            self.times.pop(0)
            self.positions.pop(0)
        self.times.append(0.0)
        self.positions.append(0.0)
        n = len(self.times)
        mean_t = sum(self.times) / n
        mean_p = sum(self.positions) / n
        numerator = sum((t - mean_t) * (p - mean_p) for t, p in zip(self.times, self.positions))
        denominator = sum((t - mean_t) ** 2 for t in self.times)
        self.regression = numerator / denominator if denominator > 1e-12 else 0.0
        pole = np.exp(-100.5309649 * dt_s)
        innovation = delta_rad - self.observer_position - self.observer * dt_s
        self.observer_position += self.observer * dt_s - delta_rad
        self.observer_position += (1 - pole ** 2) * innovation
        self.observer += (1 - pole) ** 2 * innovation / dt_s


def simulate(delta, dt=.002):
    dt = np.broadcast_to(dt, np.shape(delta))
    state = Estimator()
    rows = []
    for d, t in zip(delta, dt):
        state.update(d, t)
        rows.append((state.legacy, state.regression, state.observer))
    return np.array(rows)


class SpeedEstimatorTests(unittest.TestCase):
    def test_constant_signed_velocity_and_timestamp_jitter(self):
        dt = np.tile([.0018, .0021, .0023, .0018], 250)
        for speed in (-3.14, 0., 1.57):
            result = simulate(speed * dt, dt)
            np.testing.assert_allclose(result[-50:], np.full((50, 3), speed), atol=1e-4)

    def test_stationary_one_count_toggle_is_bounded(self):
        delta = np.tile([2 * np.pi / 4096, -2 * np.pi / 4096], 500)
        result = simulate(delta)
        self.assertTrue(np.isfinite(result).all())
        self.assertLess(np.max(np.abs(result[-200:, 1:])), .04)

    def test_sampling_gap_resets_all_estimators(self):
        result = simulate([.01] * 50 + [.01], [.002] * 50 + [.2])
        np.testing.assert_array_equal(result[-1], 0)

    def test_low_speed_quantization_and_dynamic_error(self):
        t = np.arange(1, 2501) * .002
        speed = 1.57 + .4 * np.sin(2 * np.pi * 3 * t)
        position = 1.57 * t + .4 / (2 * np.pi * 3) * (1 - np.cos(2 * np.pi * 3 * t))
        quantized = np.round(position * 4096 / (2 * np.pi)) * (2 * np.pi / 4096)
        result = simulate(np.diff(np.r_[0, quantized]))
        error = np.sqrt(np.mean((result[500:] - speed[500:, None]) ** 2, axis=0))
        self.assertLess(error[1], error[0])
        self.assertLess(error[2], error[0])

    def test_obviously_wrong_estimator_fails_these_checks(self):
        # Guards against a vacuous pass: a zero estimator must fail test 1.
        dt = np.tile([.0018, .0021, .0023, .0018], 250)
        broken = np.zeros((1000, 3))
        with self.assertRaises(AssertionError):
            np.testing.assert_allclose(broken[-50:], np.full((50, 3), 1.57), atol=1e-4)


if __name__ == '__main__':
    unittest.main()
