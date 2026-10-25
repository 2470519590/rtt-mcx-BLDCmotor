import math
import unittest
import numpy as np

from optimize_motor import score, parse_timing, three_arm_setup
from train_motor_feedforward import basis
from summarize_estimator_study import common_window_rmse


class MotorOptimizationTests(unittest.TestCase):
    def test_three_arm_uses_zero_output_baseline_and_distinct_models(self):
        self.assertEqual(three_arm_setup('pi_kp150_ki100', [1]*9, [2]*9),
                         ('baseline', [0]*9))
        self.assertEqual(three_arm_setup('pi_kp150_ki100_ff050', [1]*9, [2]*9),
                         ('friction', [1]*9))
        self.assertEqual(three_arm_setup('pi_kp150_ki100_ff100', [1]*9, [2]*9),
                         ('friction_nn', [2]*9))
        with self.assertRaises(ValueError):
            three_arm_setup('unrecognized', [], [])
    def test_common_window_uses_integer_elapsed_time_not_float_epoch(self):
        samples = [dict(tick_ms=i*10, position_mrad=round(i*.01*2*math.pi*1000))
                   for i in range(301)]
        shifted = [dict(row, tick_ms=row['tick_ms']+100000007) for row in samples]
        for window in (20, 40, 100):
            self.assertEqual(common_window_rmse(samples, 60, window),
                             common_window_rmse(shifted, 60, window))
            self.assertLess(common_window_rmse(samples, 60, window), .5)
    def test_cycle_timing_units_and_counter_validation(self):
        row = parse_timing('E,1,180000000,100,360000,361000,25000,26000,40000,41000,0,0,0,0,0')
        self.assertAlmostEqual(row['period_us'], 2000)
        self.assertAlmostEqual(row['max_loop_us'], 41000/180)
        with self.assertRaises(ValueError):
            parse_timing('E,1,0,100,360000,361000,25000,26000,40000,41000,0,0,0,0,0')
        with self.assertRaises(ValueError):
            parse_timing('E,1,2')
    def test_position_derived_speed_is_independent_of_reported_speed(self):
        samples = [dict(tick_ms=100000+i*10, speed_mrad_s=0,
                        position_mrad=round(i*.01*2*math.pi*1000), clipped=0)
                   for i in range(301)]
        result = score(samples, 60)
        self.assertEqual(result['rmse_rpm'], 60)
        self.assertLess(result['raw_tail_rmse_rpm'], .2)
        self.assertAlmostEqual(result['iae_rpm_s'], 180)

    def test_network_basis_limits_extrapolation(self):
        x = basis([-1000, -180, 180, 1000])
        np.testing.assert_array_equal(x[0], x[1])
        np.testing.assert_array_equal(x[2], x[3])
        self.assertEqual(x.shape, (4, 9))


if __name__ == '__main__':
    unittest.main()
