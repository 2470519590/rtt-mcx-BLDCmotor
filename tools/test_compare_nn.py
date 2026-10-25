"""Offline checks for control comparison metrics."""
import unittest

from compare_nn import summarize


class ComparisonTests(unittest.TestCase):
    def test_constant_tracking_and_tick_gap(self):
        rows = [dict(trial='speed_baseline', command=60, measured=58,
                     unit='RPM', nn_delta_uq_mv=-10, clipped=0,
                     tick_ms=i*10) for i in range(100)]
        result = summarize(rows)[0]
        self.assertEqual(result['rmse'], 2)
        self.assertEqual(result['tail_mae'], 2)
        self.assertEqual(result['tail_mean'], 58)
        self.assertEqual(result['max_sample_gap_ms'], 10)


if __name__ == '__main__':
    unittest.main()
