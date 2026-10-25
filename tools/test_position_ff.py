import unittest
from compare_position_ff import summarize, comparison_configs, gain_configs, step_magnitudes


class PositionMetricsTests(unittest.TestCase):
    def test_step_magnitudes(self):
        self.assertEqual(step_magnitudes([1, 10, 20, 30]), [1, 10, 20, 30])

    def test_invalid_step_magnitudes(self):
        for values in ([], [0], [-10], [31], [float('nan')], [float('inf')], [10, 10]):
            with self.assertRaises(ValueError):
                step_magnitudes(values)

    def test_gain_comparison_keeps_each_mode_and_gain(self):
        self.assertEqual(gain_configs({'baseline':0.,'ff100':1.},[8,12]),{
            'pos08000_baseline':(0.,8),'pos08000_ff100':(1.,8),
            'pos12000_baseline':(0.,12),'pos12000_ff100':(1.,12)})

    def test_invalid_or_duplicate_gains(self):
        for values in ([],[0],[float('nan')],[21],[8,8.0001]):
            with self.assertRaises(ValueError):
                gain_configs({'baseline':0},values)

    def test_mixed_strengths_keep_baseline_and_distinct_labels(self):
        self.assertEqual(comparison_configs([.5,1]),
                         {'baseline':0.,'ff0500':.5,'ff1000':1})

    def test_invalid_or_duplicate_strengths(self):
        for values in ([],[0],[float('nan')],[1.1],[.5,.5001]):
            with self.assertRaises(ValueError):
                comparison_configs(values)

    def test_exact_tracking(self):
        rows = [dict(tick_ms=i*10, position_mrad=175, clipped=0) for i in range(200)]
        result = summarize(rows,175)
        self.assertEqual(result['rmse_deg'],0)
        self.assertEqual(result['settled_0p5deg_s'],0)
        self.assertEqual(result['overshoot_deg'],0)
        self.assertEqual(result['tail_std_deg'],0)

    def test_bidirectional_overshoot(self):
        for sign in (1,-1):
            rows=[dict(tick_ms=i*10,position_mrad=sign*p,clipped=0)
                  for i,p in enumerate([0,100,200,175])]
            result=summarize(rows,sign*175)
            self.assertAlmostEqual(result['overshoot_deg'],25*180/(1000*3.141592653589793))

    def test_never_settles(self):
        rows = [dict(tick_ms=i*10, position_mrad=0, clipped=0) for i in range(200)]
        result = summarize(rows,175)
        self.assertIsNone(result['settled_0p5deg_s'])
        self.assertGreater(result['tail_mae_deg'],10)


if __name__=='__main__':
    unittest.main()
