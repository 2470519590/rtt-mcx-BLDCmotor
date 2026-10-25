import unittest
import numpy as np
from fit_friction_comparison import friction_weights, fit_models
from motor_feedforward import predict_float32


class FrictionTests(unittest.TestCase):
    def test_classical_encoding_and_zero(self):
        speeds = np.array([-30, -15, -5, 5, 15, 30])
        voltage = .1*np.clip(speeds/9, -1, 1) - .03*speeds/180
        points = [dict(rpm=float(s), residual_v=float(v)) for s, v in zip(speeds, voltage)]
        weights, coefficients = friction_weights(points)
        np.testing.assert_allclose(coefficients, [.1, -.03], atol=1e-10)
        np.testing.assert_allclose(predict_float32(speeds, weights), voltage, atol=1e-6)
        self.assertAlmostEqual(float(predict_float32([0], weights)[0]), 0, places=6)

    def test_holdout_does_not_select_or_fit_weights(self):
        points = [dict(repeat=r, rpm=s,
                       residual_v=.1*np.clip(s/9, -1, 1)+.003*(s/30)**2)
                  for r in range(3) for s in (-30, -15, -5, 5, 15, 30)]
        original = fit_models(points)
        changed = fit_models([dict(p, residual_v=p['residual_v']+.2)
                              if p['repeat'] == 2 else p for p in points])
        for name in original:
            self.assertEqual(original[name]['upload_weights'], changed[name]['upload_weights'])
            self.assertNotEqual(original[name]['heldout_voltage_rmse_v'],
                                changed[name]['heldout_voltage_rmse_v'])


if __name__ == '__main__':
    unittest.main()
