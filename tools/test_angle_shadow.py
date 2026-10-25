import unittest
import tempfile
from pathlib import Path

import numpy as np

from angle_shadow_model import load_angle_model, predict_float32, render_header, HARMONICS
from verify_angle_shadow import verify
from register_angle_phase import predict


class AngleShadowTests(unittest.TestCase):
    def test_phase_registration_preserves_periodic_features(self):
        model = self.model()
        model['hidden_weights'][0, 2] = 1
        model['hidden_bias'][0] = 2
        model['output_weights'][0] = .1
        position = np.array([0., .5, 4.])
        speed = np.array([15., -15., 30.])
        np.testing.assert_allclose(predict(position, speed, model, .7),
                                   predict(position+.7, speed, model, 0), atol=1e-12)
        np.testing.assert_allclose(predict(position, speed, model, .7+2*np.pi),
                                   predict(position, speed, model, .7), atol=1e-12)

    def model(self):
        return dict(input_mean=np.zeros(12), input_std=np.ones(12),
                    hidden_weights=np.zeros((8, 12)), hidden_bias=np.zeros(8),
                    output_weights=np.zeros(8), output_bias=np.array(.02))

    def row(self):
        return dict(nn_mode=5, fault=0, sensor_ok=1, calibrated=1,
                    position_mrad=1234, target_speed_mrad_s=1570,
                    nn_delta_uq_mv=20, uq_mv=200, base_uq_mv=200, clipped=0)

    def test_prediction_and_non_actuation_are_both_required(self):
        row = self.row()
        self.assertTrue(verify([row], self.model())['passed'])
        row['uq_mv'] = 220
        self.assertFalse(verify([row], self.model())['passed'])
        row['uq_mv'] = 200
        row['nn_delta_uq_mv'] = 90
        self.assertFalse(verify([row], self.model())['passed'])

    def test_wrong_mode_missing_health_or_all_clipped_rejected(self):
        for change in ({'nn_mode': 4}, {'fault': 1}, {'calibrated': 0}, {'clipped': 1}):
            with self.assertRaises(ValueError):
                verify([dict(self.row(), **change)], self.model())

    def test_blended_angle_application_is_checked(self):
        row = dict(self.row(), nn_mode=6, nn_alpha_milli=100, uq_mv=202)
        self.assertTrue(verify([row], self.model())['passed'])
        row['uq_mv'] = 250
        self.assertFalse(verify([row], self.model())['passed'])

    def test_truncated_input_does_not_cause_false_parity_failure(self):
        model = self.model()
        model['hidden_weights'][0, 2] = 1
        model['hidden_bias'][0] = 2
        model['output_weights'][0] = .1
        actual = float(predict_float32([1.2349], [1.5709], model)[0])
        row = dict(self.row(), nn_delta_uq_mv=int(actual*1000))
        self.assertTrue(verify([row], model)['passed'])

    def test_export_rejects_invalid_archive_without_output_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'model.npz'
            model = self.model()
            model['input_std'][0] = 0
            np.savez(source, schema='angle_voltage_candidate_v1', harmonics=HARMONICS, **model)
            with self.assertRaises(ValueError):
                render_header(source)

    def test_export_and_float32_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'model.npz'
            np.savez(source, schema='angle_voltage_candidate_v1', harmonics=HARMONICS, **self.model())
            model = load_angle_model(source)
            self.assertIn('ANGLE_MODEL_INPUTS 12U', render_header(source))
            np.testing.assert_allclose(predict_float32([0, 5], [1, -1], model), [.02, .02])


if __name__ == '__main__':
    unittest.main()
