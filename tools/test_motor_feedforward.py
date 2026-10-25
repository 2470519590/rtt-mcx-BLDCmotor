"""Offline contract tests; no serial connection or physical motor movement."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

from control_cli import MotorSerial
from motor_feedforward import KNOTS, basis, load_weights, predict_float32, model_identity
from train_motor_feedforward import fit_weights
from verify_nn_telemetry import verify


class FeedforwardTests(unittest.TestCase):
    def test_model_identity_changes_when_weights_change(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)/'model.json'
            source.write_text(json.dumps(self.model()), encoding='utf-8')
            first = model_identity(source)
            self.assertEqual(first['ff_model'], str(source.resolve()))
            self.assertEqual(len(first['ff_model_sha256']), 64)
            source.write_text(json.dumps(dict(self.model(), upload_weights=[1]*9)), encoding='utf-8')
            self.assertNotEqual(first['ff_model_sha256'], model_identity(source)['ff_model_sha256'])

    def model(self):
        return dict(schema='relu_speed_ff1', weight_units_v=.0001,
                    knots=KNOTS.tolist(), upload_weights=[0]*9)

    def test_invalid_metadata_is_rejected_before_serial_write(self):
        mutations = [dict(knots=[0]*8), dict(weight_units_v=.001),
                     dict(upload_weights=[False]*9), dict(upload_weights=[100001]*9),
                     dict(knots=[float('nan')]*8), dict(schema='residual_v2')]
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'model.json'
            for change in mutations:
                path.write_text(json.dumps(dict(self.model(), **change)), encoding='utf-8')
                motor = MotorSerial.__new__(MotorSerial)
                motor.send_checked = Mock()
                with self.subTest(change=change), self.assertRaises(ValueError):
                    motor.upload_feedforward(path)
                motor.send_checked.assert_not_called()

    def test_firmware_rejection_aborts_weight_upload(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'model.json'
            path.write_text(json.dumps(self.model()), encoding='utf-8')
            motor = MotorSerial.__new__(MotorSerial)
            motor.send_checked = Mock(side_effect=[None, None, ValueError('old firmware')])
            with self.assertRaisesRegex(ValueError, 'old firmware'):
                motor.upload_feedforward(path)
            self.assertEqual([c.args for c in motor.send_checked.call_args_list],
                             [('S', 0), ('G', 0), ('f', 0)])

    def test_upload_never_automatically_enables_control(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'model.json'
            path.write_text(json.dumps(self.model()), encoding='utf-8')
            motor = MotorSerial.__new__(MotorSerial)
            motor.send_checked = Mock()
            motor.upload_feedforward(path)
            calls = [c.args for c in motor.send_checked.call_args_list]
            self.assertEqual(len(calls), 20)
            self.assertEqual([c for c in calls if c[0] == 'G'], [('G', 0)])

    def test_float32_reference_matches_quantized_basis(self):
        weights = [152, 152, 1733, 12652, -12770, -1852, 0, 0, -1464]
        rpm = np.linspace(-1000, 1000, 2001)
        reference = np.clip(basis(rpm)@(np.array(weights)/10000), -.3, .3)
        np.testing.assert_allclose(predict_float32(rpm, weights), reference, atol=1e-6)

    def test_fit_is_finite_zero_centered(self):
        points = [dict(rpm=r, residual_v=.12*np.sign(r)) for r in [-120,-60,-30,30,60,120]]
        weights = fit_weights(points, 1e-6)
        self.assertLess(abs(float(predict_float32([0], weights)[0])), .00006)
        pred = predict_float32([p['rpm'] for p in points], weights)
        self.assertLess(float(np.sqrt(np.mean((pred-[p['residual_v'] for p in points])**2))), .002)

    def test_telemetry_detects_network_not_applied(self):
        weights=[0]*8+[1000]
        row=dict(nn_mode=3, target_speed_mrad_s=6283, nn_delta_uq_mv=100,
                 base_uq_mv=200, uq_mv=200, nn_alpha_milli=1000, clipped=0)
        self.assertFalse(verify([row],weights)['within_telemetry_precision'])
        row['uq_mv']=300
        self.assertTrue(verify([row],weights)['within_telemetry_precision'])


if __name__ == '__main__':
    unittest.main()
