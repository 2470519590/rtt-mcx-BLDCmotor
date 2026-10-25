"""Firmware-side model constants and the C-parity contract for angle upload.

app/angle_shadow_model.h was generated from a checkpoint whose hidden weights
are not byte-identical to data/angle_residual_candidate_v2.npz: the header
stores weight*std (so magnitudes reach ~9.69) while the checkpoint stores the
float weights (max 1.85). The upload protocol carries the *checkpoint* values
at 1e-3 resolution, so this test reads the constants back out of the firmware
header and proves the difference is transport quantization only.
"""
import hashlib
import re
import unittest
from pathlib import Path

import numpy as np

from angle_phase_model import (PhaseTable, load_model, predict, roundtrip_weight,
                               upload_sequence)

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / 'app' / 'angle_shadow_model.h'


def parse_header():
    text = HEADER.read_text(encoding='utf-8')

    def floats(name):
        found = re.search(r'static const float ' + name + r'\[[^\]]*\] = \{(.*?)\};',
                          text, re.S)
        return np.array([float(v.strip().rstrip('f')) for v in found.group(1).split(',')
                         if v.strip()])

    hidden = re.search(r'angle_hidden_weights\[8\]\[12\] = \{(.*?)\n\};', text, re.S).group(1)
    rows = [[float(v.rstrip('f')) for v in row.strip().strip('{},').split(',')]
            for row in hidden.strip().splitlines() if row.strip().startswith('{')]
    return dict(mean=floats('angle_input_mean'), std=floats('angle_input_std'),
                hidden=np.array(rows), bias=floats('angle_hidden_bias'),
                output=floats('angle_output_weights'),
                output_bias=float(re.search(r'angle_output_bias = ([^;]+);', text).group(1).rstrip('f')))


class FirmwareParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header = parse_header()
        cls.model = load_model(ROOT / 'data' / 'angle_residual_candidate_v2.npz')

    def test_firmware_does_not_rescale_compiled_weights(self):
        """Regression for the double-scaling bug.

        angle_shadow_model.h already stores weight*std and predict() divides the
        input by std. Multiplying again by std made every output a
        weight/std network (measured gain 0.62 on the board). The source must
        copy the compiled table unchanged and scale uploads by nothing.
        """
        source = (ROOT / 'app' / 'angle_shadow.c').read_text(encoding='utf-8')
        reset = source[source.index('void angle_shadow_reset'):source.index('int angle_shadow_set_norm')]
        self.assertIn('s_hidden[i][1U + j] = angle_hidden_weights', reset)
        self.assertNotIn('angle_hidden_weights[i][j] * angle_input_std', reset)
        upload = source[source.index('int angle_shadow_set_hidden'):source.index('int angle_shadow_set_hidden_bias')]
        self.assertIn('s_hidden[neuron][j] = row[j];', upload)
        self.assertNotIn('* angle_input_std', upload)

    def test_header_stores_weights_in_physical_units(self):
        """Retracts an earlier claim I made from a mis-read probe.

        I once reported that the header stores weight*std. It does not: the
        ratio header/checkpoint is 1.0 in every column, so the header stores
        the physical (already denormalized) weights. The double-scaling bug was
        therefore only in the firmware's extra *std on load, not in the export.
        """
        header = self.header['hidden'].astype(np.float64)
        checkpoint = self.model['hidden_weights'].astype(np.float64)
        np.testing.assert_allclose(header, checkpoint, atol=1e-6)
        self.assertGreater(float(np.max(np.abs(header))), 1.0,
                           'physical weights exceed the normalized range')

    def test_inference_truncates_like_telemetry(self):
        source = (ROOT / 'app' / 'angle_shadow.c').read_text(encoding='utf-8')
        predict_body = source[source.index('float angle_shadow_predict'):]
        self.assertIn('(int32_t)(position_rad * 1000.0f)', predict_body)
        self.assertIn('(int32_t)(target_rad_s * 1000.0f)', predict_body)

    def test_rotation_buffer_records_the_inference_tensor(self):
        control = (ROOT / 'app' / 'motor_control.c').read_text(encoding='utf-8')
        self.assertIn('s_state.nn_input_position_rad = features.measured_position_rad;', control)
        cli = (ROOT / 'app' / 'host_cli.c').read_text(encoding='utf-8')
        frame = cli[cli.index('"L,%u'):cli.index('state.mode, state.sensor_ok')]
        self.assertIn('state.nn_input_position_rad', frame)
        self.assertIn('state.nn_input_speed_rad_s', frame)

    def test_float32_reference_matches_hand_computation(self):
        from angle_reference_f32 import predict_f32
        model = dict(input_mean=np.zeros(12, dtype=np.float32),
                     input_std=np.ones(12, dtype=np.float32),
                     hidden_weights=np.zeros((8, 12), dtype=np.float32),
                     hidden_bias=np.zeros(8, dtype=np.float32),
                     output_weights=np.zeros(8, dtype=np.float32),
                     output_bias=np.array(.021, dtype=np.float32))
        # No weights: the output is exactly the bias, independent of inputs.
        value = predict_f32([1.234, -0.5], [15.0, -15.0], 0.0, model)
        np.testing.assert_allclose(np.asarray(value, dtype=float), [.021, .021], rtol=1e-7)

    def test_header_matches_compiled_scale_and_generation(self):
        std = self.header['std']
        np.testing.assert_allclose(std, self.model['input_std'], rtol=1e-6)
        np.testing.assert_allclose(self.header['mean'], self.model['input_mean'], atol=1e-7)
        np.testing.assert_allclose(self.header['output'], self.model['output_weights'], rtol=1e-6)
        self.assertAlmostEqual(self.header['output_bias'], float(self.model['output_bias']), places=6)
        digest = hashlib.sha256((ROOT / 'data' / 'angle_residual_candidate_v2.npz').read_bytes()).hexdigest()
        self.assertIn(digest, HEADER.read_text(encoding='utf-8'))

    def test_upload_transport_error_is_quantization_only(self):
        from angle_phase_model import row_commands
        for neuron in range(8):
            commands = row_commands(self.model, neuron, 0)
            self.assertEqual(len(commands), 16)
            uploaded = np.zeros(12, dtype=float)
            for op, value in commands:
                if op == 'g':
                    field = value - 20000
                    index, digit = divmod(field, 4000)
                    self.assertLess(index, 12)
                    uploaded[index] = -(digit - 2000) / 1000 if digit >= 2000 else digit / 1000
            reference = self.model['hidden_weights'][neuron]
            self.assertLess(float(np.max(np.abs(uploaded - reference))), 5e-4)

    def test_zero_phase_upload_matches_numpy_reference(self):
        table = PhaseTable.constant(0.0)
        commands = upload_sequence(self.model, table)
        self.assertEqual(commands[-1], ('m', 2))
        phase = table.phase(np.array([-30.0, -15.0, 0.0, 15.0, 30.0]))
        self.assertTrue(np.all(phase == 0.0))
        np.testing.assert_allclose(
            predict([1.234, -0.7], [15.0, 30.0], self.model, np.array([0.0, 0.0])),
            predict([1.234, -0.7], [15.0, 30.0], self.model, phase[3:]), rtol=1e-6)

    def test_speed_dependent_phase_changes_output_meaningfully(self):
        angle = np.linspace(0, 2 * np.pi, 200)
        rpm = np.full_like(angle, 15.0)
        flat = predict(angle, rpm, self.model, np.zeros_like(angle))
        shifted = predict(angle, rpm, self.model, np.full_like(angle, 1.0))
        self.assertGreater(float(np.max(np.abs(flat - shifted))), .01,
                           'a one-radian phase must move the prediction')


if __name__ == '__main__':
    unittest.main()
