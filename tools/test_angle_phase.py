"""Dynamic (speed-dependent) phase: offline fitting and upload contract.

Covers the maths that the firmware implements, the upload sequence that
mirrors app/angle_shadow.c, and the rejection rules that keep a partial
upload from silently changing the model.
"""
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from angle_phase_model import (HARMONICS, NORM_POINTS, PhaseTable, features,
                               load_model, phase_milli, predict, upload_sequence, weight_milli)

ROOT = Path(__file__).resolve().parents[1]


def synthetic_model(hidden=8, seed=3):
    rng = np.random.default_rng(seed)
    return dict(input_mean=rng.normal(0, .1, 12).astype(np.float32),
                input_std=(np.abs(rng.normal(1, .1, 12)) + .1).astype(np.float32),
                hidden_weights=rng.normal(0, .5, (hidden, 12)).astype(np.float32),
                hidden_bias=rng.normal(0, .2, hidden).astype(np.float32),
                output_weights=rng.normal(0, .1, hidden).astype(np.float32),
                output_bias=np.array(.017, dtype=np.float32),
                sha256='synthetic')


class PhaseMathTests(unittest.TestCase):
    def test_constant_table_is_speed_independent(self):
        table = PhaseTable.constant(-2.667)
        self.assertTrue(np.allclose(table.phase([-30, -15, 0, 15, 30]), -2.667))

    def test_sign_is_separate_across_normalization_points(self):
        table = PhaseTable([(-15, 1.0), (15, -1.0)], interp=0)
        np.testing.assert_allclose(table.phase([-30, -15, -5, 5, 15, 30]), [1, 1, 1, -1, -1, -1])

    def test_interpolation_is_bounded_by_neighbours(self):
        table = PhaseTable([(-30, -2.0), (0, 0.0), (30, 2.0)], interp=1)
        values = table.phase(np.linspace(-30, 30, 13))
        self.assertTrue(np.all(np.diff(values) >= -1e-12))
        self.assertLessEqual(float(np.max(np.abs(values))), 2.0)

    def test_zero_phase_matches_original_feature_layout(self):
        angle = np.array([0.1, 2.0, 6.2])
        rpm = np.array([15.0, -15.0, 30.0])
        shifted = features(angle, rpm, np.zeros_like(rpm))
        legacy = np.column_stack([rpm / 40, np.sign(rpm)] +
                                 [f(h * angle) for h in HARMONICS for f in (np.sin, np.cos)])
        np.testing.assert_allclose(shifted, legacy)

    def test_phase_equals_angle_offset_for_each_harmonic(self):
        angle = np.array([0.3, 3.0])
        rpm = np.array([15.0, 15.0])
        np.testing.assert_allclose(features(angle, rpm, np.full(2, .7)),
                                   features(angle + .7, rpm, np.zeros(2)))

    def test_rejects_unordered_duplicate_or_out_of_range_points(self):
        with self.assertRaises(ValueError):
            PhaseTable([(15, .1), (15, .2)])
        with self.assertRaises(ValueError):
            PhaseTable([(0, 4.0)])
        with self.assertRaises(ValueError):
            PhaseTable([(i * 5.0, .1) for i in range(NORM_POINTS + 1)])

    def test_upload_values_stay_inside_firmware_ranges(self):
        self.assertEqual(phase_milli(-3.142), -3142)
        with self.assertRaises(ValueError):
            phase_milli(3.143)
        self.assertEqual(weight_milli(.5, 3), 300000 + 5000)
        with self.assertRaises(ValueError):
            weight_milli(20.0, 0)


class UploadContractTests(unittest.TestCase):
    def test_sequence_order_commits_neurons_then_points(self):
        model = synthetic_model()
        table = PhaseTable([(-15, .4), (15, -.4)], interp=1)
        commands = upload_sequence(model, table)
        ops = [op for op, _ in commands]
        self.assertEqual(ops.count('g'), 8 * 12)
        self.assertEqual(ops.count('n'), 8)
        self.assertEqual(ops.count('p'), 2)
        # Interpolation/format mode is the final switch, after every point.
        self.assertEqual(commands[-1], ('m', 3))
        indices = [index for index, op in enumerate(ops) if op == 'p']
        self.assertEqual([commands[i] for i in indices], [('p', -15000), ('p', 15000)])
        for index in indices:
            self.assertEqual(commands[index - 2][0], 'M')
            self.assertEqual(commands[index - 1][0], 'u')
        first = commands[commands.index(('M', 0)):commands.index(('M', 1))]
        self.assertEqual([op for op, _ in first], ['M'] + ['g'] * 12 + ['i', 'h', 'n', 'm'])
        self.assertEqual([(value - 20000) // 4000 for op, value in first if op == 'g'],
                         list(range(12)))

    def test_upload_reproduces_checkpoint_magnitudes(self):
        model = load_model(ROOT / 'data/angle_residual_candidate_v2.npz')
        commands = upload_sequence(model, PhaseTable.constant(0.4))
        uploaded = np.full((8, 12), np.nan)
        neuron = None
        committed = set()
        for op, value in commands:
            if op == 'M':
                neuron = value
            elif op == 'g':
                field = value - 20000
                digit = field % 4000
                uploaded[neuron, field // 4000] = (-(digit - 2000) if digit >= 2000 else digit) / 1000
            elif op == 'n':
                committed.add(neuron)
        self.assertEqual(committed, set(range(8)))
        self.assertTrue(np.all(np.isfinite(uploaded)))
        np.testing.assert_array_equal(np.signbit(uploaded),
                                      np.signbit(model['hidden_weights']))
        self.assertLess(float(np.max(np.abs(uploaded - model['hidden_weights']))), 5e-4)
        self.assertLess(float(np.max(uploaded)), 2.999)

    def test_rejects_weights_beyond_the_q28_field(self):
        model = synthetic_model()
        model['hidden_weights'][0, 0] = 12.0
        with self.assertRaises(ValueError):
            upload_sequence(model, PhaseTable.constant(0.0))

    def test_default_table_is_bit_identical_to_compiled_candidate(self):
        model = load_model(ROOT / 'data/angle_residual_candidate_v2.npz')
        header = (ROOT / 'app/angle_shadow_model.h').read_text(encoding='utf-8')
        self.assertIn(model['sha256'], header)
        # One point with phase 0 and nearest-hold is exactly the old behavior:
        # the phase table is empty-equivalent and no new coefficient is applied.
        table = PhaseTable.constant(0.0)
        commands = upload_sequence(model, table)
        self.assertEqual([c for c in commands if c[0] in ('p', 'm')][-2:], [('p', 0), ('m', 2)])
        self.assertEqual(commands[-1], ('m', 2))
        self.assertEqual(predict([1.0], [15.0], model, table.phase([15.0])).shape, (1,))

    def test_model_loader_rejects_wrong_schema(self):
        directory = ROOT / 'debug' / 'phase_test_tmp'
        directory.mkdir(parents=True, exist_ok=True)
        source = directory / 'bad.npz'
        np.savez(source, schema='other_schema', harmonics=HARMONICS,
                 **{k: v for k, v in synthetic_model().items() if k != 'sha256'})
        try:
            with self.assertRaises(ValueError):
                load_model(source)
        finally:
            source.unlink(missing_ok=True)

    def test_report_roundtrip_is_deterministic(self):
        table = PhaseTable([(-15, .25), (15, -.25)], interp=0)
        document = json.loads(json.dumps({'tables': {'t': table.as_dict()}}))
        rebuilt = PhaseTable([(p['rpm'], p['phase_rad']) for p in document['tables']['t']['points']],
                             interp=document['tables']['t']['interp'])
        self.assertEqual(table.phase([-20, 20]).tolist(), rebuilt.phase([-20, 20]).tolist())


if __name__ == '__main__':
    unittest.main()
