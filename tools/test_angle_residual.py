import unittest
import tempfile
import json
import hashlib
from pathlib import Path
import numpy as np
from train_angle_residual import features, prepare, harmonic_design
from evaluate_angle_residual import evaluate


class AngleResidualTests(unittest.TestCase):
    def test_evaluation_rejects_training_recording_before_loading_model(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'training.csv'
            source.write_text('training data', encoding='utf-8')
            model = Path(directory)/'model.npz'
            model.with_suffix('.json').write_text(json.dumps({
                'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest()
            }), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'differ from training'):
                evaluate(source, model)

    def test_harmonic_comparator_receives_speed_direction_interactions(self):
        x = features(np.array([.2, .2]), np.array([15., -30.]))
        a = harmonic_design(x, 5)
        self.assertEqual(a.shape, (2, 33))
        np.testing.assert_allclose(a[0, :11], a[1, :11])
        self.assertFalse(np.allclose(a[0, 11:], a[1, 11:]))

    def test_periodic_and_direction_features(self):
        p = np.array([0., .2, 4.])
        np.testing.assert_allclose(features(p, 15), features(p+2*np.pi, 15), atol=1e-12)
        self.assertEqual(features(p, -15).shape, (3, 12))
        np.testing.assert_array_equal(features(p, -15)[:, 1], -1)

    def rows(self):
        return [dict(config='pi', repeat=rep, target_rpm=rpm, tick_ms=i*10,
                     position_mrad=round(i*.01*rpm*2*np.pi/60*1000),
                     uq_mv=100, fault=0, clipped=0, sensor_ok=1,
                     calibrated=1, mode=2, nn_mode=0)
                for rep in range(3) for rpm in (-30, 30) for i in range(1000)]

    def test_whole_repeat_split_and_startup_exclusion(self):
        data, masks = prepare(self.rows(), 'pi')
        self.assertEqual(len(data), 4800)
        for i, mask in enumerate(masks):
            self.assertEqual(set(data[mask, 0]), {i})
        self.assertTrue(np.all(sum(m.astype(int) for m in masks) == 1))

    def test_independent_record_can_have_one_repeat(self):
        rows = [r for r in self.rows() if r['repeat'] == 0]
        with self.assertRaises(ValueError):
            prepare(rows, 'pi')
        data, masks = prepare(rows, 'pi', require_three=False)
        self.assertEqual(len(data), 1600)
        self.assertEqual(masks, [])

    def test_unhealthy_data_is_not_silently_dropped(self):
        rows = self.rows()
        rows[100]['fault'] = 1
        with self.assertRaises(ValueError):
            prepare(rows, 'pi')

    def test_nn_enabled_or_insufficient_rotation_rejected(self):
        for field, value in [('nn_mode', 3), ('position_mrad', 0)]:
            rows = self.rows()
            for row in rows:
                row[field] = value
            with self.assertRaises(ValueError):
                prepare(rows, 'pi')


if __name__ == '__main__':
    unittest.main()
