"""Candidate preprocessing must preserve physical invariants and split isolation."""
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from export_nn_weights import c_float, main as export_main
from train_nn_candidate import prepare, split_groups


class CandidateTests(unittest.TestCase):
    def test_translation_and_boundary_reset(self):
        rows = [{"mode": "8", "target_position_mrad": "1000", "tick_ms": str(i*10)}
                for i in range(3)]
        rows[2]["target_position_mrad"] = "2000"
        raw = np.array([[1, 0, 1, 1, .5, .2], [1, 0, 2, 1, .6, .2],
                        [1, 0, 4, 2, .7, .2]], dtype=np.float32)
        x, _, groups = prepare(rows, raw)
        shifted = raw.copy()
        shifted[:, 3:5] += 200
        other, _, _ = prepare(rows, shifted)
        np.testing.assert_allclose(x, other, atol=1e-5)
        self.assertGreater(x[1, 3], 0)
        self.assertEqual(x[2, 3], 0)
        np.testing.assert_array_equal(groups, [0, 0, 1])

    def test_whole_segments_never_overlap(self):
        groups = np.repeat(np.arange(10), 20)
        masks = split_groups(groups)
        np.testing.assert_array_equal(sum(m.astype(int) for m in masks), np.ones(200))
        sets = [set(groups[m]) for m in masks]
        self.assertFalse(sets[0] & sets[1] or sets[0] & sets[2] or sets[1] & sets[2])

    def test_c_literals(self):
        self.assertEqual(c_float(0), "0.0f")
        self.assertEqual(c_float(1), "1.0f")
        with self.assertRaises(ValueError):
            c_float(float("nan"))

    def test_candidate_cannot_overwrite_deployed_header(self):
        with tempfile.TemporaryDirectory() as temp:
            archive = Path(temp) / "candidate.npz"
            header = Path(temp) / "nn_model.h"
            header.write_text("working baseline", encoding="utf-8")
            np.savez(archive, schema="residual_v2")
            with patch("sys.argv", ["export_nn_weights.py", str(archive), "--output", str(header)]):
                with self.assertRaisesRegex(ValueError, "incompatible"):
                    export_main()
            self.assertEqual(header.read_text(encoding="utf-8"), "working baseline")


if __name__ == "__main__":
    unittest.main()
