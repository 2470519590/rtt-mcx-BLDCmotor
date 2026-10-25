"""Offline tests for the learning-data and MCU model export pipeline."""
import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from export_nn_weights import main as export_main
from nn_model import load_learning_csv, standardize, teacher_target, train_mlp


class NnPipelineTests(unittest.TestCase):
    def test_train_and_export(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "learning.csv"
            fields = ["host_monotonic_s", "sequence", "tick_ms",
                      "target_position_mrad", "position_mrad",
                      "target_speed_mrad_s", "speed_mrad_s", "base_uq_mv",
                      "nn_delta_uq_mv", "uq_mv", "mode", "sensor_ok",
                      "calibrated", "fault", "clipped", "speed_kp_milli",
                      "voltage_limit_mv", "nn_alpha_milli", "nn_mode"]
            with source.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for i in range(50):
                    speed = 1000 + i * 10
                    writer.writerow({
                        "host_monotonic_s": i / 100.0, "sequence": i,
                        "tick_ms": i * 10, "target_position_mrad": 0,
                        "position_mrad": 0, "target_speed_mrad_s": 2000,
                        "speed_mrad_s": speed, "base_uq_mv": 400,
                        "nn_delta_uq_mv": 0, "uq_mv": 400, "mode": 2,
                        "sensor_ok": 1, "calibrated": 1, "fault": 0,
                        "clipped": 0, "speed_kp_milli": 8,
                        "voltage_limit_mv": 1000, "nn_alpha_milli": 0,
                        "nn_mode": 1,
                    })
            rows, raw = load_learning_csv(source)
            normalized, mean, std = standardize(raw)
            model = train_mlp(normalized, teacher_target(rows, raw), epochs=20)
            self.assertEqual(normalized.shape, (50, 6))
            self.assertTrue(np.isfinite(model["mse"]))
            archive = root / "model.npz"
            np.savez(archive, input_mean=mean, input_std=std,
                     hidden_weights=model["w1"], hidden_bias=model["b1"],
                     output_weights=model["w2"], output_bias=model["b2"],
                     output_limit=np.float32(.15))
            output = root / "nn_model.h"
            import sys
            old = sys.argv
            try:
                sys.argv = ["export_nn_weights.py", str(archive), "--output", str(output)]
                export_main()
            finally:
                sys.argv = old
            text = output.read_text(encoding="utf-8")
            self.assertIn("NN_MODEL_INPUTS 6U", text)
            self.assertIn("nn_model_output_limit", text)


if __name__ == "__main__":
    unittest.main()
