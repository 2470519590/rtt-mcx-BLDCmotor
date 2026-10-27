"""Position comparison metrics and report serialization; no hardware."""
import json
import math
import unittest
from unittest.mock import patch

from nn_dataset import LEARNING_FIELDS
from tune_position import CONFIGS, capture_summary, metrics
from report_position_tuning import comparison
from uart_evidence import Framer


class PositionTuningTests(unittest.TestCase):
    def test_existing_demo_profile_is_explicit_reference(self):
        self.assertEqual(CONFIGS["demo"], (8, 284, 4000, 120))

    def test_exact_negative_position_has_zero_error(self):
        rows = [dict(tick_ms=index * 10, position_mrad=-3000,
                     speed_mrad_s=0, uq_mv=-100, clipped=0)
                for index in range(100)]
        result = metrics(rows, -3000)
        self.assertEqual(result["rmse_deg"], 0)
        self.assertEqual(result["tail_p2p_deg"], 0)
        self.assertEqual(result["tail_speed_rms_rpm"], 0)
        self.assertAlmostEqual(result["end_error_deg"], 0)
        json.dumps(result)

    def test_hold_oscillation_is_not_hidden_by_zero_mean_error(self):
        rows = [dict(tick_ms=index * 10, position_mrad=1000 + (-1) ** index * 100,
                     speed_mrad_s=(-1) ** index * 1000,
                     uq_mv=(-1) ** index * 100, clipped=0)
                for index in range(100)]
        result = metrics(rows, 1000)
        self.assertAlmostEqual(result["tail_p2p_deg"], .2 * 180 / math.pi)
        self.assertAlmostEqual(result["tail_speed_rms_rpm"], 60 / (2 * math.pi))
        self.assertAlmostEqual(result["tail_voltage_p2p_v"], .2)

    def test_uart_report_replays_and_is_json_serializable(self):
        replay = Framer()
        rows = []
        for seq, tick in ((20, 200), (21, 210), (22, 220)):
            values = dict.fromkeys(LEARNING_FIELDS, 0)
            values.update(sequence=seq, tick_ms=tick, position_mrad=-2000)
            rows.append("L," + ",".join(str(values[key]) for key in LEARNING_FIELDS))
        replay.feed(("\r\n".join(rows) + "\r\n").encode(), 0, 100)
        with patch("tune_position.replay_capture", return_value=replay):
            result = capture_summary("not-a-device")
        self.assertEqual(result["learning_frames"], 3)
        self.assertEqual(result["learning_seq_steps"], {1: 2})
        self.assertEqual(result["learning_tick_steps"], {10: 2})
        self.assertEqual(result["combined_seq_steps"], {1: 2})
        self.assertNotIn("other_lines", result)
        json.dumps(result)

    def test_interleaved_telemetry_is_not_reported_as_lost_learning(self):
        replay = Framer()
        learning = dict.fromkeys(LEARNING_FIELDS, 0)
        learning.update(sequence=20, tick_ms=200)
        first = "L," + ",".join(str(learning[k]) for k in LEARNING_FIELDS)
        learning.update(sequence=22, tick_ms=220)
        last = "L," + ",".join(str(learning[k]) for k in LEARNING_FIELDS)
        replay.feed((first + "\nT,21,210,0,0,1,0,1,0,0,0,0\n" + last + "\n").encode(),
                    0, 100)
        with patch("tune_position.replay_capture", return_value=replay):
            result = capture_summary("not-a-device")
        self.assertEqual(result["learning_seq_steps"], {2: 1})
        self.assertEqual(result["combined_seq_steps"], {1: 2})

    def test_comparison_does_not_silently_accept_missing_trials(self):
        with self.assertRaises(ValueError):
            comparison([])


if __name__ == "__main__":
    unittest.main()
