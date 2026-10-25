"""Synthetic/offline validation only; no claim of real-motor identification."""
import math
import unittest
import tempfile
import csv
from pathlib import Path
from unittest.mock import Mock
import threading
import time
import io
from contextlib import redirect_stdout

from motor_identification import (TRACE_FIELDS, analyze_rows, analyze_file,
                                  fit_first_order, parse_trace, telemetry_from_trace,
                                  validate_settings, TraceRecorder)
from control_cli import MotorSerial
from nn_dataset import parse_learning


class LearningProtocolTests(unittest.TestCase):
    def test_learning_frame_protocol(self):
        line = 'L,1,100,1000,200,300,400,500,60,70,2,1,1,0,0,8,1000,250,1'
        row = parse_learning(line)
        self.assertEqual(row['target_position_mrad'], 1000)
        self.assertEqual(row['nn_mode'], 1)


def synthetic_point(point, hz, gain=12., tau=.12, clipped=False):
    transfer = gain / complex(1, 2 * math.pi * hz * tau)
    input_phasor = complex(.06, -.03)
    theta_phasor = transfer * input_phasor / complex(0, 2 * math.pi * hz)
    rows = []
    for sample in range(math.ceil(5 / hz * 100)):
        t = sample / 100
        phase = 2 * math.pi * hz * t
        carrier = complex(math.cos(phase), math.sin(phase))
        row = dict.fromkeys(TRACE_FIELDS, 0)
        row.update(sequence=sample, tick_ms=sample * 10, point=point, stage=2,
                   frequency_mhz=round(hz * 1000), phase_mrad=(phase % (2*math.pi))*1000,
                   position_mrad=(6*t + (theta_phasor * carrier).real) * 1000,
                   speed_mrad_s=(6 + (transfer * input_phasor * carrier).real) * 1000,
                   uq_mv=(.5 + (input_phasor * carrier).real)*1000,
                   excitation_mv=.05*math.sin(phase)*1000, sensor_ok=1,
                   calibrated=1, mode=7, clipped=int(clipped))
        rows.append(row)
    return rows, transfer


class IdentificationTests(unittest.TestCase):
    def test_settings_and_rejections(self):
        self.assertEqual(validate_settings(60, .05, .2, 5, 8),
                         [('J', 60), ('A', 50), ('L', 200), ('U', 5000), ('N', 8)])
        for settings in [(0, .05, .2, 5, 8), (60, float('nan'), .2, 5, 8),
                         (60, .4, .2, 5, 8), (60, .05, 5, .2, 8),
                         (60, .05, .2, 11, 8), (60, .05, .2, 5, 1)]:
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                validate_settings(*settings)

    def test_protocol(self):
        values = [1, 100, 2, 2, 500, 1000, 12000, 1000, 6283, 50, 600,
                  1, 1, 7, 0, 0, 2048, 80, 1000]
        row = parse_trace('I,' + ','.join(map(str, values)))
        self.assertEqual(row['uq_mv'], 600)
        self.assertEqual(telemetry_from_trace(row), 'T,1,100,2048,1000,1,7,1,6283,600,0,0')
        with self.assertRaises(ValueError):
            parse_trace('I,1,2')

    def test_known_frequency_response_and_model(self):
        combined = []
        for point, hz in enumerate([.2, .5, 1, 2, 5], 1):
            rows, expected = synthetic_point(point, hz)
            combined.extend(rows)
            result = analyze_rows(rows)[0]
            self.assertTrue(result['valid'], result)
            measured = complex(result['plant_real'], result['plant_imag'])
            self.assertAlmostEqual(abs(measured / expected - 1), 0, places=6)
        model = fit_first_order(analyze_rows(combined))
        self.assertTrue(model['accepted'])
        self.assertAlmostEqual(model['gain'], 12, places=6)
        self.assertAlmostEqual(model['tau_s'], .12, places=6)

    def test_invalid_clipping_sensor_partial_gap(self):
        for fault in ['clip', 'sensor', 'partial', 'gap', 'duplicate']:
            rows, _ = synthetic_point(1, 1)
            if fault == 'clip':
                rows[20]['clipped'] = 1
            elif fault == 'sensor':
                rows[20]['sensor_ok'] = 0
            elif fault == 'partial':
                rows = rows[:100]
            elif fault == 'duplicate':
                rows[20]['tick_ms'] = rows[19]['tick_ms']
            else:
                rows = rows[:20] + rows[50:]
            with self.subTest(fault=fault):
                self.assertFalse(analyze_rows(rows)[0]['valid'])

    def test_csv_and_bode_end_to_end(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'trace.csv'
            recorder = TraceRecorder(path)
            for point, hz in enumerate([.2, 1, 5], 1):
                rows, _ = synthetic_point(point, hz)
                for row in rows:
                    row['host_monotonic_s'] = 0
                    recorder.write(row)
            recorder.close()
            with redirect_stdout(io.StringIO()):
                results = analyze_file(path)
            self.assertEqual(sum(r['valid'] for r in results), 3)
            self.assertTrue(path.with_name('trace_frf.csv').exists())
            self.assertTrue(path.with_name('trace_frf.svg').exists())
            with self.assertRaises(FileExistsError):
                TraceRecorder(path)

    def test_identify_command_sequence_no_serial(self):
        motor = MotorSerial.__new__(MotorSerial)
        motor.state_lock = threading.Lock()
        motor.record_lock = threading.Lock()
        motor.recorder = None
        motor.latest = 'T,1,10,200,0,1,0,1,0,0,0,900'
        motor.received_at = time.monotonic()
        motor.send_checked = Mock()
        with tempfile.TemporaryDirectory() as temp:
            with redirect_stdout(io.StringIO()):
                motor.identify(60, .05, .2, 5, 8, str(Path(temp) / 'trace.csv'))
                self.assertEqual([c.args for c in motor.send_checked.call_args_list],
                                 [('S', 0), ('J', 60), ('A', 50), ('L', 200),
                                  ('U', 5000), ('N', 8), ('I', 1)])
                motor.finish_recording('offline test')
        motor.latest = 'T,1,10,200,0,0,0,0,0,0,0,0'
        motor.send_checked.reset_mock()
        with self.assertRaises(ValueError):
            motor.identify(60, .05, .2, 5, 8)
        motor.send_checked.assert_not_called()


if __name__ == '__main__':
    unittest.main()
