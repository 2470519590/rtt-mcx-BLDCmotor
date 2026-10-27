"""Fixed demo timing and fail-stop tests. No serial or motor hardware."""
import math
import threading
import time
import unittest
from unittest.mock import Mock, patch

from motor_demo import DemoRunner, POSITION_SETTINGS, RESTORE_SETTINGS
from web_monitor import SerialMonitor


class DemoTests(unittest.TestCase):
    def runner(self):
        self.commands, self.states, self.waits = [], [], []
        def command(op, value=0):
            self.commands.append((op, value))
            return 1000 if op == "c" else 0
        runner = DemoRunner(command, lambda: dict(connected=True, updated_at=time.time(),
                            sensor="ok", calibrated=1, fault=0),
                            lambda **values: self.states.append(values))
        runner._wait = self.waits.append
        return runner

    def test_fixed_sequence_and_relative_degrees(self):
        runner = self.runner()
        runner.run()
        self.assertEqual(self.waits, [1, 3, 1, .5, .5, .5, 1])
        targets = [v for op, v in self.commands if op == "X"]
        self.assertEqual(targets, [1000+round(math.radians(d)*1000) for d in (120, 240, 360)])
        self.assertEqual([v for op, v in self.commands if op == "R"], [60])
        self.assertEqual(self.commands[-6], ("S", 0))
        self.assertEqual(tuple(self.commands[-5:]), RESTORE_SETTINGS)
        self.assertEqual(self.states[-1]["phase"], "演示完成")

    def test_speed_and_position_use_separate_tested_gains(self):
        runner = self.runner()
        runner.run()
        run = self.commands.index(("R", 60))
        position = self.commands.index(("X", 1000 + round(math.radians(120)*1000)))
        self.assertIn(("P", 8), self.commands[:run])
        self.assertIn(("Q", 284), self.commands[:run])
        self.assertEqual(tuple(self.commands[position-6:position-1]), POSITION_SETTINGS)
        self.assertIn(("c", 0), self.commands[position-1:position])

    def test_demo_preserves_explicit_web_nn_choice(self):
        runner = self.runner()
        runner.snapshot = lambda: dict(connected=True, sensor="ok", calibrated=1,
                                       fault=0, updated_at=time.time(),
                                       control=dict(nn_mode=2, nn_alpha=.25))
        runner.run()
        self.assertIn(("G", 2), self.commands)
        self.assertIn(("K", 250), self.commands)
        self.assertNotIn(("G", 0), self.commands)
        self.assertNotIn(("K", 0), self.commands)

    def test_old_firmware_stops_without_starting(self):
        runner = self.runner()
        original = runner.command
        def request(op, value=0):
            if op == "c":
                raise ValueError("unsupported position query")
            return original(op, value)
        runner.command = request
        runner.run()
        self.assertEqual(self.commands, [("S", 0)])
        self.assertFalse(self.states[-1]["running"])
        self.assertIn("unsupported", self.states[-1]["error"])

    def test_cancellation_sends_stop_and_no_position(self):
        runner = self.runner()
        def wait(seconds):
            runner.cancel.set()
            raise InterruptedError()
        runner._wait = wait
        runner.run()
        self.assertNotIn("X", [op for op, _ in self.commands])
        self.assertNotIn("R", [op for op, _ in self.commands])
        self.assertEqual(self.commands[-1], ("S", 0))
        self.assertEqual(self.states[-1]["phase"], "演示已停止")

    def test_out_of_range_does_not_send_position(self):
        runner = self.runner()
        original = runner.command
        runner.command = lambda op, value=0: 99000 if op == "c" else original(op, value)
        runner.run()
        self.assertNotIn("X", [op for op, _ in self.commands])
        self.assertEqual(self.commands[-6], ("S", 0))
        self.assertEqual(tuple(self.commands[-5:]), RESTORE_SETTINGS)
        self.assertTrue(self.states[-1]["error"])

    def test_uncalibrated_start_rejected(self):
        runner = self.runner()
        runner.snapshot = lambda: dict(connected=True, sensor="ok", calibrated=0,
                                       updated_at=time.time(), fault=0)
        with self.assertRaisesRegex(ValueError, "校准"):
            runner.start()
        self.assertEqual(self.commands, [])

    def test_web_calibration_waits_for_firmware_and_settling(self):
        runner = self.runner()
        state = dict(connected=True, sensor="ok", calibrated=0,
                     updated_at=time.time(), fault=0, motor_mode=1)
        runner.snapshot = lambda: state
        def wait(seconds, require_calibration=True):
            self.waits.append((seconds, require_calibration))
            state.update(calibrated=1, motor_mode=0)
        runner._wait = wait
        runner.calibrate()
        self.assertEqual(self.commands, [("S", 0), ("C", 0), ("S", 0)])
        self.assertEqual(self.waits, [(3.9, False)])
        self.assertEqual(self.states[-1]["phase"], "校准完成")
        self.assertIsNone(self.states[-1]["error"])

    def test_calibration_ack_is_not_calibration_success(self):
        runner = self.runner()
        runner.snapshot = lambda: dict(connected=True, sensor="ok", calibrated=0,
                                       updated_at=time.time(), fault=0, motor_mode=0)
        runner._wait = lambda *args, **kwargs: None
        runner.calibrate()
        self.assertEqual(self.states[-1]["phase"], "校准未完成")
        self.assertTrue(self.states[-1]["error"])
        self.assertEqual(self.commands[-1], ("S", 0))

    def test_calibration_wait_keeps_existing_watchdog_alive(self):
        runner = self.runner()
        runner.snapshot = lambda: dict(connected=True, sensor="ok", calibrated=0,
                                       updated_at=time.time(), fault=0, motor_mode=1)
        # Use the real wait implementation, not the timing shortcut in runner().
        DemoRunner._wait(runner, .02, require_calibration=False)
        self.assertIn(("H", 0), self.commands)

    def test_stop_cancels_calibration(self):
        runner = self.runner()
        def wait(*args, **kwargs):
            raise InterruptedError()
        runner._wait = wait
        runner.calibrate()
        self.assertEqual(self.states[-1]["phase"], "校准已停止")
        self.assertEqual(self.commands[-1], ("S", 0))

    def test_uncalibrated_device_can_start_web_calibration(self):
        runner = self.runner()
        runner.snapshot = lambda: dict(connected=True, sensor="ok", calibrated=0,
                                       updated_at=time.time(), fault=0, motor_mode=0)
        with patch("motor_demo.threading.Thread") as thread:
            runner.start_calibration()
            thread.return_value.start.assert_called_once()
        self.assertEqual(self.states[-1]["operation"], "calibration")
        self.assertTrue(self.states[-1]["running"])
        runner.worker = Mock()
        runner.worker.is_alive.return_value = True
        with self.assertRaisesRegex(ValueError, "正在"):
            runner.start()
        with self.assertRaisesRegex(ValueError, "正在"):
            runner.start_calibration()

    def test_request_uses_single_reader_to_receive_position(self):
        monitor = SerialMonitor(115200)
        try:
            monitor.ser = Mock()
            monitor.ser.write.side_effect = lambda wire: monitor._consume_line("POS,0,-1234,1,1,0")
            self.assertEqual(monitor.request("c"), -1234)
        finally:
            monitor.ser = None
            monitor.close()

    def test_request_rejects_legacy_ack_and_negative_position_is_data(self):
        monitor = SerialMonitor(115200)
        try:
            monitor.ser = Mock()
            monitor.ser.write.side_effect = lambda wire: monitor._consume_line("ACK,c,-1,pwm_init=0")
            with self.assertRaisesRegex(ValueError, "ACK 而非 POS"):
                monitor.request("c")
            monitor.ser.write.side_effect = lambda wire: monitor._consume_line("POS,0,-1,1,1,0")
            self.assertEqual(monitor.request("c"), -1)
        finally:
            monitor.ser = None
            monitor.close()

    def test_stop_failure_not_reported_as_completion(self):
        runner = self.runner()
        original = runner.command
        def request(op, value=0):
            if op == "S":
                raise ValueError("timeout")
            return original(op, value)
        runner.command = request
        runner.run()
        self.assertEqual(self.states[-1]["phase"], "停机未确认")
        self.assertIn("停机确认失败", self.states[-1]["error"])


if __name__ == "__main__":
    unittest.main()
