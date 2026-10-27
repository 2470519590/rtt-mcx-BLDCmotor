"""Manual web control regression; no real serial port or motor."""
import math
import unittest
from unittest.mock import Mock, patch

from web_control import DEFAULT_PROFILES, position_target, profile_settings
from web_monitor import SerialMonitor


class WebControlTests(unittest.TestCase):
    def setUp(self):
        self.guard = patch("web_monitor.serial.Serial",
                           side_effect=AssertionError("No hardware in host tests"))
        self.guard.start()
        self.monitor = SerialMonitor(115200)
        # All following execution is synchronous, with explicit fake ACKs.
        self.monitor.stop_event.set()
        self.monitor.thread.join(1)
        self.monitor.keeper.join(1)
        self.monitor.demo._healthy = Mock(side_effect=AssertionError("No duplicate web interlock"))
        self.commands = []

        def request(op, value=0):
            self.commands.append((op, value))
            if op == "c":
                return -1000
            with self.monitor.changed:
                self.monitor._acknowledge(op, value)
            return 0

        self.monitor.request = Mock(side_effect=request)

    def tearDown(self):
        self.monitor.ser = None
        self.monitor.close()
        self.guard.stop()

    def test_profile_units_match_firmware_and_cli(self):
        values, commands = profile_settings(DEFAULT_PROFILES["position"])
        self.assertEqual(values["kp"], .04)
        self.assertEqual(commands, [("P", 40), ("Q", 650), ("Z", 15000),
                                    ("Y", 90), ("t", 500), ("V", 1000)])

    def test_bad_parameters_are_rejected_before_any_write(self):
        for changes in ({"kp": float("nan")}, {"posmove": 99},
                        {"posmax": 2.5}, {"voltage": 12}, {"ki": True}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    self.monitor.command(dict(action="parameters", profile="position",
                                              confirmed=True,
                                              values=DEFAULT_PROFILES["position"] | changes))
        self.assertEqual(self.commands, [])

    def test_all_parameter_fields_are_required(self):
        with self.assertRaises(ValueError):
            profile_settings({"kp": .04})

    def test_parameter_and_nn_changes_require_exact_confirmation(self):
        for action in ("parameters", "nn"):
            for confirmed in (None, False, "true", 1):
                with self.subTest(action=action, confirmed=confirmed):
                    with self.assertRaisesRegex(ValueError, "二次确认"):
                        self.monitor.command(dict(action=action, confirmed=confirmed))
        self.assertEqual(self.commands, [])

    def test_apply_parameters_stops_and_does_not_restart(self):
        values = DEFAULT_PROFILES["position"] | {"kp": .051}
        self.monitor.command(dict(action="parameters", profile="position",
                                  values=values, confirmed=True))
        self.assertEqual(self.commands[0], ("S", 0))
        self.assertNotIn("R", [op for op, _ in self.commands])
        self.assertNotIn("X", [op for op, _ in self.commands])
        self.assertEqual(self.monitor.profiles["position"]["kp"], .051)
        self.assertEqual(self.monitor.snapshot()["control"]["applied"]["kp"], .051)
        self.assertEqual(self.monitor.profiles["speed"]["kp"], .008)

    def test_unknown_nn_state_is_not_reported_as_off(self):
        self.assertIsNone(self.monitor.snapshot()["control"]["nn_mode"])
        self.assertIsNone(self.monitor.snapshot()["control"]["nn_alpha"])

    def test_nn_on_sets_nonzero_alpha_and_both_settings_acknowledged(self):
        self.monitor.command(dict(action="nn", enabled=True, alpha=.25, confirmed=True))
        self.assertEqual(self.commands, [("S", 0), ("K", 250), ("G", 2)])
        state = self.monitor.snapshot()["control"]
        self.assertEqual(state["nn_mode"], 2)
        self.assertEqual(state["nn_alpha"], .25)
        self.assertFalse(state["manual"]["active"])
        self.monitor.command(dict(action="nn", enabled=False, alpha=.25, confirmed=True))
        self.assertEqual(self.commands[-1], ("G", 0))

    def test_failed_nn_ack_does_not_claim_nn_enabled(self):
        original = self.monitor.request.side_effect
        self.monitor.request.side_effect = (
            lambda op, value=0: (_ for _ in ()).throw(ValueError("rejected"))
            if op == "G" else original(op, value))
        with self.assertRaisesRegex(ValueError, "rejected"):
            self.monitor.command(dict(action="nn", enabled=True, alpha=.25, confirmed=True))
        self.assertIsNone(self.monitor.snapshot()["control"]["nn_mode"])

    def test_speed_direction_and_same_profile_does_not_stop_on_each_target(self):
        self.monitor.command(dict(action="run", rpm=-60))
        self.assertIn(("P", 8), self.commands)
        self.assertIn(("Q", 284), self.commands)
        self.assertEqual(self.commands[-1], ("R", -60))
        self.assertTrue(self.monitor.manual["active"])
        self.commands.clear()
        self.monitor.command(dict(action="run", rpm=30))
        self.assertEqual(self.commands, [("R", 30)])

    def test_speed_zero_is_stop_and_invalid_speed_never_writes(self):
        self.monitor.command(dict(action="run", rpm=0))
        self.assertEqual(self.commands, [("S", 0)])
        self.commands.clear()
        for rpm in (1001, -1001, 2.5, float("inf"), "60", True, 10 ** 500):
            with self.assertRaises(ValueError):
                self.monitor.command(dict(action="run", rpm=rpm))
        self.assertEqual(self.commands, [])

    def test_position_relative_queries_unwrapped_position_and_uses_smooth_profile(self):
        self.monitor.command(dict(action="position", degrees=120, reference="relative"))
        self.assertEqual(self.commands[0], ("c", 0))
        self.assertIn(("P", 40), self.commands)
        self.assertIn(("t", 500), self.commands)
        self.assertEqual(self.commands[-1], ("X", -1000 + round(math.radians(120) * 1000)))
        self.commands.clear()
        self.monitor.command(dict(action="position", degrees=-10, reference="absolute"))
        self.assertEqual(self.commands, [("X", round(math.radians(-10) * 1000))])

    def test_position_overflow_is_not_wrapped_or_reset(self):
        self.assertEqual(position_target(-180, "absolute"), -3142)
        with self.assertRaises(ValueError):
            position_target(120, "relative", 99000)
        with self.assertRaises(ValueError):
            self.monitor.command(dict(action="position", degrees=6000, reference="absolute"))
        self.assertEqual(self.commands, [])

    def test_manual_changes_cannot_interleave_with_demo(self):
        self.monitor.demo_state["running"] = True
        with self.assertRaisesRegex(ValueError, "停止演示"):
            self.monitor.command(dict(action="run", rpm=60))
        self.assertEqual(self.commands, [])
        self.monitor.command(dict(action="stop"))
        self.assertEqual(self.commands[-1], ("S", 0))

    def test_legacy_browser_heartbeat_is_optional_without_second_uart_reader(self):
        self.monitor.manual["active"] = True
        self.monitor.command(dict(action="heartbeat"))
        self.assertEqual(self.commands, [])
        with patch.object(self.monitor.stop_event, "wait", side_effect=[False, True]):
            self.monitor._manual_keepalive()
        self.assertEqual(self.commands, [("H", 0)])

    def test_manual_run_persists_without_any_browser_heartbeat(self):
        self.monitor.command(dict(action="run", rpm=60))
        self.commands.clear()
        with patch.object(self.monitor.stop_event, "wait", side_effect=[False] * 30 + [True]):
            self.monitor._manual_keepalive()
        self.assertEqual(self.commands, [("H", 0)] * 30)
        self.assertTrue(self.monitor.manual["active"])

    def test_explicit_stop_ends_keepalive(self):
        self.monitor.manual["active"] = True
        self.monitor.command(dict(action="stop"))
        with patch.object(self.monitor.stop_event, "wait", side_effect=[False, True]):
            self.monitor._manual_keepalive()
        self.assertEqual(self.commands, [("S", 0)])
        self.assertFalse(self.monitor.manual["active"])

    def test_stale_display_state_does_not_reject_firmware_commands(self):
        self.monitor.data.update(sensor="error", calibrated=0, fault=2, updated_at=1)
        self.monitor.command(dict(action="run", rpm=60))
        self.assertEqual(self.commands[-1], ("R", 60))
        self.monitor.command(dict(action="position", degrees=120, reference="relative"))
        self.assertEqual(self.commands[-1], ("X", -1000 + round(math.radians(120) * 1000)))
        self.monitor.demo._healthy.assert_not_called()

    def test_firmware_fault_is_not_shown_as_running_manual_control(self):
        self.monitor.manual["active"] = True
        self.monitor._consume_line("T,10,100,1536,0,1,3,1,0,0,2,-888")
        self.assertFalse(self.monitor.manual["active"])
        self.assertIn("故障 2", self.monitor.manual["message"])


if __name__ == "__main__":
    unittest.main()
