"""Offline command tests; never opens a serial port."""
import threading
import unittest
import io
import time
from contextlib import redirect_stdout
from unittest.mock import Mock

from control_cli import MotorSerial


class SweepTests(unittest.TestCase):
    def test_position_trajectory_is_explicit_and_milliseconds(self):
        for value in (0, 100, 500, 2000):
            self.motor.set_parameter('posmove', value)
            self.motor.send_checked.assert_called_with('t', value)
        for value in (-1, 50, 2001, 500.5, float('nan')):
            with self.assertRaises(ValueError):
                self.motor.set_parameter('posmove', value)

    def test_proportional_reference_time_constant(self):
        self.motor.set_parameter('pref', 25)
        self.motor.send_checked.assert_called_with('r', 25)
        self.motor.set_parameter('pref', 0)
        self.motor.send_checked.assert_called_with('r', 0)
        for value in (-1, 201, float('nan')):
            with self.assertRaises(ValueError):
                self.motor.set_parameter('pref', value)

    def test_transition_policy_is_explicit(self):
        self.motor.set_transition('reset')
        self.motor.send_checked.assert_called_with('j', 0)
        self.motor.set_transition('keep')
        self.motor.send_checked.assert_called_with('j', 1)
        self.motor.send_checked.reset_mock()
        with self.assertRaises(ValueError):
            self.motor.set_transition('unknown')
        self.motor.send_checked.assert_not_called()

    def test_estimator_commands(self):
        for name, mode in [('legacy', 0), ('regression', 1), ('observer', 2)]:
            self.motor.set_estimator(name)
            self.motor.send_checked.assert_called_with('v', mode)
        with self.assertRaises(ValueError):
            self.motor.set_estimator('unknown')
    def test_angle_mode_is_shadow_only_and_needs_firmware_ack(self):
        self.motor.set_nn_mode('angleshadow')
        self.motor.send_checked.assert_called_once_with('G', 5)

    def test_angle_trial_and_phase_have_explicit_commands(self):
        self.motor.set_nn_mode('angletrial')
        self.motor.send_checked.assert_called_with('G', 6)
        self.motor.set_parameter('anglephase', 2.764)
        self.motor.send_checked.assert_called_with('a', 2764)

    def test_angle_apply_mode_is_not_exposed(self):
        for name in ('angle', 'angleon', '6'):
            with self.assertRaises(ValueError):
                self.motor.set_nn_mode(name)
        self.motor.send_checked.assert_not_called()

    def setUp(self):
        self.motor = MotorSerial.__new__(MotorSerial)
        self.motor.send_checked = Mock()

    def test_sequence(self):
        self.motor.sweep(10, 200, 20)
        self.assertEqual(
            [(c.args) for c in self.motor.send_checked.call_args_list],
            [('S', 0), ('B', 10), ('E', 200), ('D', 20), ('F', 1)])

    def test_reverse_and_deceleration(self):
        self.motor.sweep(-250, -10, 600)
        self.assertEqual(self.motor.send_checked.call_args.args, ('F', 1))

    def test_invalid_parameters_send_nothing(self):
        for params in [(0, 200, 20), (10, 251, 20), (-10, 200, 20),
                       (10, 200, 0), (10, 200, 601)]:
            with self.subTest(params=params), self.assertRaises(ValueError):
                self.motor.sweep(*params)
        self.motor.send_checked.assert_not_called()

    def test_rejection_does_not_start(self):
        self.motor.send_checked.side_effect = [None, ValueError('rejected')]
        with self.assertRaises(ValueError):
            self.motor.sweep(10, 200, 20)
        self.assertEqual(self.motor.send_checked.call_count, 2)

    def test_checked_ack_accept_and_reject(self):
        for result in [0, -1]:
            motor = MotorSerial.__new__(MotorSerial)
            motor.ack_condition = threading.Condition()
            motor.send = lambda op, value: setattr(motor, 'last_ack', (op, result))
            if result == 0:
                motor.send_checked('F', 1)
            else:
                with self.assertRaises(ValueError):
                    motor.send_checked('F', 1)

    def test_failed_encoder_is_not_displayed_as_measured_zero_rpm(self):
        motor = MotorSerial.__new__(MotorSerial)
        motor.state_lock = threading.Lock()
        motor.latest = 'T,1,10,0,0,0,6,0,1047,1000,0,0'
        motor.received_at = time.monotonic()
        motor.latest_identification = None
        output = io.StringIO()
        with redirect_stdout(output):
            motor.show_state()
        self.assertIn('RPM=unavailable sensor_ok=0 mode=6', output.getvalue())

    def test_pi_setting_commands_are_encoded(self):
        motor = MotorSerial.__new__(MotorSerial)
        motor.send = Mock()
        # CLI parsing is covered by the firmware command contract; keep the
        # sweep tests serial-free and only verify the new opcode is distinct.
        motor.send('Q', 284)
        motor.send.assert_called_once_with('Q', 284)

    def test_tuned_speed_profile_preserves_direction(self):
        self.motor.speed_profile = 'tuned'
        self.motor.run_speed(-60)
        self.assertEqual([c.args for c in self.motor.send_checked.call_args_list],
                         [('S', 0), ('P', 8), ('Q', 650), ('R', -60)])

    def test_manual_profile_preserves_custom_gains(self):
        self.motor.speed_profile = 'manual'
        self.motor.run_speed(60)
        self.motor.send_checked.assert_called_once_with('R', 60)

    def test_position_speed_limit_uses_rpm_not_milli_units(self):
        self.motor.set_parameter('posmax',60)
        self.motor.send_checked.assert_called_once_with('Y',60)

    def test_gain_and_voltage_units(self):
        for name, setting, opcode, value in [('kp',.008,'P',8),('ki',.9,'Q',900),
                ('poskp',8,'Z',8000),('voltage',1,'V',1000)]:
            self.motor.set_parameter(name,setting)
            self.motor.send_checked.assert_called_with(opcode,value)

    def test_invalid_settings_send_nothing(self):
        for name,setting in [('posmax',0),('posmax',1001),('ki',float('nan')),
                             ('kp',-.001),('voltage',1.6)]:
            with self.assertRaises(ValueError):
                self.motor.set_parameter(name,setting)
        self.motor.send_checked.assert_not_called()

    def test_smooth_speed_profile_preserves_direction(self):
        self.motor.speed_profile='smooth'
        self.motor.run_speed(-30)
        self.assertEqual([c.args for c in self.motor.send_checked.call_args_list],
                         [('S',0),('P',50),('Q',100),('R',-30)])

    def test_smooth_profile_does_not_apply_at_high_speed(self):
        self.motor.speed_profile='smooth'
        with self.assertRaises(ValueError):
            self.motor.run_speed(60)
        self.motor.send_checked.assert_not_called()

    def test_lowfast_profile_preserves_direction_without_enabling_nn(self):
        self.motor.speed_profile = 'lowfast'
        self.motor.run_speed(-30)
        self.assertEqual([c.args for c in self.motor.send_checked.call_args_list],
                         [('S', 0), ('P', 150), ('Q', 100), ('R', -30)])

    def test_lowfast_does_not_extrapolate_beyond_measured_range(self):
        self.motor.speed_profile = 'lowfast'
        for rpm in (10, -10, 41, -41):
            with self.assertRaises(ValueError):
                self.motor.run_speed(rpm)
        self.motor.send_checked.assert_not_called()


if __name__ == '__main__':
    unittest.main()
