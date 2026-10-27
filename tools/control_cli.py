"""Direct external USB-UART motor control. No web server required."""
import argparse
import threading
import time

import serial
import math
from datetime import datetime
from motor_identification import (TraceRecorder, analyze_file, parse_trace,
                                  telemetry_from_trace, validate_settings)
from motor_port import resolve_port
from nn_dataset import parse_learning
from motor_profiles import speed_gains
from motor_feedforward import load_weights

MAX_RPM = 1000
OPENLOOP_MAX_RPM = 250


class MotorSerial:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port=None, baudrate=baud, timeout=.05, write_timeout=.5)
        self.ser.port = port
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.tx_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.ack_condition = threading.Condition()
        self.last_ack = None
        self.record_lock = threading.Lock()
        self.recorder = None
        self.last_recording = None
        self.latest_identification = None
        self.latest_learning = None
        self.latest = None
        self.received_at = 0
        self.error = None
        self.speed_profile = 'manual'
        self.stopped = threading.Event()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.keeper = threading.Thread(target=self._heartbeat, daemon=True)
        self.reader.start()
        self.keeper.start()

    def send(self, op, value=0):
        with self.tx_lock:
            for byte in f'{op}{value}\n'.encode('ascii'):
                self.ser.write(bytes([byte]))
                time.sleep(.005)

    def run_speed(self, rpm):
        if rpm == 0 or abs(rpm) > MAX_RPM:
            raise ValueError('RPM out of range')
        profile = getattr(self, 'speed_profile', 'manual')
        if profile != 'manual':
            kp, ki = speed_gains(rpm, profile)
            self.send_checked('S', 0)
            self.send_checked('P', kp)
            self.send_checked('Q', ki)
            print(f'{profile}: Kp={kp/1000:g}, Ki={ki/1000:g}')
        self.send_checked('R', rpm)

    def upload_feedforward(self, path):
        weights = load_weights(path)
        self.send_checked('S', 0)
        self.send_checked('G', 0)
        for index, value in enumerate(weights):
            self.send_checked('f', index)
            self.send_checked('w', value)
        print('Feedforward weights uploaded to RAM; NN remains off. Use nn ffshadow before nn ff.')

    def set_parameter(self, name, setting):
        # Y uses integer RPM; unlike gains and voltage it is not milli-units.
        opcode, scale, low, high = {
            'kp': ('P', 1000, 0, .2), 'ki': ('Q', 1000, 0, 2),
            'poskp': ('Z', 1000, 0, 20), 'posmax': ('Y', 1, 1, 1000),
            'voltage': ('V', 1000, .1, 1.5),
            'anglephase': ('a', 1000, -3.142, 3.142),
            'pref': ('r', 1, 0, 200),
            'posmove': ('t', 1, 0, 2000),
        }[name]
        if not math.isfinite(setting) or not low <= setting <= high:
            raise ValueError(f'Allowed range: {low:g} to {high:g}')
        if name == 'posmove' and (setting != round(setting) or 0 < setting < 100):
            raise ValueError('posmove: 0 for legacy, or 100..2000 milliseconds')
        self.send_checked(opcode, round(setting * scale))

    def set_nn_mode(self, name):
        modes = {'off': 0, 'shadow': 1, 'on': 2, 'blended': 2,
                 'ff': 3, 'ffshadow': 4, 'angleshadow': 5, 'angletrial': 6,
                 'dynamicshadow': 7, 'dynamic': 8, 'dynamiclinear': 9}
        mode = modes.get(name.lower())
        if mode is None:
            raise ValueError('NN mode must be off, shadow, on, ff, ffshadow, angleshadow, '
                             'angletrial, dynamicshadow, dynamic or dynamiclinear')
        self.send_checked('G', mode)

    def set_estimator(self, name):
        modes = {'legacy': 0, 'regression': 1, 'observer': 2}
        if name not in modes:
            raise ValueError('estimator must be legacy, regression or observer')
        self.send_checked('v', modes[name])

    def set_transition(self, name):
        policies = {'reset': 0, 'keep': 1}
        if name not in policies:
            raise ValueError('transition must be reset or keep (settings while stopped)')
        self.send_checked('j', policies[name])

    def _read(self):
        try:
            while not self.stopped.is_set():
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('T,'):
                    with self.state_lock:
                        self.latest = line
                        self.received_at = time.monotonic()
                        self.latest_identification = None
                    parts = line.split(',')
                    if len(parts) == 12 and parts[6] != '7':
                        with self.record_lock:
                            active = self.recorder is not None and self.recorder.started
                        if active:
                            self.finish_recording(f'mode={parts[6]}, fault={parts[10]}')
                elif line.startswith('L,'):
                    try:
                        row = parse_learning(line)
                    except ValueError:
                        continue
                    with self.state_lock:
                        self.latest_learning = row
                        self.latest_identification = None
                        self.latest = ('T,{sequence},{tick_ms},0,{speed_mrad_s},'
                                       '{sensor_ok},{mode},{calibrated},'
                                       '{target_speed_mrad_s},{uq_mv},{fault},0').format(**row)
                        self.received_at = time.monotonic()
                elif line.startswith('I,'):
                    try:
                        row = parse_trace(line)
                    except ValueError:
                        continue
                    with self.state_lock:
                        self.latest = telemetry_from_trace(row)
                        self.received_at = time.monotonic()
                        self.latest_identification = row
                    with self.record_lock:
                        if self.recorder is not None:
                            self.recorder.write(row)
                            if row['stage'] and row['point'] != self.recorder.point:
                                self.recorder.point = row['point']
                                print(f"\nIdentify point {row['point']}: "
                                      f"{row['frequency_mhz']/1000:.3f} Hz")
                elif line:
                    if line.startswith('ACK,'):
                        parts = line.split(',')
                        if len(parts) >= 3:
                            with self.ack_condition:
                                try:
                                    self.last_ack = (parts[1], int(parts[2]))
                                except ValueError:
                                    self.last_ack = None
                                self.ack_condition.notify_all()
                    print('\n' + line)
        except (OSError, serial.SerialException) as exc:
            self.error = str(exc)
            self.stopped.set()

    def send_checked(self, op, value=0):
        with self.ack_condition:
            self.last_ack = None
            self.send(op, value)
            if not self.ack_condition.wait_for(
                    lambda: self.last_ack is not None and self.last_ack[0] == op,
                    timeout=2):
                raise ValueError(f'No ACK for {op}; command acceptance unknown. Send stop.')
            if self.last_ack[1] != 0:
                raise ValueError(f'Firmware rejected {op}.')

    def identify(self, rpm, amplitude, first_hz, last_hz, points, path=None):
        settings = validate_settings(rpm, amplitude, first_hz, last_hz, points)
        with self.state_lock:
            fields = self.latest.split(',') if self.latest else []
            if (len(fields) != 12 or time.monotonic() - self.received_at > 1 or
                    fields[5] != '1' or fields[7] != '1'):
                raise ValueError('Identification requires fresh sensor_ok=1 and calibrated=1. '
                                 'Use calibrate, wait for completion, then state.')
        with self.record_lock:
            if self.recorder is not None:
                raise ValueError('An identification recording is active; stop it first.')
        self.send_checked('S', 0)
        for op, value in settings:
            self.send_checked(op, value)
        path = path or f"identification_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with self.record_lock:
            self.recorder = TraceRecorder(path)
            recorded_path = self.recorder.path
        try:
            self.send_checked('I', 1)
        except (ValueError, OSError, serial.SerialException):
            # A missing ACK can occur after the target has already started.
            try:
                self.send('S', 0)
            finally:
                self.finish_recording('start failed / unknown ACK')
            raise
        frequencies = [first_hz * (last_hz / first_hz) ** (i / (points - 1))
                       for i in range(points)]
        duration = 3 + sum(7 / hz for hz in frequencies)
        print(f'Identification started: baseline={rpm} RPM, excitation={amplitude:.3f} V, '
              f'{first_hz:g}..{last_hz:g} Hz, {points} points, about {duration:.0f}s.')
        print(f'CSV: {recorded_path}\nKeep CLI open. stop cancels; analyze processes the last CSV.')

    def finish_recording(self, reason):
        with self.record_lock:
            recorder, self.recorder = self.recorder, None
            if recorder is None:
                return
            recorder.close()
            self.last_recording = str(recorder.path)
        print(f'\nRecording closed ({reason}): {recorder.rows} rows, {recorder.path}')

    def sweep(self, start, end, seconds):
        if not (0 < abs(start) <= OPENLOOP_MAX_RPM and 0 < abs(end) <= OPENLOOP_MAX_RPM):
            raise ValueError(f'Sweep endpoints must be nonzero within +/-{OPENLOOP_MAX_RPM} RPM')
        if (start > 0) != (end > 0):
            raise ValueError('Use endpoints in the same direction; test reversal separately.')
        if not 1 <= seconds <= 600:
            raise ValueError('Sweep duration must be 1..600 seconds')
        for op, value in [('S', 0), ('B', start), ('E', end), ('D', seconds), ('F', 1)]:
            self.send_checked(op, value)
        print(f'Open-loop sweep: {start} -> {end} RPM in {seconds}s; '
              'MCU runs the ramp and stops at completion. Keep CLI open; stop cancels.')

    def _heartbeat(self):
        while not self.stopped.wait(.25):
            try:
                self.send('H')
            except (OSError, serial.SerialException) as exc:
                self.error = str(exc)
                self.stopped.set()

    def show_state(self):
        with self.state_lock:
            if self.latest is None:
                print('No telemetry received.' + (' ' + self.error if self.error else ''))
                return
            fields = self.latest.split(',')
            print(self.latest)
            print(f'Data age: {(time.monotonic()-self.received_at)*1000:.0f} ms')
            if len(fields) >= 10:
                sensor_ok = fields[5] == '1'
                speed = f'{int(fields[4])*60/6283.1853:.2f}' if sensor_ok else 'unavailable'
                print(f'RPM={speed} sensor_ok={int(sensor_ok)} mode={fields[6]} '
                      f'calibrated={fields[7]} Uq={int(fields[9])/1000:.3f} V')
                print(f'Target RPM={int(fields[8])*60/6283.1853:.2f}')
            if len(fields) >= 12:
                print(f'fault={fields[10]} calibration_travel={int(fields[11])/1000:.3f} rad')
            if self.latest_identification:
                row = self.latest_identification
                print(f"Identification: point={row['point']} stage={row['stage']} "
                      f"f={row['frequency_mhz']/1000:.3f}Hz "
                      f"excitation={row['excitation_mv']/1000:.3f}V clipped={row['clipped']}")
            if fields[0] == 'T' and getattr(self, 'latest_learning', None):
                row = self.latest_learning
                if str(row['sequence']) == fields[1]:
                    print(f"NN mode={row['nn_mode']} alpha={row['nn_alpha_milli']/1000:g} "
                          f"delta={row['nn_delta_uq_mv']/1000:.3f} V "
                          f"base={row['base_uq_mv']/1000:.3f} V")

    def close(self):
        self.stopped.set()
        self.keeper.join(timeout=1)
        try:
            self.send('S')
        except (OSError, serial.SerialException):
            pass
        self.reader.join(timeout=1)
        self.ser.close()
        self.finish_recording('CLI closed or serial lost')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', nargs='?', default='auto',
                        help='External USB-UART port or auto (default)')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    try:
        args.port = resolve_port(args.port)
    except ValueError as exc:
        parser.exit(1, str(exc) + '\n')
    try:
        motor = MotorSerial(args.port, args.baud)
    except (OSError, serial.SerialException) as exc:
        parser.exit(1, f'Cannot open {args.port}: {exc}\nClose/disconnect other serial programs first.\n')
    print(f'Connected directly to {args.port}. No HTTP connection.')
    print('state | voltage 1.0 | openloop 10 | sweep 10 200 20 | stop | calibrate | kp 0.0078 | ki 0.284 | profile manual/baseline/tuned/smooth/lowfast | poskp 4.0 | posmax 60 | posmove 500 (0=legacy) | nn off/shadow/on/ff/ffshadow/angleshadow/angletrial/dynamicshadow/dynamic/dynamiclinear | anglephase radians | ffload model.json | nnalpha 0.25 | learn on/off | run 30 | position 10 | pwmtest | quit')
    print('identify 60 0.05 0.2 5 8 [output.csv] | analyze [input.csv]')
    print('estimator legacy/regression/observer | timing | timingreset (settings while stopped)')
    print('transition reset/keep (stopped only; keep affects speed-to-speed changes)')
    print('pref milliseconds (0..200, stopped only; 0 disables P-reference smoothing)')
    print(f'Closed-loop run range: +/-{MAX_RPM} RPM; open-loop range: +/-{OPENLOOP_MAX_RPM} RPM.')
    print('ACK result 0 = accepted, -1 = rejected. pwmtest requires driver 12V disconnected.')
    try:
        while not motor.stopped.is_set():
            words = input('motor> ').split()
            if not words:
                continue
            op = words[0]
            try:
                if op in ('quit', 'exit'):
                    break
                if op == 'state':
                    motor.show_state()
                elif op == 'pwmtest' and len(words) == 1:
                    if input('Disconnect driver 12V; keep NXP USB powered. Type YES to output 20kHz/50%: ').strip().upper() == 'YES':
                        motor.send('W', 50)
                        print('Request sent; ACK,W,0 confirms acceptance, ACK,W,-1 means rejected. Use stop to end; keep CLI connected.')
                elif op in ('stop', 'calibrate') and len(words) == 1:
                    motor.send('S' if op == 'stop' else 'C')
                    print('Sent; use state to check the firmware result.')
                elif op in ('run', 'openloop') and len(words) == 2:
                    rpm = int(words[1])
                    limit = MAX_RPM if op == 'run' else OPENLOOP_MAX_RPM
                    if rpm == 0 or abs(rpm) > limit:
                        raise ValueError(f'RPM must be nonzero, between -{limit} and {limit}')
                    if op == 'run':
                        motor.run_speed(rpm)
                        print('Closed-loop run accepted; use state to observe tracking.')
                    else:
                        motor.send('O', rpm)
                        print('Sent; use state to check the firmware result.')
                elif op in ('position', 'pos') and len(words) == 2:
                    degrees = float(words[1])
                    if not math.isfinite(degrees) or abs(degrees) > 5729.0:
                        raise ValueError('Position must be finite and within +/-5729 degrees')
                    motor.send_checked('X', round(degrees * math.pi / 180.0 * 1000.0))
                    print(f'Position command accepted: {degrees:g} deg relative to calibration zero.')
                elif op == 'profile' and len(words) == 2:
                    if words[1] not in ('manual', 'baseline', 'tuned', 'smooth', 'lowfast'):
                        raise ValueError('profile must be manual, baseline, tuned, smooth or lowfast')
                    motor.speed_profile = words[1]
                    print(f'Next run command uses profile {words[1]}; applies to speed steps only.')
                elif op == 'estimator' and len(words) == 2:
                    motor.set_estimator(words[1])
                elif op == 'pref' and len(words) == 2:
                    motor.set_parameter('pref', float(words[1]))
                elif op == 'transition' and len(words) == 2:
                    motor.set_transition(words[1])
                    print(f'Estimator accepted: {words[1]}')
                elif op == 'timing' and len(words) == 1:
                    motor.send_checked('e', 0)
                elif op == 'timingreset' and len(words) == 1:
                    motor.send_checked('d', 0)
                elif op == 'ffload' and len(words) == 2:
                    motor.upload_feedforward(words[1])
                elif op == 'nn' and len(words) == 2:
                    motor.set_nn_mode(words[1])
                    print(f'NN mode accepted: {words[1].lower()}')
                elif op == 'nnalpha' and len(words) == 2:
                    alpha = float(words[1])
                    if not math.isfinite(alpha) or not 0 <= alpha <= 1:
                        raise ValueError('NN alpha must be between 0 and 1')
                    motor.send_checked('K', round(alpha * 1000))
                    print(f'NN blend alpha={alpha:g}')
                elif op == 'learn' and len(words) == 2:
                    value = {'off': 0, 'on': 1}.get(words[1].lower())
                    if value is None:
                        raise ValueError('learn must be on or off')
                    motor.send_checked('T', value)
                    print(f'Learning telemetry={words[1].lower()}')
                elif op == 'sweep' and len(words) == 4:
                    motor.sweep(*(int(word) for word in words[1:]))
                elif op == 'identify' and len(words) in (6, 7):
                    motor.identify(int(words[1]), float(words[2]), float(words[3]),
                                   float(words[4]), int(words[5]),
                                   words[6] if len(words) == 7 else None)
                elif op == 'analyze' and len(words) in (1, 2):
                    path = words[1] if len(words) == 2 else motor.last_recording
                    if not path:
                        raise ValueError('No completed recording; use analyze path.csv')
                    analyze_file(path)
                elif op in ('kp', 'ki', 'poskp', 'posmax', 'posmove', 'voltage', 'anglephase') and len(words) == 2:
                    motor.set_parameter(op, float(words[1]))
                    print('Setting accepted; firmware accepts settings only while stopped.')
                else:
                    print('Use state, voltage, openloop, sweep, identify, analyze, stop, calibrate, kp, ki, profile, estimator legacy/regression/observer, timing, timingreset, poskp, posmax, posmove, nn, ffload, nnalpha, learn, run, position, pwmtest, quit.')
            except (ValueError, KeyError, TypeError, OSError, ImportError, serial.SerialException) as exc:
                print(exc)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        motor.close()


if __name__ == '__main__':
    main()
