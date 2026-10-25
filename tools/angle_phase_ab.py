"""Board A/B: traditional PI vs bounded angle compensation, with controls.

Same PI (P150/Q100), same calibration, NN-off as the reference configuration.
The comparisons isolate the angle model's incremental value:

  baseline            NN off
  global_phase000     one phase for every speed (fit on repeat0; tiny)
  per_speed_interp    speed-dependent phase table (fit on repeat0)
  per_speed_zero      same table committed with all phases = 0; must match
                      baseline within noise, or the upload path itself moved
                      the machine

Each configuration runs every target once per repeat in random order, so
rotation drift cannot favour one configuration.
"""
import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import serial

from angle_phase_model import PhaseTable, load_model, upload_sequence
from collect_nn_dataset import parse_learning, read_state, send_checked, tx
from motor_port import resolve_port

MODE_ANGLE_TRIAL = 6
MODE_OFF = 0


def load_tables(report_path):
    report = json.loads(Path(report_path).read_text(encoding='utf-8'))
    tables = {}
    for name, entry in report['phase_tables'].items():
        tables[name] = PhaseTable([(p['rpm'], p['phase_rad']) for p in entry['points']],
                                  interp=entry['interp'])
    return tables


class Rig:
    def __init__(self, port='auto'):
        self.port = serial.Serial(port=None, baudrate=115200, timeout=.03, write_timeout=.5)
        self.port.port = resolve_port(port)
        self.port.dtr = False
        self.port.rts = False
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.worker = None

    def __enter__(self):
        self.port.open()
        self.worker = threading.Thread(target=self._heartbeat, daemon=True)
        self.worker.start()
        self.command('S', 0)
        self.command('T', 1)
        return self

    def __exit__(self, *exc):
        for op, value in (('G', MODE_OFF), ('T', 0), ('S', 0)):
            try:
                self.command(op, value)
            except Exception:
                tx(self.port, 'S', 0, self.lock)
        self.done.set()
        if self.worker:
            self.worker.join(1)
        self.port.close()

    def probe_applied(self, target, seconds=1.0):
        """Short run reporting the applied offset per mode, for verification."""
        self.command('S', 0)
        time.sleep(.4)
        self.command('R', target)
        deltas = []
        applied = []
        stop_at = time.monotonic() + seconds
        while time.monotonic() < stop_at:
            line = self.port.readline().decode('ascii', 'replace').strip()
            if not line.startswith('L,'):
                continue
            row = parse_learning(line)
            deltas.append(row['nn_delta_uq_mv'])
            applied.append(row['uq_mv'] - row['base_uq_mv'])
        self.command('S', 0)
        if not deltas:
            return dict(frames=0)
        return dict(frames=len(deltas), delta_median_mv=int(np.median(deltas)),
                    applied_median_mv=int(np.median(applied)),
                    applied_min_mv=int(min(applied)), applied_max_mv=int(max(applied)))

    def read_coefficient(self, neuron, slot):
        """Read one stored coefficient (1e-4 units) for upload verification."""
        self.port.reset_input_buffer()
        tx(self.port, 'y', neuron * 100 + slot, self.lock)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            line = self.port.readline().decode('ascii', 'replace').strip()
            if line.startswith('ACK,y,'):
                return int(line.split(',')[2])
        raise RuntimeError('no answer for the y coefficient query')

    def _heartbeat(self):
        while not self.done.wait(.2):
            try:
                tx(self.port, 'H', 0, self.lock)
            except Exception:
                self.done.set()

    def command(self, op, value=0):
        send_checked(self.port, op, value, self.lock)

    def health(self):
        return read_state(self.port, require_calibrated=True)

    def upload(self, model, table):
        for op, value in upload_sequence(model, table):
            self.command(op, value)

    def step(self, target, seconds):
        self.command('S', 0)
        time.sleep(.5)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            state = read_state(self.port, timeout=.5)
            if state[6] == '0' and abs(int(state[4])) < 100:
                break
        self.command('R', target)
        rows = []
        stop_at = time.monotonic() + seconds
        while time.monotonic() < stop_at:
            line = self.port.readline().decode('ascii', 'replace').strip()
            if not line.startswith('L,'):
                continue
            row = parse_learning(line)
            if row['fault'] or not row['sensor_ok'] or not row['calibrated']:
                raise RuntimeError('unhealthy recording: ' + line)
            rows.append(row)
        self.command('S', 0)
        return rows


def metrics(rows, target, settled=None):
    ticks = np.array([r['tick_ms'] for r in rows], dtype=np.int64)
    speed = np.array([r['speed_mrad_s'] for r in rows]) * 60 / (2000 * math.pi)
    position = np.array([r['position_mrad'] for r in rows]) / 1000
    error = speed - target
    tail = ticks >= ticks[-1] - 800
    settle = settled if settled is not None else 250.0
    body = ticks >= ticks[0] + settle
    # Same integer-tick displacement window for every configuration, so a
    # smoother feedback signal cannot win on its own.
    window = []
    for i in np.flatnonzero(ticks >= ticks[-1] - 800):
        j = np.searchsorted(ticks, ticks[i] - 100, side='right') - 1
        if 0 <= j < i and ticks[i] - ticks[j] <= 115:
            window.append((position[i] - position[j]) / ((ticks[i] - ticks[j]) / 1000)
                          * 60 / (2 * math.pi))
    return dict(rmse_rpm=float(np.sqrt(np.mean(error ** 2))),
                iae=float(np.sum((np.abs(error[1:]) + np.abs(error[:-1])) * .5 * np.diff(ticks) / 1000)),
                # Steady-state metrics exclude the commanded start transient,
                # otherwise a larger initial deflection hides a steadier loop.
                ss_rmse_rpm=float(np.sqrt(np.mean(error[body] ** 2))),
                ss_iae=float(np.sum((np.abs(error[1:]) + np.abs(error[:-1]))[body[1:]]
                                    * .5 * np.diff(ticks)[body[1:]] / 1000)),
                ss_tail_std=float(np.std(speed[tail])),
                tail_mae=float(np.mean(np.abs(error[tail]))),
                tail_std=float(np.std(speed[tail])),
                displacement_tail_rmse=float(np.sqrt(np.mean((np.array(window) - target) ** 2))),
                clipped=int(sum(r['clipped'] for r in rows)),
                max_gap_ms=float(np.max(np.diff(ticks))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--phase-report', required=True)
    parser.add_argument('--configs', nargs='+',
                        default=['baseline', 'zero_phase_control', 'global',
                                 'per_speed_joint_interp'])
    parser.add_argument('--targets', nargs='+', type=int, default=[15, -15, 30, -30])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--seconds', type=float, default=6)
    parser.add_argument('--alpha', type=int, default=250)
    parser.add_argument('--seed', type=int, default=601)
    parser.add_argument('--shuffle', action='store_true',
                        help='randomize configuration order (confounds drift; avoided by default)')
    parser.add_argument('--port', default='auto')
    args = parser.parse_args()
    output = Path(args.output)
    if any(output.with_suffix(s).exists() for s in ('.csv', '.json')):
        raise FileExistsError(output)
    if any(abs(t) < 15 or abs(t) > 30 for t in args.targets):
        raise SystemExit('G6 is limited to 15..30 RPM magnitude')
    model = load_model(args.model)
    tables = load_tables(args.phase_report)
    zero = PhaseTable([(rpm, 0.0) for rpm, _ in tables['per_speed_joint_interp'].points], interp=0)
    # Rotation between configurations (not repeats) would confound a long run;
    # keep the order fixed so both halves see the same machine state.
    rng = np.random.default_rng(args.seed)
    results = []
    with Rig(args.port) as rig:
        print('health', ','.join(rig.health()), flush=True)
        with output.open('x', newline='', encoding='utf-8') as stream:
            writer = None
            for repeat in range(args.repeats):
                ordered = [str(name) for name in args.configs]
                if args.shuffle:
                    ordered = [str(name) for name in rng.permutation(ordered)]
                for name in ordered:
                    rig.command('G', MODE_OFF)
                    if name == 'baseline':
                        table = None
                        mode, alpha = MODE_OFF, 0
                    else:
                        table = zero if name == 'zero_phase_control' else tables[name]
                        mode, alpha = MODE_ANGLE_TRIAL, args.alpha
                    rig.command('P', 150)
                    rig.command('Q', 100)
                    if table is not None:
                        rig.upload(model, table)
                    rig.command('K', alpha)
                    rig.command('G', mode)
                    for target in args.targets:
                        rows = rig.step(target, args.seconds)
                        if len(rows) < args.seconds * 60:
                            raise RuntimeError('insufficient telemetry')
                        if writer is None:
                            writer = csv.DictWriter(stream, fieldnames=['config', 'repeat',
                                                                        'target_rpm'] + list(rows[0]))
                            writer.writeheader()
                        for row in rows:
                            writer.writerow(dict(config=name, repeat=repeat,
                                                 target_rpm=target, **row))
                        result = dict(config=name, repeat=repeat, target_rpm=target,
                                      phase=table.as_dict() if table else None,
                                      **metrics(rows, target))
                        results.append(result)
                        stream.flush()
                        output.with_suffix('.json').write_text(json.dumps(results, indent=2),
                                                              encoding='utf-8')
                        print(json.dumps({k: v for k, v in result.items() if k != 'phase'}),
                              flush=True)
                    rig.command('G', MODE_OFF)
        print('final', ','.join(rig.health()), flush=True)


if __name__ == '__main__':
    main()
