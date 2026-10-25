"""Upload a RAM angle model + speed-dependent phase to the running firmware.

Stop-only. Never flashes, never calibrates, never starts motion; the caller
decides what to run afterwards. Verified by the read-only x0 query and by an
optional G5 shadow collection compared against the Python reference.
"""
import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
import serial

from angle_phase_model import PhaseTable, load_model, predict, upload_sequence
from collect_nn_dataset import read_state, send_checked, tx
from motor_port import resolve_port


class AngleUploader:
    def __init__(self, port_name='auto', baud=115200):
        self.port = serial.Serial(port=None, baudrate=baud, timeout=.05, write_timeout=.5)
        self.port.port = resolve_port(port_name)
        self.port.dtr = False
        self.port.rts = False
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.worker = None

    def __enter__(self):
        self.port.open()
        self.worker = threading.Thread(target=self._heartbeat, daemon=True)
        self.worker.start()
        send_checked(self.port, 'S', 0, self.lock)
        return self

    def __exit__(self, *exc):
        try:
            send_checked(self.port, 'G', 0, self.lock)
            send_checked(self.port, 'S', 0, self.lock)
        except Exception:
            tx(self.port, 'S', 0, self.lock)
        finally:
            self.done.set()
            if self.worker:
                self.worker.join(1)
            self.port.close()

    def _heartbeat(self):
        while not self.done.wait(.2):
            try:
                tx(self.port, 'H', 0, self.lock)
            except Exception:
                self.done.set()

    def command(self, op, value):
        send_checked(self.port, op, value, self.lock)

    def query(self):
        self.port.reset_input_buffer()
        tx(self.port, 'x', 0, self.lock)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            line = self.port.readline().decode('ascii', 'replace').strip()
            if line.startswith('X,'):
                parts = line.split(',')
                if len(parts) != 8:
                    # The response also embeds the accepted-result slot.
                    raise RuntimeError('bad X frame: ' + line)
                return dict(result=int(parts[1]), point=int(parts[2]), count=int(parts[3]),
                            row_valid=int(parts[4]), points=int(parts[5]), interp=int(parts[6]),
                            g_stage=int(parts[7]))
        raise RuntimeError('no X response; firmware may predate the angle upload protocol')

    def coefficient(self, neuron, slot):
        """Read one stored coefficient back from the MCU, in 1e-4 units."""
        self.port.reset_input_buffer()
        tx(self.port, 'y', neuron * 100 + slot, self.lock)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            line = self.port.readline().decode('ascii', 'replace').strip()
            if line.startswith('ACK,y,'):
                return int(line.split(',')[2])
        raise RuntimeError('no answer for the y coefficient query')

    def read_row(self, neuron):
        row = [self.coefficient(neuron, slot) / 10000.0 for slot in range(15)]
        return (row[0], row[1:13], row[13])

    def health(self):
        return read_state(self.port, require_calibrated=True)

    def build_stamp(self):
        tx(self.port, 'z', 0, self.lock)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            line = self.port.readline().decode('ascii', 'replace').strip()
            if line.startswith('ACK,z,'):
                return int(line.split(',')[2])
        raise RuntimeError('no z build stamp; running firmware is unconfirmed')


REQUIRED_BUILD = 3003


def ensure_build(uploader, required=REQUIRED_BUILD):
    """Refuse uploads unless the running image uses the disjoint field layout."""
    stamp = uploader.build_stamp()
    if stamp != required:
        raise RuntimeError(f'firmware build {stamp}, expected {required}. '
                           'The signed weight field layout is incompatible; do not upload.')
    return stamp


def upload(uploader, model, table, verify=True):
    if table is None:
        raise ValueError('phase table required')
    commands = upload_sequence(model, table)
    for op, value in commands:
        uploader.command(op, value)
    result = uploader.query()
    report = dict(commands=len(commands), model_sha256=model['sha256'],
                  phase_table=table.as_dict(), firmware=result)
    if verify:
        expected = len(table.points)
        if (result['result'] != 0 or result['points'] != expected
                or result['interp'] != table.interp):
            raise RuntimeError(f'firmware state does not match upload: {result} '
                               f'(expected points={expected} interp={table.interp})')
        # Reading every coefficient back catches silent truncation (an earlier
        # i limiter cut a 0.0949 weight to 0.005) and staging mix-ups that the
        # status word cannot see. Costs ~120 short queries, no motor motion.
        report['coefficient_check'] = verify_upload(model, table, uploader.read_row)
    return report


def collect_shadow(uploader, speeds, alpha=1000, seconds_per_point=4.0):
    """G5 only: proves the prediction runs and never actuates."""
    from nn_dataset import parse_learning
    uploader.command('T', 1)          # learning telemetry
    uploader.command('G', 5)          # angle shadow: apply() cannot move Uq
    uploader.command('K', alpha)      # alpha deliberately nonzero
    recorded = []
    try:
        for rpm in speeds:
            uploader.command('S', 0)
            time.sleep(.4)
            uploader.command('R', rpm)
            deadline = time.monotonic() + seconds_per_point
            while time.monotonic() < deadline:
                line = uploader.port.readline().decode('ascii', 'replace').strip()
                if not line.startswith('L,'):
                    continue
                row = parse_learning(line)
                if row['nn_mode'] != 5:
                    continue
                if row['fault'] or not row['sensor_ok'] or not row['calibrated']:
                    raise RuntimeError('shadow recording unhealthy: ' + line)
                recorded.append(dict(row, cmd_rpm=rpm))
            uploader.command('S', 0)
    finally:
        uploader.command('G', 0)
        uploader.command('S', 0)
        uploader.command('T', 0)
    return recorded


def shadow_report(rows, model, table):
    """Compare the MCU's shadow output with the float32 reference.

    The reference is evaluated at the recorded command speed, not at the
    telemetry target, because start-up frames still carry the previous target.
    Frames in the first 300 ms of each step (accelerating, target not yet
    applied) are excluded and reported separately, not silently dropped.
    """
    if not rows:
        raise ValueError('no shadow frames')
    settled = [r for r in rows if r.get('settled', True)]
    if not settled:
        raise ValueError('no settled shadow frames')
    rpm = np.array([r['target_speed_mrad_s'] for r in settled], dtype=float) * 60 / (2 * np.pi * 1000)
    phase = table.phase(rpm)
    reference = predict([r['position_mrad'] / 1000 for r in settled], rpm, model, phase)
    observed = np.array([r['nn_delta_uq_mv'] / 1000 for r in settled])
    applied = [r['uq_mv'] - r['base_uq_mv'] for r in settled if not r['clipped']]
    difference = np.abs(reference - observed)
    # The telemetry carries signed integer milli-volts, so +/-1 mV is the
    # transport floor. A single frame can round twice (float32 vs libm math),
    # so the out-of-band count is reported as well as the maximum.
    tolerance = 2.5e-3
    out_of_band = int(np.count_nonzero(difference > tolerance))
    return dict(frames=len(settled), excluded_ramp_frames=len(rows) - len(settled),
                max_prediction_difference_v=float(difference.max()),
                p95_prediction_difference_v=float(np.percentile(difference, 95)),
                tolerance_v=tolerance,
                out_of_band_frames=out_of_band,
                max_abs_applied_mv=int(max(map(abs, applied))) if applied else 0,
                unclipped_frames=len(applied),
                max_abs_prediction_v=float(np.max(np.abs(observed))),
                in_range=bool(np.max(np.abs(observed)) <= .080001),
                passed=bool(out_of_band == 0 and
                            (not applied or max(map(abs, applied)) == 0)),
                phase_by_speed={round(float(s), 1): round(float(p), 4)
                                for s, p in zip(sorted(set(rpm)), table.phase(sorted(set(rpm))))})


def _mark_settled(rows, ramp_seconds=0.3):
    """Tag each frame as past the start-up transient of its own step."""
    by_step = {}
    for row in rows:
        by_step.setdefault(row['cmd_rpm'], []).append(row)
    for step in by_step.values():
        first = step[0]['tick_ms']
        for row in step:
            row['settled'] = (row['tick_ms'] - first) >= ramp_seconds * 1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--phase-report', help='fit_angle_phase.py JSON to take a table from')
    parser.add_argument('--table', default='per_speed_joint_interp')
    parser.add_argument('--phase', type=float, help='single constant phase (rad) instead')
    parser.add_argument('--interp', type=int, default=None, help='override 0/1')
    parser.add_argument('--shadow', nargs='*', type=int, help='speeds for a G5 shadow check')
    parser.add_argument('--port', default='auto')
    parser.add_argument('--report')
    args = parser.parse_args()
    model = load_model(args.model)
    if args.phase_report:
        study = json.loads(Path(args.phase_report).read_text(encoding='utf-8'))
        entry = study['phase_tables'][args.table]
        table = PhaseTable([(p['rpm'], p['phase_rad']) for p in entry['points']],
                           interp=entry['interp'])
    elif args.phase is not None:
        table = PhaseTable.constant(args.phase)
    else:
        raise SystemExit('give --phase-report or --phase')
    if args.interp is not None:
        table = PhaseTable(table.points, interp=args.interp)
    with AngleUploader(args.port) as uploader:
        print('HEALTH', ','.join(uploader.health()), flush=True)
        report = upload(uploader, model, table)
        print('UPLOAD', json.dumps(report['firmware']), flush=True)
        if args.shadow:
            rows = collect_shadow(uploader, args.shadow)
            _mark_settled(rows)
            report['shadow'] = shadow_report(rows, model, table)
            print('SHADOW', json.dumps(report['shadow']), flush=True)
        print('FINAL', ','.join(uploader.health()), flush=True)
    if args.report:
        Path(args.report).write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(args.report)


if __name__ == '__main__':
    main()
