"""Compare baseline, shadow and blended control without reset or calibration."""
import argparse
import csv
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import serial

from collect_nn_dataset import read_state, send_checked, tx
from motor_port import resolve_port
from nn_dataset import LEARNING_FIELDS, parse_learning


def summarize(rows):
    import numpy as np
    results = []
    for trial in dict.fromkeys(r['trial'] for r in rows):
        samples = [r for r in rows if r['trial'] == trial]
        y = np.array([r['measured'] for r in samples])
        target = samples[0]['command']
        error = target - y
        tail = error[-max(1, len(error) // 4):]
        ticks = np.array([r['tick_ms'] for r in samples], dtype=np.int64)
        gaps = np.diff(ticks)
        results.append(dict(trial=trial, samples=len(samples),
                            unit=samples[0]['unit'], target=target,
                            rmse=float(np.sqrt(np.mean(error ** 2))),
                            tail_mae=float(np.mean(np.abs(tail))),
                            tail_mean=float(np.mean(y[-len(tail):])),
                            tail_std=float(np.std(y[-len(tail):])),
                            max_sample_gap_ms=int(gaps.max()) if gaps.size else 0,
                            peak_abs_nn_v=max(abs(r['nn_delta_uq_mv']) for r in samples)/1000,
                            clipped=sum(r['clipped'] for r in samples)))
    return results


def plot(rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    for trial in dict.fromkeys(r['trial'] for r in rows):
        samples = [r for r in rows if r['trial'] == trial]
        i = 0 if samples[0]['unit'] == 'RPM' else 1
        t = [r['elapsed_s'] for r in samples]
        axes[i, 0].plot(t, [r['measured'] for r in samples], label=trial)
        axes[i, 1].plot(t, [r['uq_mv']/1000 for r in samples], label=trial)
        axes[i, 0].axhline(samples[0]['command'], color='gray', linestyle='--', alpha=.3)
    axes[0, 0].set_ylabel('Speed [RPM]')
    axes[1, 0].set_ylabel('Relative position [deg]')
    for ax in axes.flat:
        ax.set_xlabel('Time [s]'); ax.grid(alpha=.25)
        if ax.lines:
            ax.legend(fontsize=8)
    axes[0, 1].set_ylabel('Uq [V]'); axes[1, 1].set_ylabel('Uq [V]')
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def run(port_name, output, seconds=4.0, alpha=.1, reverse_order=False, kind='all'):
    output = Path(output).resolve()
    if any(output.with_suffix(s).exists() for s in ('.csv', '.json', '.png')):
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    port = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=.5)
    port.port = resolve_port(port_name); port.dtr = False; port.rts = False
    lock = threading.Lock(); done = threading.Event()
    def heartbeat():
        while not done.wait(.25):
            try:
                tx(port, 'H', 0, lock)
            except (OSError, serial.SerialException):
                done.set()
    def command(op, value=0):
        send_checked(port, op, value, lock)
    def record(writer, trial, duration, command_value, origin=0):
        started = time.monotonic(); count = 0; last = None; first_tick = None
        expected = 2 if trial.startswith('speed') else 8
        while time.monotonic() - started < duration:
            if done.is_set():
                raise RuntimeError('Serial heartbeat failed')
            line = port.readline().decode('ascii', 'replace').strip()
            if not line.startswith('L,'):
                continue
            try:
                row = parse_learning(line)
            except ValueError:
                continue
            if row['fault'] or not row['sensor_ok'] or not row['calibrated']:
                raise RuntimeError(f'Trial {trial}: {line}')
            if row['mode'] != expected:
                continue
            if first_tick is None:
                first_tick = row['tick_ms']
            if expected == 2:
                measured = row['speed_mrad_s'] * 60 / (2000 * math.pi)
                unit = 'RPM'
            else:
                measured = (row['position_mrad'] - origin) * 180 / (1000 * math.pi)
                unit = 'deg'
            row.update(trial=trial, elapsed_s=((row['tick_ms']-first_tick) & 0xFFFFFFFF)/1000,
                       command=command_value, measured=measured, unit=unit)
            writer.writerow(row); rows.append(row); count += 1; last = row
        if count < 20:
            raise RuntimeError(f'Trial {trial}: only {count} frames')
        return last
    port.open(); worker = threading.Thread(target=heartbeat, daemon=True); worker.start()
    try:
        port.reset_input_buffer(); tx(port, 'S', 0, lock)
        command('S'); command('T', 0); command('G', 0)
        state = read_state(port)
        print('Using existing calibration:', ','.join(state), flush=True)
        with output.open('x', newline='', encoding='utf-8') as stream:
            fields = ['trial', 'elapsed_s', 'command', 'measured', 'unit'] + LEARNING_FIELDS
            writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
            modes = [('baseline', 0), ('shadow', 1), ('blended', 2)]
            if reverse_order:
                modes.reverse()
            kinds = ('speed', 'position') if kind == 'all' else (kind,)
            for kind in kinds:
                for name, mode in modes:
                    if kind == 'position' and not rows:
                        raise RuntimeError('Position comparison needs an unwrapped L snapshot; run --kind all')
                    if kind == 'position' and abs(rows[-1]['position_mrad']) > 99000:
                        print('Position skipped: unwrapped position exceeds X range; calibration retained.', flush=True)
                        break
                    command('S'); time.sleep(.5)
                    command('G', mode); command('K', round(alpha * 1000)); command('T', 1)
                    trial = f'{kind}_{name}'
                    print('Running', trial, flush=True)
                    if kind == 'speed':
                        command('R', 60)
                        record(writer, trial, seconds, 60)
                    else:
                        command('Y', 60); command('Z', 8000)
                        # Settle at the preceding measured angle, then capture
                        # the actual unwrapped origin for an equal 10-degree step.
                        command('X', rows[-1]['position_mrad'])
                        deadline = time.monotonic() + 2; origin = None
                        while time.monotonic() < deadline:
                            line = port.readline().decode('ascii', 'replace').strip()
                            if line.startswith('L,'):
                                snapshot = parse_learning(line)
                                if snapshot['fault'] or not snapshot['sensor_ok']:
                                    raise RuntimeError(line)
                                origin = snapshot['position_mrad']
                        if origin is None:
                            raise RuntimeError('No position origin telemetry')
                        command('S')
                        target = origin + round(math.radians(10)*1000)
                        command('X', target)
                        record(writer, trial, seconds, (target-origin)*180/(1000*math.pi), origin)
                    command('S'); stream.flush()
    finally:
        for op in ('S', 'T', 'G'):
            try:
                command(op, 0)
            except Exception:
                pass
        done.set(); worker.join(1); port.close()
        if rows:
            results = summarize(rows)
            output.with_suffix('.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
            plot(rows, output.with_suffix('.png'))
            print(json.dumps(results, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='auto')
    parser.add_argument('--output', default=f'data/nn_compare_{datetime.now():%Y%m%d_%H%M%S}.csv')
    parser.add_argument('--seconds', type=float, default=4)
    parser.add_argument('--alpha', type=float, default=.1)
    parser.add_argument('--reverse-order', action='store_true')
    parser.add_argument('--kind', choices=('all', 'speed'), default='all')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 30:
        parser.error('--seconds must be 1..30')
    if not math.isfinite(args.alpha) or not 0 <= args.alpha <= 1:
        parser.error('--alpha must be 0..1')
    run(args.port, args.output, args.seconds, args.alpha, args.reverse_order, args.kind)
