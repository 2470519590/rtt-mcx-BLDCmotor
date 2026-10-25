"""Separate the average (DC) term from the angle-periodic (AC) term.

The earlier angle candidate modelled both at once: the per-speed DC offset was
subtracted during training, yet the network kept a speed/direction input and
therefore re-learned part of the DC. That makes a "DC vs AC" split impossible
on a single 0.62-scaled model. This script fits them as two explicit
quantities and answers, on frozen held-out repeats:

* how much of the voltage residual is a constant per speed (DC),
* how much is periodic in mechanical angle (AC),
* whether the DC term is what actually matters, which would explain why the
  angle network showed no closed-loop benefit.

No motor, no MCU: pure analysis of the NN-off baseline recorded with the
current calibration.
"""
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from angle_phase_model import HARMONICS  # noqa: E402

STEADY_MS = 2000
MODEL_K = 44.18  # (rad/s)/V, the firmware's speed model


def load(path, config):
    groups = {}
    with open(path, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            if row['config'] != config:
                continue
            if int(row['fault']) or int(row['clipped']) or not int(row['sensor_ok']) \
                    or not int(row['calibrated']) or int(row['mode']) != 2:
                raise ValueError('baseline must be healthy, unclipped, NN-off speed data')
            groups.setdefault((int(row['target_rpm']), int(row['repeat'])), []).append(row)
    data = []
    for (target, repeat), samples in sorted(groups.items()):
        ticks = np.array([int(r['tick_ms']) for r in samples])
        steady = [r for r in samples if int(r['tick_ms']) - ticks[0] >= STEADY_MS]
        position = np.array([int(r['position_mrad']) for r in steady]) / 1000
        voltage = np.array([int(r['uq_mv']) for r in steady]) / 1000
        data.append(dict(target=target, repeat=repeat, position=position, voltage=voltage,
                         label=voltage - target * 2 * np.pi / 60 / MODEL_K))
    return data


def design(position, harmonics):
    return np.column_stack([f(h * position) for h in harmonics for f in (np.sin, np.cos)])


def rmse(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return float(np.sqrt(np.mean((a - b) ** 2)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--config', default='pi_kp150_ki100')
    parser.add_argument('--harmonics', nargs='+', type=int, default=[1, 2, 7, 14, 21])
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    data = load(args.input, args.config)
    repeats = sorted({d['repeat'] for d in data})
    speeds = sorted({d['target'] for d in data})
    if len(repeats) < 3:
        raise ValueError('need three whole repeats: 0 train, 1 validation, 2 test')
    train = [d for d in data if d['repeat'] == 0]
    report = dict(schema='angle_dc_ac_split_v1',
                  source=str(Path(args.input).resolve()),
                  source_sha256=hashlib.sha256(Path(args.input).read_bytes()).hexdigest(),
                  config=args.config, harmonics=args.harmonics, speeds=speeds,
                  split='repeat0 fit; repeat1 selects harmonic count; repeat2 and DC origin frozen',
                  target='Recorded Uq minus target*2pi/60/44.18',
                  limitation='Closed-loop voltage residual: includes control action, estimator error and '
                             'operating drift. This is not a measured disturbance torque.',
                  points=[])
    dc = {s: float(np.mean(np.concatenate([d['label'] for d in train if d['target'] == s])))
          for s in speeds}
    # Harmonic count chosen on repeat1 only, then frozen for repeat2.
    validation = [d for d in data if d['repeat'] == 1]
    selected = None
    for count in range(1, len(args.harmonics) + 1):
        used = args.harmonics[:count]
        weights = {}
        for s in speeds:
            p = np.concatenate([d['position'] for d in train if d['target'] == s])
            y = np.concatenate([d['label'] for d in train if d['target'] == s]) - dc[s]
            a = design(p, used)
            weights[s] = np.linalg.lstsq(a.T @ a + .01 * np.eye(a.shape[1]), a.T @ y, rcond=None)[0]
        error = rmse(np.concatenate([d['label'] - dc[d['target']] for d in validation]),
                     np.concatenate([design(d['position'], used) @ weights[d['target']]
                                     for d in validation]))
        report.setdefault('validation_selection', []).append(dict(harmonics=used, rmse_v=error))
        if selected is None or error < selected[0]:
            selected = (error, used, weights)
    _, used, weights = selected
    report['selected_harmonics'] = used
    report['dc_by_speed'] = {str(k): v for k, v in dc.items()}
    report['harmonic_weights_by_speed'] = {str(k): [float(x) for x in v] for k, v in weights.items()}
    for repeat in repeats:
        entry = dict(repeat=repeat, speeds={})
        for s in speeds:
            group = [d for d in data if d['repeat'] == repeat and d['target'] == s]
            if not group:
                continue
            y = np.concatenate([d['label'] for d in group])
            ac = np.concatenate([design(d['position'], used) @ weights[d['target']] for d in group])
            point = dict(rows=int(len(y)),
                         zero_dc=rmse(y, 0.0),
                         dc_only=rmse(y, dc[s]),
                         ac_only=rmse(y, ac),
                         dc_plus_ac=rmse(y, dc[s] + ac))
            entry['speeds'][str(s)] = point
        report['points'].append(entry)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    for entry in report['points']:
        print(f"repeat {entry['repeat']}")
        for s, point in entry['speeds'].items():
            print(f"  {int(s):+4d} zero={point['zero_dc']:.4f} dc={point['dc_only']:.4f} "
                  f"ac={point['ac_only']:.4f} dc+ac={point['dc_plus_ac']:.4f}")
    print('harmonics', used, 'dc', {k: round(v, 4) for k, v in dc.items()})
    print(output)


if __name__ == '__main__':
    main()
