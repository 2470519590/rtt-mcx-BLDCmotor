"""Learn recorded angle-correlated voltage, not an identified inverse plant.

Offline only. Existing ff1 firmware is incompatible with these angle features.
Whole repeats are train/validation/test; no shuffled sample leakage.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from nn_model import train_mlp, standardize

HARMONICS = (1, 2, 7, 14, 21)


def features(position_rad, rpm):
    angle = np.asarray(position_rad, dtype=float)
    speed = np.broadcast_to(np.asarray(rpm, dtype=float), angle.shape)
    return np.column_stack([speed / 40, np.sign(speed)] +
                           [f(h * angle) for h in HARMONICS for f in (np.sin, np.cos)])


def prepare(rows, config, require_three=True):
    groups = {}
    for row in rows:
        if row['config'] == config:
            groups.setdefault((int(row['repeat']), int(row['target_rpm'])), []).append(row)
    if not groups:
        raise ValueError('no selected configuration')
    records = []
    for (repeat, target), samples in groups.items():
        ticks = np.array([int(r['tick_ms']) for r in samples])
        if len(ticks) < 300 or np.any(np.diff(ticks) <= 0) or np.max(np.diff(ticks)) > 30:
            raise ValueError('insufficient or discontinuous trial')
        for row in samples:
            if (int(row['fault']) or int(row['clipped']) or not int(row['sensor_ok'])
                    or not int(row['calibrated']) or int(row['mode']) != 2
                    or int(row['nn_mode']) != 0):
                raise ValueError('training requires healthy unclipped NN-off speed trials')
        steady = [r for r in samples if int(r['tick_ms']) - ticks[0] >= 2000]
        if len(steady) < 100:
            raise ValueError('need steady data after two-second startup exclusion')
        position = np.array([int(r['position_mrad']) for r in steady]) / 1000
        if abs(position[-1] - position[0]) < 2 * np.pi:
            raise ValueError('steady trial must span at least one mechanical revolution')
        records.extend((repeat, target, p, int(r['uq_mv']) / 1000 -
                        target * 2 * np.pi / 60 / 44.18)
                       for p, r in zip(position, steady))
    data = np.asarray(records)
    if not require_three:
        return data, []
    masks = [data[:, 0] == i for i in range(3)]
    if set(data[:, 0]) != {0, 1, 2}:
        raise ValueError('require exactly three whole repeats: 0 train, 1 validation, 2 test')
    targets = [set(data[m, 1]) for m in masks]
    if len(targets[0]) < 2 or any(t != targets[0] for t in targets):
        raise ValueError('each repeat needs the same signed speed targets')
    return data, masks


def metrics(y, predicted):
    return dict(rows=len(y), rmse_v=float(np.sqrt(np.mean((y-predicted)**2))))


def harmonic_design(x, count):
    # Match NN access to speed/direction, not an unfair shared-angle comparator.
    angular = np.column_stack([np.ones(len(x)), x[:, 2:2+2*count]])
    return np.column_stack([angular, angular*x[:, :1], angular*x[:, 1:2]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--config', default='pi_kp150_ki100')
    parser.add_argument('--output', required=True)
    parser.add_argument('--phase-report', help='fit_angle_phase angle_phase_study_v1 JSON')
    parser.add_argument('--phase-table', default='per_speed_joint_interp')
    args = parser.parse_args()
    output = Path(args.output)
    if output.suffix != '.npz' or any(output.with_suffix(s).exists() for s in ('.npz', '.json', '.png')):
        raise ValueError('use a new .npz output stem')
    phase_table = None
    phase_meta = {}
    if args.phase_report:
        import json as _json
        from angle_phase_model import PhaseTable
        study = _json.loads(Path(args.phase_report).read_text(encoding='utf-8'))
        entry = study['phase_tables'][args.phase_table]
        phase_table = PhaseTable([(p['rpm'], p['phase_rad']) for p in entry['points']],
                                 interp=entry['interp'])
        phase_meta = dict(phase_report=str(Path(args.phase_report).resolve()),
                          phase_table=args.phase_table,
                          phase_report_sha256=hashlib.sha256(
                              Path(args.phase_report).read_bytes()).hexdigest())
    source = Path(args.input)
    with source.open(newline='', encoding='utf-8') as stream:
        data, masks = prepare(list(csv.DictReader(stream)), args.config)
    train, validation, test = masks
    if phase_table is None:
        x = features(data[:, 2], data[:, 1])
    else:
        rpm = data[:, 1]
        x = features(data[:, 2], rpm, phase_table.phase(rpm))
    y = data[:, 3]
    _, mean, std = standardize(x[train])
    normalized = (x - mean) / std
    offsets = {rpm: float(y[train & (data[:, 1] == rpm)].mean())
               for rpm in sorted(set(data[:, 1]))}
    constant = np.array([offsets[rpm] for rpm in data[:, 1]])    # Direction/speed DC baseline is estimated on train only, then frozen.
    residual = y - constant
    scale = max(float(np.std(residual[train])), .001)
    candidates = []
    for epochs in (200, 800):
        model = train_mlp(normalized[train], residual[train] / scale,
                          hidden=8, epochs=epochs, seed=19)
        model['w2'] *= scale
        model['b2'] *= scale
        pred = np.maximum(normalized @ model['w1'].T + model['b1'], 0) @ model['w2'] + model['b2']
        candidates.append((metrics(residual[validation], pred[validation])['rmse_v'],
                           epochs, model, pred))
    _, epochs, model, pred = min(candidates, key=lambda r: r[0])
    # Cheap explicit harmonic comparator; select complexity using validation only.
    harmonic_candidates = []
    for count in (1, 2, 3, 4, 5):
        a = harmonic_design(x, count)
        weights = np.linalg.solve(a[train].T @ a[train] + .01*np.eye(a.shape[1]),
                                  a[train].T @ residual[train])
        harmonic = a @ weights
        harmonic_candidates.append((metrics(residual[validation], harmonic[validation])['rmse_v'],
                                     count, weights, harmonic))
    _, count, weights, harmonic = min(harmonic_candidates, key=lambda r: r[0])
    report = dict(schema='angle_voltage_candidate_v1', source=str(source.resolve()),
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  config=args.config, harmonics=list(HARMONICS), hidden=8,
                  selected_epochs=epochs, selected_explicit_harmonics=list(HARMONICS[:count]),
                  split='repeat0 train; repeat1 selects epochs/harmonics; repeat2 held out',
                  target='Recorded total voltage minus speed-model voltage and train-only per-speed DC',
                  angle_reference='Unwrapped calibration-relative position, modulo 2pi; same calibration required',
                  limitation='Closed-loop voltage correlation only, not optimal compensation or motor improvement. '
                             'No acceleration/current measurement; ff1 firmware incompatible; not deployed.',
                  **phase_meta, splits={})
    for name, mask in zip(('train', 'validation', 'test'), masks):
        report['splits'][name] = dict(
            constant=metrics(residual[mask], np.zeros(mask.sum())),
            explicit_harmonics=metrics(residual[mask], harmonic[mask]),
            nn=metrics(residual[mask], pred[mask]),
            by_rpm={str(rpm): dict(
                constant=metrics(residual[m], np.zeros(m.sum())),
                explicit_harmonics=metrics(residual[m], harmonic[m]),
                nn=metrics(residual[m], pred[m]))
                for rpm in offsets for m in [mask & (data[:, 1] == rpm)]})
    report['offline_comparison_passed'] = all(
        v['nn']['rmse_v'] < min(v['constant']['rmse_v'], v['explicit_harmonics']['rmse_v'])
        for v in report['splits']['test']['by_rpm'].values())
    np.savez(output, schema=report['schema'], harmonics=HARMONICS,             input_mean=mean, input_std=std, hidden_weights=model['w1'],
             hidden_bias=model['b1'], output_weights=model['w2'], output_bias=model['b2'],
             dc_rpm=list(offsets), dc_voltage=list(offsets.values()),
             explicit_weights=weights, explicit_harmonic_count=count)
    output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(offsets), 1, figsize=(11, 2.5*len(offsets)), constrained_layout=True)
    for ax, rpm in zip(np.atleast_1d(axes), offsets):
        m = test & (data[:, 1] == rpm)
        ax.plot(residual[m], label='Recorded AC voltage', alpha=.5)
        ax.plot(harmonic[m], label='Explicit harmonics', alpha=.8)
        ax.plot(pred[m], label='8-neuron NN', alpha=.8)
        ax.set(title=f'Held-out repeat: {rpm:+g} RPM', ylabel='Voltage (V)', xlabel='Steady samples (10 ms)')
        ax.legend(fontsize=8)
    fig.savefig(output.with_suffix('.png'), dpi=140)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
