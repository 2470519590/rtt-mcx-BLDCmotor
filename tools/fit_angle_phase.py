"""Speed-dependent phase identification for the angle-correlated candidate.

Replaces the single phase fitted by register_angle_phase.py with one phase per
normalized speed, so positive and negative speeds get independent phases. Two
parameterizations are reported:

* explicit: the recorded voltage residual is projected onto the harmonic
  design at each speed, so each phase is closed form, not a search;
* joint:    the trained network is re-evaluated under a per-speed phase, which
  is what the firmware executes, so a phase also moves the network state.

Phases are fitted on repeat0 only. repeat1 would select a complexity option;
repeat2 and any untrained intermediate speed stay frozen evaluation. Nothing
here touches the motor or the MCU.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from angle_phase_model import HARMONICS, PhaseTable, load_model, predict


def prepare(rows, config):
    groups = {}
    for row in rows:
        if row['config'] == config:
            groups.setdefault((int(row['repeat']), int(row['target_rpm'])), []).append(row)
    records = []
    for (repeat, target), samples in sorted(groups.items()):
        ticks = np.array([int(r['tick_ms']) for r in samples])
        if len(ticks) < 300 or np.any(np.diff(ticks) <= 0) or np.max(np.diff(ticks)) > 30:
            raise ValueError(f'insufficient or discontinuous trial repeat={repeat} rpm={target}')
        for row in samples:
            if (int(row['fault']) or int(row['clipped']) or not int(row['sensor_ok'])
                    or not int(row['calibrated']) or int(row['mode']) != 2
                    or int(row['nn_mode']) != 0):
                raise ValueError('identification requires healthy unclipped NN-off speed trials')
        steady = [r for r in samples if int(r['tick_ms']) - ticks[0] >= 2000]
        if len(steady) < 100:
            raise ValueError('need steady data after a 2 s startup exclusion')
        position = np.array([int(r['position_mrad']) for r in steady]) / 1000
        if abs(position[-1] - position[0]) < 2 * np.pi:
            raise ValueError('steady trial must span at least one mechanical revolution')
        records.extend((repeat, target, p, int(r['uq_mv']) / 1000 - target * 2 * np.pi / 60 / 44.18)
                       for p, r in zip(position, steady))
    data = np.asarray(records)
    masks = {int(r): data[:, 0] == r for r in np.unique(data[:, 0])}
    return data, masks


def harmonic_design(position):
    return np.column_stack([f(h * position) for h in HARMONICS for f in (np.sin, np.cos)])


def explicit_phase(residual, position):
    """Least-squares harmonic fit; return the shift each harmonic implies.

    A sin(h*a) + B cos(h*a) equals R sin(h*(a + p)) with p = atan2(B, A)/h, so
    the fundamental phase is the h=1 entry and the others should agree if the
    label really is one rotation-periodic pattern.
    """
    design = harmonic_design(position)
    coefficients, _, _, _ = np.linalg.lstsq(design, residual, rcond=None)
    phases = [float(np.arctan2(coefficients[2 * k + 1], coefficients[2 * k]) / HARMONICS[k])
              for k in range(len(HARMONICS))]
    amplitude = [float(np.hypot(coefficients[2 * k], coefficients[2 * k + 1]))
                 for k in range(len(HARMONICS))]
    explained = 1 - float(np.mean((residual - design @ coefficients) ** 2)) / \
        max(float(np.mean(residual ** 2)), 1e-12)
    return phases, amplitude, explained


def rmse(a, b):
    return float(np.sqrt(np.mean((np.asarray(a) - np.asarray(b)) ** 2)))


def coordinate_search(data, mask, model, speeds, centers, interp,
                      span=np.pi / 2, coarse=41, rounds=3):
    """Move one point phase at a time while the others stay fixed."""
    phases = [float((c + np.pi) % (2 * np.pi) - np.pi) for c in centers]

    def error(candidate):
        table = PhaseTable(list(zip(speeds, candidate)), interp=interp)
        return rmse(data[mask, 3], predict(data[mask, 2], data[mask, 1], model,
                                           table.phase(data[mask, 1])))

    best = error(phases)
    for _ in range(rounds):
        improved = False
        for index in range(len(phases)):
            grid = np.linspace(phases[index] - span, phases[index] + span, coarse)
            for candidate in grid:
                trial = list(phases)
                trial[index] = float((candidate + np.pi) % (2 * np.pi) - np.pi)
                value = error(trial)
                if value < best - 1e-9:
                    best, phases, improved = value, trial, True
        if not improved:
            break
    return phases, best


def fit_delay(data, mask, model, speed_of_row, label, table_points):
    """Scan one constant time delay shared by every speed.

    The per-speed phase is then 2*pi*f*delay, so a single parameter replaces
    four loosely determined phases and can be checked at unseen speeds: the
    relation is kinematic, not fitted.
    """
    from angle_phase_model import delay_rad
    speeds = sorted(set(int(s) for s in speed_of_row[mask]))
    speed_array = np.array([float(s) for s in speeds])
    best = None
    for delay in np.concatenate([np.arange(-2.0, 2.001, 0.01)]):
        phases = np.angle(np.exp(1j * delay_rad(speed_array, delay)))
        table = PhaseTable(list(zip(speed_array.tolist(), phases.tolist())), interp=0)
        error = rmse(label[mask], predict(data[mask, 2], data[mask, 1], model,
                                          table.phase(data[mask, 1])))
        if best is None or error < best[0]:
            best = (error, float(delay))
    for delay in np.arange(best[1] - .01, best[1] + .0101, .001):
        phases = np.angle(np.exp(1j * delay_rad(speed_array, delay)))
        table = PhaseTable(list(zip(speed_array.tolist(), phases.tolist())), interp=0)
        error = rmse(label[mask], predict(data[mask, 2], data[mask, 1], model,
                                          table.phase(data[mask, 1])))
        if error < best[0]:
            best = (error, float(delay))
    phases = np.angle(np.exp(1j * delay_rad(speed_array, best[1])))
    return best[1], best[0], PhaseTable(list(zip(speed_array.tolist(), phases.tolist())), interp=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--model', required=True)
    parser.add_argument('--config', default='pi_kp150_ki100')
    parser.add_argument('--output', required=True, help='JSON report path (must be new)')
    parser.add_argument('--holdout-speeds', nargs='*', type=int, default=[],
                        help='Speeds kept out of the phase fit and evaluated frozen')
    parser.add_argument('--skip-joint', action='store_true')
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    source = Path(args.input)
    with source.open(newline='', encoding='utf-8') as stream:
        data, masks = prepare(list(csv.DictReader(stream)), args.config)
    model = load_model(args.model)
    all_speeds = sorted(int(v) for v in np.unique(data[:, 1]))
    fit_speeds = [s for s in all_speeds if s not in args.holdout_speeds]
    if not 2 <= len(fit_speeds) <= 8:
        raise ValueError('need 2..8 fit speeds for the normalization table')
    if 0 not in masks:
        raise ValueError('repeat0 is required for the phase fit')
    speed_of_row = np.round(data[:, 1]).astype(int)
    fit_mask = masks[0] & np.isin(speed_of_row, fit_speeds)
    # Speed offsets are estimated on repeat0 only, matching train_angle_residual.
    offsets = {s: float(data[masks[0] & (speed_of_row == s), 3].mean()) for s in fit_speeds}
    label = data[:, 3] - np.array([offsets.get(s, 0.0) for s in speed_of_row])

    explicit = {}
    for speed in fit_speeds:
        group = fit_mask & (speed_of_row == speed)
        phases, amplitude, explained = explicit_phase(label[group], data[group, 2])
        explicit[speed] = dict(phases_rad=phases, phases_deg=[float(np.degrees(p)) for p in phases],
                               amplitude_v=amplitude, harmonic_explained=explained,
                               fundamental=phases[0])
    # Global single phase: the configuration the firmware had before this work.
    best_global = None
    for candidate in np.linspace(-np.pi, np.pi, 721, endpoint=False):
        table = PhaseTable([(fit_speeds[0], float(candidate))], interp=0)
        error = rmse(label[fit_mask], predict(data[fit_mask, 2], data[fit_mask, 1],
                                              model, table.phase(data[fit_mask, 1])))
        if best_global is None or error < best_global[0]:
            best_global = (error, float(candidate))
    tables = {
        'global': PhaseTable([(float(fit_speeds[0]), best_global[1])], interp=0),
        'per_speed_explicit': PhaseTable([(float(s), explicit[s]['fundamental'])
                                          for s in fit_speeds], interp=0),
    }
    joint_report = {}
    if not args.skip_joint:
        centers = [explicit[s]['fundamental'] for s in fit_speeds]
        for mode in (0, 1):
            phases, error = coordinate_search(data, fit_mask, model, [float(s) for s in fit_speeds],
                                              centers, mode)
            name = 'per_speed_joint' if mode == 0 else 'per_speed_joint_interp'
            tables[name] = PhaseTable(list(zip([float(s) for s in fit_speeds], phases)), interp=mode)
            joint_report[name] = dict(phases_rad=phases, fit_label_rmse=error)
        delay, error, table = fit_delay(data, fit_mask, model, speed_of_row, label, fit_speeds)
        tables['delay_model'] = table
        joint_report['delay_model'] = dict(delay_s=delay, fit_label_rmse=error,
                                           phases_rad=[float(p) for p in
                                                       table.phase(np.array([float(s) for s in fit_speeds]))])
    report = dict(
        schema='angle_phase_study_v1', source=str(source.resolve()),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        model=str(Path(args.model).resolve()), model_sha256=model['sha256'],
        config=args.config, fit_speeds=fit_speeds, holdout_speeds=sorted(args.holdout_speeds),
        split='repeat0 phase fit; repeat2 and holdout speeds frozen evaluation; '
              'joint tables are fitted on repeat0 with repeat1 reserved for description only',
        global_phase=dict(phase_rad=best_global[1],
                          phase_deg=float(np.degrees(best_global[1])),
                          fit_label_rmse=best_global[0]),
        explicit={str(k): v for k, v in explicit.items()}, joint=joint_report,
        phase_tables={name: t.as_dict() for name, t in tables.items()},
        limitation='Voltage residual from a closed loop: it contains controller action, estimator error and '
                   'dynamics, not a measured disturbance torque. Lower label RMSE is not a control improvement.',
        points=[])
    for speed in all_speeds:
        point = dict(rpm=speed, in_fit=speed in fit_speeds, repeats={})
        for repeat, mask in sorted(masks.items()):
            cell = mask & (speed_of_row == speed)
            if not cell.any():
                continue
            entry = dict(rows=int(cell.sum()), constant=rmse(label[cell], 0.0), label_rmse={})
            for name, table in tables.items():
                entry['label_rmse'][name] = rmse(label[cell], predict(
                    data[cell, 2], data[cell, 1], model, table.phase(data[cell, 1])))
            point['repeats'][str(repeat)] = entry
        report['points'].append(point)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    names = list(tables)
    print('speed        ' + ' '.join(f'{n[:22]:>22s}' for n in names))
    for point in report['points']:
        entry = point['repeats'].get(max(point['repeats']))
        print(f"{point['rpm']:+5d} fit={int(point['in_fit'])} " +
              ' '.join(f'{entry["label_rmse"][n]:22.4f}' for n in names))
    print(output)


if __name__ == '__main__':
    main()
