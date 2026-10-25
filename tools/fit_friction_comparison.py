"""Fit a two-parameter friction correction and an additive learned residual.

Both are encoded using the SAME existing ff1 path; no firmware change.
Labels are steady PI voltage residuals, not direct torque/friction measurements.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from motor_feedforward import KNOTS, predict_float32
from train_motor_feedforward import fit_weights


def friction_weights(points):
    rpm = np.array([p['rpm'] for p in points])
    design = np.column_stack((np.clip(rpm / 9, -1, 1), rpm / 180))
    if np.linalg.matrix_rank(design) != 2:
        raise ValueError('friction fit needs multiple distinct speed magnitudes')
    coefficients = np.linalg.lstsq(
        design, [p['residual_v'] for p in points], rcond=None)[0]
    coulomb, slope = coefficients
    weights = np.zeros(9)
    weights[0] = slope
    weights[3] = coulomb * 20
    weights[4] = -coulomb * 20
    integers = np.rint(weights * 10000).astype(int)
    # Enforce zero-at-zero after quantization, just like the NN exporter.
    integers[8] = round(-integers[:8] @ np.maximum(-KNOTS, 0))
    return integers, coefficients


def read_points(path, baseline):
    with Path(path).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    points = []
    keys = dict.fromkeys((r['config'], int(r['repeat']), float(r['target_rpm'])) for r in rows)
    for config, repeat, rpm in keys:
        if config != baseline:
            continue
        samples = [r for r in rows if r['config'] == config and
                   int(r['repeat']) == repeat and float(r['target_rpm']) == rpm]
        if len(samples) < 200:
            raise ValueError('need >=2 s per trial')
        ticks = np.array([int(r['tick_ms']) for r in samples], dtype=np.int64)
        if np.any(np.diff(ticks) <= 0) or np.max(np.diff(ticks)) > 30:
            raise ValueError('nonmonotonic or gapped telemetry')
        tail = [r for r in samples if int(r['tick_ms']) >= ticks[-1] - 800]
        if len(tail) < 60 or any(int(r['fault']) or int(r['clipped']) or
                                not int(r['sensor_ok']) or not int(r['calibrated'])
                                for r in samples):
            raise ValueError('unhealthy, clipped or incomplete trial')
        residual = np.mean([int(r['uq_mv']) for r in tail]) / 1000 - rpm * 2*np.pi/60/44.18
        points.append(dict(repeat=repeat, rpm=rpm, residual_v=float(residual)))
    for repeat in (0, 1, 2):
        speeds = {p['rpm'] for p in points if p['repeat'] == repeat}
        if len(speeds) < 6 or not any(s < 0 for s in speeds) or not any(s > 0 for s in speeds):
            raise ValueError('need three repeats with six signed speeds each')
    return points


def fit_models(points):
    train = [p for p in points if p['repeat'] == 0]
    validation = [p for p in points if p['repeat'] == 1]
    test = [p for p in points if p['repeat'] >= 2]
    classical, coefficients = friction_weights(train)
    if np.max(np.abs(classical)) > 100000:
        raise ValueError('friction weights exceed firmware upload bounds')
    residual_points = [dict(p, residual_v=p['residual_v'] -
                           float(predict_float32([p['rpm']], classical)[0])) for p in train]
    candidates = []
    for ridge in (1e-6, 1e-5, 1e-4, 1e-3, 1e-2):
        total = classical + fit_weights(residual_points, ridge)
        if np.max(np.abs(total)) > 100000:
            continue
        error = predict_float32([p['rpm'] for p in validation], total) - [
            p['residual_v'] for p in validation]
        candidates.append((float(np.mean(error**2)), ridge, total))
    if not candidates:
        raise ValueError('no representable residual model')
    _, ridge, total = min(candidates, key=lambda item: item[0])
    models = {}
    for name, weights in (('friction', classical), ('friction_nn', total)):
        grid = predict_float32(np.linspace(-30, 30, 601), weights)
        if np.max(np.abs(grid)) > .25:
            raise ValueError('candidate exceeds 0.25 V study envelope')
        models[name] = dict(schema='relu_speed_ff1', knots=KNOTS.tolist(),
            weight_units_v=.0001, upload_weights=weights.tolist(),
            study_kind=name, measured_rpm_range=[min(p['rpm'] for p in points),
                                               max(p['rpm'] for p in points)],
            split='repeat0 fit; repeat1 select residual ridge; repeat>=2 held-out',
            coulomb_residual_v=float(coefficients[0]),
            slope_residual_v_per_180rpm=float(coefficients[1]), residual_ridge=ridge,
            heldout_voltage_rmse_v=float(np.sqrt(np.mean((
                predict_float32([p['rpm'] for p in test], weights) -
                [p['residual_v'] for p in test])**2))),
            limitation='Static target-speed residual only; no proven dynamic friction identification. '
                       'Voltage fit is not closed-loop improvement.',
            points=points)
    return models


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--output-prefix', required=True)
    parser.add_argument('--baseline', default='pi_kp150_ki100')
    args = parser.parse_args()
    prefix = Path(args.output_prefix)
    destinations = {name: prefix.with_name(prefix.name + '_' + name + '.json')
                    for name in ('friction', 'friction_nn')}
    if any(p.exists() for p in destinations.values()):
        raise FileExistsError('model output already exists')
    models = fit_models(read_points(args.input, args.baseline))
    for name, model in models.items():
        model.update(source=str(Path(args.input).resolve()),
                     source_sha256=hashlib.sha256(Path(args.input).read_bytes()).hexdigest())
        destinations[name].write_text(json.dumps(model, indent=2), encoding='utf-8')
        print(name, model['heldout_voltage_rmse_v'], destinations[name])


if __name__ == '__main__':
    main()
