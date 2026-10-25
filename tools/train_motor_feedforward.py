"""Fit an eight-ReLU voltage feedforward network to measured steady voltage.

This learns plant-model bias/friction; it does not imitate a PD teacher.
The existing integral controller remains active when this residual is applied.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from motor_feedforward import KNOTS, basis, predict_float32


def fit_weights(points, ridge):
    zero_basis = basis([0.])[0, :8]
    x = basis([p['rpm'] for p in points])[:, :8]-zero_basis
    y = np.array([p['residual_v'] for p in points])
    hidden_weights = np.linalg.solve(x.T@x + ridge*np.eye(8), x.T@y)
    integers = np.rint(np.r_[hidden_weights, -hidden_weights@zero_basis]*10000).astype(int)
    integers[8] = round(-integers[:8]@zero_basis)
    if np.max(np.abs(integers)) > 100000:
        raise ValueError('weights exceed firmware upload bounds')
    return integers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--output', required=True)
    parser.add_argument('--baseline', default='baseline',
                        help='Traditional-control configuration used for voltage labels')
    parser.add_argument('--evaluate', help='Independent recording, used only after model selection')
    args = parser.parse_args()
    if Path(args.output).exists():
        raise FileExistsError(args.output)
    with Path(args.input).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    points = []
    for key in dict.fromkeys((r['config'], r['repeat'], r['target_rpm']) for r in rows):
        config, repeat, target = key
        if config != args.baseline:
            continue
        samples = [r for r in rows if (r['config'], r['repeat'], r['target_rpm']) == key]
        tail = samples[-80:]
        if len(tail) < 60 or any(int(r['fault']) or int(r['clipped']) or not int(r['sensor_ok']) or not int(r['calibrated']) for r in tail):
            continue
        speed = float(target)
        # PI steady output contains the missing friction/model voltage.
        residual = np.mean([int(r['uq_mv'])/1000 for r in tail])-speed*2*np.pi/60/44.18
        points.append(dict(repeat=int(repeat), rpm=speed, residual_v=float(residual)))
    train = [p for p in points if p['repeat'] == 0]
    validation = [p for p in points if p['repeat'] == 1]
    test = [p for p in points if p['repeat'] >= 2]
    if any(len({p['rpm'] for p in subset}) < 6 for subset in (train, validation, test)):
        raise ValueError('need six signed speeds, two training repeats and one held-out repeat')
    candidates = []
    for ridge in (1e-6, 1e-5, 1e-4, 1e-3, 1e-2):
        integers = fit_weights(train, ridge)
        pred = predict_float32([p['rpm'] for p in validation], integers)
        validation_rmse = float(np.sqrt(np.mean((pred-[p['residual_v'] for p in validation])**2)))
        candidates.append((validation_rmse, ridge, integers))
    validation_rmse, ridge, integers = min(candidates, key=lambda item: item[0])
    predictions = predict_float32([p['rpm'] for p in test], integers)
    labels = np.array([p['residual_v'] for p in test])
    report = dict(schema='relu_speed_ff1', knots=KNOTS.tolist(),
                  weight_units_v=.0001, upload_weights=integers.tolist(),
                  ridge=ridge, validation_voltage_rmse_v=validation_rmse,
                  split='repeat 0 train, repeat 1 selects ridge, repeats >=2 held-out test',
                  heldout_voltage_rmse_v=float(np.sqrt(np.mean((predictions-labels)**2))),
                  zero_voltage_rmse_v=float(np.sqrt(np.mean(labels**2))),
                  points=points,
                  baseline_config=args.baseline,
                  measured_rpm_range=[min(p['rpm'] for p in points), max(p['rpm'] for p in points)],
                  limitation='Steady-voltage fit only; real-board blended control must be tested.')
    if args.evaluate:
        if Path(args.evaluate).resolve() == Path(args.input).resolve():
            raise ValueError('independent evaluation must use a different recording')
        with Path(args.evaluate).open(newline='', encoding='utf-8') as stream:
            evaluation = list(csv.DictReader(stream))
        errors = []
        for key in dict.fromkeys((r['repeat'], r['target_rpm']) for r in evaluation if r['config'] == args.baseline):
            tail = [r for r in evaluation if r['config'] == args.baseline and
                    (r['repeat'], r['target_rpm']) == key][-80:]
            if len(tail) < 60 or any(int(r['fault']) or int(r['clipped']) or not int(r['sensor_ok']) or not int(r['calibrated']) for r in tail):
                continue
            rpm = float(key[1])
            label = np.mean([int(r['uq_mv'])/1000 for r in tail])-rpm*2*np.pi/60/44.18
            errors.append(float(predict_float32([rpm], integers)[0])-label)
        if not errors:
            raise ValueError('no healthy independent baseline trials')
        report['independent_recording'] = dict(source=str(Path(args.evaluate).resolve()),
            trials=len(errors), voltage_rmse_v=float(np.sqrt(np.mean(np.array(errors)**2))))
    Path(args.output).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
