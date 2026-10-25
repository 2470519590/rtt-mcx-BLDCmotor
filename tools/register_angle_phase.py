"""Estimate changed encoder-zero phase on repeat0; freeze for repeats1/2.

No MCU writes. This evaluates old-model transfer, not closed-loop benefit.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from angle_shadow_model import load_angle_model
from train_angle_residual import features, metrics, prepare


def predict(position, rpm, model, phase):
    x = (features(position+phase, rpm)-model['input_mean'])/model['input_std']
    return np.maximum(x@model['hidden_weights'].T+model['hidden_bias'], 0)@model['output_weights']+model['output_bias']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--model', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    source = Path(args.input)
    with source.open(newline='', encoding='utf-8') as stream:
        data, masks = prepare(list(csv.DictReader(stream)), 'pi_kp150_ki100')
    model = load_angle_model(args.model)
    train = masks[0]
    offsets = {rpm: data[train & (data[:, 1] == rpm), 3].mean() for rpm in set(data[:, 1])}
    residual = data[:, 3]-np.array([offsets[r] for r in data[:, 1]])
    # Phase only fitted on repeat0. Coarse then fine searches avoid a local-only fit.
    selected = np.flatnonzero(train)[::4]
    def loss(phase):
        return metrics(residual[selected], predict(data[selected, 2], data[selected, 1], model, phase))['rmse_v']
    grid = np.linspace(-np.pi, np.pi, 256, endpoint=False)
    coarse = min(grid, key=loss)
    phase = min(np.linspace(coarse-2*np.pi/256, coarse+2*np.pi/256, 81), key=loss)
    phase = float((phase+np.pi) % (2*np.pi)-np.pi)
    direct = predict(data[:, 2], data[:, 1], model, 0)
    aligned = predict(data[:, 2], data[:, 1], model, phase)
    report = dict(source=str(source.resolve()), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  model=str(Path(args.model).resolve()), phase_rad=phase,
                  phase_deg=float(np.degrees(phase)), split='repeat0 phase/DC fit; repeat1/2 frozen evaluation',
                  limitation='Voltage correlation after calibration phase registration, not control improvement. '
                             'No compensation applied. Phase is specific to this calibration.', splits={})
    for name, mask in zip(('registration', 'validation', 'test'), masks):
        report['splits'][name] = dict(
            constant=metrics(residual[mask], np.zeros(mask.sum())),
            unregistered=metrics(residual[mask], direct[mask]),
            registered=metrics(residual[mask], aligned[mask]),
            by_rpm={str(rpm): dict(constant=metrics(residual[m], np.zeros(m.sum())),
                                  unregistered=metrics(residual[m], direct[m]),
                                  registered=metrics(residual[m], aligned[m]))
                    for rpm in sorted(offsets) for m in [mask & (data[:, 1] == rpm)]})
    report['all_test_speeds_better_than_constant'] = all(
        r['registered']['rmse_v'] < r['constant']['rmse_v']
        for r in report['splits']['test']['by_rpm'].values())
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
