"""Evaluate a frozen angle model on a new recording; no training or UART."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from train_angle_residual import features, harmonic_design, metrics, prepare, HARMONICS


def evaluate(source, model_path):
    source, model_path = Path(source), Path(model_path)
    training = json.loads(model_path.with_suffix('.json').read_text(encoding='utf-8'))
    if hashlib.sha256(source.read_bytes()).hexdigest() == training['source_sha256']:
        raise ValueError('evaluation recording must differ from training recording')
    with source.open(newline='', encoding='utf-8') as stream:
        data, _ = prepare(list(csv.DictReader(stream)), training['config'], require_three=False)
    with np.load(model_path, allow_pickle=False) as archive:
        if (str(archive['schema']) != 'angle_voltage_candidate_v1' or
                not np.array_equal(archive['harmonics'], HARMONICS)):
            raise ValueError('incompatible angle model')
        offsets = dict(zip(archive['dc_rpm'], archive['dc_voltage']))
        if set(data[:, 1]) != set(offsets):
            raise ValueError('evaluation must cover exactly the trained signed speeds; '
                             'collect separate generalization experiments for unseen speeds')
        x = features(data[:, 2], data[:, 1])
        norm = (x - archive['input_mean']) / archive['input_std']
        pred = (np.maximum(norm @ archive['hidden_weights'].T + archive['hidden_bias'], 0)
                @ archive['output_weights'] + archive['output_bias'])
        harmonic = harmonic_design(x, int(archive['explicit_harmonic_count'])) @ archive['explicit_weights']
        residual = data[:, 3] - np.array([offsets[r] for r in data[:, 1]])
        if not np.all(np.isfinite(pred)):
            raise ValueError('nonfinite predictions')
    def result(mask):
        return dict(constant=metrics(residual[mask], np.zeros(mask.sum())),
                    explicit_harmonics=metrics(residual[mask], harmonic[mask]),
                    nn=metrics(residual[mask], pred[mask]))
    report = dict(source=str(source.resolve()),
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  model=str(model_path.resolve()), all=result(np.ones(len(data), dtype=bool)),
                  by_rpm={str(rpm): result(data[:, 1] == rpm) for rpm in sorted(offsets)},
                  limitation='Frozen-model voltage prediction only; same calibration/load, no control benefit claim.')
    report['all_tested_speeds_improve'] = all(
        r['nn']['rmse_v'] < min(r['constant']['rmse_v'], r['explicit_harmonics']['rmse_v'])
        for r in report['by_rpm'].values())
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--model', required=True)
    args = parser.parse_args()
    source = Path(args.input)
    output = source.with_name(source.stem+'_angle_evaluation.json')
    if output.exists():
        raise FileExistsError(output)
    report = evaluate(source, args.model)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
