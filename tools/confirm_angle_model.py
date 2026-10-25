"""Confirm the corrected angle model at +/-15 RPM, with two controls.

Frozen before the run:
  baseline              G0, no compensation
  angle025_applied      G6, alpha 0.25, zero phase (the only phase evidenced
                        better than nothing by the label fit: +0.25 deg)
  zero_upload_control   G6, alpha 0.25, but the uploaded tensor is the model
                        with every input weight = 0, so the network output is
                        just the constant bias. This isolates the effect of the
                        *learned* angle-dependent AC term from the DC bias and
                        from the G6 path itself.
  nn_off_upload_control G0, same upload path exercised, no compensation

Same calibration, same P150/Q100, three repeats of 6 s, fixed order.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from angle_phase_model import PhaseTable, load_model, upload_sequence
from angle_phase_ab import Rig, metrics

MODE_ANGLE_TRIAL = 6


def flatten_model(model, mode):
    """Return a copy whose hidden input weights are zero (bias-only network)."""
    flat = {k: (v.copy() if hasattr(v, 'copy') else v) for k, v in model.items()}
    if mode == 'flat':
        flat['hidden_weights'] = np.zeros_like(model['hidden_weights'])
    return flat


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--targets', nargs='+', type=int, default=[15, -15])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--seconds', type=float, default=6)
    parser.add_argument('--alpha', type=int, default=250)
    parser.add_argument('--port', default='auto')
    args = parser.parse_args()
    output = Path(args.output)
    if any(output.with_suffix(s).exists() for s in ('.csv', '.json')):
        raise FileExistsError(output)
    model = load_model(args.model)
    plans = [
        ('baseline', None, 0, 0),
        ('angle025_applied', model, MODE_ANGLE_TRIAL, args.alpha),
        ('zero_upload_control', flatten_model(model, 'flat'), MODE_ANGLE_TRIAL, args.alpha),
    ]
    table = PhaseTable.constant(0.0)
    results = []
    with Rig(args.port) as rig:
        print('health', ','.join(rig.health()), flush=True)
        with output.open('x', newline='', encoding='utf-8') as stream:
            writer = None
            for repeat in range(args.repeats):
                for name, payload, mode, alpha in plans:
                    rig.command('G', 0)
                    rig.command('P', 150)
                    rig.command('Q', 100)
                    if payload is not None:
                        rig.upload(payload, table)
                    rig.command('K', alpha)
                    rig.command('G', mode)
                    for target in args.targets:
                        rows = rig.step(target, args.seconds)
                        if len(rows) < args.seconds * 60:
                            raise RuntimeError('insufficient telemetry')
                        if writer is None:
                            writer = csv_writer(stream, rows[0])
                        for row in rows:
                            writer.writerow(dict(config=name, repeat=repeat,
                                                 target_rpm=target, **row))
                        result = dict(config=name, repeat=repeat, target_rpm=target,
                                      mode=mode, alpha=alpha,
                                      nn_delta_mean_mv=float(np.mean([r['nn_delta_uq_mv']
                                                                      for r in rows])),
                                      nn_delta_std_mv=float(np.std([r['nn_delta_uq_mv']
                                                                    for r in rows])),
                                      uq_std_mv=float(np.std([r['uq_mv'] for r in rows])),
                                      **metrics(rows, target))
                        results.append(result)
                        stream.flush()
                        output.with_suffix('.json').write_text(json.dumps(results, indent=2),
                                                              encoding='utf-8')
                        print(json.dumps({k: v for k, v in result.items()}), flush=True)
                    rig.command('G', 0)
        print('final', ','.join(rig.health()), flush=True)


def csv_writer(stream, row):
    import csv
    writer = csv.DictWriter(stream, fieldnames=['config', 'repeat', 'target_rpm'] + list(row))
    writer.writeheader()
    return writer


if __name__ == '__main__':
    main()
