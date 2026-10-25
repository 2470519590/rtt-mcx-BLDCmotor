"""Board A/B for the DC-only trim, with the controls that make it falsifiable.

Fixed order, same calibration, same P150/Q100, three repeats per configuration:

  baseline          G0
  dc_trim           G6, alpha 0.25, zero-weight network, per-speed constant
                    voltage from the measured DC (the intervention)
  zero_upload       G6, alpha 0.25, zero-weight network, all phases 0
                    (path control: must match baseline within noise)
  dc_half           G6, alpha 0.25, the same DC scaled by 0.5
                    (dose check: benefit should scale, not jump)

Speeds cover the fitted points and the two held-out intermediate speeds, so a
DC table that only "works" at its own training speeds is exposed.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from angle_phase_model import PhaseTable, load_model, upload_sequence
from angle_phase_ab import Rig, metrics
from upload_dc_trim import dc_table

MODE_ANGLE_TRIAL = 6


def rsp_read_row(rig, neuron):
    """Read one neuron's stored coefficients from the MCU (1e-4 units)."""
    values = [rig.read_coefficient(neuron, slot) for slot in range(15)]
    return (values[0] / 10000.0, [v / 10000.0 for v in values[1:13]], values[13] / 10000.0)


def flat_model(model):
    flat = {k: (v.copy() if hasattr(v, 'copy') else v) for k, v in model.items()}
    flat['hidden_weights'] = np.zeros_like(model['hidden_weights'])
    return flat


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--split', required=True)
    parser.add_argument('--targets', nargs='+', type=int, default=[15, -15, 20, -20, 30, -30])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--seconds', type=float, default=6)
    parser.add_argument('--alpha', type=int, default=250)
    parser.add_argument('--port', default='auto')
    args = parser.parse_args()
    output = Path(args.output)
    if any(output.with_suffix(s).exists() for s in ('.csv', '.json')):
        raise FileExistsError(output)
    if any(not 15 <= abs(t) <= 30 for t in args.targets):
        raise SystemExit('G6 is limited to 15..30 RPM magnitude')
    model = load_model(args.model)
    flat = flat_model(model)
    table = dc_table(args.split, {'-30': -30.0, '-20': -20.0, '-15': -15.0,
                                  '15': 15.0, '20': 20.0, '30': 30.0}, args.alpha)
    half = PhaseTable([(rpm, phase * .5) for rpm, phase in table.points], interp=1)
    zero = PhaseTable([(rpm, 0.0) for rpm, _ in table.points], interp=0)
    plans = [('baseline', None, 0), ('dc_trim', table, MODE_ANGLE_TRIAL),
             ('zero_upload', zero, MODE_ANGLE_TRIAL), ('half_bias_control', half, MODE_ANGLE_TRIAL)]
    results = []
    # Every upload is checked coefficient by coefficient before any motion, so
    # a silent truncation or a staging mix-up stops the run instead of being
    # measured as a control result.
    expected_flat = flat
    declared = set()
    with Rig(args.port) as rig:
        print('health', ','.join(rig.health()), flush=True)
        with output.open('x', newline='', encoding='utf-8') as stream:
            writer = None
            for repeat in range(args.repeats):
                for name, phase_table, mode in plans:
                    rig.command('G', 0)
                    rig.command('P', 150)
                    rig.command('Q', 100)
                    if phase_table is not None:
                        for op, value in upload_sequence(flat, phase_table):
                            rig.command(op, value)
                        if 'flat' not in declared:
                            print('upload check', json.dumps(verify_upload(
                                expected_flat, table, lambda n: rsp_read_row(rig, n))), flush=True)
                            declared.add('flat')
                    rig.command('K', args.alpha if mode else 0)
                    rig.command('G', mode)
                    # Confirm the per-speed offset that is actually active
                    # before measuring, in the same path the comparison uses.
                    for probe_target in args.targets:
                        probe = rig.probe_applied(probe_target, seconds=1.0)
                        print(json.dumps(dict(config=name, repeat=repeat, target_rpm=target,
                                              probe_target=probe_target, probe=probe)), flush=True)
                    for target in args.targets:
                        rows = rig.step(target, args.seconds)
                        if len(rows) < args.seconds * 60:
                            raise RuntimeError('insufficient telemetry')
                        if writer is None:
                            writer = csv.DictWriter(stream, fieldnames=['config', 'repeat',
                                                                        'target_rpm'] + list(rows[0]))
                            writer.writeheader()
                        for row in rows:
                            writer.writerow(dict(config=name, repeat=repeat,
                                                 target_rpm=target, **row))
                        result = dict(config=name, repeat=repeat, target_rpm=target, mode=mode,
                                      nn_delta_mean_mv=float(np.mean([r['nn_delta_uq_mv'] for r in rows])),
                                      nn_delta_std_mv=float(np.std([r['nn_delta_uq_mv'] for r in rows])),
                                      uq_mean_mv=float(np.mean([r['uq_mv'] for r in rows])),
                                      **metrics(rows, target))
                        results.append(result)
                        stream.flush()
                        output.with_suffix('.json').write_text(json.dumps(results, indent=2),
                                                              encoding='utf-8')
                        print(json.dumps({k: v for k, v in result.items() if k not in
                                          ('iae', 'ss_iae')}), flush=True)
                    rig.command('G', 0)
        print('final', ','.join(rig.health()), flush=True)


if __name__ == '__main__':
    main()
