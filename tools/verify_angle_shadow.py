"""Check angle-shadow L frames, including telemetry input quantization.

This does not assume the telemetry's truncated angle is the full-precision
inference input. Interval propagation covers that loss of precision.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from angle_shadow_model import HARMONICS, load_angle_model, predict_float32


def prediction_interval(position_mrad, target_mrad_s, model):
    p = np.asarray(position_mrad, dtype=np.float64)/1000
    s = np.asarray(target_mrad_s, dtype=np.float64)/1000
    center = predict_float32(p, s, model).astype(float)
    # Signed integer conversion truncates, so +/-1 unit is conservative.
    # sin/cos are globally h-Lipschitz in mechanical angle.
    rpm = s*60/(2*np.pi)
    x = np.column_stack([rpm/40, np.sign(rpm)] +
                        [f(h*p) for h in HARMONICS for f in (np.sin, np.cos)])
    uncertainty = np.broadcast_to(
        np.r_[.001*60/(2*np.pi*40), 0, np.repeat(HARMONICS*.001, 2)],
        x.shape).copy()
    uncertainty[s == 0, 1] = 1  # Unknown sign at a truncated zero target.
    low = (x-uncertainty-model['input_mean'])/model['input_std']
    high = (x+uncertainty-model['input_mean'])/model['input_std']
    weights = model['hidden_weights'].astype(float)
    positive, negative = np.maximum(weights, 0), np.minimum(weights, 0)
    hidden_low = np.maximum(low@positive.T+high@negative.T+model['hidden_bias'], 0)
    hidden_high = np.maximum(high@positive.T+low@negative.T+model['hidden_bias'], 0)
    output_positive = np.maximum(model['output_weights'], 0)
    output_negative = np.minimum(model['output_weights'], 0)
    # 1 mV output quantization plus floating-point/trig implementation allowance.
    lower = hidden_low@output_positive+hidden_high@output_negative+model['output_bias']-.00102
    upper = hidden_high@output_positive+hidden_low@output_negative+model['output_bias']+.00102
    return center, lower, upper


def verify(rows, model, phase_rad=0):
    rows = [r for r in rows if int(r['nn_mode']) in (5, 6)]
    if not rows:
        raise ValueError('no angle-shadow frames (nn_mode=5)')
    if any(int(r['fault']) or not int(r['sensor_ok']) or not int(r['calibrated']) for r in rows):
        raise ValueError('unhealthy angle-shadow recording')
    expected, low, high = prediction_interval(
        [int(r['position_mrad'])+phase_rad*1000 for r in rows],
        [int(r['target_speed_mrad_s']) for r in rows], model)
    observed = np.array([int(r['nn_delta_uq_mv'])/1000 for r in rows])
    changes = [int(r['uq_mv'])-int(r['base_uq_mv']) for r in rows if not int(r['clipped'])]
    if not changes:
        raise ValueError('need unclipped frames to test non-actuation')
    violations = int(np.count_nonzero((observed < low) | (observed > high)))
    application_violations = 0
    for i, r in enumerate(rows):
        if int(r['clipped']):
            continue
        applied = (int(r['uq_mv'])-int(r['base_uq_mv']))/1000
        alpha = min(int(r['nn_alpha_milli'])/1000, .25) if int(r['nn_mode']) == 6 else 0
        # Two separately truncated output voltages contribute <2 mV uncertainty.
        lower = alpha*np.clip(low[i], -.08, .08)-.0021
        upper = alpha*np.clip(high[i], -.08, .08)+.0021
        application_violations += int(not lower <= applied <= upper)
        if int(r['nn_mode']) == 5 and applied != 0:
            application_violations += 1
    report = dict(samples=len(rows), prediction_interval_violations=violations,
                  max_point_prediction_difference_v=float(np.max(np.abs(expected-observed))),
                  max_interval_width_v=float(np.max(high-low)),
                  max_unclipped_output_change_mv=max(map(abs, changes)),
                  application_interval_violations=application_violations,
                  phase_rad=phase_rad,
                  passed=violations == 0 and application_violations == 0,
                  limitation='Quantized UART prediction/application interval agreement; '
                             'not full-precision hardware parity, timing or control improvement.')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--model', required=True)
    parser.add_argument('--phase-mrad', type=int, default=0)
    args = parser.parse_args()
    source = Path(args.input)
    with source.open(newline='', encoding='utf-8') as stream:
        report = verify(list(csv.DictReader(stream)), load_angle_model(args.model), args.phase_mrad/1000)
    output = source.with_name(source.stem+'_angle_parity.json')
    if output.exists():
        raise FileExistsError(output)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit('angle shadow verification failed')


if __name__ == '__main__':
    main()
