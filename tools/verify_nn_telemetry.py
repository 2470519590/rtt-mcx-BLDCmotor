"""Compare actual MCU NN telemetry against the uploaded ff1 float32 reference."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from motor_feedforward import load_weights, predict_float32


def verify(rows, weights):
    rows = [r for r in rows if int(r['nn_mode']) in (3,4)]
    if not rows:
        raise ValueError('no feedforward telemetry')
    rpm = np.array([int(r['target_speed_mrad_s']) for r in rows])*60/(2000*np.pi)
    expected = predict_float32(rpm, weights)
    observed = np.array([int(r['nn_delta_uq_mv'])/1000 for r in rows])
    delta = np.abs(expected-observed)
    applied = np.array([int(r['uq_mv'])-int(r['base_uq_mv']) for r in rows])/1000
    wanted = np.array([int(r['nn_alpha_milli'])/1000 if int(r['nn_mode'])==3 else 0 for r in rows])*expected
    unclipped = np.array([not int(r['clipped']) for r in rows])
    max_application_error = float(np.max(np.abs(applied[unclipped]-wanted[unclipped]))) if np.any(unclipped) else None
    # Target speed and output each quantized to 1 mrad/s and 1 mV.
    return dict(samples=len(rows), max_prediction_error_v=float(delta.max()),
                max_application_error_v=max_application_error,
                within_telemetry_precision=bool(delta.max()<.0011 and
                    max_application_error is not None and max_application_error<.0021))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input');parser.add_argument('--model',required=True)
    args=parser.parse_args()
    source=Path(args.input)
    with source.open(newline='',encoding='utf-8') as stream:
        report=verify(list(csv.DictReader(stream)),load_weights(args.model))
    source.with_name(source.stem+'_nn_parity.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
    if not report['within_telemetry_precision']:
        raise SystemExit('NN telemetry mismatch')


if __name__=='__main__':
    main()
