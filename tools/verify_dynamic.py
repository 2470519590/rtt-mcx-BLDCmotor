"""Check MCU dynamic prediction AND actual voltage addition from L telemetry."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from dynamic_residual_model import features,predict
from motor_feedforward import load_weights,predict_float32


def verify(rows,model,ff_weights):
    if not rows:
        raise ValueError('no dynamic telemetry')
    if any(int(r['nn_mode']) not in (7,8,9) or int(r['clipped']) or int(r['fault'])
           or not int(r['sensor_ok']) or not int(r['calibrated']) for r in rows):
        raise ValueError('invalid dynamic telemetry')
    target=np.array([int(r['target_speed_mrad_s']) for r in rows])
    speed=np.array([int(r['speed_mrad_s']) for r in rows])
    x=features(target,speed)
    nn=predict(model,x)
    linear=np.clip(np.column_stack((x,np.ones(len(x))))@model['linear'],-.03,.03)
    mode=np.array([int(r['nn_mode']) for r in rows])
    expected=np.where(mode==9,linear,nn)
    observed=np.array([int(r['nn_delta_uq_mv']) for r in rows])/1000
    static=predict_float32(target*60/(2000*np.pi),ff_weights)
    alpha=np.array([int(r['nn_alpha_milli']) for r in rows])/1000
    wanted=static+np.where(mode==7,0,alpha*expected)
    applied=np.array([int(r['uq_mv'])-int(r['base_uq_mv']) for r in rows])/1000
    prediction_error=float(np.max(np.abs(expected-observed)))
    application_error=float(np.max(np.abs(wanted-applied)))
    return dict(samples=len(rows),max_prediction_error_v=prediction_error,
                max_application_error_v=application_error,
                passed=prediction_error < .0012 and application_error < .0022)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--model',required=True)
    parser.add_argument('--friction-model',required=True)
    args=parser.parse_args()
    source=Path(args.input)
    with source.open(newline='',encoding='utf-8') as stream:
        rows=list(csv.DictReader(stream))
    model=dict(np.load(args.model))
    weights=load_weights(args.friction_model)
    report={}
    for config in dict.fromkeys(r['config'] for r in rows):
        report[config]=verify([r for r in rows if r['config']==config],model,
                              [0]*9 if config=='baseline' else weights)
    source.with_name(source.stem+'_dynamic_parity.json').write_text(
        json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
    if not all(r['passed'] for r in report.values()):
        raise SystemExit('dynamic MCU parity failed')


if __name__=='__main__':
    main()
