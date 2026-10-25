"""Dynamic PI-lookahead candidate. This is not an optimal-voltage teacher."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from nn_model import train_mlp
from export_nn_weights import c_float, array_1d, array_2d


def features(target_mrad_s, speed_mrad_s):
    target = np.asarray(target_mrad_s, dtype=np.float32) / np.float32(1000)
    speed = np.asarray(speed_mrad_s, dtype=np.float32) / np.float32(1000)
    scale = np.float32(1.57079632679)  # 15 RPM
    return np.column_stack((np.clip(target/scale, -2, 2),
                            np.clip(speed/scale, -4, 4),
                            np.where(target*speed < 0, 1, 0))).astype(np.float32)


def prepare(rows, horizon_ms=250):
    x, y, repeats = [], [], []
    keys = dict.fromkeys((r['config'], int(r['repeat']), int(r['step'])) for r in rows)
    for config, repeat, step in keys:
        if config != 'friction':
            continue
        group = [r for r in rows if r['config'] == config and
                 int(r['repeat']) == repeat and int(r['step']) == step]
        ticks = np.array([int(r['tick_ms']) for r in group], dtype=np.int64)
        if np.any(np.diff(ticks) <= 0) or np.max(np.diff(ticks)) > 30:
            raise ValueError('invalid dynamic training timebase')
        target = np.array([int(r['target_speed_mrad_s']) for r in group])
        speed = np.array([int(r['speed_mrad_s']) for r in group])
        kp = np.array([int(r['speed_kp_milli']) for r in group])/1000
        # Reconstruct the existing PI integral from measured baseline output.
        # Learned label is its FUTURE CHANGE, not the integral itself, so it
        # cannot simply double the present integral. Still teacher-dependent.
        integral = (np.array([int(r['base_uq_mv']) for r in group])/1000 -
                    target/1000/44.18 - kp*(target-speed)/1000)
        future = np.searchsorted(ticks, ticks+horizon_ms)
        valid = future < len(ticks)
        if any(int(r['fault']) or int(r['clipped']) or not int(r['sensor_ok'])
               or not int(r['calibrated']) for r in group):
            raise ValueError('unhealthy training trajectory')
        current = np.flatnonzero(valid)
        x.extend(features(target[current], speed[current]))
        y.extend(np.clip(integral[future[current]]-integral[current], -.03, .03))
        repeats.extend([repeat]*len(current))
    if not x:
        raise ValueError('no friction trajectory samples')
    return np.array(x, dtype=np.float32), np.array(y, dtype=np.float32), np.array(repeats)


def predict(model, x):
    return np.clip(np.maximum(x @ model['w1'].T + model['b1'], 0) @ model['w2'] +
                   model['b2'], -.03, .03)


def render_header(model):
    return '''#ifndef DYNAMIC_RESIDUAL_MODEL_H
#define DYNAMIC_RESIDUAL_MODEL_H
/* Generated PI-lookahead candidate; keep separate from legacy nn_model.h. */
#define DYNAMIC_INPUTS 3U
#define DYNAMIC_HIDDEN 8U
static const float dynamic_w1[8][3] = {
''' + array_2d(model['w1']) + '''
};
static const float dynamic_b1[8] = {''' + array_1d(model['b1']) + '''};
static const float dynamic_w2[8] = {''' + array_1d(model['w2']) + '''};
static const float dynamic_b2 = ''' + c_float(model['b2']) + ''';
static const float dynamic_linear[4] = {''' + array_1d(model['linear']) + '''};
#endif
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--output', required=True)
    parser.add_argument('--header')
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists() or output.with_suffix('.json').exists():
        raise FileExistsError(output)
    with Path(args.input).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    x,y,repeat = prepare(rows)
    masks = [repeat == i for i in (0,1,2)]
    if not all(np.sum(m) >= 1000 for m in masks):
        raise ValueError('need three whole repetitions for train/validation/test')
    scale = max(float(np.std(y[masks[0]])), .001)
    candidates = []
    for epochs in (200,800):
        model = train_mlp(x[masks[0]],y[masks[0]]/scale,epochs=epochs,seed=641)
        model['w2'] *= scale
        model['b2'] *= scale
        candidates.append((float(np.mean((predict(model,x[masks[1]])-y[masks[1]])**2)),
                           epochs,model))
    _,epochs,model = min(candidates,key=lambda v:v[0])
    linear_x = np.column_stack((x,np.ones(len(x))))
    linear = np.linalg.lstsq(linear_x[masks[0]],y[masks[0]],rcond=None)[0]
    model['linear'] = linear.astype(np.float32)
    report = dict(schema='dynamic_pi_lookahead_v1',selected_epochs=epochs,
                  horizon_ms=250,output_limit_v=.03,
                  input_features=['target_rad_s/1.5707963','actual_rad_s/1.5707963',
                                  'target_times_actual_negative'],
                  teacher='existing PI integral(t+250ms)-integral(t), clipped +/-30mV',
                  split='repeat0 train; repeat1 chooses epochs; repeat2 held-out',
                  warning='PI imitation/lookahead, NOT optimal plant voltage. '
                          'Offline prediction does not establish control improvement.',
                  splits={})
    for name,mask in zip(('train','validation','test'),masks):
        report['splits'][name] = dict(samples=int(sum(mask)),
            nn_rmse_v=float(np.sqrt(np.mean((predict(model,x[mask])-y[mask])**2))),
            linear_rmse_v=float(np.sqrt(np.mean((np.clip(linear_x[mask]@linear,-.03,.03)-y[mask])**2))),
            zero_rmse_v=float(np.sqrt(np.mean(y[mask]**2))))
    np.savez(output,schema='dynamic_pi_lookahead_v1',**model)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    if args.header:
        Path(args.header).write_text(render_header(model),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
