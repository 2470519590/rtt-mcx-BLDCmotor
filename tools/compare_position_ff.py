"""Repeated relative position steps using one existing encoder calibration."""
import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import serial

from collect_nn_dataset import tx, send_checked, read_state
from motor_port import resolve_port
from motor_feedforward import load_weights, model_identity
from nn_dataset import LEARNING_FIELDS, parse_learning


def comparison_configs(alphas):
    values=list(alphas)
    if (not values or any(not math.isfinite(a) or not 0<a<=1 for a in values) or
            len({round(a*1000) for a in values}) != len(values)):
        raise ValueError('alphas must be distinct and within (0,1] at milli resolution')
    return {'baseline':0., **{f'ff{round(a*1000):04d}':a for a in values}}


def gain_configs(configs, gains):
    values=list(gains)
    if (not values or any(not math.isfinite(g) or not 0<g<=20 for g in values) or
            len({round(g*1000) for g in values}) != len(values)):
        raise ValueError('position gains must be distinct and within (0,20] at milli resolution')
    return {f'pos{round(g*1000):05d}_{name}':(alpha,g)
            for g in values for name,alpha in configs.items()}


def step_magnitudes(values):
    values = list(values)
    if (not values or any(not math.isfinite(d) or not 1 <= d <= 30 for d in values)
            or len(set(values)) != len(values)):
        raise ValueError('step magnitudes must be distinct, finite, within 1..30 degrees')
    return values


def summarize(samples, target):
    ticks = np.array([r['tick_ms'] for r in samples], dtype=np.int64)
    t = (ticks-ticks[0])/1000
    error = (np.array([r['position_mrad'] for r in samples])-target)*180/(1000*math.pi)
    tail = t >= t[-1]-.8
    outside = np.flatnonzero(np.abs(error) > .5)
    settled = None if len(outside) and outside[-1] == len(t)-1 else float(t[outside[-1]+1]) if len(outside) else 0.
    direction = np.sign(target-samples[0]['position_mrad'])
    return dict(rmse_deg=float(np.sqrt(np.mean(error**2))),
                tail_mae_deg=float(np.mean(np.abs(error[tail]))),
                tail_std_deg=float(np.std(error[tail])),
                overshoot_deg=float(max(0., np.max(error*direction))),
                iae_deg_s=float(np.sum((abs(error[1:])+abs(error[:-1]))*.5*np.diff(t))),
                settled_0p5deg_s=settled, peak_error_deg=float(np.max(abs(error))),
                clipped=sum(r['clipped'] for r in samples))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--ff-model', required=True)
    parser.add_argument('--port', default='auto')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--alpha', type=float, default=1)
    parser.add_argument('--alphas', type=float, nargs='+',
                        help='Compare several NN strengths in the same session')
    parser.add_argument('--speed-ki', type=int, default=284, help='Firmware milli-units; same for both modes')
    parser.add_argument('--speed-kp', type=int, default=8, help='Firmware milli-units; same for both modes')
    parser.add_argument('--position-kp', type=float, default=8, help='Outer loop gain; same for both modes')
    parser.add_argument('--position-kps', type=float, nargs='+',
                        help='Compare outer gains in one randomized session, each with NN off/on')
    parser.add_argument('--step-deg', type=float, nargs='+', default=[10],
                        help='Symmetric relative step magnitudes, e.g. 10 20')
    parser.add_argument('--seed', type=int, default=54)
    parser.add_argument('--origin-offset-deg', type=float, default=0,
                        help='Shift test origin from current position without resetting encoder')
    args = parser.parse_args()
    alphas = args.alphas if args.alphas is not None else [args.alpha]
    if (not 0 <= args.alpha <= 1 or not 1 <= args.repeats <= 10 or
            not 0<=args.speed_ki<=2000 or not 0<=args.speed_kp<=200 or not 0 < args.position_kp <= 20 or
            not args.step_deg or any(not math.isfinite(d) or not 1<=d<=30 for d in args.step_deg) or
            not math.isfinite(args.origin_offset_deg) or abs(args.origin_offset_deg)>180):
        parser.error('invalid alpha/repeats/gains')
    try:
        args.step_deg = step_magnitudes(args.step_deg)
        configs=comparison_configs(alphas)
    except ValueError as exc:
        parser.error(str(exc))
    output = Path(args.output)
    if any(output.with_suffix(s).exists() for s in ('.csv', '.json', '.png')):
        raise FileExistsError(output)
    weights = load_weights(args.ff_model)
    model_metadata = model_identity(args.ff_model)
    if args.alphas is None:
        configs={'baseline':0., f'ff{round(args.alpha*100):03d}':args.alpha}
    try:
        trials=(gain_configs(configs,args.position_kps) if args.position_kps is not None
                else {name:(alpha,args.position_kp) for name,alpha in configs.items()})
    except ValueError as exc:
        parser.error(str(exc))
    port = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=.5)
    port.port = resolve_port(args.port); port.dtr=False; port.rts=False
    lock = threading.Lock(); done = threading.Event()
    results=[]; traces=[]
    def command(op,value=0):
        send_checked(port,op,value,lock)
    def heartbeat():
        while not done.wait(.2):
            try:
                tx(port,'H',0,lock)
            except Exception:
                done.set()
    def capture(seconds, mode):
        rows=[]; end=time.monotonic()+seconds
        while time.monotonic()<end:
            if done.is_set():
                raise RuntimeError('heartbeat failed')
            line=port.readline().decode('ascii','replace').strip()
            if not line.startswith('L,'):
                continue
            row=parse_learning(line)
            if row['fault'] or not row['sensor_ok'] or not row['calibrated']:
                raise RuntimeError(line)
            if row['mode']==mode:
                rows.append(row)
        if not rows:
            raise RuntimeError('no learning frames')
        return rows
    port.open(); worker=threading.Thread(target=heartbeat,daemon=True); worker.start()
    try:
        command('S'); command('T',0); print('STATE',','.join(read_state(port)),flush=True)
        command('G',0)
        for index,value in enumerate(weights):
            command('f',index);command('w',value)
        command('P',args.speed_kp);command('Q',args.speed_ki);command('Y',60);command('Z',round(args.position_kp*1000));command('T',1)
        # T does not expose unwrapped position. A short baseline speed run supplies L.
        command('R',30); origin=capture(.1,2)[-1]['position_mrad'];command('S')
        origin += round(math.radians(args.origin_offset_deg)*1000)
        if abs(origin)>98000:
            raise RuntimeError('position origin beyond X range')
        with output.open('x',newline='',encoding='utf-8') as stream:
            writer=csv.DictWriter(stream,fieldnames=['config','repeat','step_deg','origin_mrad']+LEARNING_FIELDS)
            writer.writeheader(); rng=np.random.default_rng(args.seed)
            for repeat in range(args.repeats):
                for name in rng.permutation(list(trials)):
                    alpha,position_kp=trials[name]
                    command('S');command('Z',round(position_kp*1000))
                    for degrees in [sign*d for d in args.step_deg for sign in (1,-1)]:
                        # Identical baseline returns and holds for both candidates.
                        command('S');command('G',0);command('X',origin)
                        aligned=capture(2.5,8)
                        # Define each step from actual settled feedback. Baseline
                        # friction can leave a small hold error; do not reject a
                        # usable relative-step experiment because of that error.
                        actual_origin=aligned[-1]['position_mrad']
                        command('S');command('G',0 if alpha==0 else 3)
                        command('K',round(alpha*1000))
                        target=actual_origin+round(math.radians(degrees)*1000)
                        command('X',target); samples=capture(4,8);command('S')
                        for row in samples:
                            writer.writerow(dict(config=name,repeat=repeat,step_deg=degrees,origin_mrad=actual_origin,**row))
                        stream.flush()
                        result=dict(config=name,repeat=repeat,step_deg=degrees,alpha=alpha,
                                    **model_metadata,
                                    speed_ki_milli=args.speed_ki,
                                    speed_kp_milli=args.speed_kp,
                                    position_kp=position_kp,
                                    origin_offset_deg=args.origin_offset_deg,
                                    origin_error_deg=(aligned[-1]['position_mrad']-origin)*180/(1000*math.pi),
                                    **summarize(samples,target))
                        results.append(result);traces.append((result,samples,actual_origin))
                        output.with_suffix('.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
                        print(json.dumps(result),flush=True)
    finally:
        for op,value in [('S',0),('T',0),('G',0),('P',8),('Q',284),('Y',60),('Z',8000)]:
            try:
                command(op,value)
            except Exception:
                pass
        done.set();worker.join(1);port.close()
        if traces:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig,axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
            colors={name:plt.get_cmap('tab10')(index) for index,name in enumerate(trials)}
            if 'baseline' in colors:
                colors['baseline']='tab:gray'
            for result,samples,origin in traces:
                i=0 if result['step_deg']>0 else 1
                t=(np.array([r['tick_ms'] for r in samples])-samples[0]['tick_ms'])/1000
                color=colors[result['config']]
                axes[i,0].plot(t,[(r['position_mrad']-origin)*180/(1000*math.pi) for r in samples],color=color,alpha=.6,label=f"{result['config']} #{result['repeat']}")
                axes[i,1].plot(t,[r['uq_mv']/1000 for r in samples],color=color,alpha=.6)
            for i,sign in enumerate((1,-1)):
                for magnitude in args.step_deg:
                    axes[i,0].axhline(sign*magnitude,color='black',linestyle='--',alpha=.4)
                axes[i,0].set_ylabel('Position (deg)')
                axes[i,1].set_ylabel('Uq (V)')
            for ax in axes.flat:
                ax.set_xlabel('Time after first sample (s)');ax.grid(alpha=.2)
            axes[0,0].legend(fontsize=7)
            fig.savefig(output.with_suffix('.png'),dpi=150);plt.close(fig)


if __name__=='__main__':
    main()
