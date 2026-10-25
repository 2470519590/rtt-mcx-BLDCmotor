"""Aggregate independent repetitions; no serial or hardware access."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--candidate', default='scheduled')
    parser.add_argument('--baseline', default='baseline')
    args = parser.parse_args()
    source = Path(args.input)
    results = json.loads(source.with_suffix('.json').read_text(encoding='utf-8'))
    metrics = ['rmse_rpm', 'iae_rpm_s', 'tail_mae_rpm', 'raw_tail_rmse_rpm']
    summary = []
    for target in sorted({r['target_rpm'] for r in results}):
        row = dict(target_rpm=target)
        for name in [args.baseline, args.candidate]:
            selected = [r for r in results if r['config'] == name and r['target_rpm'] == target]
            if not selected:
                raise ValueError(f'no samples for {name}, {target} RPM')
            row[name] = {metric: float(np.mean([r[metric] for r in selected])) for metric in metrics}
            row[name]['repeats'] = len(selected)
        row['rmse_reduction_pct'] = 100*(1-row[args.candidate]['rmse_rpm']/row[args.baseline]['rmse_rpm'])
        row['raw_tail_rmse_reduction_pct'] = 100*(1-row[args.candidate]['raw_tail_rmse_rpm']/row[args.baseline]['raw_tail_rmse_rpm'])
        summary.append(row)
    suffix = '_summary' if args.candidate == 'scheduled' else '_'+args.candidate+'_summary'
    if args.baseline != 'baseline':
        suffix = '_'+args.candidate+'_vs_'+args.baseline+'_summary'
    source.with_name(source.stem+suffix+'.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    with source.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    targets = sorted({r['target_rpm'] for r in results}, key=lambda x:(abs(x), -x))
    height = (len(targets)+1)//2
    fig, axes = plt.subplots(height, 2, figsize=(12, 3.3*height), constrained_layout=True, squeeze=False)
    for ax, target in zip(axes.flat, targets):
        for name,color in [(args.baseline, 'tab:gray'), (args.candidate, 'tab:blue')]:
            traces = []
            for repeat in sorted({r['repeat'] for r in rows}):
                samples = [r for r in rows if r['config'] == name and int(r['target_rpm']) == target and r['repeat'] == repeat]
                if not samples:
                    continue
                t = np.array([int(r['tick_ms']) for r in samples])
                t = (t-t[0])/1000
                speed = np.array([int(r['speed_mrad_s']) for r in samples])*60/(2000*np.pi)
                traces.append((t, speed))
            grid = np.arange(0, min(t[-1] for t,_ in traces), .01)
            traces = np.array([np.interp(grid,t,speed) for t,speed in traces])
            mean = traces.mean(axis=0); std = traces.std(axis=0)
            ax.plot(grid,mean,color=color,label=name)
            ax.fill_between(grid,mean-std,mean+std,color=color,alpha=.15)
        ax.axhline(target,color='black',linestyle='--',linewidth=.8)
        ax.set(title=f'{target:+} RPM: mean +/- one run standard deviation', xlabel='Time after first captured sample (s)',ylabel='Reported speed (RPM)')
        ax.grid(alpha=.2); ax.legend()
    for ax in axes.flat[len(targets):]:
        ax.set_visible(False)
    fig.savefig(source.with_name(source.stem+suffix+'.png'),dpi=150)
    plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    main()
