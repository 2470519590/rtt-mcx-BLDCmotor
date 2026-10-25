"""Summarize matched estimator trials, keeping physical proxy separate from feedback."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def common_window_rmse(trial, target, window_ms):
    ticks = np.array([int(r['tick_ms']) for r in trial], dtype=np.int64)
    if len(ticks) < 2 or np.any(np.diff(ticks) <= 0):
        raise ValueError('need increasing sample timestamps')
    pos = np.array([int(r['position_mrad']) for r in trial]) / 1000
    velocity = []
    for i in np.flatnonzero(ticks >= ticks[-1] - 800):
        j = np.searchsorted(ticks, ticks[i]-window_ms, side='right') - 1
        if 0 <= j < i and ticks[i]-ticks[j] <= window_ms + 15:
            velocity.append((pos[i]-pos[j])/((ticks[i]-ticks[j])/1000)*60/(2*np.pi))
    if not velocity:
        raise ValueError('insufficient common-window samples')
    return float(np.sqrt(np.mean((np.array(velocity)-target)**2)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--baseline', required=True)
    args = parser.parse_args()
    source = Path(args.input).with_suffix('.json')
    rows = json.loads(source.read_text(encoding='utf-8'))
    # Same time windows for every estimator prevent a slower/smoother feedback
    # signal from winning solely because its own reported speed hides ripple.
    with source.with_suffix('.csv').open(newline='', encoding='utf-8') as stream:
        traces = list(csv.DictReader(stream))
    for row in rows:
        trial = [r for r in traces if r['config'] == row['config'] and
                 int(r['repeat']) == row['repeat'] and int(r['target_rpm']) == row['target_rpm']]
        for window in (20, 40, 100):
            row[f'common_{window}ms_tail_rmse_rpm'] = common_window_rmse(trial, row['target_rpm'], window)
    metrics = ('rmse_rpm', 'tail_mae_rpm', 'tail_std_rpm', 'raw_tail_rmse_rpm')
    metrics += tuple(f'common_{w}ms_tail_rmse_rpm' for w in (20, 40, 100))
    configs = sorted({r['config'] for r in rows})
    targets = sorted({r['target_rpm'] for r in rows})
    groups = {(c, t): [r for r in rows if r['config'] == c and r['target_rpm'] == t]
              for c in configs for t in targets}
    if not all(groups.values()) or len({len(g) for g in groups.values()}) != 1:
        raise ValueError('incomplete or unmatched trials')
    report = {'baseline': args.baseline, 'points': [],
              'limitation': 'Reported speed changes with estimator, so cross-estimator RMSE is not '
                            'an independent physical comparison. raw_tail_rmse uses the same encoder '
                            'with a common 100ms displacement window; not external ground truth. '
                            'SysTick wall cycles include preemption; timing snapshots include coast/stop.'}
    for target in targets:
        base = groups[args.baseline, target]
        for config in configs:
            group = groups[config, target]
            result = dict(config=config, target_rpm=target, repeats=len(group), metrics={})
            for metric in metrics:
                b, c = [float(np.mean([r[metric] for r in g])) for g in (base, group)]
                result['metrics'][metric] = dict(mean=c, baseline=b,
                                                 reduction_pct=100 * (1-c/b) if b else None)
            result['clipped'] = sum(r['clipped'] for r in group)
            result['max_loop_us'] = max(r['timing']['max_loop_us'] for r in group)
            result['max_period_us'] = max(r['timing']['max_period_us'] for r in group)
            result['late_periods'] = sum(r['timing']['late_periods'] for r in group)
            result['over_budget_loops'] = sum(r['timing']['over_budget_loops'] for r in group)
            report['points'].append(result)
    output = source.with_name(source.stem + '_comparison_windows.json')
    if output.exists():
        raise FileExistsError(output)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    for row in report['points']:
        metric = row['metrics']['raw_tail_rmse_rpm']
        print(row['target_rpm'], row['config'], 'raw_tail_rmse', round(metric['mean'], 3),
              'reduction_pct', round(metric['reduction_pct'], 2))
    print(output)


if __name__ == '__main__':
    main()
