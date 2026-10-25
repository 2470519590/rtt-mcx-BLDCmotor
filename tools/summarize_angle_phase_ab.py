"""Summarize the angle-phase A/B, keeping the controls separate from the gain.

Rules fixed before the run:
* 'zero_phase_control' must match 'baseline' within its own repeat-to-repeat
  spread, otherwise the upload path itself changed the machine and the
  comparison is invalid.
* A phase configuration only counts as an improvement if it beats baseline on
  every mean metric at every tested speed, with no clipping.
* Label-fit improvements (RMSE of the voltage residual) are not counted here;
  this script only reads closed-loop tracking metrics.
"""
import argparse
import json
from pathlib import Path

import numpy as np

METRICS = ('rmse_rpm', 'iae', 'ss_rmse_rpm', 'ss_iae', 'ss_tail_std', 'tail_mae',
           'tail_std', 'displacement_tail_rmse')
# Promotion decision uses the steady-state metrics plus the displacement proxy;
# whole-record RMSE/IAE are reported but not decisive (a larger startup
# excursion would otherwise dominate and hide a steadier loop).
DECISIVE = ('ss_rmse_rpm', 'ss_iae', 'ss_tail_std', 'tail_mae', 'tail_std',
            'displacement_tail_rmse')


def summarize(report_path):
    rows = json.loads(Path(report_path).read_text(encoding='utf-8'))
    available = [m for m in METRICS if m in rows[0]]
    configs = sorted({r['config'] for r in rows})
    targets = sorted({r['target_rpm'] for r in rows})
    out = dict(runs=len(rows), configs=configs, targets=targets,
               metrics=available, decisive_metrics=[m for m in DECISIVE if m in available],
               points=[], controls={})
    for target in targets:
        group = {}
        for config in configs:
            trials = [r for r in rows if r['config'] == config and r['target_rpm'] == target]
            if not trials:
                continue
            group[config] = {m: dict(mean=float(np.mean([t[m] for t in trials])),
                                     std=float(np.std([t[m] for t in trials])),
                                     repeats=len(trials)) for m in available}
            group[config]['clipped'] = int(sum(t['clipped'] for t in trials))
        for config in configs:
            if config not in group:
                continue
            entry = dict(target_rpm=target, config=config, **group[config])
            base = group.get('baseline')
            if base and config != 'baseline':
                entry['reduction_pct'] = {
                    m: 100 * (1 - group[config][m]['mean'] / base[m]['mean']) for m in available}
            out['points'].append(entry)
        if 'baseline' in group and 'zero_phase_control' in group:
            control = {}
            for m in available:
                control[m] = 100 * (1 - group['zero_phase_control'][m]['mean']
                                    / base[m]['mean'])
            control['clipped'] = group['zero_phase_control']['clipped'] - base['clipped']
            out['controls'][str(target)] = control
    return out


def verdict(summary):
    """Promotion rule, evaluated exactly as documented above."""
    decisive = summary['decisive_metrics']
    results = {}
    for target in summary['targets']:
        for config in summary['configs']:
            points = [p for p in summary['points']
                      if p['target_rpm'] == target and p['config'] == config]
            if not points or 'reduction_pct' not in points[0]:
                continue
            reduction = points[0]['reduction_pct']
            results.setdefault(config, {})[target] = dict(
                all_decisive_better=all(reduction[m] > 0 for m in decisive),
                all_metrics_better=all(v > 0 for v in reduction.values()),
                clipped=points[0]['clipped'])
    promoted = {c: all(v['all_decisive_better'] and v['clipped'] == 0 for v in per.values())
                for c, per in results.items()}
    # The zero-phase control shares the upload path and the G6 gating flag with
    # the real tables, so any improvement it shows is path overhead, not phase.
    control = summary['controls']
    control_ok = all(abs(v[m]) < 5 for v in control.values() for m in decisive)
    path_effect = {target: {m: control[target][m] for m in decisive} for target in control}
    return dict(per_config=promoted,
                control_matches_baseline=bool(control_ok),
                control_effect_pct=path_effect)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    args = parser.parse_args()
    source = Path(args.input).with_suffix('.json')
    summary = summarize(source)
    summary['verdict'] = verdict(summary)
    output = source.with_name(source.stem + '_summary.json')
    if not output.exists():
        output.write_text(json.dumps(summary, indent=2), encoding='utf-8')
    for point in summary['points']:
        reduction = point.get('reduction_pct', {})
        text = ' '.join(f'{m}={point[m]["mean"]:.3f}' for m in summary['decisive_metrics'])
        delta = ' '.join(f'{m}:{v:+.1f}%' for m, v in reduction.items()
                         if m in summary['decisive_metrics'])
        print(f"{point['target_rpm']:+4d} {point['config']:26s} {text}  {delta}")
    print('\ncontrols vs baseline (%)')
    for target, values in summary['controls'].items():
        print(f'  {int(target):+4d} ' + ' '.join(f'{m}:{v:+.2f}' for m, v in values.items()))
    print('\nverdict', json.dumps(summary['verdict'], indent=2))
    print(output)


if __name__ == '__main__':
    main()
