"""Summarize the corrected-model confirmation, with both controls.

The decisive question is not "does G6 beat G0" (the G6 path itself moves the
metrics); it is whether the learned angle-dependent AC term beats a bias-only
network on the same path. The bias-only control keeps the DC and the path
identical and removes only the learned angle dependence.
"""
import json
import sys
from pathlib import Path

import numpy as np

source = Path(__file__).resolve().parents[1] / 'data/angle_model_confirm_20261021.json'
rows = json.loads(source.read_text(encoding='utf-8'))
METRICS = ('ss_rmse_rpm', 'ss_iae', 'ss_tail_std', 'tail_mae',
           'displacement_tail_rmse', 'uq_std_mv')
configs = ['baseline', 'angle025_applied', 'zero_upload_control']
targets = sorted({r['target_rpm'] for r in rows})

print(f"{'target':>6} {'config':>20} " + ' '.join(f'{m[:18]:>18s}' for m in METRICS))
means = {}
for target in targets:
    for config in configs:
        trials = [r for r in rows if r['config'] == config and r['target_rpm'] == target]
        means[(target, config)] = {m: float(np.mean([t[m] for t in trials])) for m in METRICS}
        spread = {m: float(np.std([t[m] for t in trials])) for m in METRICS}
        print(f'{target:>6} {config:>20} ' + ' '.join(f'{means[(target, config)][m]:>18.3f}'
                                                      for m in METRICS))

print('\npaired differences (%)')
print('  A: angle025_applied vs baseline          (G6 path + DC + learned AC)')
print('  B: zero_upload_control vs baseline       (G6 path + DC only)')
print('  C: angle025_applied vs zero_upload_control (learned AC only)')
summary = {}
for target in targets:
    base = means[(target, 'baseline')]
    applied = means[(target, 'angle025_applied')]
    control = means[(target, 'zero_upload_control')]
    summary[target] = {
        'A_g6_full_vs_baseline': {m: 100 * (1 - applied[m] / base[m]) for m in METRICS},
        'B_zero_network_vs_baseline': {m: 100 * (1 - control[m] / base[m]) for m in METRICS},
        'C_learned_ac_vs_zero_network': {m: 100 * (1 - applied[m] / control[m]) for m in METRICS},
    }
    for key in ('A_g6_full_vs_baseline', 'B_zero_network_vs_baseline',
                'C_learned_ac_vs_zero_network'):
        values = summary[target][key]
        print(f'  {target:+4d} {key:32s} ' +
              ' '.join(f'{m}:{v:+.1f}' for m, v in values.items()))

decisive = ('ss_rmse_rpm', 'ss_iae', 'ss_tail_std', 'tail_mae', 'displacement_tail_rmse')
learned_gain = all(summary[t]['C_learned_ac_vs_zero_network'][m] > 0
                   for t in targets for m in decisive)
clipped = sum(r['clipped'] for r in rows)
report = dict(source=str(source), means={f'{t}|{c}': v for (t, c), v in means.items()},
              differences={str(t): v for t, v in summary.items()},
              learned_angle_term_improves_all_decisive_metrics=bool(learned_gain),
              path_effect_present=any(abs(summary[t]['B_zero_network_vs_baseline'][m]) > 3
                                      for t in targets for m in decisive),
              clipped=clipped,
              limitation='Single calibration, 3 repeats, +/-15 RPM only. A path effect '
                         'means G6-vs-G0 comparisons cannot be read as compensation value.')
out = source.with_name(source.stem + '_summary.json')
if not out.exists():
    out.write_text(json.dumps(report, indent=2), encoding='utf-8')
print('\nlearned angle term improves every decisive metric at both targets:',
      report['learned_angle_term_improves_all_decisive_metrics'])
print('G6 path itself moves metrics by >3%:', report['path_effect_present'])
print('clipped frames', clipped)
print(out)
