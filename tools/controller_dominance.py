"""Offline all-metric controller comparison; never opens hardware.

Strict dominance requires every preselected error metric to improve at every
measured target. Missing/nonsettled position trials count as failures, not as
zero settling time. This is descriptive evidence, not statistical significance.
"""
import argparse
import json
import math
from pathlib import Path


SPEED_METRICS = ('rmse_rpm', 'iae_rpm_s', 'tail_mae_rpm',
                 'tail_std_rpm', 'raw_tail_rmse_rpm')
POSITION_METRICS = ('rmse_deg', 'iae_deg_s', 'tail_mae_deg',
                    'tail_std_deg', 'overshoot_deg', 'settled_0p5deg_s')


def compare(rows, baseline, candidate, kind):
    key = 'target_rpm' if kind == 'speed' else 'step_deg'
    metrics = SPEED_METRICS if kind == 'speed' else POSITION_METRICS
    targets = sorted({r[key] for r in rows if r['config'] in (baseline, candidate)})
    if not targets or baseline == candidate:
        raise ValueError('need distinct configurations and nonempty measurements')
    result = []
    for target in targets:
        groups = [[r for r in rows if r['config'] == name and r[key] == target]
                  for name in (baseline, candidate)]
        if not all(groups) or len(groups[0]) != len(groups[1]):
            raise ValueError(f'unmatched trial count at {target}')
        compared = {}
        for metric in metrics:
            values = []
            for group in groups:
                raw = [r[metric] for r in group]
                if any(v is None for v in raw):
                    values.append(None)
                else:
                    if not all(math.isfinite(v) for v in raw):
                        raise ValueError('nonfinite metric')
                    values.append(sum(raw)/len(raw))
            b, c = values
            # Two failed settling groups do not demonstrate equivalence.
            verdict = ('unresolved' if b is None and c is None else
                       'improved' if b is None else
                       'worse' if c is None else
                       'improved' if c < b else 'worse' if c > b else 'equal')
            compared[metric] = dict(baseline=b, candidate=c, verdict=verdict,
                                   reduction_pct=100*(1-c/b) if b and c is not None else None)
        healthy = all(not r.get('clipped', 0) for group in groups for r in group)
        result.append({key:target, 'repeats':len(groups[0]), 'no_clipping':healthy,
                       'metrics':compared,
                       'all_metrics_improved':healthy and all(v['verdict']=='improved'
                                                             for v in compared.values())})
    return dict(baseline=baseline, candidate=candidate, kind=kind, targets=result,
                strict_measured_dominance=all(r['all_metrics_improved'] for r in result),
                limitation='Descriptive mean metrics only; no universal or statistical dominance claim.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--kind', choices=['speed','position'], required=True)
    args=parser.parse_args()
    source=Path(args.input).with_suffix('.json')
    result=compare(json.loads(source.read_text(encoding='utf-8')),
                   args.baseline,args.candidate,args.kind)
    output=source.with_name(source.stem+'_'+args.candidate+'_vs_'+args.baseline+'_dominance.json')
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
