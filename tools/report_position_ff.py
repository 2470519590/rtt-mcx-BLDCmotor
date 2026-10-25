"""Summarize recorded position trials without opening serial ports."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    args=parser.parse_args()
    source=Path(args.input)
    rows=json.loads(source.with_suffix('.json').read_text(encoding='utf-8'))
    summary=[]
    for step in sorted({r['step_deg'] for r in rows}):
        result=dict(step_deg=step)
        for name in dict.fromkeys(r['config'] for r in rows):
            samples=[r for r in rows if r['step_deg']==step and r['config']==name]
            if not samples:
                continue
            result[name]={metric:float(np.mean([r[metric] for r in samples]))
                          for metric in ('rmse_deg','tail_mae_deg','iae_deg_s')}
            for metric in ('tail_std_deg','overshoot_deg'):
                if all(metric in r for r in samples):
                    result[name][metric]=float(np.mean([r[metric] for r in samples]))
            settled=[r['settled_0p5deg_s'] for r in samples if r['settled_0p5deg_s'] is not None]
            result[name].update(repeats=len(samples),settled_count=len(settled),
                                mean_settled_s=float(np.mean(settled)) if settled else None)
        summary.append(result)
    source.with_name(source.stem+'_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
