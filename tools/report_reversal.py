"""Offline reversal metrics; preserve per-transition results and raw continuity."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from optimize_motor import score
from summarize_estimator_study import common_window_rmse
from uart_evidence import replay_capture
from nn_dataset import parse_learning


def transition_metrics(rows, target, previous):
    ticks = np.array([int(r['tick_ms']) for r in rows], dtype=np.int64)
    if len(ticks) < 200 or np.any(np.diff(ticks) <= 0):
        raise ValueError('incomplete or nonmonotonic transition')
    normalized = [{key:int(value) for key,value in r.items()
                   if key not in ('config',)} for r in rows]
    result = score(normalized, target)
    t = (ticks-ticks[0])/1000
    speed = np.array([int(r['speed_mrad_s']) for r in rows])*60/(2000*np.pi)
    error = speed-target
    result['first_second_rmse_rpm'] = float(np.sqrt(np.mean(error[t <= 1]**2)))
    result['first_250ms_rmse_rpm'] = float(np.sqrt(np.mean(error[t <= .25]**2)))
    result['first_250ms_peak_overshoot_rpm'] = float(
        max(0, np.max(speed[t <= .25] * np.sign(target)) - abs(target)))
    result['mean_last_second_rpm'] = float(np.mean(speed[t >= t[-1]-1]))
    for window in (20, 40, 100):
        result[f'common_{window}ms_tail_rmse_rpm'] = common_window_rmse(rows, target, window)
    # Require 50 ms continuously in the new direction, not a single noisy sample.
    crossing = None
    if previous*target < 0:
        directed = speed*np.sign(target)
        for i in range(len(rows)):
            end = np.searchsorted(ticks, ticks[i]+50)
            if end < len(rows) and np.all(directed[i:end+1] > 0):
                crossing = float(t[i])
                break
    result['new_direction_held_50ms_s'] = crossing
    result['kind'] = ('start' if previous == 0 else 'reversal' if previous*target < 0
                      else 'same_direction_step')
    return result


def reference_config(configs):
    for name in ('reference_0', 'integral_reset', 'baseline', 'friction'):
        if name in configs:
            return name
    raise ValueError('missing named reference configuration')


def aggregate_comparison(summary, metrics):
    configs = list(summary[0]['configurations'])
    reference = reference_config(configs)
    aggregate = {}
    for config in configs:
        aggregate[config] = {}
        for metric in metrics:
            pairs = [(s['configurations'][reference][metric]['mean'],
                      s['configurations'][config][metric]['mean']) for s in summary]
            if any(a is None or b is None for a, b in pairs):
                continue
            a, b = np.asarray(pairs).T
            aggregate[config][metric] = dict(
                mean_of_transition_means=float(b.mean()),
                reduction_percent=float(100 * (1 - b.mean()/a.mean())) if a.mean() else None,
                improved_transitions=int(np.sum(b < a)),
                total_transitions=len(pairs))
    return dict(reference=reference, configurations=aggregate,
                note='Equal transition weights, not pooled RMSE or statistical significance.')


def audit_leading_samples(rows, lines):
    """Catch ACK waiting consuming the earliest new-target L frames."""
    frames = [parse_learning(line['data'].decode('ascii')) for line in lines
              if line['data'].startswith(b'L,')]
    index = {r['sequence']: i for i, r in enumerate(frames)}
    first = {}
    for row in rows:
        first.setdefault((row['config'], row['repeat'], row['step']), row)
    omitted = []
    for key, row in first.items():
        i = index[int(row['sequence'])]
        j = i
        while (j > 0 and frames[j-1]['mode'] == 2
               and frames[j-1]['target_speed_mrad_s'] == frames[i]['target_speed_mrad_s']
               and frames[j-1]['nn_mode'] == frames[i]['nn_mode']):
            j -= 1
        omitted.append(dict(config=key[0], repeat=int(key[1]), step=int(key[2]),
                            leading_frames_omitted=i-j))
    return omitted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    args = parser.parse_args()
    source = Path(args.input)
    meta = json.loads(source.with_suffix('.json').read_text(encoding='utf-8'))
    if meta['failure'] or meta['cleanup_errors']:
        raise ValueError('collection failed or cleanup unconfirmed')
    with source.open(newline='', encoding='utf-8') as stream:
        raw = list(csv.DictReader(stream))
    results = []
    for event in meta['events']:
        rows = [r for r in raw if r['config'] == event['config']
                and int(r['repeat']) == event['repeat'] and int(r['step']) == event['step']]
        results.append(dict(config=event['config'], repeat=event['repeat'], step=event['step'],
                            previous_target_rpm=event['previous_target_rpm'],
                            target_rpm=event['target_rpm'],
                            **transition_metrics(rows, event['target_rpm'],
                                                 event['previous_target_rpm'])))
    metrics = ('rmse_rpm','first_second_rmse_rpm','first_250ms_rmse_rpm',
               'first_250ms_peak_overshoot_rpm',
               'tail_mae_rpm','tail_std_rpm',
               'common_100ms_tail_rmse_rpm','new_direction_held_50ms_s')
    summary = []
    configs = list(dict.fromkeys(r['config'] for r in results))
    for step in sorted({r['step'] for r in results}):
        groups = {c:[r for r in results if r['config'] == c and r['step'] == step]
                  for c in configs}
        if any(len(g) != meta['arguments']['repeats'] for g in groups.values()):
            raise ValueError('unmatched repetitions')
        row = dict(step=step, previous_target_rpm=groups[configs[0]][0]['previous_target_rpm'],
                   target_rpm=groups[configs[0]][0]['target_rpm'],
                   kind=groups[configs[0]][0]['kind'], configurations={})
        for config, group in groups.items():
            row['configurations'][config] = {
                m:dict(mean=float(np.mean([r[m] for r in group])) if all(r[m] is not None
                                                                        for r in group) else None,
                       completed=sum(r[m] is not None for r in group))
                for m in metrics}
        summary.append(row)
    replay = replay_capture(source.with_name(source.stem+'_uart'))
    frames = [line['data'].split(b',') for line in replay.lines
              if line['data'].startswith((b'T,', b'L,'))]
    seq = [int(f[1]) for f in frames]
    nonunit = sum(((b-a)&0xffffffff) != 1 for a,b in zip(seq,seq[1:]))
    leading = audit_leading_samples(raw, replay.lines)
    report = dict(summary=summary, aggregate=aggregate_comparison(summary, metrics),
                  trials=results, raw_telemetry_frames=len(seq),
                  leading_sample_audit=leading,
                  nonunit_seq_steps=nonunit, final_state=meta['final_state'],
                  timing=meta.get('timing'),
                  note=('R resets integral; P-reference filter comparison. '
                        if meta['arguments'].get('reference_study') is not None else
                        'R reset/keep policy comparison. ' if meta['arguments'].get('transition_study')
                        else 'R resets integral every transition. ') +
                       'Time zero is first captured new-target '
                       'sample; direction metric uses feedback, not external speed truth.')
    output = source.with_name(source.stem+'_report.json')
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(5,2,figsize=(12,14),constrained_layout=True)
    for ax, item in zip(axes.flat, summary):
        for config, color in zip(configs, ('tab:orange','gray','tab:blue','tab:green')):
            curves = []
            for repeat in range(meta['arguments']['repeats']):
                rows = [r for r in raw if r['config']==config and int(r['repeat'])==repeat
                        and int(r['step'])==item['step']]
                ticks = np.array([int(r['tick_ms']) for r in rows], dtype=np.int64)
                t = (ticks-ticks[0])/1000
                speed = np.array([int(r['speed_mrad_s']) for r in rows])*60/(2000*np.pi)
                curves.append((t,speed))
            grid=np.arange(0,min(t[-1] for t,_ in curves),.01)
            values=np.array([np.interp(grid,t,s) for t,s in curves])
            ax.plot(grid,values.mean(axis=0),label=config,color=color)
            ax.fill_between(grid,values.mean(axis=0)-values.std(axis=0),
                            values.mean(axis=0)+values.std(axis=0),color=color,alpha=.12)
        ax.axhline(item['target_rpm'],color='black',linestyle='--',linewidth=.8)
        ax.set(title=f"{item['previous_target_rpm']:+} -> {item['target_rpm']:+} RPM",
               xlabel='Time after first sample (s)',ylabel='Reported RPM')
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(source.with_name(source.stem+'_report.png'),dpi=130)
    plt.close(fig)
    for item in summary:
        reference = reference_config(item['configurations'])
        b=item['configurations'][reference]
        for config,c in item['configurations'].items():
            print(item['previous_target_rpm'],'->',item['target_rpm'],config,'vs',reference,
                  'RMSE',round(b['rmse_rpm']['mean'],3),round(c['rmse_rpm']['mean'],3),
                  'reduction %',round(100*(1-c['rmse_rpm']['mean']/b['rmse_rpm']['mean']),2))
    print('FRAMES',len(seq),'NONUNIT',nonunit,'FINAL',meta['final_state'])
    if nonunit:
        raise SystemExit('raw sequence discontinuity')
    if any(r['leading_frames_omitted'] for r in leading):
        raise SystemExit('CSV omits early transition frames; transient conclusions need raw recovery')


if __name__ == '__main__':
    main()
