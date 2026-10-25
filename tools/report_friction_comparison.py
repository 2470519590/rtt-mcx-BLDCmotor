"""Offline three-arm report with same-window encoder proxy and MCU parity."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from controller_dominance import compare
from motor_feedforward import load_weights, predict_float32
from summarize_estimator_study import common_window_rmse
from uart_evidence import replay_capture
from verify_nn_telemetry import verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--friction-model', required=True)
    parser.add_argument('--nn-model', required=True)
    args = parser.parse_args()
    source = Path(args.input)
    with source.open(newline='', encoding='utf-8') as stream:
        raw = list(csv.DictReader(stream))
    results = json.loads(source.with_suffix('.json').read_text(encoding='utf-8'))
    models = dict(baseline=[0]*9, friction=load_weights(args.friction_model),
                  friction_nn=load_weights(args.nn_model))
    for result in results:
        rows = [r for r in raw if r['config'] == result['config'] and
                int(r['repeat']) == result['repeat'] and
                int(r['target_rpm']) == result['target_rpm']]
        for window in (20, 40, 100):
            result[f'common_{window}ms_tail_rmse_rpm'] = common_window_rmse(
                rows, result['target_rpm'], window)
        ticks = np.array([int(r['tick_ms']) for r in rows], dtype=np.int64)
        # Explicit short-window transient metric, separate from the full trace.
        startup = [r for r, tick in zip(rows, ticks) if tick-ticks[0] <= 1000]
        error = np.array([int(r['speed_mrad_s']) for r in startup])*60/(2000*np.pi) - result['target_rpm']
        result['first_second_rmse_rpm'] = float(np.sqrt(np.mean(error**2)))
    parity = {}
    for config, weights in models.items():
        rows = [r for r in raw if r['config'] == config]
        # A G0 baseline in an early screening is not matched-path evidence.
        parity[config] = (verify(rows, weights) if all(int(r['nn_mode']) == 3 for r in rows)
                          else dict(matched_path=False, note='G0 baseline screening only'))
    replay = replay_capture(source.with_name(source.stem + '_uart'))
    frames = []
    malformed = []
    for line in replay.lines:
        fields = line['data'].split(b',')
        if fields[0] in (b'L', b'T'):
            try:
                frames.append(int(fields[1]))
            except (ValueError, IndexError):
                malformed.append(line['start'])
    steps = [(b-a) & 0xffffffff for a, b in zip(frames, frames[1:])]
    metrics = ('rmse_rpm', 'first_second_rmse_rpm', 'tail_mae_rpm', 'tail_std_rpm',
               'common_20ms_tail_rmse_rpm', 'common_40ms_tail_rmse_rpm',
               'common_100ms_tail_rmse_rpm')
    summary = []
    for target in sorted({r['target_rpm'] for r in results}):
        row = dict(target_rpm=target, configurations={})
        for config in models:
            group = [r for r in results if r['config'] == config and r['target_rpm'] == target]
            if not group:
                raise ValueError('missing comparison arm')
            row['configurations'][config] = dict(repeats=len(group), **{
                m:dict(mean=float(np.mean([r[m] for r in group])),
                       run_std=float(np.std([r[m] for r in group]))) for m in metrics})
        summary.append(row)
    comparisons = [compare(results, a, b, 'speed') for a, b in
                   (('baseline', 'friction'), ('baseline', 'friction_nn'), ('friction', 'friction_nn'))]
    rpm_grid = np.linspace(-30, 30, 601)
    report = dict(summary=summary, parity=parity, comparisons=comparisons,
                  matched_path=all(r.get('matched_ff1_path', False) for r in results),
                  max_nn_increment_over_friction_v=float(np.max(np.abs(
                      predict_float32(rpm_grid, models['friction_nn']) -
                      predict_float32(rpm_grid, models['friction'])))),
                  raw_evidence=dict(telemetry_frames=len(frames),
                                    nonunit_seq_steps=sum(s != 1 for s in steps),
                                    malformed_seq_offsets=malformed,
                                    trailing_partial_hex=replay.pending.hex()),
                  limitation='Means/run std only, no statistical significance. '
                             'Encoder-derived windows are not external speed truth. '
                             'Stationary restarts, not reversal validation.')
    destination = source.with_name(source.stem + '_report.json')
    destination.write_text(json.dumps(report, indent=2), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    targets = sorted({r['target_rpm'] for r in results}, key=lambda t:(abs(t), -t))
    fig, axes = plt.subplots((len(targets)+1)//2, 2, figsize=(12, 3.1*((len(targets)+1)//2)),
                             constrained_layout=True, squeeze=False)
    for ax, target in zip(axes.flat, targets):
        for config, color in zip(models, ('gray', 'tab:orange', 'tab:blue')):
            traces = []
            for repeat in sorted({int(r['repeat']) for r in raw}):
                rows = [r for r in raw if r['config'] == config and int(r['repeat']) == repeat
                        and int(r['target_rpm']) == target]
                if not rows:
                    continue
                ticks = np.array([int(r['tick_ms']) for r in rows], dtype=np.int64)
                t = (ticks-ticks[0])/1000
                speed = np.array([int(r['speed_mrad_s']) for r in rows])*60/(2000*np.pi)
                traces.append((t, speed))
            grid = np.arange(0, min(t[-1] for t, _ in traces), .01)
            values = np.array([np.interp(grid, t, speed) for t, speed in traces])
            ax.plot(grid, values.mean(axis=0), color=color, label=config)
            ax.fill_between(grid, values.mean(axis=0)-values.std(axis=0),
                            values.mean(axis=0)+values.std(axis=0), color=color, alpha=.13)
        ax.axhline(target, color='black', linestyle='--', linewidth=.8)
        ax.set(title=f'{target:+} RPM, mean +/- run std', xlabel='Time (s)', ylabel='Reported RPM')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    for ax in axes.flat[len(targets):]:
        ax.set_visible(False)
    fig.savefig(source.with_name(source.stem + '_report.png'), dpi=150)
    plt.close(fig)
    for row in summary:
        values = [row['configurations'][c]['rmse_rpm']['mean'] for c in models]
        print(row['target_rpm'], 'baseline/friction/NN RMSE', *(round(v, 3) for v in values),
              'reductions %', *(round(100*(1-v/values[0]), 2) for v in values[1:]))
    print('MATCHED_PATH', report['matched_path'], 'RAW', report['raw_evidence'])
    print('PARITY', parity)
    print('MAX_NN_INCREMENT_V', report['max_nn_increment_over_friction_v'])
    print(destination)
    if any(p.get('within_telemetry_precision') is False for p in parity.values()):
        raise SystemExit('MCU parity failed')
    if report['raw_evidence']['nonunit_seq_steps'] or malformed:
        raise SystemExit('telemetry sequence continuity failed')


if __name__ == '__main__':
    main()
