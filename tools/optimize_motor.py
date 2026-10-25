"""Repeated real-board speed steps, with holdout repetitions and raw-angle metric.

Uses the existing calibration; never flashes or calibrates. Records partial data.
"""
import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path

import numpy as np

from collect_nn_dataset import tx, send_checked, read_state
from motor_port import resolve_port
from nn_dataset import parse_learning, LEARNING_FIELDS
from motor_profiles import speed_gains
from motor_feedforward import load_weights, model_identity
from serial_capture import CapturedSerial


CONFIGS = {
    "pi_kp100_ki100_regression": (100, 100, 0, 0),
    "pi_kp100_ki100_observer": (100, 100, 0, 0),
    "pi_kp20_regression": (20, 284, 0, 0),
    "pi_kp20_observer": (20, 284, 0, 0),
    "pi_kp50_ki100_regression": (50, 100, 0, 0),
    "pi_kp50_ki100_observer": (50, 100, 0, 0),
    "pi_kp150_ki100_regression": (150, 100, 0, 0),
    "pi_kp150_ki100_observer": (150, 100, 0, 0),
    "baseline": (8, 284, 0, 0),
    "pi_ki450": (8, 450, 0, 0),
    "pi_ki650": (8, 650, 0, 0),
    "pi_ki900": (8, 900, 0, 0),
    "pi_ki900_ff100": (8, 900, 3, 1000),
    "pi_kp20_ki650": (20, 650, 0, 0),
    "pi_kp20_ki650_ff100": (20, 650, 3, 1000),
    "pi_kp50_ki100": (50, 100, 0, 0),
    "pi_kp50_ki100_ff100": (50, 100, 3, 1000),
    "pi_kp50_ki100_ff050": (50, 100, 3, 500),
    "pi_kp80_ki100": (80, 100, 0, 0),
    "pi_kp80_ki100_ff100": (80, 100, 3, 1000),
    "pi_kp100_ki100": (100, 100, 0, 0),
    "pi_kp100_ki100_ff100": (100, 100, 3, 1000),
    "pi_kp150_ki100": (150, 100, 0, 0),
    "pi_kp150_ki100_ff100": (150, 100, 3, 1000),
    "pi_kp150_ki100_ff050": (150, 100, 3, 500),
    "pi_kp150_ki100_angleshadow": (150, 100, 5, 1000),
    "pi_kp150_ki100_angle010": (150, 100, 6, 100),
    "pi_kp150_ki100_angle025": (150, 100, 6, 250),
    "pi_kp150_ki200": (150, 200, 0, 0),
    "pi_kp150_ki200_ff100": (150, 200, 3, 1000),
    "pi_kp200_ki100": (200, 100, 0, 0),
    "pi_kp200_ki100_ff100": (200, 100, 3, 1000),
    "pi_kp15_ki450": (15, 450, 0, 0),
    "pi_kp20": (20, 284, 0, 0),
    "pi_kp35": (35, 284, 0, 0),
    "pi_kp50": (50, 284, 0, 0),
    "pi_lowki100": (8, 100, 0, 0),
    "pi_lowki150": (8, 150, 0, 0),
    "ff050": (8, 284, 3, 500),
    "ff100": (8, 284, 3, 1000),
    "scheduled": (8, 284, 0, 0),
    "scheduled_ff100": (8, 284, 3, 1000),
    "nn010": (8, 284, 2, 100),
    "nn030": (8, 284, 2, 300),
    "nn060": (8, 284, 2, 600),
}

ESTIMATOR_MODES = {name: 1 if name.endswith('_regression') else 2
                   for name in CONFIGS if name.endswith(('_regression', '_observer'))}
TIMING_FIELDS = ('estimator_mode', 'clock_hz', 'samples', 'period_cycles',
                 'max_period_cycles', 'sensor_cycles', 'max_sensor_cycles',
                 'loop_cycles', 'max_loop_cycles', 'late_periods', 'over_budget_loops',
                 'legacy_mrad_s', 'regression_mrad_s', 'observer_mrad_s')


def three_arm_setup(config, classical_weights, learned_weights):
    """Every arm executes G3/alpha=1, including the zero-output PI control."""
    if config == 'pi_kp150_ki100':
        return 'baseline', [0] * 9
    if config == 'pi_kp150_ki100_ff050':
        return 'friction', classical_weights
    if config == 'pi_kp150_ki100_ff100':
        return 'friction_nn', learned_weights
    raise ValueError('unknown three-arm configuration')


def parse_timing(line):
    fields = line.split(',')
    if fields[0] != 'E' or len(fields) != len(TIMING_FIELDS) + 1:
        raise ValueError('invalid timing frame')
    result = dict(zip(TIMING_FIELDS, map(int, fields[1:])))
    if result['clock_hz'] <= 0 or result['period_cycles'] <= 0 or result['sensor_cycles'] <= 0:
        raise ValueError('cycle counter clock unavailable')
    for name in TIMING_FIELDS:
        if name.endswith('_cycles'):
            result[name.replace('_cycles', '_us')] = result[name] * 1e6 / result['clock_hz']
    return result


def score(samples, target):
    t = np.array([r['tick_ms'] for r in samples], dtype=float)/1000
    t -= t[0]
    speed = np.array([r['speed_mrad_s'] for r in samples])*60/(2000*math.pi)
    pos = np.array([r['position_mrad'] for r in samples])/1000
    error = speed-target
    tail = t >= max(.5, t[-1]-.8)
    # 100 ms displacement speed: separate from the firmware's reported estimator.
    raw_speed = []
    raw_times = []
    for i in range(len(t)):
        j = np.searchsorted(t, t[i]-.1)
        if j < i and t[i]-t[j] >= .08:
            raw_speed.append((pos[i]-pos[j])/(t[i]-t[j])*60/(2*math.pi))
            raw_times.append(t[i])
    raw_speed = np.array(raw_speed)
    raw_times = np.array(raw_times)
    return dict(rmse_rpm=float(np.sqrt(np.mean(error**2))),
                iae_rpm_s=float(np.sum((np.abs(error[1:])+np.abs(error[:-1]))*.5*np.diff(t))),
                tail_mae_rpm=float(np.mean(np.abs(error[tail]))),
                tail_std_rpm=float(np.std(speed[tail])),
                raw_tail_rmse_rpm=float(np.sqrt(np.mean((raw_speed[raw_times >= t[-1]-.8]-target)**2))),
                max_gap_ms=float(np.max(np.diff(t))*1000),
                clipped=int(sum(r['clipped'] for r in samples)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--configs', nargs='+', choices=CONFIGS,
                        default=['baseline', 'pi_ki450', 'pi_ki650', 'nn010', 'nn030'])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--seconds', type=float, default=3)
    parser.add_argument('--targets', nargs='+', type=int, default=[60, -60])
    parser.add_argument('--port', default='auto')
    parser.add_argument('--seed', type=int, default=21)
    parser.add_argument('--ff-model', help='Upload learned RAM output weights before testing')
    parser.add_argument('--friction-model', help='Classical ff1 model for a matched three-arm comparison')
    parser.add_argument('--three-arm', action='store_true',
                        help='fixed same-PI baseline/friction/friction+NN experiment')
    parser.add_argument('--angle-model', help='Compiled shadow model archive, for provenance only; not uploaded')
    parser.add_argument('--angle-phase', type=float, default=0, help='Registered phase in radians, RAM only')
    parser.add_argument('--estimator-study', action='store_true',
                        help='Require nnest1 v/d/e commands; record measured cycle timing')
    parser.add_argument('--auto-calibrate', action='store_true',
                        help='Calibrate once only if a fresh healthy state is uncalibrated')
    args = parser.parse_args()
    if args.repeats < 1 or not 2 <= args.seconds <= 10 or any(not 0 < abs(x) <= 180 for x in args.targets):
        parser.error('use repeats>=1, seconds 2..10, target magnitude 1..180 RPM')
    output = Path(args.output)
    manifest = output.with_name(output.stem + '_manifest.json')
    if (any(output.with_suffix(s).exists() for s in ('.csv', '.json', '.png')) or
            manifest.exists()):
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    weights = None
    model_metadata = {}
    if args.ff_model:
        weights = load_weights(args.ff_model)
        model_metadata = model_identity(args.ff_model)
    friction_weights = None
    if args.three_arm:
        if not args.ff_model or not args.friction_model or args.estimator_study:
            parser.error('--three-arm requires --friction-model and --ff-model, without estimator study')
        if any(abs(t) > 30 for t in args.targets):
            parser.error('initial three-arm study limited to +/-30 RPM')
        friction_weights = load_weights(args.friction_model)
        args.configs = ['pi_kp150_ki100', 'pi_kp150_ki100_ff100', 'pi_kp150_ki100_ff050']
    manifest.write_text(
        json.dumps(dict(arguments=vars(args), baseline_pi=[150, 100] if args.three_arm else None,
                        matched_ff1_path=bool(args.three_arm),
                        raw_capture=output.stem + '_uart',
                        note='Stationary restarts, not direct reversals. RAM-only configuration.'),
                   indent=2), encoding='utf-8')
    if any(CONFIGS[name][2] == 3 for name in args.configs) and weights is None:
        parser.error('feedforward trials require --ff-model')
    if any(name in ESTIMATOR_MODES for name in args.configs) and not args.estimator_study:
        parser.error('new estimator configurations require --estimator-study')
    if any(CONFIGS[name][2] in (5, 6) for name in args.configs):
        if not math.isfinite(args.angle_phase) or abs(args.angle_phase) > 3.142:
            parser.error('angle phase must be finite within +/-3.142 rad')
        if any(CONFIGS[name][2] == 6 for name in args.configs) and any(not 15 <= abs(t) <= 30 for t in args.targets):
            parser.error('angle application experiment limited to 15..30 RPM magnitude')
        if not args.angle_model:
            parser.error('angle shadow trials require --angle-model and nnangle1 firmware')
        from angle_shadow_model import load_angle_model
        import hashlib
        load_angle_model(args.angle_model)
        digest = hashlib.sha256(Path(args.angle_model).read_bytes()).hexdigest()
        header = Path(__file__).resolve().parents[1]/'app'/'angle_shadow_model.h'
        if digest not in header.read_text(encoding='utf-8'):
            parser.error('archive does not match the generated firmware model header')
        model_metadata.update(angle_model=str(Path(args.angle_model).resolve()), angle_model_sha256=digest)
        model_metadata.update(angle_phase_mrad=round(args.angle_phase*1000))
    port = CapturedSerial(resolve_port(args.port), output.with_suffix('').with_name(
        output.stem + '_uart'), timeout=.03)
    lock = threading.Lock(); done = threading.Event()
    results = []; traces = []
    def command(op, value=0):
        send_checked(port, op, value, lock)
    def timing_snapshot():
        command('e', 0)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            line = port.readline().decode('ascii', 'replace').strip()
            if line.startswith('E,'):
                return parse_timing(line)
        raise RuntimeError('missing timing snapshot')
    def heartbeat():
        while not done.wait(.2):
            try:
                tx(port, 'H', 0, lock)
            except Exception:
                done.set()
    def read_sample():
        if done.is_set():
            raise RuntimeError('heartbeat stopped')
        line = port.readline().decode('ascii', 'replace').strip()
        if not line.startswith('L,'):
            return None
        row = parse_learning(line)
        if row['fault'] or not row['sensor_ok'] or not row['calibrated']:
            raise RuntimeError(line)
        return row
    port.open(); worker = threading.Thread(target=heartbeat, daemon=True); worker.start()
    try:
        command('S'); command('T', 0)
        state = read_state(port, require_calibrated=False)
        print('STATE', ','.join(state), flush=True)
        if state[7] != '1':
            if not args.auto_calibrate:
                raise RuntimeError('Healthy encoder is uncalibrated; use --auto-calibrate once.')
            command('C', 0)
            print('CALIBRATING_ONCE', flush=True)
            read_state(port, timeout=10, require_calibrated=True)
            print('CALIBRATED', flush=True)
        command('G', 0)
        if args.estimator_study:
            initial_timing = timing_snapshot()
            print('TIMING_READY', json.dumps(initial_timing), flush=True)
        if any(CONFIGS[name][2] in (5, 6) for name in args.configs):
            command('a', round(args.angle_phase*1000))
        if weights is not None:
            for index, value in enumerate(weights):
                command('f', index); command('w', value)
        with output.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=['config', 'repeat', 'target_rpm']+LEARNING_FIELDS)
            writer.writeheader()
            rng = np.random.default_rng(args.seed)
            for repeat in range(args.repeats):
                for name in rng.permutation(args.configs):
                    kp, ki, mode, alpha = CONFIGS[name]
                    trial_name = name
                    trial_metadata = model_metadata
                    if args.three_arm:
                        trial_name, trial_weights = three_arm_setup(name, friction_weights, weights)
                        mode, alpha = 3, 1000
                        trial_metadata = (dict(zero_output_control=True) if trial_name == 'baseline'
                                          else model_identity(args.friction_model if
                                              trial_name == 'friction' else args.ff_model))
                        trial_metadata['matched_ff1_path'] = True
                        command('S')
                        for index, value in enumerate(trial_weights):
                            command('f', index); command('w', value)
                    command('S'); command('P', kp); command('Q', ki)
                    if args.estimator_study:
                        command('v', ESTIMATOR_MODES.get(name, 0))
                    command('G', mode); command('K', alpha); command('T', 1)
                    for target in args.targets:
                        command('S')
                        if name in ('scheduled', 'scheduled_ff100'):
                            selected_kp, selected_ki = speed_gains(target, 'tuned')
                            command('P', selected_kp); command('Q', selected_ki)
                        # Allow coast-down; verify a fresh stopped low-speed T frame.
                        time.sleep(.6)
                        settle_boundary = time.perf_counter_ns()
                        deadline = time.monotonic()+2
                        while time.monotonic() < deadline:
                            state = read_state(port, timeout=.5)
                            if (port.last_read_end_ns >= settle_boundary and
                                    state[6] == '0' and abs(int(state[4])) < 100):
                                break
                        else:
                            raise RuntimeError('rotor did not settle before step')
                        if args.estimator_study:
                            command('d', 0)
                        command('R', target)
                        samples = []; deadline = time.monotonic()+args.seconds
                        while time.monotonic() < deadline:
                            row = read_sample()
                            if row and row['mode'] == 2:
                                writer.writerow(dict(config=trial_name, repeat=repeat, target_rpm=target, **row))
                                if args.estimator_study and row['clipped']:
                                    stream.flush()
                                    raise RuntimeError('clipped estimator trial; stop and inspect before continuing')
                                if args.estimator_study and len(samples) > 20:
                                    displacement = abs(row['position_mrad'] - samples[0]['position_mrad'])
                                    if displacement > 100 and not any(abs(r['speed_mrad_s']) > 10 for r in samples[-20:]):
                                        stream.flush()
                                        raise RuntimeError('moving encoder with zero estimator feedback')
                                samples.append(row)
                        command('S'); stream.flush()
                        if len(samples) < args.seconds*60:
                            raise RuntimeError('insufficient telemetry')
                        result = dict(config=trial_name, repeat=repeat, target_rpm=target,
                                      **trial_metadata, **score(samples, target))
                        if args.estimator_study:
                            result['timing'] = timing_snapshot()
                        results.append(result); traces.append((result, samples))
                        output.with_suffix('.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
                        print(json.dumps(result), flush=True)
    finally:
        cleanup = [('S',0),('T',0),('G',0),('K',0),('P',8),('Q',284)]
        if args.estimator_study:
            cleanup.append(('v', 0))
        for op, value in cleanup:
            try:
                command(op,value)
            except Exception:
                pass
        done.set(); worker.join(1); port.close()
        if traces:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(len(args.targets), 2, figsize=(12, 4*len(args.targets)), squeeze=False)
            for result, samples in traces:
                i = args.targets.index(result['target_rpm'])
                t = (np.array([r['tick_ms'] for r in samples])-samples[0]['tick_ms'])/1000
                label = f"{result['config']} #{result['repeat']}"
                axes[i,0].plot(t, [r['speed_mrad_s']*60/(2000*math.pi) for r in samples], label=label, alpha=.7)
                axes[i,1].plot(t, [r['uq_mv']/1000 for r in samples], label=label, alpha=.7)
            for i,target in enumerate(args.targets):
                axes[i,0].axhline(target,color='black',linestyle='--')
                axes[i,0].set_ylabel('Speed (RPM)'); axes[i,1].set_ylabel('Uq (V)')
            for ax in axes.flat:
                ax.set_xlabel('Time (s)'); ax.grid(alpha=.2); ax.legend(fontsize=6)
            fig.tight_layout(); fig.savefig(output.with_suffix('.png'),dpi=140); plt.close(fig)


if __name__ == '__main__':
    main()
