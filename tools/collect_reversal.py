"""Bounded continuous speed reversals, without stop/calibration between steps.

R commands reset the firmware PI integral unless --transition-study selects
the matched reset/keep experiment. Neither policy cancels the P/FF step.
"""
import argparse
import csv
import hashlib
import json
import threading
import time
from pathlib import Path

import numpy as np
from collect_nn_dataset import tx, send_checked, read_state
from motor_feedforward import load_weights, model_identity
from motor_port import resolve_port
from nn_dataset import LEARNING_FIELDS, parse_learning
from serial_capture import CapturedSerial
from optimize_motor import parse_timing

SEQUENCE = (5, -5, 15, -15, 10, -10, 5, 15, -5, -15)


def query_result(port, op, value, lock, timeout=2.0):
    """Positive diagnostic replies are values, not command rejections."""
    tx(port, op, value, lock)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        fields = port.readline().decode('ascii', 'replace').strip().split(',')
        if len(fields) >= 3 and fields[:2] == ['ACK', op]:
            return int(fields[2])
    raise RuntimeError(f'no query reply for {op}{value}')


def parse_sample(line, target):
    if not line.startswith('L,'):
        return None
    row = parse_learning(line)
    if not row['sensor_ok'] or not row['calibrated'] or row['fault'] or row['clipped']:
        raise RuntimeError('unhealthy or clipped reversal trial: ' + line)
    if row['mode'] != 2:
        raise RuntimeError('motor left speed mode during reversal')
    expected = int(target * 2*np.pi/60*1000)
    return row if abs(row['target_speed_mrad_s'] - expected) <= 1 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--friction-model', required=True)
    parser.add_argument('--port', default='auto')
    parser.add_argument('--seconds', type=float, default=4)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--seed', type=int, default=631)
    parser.add_argument('--configs', nargs='+', choices=['baseline','friction','dynamic_nn','dynamic_linear'],
                        default=['baseline','friction'])
    parser.add_argument('--dynamic', action='store_true',
                        help='Use nndyn1 matched G7/G8/G9 execution paths')
    parser.add_argument('--auto-calibrate', action='store_true')
    parser.add_argument('--transition-study', action='store_true',
                        help='nntrans1 only: matched G7 static-only reset/keep policies')
    parser.add_argument('--reference-study', nargs='+', type=int, metavar='TAU_MS',
                        help='nnref1 only: compare P reference filter constants, include 0')
    args = parser.parse_args()
    if args.reference_study is not None:
        if (args.transition_study or not args.dynamic or
                0 not in args.reference_study or
                len(set(args.reference_study)) != len(args.reference_study) or
                any(t < 0 or t > 200 for t in args.reference_study)):
            parser.error('reference study needs --dynamic, distinct tau 0..200 including 0, no transition study')
        args.configs = [f'reference_{t}' for t in args.reference_study]
    if args.transition_study:
        if not args.dynamic:
            parser.error('--transition-study requires --dynamic for matched G7 timing')
        args.configs = ['integral_reset', 'integral_keep']
    if not 3 <= args.seconds <= 8 or not 1 <= args.repeats <= 5:
        parser.error('seconds 3..8 and repeats 1..5 required')
    if len(set(args.configs)) != len(args.configs) or (
            any(c.startswith('dynamic') for c in args.configs) and not args.dynamic):
        parser.error('distinct configurations; dynamic candidates require --dynamic')
    output = Path(args.output)
    if output.exists() or output.with_suffix('.json').exists():
        raise FileExistsError(output)
    weights = load_weights(args.friction_model)
    port = CapturedSerial(resolve_port(args.port), output.with_name(output.stem + '_uart'))
    lock = threading.Lock()
    done = threading.Event()
    errors = []
    events = []
    failure = None
    final = None
    timing = None
    firmware_build = None
    provenance = {}
    if args.dynamic:
        model_path = Path(__file__).resolve().parents[1]/'app'/'dynamic_residual_model.h'
        provenance['dynamic_header_sha256'] = hashlib.sha256(model_path.read_bytes()).hexdigest()
        provenance['note'] = 'Local compiled-header identity; not a direct MCU flash readback.'

    def command(op, value=0):
        send_checked(port, op, value, lock)

    def heartbeat():
        while not done.wait(.2):
            try:
                tx(port, 'H', 0, lock)
            except Exception as exc:
                errors.append(repr(exc))
                done.set()

    port.open()
    worker = threading.Thread(target=heartbeat, daemon=True)
    worker.start()
    try:
        command('S'); command('T', 0); command('G', 0); command('V', 1000)
        firmware_build = query_result(port, 'z', 0, lock)
        if args.transition_study and firmware_build < 3004:
            raise RuntimeError('transition study requires build 3004 or newer')
        if args.reference_study is not None and firmware_build < 3005:
            raise RuntimeError('reference study requires build 3005 or newer')
        # A previous CLI experiment can leave RAM options enabled. Restore the
        # documented baseline explicitly before collecting any new study.
        if firmware_build >= 3004:
            command('j', 0)
        if firmware_build >= 3005:
            command('r', 0)
        state = read_state(port,require_calibrated=False)
        if state[7] != '1':
            if not args.auto_calibrate:
                raise RuntimeError('uncalibrated; --auto-calibrate enables exactly one alignment')
            command('C',0)
            state=read_state(port,timeout=10)
        print('READY', ','.join(state), flush=True)
        if args.dynamic:
            command('d',0)
        # All arms use G3, including zero weights in the classical PI arm.
        with output.open('x', newline='', encoding='utf-8') as stream:
            fields = ['config', 'repeat', 'step', 'target_rpm', 'previous_target_rpm',
                      'host_read_end_ns'] + LEARNING_FIELDS
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            rng = np.random.default_rng(args.seed)
            for repeat in range(args.repeats):
                for config in rng.permutation(args.configs):
                    command('S'); command('T', 0)
                    command('P', 150); command('Q', 100); command('v', 0)
                    for index, value in enumerate(weights if config != 'baseline' else [0]*9):
                        command('f', index); command('w', value)
                    if args.transition_study:
                        command('j', int(config == 'integral_keep'))
                        if query_result(port, 'j', 2, lock) != int(config == 'integral_keep'):
                            raise RuntimeError('integral policy readback mismatch')
                    if args.reference_study is not None:
                        command('j', 0)
                        command('r', int(config.split('_')[1]))
                    mode=({'baseline':7,'friction':7,'dynamic_nn':8,'dynamic_linear':9,
                           'integral_reset':7,'integral_keep':7}[config]
                          if args.dynamic and args.reference_study is None else
                          7 if args.reference_study is not None else 3)
                    command('G',mode); command('K',1000)
                    time.sleep(.6)
                    boundary = time.perf_counter_ns()
                    deadline = time.monotonic()+3
                    while time.monotonic() < deadline:
                        state = read_state(port, timeout=.5)
                        if (port.last_read_end_ns >= boundary and state[6] == '0'
                                and abs(int(state[4])) < 100):
                            break
                    else:
                        raise RuntimeError('rotor did not settle before sequence')
                    command('T', 1)
                    previous = 0
                    for step, target in enumerate(SEQUENCE):
                        if done.is_set():
                            raise RuntimeError('heartbeat failed: ' + str(errors))
                        event = dict(config=str(config), repeat=repeat, step=step,
                                     previous_target_rpm=previous, target_rpm=target,
                                     write_begin_ns=time.perf_counter_ns())
                        command('R', target)  # deliberately NO stop here
                        event['ack_observed_ns'] = time.perf_counter_ns()
                        deadline = time.monotonic()+args.seconds
                        count = 0
                        while time.monotonic() < deadline:
                            if done.is_set():
                                raise RuntimeError('heartbeat failed: ' + str(errors))
                            line = port.readline().decode('ascii', 'replace').strip()
                            row = parse_sample(line, target)
                            if row is None:
                                continue
                            event.setdefault('first_sample_tick_ms', row['tick_ms'])
                            writer.writerow(dict(config=config, repeat=repeat, step=step,
                                target_rpm=target, previous_target_rpm=previous,
                                host_read_end_ns=port.last_read_end_ns, **row))
                            count += 1
                        stream.flush()
                        event['samples'] = count
                        events.append(event)
                        if count < args.seconds*60:
                            raise RuntimeError('insufficient telemetry')
                        print(config, repeat, step, previous, '->', target, count, flush=True)
                        previous = target
                    command('S'); command('T', 0)
    except Exception as exc:
        failure = repr(exc)
        raise
    finally:
        cleanup = []
        for op, value in [('S',0), ('T',0), ('G',0), ('K',0), ('P',8), ('Q',284)]:
            try:
                command(op, value)
            except Exception as exc:
                cleanup.append(repr(exc))
        if args.transition_study:
            try:
                command('j', 0)
            except Exception as exc:
                cleanup.append(repr(exc))
        if args.reference_study is not None:
            for op in ('j', 'r'):
                try:
                    command(op, 0)
                except Exception as exc:
                    cleanup.append(repr(exc))
        if args.dynamic:
            try:
                command('e',0)
                deadline=time.monotonic()+2
                while time.monotonic() < deadline:
                    line=port.readline().decode('ascii','replace').strip()
                    if line.startswith('E,'):
                        timing=parse_timing(line)
                        break
                if timing is None:
                    raise RuntimeError('missing dynamic timing snapshot')
            except Exception as exc:
                cleanup.append(repr(exc))
        try:
            boundary = time.perf_counter_ns()
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                state = read_state(port, timeout=.5, require_calibrated=False)
                if port.last_read_end_ns >= boundary:
                    final = ','.join(state)
                    break
        except Exception as exc:
            cleanup.append(repr(exc))
        done.set(); worker.join(1)
        try:
            port.close()
        finally:
            output.with_suffix('.json').write_text(json.dumps(dict(
                arguments=vars(args), events=events, failure=failure,
                firmware_build=firmware_build,
                heartbeat_errors=errors, cleanup_errors=cleanup, final_state=final,
                timing=timing,
                provenance=provenance,
                **model_identity(args.friction_model),
                note=('Direct R transitions; reset integral, P-reference filter study only. '
                      if args.reference_study is not None else
                      'Direct R transitions; reset vs keep PI integral within speed mode. '
                      if args.transition_study else
                      'Direct R target transitions; legacy R integral reset policy required. ') +
                     'Baseline=zero ff1 weights; all arms alpha1/P150/Q100/v0. '
                     'Dynamic study uses matched G7 shadow/G8 NN/G9 explicit linear.'),
                indent=2), encoding='utf-8')
            print('FINAL', final, 'FAILURE', failure, 'CLEANUP', cleanup, flush=True)


if __name__ == '__main__':
    main()
