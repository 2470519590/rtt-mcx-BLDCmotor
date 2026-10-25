"""Bounded, non-actuating UART evidence capture; never flashes or resets.

Normally only D<nonce> is transmitted. --stopped-check additionally sends a fixed
list of stop/read/configuration probes, never a motor start or calibration.
Current host_cli.c echoes the nonce directly
in D replies, before any motor command handler. Timeouts are inconclusive,
not evidence of firmware failure. Original bytes and read-call intervals are
preserved independently of framing. Run this file's host tests before capture.
"""
import argparse
import hashlib
import json
import re
import secrets
import threading
import time
from collections import Counter
from pathlib import Path

T_FRAME = re.compile(rb'T,(\d+),(\d+),(\d+),(-?\d+),(\d+),(\d+),(\d+),(-?\d+),(-?\d+),(\d+),(-?\d+)')
D_FRAME = re.compile(rb'D,(\d+),(\d+),(\d+),(\d+),(-?\d+),D(\d+)')


class Framer:
    def __init__(self):
        self.pending = b''
        self.offset = 0
        self.lines = []
        self.pending_begin_ns = None

    def feed(self, data, begin_ns, end_ns):
        if data and not self.pending:
            self.pending_begin_ns = begin_ns
        self.pending += data
        while b'\n' in self.pending:
            line, self.pending = self.pending.split(b'\n', 1)
            end = self.offset + len(line) + 1
            self.lines.append(dict(start=self.offset, end=end,
                                   read_begin_ns=self.pending_begin_ns, read_end_ns=end_ns,
                                   data=line.removesuffix(b'\r')))
            self.offset = end
            self.pending_begin_ns = begin_ns if self.pending else None


def match_reply(line, nonce):
    match = D_FRAME.fullmatch(line)
    return bool(match and int(match[4]) == nonce and int(match[6]) == nonce)


def eligible_reply(line, nonce, write_begin_ns, deadline_ns=None):
    # A blocking read may start before the write and contain the reply later.
    # The echoed nonce is the association; timestamps only bound observation.
    return (line['read_end_ns'] >= write_begin_ns and
            (deadline_ns is None or line['read_end_ns'] <= deadline_ns) and
            match_reply(line['data'], nonce))


STOPPED_PROBES = (b'b0\n', b'S0\n', b'z0\n', b'x0\n', b'D20\n',
                  b'E100\n', b'E0\n', b'b0\n')


def matches_request(line, request):
    if request['nonce'] is not None:
        return match_reply(line, request['nonce'])
    wire = bytes.fromhex(request['wire_hex'])
    # Fixed allowlist: no arbitrary command execution through this collector.
    if wire not in STOPPED_PROBES:
        return False
    if wire == b'b0\n':
        return bool(re.fullmatch(rb'B,(?:\d+,){6}\d+', line))
    if wire == b'E0\n':
        return bool(re.fullmatch(rb'E,[0-9a-fA-F]+,\d+,\d+,\d+', line))
    if wire == b'x0\n':
        return bool(re.fullmatch(rb'X,0,-?\d+,\d+,-?\d+,\d+,-?\d+,-?\d+', line))
    if wire == b'z0\n':
        return line in (b'ACK,z,3003,pwm_init=0', b'ACK,z,3004,pwm_init=0',
                        b'ACK,z,3005,pwm_init=0')
    return line == b'ACK,' + wire[:1] + b',0,pwm_init=0'


def eligible_request(line, request):
    return (not request.get('error') and
            request['write_begin_ns'] <= line['read_end_ns'] <= request['deadline_ns']
            and matches_request(line['data'], request))


def replay_capture(root):
    """Reconstruct frames solely from persisted raw bytes and read metadata."""
    root = Path(root)
    raw = (root / 'raw.bin').read_bytes()
    replay = Framer()
    offset = 0
    for text in (root / 'reads.jsonl').read_text().splitlines():
        row = json.loads(text)
        if row['offset'] != offset or not 0 <= row['returned'] <= row['requested']:
            raise ValueError('noncontiguous or invalid read log')
        end = offset + row['returned']
        if end > len(raw) or row['end_ns'] < row['begin_ns']:
            raise ValueError('invalid read interval or raw byte range')
        replay.feed(raw[offset:end], row['begin_ns'], row['end_ns'])
        offset = end
    if offset != len(raw):
        raise ValueError('unlogged raw bytes')
    return replay


def summarize(framer):
    telemetry = []
    other = []
    for index, line in enumerate(framer.lines):
        match = T_FRAME.fullmatch(line['data'])
        if match:
            telemetry.append(tuple(map(int, match.groups())))
        elif not D_FRAME.fullmatch(line['data']):
            other.append(dict(line=index, start=line['start'], hex=line['data'].hex()))
    return dict(complete_lines=len(framer.lines), telemetry=len(telemetry),
                seq_steps=dict(Counter((b[0] - a[0]) & 0xffffffff
                                       for a, b in zip(telemetry, telemetry[1:]))),
                tick_steps=dict(Counter((b[1] - a[1]) & 0xffffffff
                                        for a, b in zip(telemetry, telemetry[1:]))),
                other_lines=other, trailing_partial_hex=framer.pending.hex())


def run(args):
    import serial
    from motor_port import resolve_port

    if (args.count < 1 or args.count > 1000 or args.period < .05 or
            not .1 <= args.reply_timeout <= 5):
        raise ValueError('count 1..1000, period >= .05, reply timeout .1..5 required')
    if args.nonce_start is not None and not 0 <= args.nonce_start <= 1000000 - args.count:
        raise ValueError('nonce range must remain within the CLI argument range')
    if args.nonce_start is not None:
        if args.nonce_start < 100000 and not (args.nonce_start == 0 and args.count == 1
                                              and not getattr(args, 'stopped_check', False)):
            raise ValueError('use diagnostic nonces >=100000, or a single D0 compatibility probe')
        if args.nonce_start + args.count + len(STOPPED_PROBES) > 1000000:
            raise ValueError('reserve nonce range for stopped probes')
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=False)
    port_name = resolve_port(args.port)
    port = serial.Serial(port=None, baudrate=115200, timeout=.02, write_timeout=.5)
    port.dtr = False
    port.rts = False
    port.port = port_name
    framer = Framer()
    stop = threading.Event()
    ready = threading.Event()
    condition = threading.Condition()
    errors = []
    requests = []
    raw_count = 0
    origin = time.perf_counter_ns()

    def now():
        return time.perf_counter_ns() - origin

    def reader():
        nonlocal raw_count
        try:
            with (root / 'raw.bin').open('xb') as raw, (root / 'reads.jsonl').open('x') as log:
                ready.set()
                while not stop.is_set():
                    begin = now()
                    # A one-byte blocking wait avoids waiting for a large block;
                    # already queued bytes are read in bounded chunks.
                    requested = min(max(port.in_waiting, 1), 4096)
                    data = port.read(requested)
                    end = now()
                    raw.write(data)
                    log.write(json.dumps(dict(begin_ns=begin, end_ns=end,
                                              offset=raw_count, requested=requested,
                                              returned=len(data))) + '\n')
                    raw_count += len(data)
                    with condition:
                        framer.feed(data, begin, end)
                        condition.notify_all()
        except Exception as exc:
            errors.append(repr(exc))
            ready.set()
            with condition:
                condition.notify_all()

    thread = threading.Thread(target=reader, name='uart-evidence')
    failure = None
    try:
        port.open()
        thread.start()
        if not ready.wait(2):
            raise RuntimeError('reader did not start')
        time.sleep(2)
        first_nonce = args.nonce_start if args.nonce_start is not None else 100000 + secrets.randbelow(700000)
        plan = [(first_nonce + index, f'D{first_nonce + index}\n'.encode('ascii'))
                for index in range(args.count)]
        if getattr(args, 'stopped_check', False):
            # A unique nonce fence precedes every non-nonce request. One reader
            # and one outstanding request only; these ACKs are temporal matches,
            # not nonce-echo proof.
            for index, wire in enumerate(STOPPED_PROBES):
                nonce = first_nonce + args.count + index
                plan.extend(((nonce, f'D{nonce}\n'.encode('ascii')), (None, wire)))
        with (root / 'commands.jsonl').open('x') as command_log:
            for nonce, wire in plan:
                if errors:
                    raise RuntimeError('reader failed: ' + str(errors))
                record = dict(nonce=nonce, wire_hex=wire.hex(), write_begin_ns=now(),
                              byte_period_s=args.byte_period)
                try:
                    if args.byte_period:
                        record['written'] = 0
                        for byte_index, byte in enumerate(wire):
                            record['written'] += port.write(bytes([byte]))
                            if byte_index + 1 < len(wire):
                                time.sleep(args.byte_period)
                    else:
                        record['written'] = port.write(wire)
                    record['write_end_ns'] = now()
                    if record['written'] != len(wire):
                        raise RuntimeError('partial command write')
                except Exception as exc:
                    record.update(write_end_ns=now(), error=repr(exc), confirmed=False)
                    requests.append(record)
                    command_log.write(json.dumps(record) + '\n')
                    command_log.flush()
                    raise
                record['deadline_ns'] = record['write_end_ns'] + round(args.reply_timeout * 1e9)
                deadline = time.monotonic() + args.reply_timeout
                with condition:
                    while True:
                        candidates = [line for line in framer.lines
                                      if eligible_request(line, record)]
                        if candidates or errors or time.monotonic() >= deadline:
                            break
                        condition.wait(max(0, deadline - time.monotonic()))
                record['confirmed'] = len(candidates) == 1
                record['candidates'] = len(candidates)
                record['reply_offsets'] = [line['start'] for line in candidates]
                record['reply_read_end_ns'] = [line['read_end_ns'] for line in candidates]
                requests.append(record)
                command_log.write(json.dumps(record) + '\n')
                command_log.flush()
                if not record['confirmed']:
                    break
                time.sleep(max(0, args.period - (now() - record['write_begin_ns']) / 1e9))
        time.sleep(2)
    except Exception as exc:
        failure = repr(exc)
    finally:
        stop.set()
        if thread.ident is not None:
            thread.join(2)
            if thread.is_alive():
                port.cancel_read()
                thread.join()
        port.close()
        result = dict(port=port_name, elapsed_ns=now(), raw_bytes=raw_count,
                      read_timeout_s=.02, requested_commands=args.count +
                      (2 * len(STOPPED_PROBES) if getattr(args, 'stopped_check', False) else 0),
                      transmitted=len(requests), confirmed=sum(r['confirmed'] for r in requests),
                      failure=failure, reader_errors=errors, **summarize(framer))
        raw_path = root / 'raw.bin'
        if raw_path.exists():
            result['raw_sha256'] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
            replay = replay_capture(root)
            result['offline_replay_identical'] = (
                replay.lines == framer.lines and replay.pending == framer.pending)
            result['offline_requests'] = [
                dict(nonce=r['nonce'], candidates=sum(
                    eligible_request(line, r)
                    for line in replay.lines),
                    all_matching_offsets=[line['start'] for line in replay.lines
                                          if matches_request(line['data'], r)])
                for r in requests]
        result['interpretation'] = (
            'D replies use nonce matching; optional stopped probes use serialized '
            'time-bounded matching after a nonce fence. No root-cause claim.')
        (root / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps(result, indent=2))
    return 0 if not failure and not errors and result['confirmed'] == result['requested_commands'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='auto')
    parser.add_argument('--count', type=int, default=140)
    parser.add_argument('--nonce-start', type=int, default=None,
                        help='use 0 with count 1 for a D0 compatibility probe; not a unique nonce')
    parser.add_argument('--period', type=float, default=.12)
    parser.add_argument('--reply-timeout', type=float, default=.5)
    parser.add_argument('--byte-period', type=float, default=0,
                        help='host inter-byte pause, e.g. .003 for RX burst comparison')
    parser.add_argument('--output', required=True)
    parser.add_argument('--stopped-check', action='store_true',
                        help='append fixed stop/read probes and D20/E100 sweep configuration; never start')
    raise SystemExit(run(parser.parse_args()))
