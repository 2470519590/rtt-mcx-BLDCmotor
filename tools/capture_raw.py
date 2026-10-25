"""Raw capture: every byte from the MCU, with host timestamps, no filtering.

Design rules (from the review):
* single process, single reader, the port is owned for the whole run;
* never clear the input buffer, never filter by prefix, never ignore decode
  errors;
* store (host_monotonic, bytes) per read so framing can be re-derived offline;
* offline: concatenate, split on newline, keep the trailing partial segment.

The online counters are deliberately not trusted; they are only printed for
comparison with what the offline reconstruction says.
"""
import argparse
import json
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import serial  # noqa: E402
from motor_port import resolve_port  # noqa: E402

TELEMETRY = re.compile(rb'^T,(\d+),(\d+),(-?\d+),(-?\d+),(\d+),(\d+),(\d+),(-?\d+),(-?\d+),(\d+),(-?\d+)$')

# The delay before the first command is a run parameter; a module global keeps
# capture() a plain function (an earlier draft referenced an undefined name).
delay_before_first_command = 0.5


def capture(seconds, send=False, period=0.12, command='D0'):
    port = serial.Serial(resolve_port('auto'), 115200, timeout=.05)
    port.dtr = False
    port.rts = False
    reads = []
    sent = []
    start = time.monotonic()
    next_send = start + delay_before_first_command
    try:
        while time.monotonic() - start < seconds:
            now = time.monotonic()
            if send and now >= next_send:
                port.write((command + '\n').encode())
                sent.append(now - start)
                next_send += period
            chunk = port.read(4096)
            if chunk:
                reads.append((time.monotonic() - start, chunk))
    finally:
        port.close()
    return reads, sent


answer = None


def offline(reads):
    blob = b''.join(chunk for _, chunk in reads)
    lines = blob.split(b'\n')
    partial = lines[-1]
    frames = [line.rstrip(b'\r') for line in lines[:-1]]
    telemetry = []
    other = []
    for index, frame in enumerate(frames):
        if not frame:
            continue
        match = TELEMETRY.match(frame)
        if match:
            telemetry.append(tuple(int(v) for v in match.groups()) + (index,))
        else:
            other.append((index, frame))
    return blob, frames, telemetry, other, partial


def report(reads, sent):
    blob, frames, telemetry, other, partial = offline(reads)
    total = len(blob)
    span = reads[-1][0] - reads[0][0] if reads else 0
    print(f'raw bytes        {total} in {span:.2f} s = {total / max(span, 1e-9):.0f} B/s')
    print(f'complete lines   {len(frames)} (+1 trailing partial of {len(partial)} bytes)')
    print(f'telemetry frames {len(telemetry)}, other lines {len(other)}')
    if telemetry:
        seqs = [t[0] for t in telemetry]
        ticks = [t[1] for t in telemetry]
        gaps = sorted({b - a for a, b in zip(seqs, seqs[1:])})
        tick_gaps = sorted({b - a for a, b in zip(ticks, ticks[1:])})
        print(f'seq   first={seqs[0]} last={seqs[-1]} distinct step={gaps[:6]}')
        print(f'tick  first={ticks[0]} last={ticks[-1]} distinct step={tick_gaps[:6]} ms')
        rate = len(telemetry) / max(span, 1e-9)
        print(f'telemetry rate   {rate:.1f} Hz (tick span {(ticks[-1] - ticks[0]) / 1000:.2f} s)')
        # Burstiness: how many frames share one host timestamp window.
        if reads:
            per_read = [chunk.count(b'\n') for _, chunk in reads]
            print(f'lines per read   min={min(per_read)} max={max(per_read)} '
                  f'mean={sum(per_read) / len(per_read):.1f}')
    if other:
        print('non-telemetry lines (hex for the first 6):')
        for index, frame in other[:6]:
            print(f'  line {index}: {frame!r}  hex={frame.hex()}')
    if partial:
        print(f'trailing partial: {partial!r}')
    print(f'commands sent    {len(sent)}')
    return dict(bytes=total, span=span, frames=len(frames), telemetry=len(telemetry),
                partial=len(partial), other=[repr(f) for _, f in other[:20]], sent=len(sent))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=6.0)
    parser.add_argument('--send', action='store_true')
    parser.add_argument('--period', type=float, default=.12)
    parser.add_argument('--command', default='D0')
    parser.add_argument('--delay', type=float, default=.5, help='wait before first command')
    parser.add_argument('--output')
    args = parser.parse_args()
    globals()['delay_before_first_command'] = args.delay
    reads, sent = capture(args.seconds, args.send, args.period, args.command)
    summary = report(reads, sent)
    if args.output:
        Path(args.output).write_text(json.dumps(dict(summary=summary,
                                                     reads=[(round(t, 6), c.hex()) for t, c in reads]),
                                                indent=1), encoding='utf-8')
        print('saved', args.output)
