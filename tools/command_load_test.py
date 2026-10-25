"""Reader-thread capture: never blocks the command path, never loses bytes.

The previous attempt read synchronously between commands, so at ~4 KB/s of
telemetry the ACKs were buried and the "unanswered" counts were artefacts.
Here a dedicated thread drains the port continuously, tags each chunk with its
arrival time, and the command loop only records send times. Pairing happens
offline on the reassembled stream.

Profiles:
  ask-answer   send one command, wait for its ACK byte to show up (bounded)
  fixed-rate   send every N ms without waiting
Both report latency and, crucially, whether the ACK byte ever appeared at all.
"""
import argparse
import json
import re
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import serial  # noqa: E402
from motor_port import resolve_port  # noqa: E402

ACK = re.compile(rb'^ACK,(\w),(-?\d+),pwm_init=(-?\d+)')


class Reader:
    def __init__(self, port):
        self.port = port
        self.chunks = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _loop(self):
        while not self.stop.is_set():
            try:
                data = self.port.read(4096)
            except Exception:
                break
            if data:
                with self.lock:
                    self.chunks.append((time.monotonic(), data))

    def stop_reader(self):
        self.stop.set()
        self.thread.join(timeout=1)
        with self.lock:
            return list(self.chunks)


def reassemble(chunks):
    """Concatenate and attach to each line the arrival time of its last byte."""
    blob = b''.join(c for _, c in chunks)
    if not blob:
        return [], blob
    # Map byte offset -> arrival time.
    offsets = []
    total = 0
    for stamp, chunk in chunks:
        total += len(chunk)
        offsets.append((total, stamp))
    lines = []
    start = 0
    import bisect
    ends = [o for o, _ in offsets]
    for match in re.finditer(rb'[^\n]*\n', blob):
        end = match.end()
        index = bisect.bisect_left(ends, end)
        stamp = offsets[min(index, len(offsets) - 1)][1]
        text = match.group(0)[:-1].rstrip(b'\r')
        if text:
            lines.append((stamp, text))
        start = end
    tail = blob[start:]
    return lines, tail


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['ask-answer', 'fixed-rate'], default='ask-answer')
    parser.add_argument('--count', type=int, default=200)
    parser.add_argument('--period', type=float, default=.05)
    parser.add_argument('--command', default='D0')
    parser.add_argument('--timeout', type=float, default=.5)
    parser.add_argument('--settle', type=float, default=1.0)
    parser.add_argument('--listen', type=float, default=1.0)
    parser.add_argument('--output')
    args = parser.parse_args()

    port = serial.Serial(resolve_port('auto'), 115200, timeout=.02)
    port.dtr = False
    port.rts = False
    reader = Reader(port).start()
    sent = []
    try:
        time.sleep(args.settle)
        start = time.monotonic()
        for _ in range(args.count):
            port.write((args.command + '\n').encode())
            sent.append(time.monotonic())
            if args.profile == 'ask-answer':
                deadline = time.monotonic() + args.timeout
                while time.monotonic() < deadline:
                    time.sleep(args.period)
                    break
            else:
                time.sleep(args.period)
        time.sleep(args.listen)
    finally:
        chunks = reader.stop_reader()
        port.close()

    lines, tail = reassemble(chunks)
    acks = [(stamp, ACK.match(text).group(1)) for stamp, text in lines if ACK.match(text)]
    telemetry = sum(1 for _, text in lines if text.startswith(b'T,'))
    # Pair in order: each ACK consumes the oldest unmatched command with that op.
    queue = list(range(len(sent)))
    latencies = []
    matched = 0
    op = args.command[0].encode()
    cursor = 0
    for stamp, ack_op in acks:
        while cursor < len(sent) and sent[cursor] > stamp:
            break
        if cursor < len(sent) and ack_op == op:
            latencies.append((stamp - sent[cursor]) * 1000)
            matched += 1
        cursor += 1
    result = dict(profile=args.profile, command=args.command, sent=len(sent),
                  acks=len(acks), matched=matched, telemetry_lines=telemetry,
                  tail_bytes=len(tail),
                  latency_ms=dict(min=round(min(latencies), 1) if latencies else None,
                                  max=round(max(latencies), 1) if latencies else None,
                                  median=round(sorted(latencies)[len(latencies) // 2], 1)
                                  if latencies else None),
                  span_s=round((sent[-1] - sent[0]), 2) if sent else 0)
    print(json.dumps(result, indent=1))
    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()
