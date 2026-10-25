"""One continuous UART reader, persistent framing, and replayable raw evidence."""
import json
import threading
import time
from collections import deque
from pathlib import Path

from uart_evidence import Framer


class CapturedSerial:
    def __init__(self, port, root, timeout=.03, transport=None):
        import serial
        self.port = port
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.timeout = timeout
        self.origin = time.perf_counter_ns()
        self.last_read_end_ns = 0
        self._serial = transport or serial.Serial(
            port=None, baudrate=115200, timeout=.02, write_timeout=.5)
        self._serial.port = port
        self._serial.dtr = False
        self._serial.rts = False
        self._lines = deque()
        self._condition = threading.Condition()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._error = None
        self._worker = None
        self._writes = None

    def open(self):
        self._serial.open()
        self._writes = (self.root / 'writes.jsonl').open('x', encoding='utf-8')
        self._worker = threading.Thread(target=self._reader, daemon=True)
        self._worker.start()

    def _reader(self):
        framer = Framer()
        offset = 0
        try:
            with (self.root / 'raw.bin').open('xb') as raw, \
                    (self.root / 'reads.jsonl').open('x', encoding='utf-8') as log:
                while not self._stop.is_set():
                    requested = min(max(self._serial.in_waiting, 1), 4096)
                    begin = time.perf_counter_ns() - self.origin
                    data = self._serial.read(requested)
                    end = time.perf_counter_ns() - self.origin
                    raw.write(data)
                    log.write(json.dumps(dict(begin_ns=begin, end_ns=end, offset=offset,
                                              requested=requested, returned=len(data))) + '\n')
                    offset += len(data)
                    framer.feed(data, begin, end)
                    with self._condition:
                        # Framer's byte offsets remain cumulative even when its
                        # consumed line list is released to bound memory.
                        self._lines.extend(framer.lines)
                        framer.lines.clear()
                        self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                self._error = exc
                self._condition.notify_all()

    def readline(self):
        deadline = time.monotonic() + self.timeout
        with self._condition:
            while not self._lines:
                if self._error:
                    raise OSError(f'UART reader failed: {self._error}')
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._stop.is_set():
                    return b''
                self._condition.wait(remaining)
            line = self._lines.popleft()
            self.last_read_end_ns = self.origin + line['read_end_ns']
            return line['data'] + b'\n'

    def write(self, data):
        with self._write_lock:
            record = dict(begin_ns=time.perf_counter_ns() - self.origin,
                          wire_hex=data.hex())
            try:
                record['written'] = self._serial.write(data)
                if record['written'] != len(data):
                    raise OSError('partial UART write')
                return record['written']
            except Exception as exc:
                record['error'] = repr(exc)
                raise
            finally:
                record['end_ns'] = time.perf_counter_ns() - self.origin
                self._writes.write(json.dumps(record) + '\n')
                self._writes.flush()

    def close(self):
        self._stop.set()
        if self._worker:
            self._worker.join(1)
            if self._worker.is_alive():
                self._serial.cancel_read()
                self._worker.join(1)
            if self._worker.is_alive():
                raise RuntimeError('UART reader did not terminate')
        self._serial.close()
        if self._writes:
            self._writes.close()
        if self._error:
            raise OSError(f'UART capture incomplete: {self._error}')
