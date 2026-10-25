import tempfile
import threading
import unittest
from pathlib import Path

from serial_capture import CapturedSerial
from uart_evidence import replay_capture


class FakeSerial:
    def __init__(self):
        self.chunks = [b'T,1,10', b',0\nACK,', b'S,0,pwm_init=0\r\n', b'tail']
        self.cancelled = threading.Event()

    @property
    def in_waiting(self):
        return 4096

    def open(self):
        pass

    def read(self, size):
        if self.chunks:
            return self.chunks.pop(0)
        self.cancelled.wait(.005)
        return b''

    def write(self, data):
        return len(data)

    def cancel_read(self):
        self.cancelled.set()

    def close(self):
        pass


class CaptureTests(unittest.TestCase):
    def test_persistent_framing_and_raw_replay(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / 'capture'
            port = CapturedSerial('FAKE', root, timeout=.1, transport=FakeSerial())
            port.open()
            try:
                self.assertEqual(port.readline(), b'T,1,10,0\n')
                self.assertEqual(port.readline(), b'ACK,S,0,pwm_init=0\n')
                self.assertGreater(port.last_read_end_ns, port.origin)
                self.assertEqual(port.write(b'S0\n'), 3)
                self.assertEqual(port.readline(), b'')
            finally:
                port.close()
            replay = replay_capture(root)
            self.assertEqual([r['data'] for r in replay.lines],
                             [b'T,1,10,0', b'ACK,S,0,pwm_init=0'])
            self.assertEqual(replay.pending, b'tail')

    def test_reader_failure_is_not_reported_as_empty_timeout(self):
        class BrokenSerial(FakeSerial):
            def read(self, size):
                raise OSError('injected USB disconnect')
        with tempfile.TemporaryDirectory() as temp:
            port = CapturedSerial('FAKE', Path(temp)/'capture', transport=BrokenSerial())
            port.open()
            with self.assertRaisesRegex(OSError, 'injected USB disconnect'):
                port.readline()
            with self.assertRaisesRegex(OSError, 'capture incomplete'):
                port.close()


if __name__ == '__main__':
    unittest.main()
