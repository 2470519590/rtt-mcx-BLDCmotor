"""Local HTTP/telemetry regression only. Never opens or writes a serial port."""
import json
import threading
import unittest
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from unittest.mock import patch

from web_monitor import MonitorHandler, SerialMonitor


class MonitorTests(unittest.TestCase):
    def setUp(self):
        self.serial_guard = patch('web_monitor.serial.Serial',
                                  side_effect=AssertionError('No serial hardware in UI tests'))
        self.serial_guard.start()
        self.monitor = SerialMonitor(115200)

    def tearDown(self):
        self.monitor.close()
        self.serial_guard.stop()

    def test_startup_has_no_fabricated_data_or_connection(self):
        state = self.monitor.snapshot()
        self.assertFalse(state['connected'])
        self.assertIsNone(state['updated_at'])
        self.assertIsNone(state['raw'])

    def test_current_firmware_telemetry_keeps_angle_speed(self):
        self.monitor._consume_line('T,10,100,1536,1571,1,2,1,1047,300,0,-888')
        state = self.monitor.snapshot()
        self.assertEqual(state['raw'], 1536)
        self.assertEqual(state['angle_cdeg'], 13500)
        self.assertEqual(state['speed_mrad_s'], 1571)
        self.assertEqual(state['motor_mode'], 2)
        self.assertEqual(state['uq_mv'], 300)
        self.assertEqual(state['sensor'], 'ok')

    def test_bad_frames_do_not_replace_good_data(self):
        self.monitor._consume_line('T,10,100,1536,1571,1')
        for line in ('T,11,110,9999,0,1', 'T,11,110,2,0,3', 'T,not,a,number,0,1'):
            self.monitor._consume_line(line)
        self.assertEqual(self.monitor.snapshot()['sequence'], 10)

    def test_dropped_sequence_and_sensor_error(self):
        self.monitor._consume_line('T,10,100,1536,1571,1')
        self.monitor._consume_line('T,13,130,1536,0,0')
        state = self.monitor.snapshot()
        self.assertEqual(state['dropped_frames'], 2)
        self.assertEqual(state['sensor'], 'error')

    def test_http_serves_split_assets_and_unconnected_state(self):
        class Handler(MonitorHandler):
            monitor = self.monitor

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        connection = HTTPConnection('127.0.0.1', server.server_port, timeout=3)
        try:
            for asset, content in (('/', b'lang="zh-CN"'), ('/styles.css', b'.dial-wrap'),
                                   ('/monitor.js', b'EventSource')):
                connection.request('GET', asset)
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                self.assertIn(content, response.read())
            connection.request('GET', '/api/state')
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertFalse(json.loads(response.read())['connected'])
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            worker.join(2)


if __name__ == '__main__':
    unittest.main()
