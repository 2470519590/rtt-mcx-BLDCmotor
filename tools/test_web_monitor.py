"""Local HTTP/telemetry regression only. Never opens or writes a serial port."""
import json
import threading
import unittest
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from unittest.mock import Mock, patch

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

    def test_new_connection_clears_previous_demo_failure_and_replies(self):
        self.monitor._demo_update(error="旧固件错误", phase="演示未完成",
                                  origin_mrad=123, target_deg=120)
        self.monitor._consume_line('ACK,c,-1,pwm_init=0')
        candidate = Mock()
        candidate.read.return_value = b''
        candidate.in_waiting = 0
        with patch('web_monitor.serial.Serial', return_value=candidate):
            self.monitor.connect('TEST')
        state = self.monitor.snapshot()
        self.assertEqual(state['connection_id'], 1)
        self.assertEqual(state['demo'], dict(running=False, phase="待命", error=None,
                                             target_deg=None, origin_mrad=None))
        self.assertFalse(self.monitor.replies)

    def test_disconnect_clears_idle_demo_error(self):
        self.monitor._demo_update(error="旧错误", phase="演示未完成")
        self.monitor._consume_line('ACK,c,-1,pwm_init=0')
        self.monitor.disconnect()
        self.assertIsNone(self.monitor.snapshot()['demo']['error'])
        self.assertEqual(self.monitor.snapshot()['demo']['phase'], '待命')
        self.assertFalse(self.monitor.replies)

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
            with patch.object(self.monitor.demo, 'start_calibration') as start:
                connection.request('POST', '/api/calibrate', body='{}',
                                   headers={'Content-Type': 'application/json'})
                response = connection.getresponse()
                self.assertEqual(response.status, 202)
                response.read()
                start.assert_called_once_with()
            for payload in ({"action": "nn", "enabled": True, "alpha": .25},
                            {"action": "parameters", "profile": "position"},
                            [], {"action": "run", "rpm": "60"}):
                connection.request('POST', '/api/control', body=json.dumps(payload),
                                   headers={'Content-Type': 'application/json'})
                response = connection.getresponse()
                self.assertEqual(response.status, 400)
                self.assertIn("error", json.loads(response.read()))
            self.assertEqual(self.monitor.snapshot()["control"]["applied"], {})
            self.assertIsNone(self.monitor.snapshot()["control"]["nn_mode"])
            # Exercise the real HTTP router + command implementation, not a
            # mocked API response. No hardware and no display-health gating.
            sent = []
            def request(op, value=0):
                sent.append((op, value))
                with self.monitor.changed:
                    self.monitor._acknowledge(op, value)
                return 1000 if op == "c" else 0
            with patch.object(self.monitor, "request", side_effect=request):
                for payload, expected in (
                    ({"action": "run", "rpm": 60}, ("R", 60)),
                    ({"action": "position", "degrees": 120, "reference": "relative"},
                     ("X", 3094)),
                    ({"action": "stop"}, ("S", 0)),
                ):
                    connection.request("POST", "/api/control", body=json.dumps(payload),
                                       headers={"Content-Type": "application/json"})
                    response = connection.getresponse()
                    body = json.loads(response.read())
                    self.assertEqual(response.status, 200, body)
                    self.assertTrue(body["ok"])
                    self.assertEqual(sent[-1], expected)
                    self.assertEqual(body["state"]["service_revision"], "manual-control-20261026b")
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            worker.join(2)


if __name__ == '__main__':
    unittest.main()
