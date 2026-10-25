import unittest
from pathlib import Path

from uart_evidence import (Framer, eligible_reply, match_reply, summarize,
                           matches_request, eligible_request, STOPPED_PROBES)


class EvidenceTests(unittest.TestCase):
    def test_stopped_probe_allowlist_and_time_bounds(self):
        for wire, reply in (
                (b'b0\n', b'B,2640,2640,0,0,0,0,0'),
                (b'S0\n', b'ACK,S,0,pwm_init=0'),
                (b'z0\n', b'ACK,z,3003,pwm_init=0'),
                (b'z0\n', b'ACK,z,3004,pwm_init=0'),
                (b'z0\n', b'ACK,z,3005,pwm_init=0'),
                (b'x0\n', b'X,0,-1,0,0,0,0,0'),
                (b'D20\n', b'ACK,D,0,pwm_init=0'),
                (b'E100\n', b'ACK,E,0,pwm_init=0'),
                (b'E0\n', b'E,45000000,2,255,0')):
            request = dict(nonce=None, wire_hex=wire.hex(), write_begin_ns=10, deadline_ns=20)
            self.assertIn(wire, STOPPED_PROBES)
            self.assertTrue(matches_request(reply, request))
            self.assertTrue(eligible_request(dict(data=reply, read_end_ns=15), request))
            self.assertFalse(eligible_request(dict(data=reply, read_end_ns=9), request))
            self.assertFalse(eligible_request(dict(data=reply, read_end_ns=21), request))
            self.assertFalse(matches_request(reply + b',junk', request))
        self.assertFalse(matches_request(b'ACK,R,0,pwm_init=0',
                         dict(nonce=None, wire_hex=b'R60\n'.hex())))

    def test_cross_chunk_interval_and_deadline(self):
        framer = Framer()
        framer.feed(b'D,80,2,0,', 1, 2)
        framer.feed(b'100001,0,D100001\r\n', 10, 11)
        line = framer.lines[0]
        self.assertEqual(line['read_begin_ns'], 1)
        self.assertEqual(line['read_end_ns'], 11)
        self.assertFalse(eligible_reply(line, 100001, 3, 10))
        self.assertTrue(eligible_reply(line, 100001, 3, 11))

    def test_every_possible_chunk_boundary(self):
        blob = b'junk\nT,1,10,0,0,1,0,0,0,0,0,0\r\nD,80,2,0,100001,0,D100001\r\ntail'
        for split in range(len(blob) + 1):
            framer = Framer()
            framer.feed(blob[:split], 0, 1)
            framer.feed(blob[split:], 2, 3)
            self.assertEqual(framer.pending, b'tail')
            self.assertEqual(summarize(framer)['telemetry'], 1)
            self.assertTrue(match_reply(framer.lines[2]['data'], 100001))
            self.assertEqual(framer.lines[2]['start'], blob.index(b'D,'))

    def test_wrong_stale_and_malformed_replies_rejected(self):
        good = b'D,80,2,0,100001,0,D100001'
        self.assertTrue(match_reply(good, 100001))
        self.assertFalse(match_reply(good, 100002))
        self.assertFalse(match_reply(good + b',garbage', 100001))
        self.assertFalse(match_reply(b'ACK,D,0,pwm_init=0', 100001))
        self.assertFalse(eligible_reply(dict(data=good, read_end_ns=9), 100001, 10))
        self.assertTrue(eligible_reply(dict(data=good, read_begin_ns=9, read_end_ns=11), 100001, 10))

    def test_uint32_wrap_and_internal_corruption_remain_visible(self):
        framer = Framer()
        framer.feed(b'T,4294967295,4294967290,0,0,1,0,0,0,0,0,0\n'
                    b'T,0,4,0,0,1,0,0,0,0,0,0\n,0,0\n', 0, 1)
        report = summarize(framer)
        self.assertEqual(report['seq_steps'], {1: 1})
        self.assertEqual(report['tick_steps'], {10: 1})
        self.assertEqual(len(report['other_lines']), 1)

    def test_upload_row_and_snapshot_source_contract(self):
        app = Path(__file__).resolve().parents[1] / 'app'
        control = (app / 'motor_control.c').read_text(encoding='utf-8')
        cli = (app / 'host_cli.c').read_text(encoding='utf-8')
        self.assertIn('s_upload_row[ANGLE_MODEL_INPUTS + 1U]', control)
        self.assertIn('s_upload_row[index + 1U] = magnitude;', control)
        self.assertIn('static motor_state_t state;', cli)


if __name__ == '__main__':
    unittest.main()
