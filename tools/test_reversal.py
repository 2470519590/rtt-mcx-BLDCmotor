import unittest
from collect_reversal import parse_sample, SEQUENCE


class ReversalTests(unittest.TestCase):
    def test_leading_frame_audit_catches_ack_consumption(self):
        from nn_dataset import LEARNING_FIELDS
        from report_reversal import audit_leading_samples
        row = dict.fromkeys(LEARNING_FIELDS, 0)
        row.update(sequence=1, mode=2, target_speed_mrad_s=523, nn_mode=7)
        def wire(item):
            return dict(data=('L,' + ','.join(str(item[k]) for k in LEARNING_FIELDS)).encode())
        lines = [wire(row), wire(dict(row, sequence=2))]
        csv_row = dict(row, config='friction', repeat='0', step='0')
        self.assertEqual(audit_leading_samples([csv_row], lines)[0]['leading_frames_omitted'], 0)
        csv_row['sequence'] = 2
        self.assertEqual(audit_leading_samples([csv_row], lines)[0]['leading_frames_omitted'], 1)

    def test_query_positive_value_is_not_rejection(self):
        from collect_reversal import query_result
        from unittest.mock import Mock, patch
        port = Mock()
        port.readline.side_effect = [b'T,1,10\n', b'ACK,z,3005,pwm_init=0\n']
        with patch('collect_reversal.tx') as transmit:
            self.assertEqual(query_result(port, 'z', 0, None), 3005)
            transmit.assert_called_once_with(port, 'z', 0, None)

    def test_reference_selection_and_equal_weight_summary(self):
        from report_reversal import reference_config, aggregate_comparison
        self.assertEqual(reference_config(['integral_keep','integral_reset']), 'integral_reset')
        self.assertEqual(reference_config(['reference_50','reference_0']), 'reference_0')
        with self.assertRaises(ValueError):
            reference_config(['unlabeled'])
        summary = [dict(configurations={
            'reference_0': {'rmse_rpm': dict(mean=a)},
            'reference_50': {'rmse_rpm': dict(mean=b)}})
            for a, b in ((2, 1), (4, 2))]
        item = aggregate_comparison(summary, ['rmse_rpm'])
        result = item['configurations']['reference_50']['rmse_rpm']
        self.assertEqual(result['mean_of_transition_means'], 1.5)
        self.assertEqual(result['reduction_percent'], 50)
        self.assertEqual(result['improved_transitions'], 2)

    def test_direction_crossing_requires_sustained_new_direction(self):
        from report_reversal import transition_metrics
        from nn_dataset import LEARNING_FIELDS
        rows=[]
        for i in range(301):
            row=dict.fromkeys(LEARNING_FIELDS,0)
            speed=-523 if i<20 else 523
            row.update(tick_ms=i*10,speed_mrad_s=speed,
                       position_mrad=round(i*.01*.523*1000),
                       sensor_ok=1,calibrated=1,mode=2)
            rows.append({k:str(v) for k,v in row.items()})
        metric=transition_metrics(rows,5,-5)
        self.assertAlmostEqual(metric['new_direction_held_50ms_s'],.2)
        self.assertEqual(metric['kind'],'reversal')
        rows[21]['speed_mrad_s']='-523'
        metric=transition_metrics(rows,5,-5)
        self.assertAlmostEqual(metric['new_direction_held_50ms_s'],.22)

    def frame(self, **changes):
        from nn_dataset import LEARNING_FIELDS
        fields = dict.fromkeys(LEARNING_FIELDS, 0)
        fields.update(sensor_ok=1, calibrated=1, mode=2, target_speed_mrad_s=523)
        fields.update(changes)
        return 'L,' + ','.join(str(fields[k]) for k in LEARNING_FIELDS)

    def test_fresh_target_and_health(self):
        self.assertIsNotNone(parse_sample(self.frame(), 5))
        self.assertIsNone(parse_sample(self.frame(target_speed_mrad_s=-523), 5))
        self.assertIsNone(parse_sample('ACK,R,0,pwm_init=0', 5))
        for key in ('fault', 'clipped'):
            with self.assertRaises(RuntimeError):
                parse_sample(self.frame(**{key:1}), 5)
        with self.assertRaises(RuntimeError):
            parse_sample(self.frame(sensor_ok=0), 5)
        with self.assertRaises(RuntimeError):
            parse_sample(self.frame(mode=0), 5)

    def test_sequence_contains_reversal_and_same_direction_steps(self):
        self.assertTrue(any(a*b < 0 for a, b in zip(SEQUENCE, SEQUENCE[1:])))
        self.assertTrue(any(a*b > 0 for a, b in zip(SEQUENCE, SEQUENCE[1:])))
        self.assertLessEqual(max(map(abs, SEQUENCE)), 15)


if __name__ == '__main__':
    unittest.main()
