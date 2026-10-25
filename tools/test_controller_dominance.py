import unittest
from controller_dominance import compare, SPEED_METRICS, POSITION_METRICS


class DominanceTests(unittest.TestCase):
    def rows(self, metrics, key):
        return [dict(config=name, **{key:10}, clipped=0,
                     **{m:value for m in metrics})
                for name,value in [('baseline',2.),('candidate',1.)]]

    def test_every_metric_improves(self):
        self.assertTrue(compare(self.rows(SPEED_METRICS,'target_rpm'),
                                'baseline','candidate','speed')['strict_measured_dominance'])

    def test_one_worse_metric_blocks_claim(self):
        rows=self.rows(SPEED_METRICS,'target_rpm')
        rows[1]['raw_tail_rmse_rpm']=3
        self.assertFalse(compare(rows,'baseline','candidate','speed')['strict_measured_dominance'])

    def test_two_nonsettling_trials_are_not_equal(self):
        rows=self.rows(POSITION_METRICS,'step_deg')
        for row in rows:
            row['settled_0p5deg_s']=None
        result=compare(rows,'baseline','candidate','position')
        self.assertFalse(result['strict_measured_dominance'])
        self.assertEqual(result['targets'][0]['metrics']['settled_0p5deg_s']['verdict'],'unresolved')

    def test_mismatched_counts_rejected(self):
        with self.assertRaises(ValueError):
            compare(self.rows(SPEED_METRICS,'target_rpm')[:1],'baseline','candidate','speed')

    def test_clipping_blocks_claim(self):
        rows=self.rows(SPEED_METRICS,'target_rpm')
        rows[1]['clipped']=1
        self.assertFalse(compare(rows,'baseline','candidate','speed')['strict_measured_dominance'])


if __name__=='__main__':
    unittest.main()
