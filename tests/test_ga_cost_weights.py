"""Fitness priorities change ranking without changing physical pass criteria."""
import sys
from pathlib import Path
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from ga_lqr import cost_weights, summarize_trial

class CostWeightsTests(unittest.TestCase):
    def score(self,pitch,position,weights=None):
        fields=['t_free_s','pitch_true_deg','pitch_target_deg','pose_true_x_m',
                'pose_true_y_m','yaw_true_rad','velocity_world_forward_m_s',
                'pwm_sx_applied_percent','pwm_dx_applied_percent','velocity_world_lateral_m_s']
        rows=[dict(zip(fields,[i*.01,pitch,0,position,0,0,0,0,0,0])) for i in range(300)]
        with patch('ga_lqr.trial_inputs',return_value=({'success':True,'stop_reason':'durata completata'}, {}, rows)):
            return summarize_trial('unused.csv',3.,50.,weights)

    def test_pitch_priority_changes_preferred_tradeoff(self):
        a=self.score(1.5,0);b=self.score(0,.015)
        self.assertLess(a['cost'],b['cost'])
        changed_a=self.score(1.5,0,{'pitch':16});changed_b=self.score(0,.015,{'pitch':16})
        self.assertGreater(changed_a['cost'],changed_b['cost'])
        self.assertEqual(a['stabilized'],changed_a['stabilized'])
        self.assertEqual(b['stabilized'],changed_b['stabilized'])
        self.assertEqual(changed_a['cost_weights']['position'],1.)

    def test_defaults_preserve_historical_cost(self):
        self.assertAlmostEqual(self.score(1.5,.01)['cost'],2.)
        self.assertEqual(self.score(1.5,.01)['cost'],self.score(1.5,.01,cost_weights())['cost'])

    def test_invalid_coefficients_rejected(self):
        for value in (-1.,float('inf'),float('nan'),True):
            with self.assertRaises(ValueError):cost_weights({'pitch':value})
        with self.assertRaises(ValueError):cost_weights({'unknown':1})
        with self.assertRaises(ValueError):cost_weights(dict.fromkeys(cost_weights(),0))

if __name__=='__main__':unittest.main()
