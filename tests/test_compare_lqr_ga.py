"""Frozen-pair provenance and identical physical disturbance integration."""
import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from ga_lqr import decode, evaluate, save_json, scenarios
from motion_lqr import design_motion_lqr
spec = importlib.util.spec_from_file_location('compare_lqr_ga',ROOT/'scripts/ga/compare_lqr_ga.py')
compare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compare)


class CompareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal = design_motion_lqr()
        cls.candidate = decode(cls.nominal,[.1,-.2,.3,.4,-.5,.6])

    def prepare(self, folder):
        save_json(folder/'nominal_design.json',self.nominal)
        save_json(folder/'best.json',self.candidate)

    def test_pair_accepts_optimized_r_on_same_nominal(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.prepare(folder)
            genes = np.zeros(8); genes[6:] = [.2,-.1]
            save_json(folder/'best.json',decode(self.nominal,genes))
            pair = compare.load_pair(folder)
            self.assertFalse(np.array_equal(pair['baseline']['R'],pair['ga']['R']))

    def test_rejects_candidate_from_other_nominal(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.prepare(folder)
            candidate = json.loads((folder/'best.json').read_text())
            candidate['ga']['nominal_design_sha256']='other'
            save_json(folder/'best.json',candidate)
            with self.assertRaises(ValueError):
                compare.load_pair(folder)

    def test_same_pulse_seed_initial_angle_and_unchanged_gains(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.prepare(folder)
            with contextlib.redirect_stdout(io.StringIO()):
                code = compare.main(['--ga-run',str(folder),'--angles','3','--forces','.1',
                                     '--output-dir',str(folder/'out'),'--no-plots','--skip-path'])
            self.assertEqual(code,0)
            batch = next((folder/'out').iterdir())
            results = json.loads((batch/'results.json').read_text())
            self.assertEqual(len(results),2)
            traces = []
            for result,design in zip(results,[self.nominal,self.candidate]):
                path = Path(result['csv'])
                meta = json.loads(path.with_suffix('.json').read_text())
                np.testing.assert_array_equal(meta['lqr_design']['K_duty'],design['K_duty'])
                self.assertEqual(meta['configuration']['sensor_seed'],3000)
                with path.open() as stream:
                    trace = [r for r in csv.DictReader(stream) if r['phase']=='free']
                pulse = [(float(r['t_free_s']),float(r['disturbance_force_world_y_N'])) for r in trace
                         if float(r['disturbance_force_world_y_N']) != 0]
                self.assertEqual(len(pulse),50)
                self.assertAlmostEqual(pulse[0][0],5.)
                self.assertTrue(all(force==.1 for _,force in pulse))
                traces.append(pulse)
            self.assertEqual(traces[0],traces[1])
            self.assertEqual(results[0]['actual_release_pitch_deg'],results[1]['actual_release_pitch_deg'])
            self.assertTrue((batch/'results.csv').exists())

    def test_path_uses_frozen_k_and_tracking_metrics(self):
        protocol = dict(mode='lqr-path',path_file=str(ROOT/'config/paths/curve_test.json'),
                        duration_s=27.,max_pwm=50.,release_pitch_deg=3.,feedback_delay_samples=1)
        with tempfile.TemporaryDirectory() as temp:
            _,trials = evaluate(self.nominal,scenarios(1,3100,'path'),protocol,Path(temp))
            trial = trials[0]
            self.assertIn('path_completed',trial)
            self.assertNotIn('stabilized',trial)
            self.assertIn('tracking_rms_position_m',trial)
            self.assertEqual(trial['cost'],trial['tracking_rms_position_m'])
            path = Path(trial['csv'])
            meta = json.loads(path.with_suffix('.json').read_text())
            np.testing.assert_array_equal(meta['lqr_design']['K_duty'],self.nominal['K_duty'])
            self.assertEqual(meta['configuration']['mode'],'lqr-path')
            self.assertEqual(meta['configuration']['feedback_delay_samples'],1)
            with path.open() as stream:
                trace = [r for r in csv.DictReader(stream) if r['phase']=='free']
            self.assertGreater(max(float(r['path_ref_y_m']) for r in trace),.8)


if __name__=='__main__':
    unittest.main()
