"""Known plateau, stall and voltage fixtures for the static-point classifier."""
import csv
import tempfile
import unittest
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'diagnostics'))
from analyze_static_point import analyze

class StaticPointAnalysisTests(unittest.TestCase):
    def fixture(self,folder,voltage=11.55,stall=False,completed=True):
        (folder/'point.txt').write_text('wheel=sx\ndirection=forward\nmode=cold\npwm_magnitude_percent=5\nhold_s=3\nvoltage_reference_v=11.55\nvoltage_plateau_tolerance_v=0.1\n')
        (folder/'reale.config.txt').write_text('counts_per_rev 1325\n')
        (folder/'reale.status.log').write_text('final_control_sx=6 final_control_dx=6 result='+('COMPLETED' if completed else 'ABORTED'))
        rows=[]
        for i in range(701):
            t=i*.01;active=1<=t<4;distance=0 if stall else round(200*min(3,max(0,t-1)))
            rows.append(dict(t_s=t,stage=2 if active else 0 if t<1 else 3,pwm_requested_percent=5 if active else 0,
                enc_sx_raw=32768-distance,enc_dx_raw=32768,pwm_sx_applied_percent=5 if active else 0,
                pwm_dx_applied_percent=0,control_dx=6,battery_v=voltage,battery_age_s=.03))
        with (folder/'reale.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)

    def test_known_steady_plateau(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.fixture(p);result=analyze(p)
            self.assertTrue(result['accepted_static_point'])
            self.assertAlmostEqual(result['mean_velocity_counts_s'],200,places=3)

    def test_stall_is_not_an_invertible_moving_point(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.fixture(p,stall=True);result=analyze(p)
            self.assertFalse(result['accepted_static_point'])
            self.assertIn('no_resolved_regular_motion',result['rejection_reasons'])

    def test_voltage_outside_frozen_band_is_not_pooled(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.fixture(p,voltage=11.9);result=analyze(p)
            self.assertIn('voltage_outside_frozen_comparability_band',result['rejection_reasons'])

    def test_abort_cannot_be_called_a_valid_static_point(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.fixture(p,completed=False);result=analyze(p)
            self.assertFalse(result['accepted_static_point'])

if __name__=='__main__':unittest.main()
