"""Offline CLI and encoder calibration tests; never invoke hardware modes validly."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('encoder_cal', ROOT/'scripts/analyze_lqr_encoder.py')
encoder_cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(encoder_cal)


class EncoderCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)

    def encoder_csv(self, *, armed=0, other_motion=0, saturated=False, unstable=False):
        path = self.folder/'encoder.csv'
        with path.open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['armed','pwm_applied_percent','enc_sx_raw','enc_dx_raw'])
            for i in range(200):
                sx = 32768-min(100,max(0,i-50))*10
                dx = 32768-other_motion*(i//100)
                if saturated and i==100: sx=65535
                if unstable and i<30: sx+=i
                writer.writerow([armed, armed*5, sx, dx])
        return path

    def test_encoder_uses_delta_not_32768_as_cpr(self):
        result = encoder_cal.analyze(self.encoder_csv(), 'sx', 1, 'forward')
        self.assertTrue(result['checks_passed'])
        self.assertEqual(result['counts_per_rev_candidate'], 1000)
        self.assertEqual(result['delta_counts'], {'sx':-1000, 'dx':0})

    def test_bad_manual_trials_cannot_become_cpr(self):
        for kwargs in ({'other_motion':100}, {'unstable':True}):
            result = encoder_cal.analyze(self.encoder_csv(**kwargs), 'sx', 1, 'forward')
            self.assertFalse(result['checks_passed'])
            self.assertIsNone(result['counts_per_rev_candidate'])
        for kwargs in ({'armed':1}, {'saturated':True}):
            with self.assertRaises(ValueError):
                encoder_cal.analyze(self.encoder_csv(**kwargs), 'sx', 1, 'forward')
        result = encoder_cal.analyze(self.encoder_csv(), 'sx', 1, 'backward')
        self.assertFalse(result['checks_passed'])


if __name__ == '__main__':
    unittest.main()
