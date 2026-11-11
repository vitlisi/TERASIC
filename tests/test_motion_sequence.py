"""Compare the timed MuJoCo reference with the hardware phase schedule."""
import contextlib
import csv
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src_python'))
from motion_lqr import RealPathReference, design_motion_lqr
from mujoco_sim import parse_args, run

CONFIG = ROOT/'models/lqr/hardware_path_simple_40cm_28p5s_20261019.txt'


class SequenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design = design_motion_lqr()
        cls.settings = RealPathReference.read_settings(CONFIG)

    def test_reference_matches_cpp_timing_and_measured_phase_origins(self):
        # Non-ideal measured positions expose rebasing differences at transitions.
        times = [0, 1.49, 1.5, 5.5, 9.49, 9.5, 10.49, 10.5, 14,
                 17.49, 17.5, 18.49, 18.5, 22.5, 26.49, 26.5, 28.49, 28.5, 34]
        source = r'''
#include "src_cpp/terasic_reale/controllers/balance_lqr_path_simple_core.h"
#include <cstdio>
int main() {
    LqrPathModel m={0,0,0,0,0,0};
    LqrSimplePath p(28.5,.4,3.141592653589793,m,8,7,1.5,1,2);
    double t;
    while (std::scanf("%lf", &t)==1) {
        LqrMimoState s={{0,0,0,0,0,0}};s.x[0]=.02*std::sin(t);
        LqrPathReference r=p.update(t,s);
        std::printf("%.17g %.17g %.17g %.17g %d %d %.17g\n",
            r.x[0],r.x[2],r.x[3],r.x[5],r.phase,p.schedule_finished(t),p.planned_y(t));
    }
}
'''
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'reference.cpp';path.write_text(source)
            binary = Path(folder)/'reference'
            subprocess.run(['g++', '-std=c++98', '-O2', '-Wall', '-Wextra', '-Werror',
                            '-I', str(ROOT), str(path), '-o', str(binary)], check=True)
            result = subprocess.run([str(binary)], input='\n'.join(map(str, times)),
                                    capture_output=True, text=True, check=True)
        expected = np.array([[float(v) for v in line.split()] for line in result.stdout.splitlines()])
        reference = RealPathReference(self.design, self.settings)
        actual = []
        for t in times:
            s = .02*math.sin(t)
            state = np.array([-s/reference.radius, 0., 0., 0., 0., 0.])
            x, duty, info = reference.sample(t, 0., 0., np.zeros(2), state)
            self.assertTrue(np.isfinite(duty).all())
            self.assertAlmostEqual(info['velocity_ref_m_s'], reference.radius*(x[4]-x[3]))
            actual.append([info['distance_ref_m'], x[2], info['velocity_ref_m_s'], x[5],
                           info['sequence_phase'], info['sequence_finished'], info['planned_y_m']])
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)

    def test_cli_separates_sequence_and_stationary_reference(self):
        sequence = parse_args(['--mode', 'lqr-motion', '--motion-profile', 'real-path'])
        self.assertEqual(sequence.max_pwm, 50)
        stationary = parse_args(['--mode', 'lqr-motion', '--move-distance-m', '0'])
        self.assertEqual(stationary.motion_profile, 'straight')
        for flags in (['--mode', 'pd'], ['--mode', 'lqr-motion', '--duration', '30']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args([*flags, '--motion-profile', 'real-path'])

    def test_sensor_sequence_logs_phases_and_stops_motors_at_end(self):
        args = parse_args(['--mode', 'lqr-motion', '--motion-profile', 'real-path',
                          '--duration', '40', '--headless', '--imu-model', 'mpu6500',
                          '--encoder-model', 'motor'])
        with tempfile.TemporaryDirectory() as folder:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args, folder), 0)
            path = next(Path(folder).glob('*.csv'))
            with path.open() as stream:
                rows = [r for r in csv.DictReader(stream) if r['phase'] == 'free']
            status = json.loads(path.with_suffix('.status.log').read_text())
            metadata = json.loads(path.with_suffix('.json').read_text())
        self.assertEqual({int(r['sequence_phase']) for r in rows}, set(range(7)))
        self.assertEqual(next(float(r['t_free_s']) for r in rows if r['sequence_finished']=='1'), 28.5)
        self.assertAlmostEqual(float(rows[-1]['yaw_ref_rad']), math.pi)
        self.assertTrue(status['sequence_finished'])
        self.assertEqual(status['final_ctrl'], [0., 0.])
        self.assertIn('tail_max_world_return_error_m', status)
        self.assertIn('tail_max_yaw_error_deg', status)
        # Body-forward speed stays positive on the return leg, despite negative world Y speed.
        mid_return = next(r for r in rows if float(r['t_free_s']) == 22.5)
        self.assertGreater(float(mid_return['velocity_body_forward_m_s']), .05)
        self.assertLess(float(mid_return['velocity_world_forward_m_s']), -.05)
        self.assertTrue(all(abs(float(r[k]))<=50 for r in rows
                            for k in ('pwm_sx_applied_percent','pwm_dx_applied_percent')))
        self.assertEqual(metadata['schema_version'], 13)
        self.assertEqual(metadata['real_path_sequence']['duration_s'], 28.5)


if __name__ == '__main__':
    unittest.main()
