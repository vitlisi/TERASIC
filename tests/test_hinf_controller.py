"""Imported MATLAB controller: conventions, delay and nonlinear hold/rejection."""
import contextlib
import csv
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from hinf_controller import DiscreteHinf, STATE_TRANSFORM
from motion_lqr import design_motion_lqr
from mujoco_sim import parse_args, run
from mujoco_robot import MujocoRobot

CONFIG = ROOT/'models/hinf/simscape_2pwm_20261026.json'


class HinfTests(unittest.TestCase):
    def test_delay_output_sign_order_and_reset(self):
        c = DiscreteHinf(CONFIG)
        error = np.array([-.625, .02, .05, -.1, .03, -.02])
        # Positive travelled distance: thetaS=psiM-thetaM. Right wheel first
        # in the export; left first in MuJoCo. The first measurement is delayed.
        np.testing.assert_allclose(c.step(error,.01),0)
        analytic_error = np.array([.645,.02,.05,.13,.03,-.02])
        np.testing.assert_allclose(c.delayed,analytic_error)
        np.testing.assert_allclose(c.step(error,.01),
                                   (c.d@analytic_error)[::-1],atol=1e-14)
        c.reset()
        np.testing.assert_array_equal(c.state,np.zeros(12))
        np.testing.assert_array_equal(c.step(error,.01),np.zeros(2))
        with self.assertRaises(ValueError): c.step(error,.02)
        with self.assertRaises(ValueError): c.step(np.full(6,np.nan),.01)

    def test_export_rejects_wrong_units_and_malformed_matrices(self):
        original = json.loads(CONFIG.read_text())
        for key,value in [('output_order',['PWM_L','PWM_R']),('output_unit','percent'),
                          ('measurement_delay_samples',0),('Ak',[[1.]]),
                          ('controller_initial_state',[0.]),('Bk',[[float('nan')]*6]*12)]:
            with self.subTest(key=key),tempfile.TemporaryDirectory() as folder:
                p = Path(folder)/'bad.json'
                p.write_text(json.dumps(dict(original,**{key:value})))
                with self.assertRaises(ValueError): DiscreteHinf(p)

    def test_native_local_closed_loop_and_wheel_order(self):
        native = design_motion_lqr()
        t = STATE_TRANSFORM
        a = t@np.array(native['Ad'])@np.linalg.inv(t)
        b = t@np.array(native['Bd_duty'])@np.fliplr(np.eye(2))
        aa = np.block([[a,np.zeros((6,6))],[np.eye(6),np.zeros((6,6))]])
        bu = np.vstack([b,np.zeros((6,2))]);cy = np.hstack([np.zeros((6,6)),np.eye(6)])
        c = DiscreteHinf(CONFIG)
        def radius(input_matrix):
            cl = np.block([[aa+input_matrix@c.d@cy,input_matrix@c.c],[c.b@cy,c.a]])
            return max(abs(np.linalg.eigvals(cl)))
        self.assertLess(radius(bu),1.)
        self.assertGreater(radius(bu@np.fliplr(np.eye(2))),1.)

    def test_stabilization_only_cli(self):
        for extra in [['--move-distance-m=.1'],['--desired-velocity','.1'],
                      ['--max-pwm','101'],['--feedback-delay-samples','0'],
                      ['--disturbance-period-s','.5'],['--disturbance-period-s','-1'],
                      ['--motion-profile','real-path']]:
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                parse_args(['--mode','hinf',*extra])
        a = parse_args(['--mode','hinf'])
        self.assertEqual(a.max_pwm,15)
        self.assertEqual(a.move_distance_m,0)
        higher=parse_args(['--mode','hinf','--max-pwm','50'])
        self.assertEqual(higher.max_pwm,50)
        self.assertEqual(DiscreteHinf(higher.hinf_config).limit,.15)

    def simulate(self, extra, mode='hinf'):
        with tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--mode',mode,'--headless','--duration','30',
                '--imu-model','mpu6500','--encoder-model','motor',*extra])
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args,folder),0)
            p = next(Path(folder).glob('*.csv'))
            with p.open() as stream: rows = list(csv.DictReader(stream))
            metadata = json.loads(p.with_suffix('.json').read_text())
            status = json.loads(p.with_suffix('.status.log').read_text())
        self.assertTrue(status['settled'],status)
        self.assertEqual(status['final_ctrl'],[0.,0.])
        free = [r for r in rows if r['phase']=='free']
        self.assertTrue(all(abs(float(r[k])) <= 15 for r in free
            for k in ['pwm_sx_applied_percent','pwm_dx_applied_percent']))
        return status,metadata,free

    def test_sensor_hold_release_perturbation_and_telemetry(self):
        s,m,r = self.simulate(['--release-pitch-deg','1'])
        self.assertTrue(s['stabilization_completed'])
        self.assertLess(s['tail_max_world_distance_error_m'],.01)
        self.assertEqual(m['schema_version'],14)
        self.assertNotIn('lqr_design',m)
        self.assertEqual(m['hinf_design'],json.loads(CONFIG.read_text()))
        self.assertEqual(m['feedback_delay_samples'],1)
        self.assertTrue(all(float(r[0]['hinf_feedback_'+str(i)])==0 for i in range(6)))
        self.assertTrue(all(math.isfinite(float(row['hinf_xc_11'])) for row in r))
        np.testing.assert_allclose([float(r[1]['hinf_feedback_'+str(i)]) for i in range(6)],
            STATE_TRANSFORM@np.array([float(r[0][k]) for k in
                ['error_theta_rad','error_psi_rad','error_rho_rad',
                 'error_theta_dot_rad_s','error_psi_dot_rad_s','error_rho_dot_rad_s']]))

    def test_body_force_rejection_and_cleanup(self):
        seen = []
        original = MujocoRobot.step
        def step(robot):
            seen.append(float(robot.d.xfrc_applied[robot.body_id,1]))
            original(robot)
        with patch.object(MujocoRobot,'step',step):
            s,m,r = self.simulate(['--disturbance-force-y-n','.1'])
        self.assertTrue(s['stabilization_completed'])
        self.assertTrue(s['disturbance_recovered'])
        self.assertIn(.1,seen)
        self.assertEqual(s['final_disturbance_force_y_N'],0)
        active = [row for row in r if float(row['disturbance_force_world_y_N']) != 0]
        self.assertEqual(len(active),50)
        self.assertAlmostEqual(float(active[0]['t_free_s']),5)
        self.assertEqual(m['external_force']['application_point'],'body center of mass')

    def test_lqr_comparison_same_cap_and_delay(self):
        s,m,_ = self.simulate(['--move-distance-m','0','--max-pwm','15',
            '--feedback-delay-samples','1','--disturbance-force-y-n','.1'],mode='lqr-motion')
        self.assertEqual(m['feedback_delay_samples'],1)
        self.assertEqual(m['controller']['max_pwm'],15)
        self.assertTrue(s['motion_completed'])

    def test_repeated_force_windows_and_recovery_boundaries(self):
        with tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--mode','hinf','--headless','--duration','16',
                '--disturbance-force-y-n','.1','--disturbance-period-s','5'])
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args,folder),0)
            p = next(Path(folder).glob('*.csv'))
            with p.open() as stream: rows = list(csv.DictReader(stream))
            metadata = json.loads(p.with_suffix('.json').read_text())
            status = json.loads(p.with_suffix('.status.log').read_text())
        windows = [[5.,5.5],[10.,10.5],[15.,15.5]]
        self.assertEqual(metadata['disturbance_schedule']['windows_free_s'],windows)
        for row in rows:
            if row['phase'] != 'free': continue
            t = float(row['t_free_s'])
            expected = .1 if any(start <= t < end for start,end in windows) else 0.
            self.assertEqual(float(row['disturbance_force_world_y_N']),expected)
        self.assertEqual(len(status['disturbance_pulses']),3)
        # Last pulse leaves only 0.5 s: cannot establish a full 1 s of recovery.
        self.assertFalse(status['disturbance_pulses'][-1]['recovered'])
        self.assertEqual(status['final_disturbance_force_y_N'],0.)
