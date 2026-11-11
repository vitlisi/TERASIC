"""Numerical design consistency and full nonlinear sensor/control integration."""
import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src_python'))
from lqr_controller import design_lqr, solve_dlqr, LqrController, FULL_STATE_ORDER
from balance_controller import ControllerConfig
from mujoco_sim import parse_args, run


class LqrTests(unittest.TestCase):
    def test_step_load_schedule_recovery_and_cleanup_both_signs(self):
        for torque in (.01,-.01):
            with tempfile.TemporaryDirectory() as folder:
                args=parse_args(['--mode','lqr','--headless','--duration','9',
                    '--disturbance-torque-nm',str(torque),'--disturbance-start-s','2'])
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(run(args,folder),0)
                p=next(Path(folder).glob('*.csv'))
                with p.open() as stream:
                    rows=list(csv.DictReader(stream))
                active=[r for r in rows if float(r['disturbance_pitch_torque_Nm']) != 0]
                self.assertEqual(len(active),50)
                self.assertTrue(all(r['phase']=='free' for r in active))
                self.assertAlmostEqual(float(active[0]['t_free_s']),2)
                self.assertAlmostEqual(float(active[-1]['t_free_s']),2.49)
                self.assertTrue(all(float(r['disturbance_pitch_torque_Nm'])==torque for r in active))
                status=json.loads(p.with_suffix('.status.log').read_text())
                self.assertTrue(status['disturbance_recovered'])
                self.assertGreater(status['disturbance_peak_pitch_error_deg'],.5)
                self.assertEqual(status['final_disturbance_torque_Nm'],0)
                self.assertEqual(status['final_ctrl'],[0,0])
                self.assertEqual(json.loads(p.with_suffix('.json').read_text())['schema_version'],6)

    def test_invalid_step_arguments(self):
        for options in (['--disturbance-torque-nm','nan'],['--disturbance-start-s','-1'],
                        ['--disturbance-duration-s','0'],
                        ['--duration','5','--disturbance-torque-nm','.01'],
                        ['--disturbance-torque-nm','.01','--disturbance-duration-s','.001']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                parse_args(options)

    def test_full_state_order_and_reduced_feedback(self):
        design=design_lqr()
        self.assertEqual(FULL_STATE_ORDER,['theta','psi','rho','theta_dot','psi_dot','rho_dot'])
        control=LqrController(ControllerConfig(),design)
        control.reset(0)
        info=control.update(2,3,4,6,.01,ideal_pitch_deg=2,wheel_positions=(10,12))
        np.testing.assert_allclose(control.x[[0,1,3,4]],[11,np.radians(2),5,np.radians(3)])
        self.assertTrue(np.isnan(control.x[[2,5]]).all())
        self.assertTrue(np.isfinite(info['pwm_requested_percent']))
        self.assertFalse(info['yaw_valid'])
        # Even populated yaw slots cannot affect the common longitudinal PWM.
        control.x[2]=100; control.x[5]=-200
        again=control.update(2,3,4,6,.01,ideal_pitch_deg=2,wheel_positions=(10,12))
        self.assertEqual(info['pwm_requested_percent'],again['pwm_requested_percent'])
    def test_derivatives_converge_and_riccati_stabilizes(self):
        designs = [design_lqr(epsilon=e) for e in (1e-4,5e-5)]
        for key in ('Ad','Bd_duty','K_duty'):
            np.testing.assert_allclose(designs[0][key],designs[1][key],rtol=2e-3,atol=2e-5)
        d = designs[0]
        a,b,q,r = (np.array(d[k]) for k in ('Ad','Bd_duty','Q','R'))
        k,poles = solve_dlqr(a,b,q,r)
        self.assertTrue(np.all(np.abs(poles)<1))
        self.assertTrue(np.max(np.abs(np.linalg.eigvals(a)))>1)
        self.assertLess(np.max(np.abs(d['nominal_step_residual'])),1e-4)
        self.assertFalse(d['simscape_comparison']['used_for_gains'])

    def test_nonlinear_imu_release_and_zero_pwm_limit(self):
        for max_pwm,duration in ((75,12),(0,.05)):
            with tempfile.TemporaryDirectory() as folder:
                args=parse_args(['--mode','lqr','--headless','--duration',str(duration),
                                 '--release-pitch-deg','1','--max-pwm',str(max_pwm)])
                with contextlib.redirect_stdout(io.StringIO()):
                    result=run(args,folder)
                self.assertEqual(result,0)
                status=json.loads(next(Path(folder).glob('*.status.log')).read_text())
                metadata=json.loads(next(Path(folder).glob('*.json')).read_text())
                self.assertEqual(status['final_ctrl'],[0,0])
                self.assertEqual(metadata['schema_version'],5)
                self.assertIn('src_python/lqr_controller.py',metadata['sha256'])
                self.assertTrue(metadata['runtime_motor_override'])
                profile=metadata['motor_friction_profile']
                self.assertAlmostEqual(metadata['physical_parameters']['joint_frictionloss_Nm'][-1],
                                       profile['wheels']['dx']['off_frictionloss_Nm'])
                if max_pwm:
                    self.assertTrue(status['settled'])
                else:
                    self.assertEqual(status['max_abs_pwm_percent'],0)

    def test_historical_motor_model_requires_explicit_opt_out(self):
        current=parse_args(['--headless'])
        self.assertEqual(Path(current.motor_friction_profile).name,'mujoco_robot01_stribeck.json')
        historical=parse_args(['--headless','--legacy-motor-friction'])
        self.assertIsNone(historical.motor_friction_profile)
