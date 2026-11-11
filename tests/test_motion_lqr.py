"""MIMO design, sensor frame/odometry, and nonlinear 30 cm motion."""
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
sys.path.insert(0, str(ROOT/'src_python'))
from balance_controller import ControllerConfig
from lqr_controller import FULL_STATE_ORDER
from motion_lqr import (design_motion_lqr, StraightReference, MotionLqrController,
                        allocate_pwm)
from mujoco_robot import MujocoRobot
from mujoco_sim import parse_args, run


class MotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design = design_motion_lqr()

    def test_design_derivative_convergence_and_stability(self):
        other = design_motion_lqr(epsilon=5e-5)
        for key in ('Ad', 'Bd_duty', 'K_duty'):
            np.testing.assert_allclose(self.design[key], other[key], rtol=2e-3, atol=2e-5)
        a, b, k = (np.array(self.design[key]) for key in ('Ad', 'Bd_duty', 'K_duty'))
        self.assertEqual(a.shape, (6, 6))
        self.assertEqual(b.shape, (6, 2))
        self.assertEqual(k.shape, (2, 6))
        self.assertEqual(self.design['full_state_order'], FULL_STATE_ORDER)
        self.assertEqual(self.design['controllability_rank'], 6)
        self.assertLess(np.max(np.abs(np.linalg.eigvals(a-b@k))), 1)
        self.assertLess(np.max(np.abs(self.design['nominal_step_residual'])), 1e-4)
        np.testing.assert_array_equal(self.design['C'], np.eye(6))
        np.testing.assert_array_equal(self.design['D'], np.zeros((6, 2)))

    def test_reference_endpoint_units_and_smooth_distance(self):
        ref = StraightReference(self.design)
        eq = ref.equilibrium
        x0, u0, start = ref.sample(0, 4., eq)
        xf, uf, end = ref.sample(20, 4., eq)
        np.testing.assert_allclose(x0, [4., eq, 0., 0., 0., 0.])
        np.testing.assert_allclose(xf, [4.-9.375, eq, 0., 0., 0., 0.])
        np.testing.assert_allclose(u0, 0)
        np.testing.assert_allclose(uf, 0)
        self.assertEqual(start['distance_ref_m'], 0)
        self.assertEqual(end['distance_ref_m'], .3)
        for t in (3., 9.):
            _, v, a, _ = ref.profile(t)
            self.assertEqual(v, 0.)
            self.assertEqual(a, 0.)
        self.assertAlmostEqual(ref.profile(6)[1], .09375)

    def test_pwm_allocation_preserves_common_and_bounds_each_motor(self):
        c = ControllerConfig(max_pwm=75)
        limited, applied, _, _ = allocate_pwm([30., 110.], c)
        np.testing.assert_allclose(limited, [65., 75.])
        self.assertAlmostEqual(np.mean(applied), 70.)
        limited, _, _, _ = allocate_pwm([100., 120.], c)
        np.testing.assert_allclose(limited, [75., 75.])
        _, applied, _, _ = allocate_pwm([.1, -.1], c)
        np.testing.assert_array_equal(applied, [0., 0.])

    def test_pitch_and_heading_measurement_frames_and_odometry(self):
        robot = MujocoRobot()
        psi, rho, omega = .23, .7, .4
        cp, sp, cy, sy = math.cos(psi/2), math.sin(psi/2), math.cos(rho/2), math.sin(rho/2)
        robot.d.qpos[robot.root_q+3:robot.root_q+7] = [cy*cp, -cy*sp, -sy*sp, sy*cp]
        robot.d.qvel[robot.root_v+3:robot.root_v+6] = [0., -omega*math.sin(psi), omega*math.cos(psi)]
        import mujoco
        mujoco.mj_forward(robot.m, robot.d)
        self.assertAlmostEqual(math.radians(robot.true_pitch_deg()), psi)
        self.assertAlmostEqual(robot.true_yaw_rad(), rho)
        control = MotionLqrController(ControllerConfig(), self.design, StraightReference(self.design))
        control.reset(math.degrees(psi))
        info = control.update(math.degrees(psi), 0., 0., 0., .01,
            ideal_pitch_deg=math.degrees(psi), wheel_positions=(0., 0.), gyro_xyz_rad_s=robot.read_imu()[1])
        self.assertAlmostEqual(info['x_rho_dot_rad_s'], omega)
        # Pitch alone changes the relative wheel angle without translating on the ground.
        control.reset(0)
        control.update(0, 0, 0, 0, .01, ideal_pitch_deg=0, wheel_positions=(0, 0), gyro_xyz_rad_s=(0, 0, 0))
        info = control.update(math.degrees(.1), 0, 0, 0, .01, ideal_pitch_deg=math.degrees(.1),
            wheel_positions=(.1, .1), gyro_xyz_rad_s=(0, 0, 0))
        self.assertAlmostEqual(info['position_encoder_m'], 0.)

    def test_invalid_motion_arguments(self):
        for options in (['--move-duration-s', '0'], ['--move-distance-m', 'nan'],
                        ['--duration', '10'], ['--desired-velocity', '.1'],
                        ['--move-distance-m', '1']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(['--mode', 'lqr-motion', *options])

    def simulate(self, options):
        with tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--mode', 'lqr-motion', '--headless', '--duration', '20', *options])
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args, folder), 0)
            p = next(Path(folder).glob('*.csv'))
            with p.open() as stream:
                rows = list(csv.DictReader(stream))
            status = json.loads(p.with_suffix('.status.log').read_text())
            metadata = json.loads(p.with_suffix('.json').read_text())
        self.assertTrue(status['motion_completed'], status)
        self.assertEqual(status['final_ctrl'], [0., 0.])
        self.assertEqual(metadata['schema_version'], 11)
        free = [row for row in rows if row['phase'] == 'free']
        self.assertTrue(all(row['yaw_valid'] == 'True' for row in free))
        self.assertTrue(all(abs(float(row[name])) <= 75 for row in free
            for name in ('pwm_sx_applied_percent', 'pwm_dx_applied_percent')))
        return status, metadata, free

    def test_forward_30cm_with_sensor_models_and_battery(self):
        status, metadata, rows = self.simulate(['--imu-model', 'mpu6500', '--encoder-model', 'motor',
                                               '--battery-discharge'])
        self.assertLess(abs(status['final_pose_true_m'][1]-.30), .01)
        self.assertLess(status['final_battery_soc'], 1.)
        self.assertIn('gyro_bias_xyz_rad_s', metadata)
        self.assertTrue(all(row['imu_gz_raw'] for row in rows))
        self.assertEqual(float(rows[-1]['distance_ref_m']), .3)

    def test_backward_30cm_with_pitch_disturbance(self):
        status, _, _ = self.simulate(['--move-distance-m', '-.3', '--disturbance-torque-nm', '.01'])
        self.assertLess(abs(status['final_pose_true_m'][1]+.30), .01)
        self.assertTrue(status['disturbance_recovered'])

    def test_yaw_disturbance_demands_differential_motors_and_recovers(self):
        original = MujocoRobot.step
        def step_with_yaw_load(robot):
            free_time = float(robot.d.time)-2.2
            robot.d.xfrc_applied[robot.body_id, 5] = .02 if 4. <= free_time < 4.5 else 0.
            original(robot)
        with patch.object(MujocoRobot, 'step', step_with_yaw_load):
            status, _, rows = self.simulate([])
        affected = [row for row in rows if 4. <= float(row['t_free_s']) < 5.5]
        self.assertGreater(max(abs(float(row['pwm_differential_requested_percent'])) for row in affected), .5)
        self.assertGreater(max(abs(float(row['yaw_true_rad'])) for row in affected), math.radians(.1))
        self.assertLess(status['tail_max_yaw_error_deg'], 2.)
