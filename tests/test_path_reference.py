"""Spline continuity, pose guidance, decorative scene and nonlinear tracking."""
import contextlib
import copy
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src_python'))
from path_reference import PathPlan, PathReference, draw_ground_path
from motion_lqr import design_motion_lqr
from mujoco_robot import MujocoRobot
from mujoco_sim import parse_args, run
CONFIG = dict(schema_version=1,units='m',frame='local_to_release',start_s=3.,
    travel_duration_s=30.,end_heading_deg=0.,
    waypoints_xy_m=[[0,0],[0,.15],[-.08,.35],[-.08,.55],[0,.75],[0,.9]])


class PathTests(unittest.TestCase):
    def test_spline_interpolates_points_and_has_continuous_tangents_curvature(self):
        plan = PathPlan(CONFIG)
        np.testing.assert_allclose(plan.geometry(plan.knots)[0], plan.points, atol=1e-12)
        for knot in plan.knots[1:-1]:
            for derivative in (0, 1, 2):
                np.testing.assert_allclose(plan.geometry(knot-1e-9)[derivative],
                    plan.geometry(knot+1e-9)[derivative], atol=1e-6)
        before, after = plan.trajectory(0), plan.trajectory(100)
        np.testing.assert_allclose(before['xy'], [0, 0])
        np.testing.assert_allclose(after['xy'], [0, .9], atol=1e-12)
        self.assertEqual(float(before['speed']), 0)
        self.assertEqual(float(after['speed']), 0)
        self.assertEqual(float(after['yaw_rate']), 0)
        self.assertLess(np.max(plan.samples['speed']), .12)
        self.assertLess(np.min(plan.samples['yaw_rate']), -.05)
        self.assertGreater(np.max(plan.samples['yaw_rate']), .05)

    def test_heading_unwrap_across_pi(self):
        config = copy.deepcopy(CONFIG)
        config.update(waypoints_xy_m=[[0,0],[0,.2],[-.2,.2],[-.2,0],[0,0]],
                      end_heading_deg=360., travel_duration_s=60.)
        plan = PathPlan(config)
        self.assertAlmostEqual(float(plan.heading_grid[-1]), 2*math.pi)
        self.assertLess(np.max(np.abs(np.diff(plan.heading_grid))), .1)

    def test_visual_scene_does_not_modify_physics(self):
        robot, plan = MujocoRobot(), PathPlan(CONFIG)
        state, mass, geoms = robot.d.qpos.copy(), robot.m.body_mass.copy(), robot.m.ngeom
        scene = mujoco.MjvScene(robot.m, maxgeom=1000)
        draw_ground_path(scene, plan, origin_xy=[1, 2], heading=math.pi/2, free_time=18)
        self.assertGreater(scene.ngeom, 100)
        self.assertEqual(robot.m.ngeom, geoms)
        np.testing.assert_array_equal(robot.d.qpos, state)
        np.testing.assert_array_equal(robot.m.body_mass, mass)
        self.assertTrue(all(scene.geoms[i].category == mujoco.mjtCatBit.mjCAT_DECOR for i in range(scene.ngeom)))
        np.testing.assert_allclose(scene.geoms[scene.ngeom-3].pos[:2], [1, 2])
        np.testing.assert_allclose(scene.geoms[scene.ngeom-2].pos[:2], [.1, 2], atol=1e-6)

    def test_guidance_uses_estimated_pose_with_correct_lateral_sign(self):
        design = design_motion_lqr()
        plan = PathPlan(CONFIG)
        ref = PathReference(design, plan)
        state = np.array([0, ref.equilibrium, 0, 0, 0, 0])
        xref, duty, info = ref.sample(0, 0, ref.equilibrium, [.02, 0], state)
        self.assertGreater(info['path_heading_correction_rad'], 0)  # robot right of reference: turn left
        self.assertGreater(xref[2], 0)
        self.assertEqual(duty.shape, (2,))
        # A forward position offset directly changes the longitudinal error.
        xref, _, info = ref.sample(0, 0, ref.equilibrium, [0, .02], state)
        self.assertLess(info['path_error_forward_m'], 0)
        self.assertGreater(xref[0], 0)

    def test_bad_paths_rejected_before_simulation(self):
        for changes in ({'waypoints_xy_m': [[0,0],[0,0]]}, {'travel_duration_s': 0},
                        {'waypoints_xy_m': [[1,0],[1,.2]]}, {'start_s': float('nan')},
                        {'waypoints_xy_m': [[0,0],[0,float('inf')]]}):
            with self.assertRaises(ValueError):
                PathPlan(dict(CONFIG, **changes))
        self.assertEqual(parse_args(['--mode', 'lqr-path', '--duration', '20']).duration, 20)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(['--mode', 'lqr-path', '--path-file', '/tmp/absent_terasic_path.json'])

    def test_duration_and_optional_speed_limits_for_stress_runs(self):
        plan = PathPlan(dict(CONFIG,travel_duration_s=.5))
        self.assertGreater(np.max(plan.samples['speed']), .15)
        self.assertGreater(np.max(np.abs(plan.samples['yaw_rate'])), math.radians(60))
        self.assertIsNone(plan.max_speed)
        enabled = dict(CONFIG,travel_duration_s=5.,limits=dict(max_speed_m_s=.5,max_yaw_rate_deg_s=100.))
        self.assertGreater(np.max(PathPlan(enabled).samples['speed']), .15)
        with self.assertRaises(ValueError):
            PathPlan(dict(enabled,limits=dict(max_speed_m_s=.01)))
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'path.json'
            path.write_text(json.dumps(CONFIG))
            self.assertEqual(PathPlan.from_file(path,duration_override=5.).duration,5.)
            args=parse_args(['--mode','lqr-path','--path-file',str(path),'--path-duration-s','5','--duration','12'])
            self.assertEqual(args.path_duration_s,5.)
            self.assertEqual(json.loads(path.read_text())['travel_duration_s'],30.)

    def test_noisy_sensors_battery_and_endpoint_hold(self):
        xml = ROOT/'models/verifica.xml'
        before = hashlib.sha256(xml.read_bytes()).hexdigest()
        with tempfile.TemporaryDirectory() as folder:
            fixture=Path(folder)/'path.json'
            fixture.write_text(json.dumps(CONFIG))
            args = parse_args(['--mode','lqr-path','--headless','--duration','40',
                '--path-file',str(fixture),'--imu-model','mpu6500','--encoder-model','motor','--battery-discharge'])
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args, folder), 0)
            p = next(Path(folder).glob('*.csv'))
            status = json.loads(p.with_suffix('.status.log').read_text())
            meta = json.loads(p.with_suffix('.json').read_text())
            with p.open() as stream:
                rows = [row for row in csv.DictReader(stream) if row['phase'] == 'free']
        self.assertTrue(status['path_completed'], status)
        self.assertEqual(meta['schema_version'], 12)
        self.assertEqual(status['final_ctrl'], [0., 0.])
        self.assertLess(status['tracking_max_pose_error_true_m'], .02)
        self.assertLess(status['tail_max_pose_error_true_m'], .01)
        self.assertGreater(max(abs(float(row['pwm_sx_applied_percent'])-float(row['pwm_dx_applied_percent'])) for row in rows), 2)
        self.assertGreater(max(float(row['x_rho_rad']) for row in rows), .3)
        self.assertLess(min(float(row['x_rho_rad']) for row in rows), -.3)
        self.assertEqual(before, hashlib.sha256(xml.read_bytes()).hexdigest())
