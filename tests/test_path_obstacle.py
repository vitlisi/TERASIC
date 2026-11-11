"""Optional path target, frame anchoring, sensor visibility and no-contact baseline."""
import contextlib
import csv
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
from mujoco_robot import MujocoRobot
from mujoco_sim import parse_args, run
from path_reference import PathPlan

CONFIG = dict(schema_version=1, units='m', frame='local_to_release',
              start_s=1., travel_duration_s=20., waypoints_xy_m=[[0, 0], [0, 1]])


class PathObstacleTests(unittest.TestCase):
    def test_arc_position_lateral_offset_and_rotated_release_frame(self):
        plan = PathPlan(CONFIG)
        obstacle = dict(plan.obstacle_pose(.5, .1), collision=False, size_m=[1., .02, .6])
        np.testing.assert_allclose(obstacle['local_xy_m'], [-.1, .5], atol=1e-10)
        robot = MujocoRobot(path_obstacle=obstacle)
        q = robot.d.qpos.copy()
        pose = robot.place_path_obstacle([1., 2.], math.pi/2)
        np.testing.assert_allclose(pose['world_xy_m'], [.5, 1.9])
        np.testing.assert_array_equal(robot.d.qpos, q)
        gid = robot.m.geom('path_obstacle_geom').id
        np.testing.assert_allclose(robot.d.geom_xpos[gid], [.5, 1.9, .3])

    def test_noncolliding_target_visible_without_changing_dynamics(self):
        base = MujocoRobot()
        robot = MujocoRobot(path_obstacle=dict(PathPlan(CONFIG).obstacle_pose(.5), collision=False, size_m=[1., .02, .6]))
        target = robot.m.geom('path_obstacle_geom')
        self.assertEqual(int(robot.m.geom_contype[target.id]), 0)
        self.assertEqual(int(robot.m.geom_conaffinity[target.id]), 0)
        mujoco.mj_forward(robot.m, robot.d)
        reading = robot.read_ultrasonic()
        self.assertEqual(reading['ultrasonic_hit_geom_id'], target.id)
        self.assertAlmostEqual(reading['ultrasonic_distance_m'], .5-.01-.054)
        for _ in range(100):
            base.set_motors(5, 5)
            robot.set_motors(5, 5)
            base.step()
            robot.step()
        np.testing.assert_array_equal(robot.d.qpos, base.d.qpos)
        np.testing.assert_array_equal(robot.d.qvel, base.d.qvel)

    def test_collision_requires_explicit_option(self):
        robot = MujocoRobot(path_obstacle=dict(PathPlan(CONFIG).obstacle_pose(.5), collision=True))
        gid = robot.m.geom('path_obstacle_geom').id
        self.assertEqual(int(robot.m.geom_contype[gid]), 1)
        self.assertEqual(int(robot.m.geom_conaffinity[gid]), 1)
        robot.d.qpos[robot.root_q+1] = .46
        mujoco.mj_forward(robot.m, robot.d)
        self.assertTrue(any(gid in (contact.geom1, contact.geom2) for contact in robot.d.contact))

    def test_small_default_and_latched_hold_reference(self):
        from motion_lqr import design_motion_lqr
        from path_reference import PathReference
        robot = MujocoRobot(path_obstacle=dict(PathPlan(CONFIG).obstacle_pose(.5), collision=False))
        np.testing.assert_allclose(robot.m.geom('path_obstacle_geom').size, [.1, .02, .1])
        design = design_motion_lqr()
        ref = PathReference(design, PathPlan(CONFIG))
        state = np.array([.3, ref.equilibrium+.01, .2, -.1, 0., .1])
        pose = np.array([.02, .3])
        ref.request_stop(10., .098)
        xref, duty, info = ref.sample(10., 0., ref.equilibrium, pose, state)
        self.assertEqual(info['ultrasonic_stop_active'], 1)
        self.assertEqual(info['velocity_ref_m_s'], 0)
        np.testing.assert_array_equal(xref[3:], [0, 0, 0])
        np.testing.assert_array_equal(duty, [0, 0])
        ref.request_stop(11., .05)
        later, _, _ = ref.sample(12., 0., ref.equilibrium, pose+.1, state+.1)
        np.testing.assert_array_equal(later, xref)
        self.assertEqual(ref.stop_request['time_s'], 10.)

    def test_cli_guards(self):
        for flags in (['--path-obstacle'], ['--path-obstacle-collision'],
                      ['--mode', 'lqr-path', '--path-obstacle', '--path-obstacle-progress', '0'],
                      ['--mode', 'lqr-path', '--path-obstacle', '--ultrasonic-test-distance-m', '.5'],
                      ['--ultrasonic-stop-distance-m', '.1'],
                      ['--mode', 'lqr-path', '--ultrasonic-stop-distance-m', '0'],
                      ['--path-obstacle-width-m', '0']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(flags)
        self.assertFalse(parse_args([]).path_obstacle)

    def test_sensor_stop_with_collision_keeps_balance_without_contact(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'path.json'
            path.write_text(json.dumps(CONFIG))
            args = parse_args(['--headless', '--mode', 'lqr-path', '--path-file', str(path),
                               '--duration', '20', '--path-obstacle', '--path-obstacle-collision',
                               '--ultrasonic-stop-distance-m', '.10',
                               '--imu-model', 'mpu6500', '--encoder-model', 'motor'])
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args, directory), 0)
            log = next(Path(directory).glob('*.csv'))
            status = json.loads(log.with_suffix('.status.log').read_text())
            self.assertTrue(status['ultrasonic_stop_triggered'])
            self.assertTrue(status['ultrasonic_hold_settled'])
            self.assertFalse(status['path_completed'])
            self.assertEqual(status['obstacle_contact_steps'], 0)
            self.assertLessEqual(status['ultrasonic_stop_trigger']['distance_m'], .1)
            self.assertGreater(status['minimum_valid_range_after_stop_m'], .03)
            with log.open() as stream:
                rows = list(csv.DictReader(stream))
            held = [r for r in rows if r['ultrasonic_stop_active'] == '1']
            self.assertTrue(held)
            self.assertTrue(all(float(r['velocity_ref_m_s']) == 0 for r in held))
            self.assertEqual(len(set(r['distance_ref_m'] for r in held)), 1)
            self.assertTrue(any(abs(float(r['pwm_sx_applied_percent'])) > .5 for r in held))

    def test_runner_optional_target_metadata_and_detection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'path.json'
            path.write_text(json.dumps(CONFIG))
            args = parse_args(['--headless', '--mode', 'lqr-path', '--path-file', str(path),
                               '--duration', '.1', '--path-obstacle', '--path-obstacle-progress', '.5'])
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args, directory), 0)
            log = next(Path(directory).glob('*.csv'))
            meta = json.loads(log.with_suffix('.json').read_text())
            self.assertFalse(meta['path_obstacle']['collision'])
            self.assertFalse(meta['path_obstacle']['feedback_enabled'])
            with log.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(any(int(row['ultrasonic_hit_geom_id']) == meta['path_obstacle']['geom_id']
                                for row in rows if row['phase'] == 'free'))
            self.assertEqual(json.loads(log.with_suffix('.status.log').read_text())['final_ctrl'], [0, 0])


if __name__ == '__main__':
    unittest.main()
