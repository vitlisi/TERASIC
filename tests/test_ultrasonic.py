"""Front HC-SR04 geometric approximation, range validity and runner telemetry."""
import contextlib
import csv
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src_python'))
from mujoco_robot import MujocoRobot, XML_PATH
from mujoco_sim import parse_args, run
from ultrasonic_model import UltrasonicCone, DEFAULT_CONFIG


class UltrasonicTests(unittest.TestCase):
    def isolated_target(self, distance=None):
        robot = MujocoRobot(ultrasonic_test_distance_m=distance)
        # Isolate the target test; the floor is tested independently below.
        robot.m.geom('ground').rgba[3] = 0
        robot.m.mat('grid_mat').rgba[3] = 0
        mujoco.mj_forward(robot.m, robot.d)
        return robot

    def test_known_distances_and_echo_conversion(self):
        for distance in (.02, .03, .1, .5, 1., 3.9, 4.):
            with self.subTest(distance=distance):
                robot = self.isolated_target(distance)
                mujoco.mj_forward(robot.m, robot.d)
                reading = robot.read_ultrasonic()
                self.assertEqual(reading['ultrasonic_valid'], 1)
                self.assertAlmostEqual(reading['ultrasonic_distance_m'], distance)
                self.assertAlmostEqual(reading['ultrasonic_echo_us'], 2*distance/340*1e6)
                sid = robot.m.site('ultrasonic_site').id
                np.testing.assert_allclose(robot.d.site_xmat[sid].reshape(3, 3)[:, 2], [0, 1, 0], atol=1e-12)

    def test_no_hit_near_and_far_are_invalid(self):
        for distance in (None, .01, 4.1):
            robot = self.isolated_target(distance)
            mujoco.mj_forward(robot.m, robot.d)
            reading = robot.read_ultrasonic()
            self.assertEqual(reading['ultrasonic_valid'], 0)
            self.assertTrue(math.isnan(reading['ultrasonic_distance_m']))
            self.assertTrue(math.isnan(reading['ultrasonic_echo_us']))
            if distance is None:
                self.assertEqual(reading['ultrasonic_ray_distance_m'], -1)

    def test_sample_hold_and_refresh(self):
        robot = MujocoRobot(ultrasonic_test_distance_m=.5)
        mujoco.mj_forward(robot.m, robot.d)
        self.assertAlmostEqual(robot.read_ultrasonic()['ultrasonic_distance_m'], .5)
        robot.d.qpos[robot.root_q+1] += .1
        robot.d.time = .06
        mujoco.mj_forward(robot.m, robot.d)
        reading = robot.read_ultrasonic()
        self.assertAlmostEqual(reading['ultrasonic_ray_distance_m'], .4)
        self.assertAlmostEqual(reading['ultrasonic_distance_m'], .5)
        robot.d.time = .07
        self.assertAlmostEqual(robot.read_ultrasonic()['ultrasonic_distance_m'], .4)

    def test_ray_follows_yaw_and_pitch(self):
        robot = self.isolated_target(.5)
        robot.d.qpos[robot.root_q+3:robot.root_q+7] = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
        mujoco.mj_forward(robot.m, robot.d)
        self.assertEqual(robot.read_ultrasonic()['ultrasonic_valid'], 0)
        robot = self.isolated_target(.5)
        robot.start_support(5)
        sid = robot.m.site('ultrasonic_site').id
        origin = robot.d.site_xpos[sid]
        direction = robot.d.site_xmat[sid].reshape(3, 3)[:, 2]
        expected = (.054+.5-origin[1])/direction[1]
        reading = robot.read_ultrasonic()
        self.assertAlmostEqual(reading['ultrasonic_ray_distance_m'], expected)
        self.assertLessEqual(reading['ultrasonic_distance_m'], expected)
        self.assertLessEqual(reading['ultrasonic_hit_angle_deg'], 7.5+1e-12)

    def test_off_axis_target_inside_outside_cone_and_half_angle_option(self):
        for angle, valid in ((5.625, True), (9., False)):
            robot = self.isolated_target(.5)
            sid = robot.m.site('ultrasonic_site').id
            rotation = robot.d.site_xmat[sid].reshape(3, 3)
            direction = rotation @ np.array([math.sin(math.radians(angle)), 0,
                                            math.cos(math.radians(angle))])
            target = robot.m.geom('ultrasonic_test_wall')
            robot.m.geom_type[target.id] = mujoco.mjtGeom.mjGEOM_SPHERE
            target.size[:] = [.003, 0, 0]
            target.pos[:] = robot.d.site_xpos[sid]+.5*direction
            # mj_multiRay uses a bounding sphere cache computed at compilation.
            robot.m.geom_rbound[target.id] = .003
            mujoco.mj_forward(robot.m, robot.d)
            reading = robot.read_ultrasonic()
            self.assertEqual(reading['ultrasonic_ray_distance_m'], -1)
            self.assertEqual(bool(reading['ultrasonic_valid']), valid)
            if valid:
                self.assertAlmostEqual(reading['ultrasonic_distance_m'], .497, places=7)
                self.assertAlmostEqual(reading['ultrasonic_hit_angle_deg'], angle)
            else:
                config = json.loads(DEFAULT_CONFIG.read_text())
                config['angle_interpretation'] = 'half_angle'
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory)/'cone.json'
                    path.write_text(json.dumps(config))
                    cone = UltrasonicCone(robot.m, path)
                    # Larger target spans the discrete 9.375-degree ring.
                    target.size[0] = robot.m.geom_rbound[target.id] = .01
                    mujoco.mj_forward(robot.m, robot.d)
                    self.assertEqual(cone.read(robot.m, robot.d)['ultrasonic_valid'], 1)

    def test_floor_is_detected_when_inside_cone(self):
        robot = MujocoRobot()
        mujoco.mj_forward(robot.m, robot.d)
        reading = robot.read_ultrasonic()
        self.assertEqual(reading['ultrasonic_ray_distance_m'], -1)
        self.assertEqual(reading['ultrasonic_hit_geom_id'], robot.m.geom('ground').id)
        self.assertAlmostEqual(reading['ultrasonic_distance_m'], .110/math.sin(math.radians(7.5)))

    def test_cone_grid_and_invalid_configuration(self):
        robot = MujocoRobot()
        cone = robot.ultrasonic_model
        self.assertEqual(len(cone.directions), 385)
        np.testing.assert_allclose(np.linalg.norm(cone.directions, axis=1), 1.)
        self.assertGreaterEqual(cone.directions[:, 2].min(), math.cos(math.radians(7.5))-1e-12)
        for key, value in [('angle_interpretation', 'unknown'), ('sample_period_s', .06),
                           ('radial_rings', 0), ('measuring_angle_deg', float('nan'))]:
            config = json.loads(DEFAULT_CONFIG.read_text())
            config[key] = value
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'bad.json'
                path.write_text(json.dumps(config))
                with self.assertRaises(ValueError):
                    UltrasonicCone(robot.m, path)

    def test_sensor_does_not_change_dynamics(self):
        robot = MujocoRobot()
        root = ET.parse(XML_PATH).getroot()
        root.find('compiler').set('meshdir', str(ROOT/'models/meshes'))
        root.find('sensor').remove(root.find(".//rangefinder[@name='ultrasonic_range']"))
        body = root.find(".//body[@name='struttura_centrale']")
        body.remove(body.find("site[@name='ultrasonic_site']"))
        baseline = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(baseline)
        np.testing.assert_array_equal(robot.m.body_mass, baseline.body_mass)
        np.testing.assert_array_equal(robot.m.body_inertia, baseline.body_inertia)
        for _ in range(50):
            robot.d.ctrl[:] = data.ctrl[:] = [.05, .05]
            mujoco.mj_step(robot.m, robot.d)
            mujoco.mj_step(baseline, data)
        np.testing.assert_array_equal(robot.d.qpos, data.qpos)
        np.testing.assert_array_equal(robot.d.qvel, data.qvel)

    def test_runner_logs_measurements_and_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            args = parse_args(['--headless', '--duration', '.1', '--ultrasonic-test-distance-m', '.5'])
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args, directory), 0)
            path = next(Path(directory).glob('*.csv'))
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(all(int(row['ultrasonic_valid']) == 1 for row in rows))
            self.assertTrue(all(.45 < float(row['ultrasonic_distance_m']) < .55 for row in rows))
            metadata = json.loads(path.with_suffix('.json').read_text())
            self.assertFalse(metadata['ultrasonic_model']['feedback_enabled'])
            status = json.loads(path.with_suffix('.status.log').read_text())
            self.assertEqual(status['final_ctrl'], [0, 0])


if __name__ == '__main__':
    unittest.main()
