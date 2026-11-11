"""Independent checks for equivalent wheel identification and MuJoCo profiles."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'diagnostics'))
sys.path.insert(0, str(ROOT/'scripts'))
import compare_freewheel as fw
from identify_freewheel import advance, predict, fit_wheel, profile
from digital_twin import MujocoRobot
from twin_parameters import apply_parameters
from wheel_dynamics import configure_wheels


class WheelIdentificationTest(unittest.TestCase):
    def test_exact_coulomb_stop_and_viscous_decay(self):
        position, speed = advance(0., 2., 2., 0., .001, 0., .002)
        self.assertAlmostEqual(speed, 0.)
        self.assertAlmostEqual(position, 1.)  # Deceleration 2 rad/s², stop at 1s.
        position, speed = advance(0., 2., .1, 0., .001, .01, 0.)
        self.assertAlmostEqual(speed, 2*math.exp(-1))
        self.assertAlmostEqual(position, .2*(1-math.exp(-1)))

    def test_absolute_scale_is_not_identifiable(self):
        t = np.arange(0, 4, .01)
        duty = np.where((t >= 1) & (t < 2), 15., 0.)
        voltage = np.full(len(t), 12.)
        values = np.array([.0008, .013, .012, .001, .007])
        a = predict(values, t, duty, voltage)
        b = predict(2*values, t, duty, voltage, coefficient=2*.441/12)
        np.testing.assert_allclose(a, b, atol=1e-12)

    def test_recovers_synthetic_separate_losses_from_two_pulses(self):
        t = np.arange(0, 10, .02)
        duty = np.where((t >= 1) & (t < 2), 10., np.where((t >= 6) & (t < 7), 15., 0.))
        voltage = np.full(len(t), 12.8)
        actual = [.0008, .012, .011, .0009, .007]
        measured, _ = predict(actual, t, duty, voltage)
        fitted = fit_wheel(t, duty, voltage, measured, switched=True)['best']
        np.testing.assert_allclose(fitted['parameters'], actual, rtol=.005, atol=1e-7)
        self.assertLess(fitted['RMSE_counts'], .05)

    def test_runtime_profiles_switch_each_wheel_independently(self):
        robot = MujocoRobot()
        body_mass = robot.m.body_mass.copy()
        geometry_inertia = robot.m.body_inertia.copy()
        before = fw.sha(fw.XML)
        a = profile([.0008, .012, .011, .0009, .007])
        b = profile([.0009, .013, .014, .0011, .008])
        apply_parameters(robot, {'wheels': {'sx': a, 'dx': b}})
        indices = [int(robot.m.joint('perno_'+w).dofadr[0]) for w in fw.WHEELS]
        robot.set_motors(10, 0)
        np.testing.assert_allclose(robot.m.dof_damping[indices], [.012, .0011])
        robot.set_motors(0, -15)
        np.testing.assert_allclose(robot.m.dof_damping[indices], [.0009, .013])
        mujoco.mj_forward(robot.m, robot.d)
        self.assertAlmostEqual(robot.d.qfrc_actuator[indices[1]], .441*.15)
        np.testing.assert_array_equal(robot.m.body_mass, body_mass)
        np.testing.assert_array_equal(robot.m.body_inertia, geometry_inertia)
        self.assertEqual(fw.sha(fw.XML), before)

    def test_rejects_invalid_profile_before_mutation(self):
        model, _, _, _ = fw.fixed_model()
        before = model.dof_armature.copy()
        with self.assertRaises(ValueError):
            configure_wheels(model, {'sx': {'armature_kg_m2': .001}, 'dx': {'unknown': 1}})
        np.testing.assert_array_equal(before, model.dof_armature)


if __name__ == '__main__':
    unittest.main()
