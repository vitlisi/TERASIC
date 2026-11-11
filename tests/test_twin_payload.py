"""Check aggregate mass moments and preservation of the nominal model."""
import sys
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from digital_twin import MujocoRobot, digest
from twin_payload import add_centered_payload, body_inertia


def moment(center):
    return np.dot(center, center)*np.eye(3)-np.outer(center, center)


class PayloadTest(unittest.TestCase):
    def test_mass_first_moment_and_inertia_about_origin(self):
        robot = MujocoRobot()
        index = robot.body_id
        before_hash = digest(ROOT/'models/verifica.xml')
        old_mass = float(robot.m.body_mass[index])
        old_center = robot.m.body_ipos[index].copy()
        old_origin_inertia = body_inertia(robot.m, index)+old_mass*moment(old_center)
        payload = np.array([0., 0., .14])
        add_centered_payload(robot, .02, .14)
        new_mass = float(robot.m.body_mass[index])
        new_center = robot.m.body_ipos[index]
        self.assertAlmostEqual(new_mass, old_mass+.02)
        np.testing.assert_allclose(new_mass*new_center, old_mass*old_center+.02*payload, atol=1e-14)
        np.testing.assert_allclose(body_inertia(robot.m, index)+new_mass*moment(new_center),
                                   old_origin_inertia+.02*moment(payload), atol=1e-14)
        self.assertEqual(before_hash, digest(ROOT/'models/verifica.xml'))
        np.testing.assert_array_equal(robot.d.ctrl, 0.)

    def test_zero_payload_is_exact_noop(self):
        robot = MujocoRobot()
        fields = ['body_mass', 'body_ipos', 'body_inertia', 'body_iquat']
        before = {key: getattr(robot.m, key).copy() for key in fields}
        add_centered_payload(robot, 0., .10)
        for key in fields:
            np.testing.assert_array_equal(before[key], getattr(robot.m, key))


if __name__ == '__main__':
    unittest.main()
