"""Check experimental overrides do not change the nominal model or body mass."""
import sys
from pathlib import Path
import unittest
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from digital_twin import MujocoRobot, digest
from twin_parameters import apply_parameters


class ParametersTest(unittest.TestCase):
    def test_joint_inertia_override_keeps_geometry_mass_and_drive(self):
        robot=MujocoRobot()
        before=digest(ROOT/'models/verifica.xml')
        mass=robot.m.body_mass.copy();ipos=robot.m.body_ipos.copy()
        drive=robot.m.actuator_gear.copy();armature=robot.m.dof_armature.copy()
        indices=[int(robot.m.joint(n).dofadr[0]) for n in ('perno_sx','perno_dx')]
        apply_parameters(robot,{'armature_scale':.1})
        expected=armature.copy();expected[indices]*=.1
        np.testing.assert_array_equal(robot.m.dof_armature,expected)
        np.testing.assert_array_equal(robot.m.body_mass,mass)
        np.testing.assert_array_equal(robot.m.body_ipos,ipos)
        np.testing.assert_array_equal(robot.m.actuator_gear,drive)
        self.assertEqual(digest(ROOT/'models/verifica.xml'),before)

    def test_drive_scale_agrees_with_generalized_motor_torque(self):
        robot=MujocoRobot();robot.set_supply_voltage(12.)
        apply_parameters(robot,{'torque_scale':1.5})
        robot.set_motors(20,0)
        mujoco.mj_forward(robot.m,robot.d)
        index=int(robot.m.joint('perno_sx').dofadr[0])
        self.assertAlmostEqual(robot.d.qfrc_actuator[index],-.441*1.5*.2)
        self.assertEqual(robot.d.ctrl[1],0.)


if __name__=='__main__':unittest.main()
