"""Prescribed-voltage test: torque scaling, fixed nominal design, log timing."""
import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from mujoco_robot import MujocoRobot
from mujoco_sim import parse_args,run


class VoltageTests(unittest.TestCase):
    def test_voltage_scales_actual_joint_torque_without_changing_damping(self):
        robot=MujocoRobot()
        damping=robot.m.dof_damping.copy()
        robot.set_motors(20,-30)
        for voltage in (12.6,10.5,12):
            robot.set_supply_voltage(voltage)
            mujoco.mj_forward(robot.m,robot.d)
            for joint,pwm in (('perno_sx',20),('perno_dx',-30)):
                dof=int(robot.m.joint(joint).dofadr[0])
                self.assertAlmostEqual(robot.d.qfrc_actuator[dof],-.441*voltage/12*pwm/100)
            np.testing.assert_array_equal(robot.m.dof_damping,damping)
        for voltage in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):robot.set_supply_voltage(voltage)

    def test_voltage_profile_and_nominal_lqr_provenance(self):
        with tempfile.TemporaryDirectory() as folder:
            args=parse_args(['--mode','lqr','--headless','--duration','2',
                '--battery-voltage-v','12.6','--battery-final-voltage-v','10.5'])
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(run(args,folder),0)
            self.assertIn('tau SX=',output.getvalue())
            p=next(Path(folder).glob('*.csv'))
            with p.open() as stream:rows=list(csv.DictReader(stream))
            self.assertTrue(all(float(row['battery_voltage_v'])==12.6
                                for row in rows if row['phase']!='free'))
            for row in rows:
                voltage=float(row['battery_voltage_v']);pwm=float(row['pwm_applied_percent'])
                self.assertAlmostEqual(float(row['tau_sx_Nm']),-.441*voltage/12*pwm/100)
            free=[row for row in rows if row['phase']=='free']
            self.assertAlmostEqual(float(free[0]['battery_voltage_v']),12.6)
            self.assertAlmostEqual(float(free[-1]['battery_voltage_v']),10.5105)
            metadata=json.loads(p.with_suffix('.json').read_text())
            self.assertEqual(metadata['schema_version'],7)
            self.assertEqual(metadata['battery_model']['lqr_design_voltage_v'],12)
            status=json.loads(p.with_suffix('.status.log').read_text())
            self.assertAlmostEqual(status['final_battery_voltage_v'],10.5)
            self.assertEqual(status['final_ctrl'],[0,0])

    def test_invalid_voltage_arguments(self):
        for flags in (['--battery-voltage-v','0'],['--battery-final-voltage-v','-1'],
                      ['--battery-voltage-v','nan']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                parse_args(flags)
