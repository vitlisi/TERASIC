"""Charge accounting, motor energy consistency, sag and integration."""
import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from battery_model import BatteryPack,BatteryDepleted
from mujoco_sim import parse_args,run


def pack(power=5,soc=1):
    config=json.loads((ROOT/'config/mujoco_battery_3s_3000mah.json').read_text())
    config['electronics_power_w']=power
    return BatteryPack(config,soc,.441/12,.0126)


class BatteryTests(unittest.TestCase):
    def test_series_capacity_sag_and_exact_coulomb_accounting(self):
        battery=pack()
        self.assertEqual(battery.capacity_ah,3)
        self.assertAlmostEqual(battery.resistance,.051)
        load=battery.load([0,0],[0,0])
        self.assertAlmostEqual(load['battery_voltage_v'],load['battery_ocv_v']-battery.resistance*load['battery_current_A'])
        self.assertAlmostEqual(load['electronics_current_A']*load['battery_voltage_v'],5)
        battery.advance(60,load)
        expected=load['battery_current_A']*60/3600
        self.assertAlmostEqual(battery.soc,1-expected/3)
        self.assertAlmostEqual(battery.summary()['consumed_ah'],expected)

    def test_motor_load_increases_consumption_and_no_regenerative_charge(self):
        low=pack();high=pack()
        low_load=low.load([.1,.1],[0,0]);high_load=high.load([.8,.8],[0,0])
        self.assertGreater(high_load['battery_current_A'],low_load['battery_current_A'])
        self.assertLess(high_load['battery_voltage_v'],low_load['battery_voltage_v'])
        low.advance(10,low_load);high.advance(10,high_load)
        self.assertLess(high.soc,low.soc)
        regen=pack(0)
        load=regen.load([.1,.1],[100,100])
        self.assertEqual(load['battery_current_A'],0)
        regen.advance(1,load)
        self.assertEqual(regen.soc,1)

    def test_effective_circuit_matches_existing_mechanics_and_energy(self):
        battery=pack(0)
        for duty,rates in (([.5,-.3],[2,-3]),([0,0],[10,-10])):
            load=battery.load(duty,rates);voltage=load['battery_voltage_v']
            current=np.array([load['motor_sx_winding_current_A'],load['motor_dx_winding_current_A']])
            torque=battery.motor_k*current
            np.testing.assert_allclose(torque,(.441/12)*np.array(duty)*voltage-.0126*np.array(rates))
            np.testing.assert_allclose(np.array(duty)*voltage*current,
                torque*np.array(rates)+battery.motor_r*current**2,atol=1e-12)

    def test_cutoff_empty_and_unsustainable_load(self):
        for battery in (pack(soc=0),pack(power=100000)):
            with self.assertRaises(BatteryDepleted):battery.load([0,0],[0,0])
        battery=pack()
        load=battery.load([0,0],[0,0])
        battery.advance(100000,load)
        self.assertEqual(battery.soc,0)
        self.assertAlmostEqual(battery.summary()['consumed_ah'],3)

    def test_non_linear_ocv(self):
        battery=pack(0,soc=.5)
        mid=battery.load([0,0],[0,0])['battery_ocv_v']
        self.assertNotAlmostEqual(mid,(2.5+4.2)/2*3)

    def test_end_to_end_electronics_and_motor_consumption(self):
        with tempfile.TemporaryDirectory() as folder:
            args=parse_args(['--mode','lqr','--headless','--duration','2','--battery-discharge'])
            with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(run(args,folder),0)
            p=next(Path(folder).glob('*.csv'))
            with p.open() as stream:rows=list(csv.DictReader(stream))
            soc=np.array([float(row['battery_soc']) for row in rows])
            self.assertTrue(np.all(np.diff(soc)<=0))
            self.assertLess(soc[-1],soc[0])
            for row in rows:
                self.assertAlmostEqual(float(row['battery_current_A']),
                    float(row['electronics_current_A'])+float(row['motor_sx_battery_current_A'])+float(row['motor_dx_battery_current_A']))
                self.assertAlmostEqual(float(row['tau_sx_Nm']),-.441*float(row['battery_voltage_v'])/12*float(row['pwm_applied_percent'])/100)
            status=json.loads(p.with_suffix('.status.log').read_text())
            self.assertGreater(status['electronics_consumed_wh'],0)
            self.assertGreater(status['motor_sx_consumed_wh'],0)
            self.assertEqual(status['final_ctrl'],[0,0])
            self.assertEqual(json.loads(p.with_suffix('.json').read_text())['schema_version'],9)

    def test_mah_consumed_and_remaining_with_partial_initial_charge(self):
        battery=pack(soc=.5)
        load=battery.load([0,0],[0,0])
        self.assertEqual(load['battery_consumed_mah'],0)
        self.assertEqual(load['battery_remaining_mah'],1500)
        battery.advance(60,load)
        updated=battery.load([0,0],[0,0])
        self.assertAlmostEqual(updated['battery_consumed_mah'],load['battery_current_A']*60/3.6)
        self.assertAlmostEqual(updated['battery_consumed_mah']+updated['battery_remaining_mah'],1500)
        self.assertAlmostEqual(battery.summary()['consumed_mah'],updated['battery_consumed_mah'])

    def test_mode_conflicts(self):
        for flags in (['--battery-discharge','--battery-voltage-v','12'],
                      ['--battery-initial-soc','1.1'],['--electronics-power-w','5']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):parse_args(flags)

    def test_depletion_finalizes_logs_and_disables_motors(self):
        with tempfile.TemporaryDirectory() as folder:
            config=pack().config
            config['cell_capacity_ah']=.001  # synthetic tiny pack to reach cutoff quickly
            path=Path(folder)/'synthetic.json';path.write_text(json.dumps(config))
            logs=Path(folder)/'logs'
            args=parse_args(['--mode','lqr','--headless','--duration','10',
                            '--battery-discharge','--battery-config',str(path)])
            with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(run(args,logs),1)
            status=json.loads(next(logs.glob('*.status.log')).read_text())
            self.assertFalse(status['success'])
            self.assertIn('cutoff',status['stop_reason'])
            self.assertEqual(status['final_ctrl'],[0,0])
