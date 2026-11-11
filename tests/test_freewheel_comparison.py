"""Offline protocol, fixture and replay checks. Synthetic data are not experiments."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'diagnostics'))
import compare_freewheel as fw

STATUS = ('protocol_id=freewheel_v1 wheel_convention=physical_sx_dx_v2\n'
          'final_control_sx=6 final_control_dx=6\nresult=COMPLETED\n')


class FreewheelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.voltage = 2821*18/4.7/1000
        cls.source_hash = fw.sha(fw.XML)
        cls.sim, cls.details = fw.replay(fw.preview_rows(cls.voltage), 1325)
        cls.real = []
        for sample in cls.sim:
            row = dict(t_s=sample['t_s'], phase=sample['phase'], battery_v=cls.voltage,
                       battery_raw=2821, battery_age_s=0.)
            for w in fw.WHEELS:
                pwm = sample[f'pwm_{w}_applied_percent']
                row.update({f'pwm_{w}_requested_percent': pwm,
                            f'pwm_{w}_applied_percent': pwm, f'period_{w}': 50000,
                            f'high_{w}': int(pwm*500), f'control_{w}': 7 if pwm else 6,
                            f'enc_{w}_raw': 32768+sample[f'enc_{w}_counts']})
            cls.real.append(row)

    def test_cpp_plan_matches_python_and_has_three_one_second_pulses(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp)/'motor_freewheel'
            subprocess.run(['g++', '-std=c++98', '-O2', '-Wall', '-Wextra', '-Werror',
                            str(ROOT/'diagnostics/motor_freewheel.cpp'), '-o', str(binary), '-lrt'], check=True)
            output = subprocess.check_output([str(binary), 'plan'], text=True)
            rows = output.splitlines()[1:]
            self.assertEqual(len(rows), 1501)
            for row in rows:
                t, phase, left, right = map(float, row.split(','))
                self.assertEqual(fw.protocol(t), (int(phase), left))
                self.assertEqual(left, right)
            for phase in (1, 2, 3):
                self.assertEqual(sum(fw.protocol(i/100)[0] == phase for i in range(1501)), 100)
            small = subprocess.check_output([str(binary), 'plan', '--small-commands'], text=True)
            self.assertEqual(len(small.splitlines())-1, 2701)
            for row in small.splitlines()[1:]:
                t, phase, left, right = map(float, row.split(','))
                self.assertEqual(fw.protocol(t, 'freewheel_small_v1'), (int(phase), left))
                self.assertEqual(left, right)

    def test_small_protocol_signed_register_validation_and_replay(self):
        rows = fw.preview_rows(self.voltage, 'freewheel_small_v1')
        for row in rows:
            row.update(battery_raw=2821, battery_age_s=0.)
            for w in fw.WHEELS:
                pwm = row[f'pwm_{w}_applied_percent']
                row.update({f'pwm_{w}_requested_percent': pwm,
                            f'period_{w}': 50000, f'high_{w}': int(abs(pwm)*500),
                            f'control_{w}': (7 if pwm > 0 else 5) if pwm else 6,
                            f'enc_{w}_raw': 32768})
        status = STATUS.replace('freewheel_v1', 'freewheel_small_v1')
        fw.validate_real(rows, status, 'freewheel_small_v1')
        simulation, details = fw.replay(rows, 1325)
        self.assertLess(simulation[300]['torque_sx_drive_Nm'], 0)
        self.assertGreater(simulation[900]['torque_sx_drive_Nm'], 0)
        _, metrics = fw.analyze(rows, simulation, 1325, details, 'freewheel_small_v1')
        self.assertEqual(len(metrics), 60)
        rows[300]['control_sx'] = 7
        with self.assertRaises(ValueError):
            fw.validate_real(rows, status, 'freewheel_small_v1')

    def test_fixture_retains_original_joint_inertia_and_no_ground_contact(self):
        model, data, coefficient, details = fw.fixed_model()
        self.assertEqual(model.nv, 2)
        self.assertEqual(data.ncon, 0)
        self.assertAlmostEqual(coefficient, .441/12)
        np.testing.assert_allclose(details['armature_kg_m2'], [.005, .005])
        np.testing.assert_allclose(details['J_equivalent_kg_m2'], [.005+.5*.049*.032**2]*2)
        self.assertEqual(fw.sha(fw.XML), self.source_hash)
        for w in fw.WHEELS:
            self.assertAlmostEqual(self.sim[100][f'torque_{w}_drive_Nm'], coefficient*self.voltage*.1)
            self.assertGreater(self.sim[200][f'position_{w}_true_rad'], 0)
            self.assertEqual(self.sim[-1][f'torque_{w}_drive_Nm'], 0)

    def test_completed_log_replays_same_counts_and_keeps_torque_conditional(self):
        fw.validate_real(self.real, STATUS)
        simulation, details = fw.replay(self.real, 1325)
        aligned, metrics = fw.analyze(self.real, simulation, 1325, details)
        self.assertEqual(len(metrics), 6)
        self.assertTrue(all(m['RMSE_counts'] == 0 for m in metrics))
        self.assertTrue(all(m['RMSE_rad_s'] == 0 for m in metrics))
        self.assertIn('real_sx_torque_inverse_assumed_Nm', aligned[0])
        self.assertIn('real_sx_torque_PWM_proxy_Nm', aligned[0])

    def test_rejects_incomplete_wrong_registers_or_stale_battery(self):
        for field, value in [('high_sx', 1), ('enc_dx_raw', 65535),
                             ('battery_age_s', .3), ('battery_raw', 100), ('t_s', 0)]:
            rows = copy.deepcopy(self.real)
            rows[120][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                fw.validate_real(rows, STATUS)
        with self.assertRaises(ValueError):
            fw.validate_real(self.real[:-100], STATUS)
        with self.assertRaises(ValueError):
            fw.validate_real(self.real, STATUS.replace('COMPLETED', 'ABORTED'))

    def test_derivative_with_irregular_times(self):
        t = np.arange(80)*.01 + .0001*np.sin(np.arange(80))
        velocity, acceleration = fw.derivatives(t, t*t)
        np.testing.assert_allclose(velocity, 2*t, atol=1e-10)
        np.testing.assert_allclose(acceleration, 2., atol=1e-9)

    def test_comparison_cli_outputs_report_and_never_claims_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            real = folder/'synthetic.csv'
            fw.write_csv(real, self.real)
            real.with_suffix('.status.log').write_text(STATUS)
            real.with_suffix('.config.txt').write_text((ROOT/'config/controllers/lqr_mimo.txt').read_text())
            subprocess.run([sys.executable, '-B', str(ROOT/'diagnostics/compare_freewheel.py'),
                            'compare', '--real', str(real), '--output-dir', str(folder/'comparison')],
                           check=True, stdout=subprocess.DEVNULL)
            summary = json.loads((folder/'comparison/summary.json').read_text())
            self.assertFalse(summary['digital_twin_validated'])
            self.assertFalse(summary['model']['backlash_modelled'])
            for name in ('REPORT.md', 'confronto.svg', 'batteria.svg', 'metrics.csv', 'reale.csv'):
                self.assertTrue((folder/'comparison'/name).is_file())


if __name__ == '__main__':
    unittest.main()
