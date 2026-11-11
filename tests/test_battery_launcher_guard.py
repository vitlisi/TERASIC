"""Battery failure must stop before invoking an armed controller (fake tools)."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class BatteryGuardTests(unittest.TestCase):
    def test_custom_output_keeps_battery_failure_evidence_and_never_arms(self):
        cases = [('run_lqr_mimo_logged.sh', 'balance_lqr_2'),
                 ('run_lqr_path_simple_logged.sh', 'path_simple'),
                 ('run_hinf_logged.sh', 'balance_hinf'),
                 ('run_hinf_path_logged.sh', 'hinf_path'),
                 ('run_lqr_ga_logged.sh', 'balance_lqr_ga'),
                 ('run_lqr_ga_path_logged.sh', 'path_lqr_ga')]
        for script, binary in cases:
            with self.subTest(script=script), tempfile.TemporaryDirectory() as d:
                p = Path(d)
                (p/'scripts').mkdir()
                (p/'build/robot').mkdir(parents=True)
                shutil.copyfile(ROOT/'scripts'/script, p/'scripts'/script)
                cfg = p/'config.txt'
                cfg.write_text('# dummy config; must never reach controller\n')
                executable = p/'build/robot'/binary
                executable.write_text('#!/bin/sh\ntouch "'+str(p/'armed')+'"\n')
                executable.chmod(0o755)
                output = p/'custom run output'
                result = subprocess.run(['sh', str(p/'scripts'/script), 'run', str(cfg), '2', '15'],
                    env=dict(os.environ, ROBOT_LOG_DIR=str(output)), capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((p/'armed').exists())
                statuses = list(output.glob('*.status.log'))
                self.assertEqual(len(statuses), 1)
                self.assertIn('START_BLOCKED=battery_check_failed', statuses[0].read_text())

    def test_low_or_missing_battery_blocks_all_logged_controllers(self):
        cases=[('run_lqr_mimo_logged.sh','balance_lqr_2','lqr_fermo'),
               ('run_lqr_path_simple_logged.sh','path_simple','lqr_40cm_180_40cm'),
               ('run_hinf_logged.sh','balance_hinf','hinf_fermo')]
        for script,binary,tag in cases:
            for battery in ['low','missing']:
                with self.subTest(script=script,battery=battery),tempfile.TemporaryDirectory() as d:
                    p=Path(d);(p/'scripts').mkdir();(p/'build/robot').mkdir(parents=True)
                    shutil.copyfile(ROOT/'scripts'/script,p/'scripts'/script)
                    config=p/'config.txt';config.write_text('# fake config\n')
                    executable=p/'build/robot'/binary
                    executable.write_text('#!/bin/sh\ntouch "'+str(p/'controller_was_started')+'"\n')
                    executable.chmod(0o755)
                    if battery=='low':
                        executable=p/'build/robot/battery_status'
                        executable.write_text('#!/bin/sh\necho "battery 10.1 V"\nexit 2\n');executable.chmod(0o755)
                    r=subprocess.run(['sh',str(p/'scripts'/script),'run',str(config),'2','15'],capture_output=True,text=True)
                    self.assertNotEqual(r.returncode,0,r.stdout+r.stderr)
                    self.assertFalse((p/'controller_was_started').exists())
                    logs=list((p/'logs'/'real'/('hinf' if script == 'run_hinf_logged.sh' else 'lqr')).glob('*.status.log'))
                    self.assertEqual(len(logs),1)
                    self.assertIn('START_BLOCKED=battery_check_failed',logs[0].read_text())
