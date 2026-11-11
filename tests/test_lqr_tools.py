import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('lqr_audit', ROOT/'scripts/audit_lqr_model.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class LqrToolsTests(unittest.TestCase):
    def test_corrected_single_input_is_not_doubled_or_mixed_with_old_slx(self):
        data=json.loads((ROOT/'models/lqr/matrices_common_pwm_corrected.json').read_text())
        result=audit.audit(None,data)
        self.assertAlmostEqual(result['B_common_modal'][1][0],-890.5715)
        self.assertAlmostEqual(result['B_common_modal'][3][0],978.3312)
        self.assertEqual(result['B_common_modal'][4:],[[0.0],[0.0]])
        self.assertIsNone(result['slx'])
        self.assertFalse(result['state_mapping_verified'])

    @unittest.skipUnless((ROOT/'maintenance/matlab/Terasic_Simscape_linmod.slx').is_file(),
                         'Audit Simscape opzionale: materiale locale in maintenance/matlab')
    def test_common_input_cannot_regulate_differential_mode(self):
        result = audit.audit(ROOT/'maintenance/matlab/Terasic_Simscape_linmod.slx')
        self.assertEqual(result['B_common_modal'][4:], [[0.0], [0.0]])
        self.assertTrue(any(r['pole_real']==0 and r['rank_common_input']<6
                            for r in result['pbh']))
        self.assertFalse(result['hardware_gains_generated'])


    def test_launcher_retains_failed_run_status_and_csv(self):
        with tempfile.TemporaryDirectory() as d:
            project=Path(d)
            scripts=project/'scripts'; scripts.mkdir()
            launcher=scripts/'run_lqr_mimo_logged.sh'
            shutil.copyfile(ROOT/'scripts/run_lqr_mimo_logged.sh',launcher)
            config=project/'config.txt'; config.write_text('# test fixture\n')
            (project/'build/robot').mkdir(parents=True)
            binary=project/'build/robot/balance_lqr_2'
            binary.write_text('#!/bin/sh\nprintf "sample,enc_sx_raw\\n0,32768\\n"\n'
                              'echo "fake observe failure" >&2\nexit 7\n')
            binary.chmod(0o755)
            result=subprocess.run(['sh',str(launcher),'shadow',str(config),'1','15'],capture_output=True,text=True)
            self.assertEqual(result.returncode,7)
            folder=project/'logs/terasic_reale/runs/lqr_fermo/observe'
            status=list(folder.glob('*.status.log'))
            self.assertEqual(len(status),1)
            self.assertIn('exit_code=7',status[0].read_text())
            self.assertIn('fake observe failure',status[0].read_text())
            self.assertEqual(len(list(folder.glob('*.csv'))),1)
            self.assertEqual(next(folder.glob('*.config.txt')).read_text(),config.read_text())
            self.assertFalse((project/'logs/terasic_reale/tuning').exists())

    def test_launcher_interrupt_stops_child_and_records_status(self):
        with tempfile.TemporaryDirectory() as d:
            project=Path(d)
            scripts=project/'scripts'; scripts.mkdir()
            launcher=scripts/'run_lqr_mimo_logged.sh'
            shutil.copyfile(ROOT/'scripts/run_lqr_mimo_logged.sh',launcher)
            config=project/'config.txt'; config.write_text('# test fixture\n')
            (project/'build/robot').mkdir(parents=True)
            binary=project/'build/robot/balance_lqr_2'
            binary.write_text('#!/usr/bin/env python3\nimport signal,time,sys\n'
                              'def stop(*args):\n print("fake child stopped",file=sys.stderr,flush=True)\n sys.exit(0)\n'
                              'signal.signal(signal.SIGTERM,stop)\n'
                              'print("fake child ready",file=sys.stderr,flush=True)\n'
                              'while True: time.sleep(.01)\n')
            binary.chmod(0o755)
            proc=subprocess.Popen(['sh',str(launcher),'shadow',str(config),'20','15'],stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE,text=True)
            try:
                deadline=time.monotonic()+5
                status=[]
                while time.monotonic()<deadline:
                    status=list((project/'logs/terasic_reale/runs/lqr_fermo/observe').glob('*.status.log'))
                    if status and 'fake child ready' in status[0].read_text(): break
                    time.sleep(.01)
                self.assertTrue(status and 'fake child ready' in status[0].read_text())
                proc.terminate()
                proc.communicate(timeout=5)
                self.assertEqual(proc.returncode,130)
                self.assertIn('fake child stopped',status[0].read_text())
                self.assertIn('exit_code=130',status[0].read_text())
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    proc.communicate(timeout=5)

    def test_launcher_records_battery_outside_controller_execution(self):
        with tempfile.TemporaryDirectory() as d:
            project=Path(d)
            scripts=project/'scripts'; scripts.mkdir()
            launcher=scripts/'run_lqr_mimo_logged.sh'
            shutil.copyfile(ROOT/'scripts/run_lqr_mimo_logged.sh',launcher)
            for name,event in [('battery_status','battery'),('balance_lqr_2','control')]:
                (project/'build/robot').mkdir(parents=True,exist_ok=True)
                program=project/'build/robot'/name
                program.write_text('#!/usr/bin/env python3\nfrom pathlib import Path\n'
                    'p=Path(__file__).parent\n'
                    f'with (p/"events").open("a") as stream: stream.write("{event}\\n")\n'
                    f'print("{event}: test fixture")\n')
                program.chmod(0o755)
            config=project/'config.txt'; config.write_text('# fake controller config\n')
            result=subprocess.run(['sh',str(launcher),'run',str(config),'1','15'],
                                  capture_output=True,text=True)
            self.assertEqual(result.returncode,0)
            self.assertEqual((project/'build/robot/events').read_text().splitlines(),['battery','control','battery'])
            status=next((project/'logs/terasic_reale/runs/lqr_fermo/tuning').glob('*.status.log')).read_text()
            self.assertIn('Batteria PRIMA',status)
            self.assertIn('Batteria DOPO',status)
            self.assertEqual(status.count('battery: test fixture'),2)
            self.assertIn('exit_code=0',status)


if __name__=='__main__':
    unittest.main()
