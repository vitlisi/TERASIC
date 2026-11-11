"""Exercise argument capture and battery guard with fake hardware programs."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class PDLoggedTests(unittest.TestCase):
    def test_observe_and_armed_battery_failure(self):
        for controller, binary in [('pd', 'pitch_pd_fixed'), ('pd-outer', 'pitch_pd_outer')]:
            with self.subTest(controller=controller), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                (root/'scripts').mkdir()
                (root/'build/robot').mkdir(parents=True)
                shutil.copy2(ROOT/'scripts/run_pd_logged.sh', root/'scripts/run_pd_logged.sh')
                fake = root/'build/robot'/binary
                fake.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
                fake.chmod(0o755)
                command = ['sh', str(root/'scripts/run_pd_logged.sh'), controller]
                result = subprocess.run(command+['observe', '3', '.320'], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                folder = root/'logs'/'real'/controller
                self.assertEqual(len(list(folder.glob('*.csv'))), 1)
                self.assertFalse(any(p.is_dir() for p in folder.iterdir()))
                args = ['run', '1', '1', '5', '.8', '15', '10', '.320']
                if controller == 'pd-outer':
                    args += ['2', '8']
                result = subprocess.run(command+args, capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                statuses = list(folder.glob('*_run_*.status.log'))
                self.assertEqual(len(statuses), 1)
                self.assertIn('START_BLOCKED=battery_check_failed', statuses[0].read_text())
                self.assertEqual(len(list(folder.glob('*.args.txt'))), 2)
