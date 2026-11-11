"""Test del solo launcher con programma finto; nessun accesso al robot."""
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time
import unittest
ROOT=Path(__file__).resolve().parents[1]


class OfflineLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.folder=Path(self.temp.name)
        script=(ROOT/'scripts/run_pd_offline.sh').read_text()
        self.script=self.folder/'launcher.sh'
        self.script.write_text(script.replace('project=/home/root/terasic_robot',f'project={self.folder}'))
        (self.folder/'build/robot').mkdir(parents=True)
        fake=self.folder/'build/robot/pitch_pd'
        fake.write_text('''#!/usr/bin/env python3
import os,signal,sys,time
from pathlib import Path
Path('called').write_text(' '.join(sys.argv[1:]))
Path('child_pid').write_text(str(os.getpid()))
Path('child_sid').write_text(str(os.getsid(0)))
def stop(signum,frame):
    Path('child_stopped').write_text(str(signum))
    sys.exit(130)
signal.signal(signal.SIGTERM,stop)
signal.signal(signal.SIGHUP,stop)
Path('child_ready').write_text('1')
print('sample,pwm_percent',flush=True)
print('0,0',flush=True)
time.sleep(.6)
print('PD terminato; RUN SX=0 DX=0.',file=sys.stderr)
''')
        fake.chmod(0o755)
        bin=self.folder/'bin';bin.mkdir()
        (bin/'sleep').write_text('#!/bin/sh\nexec /bin/sleep .05\n');(bin/'sleep').chmod(0o755)
        (bin/'sync').write_text('#!/bin/sh\nexit 0\n');(bin/'sync').chmod(0o755)
        self.env=dict(os.environ,PATH=str(bin)+':'+os.environ['PATH'])
        self.procs=[]

    def tearDown(self):
        for p in self.procs:
            if p.poll() is None:
                p.terminate()
                p.wait(timeout=3)
        self.temp.cleanup()

    def launch(self,mode):
        self.output=(self.folder/'launcher.log').open('w')
        p=subprocess.Popen(['nohup','setsid','sh',str(self.script),mode,'5','2'],stdin=subprocess.DEVNULL,stdout=self.output,stderr=subprocess.STDOUT,env=self.env)
        self.procs.append(p)
        return p

    def wait_file(self,path):
        for _ in range(200):
            if path.exists():return
            time.sleep(.01)
        self.fail(f'Non creato {path}')

    def test_observe_session_detached_and_logs(self):
        p=self.launch('observe')
        self.wait_file(self.folder/'child_sid')
        self.assertNotEqual(int((self.folder/'child_sid').read_text()),os.getsid(0))
        self.assertEqual(p.wait(timeout=3),0)
        self.output.close()
        self.assertEqual((self.folder/'called').read_text(),'observe 2')
        status=next(self.folder.glob('logs/**/*.status.log')).read_text()
        self.assertIn('exit_code=0',status)
        self.assertIn('RUN SX=0 DX=0',status)
        self.assertFalse((self.folder/'.pd_offline_lock').exists())

    def test_terminate_launcher_stops_child(self):
        p=self.launch('run')
        self.wait_file(self.folder/'child_ready')
        pid=int((self.folder/'.pd_offline_lock/pid').read_text())
        os.kill(pid,signal.SIGTERM)
        self.assertEqual(p.wait(timeout=3),130)
        self.output.close()
        self.assertEqual((self.folder/'called').read_text(),'run 1 1 5.0 0.8 15 2')
        self.assertEqual((self.folder/'child_stopped').read_text(),str(int(signal.SIGTERM)))
        self.assertFalse((self.folder/'.pd_offline_lock').exists())

    def test_reject_attached_terminal(self):
        master,slave=pty.openpty()
        try:
            p=subprocess.Popen(['sh',str(self.script),'run','5','2'],stdin=slave,stdout=slave,stderr=slave,env=self.env)
            self.procs.append(p)
            self.assertEqual(p.wait(timeout=3),2)
            self.assertFalse((self.folder/'called').exists())
        finally:
            os.close(master);os.close(slave)

if __name__=='__main__':unittest.main()
