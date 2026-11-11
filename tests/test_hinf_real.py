"""Offline parity of the actual ARM-compatible Hinf core against Python export."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from hinf_controller import DiscreteHinf

class RealHinfTests(unittest.TestCase):
    def test_gain_trial_preinclude_matches_candidate(self):
        export=ROOT/'models/hinf/stabilization_gain_trial_20261028.json'
        header=ROOT/'src_cpp/terasic_reale/controllers/balance_hinf_gain_trial_matrices.h'
        self.assertIn(hashlib.sha256(export.read_bytes()).hexdigest(),header.read_text())
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'bridge.cpp').write_text('''#include <cstdio>
#include "balance_hinf_core.h"
int main(){RealHinf h; LqrMimoState s; double sx,dx;
while(std::scanf("%lf %lf %lf %lf %lf %lf",&s.x[0],&s.x[1],&s.x[2],&s.x[3],&s.x[4],&s.x[5])==6){
if(!h.step(s,.032,&sx,&dx))return 1;
std::printf("%.17g %.17g",sx,dx);
for(int j=0;j<12;++j)std::printf(" %.17g",h.xc[j]);
std::printf("\\n");}return 0;}''')
            subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror','-include',str(header),
                '-I'+str(ROOT/'src_cpp/terasic_reale/controllers'),str(p/'bridge.cpp'),'-o',str(p/'bridge')],check=True)
            real=np.random.default_rng(45).normal(0,.01,(2000,6))
            lines='\n'.join(' '.join(format(v,'.17g') for v in row) for row in real)
            run=subprocess.run([str(p/'bridge')],input=lines,text=True,capture_output=True,check=True)
            actual=np.loadtxt(run.stdout.splitlines());policy=DiscreteHinf(export);expected=[]
            for s in real:
                u=100*policy.step([s[1]-s[0]/.032,s[1],s[2],s[4]-s[3]/.032,s[4],s[5]],.01)
                expected.append(np.r_[u,policy.state])
            np.testing.assert_allclose(actual,expected,atol=1e-10,rtol=1e-10)

    def test_long_sequence_matches_python_states_delay_and_pwm(self):
        export=ROOT/'models/hinf/simscape_2pwm_20261026.json'
        digest=hashlib.sha256(export.read_bytes()).hexdigest()
        self.assertIn(digest,(ROOT/'src_cpp/terasic_reale/controllers/balance_hinf_matrices.h').read_text())
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'bridge.cpp').write_text('''#include <cstdio>
#include "balance_hinf_core.h"
int main(){ RealHinf h; LqrMimoState s; double sx,dx;
while(std::scanf("%lf %lf %lf %lf %lf %lf",&s.x[0],&s.x[1],&s.x[2],&s.x[3],&s.x[4],&s.x[5])==6){
if(!h.step(s,.032,&sx,&dx))return 1;
std::printf("%.17g %.17g",sx,dx);
for(int j=0;j<12;++j)std::printf(" %.17g",h.xc[j]);
for(int j=0;j<6;++j)std::printf(" %.17g",h.feedback[j]);
std::printf("\\n");}return 0;}''')
            subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src_cpp/terasic_reale/controllers'),str(p/'bridge.cpp'),'-o',str(p/'bridge')],check=True)
            rng=np.random.default_rng(42)
            real=rng.normal(0,.01,(2000,6))
            lines='\n'.join(' '.join(format(v,'.17g') for v in row) for row in real)
            out=subprocess.run([str(p/'bridge')],input=lines,text=True,capture_output=True,check=True)
            actual=np.loadtxt(out.stdout.splitlines())
            h=DiscreteHinf(export); expected=[]
            for s in real:
                # Native MuJoCo theta = pitch - s/r, same relation for rates.
                native=np.array([s[1]-s[0]/.032,s[1],s[2],s[4]-s[3]/.032,s[4],s[5]])
                u=100*h.step(native,.01)
                expected.append(np.r_[u,h.state,h.last_feedback])
            np.testing.assert_allclose(actual,expected,rtol=1e-10,atol=1e-10)
            np.testing.assert_array_equal(actual[0,:2],[0,0])

    def test_config_and_run_gates_before_hardware(self):
        with tempfile.TemporaryDirectory() as d:
            binary=Path(d)/'balance_hinf'
            subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror',str(ROOT/'src_cpp/terasic_reale/controllers/balance_hinf.cpp'),'-o',str(binary),'-lrt'],check=True)
            config=ROOT/'config/controllers/hinf_shadow.txt'
            def call(*args):return subprocess.run([str(binary),*map(str,args)],capture_output=True,text=True)
            self.assertEqual(call('check',config).returncode,0)
            r=call('run',config,10,15)
            self.assertEqual(r.returncode,2)
            self.assertIn('prima dell',r.stderr)
            self.assertEqual(call('shadow',config,10,50).returncode,2)
            self.assertEqual(call('shadow',config,10,25.01).returncode,2)
            r=call('run',config,2,25)
            self.assertEqual(r.returncode,2)
            self.assertIn('prima dell',r.stderr)
            for old,new in [('wheel_radius_m 0.032','wheel_radius_m 0.04'),('gyro_sign 1','gyro_sign -1'),('shadow_only 1','shadow_only nan')]:
                bad=Path(d)/'bad.txt';bad.write_text(config.read_text().replace(old,new))
                self.assertNotEqual(call('check',bad).returncode,0)
