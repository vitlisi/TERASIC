"""Offline-only measurement/allocator bridge and filtered synthesis plant checks."""
import ctypes
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src_python'), str(ROOT/'scripts/hinf')]
from audit_real_model import filtered_plant
from hinf_controller import DiscreteHinf
from hinf_controller import STATE_TRANSFORM
from tune_stabilization_gains import candidate


class OfflineHinfTests(unittest.TestCase):
    def test_gain_tuning_matches_explicit_input_output_scaling(self):
        import json
        parent=ROOT/'models/hinf/simscape_2pwm_20261026.json'
        e=json.loads(parent.read_text())
        t=np.eye(2)+(.6-1)*np.ones((2,2))/2
        s=np.diag([1,1,1,1,1.25,1])
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'candidate.json'
            tuned=candidate(e,.6,1.25);path.write_text(json.dumps(tuned))
            self.assertNotIn('gamma',tuned)
            actual=DiscreteHinf(path);original=DiscreteHinf(parent)
            rng=np.random.default_rng(19)
            for error in rng.normal(0,.01,(500,6)):
                adjusted=np.linalg.solve(STATE_TRANSFORM,s@STATE_TRANSFORM@error)
                expected=t@original.step(adjusted,.01)
                np.testing.assert_allclose(actual.step(error,.01),expected,atol=1e-10,rtol=1e-10)
            identity=candidate(e,1,1)
            for key in ('Ak','Bk','Ck','Dk'):np.testing.assert_array_equal(identity[key],e[key])

    def test_filtered_plant_matches_sample_difference(self):
        rng = np.random.default_rng(17)
        a = np.eye(6)+rng.normal(0,.01,(6,6)); b = rng.normal(0,.01,(6,2))
        f,g,c = filtered_plant(a,b)
        z = rng.normal(size=8); u = rng.normal(size=2)
        new = f@z+g@u; raw = a@z[:6]+b@u
        alpha = np.exp(-.01/.05)
        wheel = alpha*z[6]+(1-alpha)*(raw[0]-z[0])/.01
        yaw = alpha*z[7]+(1-alpha)*(raw[2]-z[2])/.01
        np.testing.assert_allclose(new,np.r_[raw,wheel,yaw],atol=1e-13)
        np.testing.assert_allclose(c@new,[raw[1]-raw[0],raw[1],raw[2],raw[4]-wheel,raw[4],yaw],atol=1e-13)

    def test_split_candidate_path_matches_existing_cpp_core(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'bridge.so'
            subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',
                str(ROOT/'scripts/hinf/hinf_real_sim_bridge.cpp'),'-o',str(path)],check=True)
            lib = ctypes.CDLL(str(path)); ptr = ctypes.POINTER(ctypes.c_double)
            lib.hinf_create.argtypes = [ctypes.c_double]*3; lib.hinf_create.restype = ctypes.c_void_p
            lib.hinf_destroy.argtypes = [ctypes.c_void_p]
            for name in ('hinf_measure','hinf_step'):
                fn = getattr(lib,name)
                fn.argtypes = [ctypes.c_void_p,ctypes.c_uint,ctypes.c_uint,ctypes.c_double,ctypes.c_double,ptr]
                fn.restype = ctypes.c_int
            lib.hinf_allocate_only.argtypes = [ctypes.c_double]*4+[ptr]
            lib.hinf_allocate_only.restype = ctypes.c_int
            h1,h2 = [lib.hinf_create(-3.,20.,3.) for _ in range(2)]
            policy = DiscreteHinf(ROOT/'models/hinf/simscape_2pwm_20261026.json')
            rng = np.random.default_rng(22)
            try:
                for i in range(300):
                    sx,dx = 32768+int(10*np.sin(i*.1)),32768+int(9*np.sin(i*.1))
                    accel,rate = rng.normal(0,.5,2)
                    full = (ctypes.c_double*30)(); measured = (ctypes.c_double*6)()
                    self.assertEqual(lib.hinf_step(h1,sx,dx,accel,rate,full),1)
                    self.assertEqual(lib.hinf_measure(h2,sx,dx,accel,rate,measured),1)
                    np.testing.assert_array_equal(full[:6],measured[:])
                    s,pitch,yaw,v,pd,yd = measured[:]
                    command = 100*policy.step([pitch-s/.032,pitch,yaw,pd-v/.032,pd,yd],.01)
                    np.testing.assert_allclose(command,full[6:8],atol=1e-9,rtol=1e-10)
                    allocated = (ctypes.c_double*4)()
                    self.assertEqual(lib.hinf_allocate_only(*command,20.,3.,allocated),1)
                    np.testing.assert_allclose(allocated[:],full[8:12],atol=1e-9,rtol=1e-10)
                self.assertEqual(lib.hinf_allocate_only(80,-80,50,3,allocated),1)
                np.testing.assert_array_equal(allocated[:2],[3,-3])
                self.assertEqual(lib.hinf_allocate_only(80,80,50,3,allocated),1)
                np.testing.assert_array_equal(allocated[:2],[50,50])
                self.assertEqual(lib.hinf_allocate_only(0,0,50.01,3,allocated),0)
            finally:
                lib.hinf_destroy(h1);lib.hinf_destroy(h2)


if __name__ == '__main__': unittest.main()
