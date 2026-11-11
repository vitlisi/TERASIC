"""Validate the new adapter against the previously tested real-core adapter."""
import ctypes
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from digital_twin import config_values, load_bridge
from design_real_lqr_mimo import load_core


class DigitalTwinTest(unittest.TestCase):
    def test_hinf_path_reference_error_and_feedforward_use_dynamic_controller(self):
        from hinf_controller import DiscreteHinf
        policy=DiscreteHinf(ROOT/'models/hinf/simscape_2pwm_20261026.json')
        with tempfile.TemporaryDirectory() as build:
            core=load_bridge(build,'hinf-path')
            cfg=ROOT/'config/controllers/hinf_path_40cm_28p5s.txt'
            handle=core.twin_create(bytes(cfg),0.,20.)
            self.assertTrue(handle)
            out=(ctypes.c_double*44)()
            try:
                for time in [0.,1.5,5.,9.5,10.5,14.,17.5,18.5,23.,26.5,28.5,32.9]:
                    self.assertEqual(core.twin_step(handle,32768,32768,0.,0.,0.,0.,1.,16384.,.01,time,out),1)
                    error=np.asarray(out[:6])-np.asarray(out[6:12])
                    s,pitch,yaw,v,pd,yd=error
                    expected=100*policy.step([pitch-s/.032,pitch,yaw,pd-v/.032,pd,yd],.01)+np.asarray(out[18:20])
                    np.testing.assert_allclose(out[12:14],expected,atol=1e-9,rtol=1e-10)
                    np.testing.assert_allclose(out[32:44],policy.state,atol=1e-9,rtol=1e-10)
                self.assertEqual(out[21],1.)
                self.assertAlmostEqual(out[8],np.pi)
            finally:
                core.twin_destroy(handle)
            bad=Path(build)/'bad.txt';bad.write_text(cfg.read_text()+'\nk_sx_pitch 100\n')
            self.assertFalse(core.twin_create(bytes(bad),0.,20.))
            reserved=Path(build)/'reserved.txt';reserved.write_text(cfg.read_text()+'\nturn_yaw_reserve_percent 12\n')
            self.assertFalse(core.twin_create(bytes(reserved),0.,10.))
            handle=core.twin_create(bytes(reserved),0.,20.)
            self.assertTrue(handle)
            core.twin_destroy(handle)

    def test_hinf_real_parser_delay_states_and_outputs_match_export(self):
        from hinf_controller import DiscreteHinf
        policy = DiscreteHinf(ROOT/'models/hinf/simscape_2pwm_20261026.json')
        with tempfile.TemporaryDirectory() as build:
            core = load_bridge(build, 'hinf')
            cfg = ROOT/'config/controllers/hinf_stand.txt'
            self.assertFalse(core.twin_create(bytes(cfg), 0., 26.))
            handle = core.twin_create(bytes(cfg), 0., 20.)
            self.assertTrue(handle)
            out = (ctypes.c_double*44)()
            try:
                for i in range(500):
                    self.assertEqual(core.twin_step(handle, 32768, 32768, .1*np.sin(i*.02),
                        .2*np.cos(i*.02), 0., 0., 1., 16384., .01 if i else 0., i*.01, out), 1)
                    s, pitch, yaw, v, pd, yd = out[:6]
                    expected = 100*policy.step([pitch-s/.032,pitch,yaw,pd-v/.032,pd,yd],.01)
                    np.testing.assert_allclose(out[12:14], expected, atol=1e-10, rtol=1e-10)
                    np.testing.assert_allclose(out[26:32], policy.last_feedback, atol=1e-12)
                    np.testing.assert_allclose(out[32:44], policy.state, atol=1e-10, rtol=1e-10)
                    self.assertLessEqual(max(abs(v) for v in out[16:18]), 20.)
                self.assertEqual(core.twin_step(handle, 32768, 32768, 20.,0.,0.,0.,1.,16384.,.01,5.,out),0)
            finally:
                core.twin_destroy(handle)

    def test_path_active_20_percent_matches_previous_cli_capped_profile(self):
        source = ROOT/'config/controllers/lqr_path_40cm_28p5s.txt'
        with tempfile.TemporaryDirectory() as build:
            previous = Path(build)/'previous.txt'
            text = source.read_text()
            for field in ('wheel_pwm_cap', 'max_differential_pwm', 'turn_differential_pwm'):
                text = text.replace(field+' 20', field+' 50')
            previous.write_text(text)
            core = load_bridge(build, 'path')
            handles = [core.twin_create(bytes(p), -3.5, 20.) for p in (source, previous)]
            self.assertTrue(all(handles))
            outputs = [(ctypes.c_double*26)() for _ in handles]
            try:
                for i in range(3300):
                    # Same measured inputs across recovery, both legs, turn and hold.
                    angle = .2*np.sin(i*.03)
                    for handle, output in zip(handles, outputs):
                        self.assertEqual(core.twin_step(handle, 32768, 32768, angle, 0., 0., 0.,
                                                       1., 16384., .01 if i else 0., i*.01, output), 1)
                    np.testing.assert_allclose(outputs[0][:], outputs[1][:], atol=1e-12)
            finally:
                for handle in handles:
                    core.twin_destroy(handle)

    def test_stand_matches_existing_adapter(self):
        path = ROOT/'config/controllers/lqr_mimo.txt'
        values = config_values(path)
        gains = (ctypes.c_double*12)(*[values[f'k_{w}_{s}'] for w in ('sx', 'dx')
            for s in ('position', 'pitch', 'yaw', 'velocity', 'pitch_rate', 'yaw_rate')])
        with tempfile.TemporaryDirectory() as build:
            old, new = load_core(build), load_bridge(build, 'stand')
            old.mimo_create_limited.argtypes = [ctypes.POINTER(ctypes.c_double)]+[ctypes.c_double]*3
            old.mimo_create_limited.restype = ctypes.c_void_p
            a = old.mimo_create_limited(gains, -.1, 20., 3.)
            b = new.twin_create(bytes(path), -.1, 20.)
            x, y = (ctypes.c_double*12)(), (ctypes.c_double*26)()
            try:
                for i in range(100):
                    sx, dx = 32768+i//8, 32768-i//9
                    self.assertEqual(old.mimo_step(a, sx, dx, -.1, 0., x), 1)
                    self.assertEqual(new.twin_step(b, sx, dx, -.1, 0., 0., 0., 1., 16384., .01, i*.01, y), 1)
                    np.testing.assert_allclose(y[:6], x[:6], atol=1e-12)
                    np.testing.assert_allclose(y[12:14], x[6:8], atol=1e-12)
                    np.testing.assert_allclose(y[16:18], x[8:10], atol=1e-12)
            finally:
                old.mimo_destroy(a)
                new.twin_destroy(b)

    def test_path_reads_configuration_and_obeys_faults(self):
        source = ROOT/'config/controllers/lqr_path_40cm_28p5s.txt'
        with tempfile.TemporaryDirectory() as build:
            core = load_bridge(build, 'path')
            copy = Path(build)/'path.txt'
            copy.write_text(source.read_text().replace('path_distance_m 0.40', 'path_distance_m 0.20'))
            output = (ctypes.c_double*26)()
            for file, distance in ((source, .4), (copy, .2)):
                handle = core.twin_create(bytes(file), 0., 50.)
                self.assertTrue(handle)
                try:
                    self.assertEqual(core.twin_step(handle, 32768, 32768, 0., 0., 0., 0., 1., 16384., 0., 0., output), 1)
                    self.assertEqual(core.twin_step(handle, 32768, 32768, 0., 0., 0., 0., 1., 16384., .01, 9.5, output), 1)
                    self.assertAlmostEqual(output[6], distance)
                    self.assertEqual(core.twin_step(handle, 32768, 32768, 20., 0., 0., 0., 1., 16384., .01, 9.51, output), 0)
                    self.assertEqual(output[24], 1.)
                finally:
                    core.twin_destroy(handle)


if __name__ == '__main__':
    unittest.main()
