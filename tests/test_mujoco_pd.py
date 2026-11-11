"""Controller indipendente: convenzioni fisiche, stop e log riproducibili."""
import contextlib
import csv
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from balance_controller import BalanceController, ControllerConfig
from mujoco_robot import MujocoRobot
from mujoco_sim import parse_args, run


class SimulationTests(unittest.TestCase):
    def test_arguments_finite_full_pwm_and_unbounded_duration(self):
        a=parse_args(['--duration','1000000','--max-pwm','100','--kp=-20','--kd','200'])
        self.assertEqual(a.duration,1000000)
        self.assertEqual(a.max_pwm,100)
        for flags in (['--max-pwm','101'],['--duration','0'],['--kp','nan'],['--kv','inf']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):parse_args(flags)

    def test_encoder_velocity_includes_body_rotation(self):
        c=BalanceController(ControllerConfig(),0)
        rate=30;relative=math.radians(rate)
        info=c.update(0,rate,relative,relative,.01,ideal_pitch_deg=0)
        self.assertAlmostEqual(info['velocity_encoder_m_s'],0)

    def test_encoder_feedback_opposes_forward_drift(self):
        c=BalanceController(ControllerConfig(kx=0),-1)
        info=c.update(-1,0,-5,-5,.01,ideal_pitch_deg=-1)
        self.assertGreater(info['velocity_encoder_m_s'],0)
        self.assertLess(info['pitch_target_deg'],-1)

    def test_pwm_quantization_bounds_and_zero_limit(self):
        c=BalanceController(ControllerConfig(kp=100,kd=0,kv=0,kx=0,max_pwm=75),0)
        for angle,expected in ((100,75),(-100,-75),(.001,0)):
            self.assertEqual(c.update(angle,0,0,0,.01,ideal_pitch_deg=angle)['pwm_applied_percent'],expected)
        c=BalanceController(ControllerConfig(max_pwm=0),0)
        self.assertEqual(c.update(10,100,0,0,.01,ideal_pitch_deg=10)['pwm_applied_percent'],0)
        with self.assertRaises(ValueError):c.update(float('nan'),0,0,0,.01)

    def test_mass_and_cad_principal_inertia(self):
        r=MujocoRobot()
        # DATI_SKELETON.pdf, FRAME, 2026-10-21: battery box included, wheels excluded.
        self.assertAlmostEqual(float(r.m.body('struttura_centrale').mass[0]),.75166)
        self.assertAlmostEqual(float(r.m.body_mass.sum()),.84966)
        np.testing.assert_allclose(r.m.body('struttura_centrale').ipos,
                                   [.00030,.00053,.03503],rtol=0,atol=1e-12)
        for name in ('ruota_sx','ruota_dx'):self.assertAlmostEqual(float(r.m.body(name).mass[0]),.049)
        np.testing.assert_allclose(np.sort(r.m.body('struttura_centrale').inertia),np.array([894591.49,969137.27,1450651.08])*1e-9,rtol=1e-7,atol=5e-11)

    def test_support_release_preserves_state_and_pwm_sign(self):
        states=[]
        for pwm in (-15,15):
            r=MujocoRobot();r.calibrate()
            q=r.d.qpos.copy();v=r.d.qvel.copy();r.release()
            np.testing.assert_array_equal(q,r.d.qpos);np.testing.assert_array_equal(v,r.d.qvel)
            r.set_motors(pwm,pwm);mujoco.mj_forward(r.m,r.d)
            k=int(r.m.joint('perno_sx').dofadr[0])
            self.assertAlmostEqual(float(r.d.qfrc_actuator[k]),-.441*pwm/100)
            for _ in range(20):mujoco.mj_step(r.m,r.d)
            mujoco.mj_forward(r.m,r.d)
            states.append((r.true_pitch_deg(),float(r.d.qpos[1])))
        self.assertGreater(states[1][1],states[0][1])
        self.assertLess(states[1][0],states[0][0])

    def test_end_to_end_metadata_logs_and_motor_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            a=parse_args(['--headless','--sensor','ideal','--duration','.02'])
            with contextlib.redirect_stdout(io.StringIO()):code=run(a,directory)
            self.assertEqual(code,0)
            p=next(Path(directory).glob('*.csv'))
            status=json.loads(p.with_suffix('.status.log').read_text())
            meta=json.loads(p.with_suffix('.json').read_text())
            self.assertEqual(status['final_ctrl'],[0,0])
            self.assertEqual(meta['schema_version'],3)
            self.assertFalse(any('src_cpp' in path for path in meta['sha256']))
            with p.open() as f:rows=list(csv.DictReader(f))
            held=[r for r in rows if r['phase']!='free']
            self.assertTrue(all(float(r['pwm_applied_percent'])==0 for r in held))
            self.assertEqual(len([r for r in rows if r['phase']=='free']),2)

if __name__=='__main__':unittest.main()
