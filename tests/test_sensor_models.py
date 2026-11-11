"""Sensor frame, rotor reduction, quantization, ODR, delays and integration."""
import contextlib
import copy
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
from sensor_models import Mpu6500,MotorEncoders
from mujoco_robot import MujocoRobot
from mujoco_sim import parse_args,run
PROFILE=json.loads((ROOT/'config/mujoco_sensors_mpu6500.json').read_text())


def noiseless():
    config=copy.deepcopy(PROFILE['imu'])
    config['accel_noise_density_g_sqrt_hz']=0
    config['gyro_noise_density_deg_s_sqrt_hz']=0
    return config


class SensorTests(unittest.TestCase):
    def test_rotor_count_scale_and_independent_wheels(self):
        encoder=MotorEncoders(PROFILE['encoder'])
        encoder.update_positions([0,0]);encoder.read(0)
        # One motor revolution = 1/30 wheel turn = 44 x4-decoded counts.
        encoder.update_positions([2*np.pi/30,0])
        self.assertEqual(encoder.counts.tolist(),[32768+44,32768])
        angle,velocity,right,_=encoder.read(.01)
        self.assertAlmostEqual(angle,2*np.pi/30)
        self.assertEqual(right,0)
        self.assertGreater(velocity,0)
        encoder.update_positions([2*np.pi,2*np.pi])
        self.assertEqual(encoder.counts.tolist(),[32768+1320,32768-1320])
        left,_,right,_=encoder.read(.02)
        self.assertAlmostEqual(left,2*np.pi);self.assertAlmostEqual(right,2*np.pi)

    def test_counter_saturates_and_reverses_without_rollover(self):
        encoder=MotorEncoders(PROFILE['encoder'])
        encoder.update_positions([200,0])
        self.assertEqual(encoder.counts[0],65535)
        with self.assertRaises(ValueError):encoder.read(0)
        encoder.update_positions([199,0])
        self.assertLess(encoder.counts[0],65535)
        self.assertGreater(encoder.counts[0],65235)

    def test_adc_scale_and_output_hold(self):
        imu=Mpu6500(noiseless())
        imu.update(0,[0,0,9.81],np.radians([0,10,0]))
        self.assertEqual(imu.accel_raw.tolist(),[0,0,16384])
        self.assertEqual(imu.gyro_raw.tolist(),[0,1310,0])
        before=imu.gyro_raw.copy()
        imu.update(.001,[0,0,9.81],np.radians([0,20,0]))
        np.testing.assert_array_equal(imu.gyro_raw,before)

    def test_delay_noise_reproducibility_and_clipping(self):
        a=Mpu6500(PROFILE['imu'],123);b=Mpu6500(PROFILE['imu'],123)
        for i in range(100):
            for imu in (a,b):imu.update(i*.001,[0,0,9.81],[0,0,0])
        np.testing.assert_array_equal(a.accel_raw,b.accel_raw)
        c=Mpu6500(noiseless())
        c.update(0,[0,0,9.81],[0,0,0])
        for i in range(1,101):c.update(i*.001,[0,0,98.1],np.radians([0,1000,0]))
        self.assertTrue(c.saturated)
        self.assertEqual(c.accel_raw[2],32767)
        self.assertLessEqual(abs(c.gyro_raw[1]/131),250)
        # At low frequency the ramp lag approximates the quoted total group delay.
        c=Mpu6500(noiseless())
        for i in range(501):
            t=i*.001;c.update(t,[t*9.81,0,9.81],np.radians([t*100,0,0]))
        acc,gyro=c.read()
        self.assertAlmostEqual(.5-acc[0]/9.81,.0118,delta=.0002)
        self.assertAlmostEqual(.5-math.degrees(gyro[0])/100,.0059,delta=.0002)

    def test_imu_frame_and_confirmed_track(self):
        robot=MujocoRobot();robot.start_support(0)
        site=robot.m.site('imu_site').id
        rotation=robot.d.site_xmat[site].reshape(3,3)
        np.testing.assert_allclose(rotation[:,0],[0,1,0],atol=1e-15)
        np.testing.assert_allclose(rotation[:,1],[-1,0,0],atol=1e-15)
        self.assertEqual(robot.wheel_track_m,0.16444)
        centers=[]
        for name in ('ruota_sx','ruota_dx'):
            body=robot.m.body(name).id
            candidates=np.flatnonzero((robot.m.geom_bodyid==body)&
                                     (robot.m.geom_type==mujoco.mjtGeom.mjGEOM_CYLINDER))
            self.assertEqual(len(candidates),1)
            centers.append(robot.d.geom_xpos[candidates[0]])
        self.assertAlmostEqual(float(np.linalg.norm(centers[1]-centers[0])),robot.wheel_track_m)
        robot.d.qvel[robot.root_v+3]=-1
        mujoco.mj_forward(robot.m,robot.d)
        self.assertAlmostEqual(robot.read_imu()[1][1],1)

    def test_full_pipeline_keeps_balance_and_logs_all_axes(self):
        with tempfile.TemporaryDirectory() as folder:
            args=parse_args(['--mode','lqr','--headless','--duration','12',
                            '--imu-model','mpu6500','--encoder-model','motor'])
            with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(run(args,folder),0)
            p=next(Path(folder).glob('*.csv'))
            with p.open() as stream:rows=list(csv.DictReader(stream))
            self.assertIn('imu_gz_raw',rows[-1])
            self.assertIn('encoder_dx_count',rows[-1])
            status=json.loads(p.with_suffix('.status.log').read_text())
            self.assertTrue(status['settled'])
            metadata=json.loads(p.with_suffix('.json').read_text())
            self.assertEqual(metadata['schema_version'],10)
            self.assertFalse(metadata['sensor_models']['encoder_cpr_hardware_validated'])
