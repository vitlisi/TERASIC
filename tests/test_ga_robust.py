"""Robust-GA fairness, held-out selection and persistent motor/sensor overrides."""
import copy
import contextlib
import importlib.util
import io
from unittest.mock import patch
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from ga_robust import schedule,run_search,validate_config
from ga_lqr import apply_scenario,decode,evaluate
from mujoco_robot import MujocoRobot
from motion_lqr import design_motion_lqr
from wheel_dynamics import configure_wheels


class RobustTests(unittest.TestCase):
    def config(self):
        return json.loads((ROOT/'config/ga/lqr_motor_uncertainties.json').read_text())

    def test_launcher_selects_new_protocol_without_running_optimizer(self):
        spec=importlib.util.spec_from_file_location('tune_robust_test',ROOT/'scripts/ga/tune_lqr_ga.py')
        launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)
        with tempfile.TemporaryDirectory() as temp, patch('ga_robust.run_search',return_value={'status':'mocked'}) as search:
            with contextlib.redirect_stdout(io.StringIO()):
                code=launcher.main(['--config',str(ROOT/'config/ga/lqr_motor_uncertainties.json'),
                                    '--population','4','--generations','1','--optimize-r',
                                    '--output-dir',temp,'--no-plots'])
            self.assertEqual(code,0);search.assert_called_once()
            nominal,config,batch=search.call_args.args
            self.assertTrue(config['optimize_r'])
            self.assertTrue(config['resample_each_generation'])
            self.assertIn('wheel_profiles',nominal)
            self.assertTrue((batch/'motor_parameters.json').exists())
            self.assertEqual(config['protocol']['cost_weights']['pitch'],4.)

    def test_schedule_repeatable_distinct_and_motor_only(self):
        c=self.config();a=schedule(c);self.assertEqual(a,schedule(c))
        all_banks=a['training_by_generation']+[a['selection'],a['validation']]
        seeds=[{s['seed'] for s in bank} for bank in all_banks]
        self.assertEqual(sum(map(len,seeds)),len(set.union(*seeds)))
        for bank in all_banks:
            for s in bank:
                self.assertEqual(s['friction_scale'],1.)
                self.assertEqual(s['chassis_inertia_scale'],1.)
                self.assertEqual(s['imu_noise_scale'],[1.,1.])
                self.assertNotEqual(s['encoder_noise_seed'],s['seed'])
                for key in ('motor_scale','armature_scale','damping_scale','joint_friction_scale'):
                    self.assertTrue(all(.9<=v<=1.1 for v in s[key]))
                if s['nominal_physics']:
                    self.assertEqual(s['damping_scale'],[1.,1.])
                    self.assertEqual(s['encoder_noise_std_counts'],0.)
                else:self.assertTrue(.1<=s['encoder_noise_std_counts']<=1.)

    def test_bad_ranges_seeds_and_disturbances_rejected(self):
        for mutate in (lambda c:c['uncertainties'].update(damping=1.),
                       lambda c:c.update(selection_seed=c['train_seed']),
                       lambda c:c['protocol'].update(disturbance_force_y_n=.2)):
            c=self.config();mutate(c)
            with self.assertRaises(ValueError):validate_config(c)

    def test_per_generation_cache_and_common_selection_without_validation_leak(self):
        c=self.config();c.update(population=4,generations=2)
        calls=[]
        def decoder(nominal,genes,delay):return dict(genes=list(genes),ga={})
        def evaluator(design,scenes,protocol,folder=None):
            calls.append((tuple(design['genes']),[s['name'] for s in scenes],folder))
            zero=not any(design['genes']);first=scenes[0]['name']
            cost=(10. if zero else 1.) if first.startswith('train') else ((0. if zero else 5.) if first.startswith('selection') else (99. if zero else 0.))
            return [0,0.,cost],[dict(failed=False,cost=cost)]
        with tempfile.TemporaryDirectory() as temp:
            result=run_search({},c,temp,evaluator,decoder)
            best=json.loads((Path(temp)/'best.json').read_text())
            self.assertEqual(best['genes'],[0.]*8)
            self.assertEqual(best['ga']['selection_rank'],[0,0.,0.])
            self.assertEqual(result['validation']['ga'],[0,0.,99.])
            history=json.loads((Path(temp)/'history.json').read_text())
            self.assertEqual(len(history),8)
            train_calls=[x for x in calls if x[1][0].startswith('train')]
            g1={x[0] for x in train_calls if 'g001' in x[1][0]}
            g2={x[0] for x in train_calls if 'g002' in x[1][0]}
            self.assertGreaterEqual(len(g1&g2),2)
            for gen in range(2):
                self.assertEqual(len({r['scenario_bank_hash'] for r in history if r['generation']==gen}),1)
            last_selection=max(i for i,x in enumerate(calls) if x[1][0].startswith('selection'))
            first_validation=min(i for i,x in enumerate(calls) if x[1][0].startswith('validation'))
            self.assertLess(last_selection,first_validation)

    def test_on_off_motor_factors_persist_and_sources_unchanged(self):
        profile=json.loads((ROOT/'config/mujoco_motors_sx_dx_20261030.json').read_text())['wheels']
        original=copy.deepcopy(profile);r=MujocoRobot();r.wheel_dynamics=configure_wheels(r.m,profile)
        s=schedule(self.config())['training_by_generation'][0][1];before=copy.deepcopy(s)
        apply_scenario(r,s)
        for command,prefix in [([0,0],'off_'),([5,-5],''),([0,0],'off_')]:
            r.set_motors(*command)
            for i,w in enumerate(('sx','dx')):
                adr=int(r.m.joint('perno_'+w).dofadr[0])
                self.assertAlmostEqual(r.m.dof_damping[adr],profile[w][prefix+'damping_Nm_s_rad']*s['damping_scale'][i])
                self.assertAlmostEqual(r.m.dof_frictionloss[adr],profile[w][prefix+'frictionloss_Nm']*s['joint_friction_scale'][i])
                self.assertAlmostEqual(r.m.dof_armature[adr],profile[w]['armature_kg_m2']*s['armature_scale'][i])
        self.assertEqual(profile,original);self.assertEqual(s,before)

    def test_sensor_noise_scales_config_and_reproducibility(self):
        path=ROOT/'config/mujoco_sensors_mpu6500.json';original=json.loads(path.read_text())
        a,b=MujocoRobot(),MujocoRobot()
        ca=a.configure_sensors('mpu6500','motor',path,123,noise_scales=[2.,.5])
        cb=b.configure_sensors('mpu6500','motor',path,123,noise_scales=[2.,.5])
        self.assertEqual(ca,cb)
        self.assertAlmostEqual(ca['imu']['accel_noise_density_g_sqrt_hz'],2*original['imu']['accel_noise_density_g_sqrt_hz'])
        self.assertAlmostEqual(ca['imu']['gyro_noise_density_deg_s_sqrt_hz'],.5*original['imu']['gyro_noise_density_deg_s_sqrt_hz'])
        for i in range(1,101):
            for r in (a,b):r.imu_model.update(i*.001,[0,0,9.81],[0,0,0])
        np.testing.assert_array_equal(a.imu_model.accel_raw,b.imu_model.accel_raw)
        self.assertEqual(original,json.loads(path.read_text()))

    def test_two_short_rollouts_record_effective_uncertainties_and_noise(self):
        c=self.config();profile=json.loads((ROOT/'config/mujoco_motors_sx_dx_20261030.json').read_text())['wheels']
        d=decode(design_motion_lqr(wheel_profiles=profile),np.zeros(8))
        protocol=dict(c['protocol'],duration_s=3.)
        scenes=schedule(c)['training_by_generation'][0][:2]
        with tempfile.TemporaryDirectory() as temp:
            _,trials=evaluate(d,scenes,protocol,temp)
            for s,t in zip(scenes,trials):
                meta=json.loads(Path(t['csv']).with_suffix('.json').read_text())
                self.assertEqual(meta['ga_scenario']['damping_scale'],s['damping_scale'])
                self.assertEqual(meta['lqr_design']['K_duty'],d['K_duty'])
                self.assertEqual(meta['sensor_models']['configuration']['imu']['gyro_noise_density_deg_s_sqrt_hz'],.01)
                r=MujocoRobot()
                for i,w in enumerate(('sx','dx')):
                    adr=int(r.m.joint('perno_'+w).dofadr[0])
                    self.assertAlmostEqual(meta['physical_parameters']['joint_damping_Nm_s_rad'][adr],profile[w]['damping_Nm_s_rad']*s['damping_scale'][i])

if __name__=='__main__':unittest.main()
