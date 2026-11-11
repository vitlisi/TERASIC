"""GA integration: frozen design, reproducible plants, scoring and manual export."""
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src_python'))
from ga_lqr import (apply_scenario, breed, decode, evaluate, load_design,
                    rank_trials, save_json, scenarios, summarize_trial, validate_design, scale_input_penalty)
from motion_lqr import design_motion_lqr
from mujoco_robot import MujocoRobot
from mujoco_sim import parse_args


class GaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal = design_motion_lqr()

    def test_input_penalty_scale_changes_r_and_k_consistently(self):
        original = copy.deepcopy(self.nominal)
        for scale in (.5, 1., 2.):
            changed = scale_input_penalty(self.nominal,scale)
            validate_design(changed)
            np.testing.assert_allclose(changed['R'],np.asarray(original['R'])*scale)
            for key in ('Q','Ad','Bd_duty'):
                self.assertEqual(changed[key],original[key])
            if scale == 1.:
                np.testing.assert_allclose(changed['K_duty'],original['K_duty'])
        self.assertEqual(self.nominal,original)
        for scale in (0.,-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                scale_input_penalty(self.nominal,scale)

    def test_zero_genes_reproduce_baseline_and_export(self):
        design = decode(self.nominal,np.zeros(6))
        for key in ('Q','R','K_duty','K_pwm_percent','Ad','Bd_duty'):
            np.testing.assert_array_equal(design[key],self.nominal[key])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'best.json'
            save_json(path,design)
            self.assertEqual(load_design(path),design)
        self.assertLess(design['ga']['delayed_spectral_radius'],1)

    def test_eight_genes_optimize_both_input_weights_and_keep_legacy(self):
        zero = decode(self.nominal, np.zeros(8))
        np.testing.assert_array_equal(zero['K_duty'], self.nominal['K_duty'])
        genes = np.zeros(8); genes[6:] = [.3, -.2]
        changed = decode(self.nominal, genes)
        np.testing.assert_allclose(np.diag(changed['R']),
                                   np.diag(self.nominal['R'])*10**genes[6:])
        np.testing.assert_array_equal(changed['Q'], self.nominal['Q'])
        validate_design(changed)
        self.assertFalse(np.allclose(changed['K_duty'], self.nominal['K_duty']))
        population = np.random.default_rng(1).uniform(-1,1,(4,8))
        children = breed(population, [[0,0,i] for i in range(4)], np.random.default_rng(2))
        self.assertEqual(children.shape, (4,8))

    def test_calibrated_profile_is_used_in_design_and_rollout(self):
        profile = json.loads((ROOT/'config/mujoco_motors_sx_dx_20261030.json').read_text())['wheels']
        nominal = design_motion_lqr(wheel_profiles=profile)
        self.assertEqual(nominal['wheel_profiles'], profile)
        self.assertFalse(np.allclose(nominal['Ad'], self.nominal['Ad']))
        protocol = dict(duration_s=3.,max_pwm=50.,release_pitch_deg=3.,feedback_delay_samples=1)
        with tempfile.TemporaryDirectory() as folder:
            _, trials = evaluate(decode(nominal,np.zeros(8)),scenarios(1,1000,'profile'),protocol,folder)
            meta = json.loads(Path(trials[0]['csv']).with_suffix('.json').read_text())
            indices = [int(MujocoRobot().m.joint('perno_'+w).dofadr[0]) for w in ('sx','dx')]
            for w, i in zip(('sx','dx'), indices):
                self.assertAlmostEqual(meta['physical_parameters']['joint_armature_kg_m2'][i],
                                       profile[w]['armature_kg_m2'])
            self.assertEqual(meta['lqr_design']['wheel_profiles'],profile)

    def test_historical_rounded_track_is_accepted_without_changing_gains(self):
        old = dict(self.nominal,track_width_m=.1644)
        loaded = validate_design(old)
        self.assertEqual(loaded,old)

    def test_export_rejects_wrong_conventions_and_inconsistent_k(self):
        for change in (dict(dt_s=.02),dict(input_order=['duty_dx','duty_sx']),
                       dict(K_pwm_percent=np.zeros((2,6)).tolist()),
                       dict(track_width_m=.2),dict(Q=(-np.eye(6)).tolist())):
            with self.assertRaises(ValueError):
                validate_design(dict(self.nominal,**change))
        for genes in ([0]*5,[0,0,0,0,0,float('nan')],[2]*6):
            with self.assertRaises(ValueError):
                decode(self.nominal,genes)

    def test_candidate_uses_frozen_dynamics_and_positive_weights(self):
        design = decode(self.nominal,[.1,-.2,.3,.4,-.5,.6])
        np.testing.assert_array_equal(design['Ad'],self.nominal['Ad'])
        np.testing.assert_array_equal(design['Bd_duty'],self.nominal['Bd_duty'])
        self.assertTrue(np.all(np.diag(design['Q'])>0))
        np.testing.assert_array_equal(design['R'],self.nominal['R'])
        self.assertFalse(np.allclose(design['K_duty'],self.nominal['K_duty']))

    def test_scenario_changes_effective_parameters_without_contamination(self):
        fresh, changed = MujocoRobot(),MujocoRobot()
        scenario = scenarios(2,1000,'train')[1]
        before = copy.deepcopy(scenario)
        apply_scenario(changed,scenario)
        self.assertEqual(before,scenario)
        np.testing.assert_allclose(changed.m.geom_friction,fresh.m.geom_friction*scenario['friction_scale'])
        np.testing.assert_allclose(changed.m.body_inertia[changed.body_id],
            fresh.m.body_inertia[fresh.body_id]*scenario['chassis_inertia_scale'])
        for i,name in enumerate(('perno_sx','perno_dx')):
            adr = int(fresh.m.joint(name).dofadr[0])
            self.assertAlmostEqual(changed.m.dof_armature[adr],fresh.m.dof_armature[adr]*scenario['armature_scale'][i])
        np.testing.assert_allclose(changed.m.actuator_gear[:,0],
                                   fresh.m.actuator_gear[:,0]*scenario['motor_scale'])
        nominal = MujocoRobot()
        apply_scenario(nominal,scenarios(1,1000,'train')[0])
        np.testing.assert_array_equal(nominal.m.body_inertia,fresh.m.body_inertia)
        np.testing.assert_array_equal(nominal.m.actuator_gear,fresh.m.actuator_gear)
        self.assertEqual(scenarios(5,1000,'train'),scenarios(5,1000,'train'))

    def test_short_failed_trial_cannot_win_on_small_cost(self):
        good = rank_trials([dict(failed=False,missing_time_s=0,cost=1000.)])
        bad = rank_trials([dict(failed=True,missing_time_s=9,cost=0.)])
        self.assertLess(tuple(good),tuple(bad))

    def test_physical_protection_is_failure_software_error_is_fatal_and_interrupt_stops(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'trial.csv'
            path.write_text('phase\n')
            save_json(path.with_suffix('.json'),{})
            status = path.with_suffix('.status.log')
            save_json(status,dict(success=False,stop_reason=
                'errore: ValueError: Contatore encoder simulato saturo; nessun rollover'))
            self.assertTrue(summarize_trial(path)['failed'])
            save_json(status,dict(success=False,stop_reason='errore: KeyError: campo_mancante'))
            with self.assertRaises(RuntimeError):
                summarize_trial(path)
            save_json(status,dict(success=False,stop_reason='interruzione utente'))
            with self.assertRaises(KeyboardInterrupt):
                summarize_trial(path)

    def test_elitism_bounds_and_seed_reproducibility(self):
        population = np.random.default_rng(1).uniform(-1,1,(12,6))
        ranks = [[0,0,float(12-i)] for i in range(12)]
        first = breed(population,ranks,np.random.default_rng(42))
        second = breed(population,ranks,np.random.default_rng(42))
        np.testing.assert_array_equal(first,second)
        np.testing.assert_array_equal(first[:2],population[[11,10]])
        self.assertLessEqual(np.max(np.abs(first)),1)

    def test_ten_second_hold_allowed_but_short_motion_rejected(self):
        args = parse_args(['--mode','lqr-motion','--move-distance-m','0','--duration','10'])
        self.assertEqual(args.duration,10)
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            parse_args(['--mode','lqr-motion','--duration','10'])

    def test_nonlinear_evaluator_fixed_gains_and_repeatability(self):
        design = decode(self.nominal,np.zeros(6))
        protocol = dict(duration_s=10.,max_pwm=50.,release_pitch_deg=3.,feedback_delay_samples=1)
        trials = scenarios(2,1000,'test')
        with tempfile.TemporaryDirectory() as folder:
            rank, results = evaluate(design,trials,protocol,folder)
            again_rank, again = evaluate(design,trials,protocol)
            self.assertEqual(rank,again_rank)
            for result, other in zip(results,again):
                self.assertTrue(result['survived'])
                self.assertEqual(result['cost'],other['cost'])
                self.assertLessEqual(result['peak_pwm_percent'],50)
                meta = json.loads(Path(result['csv']).with_suffix('.json').read_text())
                self.assertEqual(meta['lqr_design']['K_duty'],design['K_duty'])
                self.assertIn('actuator_gear',meta['physical_parameters'])
                self.assertIn('ga_scenario',meta)
                self.assertAlmostEqual(meta['release_true_pitch_deg'],3.,delta=.05)


if __name__ == '__main__':
    unittest.main()
