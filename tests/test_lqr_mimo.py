from pathlib import Path
import json
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


class MimoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder=tempfile.TemporaryDirectory();cls.binary=Path(cls.folder.name)/'mimo'
        subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror',
            str(ROOT/'src_cpp/terasic_reale/controllers/balance_lqr_mimo.cpp'),
            '-o',str(cls.binary),'-lrt'],check=True,capture_output=True)
        cls.config=ROOT/'models/lqr/hardware_mimo_shadow_20261019.txt'

    @classmethod
    def tearDownClass(cls):cls.folder.cleanup()

    def test_encoder_differential_allocation_and_mmio_stop(self):
        for name in ['test_lqr_mimo_core','test_lqr_mimo_mmio']:
            with self.subTest(program=name):
                binary=Path(self.folder.name)/name
                subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror',
                    str(ROOT/'tests'/(name+'.cpp')),'-o',str(binary),'-lrt'],check=True,capture_output=True)
                subprocess.run([str(binary)],check=True)

    def test_shadow_cannot_arm_before_hardware(self):
        checked=subprocess.run([str(self.binary),'check',str(self.config)],capture_output=True,text=True)
        self.assertEqual(checked.returncode,0,checked.stderr)
        blocked=subprocess.run([str(self.binary),'run',str(self.config),'3','15'],capture_output=True,text=True)
        self.assertEqual(blocked.returncode,2)
        self.assertIn('prima dell',blocked.stderr)
        self.assertNotIn('Scheduling',blocked.stderr)

    def test_legacy_channel_config_rejected_before_hardware(self):
        text=self.config.read_text().replace('encoder_sx_sign -1','encoder_sx_sign 1').replace('encoder_dx_sign 1','encoder_dx_sign -1')
        p=Path(self.folder.name)/'legacy.txt';p.write_text(text)
        result=subprocess.run([str(self.binary),'check',str(p)],capture_output=True,text=True)
        self.assertEqual(result.returncode,2)
        self.assertNotIn('Scheduling',result.stderr)

    def test_optional_narrow_startup_limit(self):
        original=self.config.read_text()
        for suffix,expected in [('startup_max_error_deg 0.5',0),('startup_max_error_deg 6',2),
                                ('startup_max_error_deg nan',2),('startup_max_error_deg 0',2),
                                ('startup_max_error_deg .5\nstartup_max_error_deg .5',2)]:
            path=Path(self.folder.name)/'startup.txt';path.write_text(original+'\n'+suffix+'\n')
            r=subprocess.run([str(self.binary),'check',str(path)],capture_output=True,text=True)
            self.assertEqual(r.returncode,expected,r.stderr)

    def test_old_single_output_file_rejected(self):
        result=subprocess.run([str(self.binary),'check',str(ROOT/'models/lqr/hardware_position_trial_20261018.txt')],
                              capture_output=True,text=True)
        self.assertEqual(result.returncode,2)

    def test_geometry_limit_and_matrix_completeness(self):
        text=self.config.read_text()
        for invalid in [text.replace('wheel_track_m 0.16444','wheel_track_m nan'),
                        text.replace('max_differential_pwm 3','max_differential_pwm 50'),
                        '\n'.join(x for x in text.splitlines() if not x.startswith('k_dx_yaw_rate '))]:
            p=Path(self.folder.name)/'invalid.txt';p.write_text(invalid)
            result=subprocess.run([str(self.binary),'check',str(p)],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)

    def test_longitudinal_common_feedback_is_preserved(self):
        c=json.loads((ROOT/'tests/fixtures/mimo_reference.json').read_text())
        old=dict(x.split() for x in (ROOT/'models/lqr/hardware_position_trial_20261018.txt').read_text().splitlines()
                 if x and not x.startswith('#'))
        k=c['K_pwm_percent']
        for col,key in zip([0,1,3,4],['k_position','k_pitch','k_velocity','k_pitch_rate']):
            self.assertAlmostEqual(.5*(k[0][col]+k[1][col]),float(old[key]),places=10)
        for col in [2,5]:self.assertAlmostEqual(k[0][col],-k[1][col],places=10)


if __name__=='__main__':unittest.main()
