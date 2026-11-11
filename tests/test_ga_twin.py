"""The transferred GA core must delay feedback, unlike the ordinary LQR core."""
import ctypes
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from digital_twin import config_values
from simulate_ga_twin import load_ga_bridge


class GaTwinTest(unittest.TestCase):
    def test_command_uses_previous_feedback_with_frozen_gains(self):
        config = ROOT/'config/controllers/ga_stand.txt'
        cfg = config_values(config)
        keys = ('position', 'pitch', 'yaw', 'velocity', 'pitch_rate', 'yaw_rate')
        gains = np.array([[cfg['k_'+wheel+'_'+key] for key in keys]
                          for wheel in ('sx', 'dx')])
        with tempfile.TemporaryDirectory() as folder:
            core = load_ga_bridge(folder, 'stand')
            handle = core.twin_create(bytes(config), 0., 20.)
            self.assertTrue(handle)
            output = (ctypes.c_double*44)()
            previous = np.zeros(6)
            try:
                for i in range(50):
                    self.assertEqual(core.twin_step(handle, 32768, 32768,
                        .1*np.sin(i*.2), .2*np.cos(i*.2), 0., 0., 1., 16384.,
                        .01 if i else 0., i*.01, output), 1)
                    np.testing.assert_allclose(output[12:14],
                        -cfg['motor_sign']*gains@previous, atol=1e-10)
                    previous = np.array(output[:6])
            finally:
                core.twin_destroy(handle)


if __name__ == '__main__':
    unittest.main()
