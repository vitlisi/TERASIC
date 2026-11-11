#!/usr/bin/env python3
"""Run the deployed GA controller, including its one-sample feedback delay.

Uses the same offline CLI as digital_twin.py; no hardware entry point is called.
"""
import ctypes
from pathlib import Path
import subprocess
import sys

import digital_twin as twin


def load_ga_bridge(folder, kind):
    if kind not in ('stand', 'path'):
        raise ValueError('GA supports stand and path only')
    library = Path(folder)/(kind+'_ga.so')
    command = ['g++', '-std=c++98', '-O2', '-Wall', '-Wextra', '-Werror',
               '-shared', '-fPIC']
    if kind == 'path':
        command.append('-DTWIN_PATH')
    subprocess.run(command+[str(twin.ROOT/'diagnostics/digital_twin_ga_bridge.cpp'),
                            '-o', str(library), '-lrt'], check=True)
    core = ctypes.CDLL(str(library))
    core.twin_create.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_double]
    core.twin_create.restype = ctypes.c_void_p
    core.twin_destroy.argtypes = [ctypes.c_void_p]
    core.twin_step.argtypes = ([ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]
                              +[ctypes.c_double]*8+[ctypes.POINTER(ctypes.c_double)])
    core.twin_step.restype = ctypes.c_int
    return core


def main():
    if 'simulate' in sys.argv:
        for option, filename in [('stand', 'ga_stand.txt'), ('path', 'ga_path.txt')]:
            flag = '--'+option+'-config'
            if not any(a == flag or a.startswith(flag+'=') for a in sys.argv):
                sys.argv.extend([flag, str(twin.ROOT/'config/controllers'/filename)])
    original = twin.load_bridge
    original_simulate = twin.simulate_case
    def simulate_ga(args, kind, folder, build):
        summary = original_simulate(args, kind, folder, build)
        summary['controller_type'] = 'GA LQR, one-sample feedback delay'
        summary['feedback_delay_samples'] = 1
        for relative in ('scripts/simulate_ga_twin.py',
                         'diagnostics/digital_twin_ga_bridge.cpp',
                         'diagnostics/lqr_ga_feedback_delay.h'):
            summary['sha256'][relative] = twin.digest(twin.ROOT/relative)
        twin.write_json(folder/'summary.json', summary)
        return summary
    twin.load_bridge = load_ga_bridge
    twin.simulate_case = simulate_ga
    try:
        return twin.main()
    finally:
        twin.load_bridge = original
        twin.simulate_case = original_simulate


if __name__ == '__main__':
    raise SystemExit(main())
