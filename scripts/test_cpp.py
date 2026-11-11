#!/usr/bin/env python3
"""Compile and run offline C++ tests; no access to the robot."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    with tempfile.TemporaryDirectory(prefix='terasic-cpp-') as folder:
        for source in sorted((ROOT/'tests').glob('test_*.cpp')):
            binary = Path(folder)/source.stem
            subprocess.run(['g++', '-std=c++98', '-O2', '-Wall', '-Wextra', '-Werror',
                            str(source), '-o', str(binary), '-lrt'], check=True)
            args = [str(binary)]
            if source.stem == 'test_lqr_path_simple':
                args.append(str(ROOT/'models/lqr/hardware_path_simple_40cm_28p5s_20261019.txt'))
            if source.stem == 'test_inverse_lut_protocol':
                # Synthetic bounded protocol input; no historical logs required.
                plan = Path(folder)/'inverse_protocol.txt'
                plan.write_text('0 0 3 0.6 3\n0 1 3 0.6 4\n'
                                '0 0 4 0.8 4\n0 1 4 0.8 5\n'
                                '0 0 6 1.2 6\n0 1 6 1.2 7\n')
                args.append(str(plan))
            subprocess.run(args, check=True)
            print(source.name+': OK', flush=True)


if __name__ == '__main__':
    main()
