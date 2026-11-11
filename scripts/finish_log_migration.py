#!/usr/bin/env python3
"""Remove only the temporary active-GA symlink after its run finishes."""
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[1]
LINK = ROOT/'logs/final_campaign/09_ga/stand/uncertainties_2pct/ga_20261110_113003_e69593'
TARGET = ROOT/'logs/mujoco/ga/uncertainties_2pct/ga_20261110_113003_e69593'

def main():
    while LINK.is_symlink():
        if (TARGET/'summary.json').exists() or (TARGET/'error.json').exists():
            if LINK.resolve() != TARGET.resolve():
                raise RuntimeError('Compatibility link changed; refusing cleanup')
            LINK.unlink()
            parent = LINK.parent
            while parent != ROOT/'logs':
                try:
                    parent.rmdir()
                except OSError:
                    break
                parent = parent.parent
            print('Temporary GA path removed; logs now use real/ and mujoco/.', flush=True)
            return
        time.sleep(5)

if __name__ == '__main__':
    main()
