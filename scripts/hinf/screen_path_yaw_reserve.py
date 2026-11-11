#!/usr/bin/env python3
"""Offline Hinf path yaw-headroom sweep using the current MuJoCo model."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'config/controllers/hinf_path_40cm_28p5s.txt'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--reserve', type=float, nargs='+', default=[0, 3, 6, 9])
    parser.add_argument('--voltage', type=float, default=11.318)
    parser.add_argument('--initial-error', type=float, default=-2.895)
    args = parser.parse_args()
    if not all(math.isfinite(v) and 0 <= v < 12 for v in args.reserve):
        parser.error('Each yaw reserve must be finite and in [0, 12) percent')
    if args.output_dir.exists():
        parser.error('Output directory exists; use a new directory')
    args.output_dir.mkdir(parents=True)
    source = BASE.read_text()
    results = []
    for reserve in args.reserve:
        name = f'reserve_{reserve:g}'
        config = args.output_dir / (name + '.txt')
        config.write_text(source + f'\nturn_yaw_reserve_percent {reserve:g}\n')
        folder = args.output_dir / name
        command = [sys.executable, '-B', str(ROOT / 'scripts/digital_twin.py'),
                   'simulate', '--case', 'hinf-path', '--path-duration', '33',
                   '--hinf-pwm', '12', '--voltage', str(args.voltage),
                   '--initial-error', str(args.initial_error),
                   '--hinf-path-config', str(config.resolve()),
                   '--output-dir', str(folder.resolve())]
        completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        (args.output_dir / (name + '.console.txt')).write_text(completed.stdout + completed.stderr)
        case = folder / 'hinf-path'
        summary = json.loads((case / 'summary.json').read_text())
        with (case / 'simulation.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        last = rows[-1]
        at_turn_end = rows[min(1750, len(rows)-1)]
        applied = [(float(r['pwm_sx_applied_percent']),
                    float(r['pwm_dx_applied_percent'])) for r in rows]
        result = dict(reserve_percent=reserve, success=summary['success'],
                      fault=summary['fault'], duration_s=summary['simulated_duration_s'],
                      yaw_at_turn_end_deg=math.degrees(float(at_turn_end['yaw_rad'])),
                      final_yaw_deg=math.degrees(float(last['yaw_rad'])),
                      final_xy_m=[float(last['true_x_m']), float(last['true_y_m'])],
                      return_error_m=math.hypot(float(last['true_x_m']), float(last['true_y_m'])),
                      max_abs_pitch_deg=max(abs(float(r['pitch_fused_deg'])) for r in rows),
                      saturated_fraction=sum(max(abs(sx), abs(dx)) >= 11.999
                                             for sx, dx in applied) / len(rows),
                      config=str(config), figure=str(case / 'simulation.svg'))
        results.append(result)
        print(json.dumps(result), flush=True)
    (args.output_dir / 'results.json').write_text(json.dumps(results, indent=2) + '\n')


if __name__ == '__main__':
    main()
