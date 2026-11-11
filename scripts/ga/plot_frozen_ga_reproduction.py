#!/usr/bin/env python3
"""Compare frozen LQR and GA-LQR on matched current-twin simulations."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-report-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def read_case(kind, label, run):
    old = f'logs/final_campaign/09_ga/{kind}/{run}/{label}/{kind}/'
    manifest = json.loads((ROOT/'maintenance/log_layout_migration/manifest.json').read_text())
    paths = {row['old']: ROOT/row['new'] for row in manifest['files'] if row.get('old')}
    csv_path = paths[old+'simulation.csv']
    summary_path = paths[old+'summary.json']
    with csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    return ({key: np.array([float(r[key]) for r in rows])
             for key in rows[0] if key != 'phase'},
            json.loads(summary_path.read_text()), csv_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', default='reproduced_20261110')
    parser.add_argument('--output-root', type=Path,
                        default=ROOT / 'deliverables/final_campaign/09_ga')
    args = parser.parse_args()
    for kind in ('stand', 'path'):
        pair = {name: read_case(kind, name, args.run) for name in ('baseline', 'ga')}
        a, b = (pair[name][0] for name in ('baseline', 'ga'))
        if len(a['t_s']) != len(b['t_s']) or not np.allclose(a['t_s'], b['t_s']):
            raise ValueError('Time grids differ')
        if not all(pair[name][1]['success'] for name in pair):
            raise ValueError('Both cases must complete to compare full traces')
        fig, axes = plt.subplots(3, 2, figsize=(14, 11), layout='constrained')
        colors = {'baseline': 'tab:blue', 'ga': 'tab:orange'}
        labels = {'baseline': 'Baseline LQR', 'ga': 'Frozen GA-LQR'}
        for name, (data, _, _) in pair.items():
            t = data['t_s']
            pitch = data['pitch_fused_deg']
            position = data['position_m']
            yaw = np.degrees(data['yaw_rad'])
            speed = data['velocity_m_s']
            axes[0, 0].plot(t, pitch, color=colors[name], label=labels[name], lw=1)
            axes[0, 1].plot(t, position, color=colors[name], label=labels[name], lw=1)
            axes[1, 0].plot(t, yaw, color=colors[name], label=labels[name], lw=1)
            axes[1, 1].plot(t, speed, color=colors[name], label=labels[name], lw=1)
            axes[2, 0].plot(t, data['pwm_sx_applied_percent'],
                            color=colors[name], label=labels[name], lw=1)
            axes[2, 1].plot(t, data['pwm_dx_applied_percent'],
                            color=colors[name], label=labels[name], lw=1)
        if kind == 'path':
            axes[0, 1].plot(a['t_s'], a['position_reference_m'], 'k--', lw=1,
                            label='Position reference')
            axes[1, 0].plot(a['t_s'], np.degrees(a['yaw_reference_rad']), 'k--', lw=1,
                            label='Yaw reference')
        titles = ('Estimated pitch [deg]', 'Encoder position [m]',
                  'Estimated yaw [deg]', 'Encoder velocity [m/s]',
                  'Left applied PWM [%]', 'Right applied PWM [%]')
        for ax, title in zip(axes.flat, titles):
            ax.set(ylabel=title, xlabel='Time since control start [s]',
                   xlim=(0, a['t_s'][-1]))
            ax.grid(alpha=.25)
            ax.legend(loc='best', fontsize=8)
        fig.suptitle(f'Current MuJoCo — baseline LQR versus frozen GA-LQR ({kind})\n'
                     'Matched release, voltage, sensor seed and 20% PWM cap; complete traces from t=0')
        out = args.output_root / kind / args.run
        out.mkdir(parents=True, exist_ok=True)
        for ext in ('svg', 'pdf', 'png'):
            path = out / ('lqr_vs_ga.' + ext)
            fig.savefig(path, dpi=180)
            print(path)
        plt.close(fig)
        metrics = {}
        for name, (data, summary, folder) in pair.items():
            final = summary['final_xy_true_m']
            metrics[name] = {
                'tail_pitch_rms_deg': summary['tail_pitch_rms_deg'],
                'tail_speed_rms_m_s': summary['tail_speed_rms_m_s'],
                'return_error_m': math.hypot(*final),
                'success': summary['success'],
                'samples': summary['samples'],
                'csv': str(folder),
                'csv_sha256': hashlib.sha256(folder.read_bytes()).hexdigest(),
            }
        (out / 'metrics.json').write_text(json.dumps({
            'kind': kind, 'source': 'new 2026-11-10 reproduction, not recovered historical logs',
            'conditions': {'initial_error_deg': -2.895, 'voltage_V': 11.318,
                           'sensor_seed': 0, 'pwm_cap_percent': 20,
                           'duration_s': 10 if kind == 'stand' else 33},
            'ga_feedback_delay_samples': 1,
            'metrics': metrics}, indent=2) + '\n')


if __name__ == '__main__':
    main()
