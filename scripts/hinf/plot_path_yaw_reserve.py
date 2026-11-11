#!/usr/bin/env python3
"""Plot complete Hinf path yaw-reserve screening traces in English."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-report-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sweep', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = json.loads((args.sweep / 'results.json').read_text())
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout='constrained')
    for entry in results:
        name = f"Yaw reserve {entry['reserve_percent']:g}%"
        path = args.sweep / f"reserve_{entry['reserve_percent']:g}" / 'hinf-path/simulation.csv'
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        t = [float(r['t_s']) for r in rows]
        yaw = [math.degrees(float(r['yaw_rad'])) for r in rows]
        axes[0, 0].plot(t, yaw, label=name)
        axes[0, 1].plot(t, [float(r['pitch_fused_deg']) for r in rows], label=name)
        axes[1, 0].plot([float(r['true_x_m']) for r in rows],
                        [float(r['true_y_m']) for r in rows], label=name)
    reference_path = args.sweep / 'reserve_0/hinf-path/simulation.csv'
    with reference_path.open() as stream:
        reference = list(csv.DictReader(stream))
    axes[0, 0].plot([float(r['t_s']) for r in reference],
                    [math.degrees(float(r['yaw_reference_rad'])) for r in reference],
                    'k--', label='Requested yaw')
    axes[0, 0].set(ylabel='Yaw [deg]', xlabel='Time since release [s]', xlim=(0, 33))
    axes[0, 1].set(ylabel='Estimated pitch error [deg]', xlabel='Time since release [s]', xlim=(0, 33))
    axes[1, 0].plot([0], [0], 'ko', label='Start / planned return')
    axes[1, 0].set(xlabel='True X in MuJoCo [m]', ylabel='True Y in MuJoCo [m]', aspect='equal')
    reserve = [r['reserve_percent'] for r in results]
    error_cm = [100 * r['return_error_m'] if r['success'] else float('nan') for r in results]
    axes[1, 1].plot(reserve, error_cm, 'o-', color='tab:blue', label='Return error (completed only)')
    axes[1, 1].set(xlabel='Yaw reserve [% PWM]', ylabel='Return error [cm]')
    for ax in axes.flat:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.suptitle('H-infinity path in MuJoCo — yaw headroom sweep at 12% PWM\n'
                 'Complete traces from t=0; 9% reserve stops on attitude limit')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.output.with_suffix('.pdf'))
    fig.savefig(args.output.with_suffix('.png'), dpi=200)
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
