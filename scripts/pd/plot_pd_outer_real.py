#!/usr/bin/env python3
"""Plot a complete real PD outer-loop acquisition and its feedback terms."""
import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-report-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--kx', type=float, default=2.0)
    parser.add_argument('--kv', type=float, default=8.0)
    args = parser.parse_args()
    with args.csv.open() as stream:
        rows = list(csv.DictReader(stream))
    required = ('t_ns', 'theta_fused_deg', 'gyro_deg_s', 'position_m',
                'velocity_m_s', 'outer_offset_deg', 'pwm_percent')
    if not rows or any(key not in rows[0] for key in required):
        parser.error('A nonempty real PD outer-loop CSV is required')
    def column(key):
        return np.array([float(r[key]) for r in rows])
    t = (column('t_ns') - float(rows[0]['t_ns'])) / 1e9
    if np.any(np.diff(t) <= 0):
        parser.error('Timestamps must increase strictly')
    pitch, target = column('theta_fused_deg'), column('outer_offset_deg')
    gyro, position, velocity, pwm = (column(k) for k in
                                      ('gyro_deg_s', 'position_m', 'velocity_m_s', 'pwm_percent'))
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True, layout='constrained')
    def panel(ax, ylabel, *series):
        for values, label, style in series:
            ax.plot(t, values, style, linewidth=1.2, label=label)
        ax.set(ylabel=ylabel, xlim=(0, max(t[-1], .01)))
        ax.grid(alpha=.25)
        if len(series) > 1:
            ax.legend(loc='best')
    panel(axes[0, 0], 'Pitch error [deg]',
          (pitch, 'Measured pitch error', 'C0-'),
          (target, 'Outer-loop pitch target', 'C1--'))
    panel(axes[0, 1], 'Pitch rate [deg/s]', (gyro, 'IMU gyro', 'C0-'))
    panel(axes[1, 0], 'Longitudinal position [m]',
          (position, 'Encoder odometry', 'C0-'),
          (np.zeros_like(t), 'Release position', 'k--'))
    panel(axes[1, 1], 'Longitudinal velocity [m/s]',
          (velocity, 'Encoder-derived', 'C0-'))
    panel(axes[2, 0], 'Applied PWM [%]',
          (pwm, 'Both wheels', 'C0-'),
          (np.full_like(t, 15), '+15% cap', 'C3--'),
          (np.full_like(t, -15), '-15% cap', 'C3--'))
    panel(axes[2, 1], 'Pitch target components [deg]',
          (-args.kx*position, 'Position contribution', 'C2-'),
          (-args.kv*velocity, 'Velocity contribution', 'C4-'),
          (target, 'Clipped total', 'k--'))
    for ax in axes[-1]:
        ax.set_xlabel('Time since first logged sample [s]')
    fig.suptitle('Real robot — PD with position and velocity outer loop\n'
                 f'{args.csv.stem}; complete acquisition to safety stop ({t[-1]:.2f} s)')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for suffix in ('svg', 'pdf', 'png'):
        path = args.output_dir / (args.csv.stem + '.' + suffix)
        fig.savefig(path, dpi=200)
        print(path)
    plt.close(fig)


if __name__ == '__main__':
    main()
