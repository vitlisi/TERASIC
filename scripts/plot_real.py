#!/usr/bin/env python3
"""Plot an existing real PD/LQR/Hinf/GA acquisition; never starts hardware."""
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
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    with args.csv.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows or 't_ns' not in rows[0]:
        parser.error('A nonempty real acquisition with t_ns is required')
    t = np.array([(int(r['t_ns'])-int(rows[0]['t_ns']))/1e9 for r in rows])
    if np.any(np.diff(t) <= 0):
        parser.error('Strictly increasing real timestamps are required')
    profile_end = next((float(t[i]) for i, row in enumerate(rows)
                        if row.get('schedule_finished') == '1'), None)
    pd_record = 'theta_fused_deg' in rows[0] and 'pitch_fused_deg' not in rows[0]
    fields = [(['pitch_fused_deg', 'theta_fused_deg'],
               'Pitch error from fixed reference [deg]' if pd_record else 'Estimated pitch [deg]'),
              (['gyro_deg_s'], 'Pitch angular velocity [deg/s]'),
              (['position_m'], 'Odometry position [m]'),
              (['velocity_m_s'], 'Odometry velocity [m/s]'),
              (['yaw_fused_rad', 'yaw_encoder_rad'], 'Estimated yaw [rad]'),
              (['yaw_rate_gyro_rad_s', 'yaw_rate_encoder_rad_s'], 'Estimated yaw rate [rad/s]'),
              (['pwm_sx_applied_percent', 'pwm_percent'], 'Applied left PWM [%]'),
              (['pwm_dx_applied_percent', 'pwm_percent'], 'Applied right PWM [%]')]
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=True, layout='constrained')
    for ax, (alternatives, label) in zip(axes.flat, fields):
        field = next((key for key in alternatives if key in rows[0]), None)
        if field:
            values = [float(r[field]) if r[field] else np.nan for r in rows]
            ax.plot(t, values, linewidth=1)
            if pd_record and 'outer_offset_deg' in rows[0] and ax is axes[0, 0]:
                ax.plot(t, [float(r['outer_offset_deg']) for r in rows],
                        linewidth=1, linestyle='--', label='Outer-loop pitch target (relative)')
                ax.legend(loc='best')
        else:
            ax.text(.5, .5, 'Not recorded', transform=ax.transAxes, ha='center')
        ax.set(xlabel='Time since first logged sample [s]', ylabel=label, xlim=(0, max(t[-1], .01)))
        if profile_end is not None:
            ax.axvline(profile_end, color='0.35', linestyle='--', linewidth=.8)
        ax.grid(alpha=.25)
    title = 'Real robot — complete recorded trace\n'+args.csv.stem
    if profile_end is not None:
        title += f'\nDashed line: motion profile completed at {profile_end:.2f} s; control recording continues'
    fig.suptitle(title)
    folder = args.output_dir or args.csv.parent/'plot'
    folder.mkdir(parents=True, exist_ok=True)
    for ext in ('svg', 'pdf', 'png'):
        path = folder/(args.csv.stem+'.'+ext)
        fig.savefig(path, dpi=200)
        print(path)
    plt.close(fig)


if __name__ == '__main__':
    main()
