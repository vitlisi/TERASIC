#!/usr/bin/env python3
"""Export motion telemetry for straight and timed-return sequences."""
import csv
import json
from pathlib import Path
import argparse
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-report-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('csv',type=Path)
parser.add_argument('--format',choices=('pdf','svg','both'),default='both')
parser.add_argument('--free-only', action='store_true', help='Explicit optional free-motion detail; default keeps the full acquisition')
args=parser.parse_args()
path = args.csv.resolve()
with path.open() as stream:
    rows = list(csv.DictReader(stream))
if args.free_only:
    rows = [row for row in rows if row['phase'] == 'free']
if not rows or 'distance_ref_m' not in rows[0]:
    raise SystemExit('A six-state LQR/H-infinity log is required')
def series(name):
    values=np.array([float(row[name]) if row[name] not in ('', None) else np.nan for row in rows])
    # RL fixed-target resets: display estimates and references in the same frame as truth.
    if name=='distance_ref_m':
        values+=np.array([float(row.get('initial_position_offset_m',0.) or 0.) for row in rows])
    if name in ('x_rho_rad','yaw_ref_rad'):
        values+=np.radians([float(row.get('initial_yaw_offset_deg',0.) or 0.) for row in rows])
    return values
t = series('t_free_s' if args.free_only else 't_s')
t = t - t[0]
fig, axes = plt.subplots(3, 2, figsize=(12, 10))
for field, label in [('pitch_true_deg', 'MuJoCo'), ('pitch_est_deg', 'IMU estimate'), ('pitch_target_deg', 'Reference')]:
    axes[0, 0].plot(t, series(field), label=label)
axes[0, 0].set(ylabel='Pitch [°]')
sequence = 'sequence_phase' in rows[0]
stationary = not sequence and np.nanmax(np.abs(series('distance_ref_m'))) < 1e-12
distance_fields = [('distance_ref_m', 'Reference s'), ('position_encoder_m', 'Odometry s'),
                   ('pose_true_y_m', 'Y MuJoCo' if sequence else 'MuJoCo')]
for field, label in distance_fields:
    axes[0, 1].plot(t, 100*series(field), label=label)
axes[0, 1].set(ylabel='Distance [cm]')
if sequence:
    axes[0, 1].set(ylabel='Accumulated longitudinal distance [cm]')
    axes[0, 1].plot(t, 100*series('planned_y_m'), label='Ideal out-and-back Y')
true_velocity = 'velocity_body_forward_m_s' if sequence else 'velocity_world_forward_m_s'
for field, label in [('velocity_ref_m_s', 'Reference'), ('velocity_encoder_m_s', 'Encoder + gyro'), (true_velocity, 'MuJoCo')]:
    axes[1, 0].plot(t, series(field), label=label)
axes[1, 0].set(ylabel='Velocity [m/s]')
for field, label in [('pwm_sx_applied_percent', 'Left'), ('pwm_dx_applied_percent', 'Right')]:
    axes[1, 1].plot(t, series(field), label=label)
axes[1, 1].set(ylabel='Motor PWM [%]')
for field, label in [('x_rho_rad', 'Gyro + encoder estimate'), ('yaw_true_rad', 'MuJoCo'), ('yaw_ref_rad', 'Reference')]:
    axes[2, 0].plot(t, np.degrees(series(field)), label=label)
axes[2, 0].set(ylabel='Yaw [°]')
axes[2, 1].plot(100*series('pose_true_x_m'), 100*series('pose_true_y_m'), label='MuJoCo')
axes[2, 1].plot(100*series('pose_est_x_m'), 100*series('pose_est_y_m'), label='Odometry', alpha=.75)
goal = 0. if sequence else 100*series('distance_ref_m')[-1]
if sequence:
    axes[2, 1].plot(np.zeros(len(t)), 100*series('planned_y_m'), '--', label='Ideal sequence')
if sequence:
    axes[2, 1].scatter([0], [0], color='green', label='Ideal start/return', zorder=5)
    axes[2, 1].scatter([100*series('pose_true_x_m')[-1]], [100*series('pose_true_y_m')[-1]],
                       color='red', label='Simulated end', zorder=5)
elif stationary:
    axes[2, 1].scatter([0], [0], color='green', label='Stationary reference', zorder=5)
else:
    axes[2, 1].scatter([0, 0], [0, goal], color=['green', 'red'], zorder=5)
    axes[2, 1].annotate('A', (0, 0), xytext=(6, 0), textcoords='offset points')
    axes[2, 1].annotate('B', (0, goal), xytext=(6, 0), textcoords='offset points')
axes[2, 1].set(xlabel='Local X [cm]', ylabel='Local Y [cm]')
axes[2, 1].set_aspect('equal', adjustable='datalim' if sequence or stationary else 'box')
for axis in axes.flat:
    axis.grid(alpha=.25)
    axis.legend(loc='best', fontsize=8)
    if axis is not axes[2, 1]:
        axis.set_xlabel('Time since first logged sample [s]')
        axis.set_xlim(left=0)
metadata_path = path.with_suffix('.json')
metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
controller = 'H∞' if metadata.get('configuration',{}).get('mode') == 'hinf' else 'LQR'
controller = metadata.get('controller_label',controller)
genes = metadata.get('lqr_design',{}).get('ga',{}).get('genes_log10',[])
if controller == 'LQR' and any(abs(g)>1e-12 for g in genes):
    controller = 'LQR GA'
fig.suptitle(controller+' two-motor: '+('stabilization' if stationary else 'motion and stabilization')
             +', encoder + IMU feedback')
if any(abs(float(rows[0].get(k,0.)))>1e-12 for k in ['initial_position_offset_m','initial_yaw_offset_deg']):
    fig.suptitle(fig._suptitle.get_text()+'\nInitial errors relative to target: '+
        f"s={100*float(rows[0].get('initial_position_offset_m',0.)):.1f} cm, "+
        f"yaw={float(rows[0].get('initial_yaw_offset_deg',0.)):.1f} deg")
fig.tight_layout()
for extension in (('pdf','svg') if args.format=='both' else (args.format,)):
    output=path.with_suffix('.'+extension)
    fig.savefig(output)
    print(output)
plt.close(fig)
