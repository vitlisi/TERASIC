#!/usr/bin/env python3
"""Plot reference/estimated/true curve, tracking errors and balance from schema 12."""
import csv
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
if not rows or 'path_ref_x_m' not in rows[0]:
    raise SystemExit('An lqr-path CSV with schema 12 is required')
def series(key):
    return np.array([float(row[key]) if row[key] not in ('', None) else np.nan for row in rows])
t = series('t_free_s' if args.free_only else 't_s')
t = t - t[0]
fig, axes = plt.subplots(3, 2, figsize=(12, 10))
for x,y,label in [('path_ref_x_m','path_ref_y_m','Reference'),
                  ('pose_est_x_m','pose_est_y_m','Encoder + IMU estimate'),
                  ('pose_true_x_m','pose_true_y_m','MuJoCo')]:
    axes[0,0].plot(100*series(x),100*series(y),label=label)
axes[0,0].scatter([0,100*series('path_ref_x_m')[-1]],[0,100*series('path_ref_y_m')[-1]],color=['green','red'])
axes[0,0].set(xlabel='Local X [cm]',ylabel='Local Y [cm]')
axes[0,0].set_aspect('equal',adjustable='box')
x=100*series('path_ref_x_m'); y=100*series('path_ref_y_m')
span=max(float(np.ptp(x))+10.,float(np.ptp(y))*.6,20.)
center=float((x.min()+x.max())/2)
axes[0,1].plot(t,1000*series('path_pose_error_m'),label='Estimated error')
axes[0,1].plot(t,1000*series('path_true_pose_error_m'),label='True error')
axes[0,1].set(ylabel='XY error [mm]')
for field,label in [('x_rho_rad','Estimate'),('yaw_true_rad','MuJoCo'),('path_nominal_yaw_rad','Path'),('yaw_ref_rad','Guidance')]:
    axes[1,0].plot(t,np.degrees(series(field)),label=label)
axes[1,0].set(ylabel='Yaw [deg]')
for field,label in [('pwm_sx_applied_percent','Left'),('pwm_dx_applied_percent','Right')]:
    axes[1,1].plot(t,series(field),label=label)
axes[1,1].set(ylabel='PWM [%]')
for field,label in [('pitch_est_deg','Estimate'),('pitch_true_deg','MuJoCo'),('pitch_target_deg','Reference')]:
    axes[2,0].plot(t,series(field),label=label)
axes[2,0].set(ylabel='Pitch [deg]')
for field,label in [('velocity_encoder_m_s','Estimate'),('velocity_ref_m_s','Reference')]:
    axes[2,1].plot(t,series(field),label=label)
axes[2,1].set(ylabel='Velocity [m/s]')
for axis in axes.flat:
    axis.legend(fontsize=8,loc='upper left',bbox_to_anchor=(1.02,1)) if axis is axes[0,0] else axis.legend(fontsize=8,loc='best')
    axis.grid(alpha=.25)
    if axis is not axes[0,0]:
        axis.set_xlabel('Time since first logged sample [s]')
        axis.set_xlim(left=0)
fig.suptitle('Curved path: estimated-pose guidance and LQR stabilization')
fig.tight_layout()
for extension in (('pdf','svg') if args.format=='both' else (args.format,)):
    output=path.with_suffix('.'+extension)
    fig.savefig(output)
    print(output)
plt.close(fig)
