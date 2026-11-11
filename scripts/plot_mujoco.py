#!/usr/bin/env python3
"""Plot a recorded PD simulation: python3 scripts/plot_mujoco.py PATH.csv."""
import csv
import json
from pathlib import Path
import argparse
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-report-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('csv',type=Path)
parser.add_argument('--format',choices=('pdf','svg','both'),default='both')
parser.add_argument('--free-only', action='store_true', help='Explicit optional free-motion detail; default keeps the full acquisition')
args=parser.parse_args()
p=args.csv.resolve()
with p.open() as f:
    rows=list(csv.DictReader(f))
if args.free_only:
    rows=[r for r in rows if r['phase']=='free']
if not rows:raise SystemExit('The log contains no free-motion phase')
def series(name):return [float(r[name]) for r in rows]
t=series('t_free_s' if args.free_only else 't_s')
t=[value-t[0] for value in t]
fig,axes=plt.subplots(4,1,figsize=(10,9),sharex=True)
for field,label in [('pitch_true_deg','Geometric truth'),('pitch_est_deg','Estimate'),('pitch_target_deg','Reference')]:
    axes[0].plot(t,series(field),label=label)
axes[0].set_ylabel('Pitch [°]');axes[0].legend(loc='upper right')
axes[1].plot(t,series('pwm_applied_percent'),color='tab:orange');axes[1].set_ylabel('PWM [%]')
axes[2].plot(t,series('velocity_encoder_m_s'),label='Encoder')
axes[2].plot(t,series('velocity_world_y_m_s'),label='MuJoCo, diagnostic truth',alpha=.65)
axes[2].set_ylabel('Velocity [m/s]');axes[2].legend(loc='upper right')
if 'disturbance_pitch_torque_Nm' in rows[0]:
    axes[3].step(t,series('disturbance_pitch_torque_Nm'),where='post',label='External chassis torque')
    axes[3].set(ylabel='Disturbance [N m]',xlabel='Time since first logged sample [s]')
else:
    axes[3].plot(t,series('forward_y_m'),label='World position')
    axes[3].plot(t,series('position_encoder_m'),label='Odometry',alpha=.65)
    axes[3].set(ylabel='Position [m]',xlabel='Time since first logged sample [s]')
axes[3].legend(loc='upper right')
for a in axes:
    a.grid(alpha=.25)
    a.set_xlim(left=0)
metadata=json.loads(p.with_suffix('.json').read_text()) if p.with_suffix('.json').exists() else {}
mode=metadata.get('configuration',{}).get('mode','pd')
controller=metadata.get('controller',{})
pd_label=('PD pitch (Kx=Kv=0)' if controller.get('kx')==0 and controller.get('kv')==0
          else 'PD pitch + encoder feedback')
fig.suptitle('Controller MuJoCo - '+('LQR' if mode=='lqr' else pd_label))
fig.tight_layout()
for extension in (('pdf','svg') if args.format=='both' else (args.format,)):
    out=p.with_suffix('.'+extension)
    fig.savefig(out)
    print(out)
plt.close(fig)
