"""Matched real/MuJoCo compensated raised-wheel overlay, full traces from t=0."""
import argparse,csv,json,subprocess
from pathlib import Path
from simulate_compensated_freewheel import ROOT,load_compensator,simulate,write_csv
import numpy as np
import matplotlib.pyplot as plt

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);a=p.parse_args()
    report=json.loads((a.trial/'analysis.json').read_text());state=json.loads((a.trial/'state.json').read_text())
    if not report['accepted'] or state['mode']!='run':raise RuntimeError('Verified complete real run required')
    real=[{k:float(v) for k,v in r.items()} for r in csv.DictReader((a.trial/'raw/reale.csv').open())]
    out=a.trial/'comparison';out.mkdir(exist_ok=False)
    library=out/'compensator.so';subprocess.run(['g++','-std=c++98','-O2','-shared','-fPIC',str(ROOT/'diagnostics/stribeck_compensation_bridge.cpp'),'-o',str(library)],check=True)
    profile=json.loads((ROOT/'config/mujoco_robot01_stribeck.json').read_text())
    model,details=simulate(real,profile,load_compensator(library.resolve()),True)
    write_csv(out/'mujoco.csv',model);(out/'profile_frozen.json').write_text(json.dumps(profile,indent=2)+'\n')
    fig,axes=plt.subplots(3,2,figsize=(15,10),sharex=True);t=np.array([r['t_s'] for r in real]);metrics={}
    for j,w in enumerate(('sx','dx')):
        real_speed=np.array([r[f'filtered_speed_{w}_rad_s'] for r in real]);sim_speed=np.array([r[f'filtered_speed_{w}_rad_s'] for r in model]);error=sim_speed-real_speed
        axes[0,j].plot(t,[r[f'nominal_{w}_percent'] for r in real],color='.35',ls=':',lw=1,label='Nominal PWM')
        axes[0,j].plot(t,[r[f'pwm_{w}_applied_percent'] for r in real],color='#e53935',lw=1,label='Real applied PWM')
        axes[0,j].plot(t,[r[f'applied_{w}_percent'] for r in model],color='#198038',ls='--',lw=1,label='MuJoCo applied PWM')
        axes[1,j].plot(t,real_speed,color='#e53935',lw=1,label='Real robot, compensation ON')
        axes[1,j].plot(t,sim_speed,color='#198038',ls='--',lw=1,label='MuJoCo with Stribeck, compensation ON')
        axes[2,j].plot(t,error,color='#713d9b',lw=.8);axes[2,j].axhline(0,color='black',lw=.6)
        axes[0,j].set_title('Left wheel (SX)' if w=='sx' else 'Right wheel (DX)')
        axes[0,j].legend(fontsize=8);axes[1,j].legend(fontsize=8)
        for ax in axes[:,j]:ax.set_xlim(0,t[-1]);ax.grid(alpha=.2);ax.set_xlabel('Time since acquisition start [s]')
        metrics[w]=dict(speed_rmse_rad_s=float(np.sqrt(np.mean(error**2))),speed_mae_rad_s=float(np.mean(abs(error))),speed_max_absolute_error_rad_s=float(max(abs(error))))
    for ax,label in zip(axes[:,0],['PWM [%]','Wheel speed [rad/s]','MuJoCo - real speed [rad/s]']):ax.set_ylabel(label)
    fig.suptitle('Robot01 — compensated raised-wheel pulse response and error\nReal robot versus MuJoCo with Stribeck; complete acquisition from t=0',fontsize=15)
    fig.text(.5,.025,'Fixed chassis, no ground contact; identical nominal commands, recorded voltage and timestamps.\nSame C++ compensator, PWM quantization, quantized encoders and 50 ms speed filter; each controller uses its own encoder feedback.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.08,1,.93))
    for ext in ('png','svg'):fig.savefig(out/('real_vs_mujoco_compensated.'+ext),dpi=160)
    plt.close(fig);(out/'summary.json').write_text(json.dumps(dict(fixture=details,metrics=metrics,real_analysis=report),indent=2)+'\n')
    print(out/'real_vs_mujoco_compensated.png')
if __name__=='__main__':main()
