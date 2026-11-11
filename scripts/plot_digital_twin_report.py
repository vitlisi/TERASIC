#!/usr/bin/env python3
"""Reproduce historical digital-twin SVGs offline; never acquires or fits data."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'diagnostics'))
os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-report-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from analyze_static_point import analyze as static_analyze
from analyze_compensated_freewheel import analyze as compensated_analyze
from compare_freewheel import read_csv, write_csv
from replay_stateful_motor import replay, filtered_speed

plt.rcParams.update({'svg.fonttype':'none', 'font.size':10})
VOLTAGE_FLAG = 'voltage_outside_frozen_comparability_band'


def regular(analysis):
    return not [r for r in analysis['rejection_reasons'] if r != VOLTAGE_FLAG]


def save(fig, output, name, title, caption):
    fig.suptitle(title, fontsize=15)
    fig.text(.5,.02,caption,ha='center',fontsize=9)
    fig.tight_layout(rect=(.025,.085,.995,.91))
    fig.savefig(output/(name+'.svg'),bbox_inches='tight',pad_inches=.15)
    fig.savefig(output/(name+'.png'),dpi=120,bbox_inches='tight',pad_inches=.15)
    plt.close(fig)
    print('SVG: '+str(output/(name+'.svg')),flush=True)


def pulse_figures(inputs, manifest, profile, base, output, analysis):
    metrics = {}
    for variant, wheels, state in [('nominal',None,None),('stribeck',base,profile)]:
        fig,axes=plt.subplots(3,4,figsize=(18,10),sharex='col')
        metrics[variant]={}
        for col,name in enumerate(sorted(manifest['pulse_records'], key=lambda n:('rep2' in n, '_dx_' in n))):
            folder=inputs/name; raw=read_csv(folder/'reale.csv')
            status=(folder/'reale.status.log').read_text()
            if len(raw)!=2601 or 'result=COMPLETED' not in status or 'final_control_sx=6 final_control_dx=6' not in status:
                raise ValueError('Incomplete historical pulse record: '+name)
            wheel='sx' if '_sx_' in name else 'dx'
            simulated,fixture=replay(raw,1325,wheels,state)
            write_csv(analysis/(folder.name+'_'+variant+'.csv'),simulated)
            t,real=filtered_speed(raw,wheel);_,speed=filtered_speed(simulated,wheel,True)
            time=t-t[0];error=speed-real
            axes[0,col].plot(time,[r['pwm_'+wheel+'_applied_percent'] for r in raw],color='.3',lw=1)
            axes[0,col].set_title(('Left wheel' if wheel=='sx' else 'Right wheel')+' — repetition '+('2' if 'rep2' in name else '1'))
            axes[1,col].plot(time,real,color='#d32f2f',lw=1,label='Real robot')
            axes[1,col].plot(time,speed,color='#1768b5',ls='--',lw=1,label='MuJoCo, '+('equal nominal motors' if variant=='nominal' else 'calibrated Stribeck'))
            axes[1,col].legend(fontsize=8)
            axes[2,col].plot(time,error,color='#713d9b',lw=.8);axes[2,col].axhline(0,color='black',lw=.6)
            for ax in axes[:,col]:ax.set_xlim(0,time[-1]);ax.grid(alpha=.2);ax.set_xlabel('Time since acquisition start [s]')
            metrics[variant][folder.name]=dict(rmse_rad_s=float(np.sqrt(np.mean(error**2))),max_abs_error_rad_s=float(max(abs(error))),fixture=fixture)
        for ax,label in zip(axes[:,0],['Applied PWM [%]','Wheel speed [rad/s]','MuJoCo - real speed [rad/s]']):ax.set_ylabel(label)
        save(fig,output,'01_nominal_motor_pulses' if variant=='nominal' else '02_stribeck_motor_pulses',
             'Raised-wheel PWM pulses — real robot versus '+('equal nominal motors' if variant=='nominal' else 'calibrated MuJoCo with Stribeck'),
             'Complete historical acquisitions; compensation OFF. Fixed chassis, no ground contact; same recorded physical PWM, voltage and timestamps.\nQuantized encoders and the same 50 ms speed filter. Nominal = original XML hypothesis; current = frozen separate wheel/direction parameters. No refitting.')
    (analysis/'pulse_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')


def static_points(inputs,manifest,profile,base,analysis):
    fingerprint={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [
        ROOT/'models/verifica.xml',ROOT/'config/mujoco_robot01_stribeck.json',ROOT/'config/mujoco_motors_sx_dx_20261030.json',
        ROOT/'src_python/stribeck_motor.py',ROOT/'src_python/wheel_dynamics.py',ROOT/'diagnostics/replay_stateful_motor.py',
        ROOT/'diagnostics/compare_freewheel.py',ROOT/'diagnostics/analyze_static_point.py']}
    fingerprint['static_aggregation_version']=1
    previous={}
    cache=analysis/'static_model_points.json'
    if cache.exists():
        old=json.loads(cache.read_text())
        if old['fingerprint']==fingerprint:previous={r['key']:r for r in old['points']}
    result=[]
    for i,record in enumerate(manifest['static_records'],1):
        folder=inputs/record['folder'];raw=read_csv(folder/'reale.csv')
        signature=hashlib.sha256(b''.join(hashlib.sha256((folder/n).read_bytes()).digest() for n in ['reale.csv','point.txt','reale.config.txt','reale.status.log'])).hexdigest()
        cached=previous.get(record['key'])
        if cached and cached.get('input_sha256')==signature:
            result.append(cached)
            continue
        actual=static_analyze(folder)
        if abs(actual['mean_velocity_rad_s']-record['analysis']['mean_velocity_rad_s'])>1e-10:
            raise ValueError('Historical mean cannot be reproduced: '+record['key'])
        item=dict(key=record['key'],metadata=record['metadata'],real=actual,input_sha256=signature)
        for variant,wheels,state in [('nominal',None,None),('stribeck',base,profile)]:
            prediction,fixture=replay(raw,1325,wheels,state)
            synthetic=[dict(r,**{'enc_'+w+'_raw':32768+s['enc_'+w+'_counts'] for w in ('sx','dx')}) for r,s in zip(raw,prediction)]
            item[variant]=static_analyze(folder,rows=synthetic)
        result.append(item)
        if i%16==0:print('Static replay '+str(i)+'/328',flush=True)
    cache.write_text(json.dumps(dict(fingerprint=fingerprint,points=result),indent=2)+'\n')
    return result


def hysteresis_figures(points,output):
    for variant in [None,'nominal','stribeck']:
        for zoom in ([False] if variant is None else [False,True]):
            fig,axes=plt.subplots(1,2,figsize=(15,6),sharey=True)
            for col,wheel in enumerate(('sx','dx')):
                ax=axes[col]
                for mode,style,label in [('cold','-','From rest'),('moving','--','After pre-run')]:
                    for direction,sign in [('reverse',-1),('forward',1)]:
                        group=sorted([r for r in points if r['metadata']['wheel']==wheel and r['metadata']['mode']==mode and r['metadata']['direction']==direction],key=lambda r:sign*float(r['metadata']['pwm_magnitude_percent']))
                        x=np.array([sign*float(r['metadata']['pwm_magnitude_percent']) for r in group])
                        sources=[('real','#c62828','Real robot')]+([(variant,'#1768b5','MuJoCo')] if variant else [])
                        for source,color,title in sources:
                            y=np.array([r[source]['mean_velocity_rad_s'] for r in group])
                            ax.plot(x,y,style,color=color,marker='.' if source=='real' else None,ms=4,lw=1.2,label=title+' — '+label if direction=='forward' else None)
                            bad=[j for j,r in enumerate(group) if not regular(r[source])]
                            ax.scatter(x[bad],y[bad],color=color,marker='x',s=22,zorder=4)
                        flagged=[j for j,r in enumerate(group) if VOLTAGE_FLAG in r['real']['rejection_reasons']]
                        ax.scatter(x[flagged],[group[j]['real']['mean_velocity_rad_s'] for j in flagged],color='.5',marker='x',s=22,zorder=5)
                ax.set(title='Left wheel (SX)' if wheel=='sx' else 'Right wheel (DX)',xlabel='Applied PWM [%]')
                ax.axhline(0,color='.5',lw=.6);ax.axvline(0,color='.5',lw=.6);ax.grid(alpha=.2)
                ax.set_xlim(-7 if zoom else -20,7 if zoom else 20)
                if zoom:ax.set_ylim(-2,2)
            axes[0].set_ylabel('Steady-state wheel speed [rad/s]');axes[0].legend(fontsize=8)
            name='03_measured_hysteresis' if variant is None else '04_nominal_hysteresis' if variant=='nominal' else '05_stribeck_hysteresis'
            title='Measured raised-wheel characteristics' if variant is None else 'Raised-wheel real/MuJoCo characteristics — '+('equal nominal motors' if variant=='nominal' else 'calibrated Stribeck motors')
            save(fig,output,name+('_zoom' if zoom else '_full'),title+'\n0.5% PWM steps'+(' — threshold zoom' if zoom else ''),
                'Both signs and start conditions. Each point is an isolated acquisition; pre-run: recorded +/-20% for 2 s. Same 50 ms encoder filter and steady windows.\nCrosses: non-regular motion; gray: recorded voltage outside the historical comparability band. Stribeck parameters were calibrated on this grid; this is not independent validation.')


def compensated_figures(inputs,manifest,output,analysis):
    source=inputs/manifest['compensated_run'];trial=analysis/'compensated_trial'
    if not trial.exists():
        trial.mkdir();shutil.copytree(source/'raw',trial/'raw');shutil.copy2(source/'state.json',trial/'state.json')
    report=compensated_analyze(trial,'run')
    (trial/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['accepted']:raise ValueError('Compensated acquisition rejected: '+str(report['flags']))
    comparison=trial/'comparison'
    if not comparison.exists():subprocess.run([sys.executable,'-B',str(ROOT/'diagnostics/plot_compensated_freewheel.py'),str(trial)],check=True)
    shutil.copy2(comparison/'real_vs_mujoco_compensated.svg',output/'06_compensated_pulses_full.svg')
    shutil.copy2(comparison/'real_vs_mujoco_compensated.png',output/'06_compensated_pulses_full.png')
    real=read_csv(trial/'raw/reale.csv');model=[{k: (1. if v=='True' else 0. if v=='False' else float(v)) for k,v in row.items()} for row in csv.DictReader((comparison/'mujoco.csv').open())]
    fig,axes=plt.subplots(2,4,figsize=(17,8),sharex='col')
    for col,(wheel,window,title) in enumerate([('sx',(.5,3.5),'Left, -4%'),('dx',(.5,3.5),'Right, -4%'),('sx',(12.5,15.5),'Left, +4%'),('dx',(12.5,15.5),'Right, +4%')]):
        t=np.array([r['t_s']-real[0]['t_s'] for r in real]);rv=np.array([r['filtered_speed_'+wheel+'_rad_s'] for r in real]);sv=np.array([r['filtered_speed_'+wheel+'_rad_s'] for r in model])
        mask=(t>=window[0])&(t<=window[1])
        axes[0,col].plot(t[mask],rv[mask],color='#d32f2f',lw=1,label='Real robot');axes[0,col].plot(t[mask],sv[mask],'--',color='#198038',lw=1,label='MuJoCo with Stribeck')
        axes[0,col].set_title(title);axes[0,col].legend(fontsize=8)
        axes[1,col].plot(t[mask],(sv-rv)[mask],color='#713d9b',lw=.8);axes[1,col].axhline(0,color='black',lw=.6)
        for ax in axes[:,col]:ax.set_xlim(*window);ax.grid(alpha=.2);ax.set_xlabel('Time since acquisition start [s]')
    axes[0,0].set_ylabel('Wheel speed [rad/s]');axes[1,0].set_ylabel('MuJoCo - real speed [rad/s]')
    save(fig,output,'06_compensated_pulses_zoom','Compensated raised-wheel pulses — low-command transition zooms',
         'Additional detail only; the complete 26 s acquisition is retained separately. Exact bounded C++ Stribeck compensator on both platforms.\nRecorded voltage/timestamps, quantized PWM/encoders and the same 50 ms filter; each controller uses its own encoder feedback.')
    print('SVG: '+str(output/'06_compensated_pulses_full.svg'),flush=True)


def green_figures(inputs,manifest,points,profile,base,output,analysis):
    green=[]
    for name in manifest['green_records']:
        folder=inputs/name;raw=read_csv(folder/'reale.csv');status=(folder/'reale.status.log').read_text()
        if len(raw)!=1001 or 'result=COMPLETED' not in status:raise ValueError('Incomplete LUT point: '+name)
        wheel=re.search(r'_(sx|dx)_',folder.name)[1];direction='forward' if '_forward_' in folder.name else 'reverse'
        t,speed=filtered_speed(raw,wheel);mask=(t>=6.48)&(t<6.98)
        if mask.sum()<45:raise ValueError('Missing LUT steady-state window')
        prediction,_=replay(raw,1325,base,profile);_,sim=filtered_speed(prediction,wheel,True)
        plateau=next(r for r in raw if r['stage']==2)
        green.append(dict(point=folder.name,wheel=wheel,direction=direction,repeat=2 if 'rep2' in folder.name else 1,
            command=abs(plateau['lut_virtual_command']),target=abs(plateau['target_velocity_rad_s']),
            real=abs(float(np.mean(speed[mask]))),mujoco=abs(float(np.mean(sim[mask]))),
            voltage=float(np.mean([r['battery_v'] for r,m in zip(raw,mask) if m]))))
    write_csv(analysis/'moving_lut_points.csv',green)
    for simulation in [False,True]:
        fig,axes=plt.subplots(2,2,figsize=(14,9),sharex=True,sharey=True)
        for index,(wheel,direction) in enumerate([('sx','forward'),('sx','reverse'),('dx','forward'),('dx','reverse')]):
            ax=axes.flat[index]
            red=sorted([r for r in points if r['metadata']['wheel']==wheel and r['metadata']['direction']==direction and r['metadata']['mode']=='moving'],key=lambda r:float(r['metadata']['pwm_magnitude_percent']))
            ax.plot([float(r['metadata']['pwm_magnitude_percent']) for r in red],[abs(r['real']['mean_velocity_rad_s']) for r in red],'.-',color='#b52c2c',lw=1,label='Real, direct physical PWM')
            for repeat,marker in [(1,'o'),(2,'x')]:
                data=sorted([r for r in green if r['wheel']==wheel and r['direction']==direction and r['repeat']==repeat],key=lambda r:r['command'])
                ax.plot([r['command'] for r in data],[r['real'] for r in data],marker+'-',color='#198038',fillstyle='none',ms=5,lw=.9,label='Real static LUT, repeat '+str(repeat))
                if simulation:ax.plot([r['command'] for r in data],[r['mujoco'] for r in data],'--',color='#1768b5',lw=1,label='MuJoCo, recorded LUT output PWM' if repeat==1 else None)
                if repeat==1:
                    slope=data[0]['target']/data[0]['command'];ax.plot([0,20],[0,20*slope],'--',color='.25',lw=.9,label='Frozen linear target')
            ax.axvspan(0,2.85,color='.8',alpha=.3);ax.set_xlim(0,20);ax.set_ylim(bottom=0)
            ax.set(title=('Left' if wheel=='sx' else 'Right')+' wheel — '+direction,xlabel='Command [%]: physical PWM (red), virtual LUT input (green)',ylabel='Wheel speed magnitude [rad/s]');ax.grid(alpha=.2);ax.legend(fontsize=7)
        save(fig,output,'08_static_lut_real_vs_mujoco' if simulation else '07_static_lut_measured_linearization',
             'Raised-wheel static speed LUT — '+('real measurements and calibrated MuJoCo replay' if simulation else 'independent moving-state measurements'),
             'Historical full static inverse LUT; distinct from the bounded +/-0.5 PWM Stribeck compensator. Red: +/-20% pre-run for 2 s; green: +/-8% for 3 s.\nRecorded variable voltage is retained; no normalization or refitting. Gray: unvalidated low-input region. Moving-state data do not establish autonomous start or loaded control.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs',type=Path,default=ROOT/'logs/real/digital_twin/historical_inputs')
    p.add_argument('--output-dir',type=Path,default=ROOT/'logs/real/digital_twin/plot')
    p.add_argument('--analysis-dir',type=Path,default=ROOT/'logs/mujoco/digital_twin/historical_analysis')
    a=p.parse_args();manifest=json.loads((a.inputs/'inputs_manifest.json').read_text())
    for item in manifest['files']:
        if hashlib.sha256((a.inputs/item['path']).read_bytes()).hexdigest()!=item['sha256']:raise ValueError('Historical input changed: '+item['path'])
    a.output_dir.mkdir(parents=True,exist_ok=True);a.analysis_dir.mkdir(parents=True,exist_ok=True)
    profile=json.loads((ROOT/'config/mujoco_robot01_stribeck.json').read_text());base=json.loads((ROOT/'config/mujoco_motors_sx_dx_20261030.json').read_text())['wheels']
    pulse_figures(a.inputs,manifest,profile,base,a.output_dir,a.analysis_dir)
    compensated_figures(a.inputs,manifest,a.output_dir,a.analysis_dir)
    points=static_points(a.inputs,manifest,profile,base,a.analysis_dir)
    thresholds=[]
    for wheel in ('sx','dx'):
        for direction in ('forward','reverse'):
            for mode in ('cold','moving'):
                group=[r for r in points if r['metadata']['wheel']==wheel and r['metadata']['direction']==direction and r['metadata']['mode']==mode and float(r['metadata']['pwm_magnitude_percent'])>0]
                item=dict(wheel=wheel,direction=direction,start_condition=mode)
                for source in ('real','nominal','stribeck'):
                    candidates=[r for r in group if regular(r[source])]
                    first=min(candidates,key=lambda r:float(r['metadata']['pwm_magnitude_percent'])) if candidates else None
                    item[source]=dict(first_regular_pwm_percent=float(first['metadata']['pwm_magnitude_percent']) if first else None,speed_at_first_regular_rad_s=abs(first[source]['mean_velocity_rad_s']) if first else None)
                thresholds.append(item)
    (a.analysis_dir/'threshold_summary.json').write_text(json.dumps(thresholds,indent=2)+'\n')
    hysteresis_figures(points,a.output_dir)
    green_figures(a.inputs,manifest,points,profile,base,a.output_dir,a.analysis_dir)
    (a.output_dir/'README.md').write_text('# Historical digital-twin SVGs\n\nReproduce with `env/bin/python -B scripts/plot_digital_twin_report.py`. No new acquisitions, fitting or hardware calls. Source records and SHA256 mapping: `logs/real/digital_twin/historical_inputs/inputs_manifest.json`. Full figures and separate zooms are retained. The static Stribeck grid is calibration data; the pulse records are separate acquisitions. The historical static inverse LUT and the current bounded Stribeck compensator are distinct experiments. No complete compensated cold/pre-run static hysteresis grid was acquired, so it is not fabricated from the moving-only LUT points.\n')
    print('Complete: '+str(a.output_dir),flush=True)


if __name__=='__main__':main()
