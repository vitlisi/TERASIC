#!/usr/bin/env python3
"""Compare frozen native LQR and GA gains on identical releases and COM pushes."""
import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src_python'))
import numpy as np
from ga_lqr import design_hash, evaluate, load_design, rank_trials, save_json, scenarios


def load_pair(folder):
    baseline = load_design(folder/'nominal_design.json')
    candidate = load_design(folder/'best.json')
    if candidate.get('ga',{}).get('nominal_design_sha256') != design_hash(baseline):
        raise ValueError('Il candidato non appartiene al nominale selezionato')
    for key in ('Ad','Bd_duty'):
        if not np.array_equal(baseline[key],candidate[key]):
            raise ValueError('Il confronto richiede lo stesso modello nominale: '+key)
    if baseline.get('wheel_profiles') != candidate.get('wheel_profiles'):
        raise ValueError('Profili motori differenti fra baseline e candidato')
    return dict(baseline=baseline,ga=candidate)


def read_trace(path):
    with Path(path).open() as stream:
        rows = [r for r in csv.DictReader(stream) if r['phase']=='free']
    return {k:np.array([float(r[k]) for r in rows]) for k in rows[0]
            if k in ('t_free_s','pitch_true_deg','pitch_target_deg','pose_true_y_m',
                     'velocity_world_forward_m_s','yaw_true_rad','pwm_sx_applied_percent',
                     'pwm_dx_applied_percent','disturbance_force_world_y_N','pose_true_x_m',
                     'path_true_pose_error_m','path_ref_x_m','path_ref_y_m','path_nominal_yaw_rad')}


def overlay(case, results, output):
    os.environ.setdefault('MPLCONFIGDIR','/tmp/terasic-ga-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    path_case = case.get('kind') == 'path'
    fig, axes = plt.subplots(3,2,figsize=(12,10),sharex=not path_case)
    fields = [('pitch_true_deg','Pitch vero [deg]',1),('pose_true_y_m','Y vera [cm]',100),
              ('velocity_world_forward_m_s','Velocita vera [m/s]',1),('yaw_true_rad','Yaw vero [deg]',180/math.pi),
              ('pwm_sx_applied_percent','PWM SX [%]',1),('pwm_dx_applied_percent','PWM DX [%]',1)]
    target_axes = list(axes.flat)
    if path_case:
        from path_reference import PathPlan
        xy = PathPlan.from_file(case['path_file']).visual_points()
        axes[0,0].plot(100*xy[:,0],100*xy[:,1],'k--',label='Path S')
        axes[0,0].set(xlabel='X vera [cm]',ylabel='Y vera [cm]',aspect='equal')
        fields = [('path_true_pose_error_m','Errore XY vero [mm]',1000),fields[0],fields[3],fields[4],fields[5]]
        target_axes = target_axes[1:]
    for label, color in [('baseline','tab:blue'),('ga','tab:orange')]:
        trace = read_trace(results[label]['csv'])
        if path_case:
            axes[0,0].plot(100*trace['pose_true_x_m'],100*trace['pose_true_y_m'],
                           color=color,label='LQR classico' if label=='baseline' else 'LQR GA')
        for axis,(key,title,scale) in zip(target_axes,fields):
            axis.plot(trace['t_free_s'],scale*trace[key],label='LQR classico' if label=='baseline' else 'LQR GA',color=color)
            axis.set_ylabel(title)
        if label=='baseline' and not path_case:
            axes[0,0].plot(trace['t_free_s'],trace['pitch_target_deg'],'k--',label='Equilibrio')
            for axis in list(axes.flat)[1:4]:
                axis.axhline(0,color='k',linestyle='--',linewidth=.8)
    if case['force_y_n']:
        for start in np.arange(5.,case['duration_s'],5.):
            for axis in axes.flat:
                axis.axvspan(start,min(start+.5,case['duration_s']),color='gray',alpha=.18)
    for axis in axes.flat:
        axis.grid(alpha=.25)
        axis.legend(fontsize=8)
        if not path_case or axis is not axes[0,0]:
            axis.set_xlabel('Tempo dal rilascio [s]')
    fig.suptitle(('LQR classico / LQR GA: Path S, ' if path_case else 'LQR classico / LQR GA: ')
                +f'partenza {case["angle_deg"]:+g} deg, forza Y al COM {case["force_y_n"]:+g} N')
    fig.tight_layout()
    fig.savefig(output,format='svg')
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ga-run',type=Path,required=True,help='Cartella di ricerca con best.json e nominal_design.json')
    parser.add_argument('--angles',type=float,nargs='+',default=[-5.,5.])
    parser.add_argument('--forces',type=float,nargs='+',default=[0.,.2],help='Forza world Y al COM [N]')
    parser.add_argument('--skip-path',action='store_true',help='Esegue solo i casi di stabilizzazione')
    parser.add_argument('--path-file',type=Path,default=ROOT/'config/paths/curve_test.json')
    parser.add_argument('--path-duration',type=float,default=40.,help='Durata totale del caso S, non del viaggio')
    parser.add_argument('--path-angle',type=float,default=3.,help='Pitch iniziale del caso S [deg assoluti]')
    parser.add_argument('--duration',type=float,default=10.)
    parser.add_argument('--max-pwm',type=float,default=50.)
    parser.add_argument('--sensor-seed',type=int,default=3000)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'logs/mujoco/lqr_vs_ga')
    parser.add_argument('--no-plots',action='store_true')
    args = parser.parse_args(argv)
    if (not all(math.isfinite(v) and abs(v)<=5 for v in args.angles)
            or not all(math.isfinite(v) for v in args.forces)
            or not math.isfinite(args.duration) or args.duration<8
            or not math.isfinite(args.max_pwm) or not 0<args.max_pwm<=100 or args.sensor_seed<0):
        parser.error('Angoli finiti entro +/-5 gradi, forze finite, durata >=8 s, PWM (0,100], seed >=0 richiesti')
    plan = None
    if not args.skip_path:
        from path_reference import PathPlan
        try:
            plan = PathPlan.from_file(args.path_file)
        except (OSError,ValueError,KeyError) as error:
            parser.error(str(error))
        if (not math.isfinite(args.path_duration) or args.path_duration < plan.start+plan.duration+5
                or not math.isfinite(args.path_angle) or abs(args.path_angle)>5):
            parser.error('Path: durata sufficiente per viaggio + 5 s di hold e angolo entro +/-5 richiesti')
    try:
        pair = load_pair(args.ga_run.resolve())
    except (OSError,ValueError,KeyError) as error:
        parser.error(str(error))
    delay = pair['ga']['ga']['feedback_delay_samples']
    batch = args.output_dir.resolve()/('compare_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    batch.mkdir(parents=True,exist_ok=False)
    for label, design in pair.items():
        save_json(batch/(label+'_design.json'),design)
    cases = [dict(name=f'case_{i:02d}',kind='stabilization',angle_deg=angle,force_y_n=force,duration_s=args.duration,
                  sensor_seed=args.sensor_seed+i)
             for i,(angle,force) in enumerate((a,f) for a in args.angles for f in args.forces)]
    if plan is not None:
        cases.append(dict(name='path_s',kind='path',angle_deg=args.path_angle,force_y_n=0.,
                          duration_s=args.path_duration,sensor_seed=args.sensor_seed+len(cases),
                          path_file=str(args.path_file.resolve())))
        save_json(batch/'path_config.json',json.loads(args.path_file.read_text()))
    save_json(batch/'config.json',dict(ga_run=str(args.ga_run.resolve()),cases=cases,
        max_pwm=args.max_pwm,feedback_delay_samples=delay,physical_model='nominal, unchanged',
        force=dict(axis='world Y',point='chassis COM',start_s=5.,duration_s=.5,period_s=5.),
        angle_convention='absolute MuJoCo pitch; supported calibration at zero, reposition then release',
        purpose='fixed-gain evaluation only; no evolution or synthesis'))
    results = []
    print('Log confronto: '+str(batch),flush=True)
    def persist():
        save_json(batch/'results.json',results)
        if results:
            with (batch/'results.csv').open('w',newline='') as stream:
                keys = list(dict.fromkeys(k for row in results for k in row))
                writer = csv.DictWriter(stream,fieldnames=keys)
                writer.writeheader()
                writer.writerows(results)
    try:
        for case in cases:
            protocol = dict(duration_s=case['duration_s'],max_pwm=args.max_pwm,release_pitch_deg=case['angle_deg'],
                feedback_delay_samples=delay,disturbance_force_y_n=case['force_y_n'],
                disturbance_start_s=5.,disturbance_duration_s=.5,disturbance_period_s=5.)
            if case['kind']=='path':
                protocol.update(mode='lqr-path',path_file=case['path_file'])
            scenario = scenarios(1,case['sensor_seed'],case['name'])
            paired = {}
            for label,design in pair.items():
                _,trials = evaluate(design,scenario,protocol,batch/case['name']/label)
                row = dict(controller=label,case=case['name'],kind=case['kind'],requested_angle_deg=case['angle_deg'],
                           force_y_n=case['force_y_n'],sensor_seed=case['sensor_seed'],**trials[0])
                results.append(row)
                paired[label] = row
                persist()
                print(f'{case["name"]}: {label}, partenza {case["angle_deg"]:+g} deg, '
                      f'forza {case["force_y_n"]:+g} N: completato={not row["failed"]}, '
                      f'costo={row["cost"]:.4f}',flush=True)
            if not args.no_plots:
                environment = dict(os.environ,MPLCONFIGDIR='/tmp/terasic-ga-matplotlib')
                for row in paired.values():
                    plot = 'plot_path.py' if case['kind']=='path' else 'plot_motion.py'
                    subprocess.run([sys.executable,str(ROOT/'scripts'/plot),row['csv'],'--format','svg'],
                                   check=True,env=environment,stdout=subprocess.DEVNULL)
                overlay(case,paired,batch/case['name']/'comparison.svg')
        summary = dict(status='complete',cases=len(cases),controllers={
            label:dict(stabilized=sum(bool(r.get('stabilized')) for r in results if r['controller']==label and r['kind']=='stabilization'),
                       survived=sum(bool(r.get('survived')) for r in results if r['controller']==label and r['kind']=='stabilization'),
                       rank=rank_trials([r for r in results if r['controller']==label and r['kind']=='stabilization'])) for label in pair},
            stabilization_cases=sum(c['kind']=='stabilization' for c in cases),
            path_results=[r for r in results if r['kind']=='path'],
            note='Same frozen K in every case. Recovery time is the first stable one-second window, not post-push recovery.')
        save_json(batch/'summary.json',summary)
        print(json.dumps(summary,indent=2),flush=True)
    except BaseException as error:
        save_json(batch/'error.json',dict(type=type(error).__name__,message=str(error)))
        raise
    return 0


if __name__=='__main__':
    raise SystemExit(main())
