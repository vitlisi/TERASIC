#!/usr/bin/env python3
"""Plot baseline and GA from existing validation logs; never runs a simulation."""
import argparse
import csv
import json
import os
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ga-run', type=Path, required=True)
    parser.add_argument('--scenario', default='validation_00')
    parser.add_argument('--results-file', default='validation.json', help='Local results filename; replay_results.json for replay')
    parser.add_argument('--output-dir', type=Path, help='Report figure directory; default: GA run/confronto_performance')
    args = parser.parse_args()
    folder = args.ga_run.resolve()
    if Path(args.results_file).name != args.results_file:
        raise ValueError('Specify only the results basename')
    validation = json.loads((folder/args.results_file).read_text())
    config = json.loads((folder/'config.json').read_text())
    traces, trials, metadata = {}, {}, {}
    for name in ('baseline', 'ga'):
        matches = [t for t in validation[name]['trials'] if t['scenario'] == args.scenario]
        if len(matches) != 1:
            raise ValueError('Missing or ambiguous scenario: '+args.scenario)
        trials[name] = matches[0]
        candidates = list((folder/config.get('trace_directory','validation')/name/args.scenario).glob('*.csv'))
        if len(candidates) != 1:
            raise ValueError('Missing or ambiguous log for '+name)
        path = candidates[0]
        metadata[name] = json.loads(path.with_suffix('.json').read_text())
        with path.open() as stream:
            rows = [r for r in csv.DictReader(stream) if r['phase'] == 'free']
        a = lambda key: np.array([float(r[key]) for r in rows])
        traces[name] = dict(time=a('t_free_s'),
            pitch=a('pitch_true_deg')-a('pitch_target_deg'),
            position=1000*np.hypot(a('pose_true_x_m'),a('pose_true_y_m')),
            speed=np.hypot(a('velocity_world_forward_m_s'),a('velocity_world_lateral_m_s')),
            sx=a('pwm_sx_applied_percent'), dx=a('pwm_dx_applied_percent'))
    common = min(len(traces['baseline']['time']), len(traces['ga']['time']))
    if not common or not np.allclose(traces['baseline']['time'][:common], traces['ga']['time'][:common]):
        raise ValueError('Time grids differ on common support')
    for key in ('physical_parameters', 'ga_scenario', 'motor_friction_profile'):
        if metadata['baseline'].get(key) != metadata['ga'].get(key):
            raise ValueError('Conditions differ: '+key)
    if metadata['baseline']['configuration']['sensor_seed'] != metadata['ga']['configuration']['sensor_seed']:
        raise ValueError('Sensor seeds differ')

    os.environ.setdefault('MPLCONFIGDIR','/tmp/terasic-ga-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3,2,figsize=(15,11))
    fig.subplots_adjust(top=.86,bottom=.09,hspace=.43,wspace=.24)
    replay = config.get('replay_only',False)
    title = ('MuJoCo — baseline LQR and provisional GA-LQR' if replay
             else 'MuJoCo — baseline LQR versus GA-tuned LQR (Q and R)')
    fig.suptitle(title,fontsize=17,y=.985)
    robust = [validation[k]['rank'][2] for k in ('baseline','ga')]
    nominal = [trials[k]['cost'] for k in ('baseline','ga')]
    subtitle = f'{args.scenario} | single-trial cost: {nominal[0]:.4f} → {nominal[1]:.4f}'
    subtitle += (' | replay, no new optimization or final validation' if replay else
                 f' | all-scenario cost: {robust[0]:.4f} → {robust[1]:.4f}')
    fig.text(.5,.95,subtitle,ha='center',fontsize=11)
    duration=config['protocol']['duration_s']; cap=config['protocol']['max_pwm']
    fields=[('pitch','Pitch error [°]','Tilt relative to equilibrium'),
            ('pitch','Pitch error [°]','Final 2 s detail'),
            ('position','Planar distance [mm]','Displacement from initial position'),
            ('speed','Planar speed [m/s]','Ground-speed magnitude'),
            ('sx','PWM SX [%]','Applied command — SX wheel'),
            ('dx','PWM DX [%]','Applied command — DX wheel')]
    for index,(ax,(key,ylabel,title)) in enumerate(zip(axes.flat,fields)):
        for name,color,label in [('baseline','tab:orange','Baseline LQR'),
                                 ('ga','tab:blue','GA-LQR (provisional)' if replay else 'GA-tuned LQR')]:
            trace=traces[name]; mask=trace['time']>=duration-2 if index==1 else np.ones(len(trace['time']),dtype=bool)
            ax.plot(trace['time'][mask],trace[key][mask],color=color,label=label,lw=1.1,alpha=.9)
        if key=='pitch':
            limits=[-.3,.3]
        elif key=='position': limits=[10]
        elif key=='speed': limits=[.02]
        else: limits=[-cap,cap]
        for limit in limits: ax.axhline(limit,color='gray',ls='--',lw=1)
        if index!=1: ax.axvspan(duration-2,duration,color='gray',alpha=.09)
        if key in ('sx','dx'): ax.set_ylim(-cap*1.1,cap*1.1)
        ax.set(title=title,xlabel='Time since control start [s]',ylabel=ylabel)
        ax.set_xlim((duration-2, duration) if index==1 else (0, duration))
        ax.grid(alpha=.2)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.932),ncol=2,frameon=False)
    outcomes = '; '.join(f'{name}: ' + ('duration completed' if trials[name].get('survived') else 'stopped early')
                         + (', settling criteria met' if trials[name].get('stabilized') else ', settling criteria not met')
                         for name in ('baseline','ga'))
    fig.text(.5,.025,'Same SX/DX and Stribeck model, sensor seed and protocol. '
             'Gray: final assessment. Dashed: thresholds or PWM limits.',ha='center',fontsize=9)
    fig.text(.5,.008,outcomes,ha='center',fontsize=9)
    output=args.output_dir.resolve() if args.output_dir else folder/'confronto_performance'
    output.mkdir(parents=True,exist_ok=True)
    for extension in ('svg','pdf','png'):
        fig.savefig(output/('confronto_'+args.scenario+'.'+extension),dpi=160)
    plt.close(fig)
    (output/'metriche.json').write_text(json.dumps(dict(scenario=args.scenario,
        baseline=trials['baseline'],ga=trials['ga'],**{('replay_rank' if replay else 'robust_validation'):
            {k:validation[k]['rank'] for k in ('baseline','ga')}},
        source_run=str(folder),protocol=config['protocol']),indent=2)+'\n')
    print(output/('confronto_'+args.scenario+'.svg'))


if __name__=='__main__':
    main()
