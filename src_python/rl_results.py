"""Rebuild the cumulative RL experiment register from immutable run folders."""
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from ga_lqr import save_json
from mujoco_robot import ROOT

LOG_ROOT=ROOT/'logs/mujoco/rl'


def config_signature(config):
    # Seed, budget and provenance vary between continuation segments.
    keys=('duration_s','dt_s','max_pwm','feedback_delay_samples','pitch_limit_deg',
          'initial_pitch_range_deg','randomize_model','observation_scales','reward')
    value={k:config[k] for k in keys}
    for key in ['initial_position_offset_range_m','initial_yaw_offset_range_deg','reset_offset_probability']:
        if key in config:value[key]=config[key]
    if 'finetune_initial_std' in config:value['finetune_initial_std']=config['finetune_initial_std']
    value['ppo']={k:v for k,v in config['ppo'].items() if k!='total_timesteps'}
    value['sources']={k:v for k,v in config.get('source_sha256',{}).items()
                      if not k.startswith('scripts/') and k!='src_python/rl_results.py'}
    value['versions']=config.get('versions',{})
    return hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()[:12]


def write_csv(path, rows):
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    temporary=path.with_suffix('.tmp')
    with temporary.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)
    temporary.replace(path)


def collect(log_root=LOG_ROOT):
    log_root=Path(log_root)
    training=[];evaluations=[];warnings=[]
    for path in sorted((log_root/'training').glob('ppo_*')):
        status_path=path/'training_status.json'
        if not status_path.exists():continue
        try:
            status=json.loads(status_path.read_text());config=json.loads((path/'config.json').read_text())
            episodes=[]
            if (path/'monitor.csv').exists():
                with (path/'monitor.csv').open() as f:episodes=list(csv.DictReader(l for l in f if not l.startswith('#')))
            wall=status.get('training_wall_s')
            if wall is None and (path/'progress.csv').exists():
                with (path/'progress.csv').open() as f:progress=list(csv.DictReader(f))
                if progress:wall=float(progress[-1]['time/time_elapsed'])
            start=status.get('start_timesteps',0);end=status['actual_timesteps']
            last=episodes[-100:]
            training.append(dict(run=str(path.resolve()),name=path.name,signature=config_signature(config),
                parent=status.get('parent_run'),status=status['status'],seed=config['seed'],
                phase=status.get('phase','ppo'),actor_source_steps=config.get('actor_transfer',{}).get('source_steps'),
                actor_source_run=config.get('actor_transfer',{}).get('source_run'),critic_samples=status.get('critic_samples',0),
                start_steps=start,total_steps=end,additional_steps=end-start,training_wall_s=wall,
                episodes_in_segment=len(episodes),last100_episodes=len(last),
                last100_mean_duration_s=float(np.mean([int(r['l'])*config['dt_s'] for r in last])) if last else None,
                last100_complete=sum(r['stop_reason']=='durata completata' if 'stop_reason' in r else
                    int(r['l'])==round(config['duration_s']/config['dt_s']) for r in last),
                last100_mean_reward=float(np.mean([float(r['r']) for r in last])) if last else None,
                exploration_sigma_sx_pwm_percent=math.exp(status['log_std'][0])*config['max_pwm'] if 'log_std' in status else None,
                exploration_sigma_dx_pwm_percent=math.exp(status['log_std'][1])*config['max_pwm'] if 'log_std' in status else None))
        except (ValueError,KeyError,OSError) as error:
            warnings.append(f'{path}: {error}')
    by_run={r['run']:r for r in training}
    def cumulative(row, seen):
        if row['run'] in seen or row['training_wall_s'] is None:return None
        if not row['parent']:return row['training_wall_s']
        previous=by_run.get(str(Path(row['parent']).resolve()))
        if previous is None:return None
        value=cumulative(previous,seen|{row['run']})
        return None if value is None else row['training_wall_s']+value
    for row in training:row['cumulative_training_wall_s']=cumulative(row,set())
    for path in sorted((log_root/'evaluation').glob('eval_*')):
        if not (path/'summary.json').exists():continue
        try:
            config=json.loads((path/'config.json').read_text())
            results=json.loads((path/'summary.json').read_text())['results']
            parent=str(Path(config['training_run']).resolve());record=by_run.get(parent,{})
            complete=len(results)==len(config['angles'])*len(config.get('controllers',['lqr','rl'])) and not (path/'error.json').exists()
            for result in results:
                trace_path=Path(result['csv'])
                if not trace_path.exists():
                    # Permit moving the whole project to a different machine.
                    relative=trace_path.parts[trace_path.parts.index(path.name)+1:]
                    trace_path=path.joinpath(*relative)
                with trace_path.open() as f:rows=list(csv.DictReader(f))
                tail=[r for r in rows if float(r['t_free_s'])>=config['config']['duration_s']-2-1e-8]
                maxima={}
                if tail:
                    maxima=dict(tail_pitch_deg=max(abs(float(r['pitch_true_deg'])-float(r['pitch_target_deg'])) for r in tail),
                        tail_position_mm=1000*max(math.hypot(float(r['pose_true_x_m']),float(r['pose_true_y_m'])) for r in tail),
                        tail_yaw_deg=max(abs(math.degrees(float(r['yaw_true_rad']))) for r in tail),
                        tail_speed_m_s=max(math.hypot(float(r['velocity_world_forward_m_s']),float(r['velocity_world_lateral_m_s'])) for r in tail))
                failed=[]
                for key,limit in [('tail_pitch_deg',.3),('tail_position_mm',10.),('tail_yaw_deg',2.),('tail_speed_m_s',.02)]:
                    if key not in maxima or maxima[key]>=limit:failed.append(key)
                if len(tail)<200:failed.insert(0,'tail_window_missing')
                if not result['success']:failed.insert(0,'survival')
                evaluations.append(dict(evaluation=path.name,training_run=parent,total_steps=record.get('total_steps'),
                    signature=record.get('signature'),evaluation_complete=complete,seed=config['seed'],force_y_n=config['force_y_n'],
                    controller=result['controller'],initial_position_offset_m=config.get('position_offset_m',0.),
                    initial_yaw_offset_deg=config.get('yaw_offset_deg',0.),angle_deg=result['requested_pitch_deg'],survived=result['success'],
                    stabilized=result['stabilized'],pitch_rms_deg=result['pitch_rms_deg'],position_rms_mm=1000*result['position_rms_m'],
                    pwm_rms_percent=result['pwm_rms_percent'],discounted_return=result['discounted_return'],
                    failed_criteria=';'.join(failed),**maxima))
        except (ValueError,KeyError,OSError) as error:
            warnings.append(f'{path}: {error}')
    return dict(training=training,evaluations=evaluations,warnings=warnings)


def update_registry(log_root=LOG_ROOT):
    log_root=Path(log_root);log_root.mkdir(parents=True,exist_ok=True)
    data=collect(log_root)
    save_json(log_root/'risultati.json',data)
    write_csv(log_root/'training_results.csv',data['training'])
    write_csv(log_root/'evaluation_results.csv',data['evaluations'])
    text='# Registro cumulativo RL\n\n'
    text+='Rigenerato dai log originali, che non vengono sovrascritti. Gli errori nella coda finale sono massimi negli ultimi 2 s.\n\n'
    text+='## Addestramenti e continuazioni\n\n'
    text+='Ogni riga e\' un segmento. I passi totali appartengono alla sua linea di checkpoint: non sommare ricerche indipendenti. '
    text+='La firma identifica reward, rete/scale e iperparametri confrontabili; il parent indica la continuazione. '
    text+='Gli episodi completi di training indicano sopravvivenza con esplorazione e varianti, non superamento delle tolleranze.\n\n'
    text+='| Run | Parent | Firma | Passi inizio -> fine | Aggiunti | Minuti segmento / cumulativi | Durata media ultimi 100 [s] | Completi ultimi 100 |\n|---|---|---|---|---|---|---|---|\n'
    for r in data['training']:
        parent=Path(r['parent']).name if r['parent'] else 'da zero'
        minutes=f'{r["training_wall_s"]/60:.1f}' if r['training_wall_s'] is not None else 'n/d'
        cumulative_minutes=f'{r["cumulative_training_wall_s"]/60:.1f}' if r['cumulative_training_wall_s'] is not None else 'n/d'
        duration=f'{r["last100_mean_duration_s"]:.2f}' if r['last100_mean_duration_s'] is not None else 'n/d'
        text+=f'| {r["name"]} | {parent} | {r["signature"]} | {r["start_steps"]} -> {r["total_steps"]} | {r["additional_steps"]} | {minutes} / {cumulative_minutes} | {duration} | {r["last100_complete"]}/{r["last100_episodes"]} |\n'
    transfers=[r for r in data['training'] if r.get('actor_source_run')]
    if transfers:
        text+='\n### Linee inizializzate da actor RL esistenti\n\n'
        for r in transfers:
            text+=f"- {r['name']}: actor da {Path(r['actor_source_run']).name} ({r['actor_source_steps']} passi originari); fase {r['phase']}, campioni critic {r['critic_samples']}. I contatori della nuova linea non includono l'addestramento originario.\n"
    text+='\n## Valutazioni deterministiche\n\n'
    text+='Soglie: pitch <0.3 deg, posizione <10 mm, yaw <2 deg, velocita planare <0.02 m/s per tutti gli ultimi 2 s. '
    text+='Le valutazioni ripetute guidano la scelta: non sono un test finale indipendente.\n\n'
    text+='| Valutazione | Passi totali | Seed | Forza [N] | s0 [mm] | yaw0 [deg] | RL sopravvive | RL stabilizza | LQR stabilizza | Stato |\n|---|---|---|---|---|---|---|---|---|---|\n'
    names=list(dict.fromkeys(r['evaluation'] for r in data['evaluations']))
    for name in names:
        rows=[r for r in data['evaluations'] if r['evaluation']==name]
        rl=[r for r in rows if r['controller']=='rl'];lqr=[r for r in rows if r['controller']=='lqr'];r=rows[0]
        text+=f'| {name} | {r["total_steps"]} | {r["seed"]} | {r["force_y_n"]:g} | {1000*r.get("initial_position_offset_m",0):g} | {r.get("initial_yaw_offset_deg",0):g} | {sum(x["survived"] for x in rl)}/{len(rl)} | {sum(x["stabilized"] for x in rl)}/{len(rl)} | {sum(x["stabilized"] for x in lqr)}/{len(lqr)} | {"completa" if r["evaluation_complete"] else "parziale/errore"} |\n'
    text+='\n## Diagnostica RL: massimi ultimi 2 secondi\n\n'
    text+='Dati disponibili nella finestra 8-10 s: n/d se il robot si arresta prima. Finestre incomplete sono marcate.\n\n'
    text+='| Valutazione | Passi | Angolo [deg] | s0 [mm] | yaw0 [deg] | Pitch [deg] | Posizione [mm] | Yaw [deg] | Velocita [m/s] | Fuori soglia |\n|---|---|---|---|---|---|---|---|---|---|\n'
    def fmt(row,key,precision):
        return f'{row[key]:.{precision}f}' if row.get(key) is not None else 'n/d'
    for r in data['evaluations']:
        if r['controller']!='rl':continue
        text+=f'| {r["evaluation"]} | {r["total_steps"]} | {r["angle_deg"]:+g} | {1000*r.get("initial_position_offset_m",0):g} | {r.get("initial_yaw_offset_deg",0):g} | {fmt(r,"tail_pitch_deg",3)} | {fmt(r,"tail_position_mm",2)} | {fmt(r,"tail_yaw_deg",2)} | {fmt(r,"tail_speed_m_s",4)} | {r["failed_criteria"] or "nessuno"} |\n'
    text+='\nDettaglio per angolo/controllore, contributi RMS e massimi finali in `evaluation_results.csv`; episodi e tempi in `training_results.csv`. '
    text+='I tempi delle valutazioni non sono inclusi nei minuti di addestramento. '
    text+='La durata necessaria a stabilizzare si potra\' limitare solo fra due traguardi misurati della stessa linea; non e\' una garanzia generale su altri seed/modelli.\n'
    if data['warnings']:text+='\n## Avvisi\n\n'+'\n'.join('- '+w for w in data['warnings'])+'\n'
    temporary=log_root/'RISULTATI.tmp';temporary.write_text(text);temporary.replace(log_root/'RISULTATI.md')
    return data
