"""Motor-uncertainty GA with common scenarios within each generation.

Physical parameters are fixed inside a rollout. Generation winners are compared
on a separate common selection bank before one final held-out validation.
"""
import copy
import csv
import math
from pathlib import Path
import numpy as np
from ga_lqr import breed, decode, design_hash, evaluate, save_json


def validate_config(config):
    if config.get('resample_each_generation') is not True:
        raise ValueError('Modalita robusta richiesta')
    if config.get('uncertainties',{}).keys() != {'armature','damping','joint_friction','motor'}:
        raise ValueError('Richieste quattro famiglie di incertezze motore')
    for value in config['uncertainties'].values():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<=value<1:
            raise ValueError('Incertezze relative richieste in [0,1)')
    noise=config['measurement_noise']
    if set(noise) != {'imu_accel_scale','imu_gyro_scale','encoder_std_counts_range'}:
        raise ValueError('Configurazione rumore non riconosciuta')
    for k in ('imu_accel_scale','imu_gyro_scale'):
        if not math.isfinite(noise[k]) or noise[k]<0:
            raise ValueError('Scala rumore non valida')
    limits=noise['encoder_std_counts_range']
    if len(limits)!=2 or not all(math.isfinite(v) for v in limits) or not 0<=limits[0]<=limits[1]:
        raise ValueError('Intervallo rumore encoder non valido')
    for k in ('selection_seed','selection_scenarios'):
        if isinstance(config[k],bool) or not isinstance(config[k],int) or config[k]<0:
            raise ValueError(k+' intero non negativo richiesto')
    if config['selection_scenarios']<2:
        raise ValueError('Selezione: nominale e almeno una variante richiesti')
    groups=[set(range(config['train_seed'],config['train_seed']+config['train_scenarios']*config['generations'])),
            set(range(config['selection_seed'],config['selection_seed']+config['selection_scenarios'])),
            set(range(config['validation_seed'],config['validation_seed']+config['validation_scenarios']))]
    if any(groups[i]&groups[j] for i in range(3) for j in range(i)):
        raise ValueError('Seed training/selezione/validazione sovrapposti')
    if any(k.startswith('disturbance_') for k in config['protocol']):
        raise ValueError('Questa prima campagna esclude disturbi esterni')


def motor_scenarios(count,seed,prefix,uncertainties,noise):
    rng=np.random.default_rng(seed)
    result=[]
    for i in range(count):
        nominal=i==0
        def factors(name):
            width=uncertainties[name]
            return [1.,1.] if nominal else rng.uniform(1-width,1+width,2).tolist()
        result.append(dict(name=f'{prefix}_{i:02d}',seed=seed+i,
            encoder_noise_seed=seed+i+2**32,
            motor_scale=factors('motor'),armature_scale=factors('armature'),
            damping_scale=factors('damping'),joint_friction_scale=factors('joint_friction'),
            friction_scale=1.,chassis_inertia_scale=1.,
            imu_noise_scale=[noise['imu_accel_scale'],noise['imu_gyro_scale']],
            encoder_noise_std_counts=0. if nominal else float(rng.uniform(*noise['encoder_std_counts_range'])),
            nominal_physics=nominal,
            noise_note='Zero-mean Gaussian sensor noise; IMU injected before sensor filters; encoder position noise differentiated and filtered.'))
    return result


def schedule(config):
    validate_config(config)
    def bank(n,seed,prefix):
        return motor_scenarios(n,seed,prefix,config['uncertainties'],config['measurement_noise'])
    return dict(training_by_generation=[bank(config['train_scenarios'],
                config['train_seed']+g*config['train_scenarios'],f'train_g{g+1:03d}')
                for g in range(config['generations'])],
                selection=bank(config['selection_scenarios'],config['selection_seed'],'selection'),
                validation=bank(config['validation_scenarios'],config['validation_seed'],'validation'))


def run_search(nominal,config,batch,evaluator=None,decoder=None):
    """evaluator/decoder injection permits orchestration tests without rollouts."""
    evaluator=evaluate if evaluator is None else evaluator
    decoder=decode if decoder is None else decoder
    batch=Path(batch);banks=schedule(config);save_json(batch/'scenarios.json',banks)
    protocol=config['protocol'];delay=protocol['feedback_delay_samples']
    n=8 if config['optimize_r'] else 6
    rng=np.random.default_rng(config['ga_seed'])
    population=rng.uniform(-1,1,(config['population'],n));population[0]=0.
    history=[];winners=[];shortlist={tuple(np.zeros(n)):dict(genes=np.zeros(n).tolist(),origin='baseline')}
    evaluated=0
    try:
        for generation,scenes in enumerate(banks['training_by_generation']):
            # A fresh cache prevents stale fitness from earlier environments.
            cache={};ranks=[];valid=[]
            for index,genes in enumerate(population):
                key=tuple(float(x) for x in genes);cached=key in cache
                if not cached:
                    try:
                        design=decoder(nominal,genes,delay)
                    except (ValueError,np.linalg.LinAlgError) as error:
                        entry=dict(rank=[len(scenes)+1,protocol['duration_s']*len(scenes),1e12],
                                   invalid_design=str(error),trials=[])
                    else:
                        rank,trials=evaluator(design,scenes,protocol)
                        entry=dict(rank=rank,trials=trials);evaluated+=1
                    cache[key]=entry
                entry=cache[key];ranks.append(entry['rank'])
                if 'invalid_design' not in entry:valid.append(index)
                history.append(dict(generation=generation,individual=index,genes=list(key),
                    cached=cached,scenario_bank_hash=design_hash(scenes),**entry))
                save_json(batch/'history.json',history)
                print(f'Generazione {generation+1}/{config["generations"]}, individuo {index+1}/{len(population)}: '
                      f'fallimenti={entry["rank"][0]}, costo={entry["rank"][2]:.4f}'+(' [cache generazione]' if cached else ''),flush=True)
            if valid:
                index=min(valid,key=lambda i:tuple(ranks[i]));key=tuple(float(x) for x in population[index])
                winner=dict(generation=generation,genes=list(key),rank=ranks[index],scenario_bank_hash=design_hash(scenes))
                winners.append(winner);shortlist.setdefault(key,dict(genes=list(key),origin=f'generation_{generation+1}'))
                save_json(batch/'generation_winners.json',winners)
                provisional=decoder(nominal,population[index],delay)
                provisional['ga']['protocol']=copy.deepcopy(protocol)
                provisional['ga']['note']='Winner of this generation only; costs across generations use different scenarios.'
                save_json(batch/'generation_best.json',provisional)
            save_json(batch/'checkpoint.json',dict(completed_generation=generation,population=population.tolist(),
                ranks=ranks,rng_state=rng.bit_generator.state,scenario_bank_hash=design_hash(scenes),
                generation_winners=winners,note='Diagnostic snapshot; no automatic resume. No cross-generation fitness comparison.'))
            if generation+1<config['generations']:population=breed(population,ranks,rng)
        # Common selection bank determines the winner, never the validation bank.
        selection=[]
        for key,candidate in shortlist.items():
            try:design=decoder(nominal,np.asarray(key),delay)
            except (ValueError,np.linalg.LinAlgError):continue
            print('Selezione comune: '+candidate['origin'],flush=True)
            rank,trials=evaluator(design,banks['selection'],protocol)
            selection.append(dict(**candidate,rank=rank,trials=trials))
            save_json(batch/'selection.json',selection)
        if not selection:raise RuntimeError('Nessun candidato valido per la selezione')
        chosen=min(selection,key=lambda item:tuple(item['rank']))
        best=decoder(nominal,np.asarray(chosen['genes']),delay)
        best['ga'].update(protocol=copy.deepcopy(protocol),selection_rank=chosen['rank'],
            selection_origin=chosen['origin'],policy_selection='Generation winners + baseline; common selection bank; validation held out.')
        save_json(batch/'best.json',best)
        comparisons={}
        for label,design in [('baseline',decoder(nominal,np.zeros(n),delay)),('ga',best)]:
            print('Validazione indipendente: '+label,flush=True)
            rank,trials=evaluator(design,banks['validation'],protocol,batch/'validation'/label)
            comparisons[label]=dict(rank=rank,trials=trials);save_json(batch/'validation.json',comparisons)
        baseline=next(row for row in selection if row['origin']=='baseline')
        result=dict(status='complete',scenario_policy='new common bank per generation; physics fixed within rollout',
            unique_genomes=len({tuple(r['genes']) for r in history}),training_evaluations=evaluated,
            shortlist_candidates=len(selection),best_selection_rank=chosen['rank'],baseline_selection_rank=baseline['rank'],
            validation={k:v['rank'] for k,v in comparisons.items()},
            validation_improved=tuple(comparisons['ga']['rank'])<tuple(comparisons['baseline']['rank']),
            best_config=str(batch/'best.json'),note='No active controller replaced; no external disturbances; no cross-generation training best rank.')
        save_json(batch/'summary.json',result)
        with (batch/'history.csv').open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['generation','individual','cached','scenario_bank_hash',
                'failed_scenarios','missing_time_s','robust_cost',*['z'+str(i+1) for i in range(n)]])
            for row in history:writer.writerow([row['generation'],row['individual'],row['cached'],row['scenario_bank_hash'],*row['rank'],*row['genes']])
        return result
    except BaseException as error:
        save_json(batch/'error.json',dict(type=type(error).__name__,message=str(error)));raise
