#!/usr/bin/env python3
"""Tune native MuJoCo LQR Q with a real-coded genetic algorithm, offline only."""
import argparse
import csv
from datetime import datetime
import hashlib
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
from motion_lqr import design_motion_lqr
from ga_lqr import breed, decode, evaluate, save_json, scenarios, scale_input_penalty, cost_weights


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/ga/lqr_stabilization.json')
    parser.add_argument('--population',type=int)
    parser.add_argument('--generations',type=int)
    parser.add_argument('--train-scenarios',type=int)
    parser.add_argument('--validation-scenarios',type=int)
    parser.add_argument('--seed',type=int)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--optimize-r',dest='optimize_r',action='store_true',help='Ottimizza sei pesi Q e due pesi R (default attuale)')
    mode.add_argument('--fixed-r',dest='optimize_r',action='store_false',help='Ricerca storica sui soli sei pesi Q')
    parser.set_defaults(optimize_r=None)
    for name in cost_weights():
        parser.add_argument('--cost-'+name,type=float,help='Coefficiente fitness '+name+' (non modifica R)')
    parser.add_argument('--r-scale',type=float,help='Moltiplica R di riferimento prima della ricerca (default 1)')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'logs/mujoco/ga')
    parser.add_argument('--no-plots',action='store_true')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    for option in ('population','generations','train_scenarios','validation_scenarios'):
        if getattr(args,option) is not None:
            config[option] = getattr(args,option)
    if args.seed is not None:
        config['ga_seed'] = args.seed
    config['optimize_r'] = args.optimize_r if args.optimize_r is not None else config.get('optimize_r',True)
    if not isinstance(config['optimize_r'],bool):
        parser.error('optimize_r deve essere booleano')
    config['r_scale'] = args.r_scale if args.r_scale is not None else config.get('r_scale',1.0)
    if not math.isfinite(config['r_scale']) or config['r_scale'] <= 0:
        parser.error('--r-scale deve essere positivo e finito')
    if config['population'] < 4 or config['generations'] < 1:
        parser.error('Popolazione >=4, generazioni >=1 richieste')
    if config['train_scenarios'] < 2 or config['validation_scenarios'] < 2:
        parser.error('Almeno nominale + una variante per training e validazione')
    for key in ('ga_seed','train_seed','validation_seed'):
        if not isinstance(config[key],int) or config[key] < 0:
            parser.error('Seed interi non negativi richiesti')
    if set(range(config['train_seed'],config['train_seed']+config['train_scenarios'])) & set(
            range(config['validation_seed'],config['validation_seed']+config['validation_scenarios'])):
        parser.error('Seed training e validazione devono essere disgiunti')
    protocol = config['protocol']
    weights = dict(protocol.get('cost_weights',{}))
    for name in cost_weights():
        value = getattr(args,'cost_'+name)
        if value is not None:
            weights[name] = value
    try:
        protocol['cost_weights'] = cost_weights(weights)
    except ValueError as error:
        parser.error(str(error))
    if (not all(math.isfinite(protocol[k]) for k in ('duration_s','release_pitch_deg','max_pwm'))
            or protocol['duration_s'] < 3 or abs(protocol['release_pitch_deg']) > 5
            or not 0 < protocol['max_pwm'] <= 100 or protocol['feedback_delay_samples'] not in (0,1)):
        parser.error('Protocollo non valido')
    if config.get('resample_each_generation'):
        from ga_robust import validate_config
        validate_config(config)
    batch = args.output_dir.resolve()/('ga_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    batch.mkdir(parents=True,exist_ok=False)
    print('Log GA: '+str(batch),flush=True)
    sources = [ROOT/'src_python'/name for name in ('ga_lqr.py','motion_lqr.py','mujoco_sim.py',
               'mujoco_robot.py','sensor_models.py','lqr_controller.py','balance_controller.py','wheel_dynamics.py','ga_robust.py',
               'stribeck_motor.py')]
    sources += [Path(__file__),ROOT/'models/verifica.xml',ROOT/'config/mujoco_sensors_mpu6500.json',
                ROOT/'config/mujoco_robot01_stribeck.json']
    config['source_sha256'] = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    save_json(batch/'config.json',config)
    profile_path = Path(config.get('motor_parameters', 'config/mujoco_motors_sx_dx_20261030.json'))
    if not profile_path.is_absolute():
        profile_path = ROOT/profile_path
    profile = json.loads(profile_path.read_text())
    if set(profile) != {'wheels'} or set(profile['wheels']) != {'sx','dx'}:
        raise ValueError('Profilo GA richiesto: dinamiche complete SX/DX')
    save_json(batch/'motor_parameters.json', profile)
    config['source_sha256'][str(profile_path)] = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    save_json(batch/'config.json', config)
    nominal = scale_input_penalty(
        design_motion_lqr(wheel_profiles=profile['wheels']), config['r_scale'])
    save_json(batch/'nominal_design.json',nominal)
    if config.get('resample_each_generation'):
        from ga_robust import run_search
        result = run_search(nominal,config,batch)
        if not args.no_plots:
            environment = dict(os.environ,MPLCONFIGDIR='/tmp/terasic-ga-matplotlib')
            for path in sorted((batch/'validation').rglob('*.csv')):
                subprocess.run([sys.executable,str(ROOT/'scripts/plot_motion.py'),str(path),'--format','svg'],
                               env=environment,check=True,stdout=subprocess.DEVNULL)
        print(json.dumps(result,indent=2),flush=True)
        return 0
    train = scenarios(config['train_scenarios'],config['train_seed'],'train')
    validation = scenarios(config['validation_scenarios'],config['validation_seed'],'validation')
    save_json(batch/'scenarios.json',dict(training=train,validation=validation))
    rng = np.random.default_rng(config['ga_seed'])
    # Optional historical mode permits a controlled six-gene comparison.
    gene_count = 8 if config['optimize_r'] else 6
    population = rng.uniform(-1,1,(config['population'],gene_count))
    population[0] = 0.
    cache, history = {}, []
    best_key, best_rank, best_design = None,None,None
    try:
        for generation in range(config['generations']):
            ranks = []
            for index, genes in enumerate(population):
                key = tuple(float(x) for x in genes)
                cached = key in cache
                if not cached:
                    try:
                        design = decode(nominal,genes,protocol['feedback_delay_samples'])
                    except (ValueError,np.linalg.LinAlgError) as error:
                        entry = dict(rank=[len(train)+1,protocol['duration_s']*len(train),1e12],
                                     invalid_design=str(error),trials=[])
                    else:
                        rank, trials = evaluate(design,train,protocol)
                        entry = dict(rank=rank,trials=trials)
                    cache[key] = entry
                entry = cache[key]
                rank = entry['rank']
                ranks.append(rank)
                history.append(dict(generation=generation,individual=index,genes=list(key),
                                    cached=cached,**entry))
                if 'invalid_design' not in entry and (best_rank is None or tuple(rank)<tuple(best_rank)):
                    best_key,best_rank = key,rank
                    best_design = decode(nominal,genes,protocol['feedback_delay_samples'])
                    best_design['ga']['protocol'] = protocol
                    best_design['ga']['training_rank'] = rank
                    save_json(batch/'best.json',best_design)
                save_json(batch/'history.json',history)
                print(f'Generazione {generation+1}/{config["generations"]}, individuo '
                      f'{index+1}/{len(population)}: fallimenti={rank[0]}, costo={rank[2]:.4f}'
                      +(' [cache]' if cached else ''),flush=True)
            save_json(batch/'checkpoint.json',dict(completed_generation=generation,
                population=population.tolist(),ranks=ranks,rng_state=rng.bit_generator.state,
                best_genes=list(best_key) if best_key is not None else None,
                unique_evaluations=len(cache),note='Snapshot diagnostico; il launcher non riprende run interrotti'))
            if generation+1 < config['generations']:
                population = breed(population,ranks,rng)
        if best_design is None:
            raise RuntimeError('Nessun design ammissibile')
        # Validation is run once, after selection. It does not feed the GA.
        comparisons = {}
        for label, design in [('baseline',decode(nominal,np.zeros(gene_count),protocol['feedback_delay_samples'])),
                              ('ga',best_design)]:
            print('Validazione indipendente: '+label,flush=True)
            rank,trials = evaluate(design,validation,protocol,batch/'validation'/label)
            comparisons[label] = dict(rank=rank,trials=trials)
            save_json(batch/'validation.json',comparisons)
        better = tuple(comparisons['ga']['rank']) < tuple(comparisons['baseline']['rank'])
        result = dict(status='complete',validation_improved=better,
            unique_evaluations=len(cache),best_training_rank=best_rank,
            baseline_training_rank=cache[tuple(np.zeros(gene_count))]['rank'],
            validation={label:entry['rank'] for label,entry in comparisons.items()},
            best_config=str(batch/'best.json'),
            note='No active controller replaced. Validation covers only the declared simulated scenarios.')
        save_json(batch/'summary.json',result)
        with (batch/'history.csv').open('w',newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['generation','individual','cached','failed_scenarios','missing_time_s','robust_cost',
                             *['z'+str(i+1) for i in range(gene_count)]])
            for row in history:
                writer.writerow([row['generation'],row['individual'],row['cached'],*row['rank'],*row['genes']])
        if not args.no_plots:
            environment = dict(os.environ,MPLCONFIGDIR='/tmp/terasic-ga-matplotlib')
            for path in sorted((batch/'validation').rglob('*.csv')):
                subprocess.run([sys.executable,str(ROOT/'scripts/plot_motion.py'),str(path),'--format','svg'],
                               env=environment,check=True,stdout=subprocess.DEVNULL)
        print(json.dumps(result,indent=2),flush=True)
    except BaseException as error:
        save_json(batch/'error.json',dict(type=type(error).__name__,message=str(error)))
        raise
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
