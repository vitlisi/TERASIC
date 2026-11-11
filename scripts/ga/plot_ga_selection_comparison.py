#!/usr/bin/env python3
"""Compare completed common-selection evaluations, including interrupted runs."""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src_python'))
from ga_lqr import rank_trials


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ga-run',required=True,type=Path)
    args=parser.parse_args();folder=args.ga_run.resolve()
    config=json.loads((folder/'config.json').read_text())
    scenarios=json.loads((folder/'scenarios.json').read_text())['selection']
    selection=json.loads((folder/'selection.json').read_text())
    baseline=next(r for r in selection if r['origin']=='baseline')
    best=min(selection,key=lambda r:tuple(r['rank']))
    winners=json.loads((folder/'generation_winners.json').read_text())
    expected=len({tuple([0.]*len(baseline['genes'])),*(tuple(r['genes']) for r in winners)})
    partial=len(selection)<expected or not (folder/'summary.json').exists()
    for row in selection:
        if row['rank']!=rank_trials(row['trials']):raise ValueError('Inconsistent ranking')
        if [t['scenario'] for t in row['trials']]!=[s['name'] for s in scenarios]:
            raise ValueError('Different or incomplete scenarios')
        for t,b in zip(row['trials'],baseline['trials']):
            if t['cost_weights']!=b['cost_weights'] or t['actual_release_pitch_deg']!=b['actual_release_pitch_deg']:
                raise ValueError('Comparison conditions differ')
    output=folder/'confronto_performance';output.mkdir(exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR','/tmp/terasic-ga-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(15,11))
    fig.subplots_adjust(top=.85,bottom=.075,hspace=.48,wspace=.23)
    prefix='PARTIAL — ' if partial else ''
    fig.suptitle(prefix+'Baseline and GA on the same selection scenarios',fontsize=17,y=.985)
    widths=config['uncertainties'];width_text=', '.join(f'{k} ±{100*v:g}%' for k,v in widths.items())
    fig.text(.5,.95,width_text+' | common sensor noise and seeds',ha='center',fontsize=10)
    fig.text(.5,.927,f'Evaluated finalists: {len(selection)}/{expected} | displayed candidate: {best["origin"]}'
        f' | robust cost: {baseline["rank"][2]:.4f} → {best["rank"][2]:.4f}',ha='center',fontsize=11)
    x=np.arange(len(scenarios));labels=['N']+[str(i) for i in range(1,len(scenarios))]
    specs=[('cost',1,'Single-trial cost','Cost',None),
           ('position_rms_m',1000,'RMS position','RMS distance [mm]',None),
           ('pwm_rms_percent',1,'PWM RMS','PWM RMS [%]',None),
           ('pitch_rms_deg',1,'RMS pitch error','RMS pitch [°]',None),
           ('tail_max_pitch_error_deg',1,'Maximum pitch error in final 2 s','Maximum error [°]',.3),
           ('saturation_fraction',100,'Samples with at least one saturated motor','Saturation [%]',None)]
    for ax,(key,scale,title,ylabel,limit) in zip(axes.flat,specs):
        for offset,row,color,label in [(-.19,baseline,'tab:orange','Baseline LQR'),(.19,best,'tab:blue','Best evaluated GA')]:
            ax.bar(x+offset,[t[key]*scale for t in row['trials']],width=.36,color=color,label=label)
        ax.set(title=title,ylabel=ylabel,xlabel='Common scenario (N = physical nominal)',xticks=x,xticklabels=labels)
        ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        if limit is not None:ax.axhline(limit,color='gray',ls='--',lw=1.2,label='Final pitch threshold 0.3°')
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.908),ncol=2,frameon=False)
    fig.text(.5,.028,'Common selection: fair comparison of evaluated candidates. '
             'No new simulation; independent validation is not available.' if partial else 'Common selection-set results.',
             ha='center',fontsize=10)
    for ext in ('svg','png'):fig.savefig(output/('confronto_selezione.'+ext),dpi=160)
    plt.close(fig)
    summary={}
    for key,row in [('baseline',baseline),('ga_provvisorio',best)]:
        summary[key]=dict(origin=row['origin'],rank=row['rank'],survived=sum(t['survived'] for t in row['trials']),
            stabilized=sum(t['stabilized'] for t in row['trials']),
            mean_metrics={field:float(np.mean([t[field] for t in row['trials']])) for field in
            ['cost','position_rms_m','pitch_rms_deg','pwm_rms_percent','saturation_fraction','tail_max_pitch_error_deg']})
    report=dict(partial=partial,evaluated_finalists=len(selection),expected_finalists=expected,
        uncertainty_bounds=config['uncertainties'],comparison=summary,common_scenarios=scenarios,
        note='Selection-stage results only; not final validation. No time-series CSVs were retained for this stage.')
    (output/'metriche_selezione.json').write_text(json.dumps(report,indent=2)+'\n')
    with (output/'metriche_per_scenario.csv').open('w',newline='') as stream:
        keys=['scenario','cost','pitch_rms_deg','position_rms_m','pwm_rms_percent','saturation_fraction','tail_max_pitch_error_deg','survived','stabilized']
        writer=csv.DictWriter(stream,fieldnames=['controller']+keys);writer.writeheader()
        for name,row in [('baseline',baseline),('ga_provvisorio',best)]:
            for trial in row['trials']:writer.writerow(dict(controller=name,**{k:trial[k] for k in keys}))
    reduction=100*(1-best['rank'][2]/baseline['rank'][2])
    lines=['# Confronto sul banco comune di selezione — '+('risultato parziale' if partial else 'risultato completo'),
        '',f'Ricerca: `{folder.name}`. Configurazione letta dal run: incertezze {width_text}, Q/R variabili e rumore sensoriale attivo. Nessuna nuova simulazione.',
        '',f'Le {config["generations"]} generazioni sono completate. Il run registra un’interruzione durante la selezione finale: valutati {len(selection)} dei {expected} finalisti distinti, baseline inclusa. Mancano `best.json`, `validation.json` e `summary.json`. Non è disponibile una validazione finale indipendente.' if partial else '',
        '',f'Il migliore fra quelli già valutati proviene da `{best["origin"]}`: costo robusto {best["rank"][2]:.4f}, contro {baseline["rank"][2]:.4f} della baseline (riduzione {reduction:.1f}%). Non è dimostrato che sia il migliore dell’intera shortlist.',
        '','| Media sugli 11 scenari di selezione | Baseline | GA provvisorio |','|---|---:|---:|']
    for label,field,scale in [('RMS pitch [°]','pitch_rms_deg',1),('Posizione RMS [mm]','position_rms_m',1000),('PWM RMS [%]','pwm_rms_percent',1),('Saturation [%]','saturation_fraction',100),('Picco pitch negli ultimi 2 s [°]','tail_max_pitch_error_deg',1)]:
        a=summary['baseline']['mean_metrics'][field]*scale;b=summary['ga_provvisorio']['mean_metrics'][field]*scale
        lines.append(f'| {label} | {a:.4f} | {b:.4f} |')
    lines+=['','Entrambi completano 11/11 prove, ma rispettano tutti i criteri finali in 0/11 prove. Il GA riduce posizione, sforzo e saturazione; il pitch RMS medio peggiora. Non è una stabilizzazione validata.',
        '','Il costo minimo fra generazioni diverse non viene usato per scegliere il candidato: ogni generazione affronta un diverso banco. Il valore di partenza confrontabile è il costo della baseline sul banco comune di selezione.',
        '','Ogni confronto abbina gli stessi 11 identificativi di scenario, la stessa configurazione dei pesi e gli stessi angoli di rilascio. I fattori fisici e i semi sono quelli dell’unico banco salvato in `scenarios.json`; il motore di valutazione lo passa identico a ciascun candidato. Sono verificati i ranking ricalcolandoli dalle metriche.',
        '','I CSV temporali di questa fase sono temporanei e non sono più disponibili. Il grafico riporta quindi metriche per scenario, senza ricostruire curve temporali inesistenti.',
        '','[Grafico SVG](confronto_selezione.svg) · [PNG](confronto_selezione.png) · [Metriche JSON](metriche_selezione.json) · [Tabella CSV](metriche_per_scenario.csv)',
        '','Per concludere il protocollo servono le tre valutazioni di selezione mancanti e la validazione finale; non è necessario ripetere le dieci generazioni già registrate. Non sono stati riavviati esperimenti né modificati guadagni.']
    (output/'REPORT_SELEZIONE.md').write_text('\n'.join(lines)+'\n')
    print(output/'confronto_selezione.svg')


if __name__=='__main__':main()
