#!/usr/bin/env python3
"""Fixed-frame MuJoCo preview or replay of a completed freewheel_v1 hardware run."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src_python'))
from wheel_dynamics import configure_wheels, update_wheels
XML = ROOT / 'models/verifica.xml'
WHEELS = ('sx', 'dx')
STARTS = (1., 6., 11.)
AMPLITUDES = (10., 15., 20.)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_csv(path, rows):
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path):
    with Path(path).open(newline='') as stream:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(stream)]


def segments(protocol_id='freewheel_v1'):
    if protocol_id == 'freewheel_repeat_low_v1':
        values=(5.,-5.,6.,-6., -6.,6.,-5.,5., 6.,5.,-6.,-5.,
                -5.,-6.,5.,6., 5.,6.,-5.,-6.)
        return [(1.+4*i,2.,p) for i,p in enumerate(values)]
    if protocol_id == 'freewheel_startup_low_v1':
        return segments('freewheel_static_v1')[:8]
    if protocol_id == 'freewheel_holding_v1':
        return [(1.+18*b+2*k, 2., p*(1 if b%2 == 0 else -1))
                for b in range(4) for k,p in enumerate((10.,8.,6.,5.,4.,3.,2.))]
    if protocol_id == 'freewheel_static_v1':
        values = (3., -3., 4., -4., 5., -5., 6., -6., 8., -8.,
                  10., -10., 15., -15., 20., -20., -5., 5., -6., 6.)
        return [(1.+4*i, 2., p) for i, p in enumerate(values)]
    if protocol_id == 'freewheel_v1':
        return [(s, 1., p) for s, p in zip(STARTS, AMPLITUDES)]
    if protocol_id != 'freewheel_small_v1':
        raise ValueError('Unknown protocol')
    isolated = [(1.+2*i, .5, p) for i, p in enumerate((3., -3., 5., -5., 8., -8.))]
    return isolated + [(14.+4*b+.25*k, .25, p*(1 if k%2 == 0 else -1))
                       for b, p in enumerate((3., 5., 8.)) for k in range(8)]


def duration(protocol_id):
    if protocol_id == 'freewheel_repeat_low_v1':
        return 81
    if protocol_id == 'freewheel_startup_low_v1':
        return 33
    if protocol_id == 'freewheel_holding_v1':
        return 73
    if protocol_id == 'freewheel_static_v1':
        return 81
    return 15 if protocol_id == 'freewheel_v1' else 27


def protocol(t, protocol_id='freewheel_v1'):
    for phase, (start, width, pwm) in enumerate(segments(protocol_id), 1):
        if start <= t < start + width:
            return phase, pwm
    return 0, 0.


def preview_rows(voltage, protocol_id='freewheel_v1'):
    rows = []
    for i in range(duration(protocol_id)*100+1):
        t = i / 100
        phase, pwm = protocol(t, protocol_id)
        row = dict(t_s=t, phase=phase, battery_v=voltage)
        for w in WHEELS:
            row[f'pwm_{w}_applied_percent'] = pwm
        rows.append(row)
    return rows


def configuration(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        words = line.split('#')[0].split()
        if len(words) == 2:
            values[words[0]] = float(words[1])
    if (values['encoder_sx_sign'], values['encoder_dx_sign']) != (-1., 1.):
        raise ValueError('Expected physical_sx_dx_v2 encoder signs -1,+1')
    cpr = values['counts_per_rev']
    if not math.isfinite(cpr) or cpr <= 0:
        raise ValueError('Invalid CPR')
    return cpr


def validate_real(rows, status, protocol_id='freewheel_v1'):
    required = {'t_s', 'phase', 'battery_v', 'battery_raw', 'battery_age_s'}
    for w in WHEELS:
        required.update((f'enc_{w}_raw', f'pwm_{w}_applied_percent',
                         f'pwm_{w}_requested_percent', f'control_{w}', f'period_{w}', f'high_{w}'))
    if not duration(protocol_id)*100-2 <= len(rows) <= duration(protocol_id)*100+3 or not required.issubset(rows[0]):
        raise ValueError('Incomplete run or unexpected CSV schema')
    if any(not math.isfinite(v) for r in rows for v in r.values()):
        raise ValueError('Non-finite CSV data')
    for marker in ('protocol_id='+protocol_id, 'wheel_convention=physical_sx_dx_v2',
                   'result=COMPLETED', 'final_control_sx=6 final_control_dx=6'):
        if marker not in status:
            raise ValueError('Missing status marker: ' + marker)
    if 'result=ABORTED' in status:
        raise ValueError('Aborted hardware run')
    t = np.array([r['t_s'] for r in rows])
    if not (0 <= t[0] < .02 and duration(protocol_id) <= t[-1] < duration(protocol_id)+.05 and
            np.all((np.diff(t) > .002) & (np.diff(t) < .041))):
        raise ValueError('Unexpected duration or sample timing')
    for r in rows:
        phase, expected = protocol(r['t_s'], protocol_id)
        # MMIO timestamp follows the command write; allow boundary microseconds.
        at_edge = any(abs(r['t_s'] - e) < .003 for s, width, _ in segments(protocol_id) for e in (s, s+width))
        if not at_edge and r['phase'] != phase:
            raise ValueError('Unexpected pulse phase')
        if r['battery_v']:
            if not (10.5 <= r['battery_v'] <= 13 and 0 <= r['battery_age_s'] <= .151):
                raise ValueError('Invalid/stale voltage')
            if abs(r['battery_v'] - r['battery_raw']*18/4.7/1000) > 1e-6:
                raise ValueError('ADC voltage scale mismatch')
        for w in WHEELS:
            control, period, high = (r[f'{k}_{w}'] for k in ('control', 'period', 'high'))
            duty = ((1 if int(control)&2 else -1)*100*high/period if int(control) & 1 else 0) if period else 0
            if int(control) not in ((6, 7) if protocol_id == 'freewheel_v1' else (5, 6, 7)) or period != 50000 or not 0 <= high <= period:
                raise ValueError('Unexpected hardware PWM registers')
            if abs(duty-r[f'pwm_{w}_applied_percent']) > 1e-6:
                raise ValueError('PWM readback mismatch')
            requested = r[f'pwm_{w}_requested_percent']
            if abs(duty-requested) > .002 or (not at_edge and abs(duty-expected) > .002):
                raise ValueError('Applied PWM does not match protocol')
            raw = r[f'enc_{w}_raw']
            if not 256 < raw < 65279 or raw != int(raw):
                raise ValueError('Encoder limit/non-integer; no wrap assumed')
            if duty and not r['battery_v']:
                raise ValueError('Pulse without a voltage measurement')
    for phase, (_, width, _) in enumerate(segments(protocol_id), 1):
        selected = [r for r in rows if r['phase'] == phase]
        if not width*100-3 <= len(selected) <= width*100+3:
            raise ValueError('Missing/truncated pulse')
    if any(rows[-1][f'pwm_{w}_applied_percent'] for w in WHEELS):
        raise ValueError('Final sample must have motors OFF')


def fixed_model(armature_scale=1., wheels=None):
    """Remove root freedom; unlike a soft weld this fixture cannot pitch or move."""
    tree = ET.parse(XML)
    root = tree.getroot()
    root.find('compiler').set('meshdir', str(ROOT/'models/meshes'))
    body = root.find(".//body[@name='struttura_centrale']")
    body.remove(body.find('freejoint'))
    body.set('pos', '0 0 0.30')
    weld = root.find(".//equality/weld[@name='manual_support']")
    root.find('equality').remove(weld)
    for w in WHEELS:
        joint = body.find(f".//joint[@name='perno_{w}']")
        joint.set('armature', str(float(joint.get('armature'))*armature_scale))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    profiles = configure_wheels(model, wheels or {})
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if model.nv != 2 or data.ncon:
        raise ValueError('Fixture must have exactly two wheel DOFs and no contact')
    # Same nominal PWM -> torque conversion as MujocoRobot; no retuning.
    nominal = float(model.numeric('V_nom').data[0])
    coefficient = float(model.numeric('motor_max_torque').data[0])/nominal
    mass = np.zeros((model.nv, model.nv))
    if hasattr(data, 'qM'):  # MuJoCo before the mass-matrix API change.
        mujoco.mj_fullM(model, mass, data.qM)
    else:
        mujoco.mj_fullM(model, data, mass)
    details = dict(fixture='kinematic fixed chassis, axle height 0.30m, no ground contact',
                   armature_scale=armature_scale, torque_per_duty_per_volt_Nm=coefficient,
                   J_equivalent_kg_m2=np.diag(mass).tolist(),
                   armature_kg_m2=model.dof_armature.tolist(),
                   damping_Nm_s_rad=model.dof_damping.tolist(),
                   frictionloss_Nm=model.dof_frictionloss.tolist(),
                   wheel_profiles=profiles,
                   backlash_modelled=False, controller='none: open-loop PWM')
    return model, data, coefficient, details


def replay(rows, cpr, armature_scale=1., wheels=None):
    model, data, coefficient, details = fixed_model(armature_scale, wheels)
    joints = [int(model.joint('perno_'+w).qposadr[0]) for w in WHEELS]
    dofs = [int(model.joint('perno_'+w).dofadr[0]) for w in WHEELS]
    first_voltage = next(r['battery_v'] for r in rows if r['battery_v'] > 0)
    previous_t = 0.
    simulated = []
    for row in rows:
        target = row['t_s']
        while previous_t < target - 1e-12:
            model.opt.timestep = min(.001, target-previous_t)
            mujoco.mj_step(model, data)
            previous_t = float(data.time)
        voltage = row['battery_v'] or first_voltage
        result = dict(t_s=target, phase=row['phase'], battery_v=voltage)
        for i, w in enumerate(WHEELS):
            model.actuator('motore_'+w).gear[0] = -coefficient*voltage
            data.ctrl[i] = row[f'pwm_{w}_applied_percent']/100
        update_wheels(model, data.ctrl*100, details['wheel_profiles'])
        mujoco.mj_forward(model, data)
        if data.ncon:
            raise ValueError('Unexpected wheel contact')
        for i, w in enumerate(WHEELS):
            angle = float(data.qpos[joints[i]])
            # Oracle wheel angle converted to raw-channel counts, as in twin.
            counts = int(np.rint(angle*cpr/(2*math.pi)*(1 if w == 'sx' else -1)))
            result.update({f'pwm_{w}_applied_percent': row[f'pwm_{w}_applied_percent'],
                f'enc_{w}_counts': counts, f'position_{w}_true_rad': -angle,
                f'velocity_{w}_true_rad_s': -float(data.qvel[dofs[i]]),
                f'torque_{w}_drive_Nm': -float(data.qfrc_actuator[dofs[i]]),
                f'torque_{w}_net_Nm': -float(data.qfrc_actuator[dofs[i]] +
                    data.qfrc_passive[dofs[i]] + data.qfrc_constraint[dofs[i]])})
        simulated.append(result)
    return simulated, details


def derivatives(t, position):
    """Identical local cubic fit for both count traces; no filtered real vs oracle."""
    velocity, acceleration = np.zeros(len(t)), np.zeros(len(t))
    for i in range(len(t)):
        lo = max(0, min(i-4, len(t)-9))
        x = t[lo:lo+9]-t[i]
        coefficients = np.polynomial.polynomial.polyfit(x/.04, position[lo:lo+9], 3)
        velocity[i] = coefficients[1]/.04
        acceleration[i] = 2*coefficients[2]/.04**2
    return velocity, acceleration


def analyze(real, simulated, cpr, details, protocol_id='freewheel_v1'):
    t = np.array([r['t_s'] for r in simulated])
    voltage = np.array([r['battery_v'] for r in simulated])
    aligned = [dict(t_s=float(v), battery_v=float(voltage[i])) for i, v in enumerate(t)]
    metrics = []
    for j, w in enumerate(WHEELS):
        traces = {'mujoco': np.array([r[f'enc_{w}_counts'] for r in simulated])}
        if real:
            raw = np.array([r[f'enc_{w}_raw'] for r in real])
            traces['real'] = raw-raw[0]
        for label, counts in traces.items():
            position = counts*(2*math.pi/cpr)*(-1 if w == 'sx' else 1)
            speed, acceleration = derivatives(t, position)
            for i, output in enumerate(aligned):
                output.update({f'{label}_{w}_counts': int(counts[i]),
                    f'{label}_{w}_position_rad': float(position[i]),
                    f'{label}_{w}_velocity_rad_s': float(speed[i]),
                    f'{label}_{w}_acceleration_rad_s2': float(acceleration[i])})
            if label == 'real':
                # Conditional inverse dynamics, NOT an independent torque measure.
                profile = details.get('wheel_profiles', {}).get(w)
                damping = np.full(len(t), details['damping_Nm_s_rad'][j])
                friction = np.full(len(t), details['frictionloss_Nm'][j])
                if profile:
                    off = np.array([r[f'pwm_{w}_applied_percent'] == 0 for r in simulated])
                    damping = np.where(off, profile['off_damping_Nm_s_rad'], profile['damping_Nm_s_rad'])
                    friction = np.where(off, profile['off_frictionloss_Nm'], profile['frictionloss_Nm'])
                inferred = (details['J_equivalent_kg_m2'][j]*acceleration +
                            damping*speed + friction*np.sign(speed))
                for i, output in enumerate(aligned):
                    output[f'real_{w}_torque_inverse_assumed_Nm'] = float(inferred[i])
        pwm = np.array([r[f'pwm_{w}_applied_percent'] for r in simulated])
        for i, output in enumerate(aligned):
            output[f'mujoco_{w}_pwm_percent'] = float(pwm[i])
            output[f'mujoco_{w}_torque_drive_Nm'] = simulated[i][f'torque_{w}_drive_Nm']
            output[f'mujoco_{w}_torque_net_Nm'] = simulated[i][f'torque_{w}_net_Nm']
            if real:
                output[f'real_{w}_pwm_percent'] = real[i][f'pwm_{w}_applied_percent']
                output[f'real_{w}_torque_PWM_proxy_Nm'] = float(
                    details['torque_per_duty_per_volt_Nm']*voltage[i]*pwm[i]/100)
        if real:
            for phase, (start, width, amplitude) in enumerate(segments(protocol_id), 1):
                mask = (t >= start) & (t < start+width)
                # Include post-pulse coast/braking response up to next slot.
                following = segments(protocol_id)[phase:phase+1]
                stop = min(start+4, following[0][0]) if following else min(start+4, duration(protocol_id))
                window = (t >= start) & (t < stop)
                record = dict(wheel=w, pulse=phase, PWM_percent=amplitude, start_s=start, width_s=width,
                              voltage_min_V=float(min(voltage[mask])),
                              voltage_mean_V=float(np.mean(voltage[mask])))
                for name, unit in [('counts', 'counts'), ('velocity_rad_s', 'rad_s')]:
                    a = np.array([r[f'real_{w}_{name}'] for r in aligned])
                    b = np.array([r[f'mujoco_{w}_{name}'] for r in aligned])
                    record['RMSE_'+unit] = float(np.sqrt(np.mean((a[window]-b[window])**2)))
                for label in ('real', 'mujoco'):
                    record[label+'_travel_counts_during_pulse'] = int(
                        traces[label][mask][-1]-traces[label][mask][0])
                    v = np.array([r[f'{label}_{w}_velocity_rad_s'] for r in aligned])
                    record[label+'_peak_speed_rad_s'] = float(max(abs(v[mask])))
                denominator = abs(record['real_travel_counts_during_pulse'])
                record['pulse_travel_difference_percent_of_real'] = (
                    100*(abs(record['mujoco_travel_counts_during_pulse'])-denominator)/denominator
                    if denominator else None)
                metrics.append(record)
    return aligned, metrics


def plots(folder, aligned, has_real, protocol_id='freewheel_v1'):
    os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir())/'terasic-freewheel-matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = [r['t_s'] for r in aligned]
    fig, axes = plt.subplots(4, 2, figsize=(14, 12), sharex=True)
    for j, w in enumerate(WHEELS):
        for k, (field, label) in enumerate((('pwm_percent', 'PWM applicata [%]'),
                ('counts', 'Encoder: conteggi dal primo campione'),
                ('velocity_rad_s', 'Velocità da encoder [rad/s]'),
                ('torque_drive_Nm', 'Coppia motrice / stime [N m]'))):
            ax = axes[k, j]
            ax.plot(t, [r[f'mujoco_{w}_{field}'] for r in aligned], label='MuJoCo')
            if has_real:
                if k < 3:
                    ax.plot(t, [r[f'real_{w}_{field}'] for r in aligned], 'k', lw=1, label='Reale')
                else:
                    ax.plot(t, [r[f'real_{w}_torque_inverse_assumed_Nm'] for r in aligned],
                            'k', lw=.8, label='Reale: stima con inerzia/attriti del modello')
                    ax.plot(t, [r[f'real_{w}_torque_PWM_proxy_Nm'] for r in aligned],
                            '--', label='Reale: proxy PWM × V (stessa costante)')
            ax.set_ylabel(label)
            ax.grid(alpha=.25)
            if k == 0:
                ax.set_title('Ruota fisica '+w.upper())
            ax.legend(fontsize=7)
        axes[-1, j].set_xlabel('Tempo monotono dall’avvio [s]')
    subtitle = ('Confronto: la coppia reale è stimata, non misurata' if has_real else
                'Anteprima MuJoCo — nessun dato reale')
    title = 'impulsi 10 / 15 / 20%' if protocol_id == 'freewheel_v1' else 'piccoli comandi ±3 / ±5 / ±8% e inversioni'
    fig.suptitle('Ruote libere — telaio fisso, '+title+'\n'+subtitle, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(folder/'confronto.svg')
    fig.savefig(folder/'confronto.png', dpi=140)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 3))
    ax.plot(t, [r['battery_v'] for r in aligned])
    ax.set(xlabel='Tempo [s]', ylabel='Tensione [V]',
           title='Tensione ADC del reale usata nel replay' if has_real else 'Tensione ipotizzata per l’anteprima')
    ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(folder/'batteria.svg')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('simulate', 'compare'))
    parser.add_argument('--protocol', choices=('freewheel_v1', 'freewheel_small_v1'), default='freewheel_v1')
    parser.add_argument('--real', type=Path)
    parser.add_argument('--status', type=Path)
    parser.add_argument('--config', type=Path, help='Snapshot CPR/signs; defaults to REAL.config.txt')
    parser.add_argument('--voltage', type=float, help='Required for simulation-only preview')
    parser.add_argument('--armature-scale', type=float, default=1., help='Explicit experimental variant')
    parser.add_argument('--parameters', type=Path, help='Explicit JSON with separate sx/dx wheel profiles')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.armature_scale) or args.armature_scale <= 0:
        parser.error('Positive finite armature scale required')
    real, status_path = None, None
    if args.mode == 'compare':
        if not args.real or args.voltage is not None:
            parser.error('compare requires --real and uses measured voltage, without --voltage')
        status_path = args.status or args.real.with_suffix('.status.log')
        real = read_csv(args.real)
        validate_real(real, status_path.read_text(), args.protocol)
        rows = real
        config = args.config or args.real.with_suffix('.config.txt')
    else:
        if args.real or args.status or args.voltage is None or not math.isfinite(args.voltage) or not 10.5 <= args.voltage <= 13:
            parser.error('simulate requires --voltage in [10.5,13] and no hardware log')
        rows = preview_rows(args.voltage, args.protocol)
        config = args.config or ROOT/'config/controllers/lqr_mimo.txt'
    cpr = configuration(config)
    # Refuse overwriting a previous comparison; validate inputs before creating output.
    parameters = json.loads(args.parameters.read_text()) if args.parameters else {}
    if set(parameters)-{'wheels'}:
        parser.error('Freewheel parameter JSON supports only wheels')
    simulated, details = replay(rows, cpr, args.armature_scale, parameters.get('wheels'))
    aligned, metrics = analyze(real, simulated, cpr, details, args.protocol)
    folder = args.output_dir
    folder.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(config, folder/'encoder.config.txt')
    inputs = [XML, config, Path(__file__), ROOT/'diagnostics/motor_freewheel.cpp',
              ROOT/'diagnostics/motor_pulse.cpp', ROOT/'src_cpp/terasic_reale/hardware/robot_wheel_map.h',
              ROOT/'src_python/wheel_dynamics.py']
    if args.parameters:
        inputs.append(args.parameters)
        shutil.copyfile(args.parameters, folder/'parameters.json')
    if real:
        shutil.copyfile(args.real, folder/'reale.csv')
        shutil.copyfile(status_path, folder/'reale.status.log')
        inputs += [args.real, status_path]
        source_manifest = args.real.parent/'sources.sha256'
        if source_manifest.is_file():
            shutil.copyfile(source_manifest, folder/'hardware_sources.sha256')
            inputs.append(source_manifest)
    write_csv(folder/'simulation.csv', simulated)
    write_csv(folder/'aligned_signals.csv', aligned)
    if metrics:
        write_csv(folder/'metrics.csv', metrics)
    report = dict(protocol_id=args.protocol, mode=args.mode, real_test_available=bool(real),
        digital_twin_validated=False, counts_per_rev=cpr, model=details, metrics=metrics,
        wheel_convention='physical_sx_dx_v2', duration_s=duration(args.protocol),
        protocol_segments=[dict(start_s=s, width_s=w, pwm_percent=p) for s,w,p in segments(args.protocol)],
        timing='Actual monotonic hardware timestamps; inputs held between samples; physics <=1ms',
        voltage='ADC ~10Hz, last available sample held; initial OFF gap filled with first valid voltage',
        derivatives='Local cubic polynomial, 9 samples (~90ms); identical for real and quantized MuJoCo',
        torque_warning='PWM proxy shares model constant; inverse estimate assumes model inertia and friction. Neither measures real torque independently.',
        stop_mode='RUN=0, high=0, control=0x06. Actual electrical braking response not yet identified.',
        inputs_sha256={str(p.resolve()): sha(p) for p in inputs},
        versions=dict(mujoco=mujoco.__version__, numpy=np.__version__))
    (folder/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    plots(folder, aligned, bool(real), args.protocol)
    lines = ['# Ruote libere — '+('confronto reale / MuJoCo' if real else 'anteprima solo simulata'), '',
        ('Impulsi 10%, 15%, 20%, di 1 s a t=1,6,11 s; durata totale 15 s. Due ruote insieme, avanti.' if args.protocol == 'freewheel_v1' else 'Protocollo freewheel_small_v1: 27 s, entrambe le ruote. Impulsi isolati ±3/5/8% di 0,5 s a t=1/3/5/7/9/11 s. Blocchi ±3/5/8% a t=14/18/22 s, otto semiperiodi da 0,25 s per blocco. Pause OFF fra i blocchi. Profilo congelato, nessun rifit.'), '',
        'Telaio MuJoCo cinematicamente fisso, ruote sollevate, nessun contatto e nessun LQR. '
        'Masse, inerzie, armature e attriti originali, salvo variante esplicitata nel JSON. '
        'Il dongle solidale al telaio fissato non cambia questa dinamica delle ruote.', '',
        f'CPR={cpr:g}; armature_scale={args.armature_scale:g}. '
        'Stessi segni fisici v2; encoder confrontati in conteggi, velocità positiva in avanti.', '',
        'Nel confronto si riproducono i readback PWM e la tensione ADC del reale, mantenuti fra campioni. '
        'PWM coincidenti verificano gli ingressi, non la fedeltà del motore.', '',
        '**Coppie reali non misurate.** Proxy = (0,441/12) × V × duty: coincide per costruzione con '
        'la conversione del modello. Stima inversa = J_ipotizzata × accelerazione + b_ipotizzato × velocità '
        '+ attrito_ipotizzato × segno(velocità). Usa le inerzie e gli attriti del modello, '
        'quindi non consente di identificare indipendentemente J e coppia. Le derivate su 9 campioni '
        'sono sensibili alla quantizzazione e smussano i transitori; non interpretare i picchi come misure.', '',
        'RUN=0 è replicato con comando nullo e attriti del modello. Un diverso decadimento può '
        'evidenziare frenatura elettrica/attriti mancanti. Il circuito del driver in OFF va verificato '
        'prima di attribuire tutto ad armature.', '',
        'Questo test isola la trasmissione a vuoto. Non identifica il backlash senza una misura '
        'indipendente della ruota, né valida contatto, bilanciamento e path. Nessuna soglia di '
        'accettazione o percentuale globale di fedeltà viene inventata.', '',
        '[Grafici](confronto.svg), [tensione](batteria.svg), [segnali](aligned_signals.csv), '
        '[parametri e provenienza](summary.json).']
    if metrics:
        lines += ['', '| Ruota | PWM | RMSE encoder [conteggi] | RMSE velocità [rad/s] | Δ corsa impulso [% reale] |',
                  '|---|---:|---:|---:|---:|']
        for r in metrics:
            pct = r['pulse_travel_difference_percent_of_real']
            lines.append(f"| {r['wheel']} | {r['PWM_percent']:g} | {r['RMSE_counts']:.2f} | "
                         f"{r['RMSE_rad_s']:.3f} | {pct:.1f} |" if pct is not None else
                         f"| {r['wheel']} | {r['PWM_percent']:g} | {r['RMSE_counts']:.2f} | {r['RMSE_rad_s']:.3f} | n/d |")
    (folder/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(folder)


if __name__ == '__main__':
    main()
