"""Simulazione indipendente: PD, LQR o Hinf MuJoCo, ingresso PWM."""
import argparse
from contextlib import contextmanager, nullcontext
from dataclasses import asdict
from datetime import datetime
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
import threading
import time
import uuid
import mujoco
import numpy as np
from balance_controller import BalanceController, ControllerConfig
from mujoco_robot import MujocoRobot, Calibration, ROOT, XML_PATH, DEFAULT_MOTOR_FRICTION_PROFILE
from battery_model import BatteryPack, BatteryDepleted
from viewer_hud import build_hud

@contextmanager
def open_viewer(robot):
    from mujoco import viewer
    threads = set(threading.enumerate())
    handle = viewer.launch_passive(robot.m, robot.d)
    new = [t for t in threading.enumerate() if t not in threads]
    try:
        with handle:
            yield handle
    finally:
        handle.close()
        for t in new:
            t.join(timeout=5)

def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--hide-ultrasonic-cone', action='store_true',
                   help='Nasconde il cono grafico senza disabilitare il sensore')
    p.add_argument('--ultrasonic-test-distance-m', type=float, default=None,
                   help='Parete fissa opzionale davanti al sensore nella posa iniziale [m]')
    p.add_argument('--ultrasonic-config', default=str(ROOT/'config/mujoco_ultrasonic_hcsr04.json'))
    p.add_argument('--exit-on-stop', action='store_true')
    p.add_argument('--sensor', choices=('imu', 'ideal'), default='imu')
    p.add_argument('--imu-model',choices=('ideal','mpu6500'),default='ideal')
    p.add_argument('--encoder-model',choices=('ideal','motor'),default='ideal')
    p.add_argument('--sensor-config',default=str(ROOT/'config/mujoco_sensors_mpu6500.json'))
    p.add_argument('--sensor-seed',type=int,default=0)
    p.add_argument('--encoder-counts-per-rev',type=float,default=None,help='Conteggi FPGA per giro RUOTA')
    p.add_argument('--mode', choices=('pd', 'lqr', 'lqr-motion', 'lqr-path', 'hinf', 'observe'), default='pd')
    p.add_argument('--hinf-config', default=str(ROOT/'models/hinf/simscape_2pwm_20261026.json'))
    p.add_argument('--lqr-config', default=None, help='Design sei stati esportato dal GA, JSON')
    friction = p.add_mutually_exclusive_group()
    friction.add_argument('--motor-friction-profile', default=str(DEFAULT_MOTOR_FRICTION_PROFILE),
                   help='Robot01 Stribeck JSON; default for new trials, controller gains unchanged')
    friction.add_argument('--legacy-motor-friction', action='store_true',
                   help='Explicit historical reproduction without Stribeck')
    p.add_argument('--feedback-delay-samples', type=int, choices=(0,1), default=None,
                   help='lqr-motion: ritardo opzionale per confronto con Hinf (un campione)')
    p.add_argument('--path-file', default=str(ROOT/'config/paths/curve_test.json'), help='Percorso JSON per lqr-path')
    p.add_argument('--path-duration-s',type=float,default=None,help='Sovrascrive travel_duration_s del JSON per questa prova')
    p.add_argument('--path-obstacle', action='store_true',
                   help='Oggetto visibile al sensore lungo il path; non collidente per default')
    p.add_argument('--path-obstacle-progress', type=float, default=.65,
                   help='Posizione oggetto come frazione della lunghezza del path (0,1]')
    p.add_argument('--path-obstacle-lateral-m', type=float, default=0.,
                   help='Offset rispetto al path: positivo a sinistra [m]')
    p.add_argument('--path-obstacle-collision', action='store_true',
                   help='Abilita collisioni fisiche dell oggetto; nessun evitamento automatico')
    p.add_argument('--path-obstacle-width-m', type=float, default=.20)
    p.add_argument('--path-obstacle-depth-m', type=float, default=.04)
    p.add_argument('--path-obstacle-height-m', type=float, default=.20)
    p.add_argument('--ultrasonic-stop-distance-m', type=float, default=None,
                   help='Arresto latched del percorso con LQR attivo alla soglia sensore [m]')
    p.add_argument('--move-distance-m', type=float, default=.30, help='Distanza rettilinea relativa al rilascio')
    p.add_argument('--move-start-s', type=float, default=3., help='Inizio movimento dopo il rilascio')
    p.add_argument('--move-duration-s', type=float, default=6., help='Durata del riferimento quintico')
    p.add_argument('--motion-profile', choices=('straight', 'real-path'), default='straight',
                   help='In lqr-motion: retta oppure sequenza temporizzata del path reale')
    p.add_argument('--real-path-config', default=str(ROOT/'models/lqr/hardware_path_simple_40cm_28p5s_20261019.txt'),
                   help='Configurazione da cui leggere distanze, tempi e limite PWM della sequenza')
    for (name, default) in [('kp', 83), ('kd', 3.74), ('kv', 8), ('kx', 2), ('max-pwm', None), ('max-pitch-offset-deg', 3), ('filter-tau-s', 0.5), ('duration', 40), ('initial-pitch-deg', 0), ('desired-velocity', 0), ('pitch-limit-deg', 30)]:
        p.add_argument('--' + name, type=float, default=default)
    p.add_argument('--release-pitch-deg', type=float, default=None)
    p.add_argument('--disturbance-torque-nm', type=float, default=0,
                   help='Gradino di coppia esterna sul pitch, 0 disabilita')
    p.add_argument('--disturbance-force-y-n', type=float, default=0,
                   help='Forza esterna world +Y al COM del telaio [N]; non una coppia pitch')
    p.add_argument('--disturbance-start-s', type=float, default=5,
                   help='Secondi dopo il rilascio')
    p.add_argument('--disturbance-duration-s', type=float, default=0.5,
                   help='Durata del gradino prima della rimozione')
    p.add_argument('--disturbance-period-s', type=float, default=None,
                   help='Ripete forza/coppia ogni N secondi; default singolo impulso')
    p.add_argument('--battery-voltage-v', type=float, default=None,
                   help='Tensione iniziale imposta; default V_nom XML')
    p.add_argument('--battery-final-voltage-v', type=float, default=None,
                   help='Tensione finale della rampa durante la fase libera; nessun modello SOC')
    p.add_argument('--battery-discharge',action='store_true',help='Modello SOC con consumi elettronica e motori')
    p.add_argument('--battery-config',default=str(ROOT/'config/mujoco_battery_3s_3000mah.json'))
    p.add_argument('--battery-initial-soc',type=float,default=1.)
    p.add_argument('--electronics-power-w',type=float,default=None,help='Budget elettronica visto dal pacco')
    p.add_argument('--tag', default='balance_pd')
    a = p.parse_args(argv)
    if a.legacy_motor_friction:
        a.motor_friction_profile = None
    if a.lqr_config is not None and a.mode not in ('lqr-motion','lqr-path'):
        p.error('--lqr-config richiede lqr-motion oppure lqr-path')
    if a.mode == 'hinf':
        tokens = argv if argv is not None else sys.argv[1:]
        if any(token.split('=')[0] == '--move-distance-m' for token in tokens) and a.move_distance_m != 0:
            p.error('Hinf supporta solo stabilizzazione: distanza zero')
        if a.desired_velocity != 0:
            p.error('Hinf supporta solo stabilizzazione: velocita zero')
        if a.feedback_delay_samples not in (None,1):
            p.error('Il controllore Hinf esportato richiede un campione di ritardo')
        a.move_distance_m, a.move_start_s, a.move_duration_s = 0., 0., 1.
    elif a.feedback_delay_samples is not None and a.mode not in ('lqr-motion','lqr-path'):
        p.error('--feedback-delay-samples richiede lqr-motion, lqr-path oppure hinf')
    sequence = a.motion_profile == 'real-path'
    sequence_settings = None
    if sequence:
        if a.mode != 'lqr-motion':
            p.error('--motion-profile real-path richiede lqr-motion')
        from motion_lqr import RealPathReference
        try:
            sequence_settings = RealPathReference.read_settings(a.real_path_config)
        except (OSError, ValueError, KeyError) as error:
            p.error('Configurazione sequenza non valida: '+str(error))
        total = (sequence_settings['initial_hold_s']+2*sequence_settings['leg_duration_s']
                 +2*sequence_settings['between_hold_s']+sequence_settings['turn_duration_s']
                 +sequence_settings['final_hold_s'])
        if a.duration < total+5:
            p.error('Prevedere almeno 5 s di mantenimento dopo la sequenza reale')
    if a.max_pwm is None:
        a.max_pwm = (sequence_settings['wheel_pwm_cap'] if sequence else
                     (15.0 if a.mode in ('pd', 'observe', 'hinf') else 75.0))
    if a.mode == 'hinf':
        from hinf_controller import DiscreteHinf
        try:
            DiscreteHinf(a.hinf_config)
        except (OSError, ValueError, KeyError, TypeError) as error:
            p.error('Configurazione Hinf non valida: '+str(error))
    if (a.path_obstacle and a.mode != 'lqr-path') or (a.path_obstacle_collision and not a.path_obstacle):
        p.error('--path-obstacle richiede lqr-path; collisioni richiedono --path-obstacle')
    if a.path_obstacle and a.ultrasonic_test_distance_m is not None:
        p.error('Scegliere ostacolo sul path oppure parete frontale di test')
    if not math.isfinite(a.path_obstacle_progress) or not 0 < a.path_obstacle_progress <= 1:
        p.error('Progressione ostacolo in (0,1] richiesta')
    if any(not math.isfinite(v) or v <= 0 for v in (a.path_obstacle_width_m,
            a.path_obstacle_depth_m, a.path_obstacle_height_m)):
        p.error('Dimensioni ostacolo positive e finite richieste')
    if a.ultrasonic_stop_distance_m is not None and (a.mode != 'lqr-path' or
            not math.isfinite(a.ultrasonic_stop_distance_m) or a.ultrasonic_stop_distance_m <= 0):
        p.error('Soglia arresto positiva e finita, solo in lqr-path')
    if a.ultrasonic_test_distance_m is not None and (not math.isfinite(a.ultrasonic_test_distance_m) or a.ultrasonic_test_distance_m <= 0):
        p.error('Distanza ostacolo positiva e finita richiesta')
    if not all((math.isfinite(v) for v in vars(a).values() if isinstance(v, float))):
        p.error('Numeri finiti richiesti')
    if a.duration <= 0 or not 0 <= a.max_pwm <= 100 or a.max_pitch_offset_deg < 0 or (a.filter_tau_s < 0) or (a.pitch_limit_deg <= 0):
        p.error('Durata positiva, PWM 0..100, limiti validi richiesti')
    if abs(a.initial_pitch_deg) > 5 or (a.release_pitch_deg is not None and abs(a.release_pitch_deg) > 5):
        p.error('Postura sostenuta entro +/-5°')
    if a.disturbance_start_s < 0 or a.disturbance_duration_s <= 0:
        p.error('Inizio disturbo non negativo e durata positiva richiesti')
    if a.disturbance_period_s is not None and a.disturbance_period_s <= a.disturbance_duration_s:
        p.error('Il periodo dei disturbi deve superare la durata di ciascun impulso')
    if (a.disturbance_torque_nm != 0 or a.disturbance_force_y_n != 0) and (a.disturbance_duration_s < .01 or
            a.disturbance_start_s + a.disturbance_duration_s > a.duration):
        p.error('Disturbo di almeno 10 ms e interamente entro la durata libera')
    if any(v is not None and v <= 0 for v in (a.battery_voltage_v,a.battery_final_voltage_v)):
        p.error('Tensioni batteria positive richieste')
    if not 0<=a.battery_initial_soc<=1 or (a.electronics_power_w is not None and a.electronics_power_w<0):
        p.error('SOC 0..1 e potenza elettronica non negativa richiesti')
    if a.battery_discharge and (a.battery_voltage_v is not None or a.battery_final_voltage_v is not None):
        p.error('Scarica SOC e tensione imposta sono modalità alternative')
    if a.electronics_power_w is not None and not a.battery_discharge:
        p.error('--electronics-power-w richiede --battery-discharge')
    if a.sensor_seed<0 or (a.encoder_counts_per_rev is not None and a.encoder_counts_per_rev<=0):
        p.error('Seed non negativo e conteggi/giro positivi richiesti')
    if a.encoder_counts_per_rev is not None and a.encoder_model!='motor':
        p.error('--encoder-counts-per-rev richiede --encoder-model motor')
    if a.move_start_s < 0 or a.move_duration_s <= 0:
        p.error('Inizio movimento non negativo e durata positiva richiesti')
    if a.path_duration_s is not None and (a.path_duration_s <= 0 or a.mode != 'lqr-path'):
        p.error('--path-duration-s positivo e modalita lqr-path richiesti')
    if a.mode == 'lqr-motion':
        if a.desired_velocity != 0:
            p.error('lqr-motion genera la propria velocità; --desired-velocity deve essere zero')
        if not sequence and a.move_distance_m != 0 and a.move_start_s+a.move_duration_s+5 > a.duration:
            p.error('Prevedere almeno 5 s di mantenimento dopo il movimento')
        if not sequence and abs(a.move_distance_m)/a.move_duration_s*1.875 > .15:
            p.error('Primo test locale: velocità massima del riferimento <=0.15 m/s')
    if a.mode == 'lqr-path':
        from path_reference import PathPlan
        try:
            plan=PathPlan.from_file(a.path_file,a.path_duration_s)
        except (ValueError, KeyError, TypeError, OSError) as error:
            p.error('Path non valido: '+str(error))
        if a.desired_velocity != 0:
            p.error('lqr-path genera la propria velocita; --desired-velocity deve essere zero')
    if not a.tag or any((c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in a.tag)):
        p.error('Tag: lettere, numeri, _ o -')
    return a

def run(args, logdir=None, *, design_override=None, scenario=None):
    path_enabled=args.mode=='lqr-path'
    sequence_enabled=args.mode=='lqr-motion' and args.motion_profile=='real-path'
    plan=None
    if path_enabled:
        from path_reference import PathPlan, PathReference, draw_ground_path
        plan=PathPlan.from_file(args.path_file,args.path_duration_s)
    obstacle = (dict(plan.obstacle_pose(args.path_obstacle_progress,args.path_obstacle_lateral_m),
                     collision=args.path_obstacle_collision,
                     size_m=[args.path_obstacle_width_m,args.path_obstacle_depth_m,args.path_obstacle_height_m])
                if args.path_obstacle else None)
    robot = MujocoRobot(ultrasonic_test_distance_m=args.ultrasonic_test_distance_m,
                        ultrasonic_config=args.ultrasonic_config, path_obstacle=obstacle)
    if args.ultrasonic_stop_distance_m is not None and not (
            robot.ultrasonic_model.minimum <= args.ultrasonic_stop_distance_m <= robot.ultrasonic_model.maximum):
        raise ValueError('Soglia arresto fuori dalla portata del sensore configurato')
    # An exported GA design freezes the physical motor profile as well as Ad/Bd.
    frozen_design = design_override
    if frozen_design is None and args.lqr_config is not None:
        from ga_lqr import load_design
        frozen_design = load_design(args.lqr_config)
    if frozen_design is not None and 'wheel_profiles' in frozen_design:
        from wheel_dynamics import configure_wheels
        robot.wheel_dynamics = configure_wheels(robot.m, frozen_design['wheel_profiles'])
        mujoco.mj_setConst(robot.m, robot.d)
        mujoco.mj_forward(robot.m, robot.d)
    if getattr(args, 'motor_friction_profile', None):
        robot.configure_motor_friction(json.loads(Path(args.motor_friction_profile).read_text()))
    obstacle_contact_steps = 0
    battery = None
    battery_load = None
    if args.battery_discharge:
        wheel_dofs=[int(robot.m.joint(j).dofadr[0]) for j in ('perno_sx','perno_dx')]
        overrides={} if args.electronics_power_w is None else dict(electronics_power_w=args.electronics_power_w)
        battery=BatteryPack.from_file(args.battery_config,args.battery_initial_soc,
            robot.stall_torque/robot.nominal_voltage,robot.m.dof_damping[wheel_dofs],overrides)
        print('Batteria SOC 3S: riferimento provvisorio; elettronica '
              +str(battery.electronics_w)+' W ipotizzati, correnti motori inferite.',flush=True)
        try:
            battery_load=battery.load([0,0],[0,0])
            robot.set_supply_voltage(battery_load['battery_voltage_v'])
        except BatteryDepleted as error:
            print('Avvio annullato: '+str(error),flush=True)
            return 1
    battery_enabled = args.battery_voltage_v is not None or args.battery_final_voltage_v is not None
    battery_initial = args.battery_voltage_v if args.battery_voltage_v is not None else robot.nominal_voltage
    battery_final = args.battery_final_voltage_v if args.battery_final_voltage_v is not None else battery_initial
    def battery_voltage_at(free_time):
        fraction = max(0.,min(1.,free_time/args.duration))
        return battery_initial + (battery_final-battery_initial)*fraction
    if battery_enabled:
        robot.set_supply_voltage(battery_initial)
    if scenario is not None:
        if args.mode not in ('lqr-motion','lqr-path') or battery_enabled or battery is not None:
            raise ValueError('Scenari GA: solo LQR con alimentazione nominale fissa')
        from ga_lqr import apply_scenario
        scenario = apply_scenario(robot, scenario)
    noise_rng = np.random.default_rng(scenario.get('encoder_noise_seed',scenario['seed'])) if scenario is not None else None
    previous_encoder_noise = np.zeros(2)
    encoder_noise_velocity = np.zeros(2)
    dt = 0.01
    n = round(dt / robot.m.opt.timestep)
    if not math.isclose(n * robot.m.opt.timestep, dt):
        raise ValueError('Timestep deve dividere 10 ms')
    config = ControllerConfig(kp=args.kp, kd=args.kd, kv=args.kv, kx=args.kx, max_pwm=args.max_pwm, max_pitch_offset_deg=args.max_pitch_offset_deg, filter_tau_s=args.filter_tau_s)
    body = robot.m.body('struttura_centrale')
    equilibrium = -math.degrees(math.atan2(body.ipos[1], body.ipos[2]))
    control = BalanceController(config, equilibrium)
    design = None
    hinf_enabled = args.mode == 'hinf'
    motion = args.mode in ('lqr-motion','lqr-path','hinf')
    if motion:
        from motion_lqr import MotionLqrController, StraightReference, RealPathReference, design_motion_lqr
        if design_override is not None or args.lqr_config is not None:
            from ga_lqr import validate_design, load_design
            if args.mode not in ('lqr-motion','lqr-path'):
                raise ValueError('Design esterno: solo LQR sei stati')
            design = (validate_design(design_override) if design_override is not None
                      else load_design(args.lqr_config))
            required_delay = design.get('ga',{}).get('feedback_delay_samples')
            if required_delay is not None:
                if args.feedback_delay_samples not in (None,required_delay):
                    raise ValueError('Ritardo diverso da quello di progetto GA')
                args.feedback_delay_samples = required_delay
            if not math.isclose(design['equilibrium_pitch_deg'],equilibrium,abs_tol=1e-8):
                raise ValueError('Equilibrio del design diverso dal modello')
        else:
            design = design_motion_lqr(dt)
        if sequence_enabled:
            reference = RealPathReference(design, RealPathReference.read_settings(args.real_path_config))
        else:
            reference = PathReference(design,plan) if path_enabled else StraightReference(design, args.move_distance_m, args.move_start_s, args.move_duration_s)
        if hinf_enabled:
            from hinf_controller import HinfController
            control = HinfController(config, design, reference, args.hinf_config)
            print('Hinf importato: ordine '+str(len(control.hinf.state))+', due PWM, ritardo 1 campione', flush=True)
        else:
            control = MotionLqrController(config, design, reference,
                feedback_delay_samples=args.feedback_delay_samples or 0)
            print('LQR movimento K PWM/rad (SX, DX): ' + str(design['K_pwm_percent']), flush=True)
    elif args.mode == 'lqr':
        from lqr_controller import LqrController, design_lqr
        design = design_lqr(dt)
        control = LqrController(config, design)
        print('LQR MuJoCo K PWM/rad: ' + str(design['K_pwm_percent']), flush=True)
    calibration = Calibration()
    angle_bias = gyro_bias = 0.0
    gyro_bias_xyz = np.zeros(3)
    reposition_steps = 100 if args.release_pitch_deg is not None else 0
    release_step = 220 + reposition_steps
    final_step = release_step + math.ceil(args.duration / dt)
    disturbance_enabled = args.disturbance_torque_nm != 0
    force_enabled = args.disturbance_force_y_n != 0
    disturbance_on = math.ceil(args.disturbance_start_s/dt-1e-9)
    disturbance_off = math.ceil((args.disturbance_start_s+args.disturbance_duration_s)/dt-1e-9)
    period_steps = (math.ceil(args.disturbance_period_s/dt-1e-9)
                    if args.disturbance_period_s is not None else None)
    free_steps = math.ceil(args.duration/dt)
    pulse_starts = (range(disturbance_on,free_steps,period_steps)
                    if period_steps is not None else [disturbance_on])
    pulse_windows = [(start,min(start+disturbance_off-disturbance_on,free_steps))
                     for start in pulse_starts if start < free_steps]
    def disturbance_active(step):
        return any(start <= step < end for start,end in pulse_windows)
    robot.start_support(args.initial_pitch_deg)
    sensor_config=None
    if args.imu_model!='ideal' or args.encoder_model!='ideal':
        sensor_config=robot.configure_sensors(args.imu_model,args.encoder_model,args.sensor_config,
                                            args.sensor_seed,args.encoder_counts_per_rev,
                                            noise_scales=scenario.get('imu_noise_scale') if scenario is not None else None)
    folder = Path(logdir) if logdir is not None else ROOT / 'logs/mujoco/runs' / args.tag
    folder.mkdir(parents=True, exist_ok=True)
    base = folder / (datetime.now().strftime(args.tag + '_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8])
    sources = [XML_PATH, Path(__file__), ROOT / 'src_python/balance_controller.py', ROOT / 'src_python/mujoco_robot.py', *sorted((ROOT / 'models/meshes').glob('*.STL'))]
    sources.append(ROOT/'src_python/viewer_hud.py')
    metadata = dict(schema_version=3, configuration=vars(args), controller=asdict(config), controller_source='independent Python PD + encoder feedback', equilibrium_pitch_deg=equilibrium, motor_model='PWM stall torque + effective velocity damping; no additional back-emf', mujoco_version=mujoco.__version__, physics_dt_s=float(robot.m.opt.timestep), control_dt_s=dt, support_model='temporary soft weld; motors off until release', calibration_reference='known initial support angle; IMU bias measured there', physical_parameters=dict(body_mass_kg=robot.m.body_mass.tolist(), body_com_local_m=robot.m.body_ipos.tolist(), body_inertia_principal_kg_m2=robot.m.body_inertia.tolist(), body_inertia_quat=robot.m.body_iquat.tolist(), joint_armature_kg_m2=robot.m.dof_armature.tolist(), joint_damping_Nm_s_rad=robot.m.dof_damping.tolist(), joint_frictionloss_Nm=robot.m.dof_frictionloss.tolist(), geom_friction=robot.m.geom_friction.tolist(), motor_stall_torque_Nm=robot.stall_torque, voltage_V=robot.voltage), sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    if robot.motor_friction_profile is not None:
        profile_path = Path(args.motor_friction_profile).resolve()
        metadata['motor_model'] = 'PWM torque + measured wheel/direction Stribeck friction; updated every physics step'
        metadata['motor_friction_profile'] = robot.motor_friction_profile
        metadata['runtime_motor_override'] = True
        for source in (profile_path, ROOT/'src_python/stribeck_motor.py'):
            key = str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) else str(source)
            metadata['sha256'][key] = hashlib.sha256(source.read_bytes()).hexdigest()
    if design is not None:
        metadata.update(schema_version=5, controller_source='MuJoCo finite-difference discrete LQR', lqr_design=design,
                        yaw_estimation='unavailable; rho/rho_dot NaN, excluded from feedback')
        for path in (ROOT/'src_python/lqr_controller.py', ROOT/'models/lqr/matrices_common_pwm_corrected.json'):
            metadata['sha256'][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    if scenario is not None:
        metadata['ga_scenario'] = scenario
        metadata['physical_parameters']['actuator_gear'] = robot.m.actuator_gear.tolist()
    if args.lqr_config is not None or design_override is not None:
        metadata['lqr_design_origin'] = args.lqr_config or 'frozen design supplied by GA evaluator'
        source = ROOT/'src_python/ga_lqr.py'
        metadata['sha256'][str(source.relative_to(ROOT))] = hashlib.sha256(source.read_bytes()).hexdigest()
    if disturbance_enabled:
        metadata.update(schema_version=6, disturbance=dict(type='finite-duration external torque step',
                        body='struttura_centrale', axis='world -X; positive increases psi',
                        torque_nm=args.disturbance_torque_nm, actual_start_free_s=disturbance_on*dt,
                        actual_end_free_s=disturbance_off*dt, changes_motor_reference=False))
    if force_enabled:
        metadata['external_force'] = dict(body='struttura_centrale',axis='world +Y',
            application_point='body center of mass',force_n=args.disturbance_force_y_n,
            actual_start_free_s=disturbance_on*dt,actual_end_free_s=disturbance_off*dt,
            note='Not automatically equivalent to Simscape force at Skeleton F1 in AttachedFrame')
    if disturbance_enabled or force_enabled:
        schedule = dict(period_s=period_steps*dt if period_steps is not None else None,
            windows_free_s=[[start*dt,end*dt] for start,end in pulse_windows],
            application='constant amplitude within each window; zero outside')
        metadata['disturbance_schedule'] = schedule
    if battery_enabled:
        metadata.update(schema_version=7,
            battery_model=dict(type='prescribed voltage ramp; no SOC, battery current or runtime estimate',
                initial_voltage_v=battery_initial, final_voltage_v=battery_final,
                ramp_duration_free_s=args.duration, nominal_motor_reference_v=robot.nominal_voltage,
                lqr_design_voltage_v=robot.nominal_voltage if design is not None else None,
                gain_policy='fixed nominal LQR; no voltage compensation',
                motor_scaling='tau=-nominal_stall_torque*(V_supply/V_nom)*duty; damping unchanged'))
    if battery is not None:
        metadata.update(schema_version=9,battery_model=dict(type='SOC + static OCV/internal resistance + load consumption',
            configuration=battery.config,initial_soc=args.battery_initial_soc,
            capacity_ah=battery.capacity_ah,pack_resistance_ohm=battery.resistance,
            effective_motor_k=battery.motor_k.tolist(),effective_motor_resistance_ohm=battery.motor_r.tolist(),
            lqr_design_voltage_v=robot.nominal_voltage if design is not None else None,
            warning_voltage_v=10.5,warning_is_not_cutoff=True,
            motor_model_changed=False,regenerative_charging=False))
        source=Path(args.battery_config)
        metadata['sha256'][str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
        source=ROOT/'src_python/battery_model.py'
        metadata['sha256'][str(source.relative_to(ROOT))]=hashlib.sha256(source.read_bytes()).hexdigest()
    if sensor_config is not None:
        metadata.update(schema_version=10,sensor_models=dict(configuration=sensor_config,
            imu=args.imu_model,encoder=args.encoder_model,seed=args.sensor_seed,
            counts_per_wheel_rev=robot.encoder_model.counts_per_rev if robot.encoder_model is not None else None,
            encoder_cpr_hardware_validated=False,track_width_m=robot.wheel_track_m,
            encoder_reset_phase='calibration_end; no reset at release',
            gyro_frame='imu site Y=-body X for pitch; Z must be transformed for world yaw',
            note='Optional hardware-like approximation; no exact DLPF transfer function or real bias calibration'))
        for source in (Path(args.sensor_config),ROOT/'src_python/sensor_models.py'):
            metadata['sha256'][str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
    if motion:
        metadata.update(schema_version=11,
            controller_source='six-state two-input MuJoCo LQR + quintic straight reference',
            yaw_estimation='pitch-compensated gyro + differential encoder complementary filter',
            motion=dict(distance_m=args.move_distance_m, start_free_s=args.move_start_s,
                duration_s=args.move_duration_s, heading_ref_rad=0., frame='local to release; forward +Y',
                feedback='encoder angles/rates and calibrated IMU only; no MuJoCo pose',
                reference='quintic distance with quasi-static linear-model pitch/duty feedforward',
                saturation_policy='common PWM priority; differential uses remaining headroom',
                tolerances=dict(position_m=.01, lateral_m=.01, yaw_deg=2., speed_m_s=.02, pitch_deg=.3)))
        source=ROOT/'src_python/motion_lqr.py'
        metadata['sha256'][str(source.relative_to(ROOT))]=hashlib.sha256(source.read_bytes()).hexdigest()
        metadata['feedback_delay_samples'] = 1 if hinf_enabled else control.feedback_delay_samples
    if hinf_enabled:
        from hinf_controller import STATE_TRANSFORM
        metadata.update(schema_version=14,
            controller_source='imported Simscape discrete Hinf; stationary hold only',
            hinf_design=control.hinf.export, plant_linearization=metadata.pop('lqr_design'),
            hinf_exported_pwm_limit_percent=100*control.hinf.limit,
            hinf_runtime_pwm_limit_percent=config.max_pwm,
            hinf_limit_overridden=not math.isclose(config.max_pwm,100*control.hinf.limit),
            hinf_state_transform=STATE_TRANSFORM.tolist(),
            hinf_telemetry='hinf_feedback_0..5: delayed transformed errors used for u(k); hinf_xc_*: internal state after update',
            input_order_mapping='Hinf [PWM_R,PWM_L] -> MuJoCo [duty_sx=PWM_L,duty_dx=PWM_R]')
        metadata['motion'].update(reference='constant position/yaw and equilibrium pitch',
            distance_m=0.,start_free_s=0.,duration_s=0.)
        for source in (Path(args.hinf_config),ROOT/'src_python/hinf_controller.py'):
            metadata['sha256'][str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
    if sequence_enabled:
        metadata.update(schema_version=13,
            controller_source='six-state MuJoCo LQR + hardware timed return sequence',
            real_path_sequence=dict(settings=control.reference.settings,
                duration_s=control.reference.schedule_duration,
                longitudinal_reference='rebased on sensor odometry at each phase entry',
                comparison='Same schedule as hardware; gains/feedforward/estimator remain MuJoCo implementations',
                ideal_final_pose_m=[0., 0.]))
        metadata['motion'].update(profile='real-path', distance_m=2*control.reference.distance,
            start_free_s=control.reference.start, duration_s=control.reference.schedule_duration,
            heading_ref_rad=control.reference.turn,
            reference='hardware timed forward/pivot/forward sequence; no arrival gates')
        source=Path(args.real_path_config)
        metadata['sha256'][str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
    if path_enabled:
        metadata.update(schema_version=12,
            controller_source='sensor pose guidance + six-state two-input MuJoCo LQR',
            path_reference=dict(configuration=plan.config, length_m=plan.length,
                max_nominal_speed_m_s=float(np.max(plan.samples['speed'])),
                max_nominal_yaw_rate_deg_s=float(np.degrees(np.max(np.abs(plan.samples['yaw_rate'])))),
                geometry='C2 clamped cubic spline through XY points; quintic time parameter',
                guidance='bounded heading correction from estimated lateral pose error',
                visualization='viewer.user_scn decorative geoms only; no model/contact changes',
                tolerances=dict(final_estimated_pose_m=.02,final_true_pose_m=.025,final_yaw_deg=3,
                    max_tracking_true_pose_m=.04,pitch_deg=.3,speed_m_s=.02)))
        metadata['motion'].update(distance_m=plan.length,start_free_s=plan.start,duration_s=plan.duration,
            heading_ref_rad=float(plan.heading_grid[-1]),
            reference='XY spline with local longitudinal/yaw model feedforward and lateral guidance')
        for source in (Path(args.path_file),ROOT/'src_python/path_reference.py'):
            metadata['sha256'][str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
    metadata['ultrasonic_model'] = dict(type='HC-SR04 sampled geometric cone',
        configuration=robot.ultrasonic_model.config,
        half_angle_deg=robot.ultrasonic_model.half_angle_deg,
        ray_count=len(robot.ultrasonic_model.directions),
        axial_sensor='ultrasonic_range', site='ultrasonic_site',
        feedback_enabled=args.ultrasonic_stop_distance_m is not None, invalid_distance='NaN',
        test_wall_distance_initial_m=args.ultrasonic_test_distance_m,
        limitations='No noise, acoustic reflections, material response or flight delay; nearest sampled visible surface, echo width derived not TTL simulation')
    if obstacle is not None:
        metadata['path_obstacle'] = dict(obstacle,
            feedback_enabled=False, geom_id=robot.m.geom('path_obstacle_geom').id)
    if args.ultrasonic_stop_distance_m is not None:
        metadata['ultrasonic_stop'] = dict(threshold_m=args.ultrasonic_stop_distance_m,
            policy='Latch estimated position/yaw hold; zero velocity reference, LQR stays active',
            automatic_restart=False, distance_origin='Sensor site, not robot center or body clearance')
    for source in (ROOT/'src_python/ultrasonic_model.py', Path(args.ultrasonic_config)):
        metadata['sha256'][str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
    fields = ['t_s', 't_free_s', 'phase', 'pitch_true_deg', 'pitch_acc_deg', 'pitch_est_deg', 'pitch_target_deg', 'gyro_deg_s', 'accel_g', 'velocity_encoder_m_s', 'position_encoder_m', 'forward_y_m', 'velocity_world_y_m_s', 'pwm_requested_percent', 'pwm_percent', 'pwm_applied_percent', 'duty_normalized', 'tau_sx_Nm', 'tau_dx_Nm', 'vel_sx_rad_s', 'vel_dx_rad_s', 'saturated', 'support_active']
    fields += ['ultrasonic_ray_distance_m','ultrasonic_distance_m','ultrasonic_valid',
               'ultrasonic_echo_us','ultrasonic_sample_time_s','ultrasonic_cone_distance_m',
               'ultrasonic_hit_geom_id','ultrasonic_hit_angle_deg','ultrasonic_hit_rays']
    if design is not None:
        fields += ['lqr_psi_error_rad', 'lqr_theta_dot_error_rad_s', 'lqr_psi_dot_rad_s',
                   'x_theta_rad', 'x_psi_rad', 'x_rho_rad', 'x_theta_dot_rad_s',
                   'x_psi_dot_rad_s', 'x_rho_dot_rad_s', 'yaw_valid']
    if disturbance_enabled:
        fields += ['disturbance_pitch_torque_Nm']
    if force_enabled:
        fields += ['disturbance_force_world_y_N']
    if battery_enabled:
        fields += ['battery_voltage_v']
    if battery is not None:
        fields += ['battery_voltage_v','battery_ocv_v','battery_soc','battery_current_A',
                   'battery_consumed_mah','battery_remaining_mah',
                   'electronics_current_A','motor_sx_winding_current_A','motor_dx_winding_current_A',
                   'motor_sx_battery_current_A','motor_dx_battery_current_A']
    if robot.imu_model is not None:
        fields += ['imu_ax_raw','imu_ay_raw','imu_az_raw','imu_gx_raw','imu_gy_raw','imu_gz_raw','imu_saturated']
    if robot.encoder_model is not None:
        fields += ['encoder_sx_count','encoder_dx_count']
    if motion:
        fields += ['pwm_sx_requested_percent','pwm_dx_requested_percent',
            'pwm_sx_applied_percent','pwm_dx_applied_percent',
            'pwm_common_requested_percent','pwm_differential_requested_percent',
            'distance_ref_m','velocity_ref_m_s','acceleration_ref_m_s2','yaw_ref_rad',
            'distance_error_m','yaw_error_rad','yaw_encoder_rad','pose_est_x_m','pose_est_y_m',
            'reference_duty_sx','reference_duty_dx',
            'error_theta_rad','error_psi_rad','error_rho_rad','error_theta_dot_rad_s',
            'error_psi_dot_rad_s','error_rho_dot_rad_s',
            'pose_true_x_m','pose_true_y_m','yaw_true_rad','velocity_world_forward_m_s','velocity_world_lateral_m_s',
            'duty_sx_normalized','duty_dx_normalized',
            'gyro_x_cal_rad_s','gyro_y_cal_rad_s','gyro_z_cal_rad_s',
            'encoder_pos_sx_rad','encoder_pos_dx_rad']
    if sequence_enabled:
        fields += ['sequence_phase', 'sequence_finished', 'planned_y_m',
                   'velocity_world_x_m_s', 'velocity_body_forward_m_s']
    if hinf_enabled:
        fields += ['hinf_feedback_'+str(i) for i in range(6)]
        fields += ['hinf_xc_'+str(i) for i in range(len(control.hinf.state))]
    if path_enabled:
        fields += ['ultrasonic_stop_active','path_reference_time_s']
        fields += ['velocity_world_x_m_s','path_ref_x_m','path_ref_y_m','path_nominal_yaw_rad','path_yaw_rate_ref_rad_s',
                   'path_error_forward_m','path_error_lateral_m','path_pose_error_m',
                   'path_heading_correction_rad','path_distance_command_m','path_progress','path_true_pose_error_m']
    rows = []
    reason = 'durata completata'
    success = True
    actual_release = None
    release_pose = None
    finished = False

    def persist():
        nonlocal finished
        if finished:
            return
        robot.set_motors(0, 0)
        robot.set_pitch_disturbance(0)
        robot.d.xfrc_applied[robot.body_id,1] = 0.
        if battery_enabled:
            elapsed = float(robot.d.time)-actual_release if actual_release is not None else 0.
            robot.set_supply_voltage(battery_voltage_at(elapsed))
        if battery is not None:
            try:
                stopped_load=battery.load([0,0],[float(robot.d.qvel[j]) for j in wheel_dofs])
                robot.set_supply_voltage(stopped_load['battery_voltage_v'])
            except BatteryDepleted:
                pass  # terminal/cutoff state: still finalize logs and zero motors
        mujoco.mj_forward(robot.m, robot.d)
        free = [r for r in rows if r['phase'] == 'free']
        tail = [r for r in free if r['t_free_s'] >= free[-1]['t_free_s'] - 5] if free else []
        status = dict(success=success, stop_reason=reason, free_time_s=max(0, float(robot.d.time) - actual_release) if actual_release is not None else 0, final_ctrl=robot.d.ctrl.tolist(), samples=len(rows))
        if force_enabled:
            status['final_disturbance_force_y_N'] = float(robot.d.xfrc_applied[robot.body_id,1])
        if obstacle is not None:
            status['obstacle_contact_steps'] = obstacle_contact_steps
        if args.ultrasonic_stop_distance_m is not None:
            request = control.reference.stop_request
            status.update(ultrasonic_stop_triggered=request is not None,
                ultrasonic_stop_trigger=request)
            if request is not None:
                after = [r for r in free if r['t_free_s'] >= request['time_s'] and r['ultrasonic_valid']]
                status['minimum_valid_range_after_stop_m'] = min(
                    (r['ultrasonic_distance_m'] for r in after), default=None)
        if battery_enabled:
            status['final_battery_voltage_v'] = robot.voltage
        if battery is not None:
            status.update(battery.summary(),final_battery_voltage_v=robot.voltage)
        if disturbance_enabled or force_enabled:
            affected = [r for r in free if r['t_free_s'] >= disturbance_on*dt]
            recoveries = []
            for index,(start,end) in enumerate(pulse_windows):
                next_start = pulse_windows[index+1][0] if index+1<len(pulse_windows) else free_steps
                post = [r for r in affected if end*dt <= r['t_free_s'] < next_start*dt]
                consecutive = 0
                recovery = None
                for row in post:
                    stable = (abs(row['pitch_true_deg']-(row['pitch_target_deg'] if motion else equilibrium))<.3 and
                              abs(row['velocity_encoder_m_s']-(row['velocity_ref_m_s'] if motion else args.desired_velocity))<.02)
                    consecutive = consecutive+1 if stable else 0
                    if consecutive >= 100:
                        recovery = row['t_free_s']-99*dt-end*dt
                        break
                recoveries.append(dict(start_free_s=start*dt,end_free_s=end*dt,
                    recovered=bool(success and recovery is not None),recovery_time_s=recovery))
            recovery = recoveries[-1]['recovery_time_s'] if recoveries else None
            status.update(disturbance_recovered=bool(success and recovery is not None),
                          disturbance_recovery_time_s=recovery,
                          disturbance_pulses=recoveries,
                          all_disturbance_pulses_recovered=bool(recoveries and all(p['recovered'] for p in recoveries)),
                          disturbance_peak_pitch_error_deg=max((abs(r['pitch_true_deg']-equilibrium)
                              for r in affected),default=None),
                          disturbance_peak_pwm_percent=max((abs(r['pwm_applied_percent'])
                              for r in affected),default=None),
                          final_disturbance_torque_Nm=float(-robot.d.xfrc_applied[robot.body_id,3]))
        if tail:
            status.update(final_pitch_deg=free[-1]['pitch_true_deg'], final_velocity_m_s=free[-1]['velocity_encoder_m_s'], final_forward_y_m=free[-1]['forward_y_m'], tail_max_pitch_error_deg=max((abs(r['pitch_true_deg'] - equilibrium) for r in tail)), tail_max_velocity_m_s=max((abs(r['velocity_encoder_m_s']) for r in tail)), tail_position_range_m=max((r['forward_y_m'] for r in tail)) - min((r['forward_y_m'] for r in tail)), max_abs_pwm_percent=max((abs(r['pwm_applied_percent']) for r in free)))
            status['settled'] = bool(success and len(tail) >= 490 and (status['tail_max_pitch_error_deg'] < 0.3) and (status['tail_max_velocity_m_s'] < 0.02) and (status['tail_position_range_m'] < 0.03))
        else:
            status['settled'] = False
        if sequence_enabled:
            from motion_lqr import wrap_angle
            status.update(sequence_completed=False, sequence_finished=False,
                          sequence_duration_s=control.reference.schedule_duration)
            if tail:
                status.update(
                    sequence_finished=bool(free[-1]['sequence_finished']),
                    final_pose_est_m=[free[-1]['pose_est_x_m'], free[-1]['pose_est_y_m']],
                    final_pose_true_m=[free[-1]['pose_true_x_m'], free[-1]['pose_true_y_m']],
                    tail_max_distance_error_m=max(abs(r['distance_error_m']) for r in tail),
                    tail_max_world_return_error_m=max(math.hypot(r['pose_true_x_m'],r['pose_true_y_m']) for r in tail),
                    tail_max_yaw_error_deg=max(abs(math.degrees(wrap_angle(r['yaw_true_rad']-control.reference.turn))) for r in tail),
                    tail_max_world_speed_m_s=max(math.hypot(r['velocity_world_x_m_s'],r['velocity_world_forward_m_s']) for r in tail),
                    max_abs_pwm_percent=max(abs(r[n]) for r in free for n in ('pwm_sx_applied_percent','pwm_dx_applied_percent')))
                status['sequence_completed']=bool(success and status['sequence_finished'] and status['settled']
                    and tail[0]['t_free_s'] >= control.reference.schedule_duration-dt
                    and status['tail_max_world_return_error_m'] < .03
                    and status['tail_max_yaw_error_deg'] < 3 and status['tail_max_world_speed_m_s'] < .02)
        elif motion and tail and not path_enabled:
            from motion_lqr import wrap_angle
            final=free[-1]
            status.update(final_distance_est_m=final['position_encoder_m'],
                final_pose_est_m=[final['pose_est_x_m'],final['pose_est_y_m']],
                final_pose_true_m=[final['pose_true_x_m'],final['pose_true_y_m']],
                final_yaw_est_deg=math.degrees(final['x_rho_rad']),
                final_yaw_true_deg=math.degrees(final['yaw_true_rad']),
                tail_max_distance_error_m=max(abs(r['position_encoder_m']-args.move_distance_m) for r in tail),
                tail_max_world_distance_error_m=max(abs(r['pose_true_y_m']-args.move_distance_m) for r in tail),
                tail_max_lateral_error_m=max(abs(r['pose_true_x_m']) for r in tail),
                tail_max_yaw_error_deg=max(abs(math.degrees(wrap_angle(r['yaw_true_rad']))) for r in tail),
                tail_max_world_speed_m_s=max(abs(r['velocity_world_forward_m_s']) for r in tail),
                peak_pitch_error_deg=max(abs(r['pitch_true_deg']-equilibrium) for r in free))
            status['max_abs_pwm_percent']=max(abs(r[name]) for r in free
                for name in ('pwm_sx_applied_percent','pwm_dx_applied_percent'))
            status['motion_completed']=bool(status['settled'] and
                tail[-1]['t_free_s']>=(5 if args.move_distance_m == 0 else args.move_start_s+args.move_duration_s+5)-dt-1e-8 and
                status['tail_max_distance_error_m']<.01 and status['tail_max_world_distance_error_m']<.01 and
                status['tail_max_lateral_error_m']<.01 and status['tail_max_yaw_error_deg']<2 and
                status['tail_max_world_speed_m_s']<.02)
        elif motion and not path_enabled:
            status['motion_completed']=False
        if hinf_enabled:
            status['stabilization_completed'] = status.pop('motion_completed',False)
        if motion and free:
            status.update(pitch_error_rms_deg=float(np.sqrt(np.mean([
                (r['pitch_true_deg']-r['pitch_target_deg'])**2 for r in free]))),
                pwm_motor_rms_percent=float(np.sqrt(np.mean([
                    r[k]**2 for r in free for k in ('pwm_sx_applied_percent','pwm_dx_applied_percent')]))),
                saturation_fraction=float(np.mean([r['saturated'] for r in free])))
        if path_enabled:
            status['path_completed']=False
            if tail:
                from motion_lqr import wrap_angle
                goal=plan.points[-1]
                goal_yaw=float(plan.heading_grid[-1])
                tracking=[r for r in free if plan.start <= r['t_free_s'] <= plan.start+plan.duration]
                status.update(final_goal_xy_m=goal.tolist(),path_length_m=plan.length,
                    final_pose_est_m=[free[-1]['pose_est_x_m'],free[-1]['pose_est_y_m']],
                    final_pose_true_m=[free[-1]['pose_true_x_m'],free[-1]['pose_true_y_m']],
                    final_yaw_est_deg=math.degrees(free[-1]['x_rho_rad']),
                    final_yaw_true_deg=math.degrees(free[-1]['yaw_true_rad']),
                    tail_max_pose_error_est_m=max(math.hypot(r['pose_est_x_m']-goal[0],r['pose_est_y_m']-goal[1]) for r in tail),
                    tail_max_pose_error_true_m=max(math.hypot(r['pose_true_x_m']-goal[0],r['pose_true_y_m']-goal[1]) for r in tail),
                    tail_max_yaw_error_deg=max(abs(math.degrees(wrap_angle(r['yaw_true_rad']-goal_yaw))) for r in tail),
                    tail_max_world_speed_m_s=max(math.hypot(r['velocity_world_y_m_s'],r['velocity_world_x_m_s']) for r in tail),
                    tracking_max_pose_error_true_m=max((r['path_true_pose_error_m'] for r in tracking),default=None),
                    tracking_rms_pose_error_true_m=math.sqrt(np.mean([r['path_true_pose_error_m']**2 for r in tracking])) if tracking else None,
                    peak_pitch_error_deg=max(abs(r['pitch_true_deg']-equilibrium) for r in free),
                    max_abs_pwm_percent=max(abs(r[name]) for r in free for name in ('pwm_sx_applied_percent','pwm_dx_applied_percent')))
                status['path_completed']=bool(status['settled'] and
                control.reference.stop_request is None and
                    tail[-1]['t_free_s']>=plan.start+plan.duration+5-dt-1e-8 and
                    status['tail_max_pose_error_est_m']<.02 and status['tail_max_pose_error_true_m']<.025 and
                    status['tail_max_yaw_error_deg']<3 and status['tail_max_world_speed_m_s']<.02 and
                    status['tracking_max_pose_error_true_m'] is not None and status['tracking_max_pose_error_true_m']<.04)
        if args.ultrasonic_stop_distance_m is not None and control.reference.stop_request is not None:
            status['ultrasonic_hold_settled'] = bool(status['settled'] and obstacle_contact_steps == 0)
        base.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
        base.with_suffix('.status.log').write_text(json.dumps(status, indent=2) + '\n')
        print('Log: ' + str(base.with_suffix('.csv')) + '\n' + json.dumps(status, indent=2), flush=True)
        finished = True
    context = nullcontext(None) if args.headless else open_viewer(robot)
    try:
        with base.with_suffix('.csv').open('x', newline='') as stream, context as viewer:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            if viewer:
                viewer.cam.lookat[:] = [0, args.move_distance_m/2 if motion else 0, 0.065]
                viewer.cam.distance = max(.7, abs(args.move_distance_m)*2) if motion else .5
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -20
                if sequence_enabled:
                    viewer.cam.lookat[:] = [0, control.reference.distance/2, 0.065]
                    viewer.cam.distance = max(.8, control.reference.distance*2)
                if path_enabled:
                    points=plan.visual_points()
                    center=(points.min(axis=0)+points.max(axis=0))/2
                    viewer.cam.lookat[:2]=robot.d.xpos[robot.body_id,:2]+center
                    viewer.cam.distance=max(.8,float(np.linalg.norm(points.max(axis=0)-points.min(axis=0)))*1.7)
                    viewer.cam.elevation=-45
                    with viewer.lock():
                        draw_ground_path(viewer.user_scn,plan,robot.d.xpos[robot.body_id,:2])
                viewer.sync()
            wall = time.monotonic()
            previous_phase = None
            for i in range(final_step):
                if viewer and (not viewer.is_running()):
                    reason = 'finestra chiusa'
                    success = False
                    break
                phase = 'settling' if i < 20 else 'calibration' if i < 220 else 'reposition' if i < release_step else 'free'
                if phase != previous_phase:
                    print(f't={i * dt:.2f}s: {phase}', flush=True)
                    previous_phase = phase
                if i == 220:
                    (angle_bias, gyro_bias) = calibration.finish()
                    metadata.update(angle_bias_deg=angle_bias, gyro_bias_deg_s=gyro_bias)
                    if motion:
                        gyro_bias_xyz=calibration.gyro_bias_vector()
                        metadata['gyro_bias_xyz_rad_s']=gyro_bias_xyz.tolist()
                    control.reset(args.initial_pitch_deg)
                    if robot.encoder_model is not None:
                        robot.encoder_model.reset([float(robot.d.sensor(name).data[0])
                            for name in ('encoder_pos_sx','encoder_pos_dx')])
                if phase == 'reposition':
                    f = min(1, (i - 220 + 1) / 50)
                    s = f * f * (3 - 2 * f)
                    robot.move_support(args.initial_pitch_deg + s * (args.release_pitch_deg - args.initial_pitch_deg))
                if i == release_step:
                    robot.release()
                    actual_release = float(robot.d.time)
                    metadata.update(actual_release_s=actual_release, release_true_pitch_deg=robot.true_pitch_deg())
                    if motion:
                        release_pose=(robot.d.xpos[robot.body_id,:2].copy(),robot.true_yaw_rad())
                        metadata['release_world_pose']=[*release_pose[0].tolist(),release_pose[1]]
                        if obstacle is not None:
                            metadata['path_obstacle'].update(robot.place_path_obstacle(*release_pose))
                if battery_enabled:
                    robot.set_supply_voltage(battery_voltage_at((i-release_step)*dt))
                disturbance_torque = args.disturbance_torque_nm if (phase == 'free' and
                    disturbance_active(i-release_step)) else 0.
                robot.set_pitch_disturbance(disturbance_torque)
                disturbance_force = args.disturbance_force_y_n if (phase == 'free' and
                    disturbance_active(i-release_step)) else 0.
                robot.d.xfrc_applied[robot.body_id,1] = disturbance_force
                transition = phase == 'free' and (i-release_step == disturbance_on or
                    disturbance_active(i-release_step) != disturbance_active(i-release_step-1))
                if disturbance_enabled and transition:
                    print(f't libero={(i-release_step)*dt:.2f}s: disturbo {disturbance_torque:+.4f} Nm',flush=True)
                if force_enabled and transition:
                    print(f't libero={(i-release_step)*dt:.2f}s: forza telaio Y mondo '
                          f'{float(robot.d.xfrc_applied[robot.body_id,1]):+.4f} N al COM',flush=True)
                mujoco.mj_forward(robot.m, robot.d)
                (acc, gyro_raw) = robot.read_imu()
                accel_pitch = math.degrees(math.atan2(-acc[0], acc[2])) - angle_bias + args.initial_pitch_deg
                gyro = math.degrees(gyro_raw[1]) - gyro_bias
                (pos_l, vel_l, pos_r, vel_r) = robot.read_encoders()
                if scenario is not None and phase == 'free':
                    # Position noise enters both angle feedback and its filtered derivative.
                    if robot.encoder_model is None:
                        raise ValueError('Scenari GA: richiesto encoder motor')
                    sigma = scenario['encoder_noise_std_counts'] * 2*math.pi / robot.encoder_model.counts_per_rev
                    noise = noise_rng.normal(0., sigma, 2)
                    derivative = (noise-previous_encoder_noise)/dt
                    alpha = math.exp(-dt/robot.encoder_model.config['velocity_filter_tau_s'])
                    encoder_noise_velocity = alpha*encoder_noise_velocity+(1-alpha)*derivative
                    previous_encoder_noise = noise
                    pos_l, pos_r = pos_l+noise[0], pos_r+noise[1]
                    vel_l, vel_r = vel_l+encoder_noise_velocity[0], vel_r+encoder_noise_velocity[1]
                info = dict(pitch_est_deg=control.pitch, pitch_target_deg=equilibrium, velocity_encoder_m_s=0, position_encoder_m=0, pwm_requested_percent=0, pwm_percent=0, pwm_applied_percent=0, saturated=False)
                if phase == 'calibration':
                    calibration.add(acc, gyro_raw)
                if phase == 'reposition':
                    alpha = math.exp(-dt / config.filter_tau_s) if config.filter_tau_s else 0
                    control.pitch = alpha * (control.pitch + gyro * dt) + (1 - alpha) * accel_pitch
                if phase == 'free':
                    if args.ultrasonic_stop_distance_m is not None:
                        reading = robot.read_ultrasonic()
                        if reading['ultrasonic_valid'] and reading['ultrasonic_distance_m'] <= args.ultrasonic_stop_distance_m:
                            if control.reference.stop_request is None:
                                print(f"Arresto ultrasuoni: {reading['ultrasonic_distance_m']:.3f} m; LQR resta attivo", flush=True)
                            control.reference.request_stop((i-release_step)*dt, reading['ultrasonic_distance_m'])
                    extra = dict(wheel_positions=(pos_l,pos_r)) if design is not None else {}
                    if motion:
                        extra.update(gyro_xyz_rad_s=gyro_raw-gyro_bias_xyz,free_time_s=(i-release_step)*dt)
                    info = control.update(accel_pitch, gyro, vel_l, vel_r, dt, robot.true_pitch_deg() if args.sensor == 'ideal' else None, args.desired_velocity, **extra)
                true = robot.true_pitch_deg()
                finite = bool(np.isfinite(np.r_[acc, gyro_raw, robot.d.qpos, robot.d.qvel]).all())
                sensor_saturated=phase=='free' and robot.imu_model is not None and robot.imu_model.saturated
                bad = not finite or abs(true) > args.pitch_limit_deg or sensor_saturated
                if bad or args.mode == 'observe':
                    info['pwm_applied_percent'] = 0
                if motion:
                    robot.set_motors(0 if bad else info.get('pwm_sx_applied_percent',0),
                                     0 if bad else info.get('pwm_dx_applied_percent',0))
                    if bad:
                        info.update(pwm_sx_applied_percent=0.,pwm_dx_applied_percent=0.)
                else:
                    robot.set_motors(info['pwm_applied_percent'], info['pwm_applied_percent'])
                if battery is not None:
                    battery_load=battery.load(-robot.d.ctrl,[float(robot.d.qvel[j]) for j in wheel_dofs])
                    robot.set_supply_voltage(battery_load['battery_voltage_v'])
                mujoco.mj_forward(robot.m, robot.d)
                torques = [float(robot.d.qfrc_actuator[int(robot.m.joint(j).dofadr[0])]) for j in ('perno_sx', 'perno_dx')]
                ultrasonic = robot.read_ultrasonic()
                info['ultrasonic_distance_m'] = ultrasonic['ultrasonic_distance_m']
                row = dict(t_s=i * dt, t_free_s=(i - release_step) * dt if phase == 'free' else '', phase=phase, pitch_true_deg=true, pitch_acc_deg=accel_pitch, gyro_deg_s=gyro, accel_g=float(np.linalg.norm(acc) / 9.81), forward_y_m=float(robot.d.xpos[robot.body_id, 1]), velocity_world_y_m_s=float(robot.d.qvel[robot.root_v + 1]), duty_normalized=info['pwm_applied_percent'] / 100, tau_sx_Nm=torques[0], tau_dx_Nm=torques[1], vel_sx_rad_s=vel_l, vel_dx_rad_s=vel_r, support_active=int(robot.d.eq_active[robot.support_id]), **info)
                row.update(ultrasonic)
                if disturbance_enabled:
                    row['disturbance_pitch_torque_Nm'] = disturbance_torque
                if force_enabled:
                    row['disturbance_force_world_y_N'] = disturbance_force
                if battery_enabled:
                    row['battery_voltage_v'] = robot.voltage
                if battery is not None:
                    row.update(battery_load)
                if robot.imu_model is not None:
                    row.update(zip(('imu_ax_raw','imu_ay_raw','imu_az_raw','imu_gx_raw','imu_gy_raw','imu_gz_raw'),
                                   np.r_[robot.imu_model.accel_raw,robot.imu_model.gyro_raw].tolist()))
                    row['imu_saturated']=robot.imu_model.saturated
                if robot.encoder_model is not None:
                    row.update(encoder_sx_count=int(robot.encoder_model.counts[0]),
                               encoder_dx_count=int(robot.encoder_model.counts[1]))
                if motion:
                    from motion_lqr import wrap_angle
                    if release_pose is not None:
                        displacement=robot.d.xpos[robot.body_id,:2]-release_pose[0]
                        heading=release_pose[1]
                        local=np.array([[math.cos(heading),math.sin(heading)],
                                        [-math.sin(heading),math.cos(heading)]]) @ displacement
                        row.update(pose_true_x_m=float(local[0]),pose_true_y_m=float(local[1]),
                            yaw_true_rad=wrap_angle(robot.true_yaw_rad()-heading),
                            velocity_world_lateral_m_s=float(math.cos(heading)*robot.d.qvel[robot.root_v]+
                                math.sin(heading)*robot.d.qvel[robot.root_v+1]),
                            velocity_world_forward_m_s=float(-math.sin(heading)*robot.d.qvel[robot.root_v]+
                                math.cos(heading)*robot.d.qvel[robot.root_v+1]))
                        if sequence_enabled:
                            vx = float(math.cos(heading)*robot.d.qvel[robot.root_v]
                                       +math.sin(heading)*robot.d.qvel[robot.root_v+1])
                            row['velocity_world_x_m_s'] = vx
                            row['velocity_body_forward_m_s'] = float(
                                -math.sin(row['yaw_true_rad'])*vx
                                +math.cos(row['yaw_true_rad'])*row['velocity_world_forward_m_s'])
                    row.update(duty_sx_normalized=float(robot.d.ctrl[0]),duty_dx_normalized=float(robot.d.ctrl[1]),
                        gyro_x_cal_rad_s=float(gyro_raw[0]-gyro_bias_xyz[0]),
                        gyro_y_cal_rad_s=float(gyro_raw[1]-gyro_bias_xyz[1]),
                        gyro_z_cal_rad_s=float(gyro_raw[2]-gyro_bias_xyz[2]),
                        encoder_pos_sx_rad=pos_l,encoder_pos_dx_rad=pos_r)
                if path_enabled:
                    row['velocity_world_x_m_s']=float(robot.d.qvel[robot.root_v])
                    if phase=='free':
                        row['path_true_pose_error_m']=math.hypot(row['pose_true_x_m']-info['path_ref_x_m'],
                                                               row['pose_true_y_m']-info['path_ref_y_m'])
                writer.writerow(row)
                rows.append(row)
                if phase == 'free' and (i-release_step) % round(1/dt) == 0:
                    print(f"t libero={(i-release_step)*dt:.2f}s | "
                          f"PWM={info['pwm_applied_percent']:+.3f}% | "
                          f"tau SX={torques[0]:+.5f} Nm DX={torques[1]:+.5f} Nm | "
                          f"V={robot.voltage:.3f} V",flush=True)
                    if motion:
                        print(f"PWM SX/DX={info['pwm_sx_applied_percent']:+.3f}/{info['pwm_dx_applied_percent']:+.3f}% | "
                              f"s={info['position_encoder_m']:.4f}/{info['distance_ref_m']:.4f} m | "
                              f"yaw={math.degrees(info['x_rho_rad']):+.2f}°",flush=True)
                    if battery is not None:
                        print(f"SOC={100*battery.soc:.4f}% | consumati={1000*battery.used_ah.sum():.3f} mAh | "
                              f"I batteria={battery_load['battery_current_A']:.4f} A | "
                              f"elettronica={battery_load['electronics_current_A']:.4f} A | "
                              f"motori SX/DX={battery_load['motor_sx_battery_current_A']:.4f}/"
                              f"{battery_load['motor_dx_battery_current_A']:.4f} A",flush=True)
                if bad:
                    success = False
                    reason = ('stato non finito' if not finite else 'IMU simulata saturata' if sensor_saturated
                              else 'limite pitch geometrico')
                    break
                for _ in range(n):
                    if battery is not None:
                        wheel_rates=[float(robot.d.qvel[j]) for j in wheel_dofs]
                        battery_load=battery.load(-robot.d.ctrl,wheel_rates)
                        robot.set_supply_voltage(battery_load['battery_voltage_v'])
                    robot.step()
                    if obstacle is not None:
                        gid = robot.m.geom('path_obstacle_geom').id
                        if any(gid in (c.geom1, c.geom2) for c in robot.d.contact):
                            obstacle_contact_steps += 1
                    if battery is not None:
                        battery.advance(float(robot.m.opt.timestep),battery_load)
                if viewer:
                    with viewer.lock():
                        viewer.user_scn.ngeom = 0
                        if path_enabled and release_pose is not None:
                            draw_ground_path(viewer.user_scn,plan,release_pose[0],release_pose[1],
                                             info.get('path_reference_time_s',(i-release_step)*dt))
                        if not args.hide_ultrasonic_cone:
                            from ultrasonic_model import draw_ultrasonic_cone
                            draw_ultrasonic_cone(viewer.user_scn, robot.m, robot.d, robot.ultrasonic_model)
                    viewer.set_texts(build_hud(phase, args.mode, info, robot.voltage,
                        torques, disturbance_torque, battery, battery_load,
                        force_y_n=float(robot.d.xfrc_applied[robot.body_id,1]),
                        actual_start_pitch_deg=metadata.get('release_true_pitch_deg')))
                    viewer.sync()
                    time.sleep(max(0, wall + (i + 1) * dt - time.monotonic()))
            robot.set_motors(0, 0)
            mujoco.mj_forward(robot.m, robot.d)
            persist()
            if viewer:
                final_load=battery_load
                if battery is not None:
                    try:
                        final_load=battery.load([0,0],[float(robot.d.qvel[j]) for j in wheel_dofs])
                    except BatteryDepleted:
                        final_load=None
                viewer.set_texts(build_hud(phase, args.mode, info, robot.voltage,
                    (0.,0.), 0., battery, final_load, stopped_reason=reason,
                    actual_start_pitch_deg=metadata.get('release_true_pitch_deg')))
                viewer.sync()
            if viewer and (not args.exit_on_stop):
                print('Simulazione ferma: ' + reason + '. Chiudere finestra o Ctrl+C.', flush=True)
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.05)
    except BatteryDepleted as error:
        reason=str(error)
        success=False
    except KeyboardInterrupt:
        if not finished:
            reason = 'interruzione utente'
            success = False
    except Exception as e:
        if not finished:
            reason = f'errore: {type(e).__name__}: {e}'
            success = False
    finally:
        persist()
    return 0 if success else 1
if __name__ == '__main__':
    raise SystemExit(run(parse_args()))
