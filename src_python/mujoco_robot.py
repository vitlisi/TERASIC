"""Interfaccia fisica MuJoCo: PWM, IMU, encoder e supporto temporaneo."""
import math
from pathlib import Path
import mujoco
import numpy as np
import json
from ultrasonic_model import UltrasonicCone, DEFAULT_CONFIG as ULTRASONIC_CONFIG
from wheel_dynamics import update_wheels
ROOT = Path(__file__).resolve().parents[1]
XML_PATH = ROOT / "models/verifica.xml"
DEFAULT_MOTOR_FRICTION_PROFILE = ROOT / 'config/mujoco_robot01_stribeck.json'


class MujocoRobot:
    def __init__(self, xml_path=XML_PATH, ultrasonic_test_distance_m=None,
                 ultrasonic_config=ULTRASONIC_CONFIG, path_obstacle=None):
        self.path_obstacle = path_obstacle
        self.wheel_dynamics = {}
        self.motor_friction_profile = None
        if ultrasonic_test_distance_m is None and path_obstacle is None:
            self.m = mujoco.MjModel.from_xml_path(str(xml_path))
        else:
            # Optional static test target; never rewrite the user's XML.
            import xml.etree.ElementTree as ET
            tree = ET.parse(xml_path)
            root = tree.getroot()
            compiler = root.find('compiler')
            compiler.set('meshdir', str((Path(xml_path).resolve().parent /
                                        compiler.get('meshdir', '')).resolve()))
            if ultrasonic_test_distance_m is not None:
                distance = float(ultrasonic_test_distance_m)
                if not math.isfinite(distance) or distance <= 0:
                    raise ValueError('Distanza ostacolo positiva e finita richiesta')
                site = root.find(".//site[@name='ultrasonic_site']")
                front = float(site.get('pos').split()[1])
                ET.SubElement(root.find('worldbody'), 'geom', dict(
                    name='ultrasonic_test_wall', type='box', size='0.5 0.01 0.3',
                    pos=f'0 {front+distance+.01} 0.3', rgba='0.9 0.4 0.15 1', mass='0'))
            if path_obstacle is not None:
                dimensions = np.asarray(path_obstacle.get('size_m', [.20, .04, .20]), dtype=float)
                if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
                    raise ValueError('Dimensioni ostacolo positive e finite richieste')
                target = ET.SubElement(root.find('worldbody'), 'body', dict(
                    name='path_obstacle', mocap='true', pos=f'0 0 {dimensions[2]/2}'))
                collision = '1' if path_obstacle.get('collision', False) else '0'
                ET.SubElement(target, 'geom', dict(name='path_obstacle_geom',
                    type='box', size=' '.join(str(v/2) for v in dimensions), rgba='0.9 0.4 0.15 0.8',
                    mass='0', contype=collision, conaffinity=collision))
            self.m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.d = mujoco.MjData(self.m)
        self.ultrasonic_model = UltrasonicCone(self.m, ultrasonic_config)
        def parameter(name):
            return float(self.m.numeric(name).data[0])
        self.voltage = parameter('V_nom')
        self.nominal_voltage = self.voltage
        self.wheel_track_m=parameter('wheel_track_m')
        self.imu_model=None
        self.encoder_model=None
        self.stall_current = parameter('I_stall')
        self.gear = parameter('motor_gear')
        self.stall_torque = parameter('motor_max_torque')
        if min(self.voltage, self.stall_current, self.gear, self.stall_torque) <= 0:
            raise ValueError('Parametri motore non positivi')
        self.resistance = self.voltage / self.stall_current
        self.kt = self.stall_torque / (self.gear * self.stall_current)
        self.torque_per_duty = self.voltage * self.kt * self.gear / self.resistance
        for name in ('motore_sx', 'motore_dx'):
            self.m.actuator(name).gear[0] = -self.torque_per_duty
        self.root_q = int(self.m.joint('root').qposadr[0])
        self.root_v = int(self.m.joint('root').dofadr[0])
        self.body_id = self.m.body('struttura_centrale').id
        self.upright = self.d.qpos[self.root_q:self.root_q+7].copy()
        if self.path_obstacle is not None:
            self.place_path_obstacle(self.d.qpos[self.root_q:self.root_q+2], 0.)

    def place_path_obstacle(self, origin_xy, heading):
        """Anchor static test target to the actual release frame; no robot-state edits."""
        if self.path_obstacle is None:
            return None
        rotation = np.array([[math.cos(heading), -math.sin(heading)],
                             [math.sin(heading), math.cos(heading)]])
        xy = np.asarray(origin_xy)+rotation @ np.asarray(self.path_obstacle['local_xy_m'])
        yaw = heading+self.path_obstacle['local_yaw_rad']
        mocap = int(self.m.body('path_obstacle').mocapid[0])
        self.d.mocap_pos[mocap] = [*xy, self.m.geom('path_obstacle_geom').size[2]]
        self.d.mocap_quat[mocap] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        self.ultrasonic_model.sample = None
        self.ultrasonic_model.hit_point = None
        mujoco.mj_forward(self.m, self.d)
        return dict(world_xy_m=xy.tolist(), world_yaw_rad=yaw,
                    geom_id=self.m.geom('path_obstacle_geom').id)

    def read_imu(self):
        if self.imu_model is not None:
            return self.imu_model.read()
        return self.d.sensor('accel').data.copy(), self.d.sensor('giroscopio').data.copy()

    def read_encoders(self):
        if self.encoder_model is not None:
            return self.encoder_model.read(float(self.d.time))
        return tuple(float(self.d.sensor(name).data[0]) for name in
                     ('encoder_pos_sx', 'encoder_vel_sx', 'encoder_pos_dx', 'encoder_vel_dx'))

    def read_ultrasonic(self):
        """Read sampled cone; call after mj_forward when editing pose/geometry."""
        return self.ultrasonic_model.read(self.m, self.d)

    def configure_sensors(self,imu_mode,encoder_mode,path,seed=0,counts_per_rev=None,noise_scales=None):
        from sensor_models import Mpu6500,MotorEncoders
        config=json.loads(Path(path).read_text())
        if noise_scales is not None:
            scales = np.asarray(noise_scales,dtype=float)
            if scales.shape != (2,) or not np.isfinite(scales).all() or np.any(scales<0):
                raise ValueError('Scale rumore IMU finite e non negative richieste')
            config['imu']['accel_noise_density_g_sqrt_hz'] *= float(scales[0])
            config['imu']['gyro_noise_density_deg_s_sqrt_hz'] *= float(scales[1])
        if not math.isclose(config['track_width_m'],self.wheel_track_m,abs_tol=1e-9):
            raise ValueError('Carreggiata profilo sensori diversa dal modello XML')
        if imu_mode=='mpu6500':
            internal_dt=1/config['imu']['internal_rate_hz']
            if not math.isclose(self.m.opt.timestep,internal_dt):
                raise ValueError('Profilo MPU-6500 richiede fisica a 1 kHz')
            self.imu_model=Mpu6500(config['imu'],seed)
        if encoder_mode=='motor':
            self.encoder_model=MotorEncoders(config['encoder'],counts_per_rev)
        self.update_sensor_models()
        return config

    def update_sensor_models(self):
        if self.imu_model is None and self.encoder_model is None:
            return
        mujoco.mj_forward(self.m,self.d)
        if self.imu_model is not None:
            self.imu_model.update(float(self.d.time),self.d.sensor('accel').data,self.d.sensor('giroscopio').data)
        if self.encoder_model is not None:
            self.encoder_model.update_positions([float(self.d.sensor(name).data[0])
                for name in ('encoder_pos_sx','encoder_pos_dx')])

    def step(self):
        if self.motor_friction_profile is not None:
            from stribeck_motor import update
            update(self.m, self.d, self.d.ctrl*100, self.motor_friction_profile)
        mujoco.mj_step(self.m,self.d)
        self.update_sensor_models()

    def configure_motor_friction(self, profile):
        """Select the measured wheel/direction friction law for physics steps.

        Explicit opt-in: legacy linear plant designs and old runs retain their
        original dynamics. The supplied profile is copied to prevent caller edits.
        """
        import copy
        from stribeck_motor import configure, update
        selected = copy.deepcopy(profile)
        configure(self.m, selected)
        self.motor_friction_profile = selected
        self.wheel_dynamics = {}
        update(self.m, self.d, self.d.ctrl*100, selected)
        mujoco.mj_setConst(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)

    def set_motors(self, left_pwm_percent, right_pwm_percent):
        # Comandi percentuali -> duty normalizzato. L'attuatore converte in Nm.
        self.d.ctrl[:] = np.clip([left_pwm_percent, right_pwm_percent], -100, 100) / 100
        if self.motor_friction_profile is not None:
            from stribeck_motor import update
            update(self.m, self.d, self.d.ctrl*100, self.motor_friction_profile)
        elif self.wheel_dynamics:
            update_wheels(self.m, self.d.ctrl*100, self.wheel_dynamics)

    def set_supply_voltage(self, voltage_v):
        """Prescribed supply voltage; motor constants stay at nominal reference.

        Not a SOC/current model. Existing effective velocity damping is retained.
        """
        if not math.isfinite(voltage_v) or voltage_v <= 0:
            raise ValueError('Tensione di alimentazione positiva e finita richiesta')
        self.voltage = float(voltage_v)
        self.torque_per_duty = self.voltage*self.kt*self.gear/self.resistance
        for name in ('motore_sx','motore_dx'):
            self.m.actuator(name).gear[0] = -self.torque_per_duty

    def set_pitch_disturbance(self, torque_nm):
        """External world -X torque on chassis; positive increases pitch psi.

        A physical load, independent of motor commands and sensor readings.
        """
        if not math.isfinite(torque_nm):
            raise ValueError('Coppia di disturbo non finita')
        self.d.xfrc_applied[self.body_id, 3] = -torque_nm

    def start_support(self, pitch_deg=0):
        # Solo posa iniziale: nessuna riscrittura di qpos durante la fisica.
        half = math.radians(pitch_deg) / 2
        quat = [math.cos(half), -math.sin(half), 0, 0]
        self.d.qpos[self.root_q+3:self.root_q+7] = quat
        mocap = int(self.m.body('operator_support').mocapid[0])
        self.d.mocap_pos[mocap] = self.d.qpos[self.root_q:self.root_q+3]
        self.d.mocap_quat[mocap] = quat
        self.d.eq_active[self.support_id] = True
        self.m.site('support_marker').rgba[3] = 1
        mujoco.mj_forward(self.m, self.d)

    @property
    def support_id(self):
        return self.m.equality('manual_support').id

    def move_support(self, pitch_deg):
        # Muove solo il riferimento del vincolo, mai la posa del robot.
        half = math.radians(pitch_deg)/2
        mocap = int(self.m.body('operator_support').mocapid[0])
        self.d.mocap_quat[mocap] = [math.cos(half), -math.sin(half), 0, 0]

    def release(self):
        self.d.eq_active[self.support_id] = False
        self.m.site('support_marker').rgba[3] = 0
        mujoco.mj_forward(self.m, self.d)

    def support_force(self):
        # Isola la sola reazione weld: qfrc_constraint comprende anche contatti.
        force = np.zeros(self.m.nv)
        if self.d.eq_active[self.support_id]:
            mask = ((self.d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                    & (self.d.efc_id == self.support_id))
            constraint_force = np.where(mask, self.d.efc_force, 0.0)
            mujoco.mj_mulJacTVec(self.m, self.d, force, constraint_force)
        return force[self.root_v:self.root_v+6]

    def calibrate(self):
        # Utilita' per test senza viewer; il runner usa le stesse letture a 100 Hz.
        self.start_support()
        for _ in range(round(0.2 / self.m.opt.timestep)):
            mujoco.mj_step(self.m, self.d)
        calibration = Calibration()
        for _ in range(200):
            mujoco.mj_forward(self.m, self.d)
            acc, gyro = self.read_imu()
            calibration.add(acc, gyro)
            for _ in range(round(0.01 / self.m.opt.timestep)):
                mujoco.mj_step(self.m, self.d)
        return calibration.finish()[0]

    def true_pitch_deg(self):
        rot = self.d.xmat[self.body_id].reshape(3, 3)
        # Heading-independent pitch for Rz(rho) Ry(roll) Rx(-psi).
        return math.degrees(math.atan2(-rot[2, 1], rot[2, 2]))

    def true_yaw_rad(self):
        """Geometric heading for diagnostics only; not a controller sensor."""
        rot = self.d.xmat[self.body_id].reshape(3, 3)
        return math.atan2(rot[1, 0], rot[0, 0])


class Calibration:
    def __init__(self):
        self.angles = []
        self.gyros = []
        self.gyro_vectors = []

    def add(self, acc, gyro):
        angle = math.degrees(math.atan2(-acc[0], acc[2]))
        norm = float(np.linalg.norm(acc) / 9.81)
        if (not np.isfinite(np.r_[acc, gyro]).all() or not 0.75 <= norm <= 1.25
                or acc[2] / 9.81 < 11000 / 16384 or abs(angle) > 5):
            raise ValueError(f'calibrazione: postura/IMU non valida, pitch={angle:.3f}, g={norm:.3f}')
        self.angles.append(angle)
        self.gyros.append(math.degrees(gyro[1]))
        self.gyro_vectors.append(np.asarray(gyro).copy())

    def finish(self):
        if len(self.angles) != 200:
            raise ValueError('calibrazione: richiesti 200 campioni')
        bias = float(np.mean(self.gyros))
        if max(self.angles) - min(self.angles) > 3 or abs(bias) > 10:
            raise ValueError('calibrazione: movimento o bias eccessivo')
        return float(np.mean(self.angles)), bias

    def gyro_bias_vector(self):
        self.finish()  # same complete calibration and motion guards
        bias = np.mean(self.gyro_vectors, axis=0)
        if np.max(np.abs(np.degrees(bias))) > 10:
            raise ValueError('calibrazione: bias giroscopio eccessivo')
        return bias


