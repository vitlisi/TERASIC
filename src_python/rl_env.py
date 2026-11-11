"""Gymnasium adapter: sensor-only actor observations, shared MuJoCo plant/estimator.

Finite-horizon task: 10 seconds is terminal (not a time-limit truncation).
The critic observes remaining time; the actor explicitly ignores that feature.
"""
import copy
import math
from pathlib import Path

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from balance_controller import ControllerConfig
from ga_lqr import apply_scenario, scenarios
from motion_lqr import MotionLqrController, StraightReference, allocate_pwm, wrap_angle
from mujoco_robot import MujocoRobot, Calibration, ROOT
from rl_reward import reward_terms, validate_reward


class SensorEstimator(MotionLqrController):
    """Reuse the existing estimator/reference, without executing LQR feedback."""
    def command(self, error, dutyref, dt):
        self.measured_error = error.copy()
        return np.zeros(2)


class OffsetHoldReference(StraightReference):
    """Known fixed target displacement; no physical truth in feedback."""
    def __init__(self,design,position_offset_m,yaw_offset_deg):
        super().__init__(design,0.)
        self.position_offset=position_offset_m
        self.yaw_offset=math.radians(yaw_offset_deg)

    def sample(self,*args,**kwargs):
        state,duty,info=super().sample(*args,**kwargs)
        state[0]+=self.position_offset/self.radius
        state[2]=-self.yaw_offset
        info.update(distance_ref_m=-self.position_offset,yaw_ref_rad=-self.yaw_offset)
        return state,duty,info


class BalanceEnv(gym.Env):
    metadata = {'render_modes':[]}

    def __init__(self, config, design, training=True):
        self.config = copy.deepcopy(config)
        self.design = copy.deepcopy(design)
        self.training = training
        validate_reward(config['reward'])
        self.dt = float(config['dt_s'])
        self.steps = round(config['duration_s']/self.dt)
        self.scales = np.asarray(config['observation_scales'],dtype=float)
        if (self.scales.shape!=(6,) or not np.isfinite(self.scales).all() or np.any(self.scales<=0)
                or self.dt!=.01 or self.steps<1 or not math.isclose(self.steps*self.dt,config['duration_s'])
                or not 0<config['max_pwm']<=100 or config['feedback_delay_samples'] not in (0,1)
                or not 0<config['pitch_limit_deg']<=30):
            raise ValueError('Configurazione RL non valida')
        low,high = config['initial_pitch_range_deg']
        if not -5<=low<=high<=5:
            raise ValueError('Intervallo pitch iniziale entro +/-5 gradi richiesto')
        probability=config.get('reset_offset_probability',0.)
        if not math.isfinite(probability) or not 0<=probability<=1:
            raise ValueError('Probabilita offset non valida')
        for key,limit in [('initial_position_offset_range_m',.03),('initial_yaw_offset_range_deg',5.)]:
            low,high=config.get(key,[0.,0.])
            if not all(math.isfinite(v) for v in (low,high)) or not -limit<=low<=high<=limit:
                raise ValueError('Intervallo offset non valido: '+key)
        self.action_space = spaces.Box(-1.,1.,shape=(2,),dtype=np.float32)
        self.observation_space = spaces.Box(np.r_[np.full(6,-np.inf),0.].astype(np.float32),
                                            np.r_[np.full(6,np.inf),1.].astype(np.float32))
        self.robot = None
        self.done = True
        self.control_config = ControllerConfig(max_pwm=config['max_pwm'])

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.close()
        self.done = False
        self.index = 0
        options = options or {}
        angle = float(options.get('angle_deg',self.np_random.uniform(*self.config['initial_pitch_range_deg'])))
        if not math.isfinite(angle) or abs(angle)>5:
            raise ValueError('Pitch di rilascio entro +/-5 richiesto')
        self.force = float(options.get('force_y_n',0.))
        if not math.isfinite(self.force):
            raise ValueError('Forza non finita')
        sensor_seed = int(options.get('sensor_seed',self.np_random.integers(0,2**31-1)))
        self.sensor_seed = sensor_seed
        use_offsets=(self.training and self.config.get('reset_offset_probability',0.)>0
                     and self.np_random.random()<self.config['reset_offset_probability'])
        position=self.np_random.uniform(*self.config.get('initial_position_offset_range_m',[0.,0.])) if use_offsets else 0.
        yaw=self.np_random.uniform(*self.config.get('initial_yaw_offset_range_deg',[0.,0.])) if use_offsets else 0.
        self.initial_position_offset=float(options.get('position_offset_m',position))
        self.initial_yaw_offset=math.radians(float(options.get('yaw_offset_deg',yaw)))
        if (not math.isfinite(self.initial_position_offset) or abs(self.initial_position_offset)>.03
                or not math.isfinite(self.initial_yaw_offset) or abs(self.initial_yaw_offset)>math.radians(5)):
            raise ValueError('Offset iniziali entro 3 cm e 5 gradi richiesti')
        self.robot = MujocoRobot()
        if 'scenario' in options:
            scenario = options['scenario']
        elif self.training and self.config['randomize_model']:
            scenario = scenarios(2,sensor_seed,'rl')[1]
        else:
            scenario = scenarios(1,sensor_seed,'rl')[0]
        self.scenario = apply_scenario(self.robot,scenario)
        self.noise_rng = np.random.default_rng(np.random.SeedSequence([sensor_seed,1]))
        self.previous_noise = self.noise_velocity = np.zeros(2)
        self.robot.configure_sensors('mpu6500','motor',ROOT/'config/mujoco_sensors_mpu6500.json',sensor_seed)
        self.estimator = SensorEstimator(self.control_config,self.design,OffsetHoldReference(self.design,self.initial_position_offset,math.degrees(self.initial_yaw_offset)))
        self.previous_error = np.zeros(6)
        self.robot.start_support(0.)
        calibration = Calibration()
        self.substeps = round(self.dt/self.robot.m.opt.timestep)
        if not math.isclose(self.substeps*self.robot.m.opt.timestep,self.dt):
            raise ValueError('Passo fisico incompatibile')
        for i in range(320):
            if i==220:
                self.angle_bias,self.gyro_bias = calibration.finish()
                self.gyro_bias_xyz = calibration.gyro_bias_vector()
                self.estimator.reset(0.)
                self.robot.encoder_model.reset([float(self.robot.d.sensor(k).data[0])
                                               for k in ('encoder_pos_sx','encoder_pos_dx')])
            if i>=220:
                f = min(1.,(i-220+1)/50)
                self.robot.move_support(angle*f*f*(3-2*f))
            acc,gyro = self.robot.read_imu()
            if 20<=i<220:
                calibration.add(acc,gyro)
            if i>=220:
                accel = math.degrees(math.atan2(-acc[0],acc[2]))-self.angle_bias
                rate = math.degrees(gyro[1])-self.gyro_bias
                alpha = math.exp(-self.dt/self.control_config.filter_tau_s)
                self.estimator.pitch = alpha*(self.estimator.pitch+rate*self.dt)+(1-alpha)*accel
            for _ in range(self.substeps):
                self.robot.step()
        self.robot.release()
        self.origin = self.robot.d.xpos[self.robot.body_id,:2].copy()
        self.origin_yaw = self.robot.true_yaw_rad()
        # Fixed world target specified by the known reset fixture, never recentered in flight.
        self.origin-=self.initial_position_offset*np.array([-math.sin(self.origin_yaw),math.cos(self.origin_yaw)])
        self.initial_pitch = self.robot.true_pitch_deg()
        self.pwm = np.zeros(2)
        self.observation = self._observe()
        self.last_row = self._row()
        self.last_truth = self._truth()
        return self.observation.copy(),dict(initial_pitch_deg=self.initial_pitch,scenario=self.scenario,
                                            initial_position_offset_m=self.initial_position_offset,
                                            initial_yaw_offset_deg=math.degrees(self.initial_yaw_offset),
                                            position_target_world_m=self.origin.tolist(),
                                            yaw_target_world_rad=self.origin_yaw-self.initial_yaw_offset,
                                            sensor_seed=sensor_seed)

    def _observe(self):
        acc,gyro = self.robot.read_imu()
        left,lv,right,rv = self.robot.read_encoders()
        sigma = self.scenario['encoder_noise_std_counts']*2*math.pi/self.robot.encoder_model.counts_per_rev
        noise = self.noise_rng.normal(0.,sigma,2)
        derivative = (noise-self.previous_noise)/self.dt
        alpha = math.exp(-self.dt/self.robot.encoder_model.config['velocity_filter_tau_s'])
        self.noise_velocity = alpha*self.noise_velocity+(1-alpha)*derivative
        self.previous_noise = noise
        self.info = self.estimator.update(
            math.degrees(math.atan2(-acc[0],acc[2]))-self.angle_bias,
            math.degrees(gyro[1])-self.gyro_bias,lv+self.noise_velocity[0],rv+self.noise_velocity[1],self.dt,
            wheel_positions=(left+noise[0],right+noise[1]),
            gyro_xyz_rad_s=gyro-self.gyro_bias_xyz,free_time_s=self.index*self.dt)
        # Express measured pose in the fixed target frame using prescribed offsets.
        self.info['position_encoder_m']+=self.initial_position_offset
        self.info['pose_est_y_m']+=self.initial_position_offset
        current = self.estimator.measured_error
        delayed = self.previous_error if self.config['feedback_delay_samples'] else current
        obs = np.r_[delayed/self.scales,max(0.,1-self.index/self.steps)].astype(np.float32)
        self.previous_error = current.copy()
        if not np.isfinite(obs).all():
            raise FloatingPointError('Osservazione RL non finita')
        return obs

    def _truth(self):
        robot = self.robot
        gyro = robot.d.sensor('giroscopio').data
        pitch = math.radians(robot.true_pitch_deg())
        return dict(pitch_error_deg=robot.true_pitch_deg()-self.design['equilibrium_pitch_deg'],
                    pitch_rate_deg_s=math.degrees(gyro[1]),
                    position_m=float(np.linalg.norm(robot.d.xpos[robot.body_id,:2]-self.origin)),
                    speed_m_s=float(np.linalg.norm(robot.d.qvel[robot.root_v:robot.root_v+2])),
                    yaw_deg=math.degrees(wrap_angle(robot.true_yaw_rad()-self.origin_yaw+self.initial_yaw_offset)),
                    yaw_rate_deg_s=math.degrees(-gyro[0]*math.sin(pitch)+gyro[2]*math.cos(pitch)))

    def _row(self):
        robot = self.robot
        displacement = robot.d.xpos[robot.body_id,:2]-self.origin
        c,s = math.cos(self.origin_yaw),math.sin(self.origin_yaw)
        local = np.array([[c,s],[-s,c]])@displacement
        row = dict(t_s=self.index*self.dt,t_free_s=self.index*self.dt,phase='free',
            initial_position_offset_m=self.initial_position_offset,
            initial_yaw_offset_deg=math.degrees(self.initial_yaw_offset),
            pitch_true_deg=robot.true_pitch_deg(),**self.info,
            pose_true_x_m=float(local[0]),pose_true_y_m=float(local[1]),
            yaw_true_rad=wrap_angle(robot.true_yaw_rad()-self.origin_yaw+self.initial_yaw_offset),
            velocity_world_lateral_m_s=float(c*robot.d.qvel[robot.root_v]+s*robot.d.qvel[robot.root_v+1]),
            velocity_world_forward_m_s=float(-s*robot.d.qvel[robot.root_v]+c*robot.d.qvel[robot.root_v+1]),
            disturbance_force_world_y_N=float(robot.d.xfrc_applied[robot.body_id,1]))
        row.update(pwm_sx_applied_percent=float(self.pwm[0]),pwm_dx_applied_percent=float(self.pwm[1]))
        return row

    def step(self, action):
        if self.done:
            raise RuntimeError('Episodio finito: chiamare reset')
        action = np.asarray(action,dtype=float)
        if action.shape!=(2,) or not np.isfinite(action).all():
            raise ValueError('Due azioni finite richieste')
        used_observation = self.observation.copy()
        _,self.pwm,_,_ = allocate_pwm(self.config['max_pwm']*np.clip(action,-1,1),self.control_config)
        self.robot.set_motors(*self.pwm)
        time = self.index*self.dt
        force = self.force if time>=5 and (time-5)%5 < .5-1e-9 else 0.
        self.robot.d.xfrc_applied[self.robot.body_id,1] = force
        reason = None
        try:
            for _ in range(self.substeps):
                self.robot.step()
                if not np.isfinite(np.r_[self.robot.d.qpos,self.robot.d.qvel]).all():
                    reason = 'stato non finito';break
                if abs(self.robot.true_pitch_deg())>self.config['pitch_limit_deg']:
                    reason = 'limite pitch';break
                if self.robot.imu_model.saturated:
                    reason = 'IMU simulata saturata';break
            self.index += 1
            if reason is None:
                self.observation = self._observe()
        except ValueError as error:
            if str(error) not in ('Contatore encoder simulato saturo; nessun rollover',
                                  'Posizioni encoder non finite','Ingresso IMU non valido'):
                self.close();raise
            reason = str(error)
        failed = reason is not None
        truth = self._truth()
        if not all(math.isfinite(v) for v in truth.values()):
            truth = self.last_truth.copy()
        self.last_truth = truth.copy()
        reward,terms = reward_terms(truth,self.pwm,self.config['max_pwm'],self.config['reward'],failed)
        self.done = failed or self.index>=self.steps
        if reason != 'stato non finito':
            self.last_row = self._row()
        self.last_row.update(zip(('observation_'+str(i) for i in range(7)),used_observation.tolist()))
        self.last_row.update(pwm_requested_percent=float(np.mean(self.config['max_pwm']*action)),
            pwm_applied_percent=float(np.mean(self.pwm)),pwm_percent=float(np.mean(self.pwm)),
            saturated=bool(np.any(np.abs(action)>1.)))
        self.last_row.update(reward=reward,**{'reward_penalty_'+k:v for k,v in terms.items()})
        if self.done:
            self.robot.set_motors(0,0)
            self.robot.d.xfrc_applied[:] = 0
        return self.observation.copy(),reward,self.done,False,dict(
            failed=failed,stop_reason=reason or ('durata completata' if self.done else 'running'),
            reward_terms=terms,truth=truth,initial_pitch_deg=self.initial_pitch,sensor_seed=self.sensor_seed,
            motor_sx_scale=self.scenario['motor_scale'][0],motor_dx_scale=self.scenario['motor_scale'][1],
            armature_sx_scale=self.scenario['armature_scale'][0],armature_dx_scale=self.scenario['armature_scale'][1],
            friction_scale=self.scenario['friction_scale'],chassis_inertia_scale=self.scenario['chassis_inertia_scale'],
            encoder_noise_std_counts=self.scenario['encoder_noise_std_counts'])

    def close(self):
        self.done = True
        if self.robot is not None:
            self.robot.set_motors(0,0)
            self.robot.d.xfrc_applied[:] = 0
