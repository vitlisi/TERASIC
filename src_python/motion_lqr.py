"""Two-input local LQR and sensor-only odometry for a short straight move.

Canonical state: [theta, psi, rho, theta_dot, psi_dot, rho_dot].
The six-state design is a no-slip projection, not the full MuJoCo state.
"""
import math
from pathlib import Path
import mujoco
import numpy as np
from balance_controller import BalanceController
from lqr_controller import FULL_STATE_ORDER, solve_dlqr
from mujoco_robot import MujocoRobot


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def design_motion_lqr(dt=.01, epsilon=1e-4, wheel_profiles=None):
    if not math.isfinite(dt) or dt <= 0 or not math.isfinite(epsilon) or epsilon <= 0:
        raise ValueError('Periodo ed epsilon positivi e finiti richiesti')
    robot = MujocoRobot()
    m, d = robot.m, robot.d
    if wheel_profiles is not None:
        from wheel_dynamics import configure_wheels
        configure_wheels(m, wheel_profiles)
        mujoco.mj_setConst(m, d)
    m.dof_frictionloss[:] = 0  # design only; physical plant keeps dry friction
    m.opt.iterations = 100
    m.opt.tolerance = 1e-12
    equilibrium = -math.atan2(m.body('struttura_centrale').ipos[1],
                             m.body('struttura_centrale').ipos[2])
    robot.start_support(math.degrees(equilibrium))
    for _ in range(round(2 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    robot.release()
    base = d.qpos.copy()
    wheel_q = [int(m.joint(name).qposadr[0]) for name in ('perno_sx', 'perno_dx')]
    wheel_v = [int(m.joint(name).dofadr[0]) for name in ('perno_sx', 'perno_dx')]
    radius, track = .032, robot.wheel_track_m
    steps = round(dt / m.opt.timestep)
    if not math.isclose(steps * m.opt.timestep, dt):
        raise ValueError('Il periodo LQR deve essere multiplo del passo fisico')

    def transition(z, duty):
        mujoco.mj_resetData(m, d)
        d.eq_active[:] = False
        d.qpos[:] = base
        theta, pitch_error, yaw, wheel_rate, pitch_rate, yaw_rate = z
        pitch = equilibrium + pitch_error
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        # Rz(yaw) Rx(-pitch), quaternion [w,x,y,z].
        d.qpos[robot.root_q+3:robot.root_q+7] = [cy*cp, -cy*sp, -sy*sp, sy*cp]
        displacement = radius*(pitch_error-theta)
        d.qpos[robot.root_q:robot.root_q+2] += [-math.sin(yaw)*displacement,
                                              math.cos(yaw)*displacement]
        d.qpos[wheel_q] = base[wheel_q] + [theta+track*yaw/(2*radius),
                                         theta-track*yaw/(2*radius)]
        d.qvel[:] = 0
        speed = radius*(pitch_rate-wheel_rate)
        d.qvel[robot.root_v:robot.root_v+2] = [-math.sin(yaw)*speed,
                                              math.cos(yaw)*speed]
        # Free-joint angular qvel is expressed in the body frame.
        d.qvel[robot.root_v+3:robot.root_v+6] = [-pitch_rate,
            -yaw_rate*math.sin(pitch), yaw_rate*math.cos(pitch)]
        d.qvel[wheel_v] = [wheel_rate+track*yaw_rate/(2*radius),
                            wheel_rate-track*yaw_rate/(2*radius)]
        d.ctrl[:] = duty
        for _ in range(steps):
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        rotation = d.xmat[robot.body_id].reshape(3, 3)
        pitch = math.radians(robot.true_pitch_deg())
        gyro = robot.read_imu()[1]
        return np.array([np.mean(d.qpos[wheel_q]-base[wheel_q]), pitch-equilibrium,
            math.atan2(rotation[1, 0], rotation[0, 0]), np.mean(d.qvel[wheel_v]),
            gyro[1], -gyro[0]*math.sin(pitch)+gyro[2]*math.cos(pitch)])

    a = np.column_stack([(transition(np.eye(6)[j]*epsilon, [0, 0])-
                          transition(-np.eye(6)[j]*epsilon, [0, 0]))/(2*epsilon)
                         for j in range(6)])
    b = np.column_stack([(transition(np.zeros(6), np.eye(2)[j]*epsilon)-
                          transition(np.zeros(6), -np.eye(2)[j]*epsilon))/(2*epsilon)
                         for j in range(2)])
    residual = transition(np.zeros(6), [0, 0])
    if np.max(np.abs(residual)) > 1e-4:
        raise ValueError('Postura nominale non sufficientemente vicina all’equilibrio')
    controllability = np.column_stack([np.linalg.matrix_power(a, j) @ b for j in range(6)])
    rank = int(np.linalg.matrix_rank(controllability))
    if rank != 6:
        raise ValueError('Modello a sei stati non controllabile')
    limits = np.array([.02/radius, math.radians(3), math.radians(5),
                       .5, math.radians(30), math.radians(30)])
    q = np.diag(1/limits**2)
    # Equal wheel duties incur the same total effort penalty as the old LQR.
    r = np.eye(2)/(2*.15**2)
    k, poles = solve_dlqr(a, b, q, r)
    result = dict(full_state_order=FULL_STATE_ORDER, state_order=FULL_STATE_ORDER,
        input_order=['duty_sx', 'duty_dx'], output_order=FULL_STATE_ORDER,
        Ad=a.tolist(), Bd_duty=b.tolist(), C=np.eye(6).tolist(), D=np.zeros((6, 2)).tolist(),
        Q=q.tolist(), R=r.tolist(), K_duty=k.tolist(), K_pwm_percent=(100*k).tolist(),
        closed_loop_poles=[[float(v.real), float(v.imag)] for v in poles],
        open_loop_poles=[[float(v.real), float(v.imag)] for v in np.linalg.eigvals(a)],
        equilibrium_pitch_deg=math.degrees(equilibrium), dt_s=dt,
        finite_difference_epsilon=epsilon, controllability_rank=rank,
        nominal_step_residual=residual.tolist(), design_joint_frictionloss=0,
        plant_joint_frictionloss='unchanged', wheel_radius_m=radius, track_width_m=track,
        bryson_state_limits=limits.tolist(), bryson_common_duty_limit=.15,
        method='central differences of MuJoCo rollout; six-state no-slip lift/projection',
        output_note='C/D describe estimated states, not raw ADC or encoder counts',
        simscape_used_for_gains=False)
    if wheel_profiles is not None:
        import copy
        result['wheel_profiles'] = copy.deepcopy(wheel_profiles)
    return result


class StraightReference:
    """Quintic distance profile plus local quasi-static model feedforward."""
    def __init__(self, design, distance=.30, start=3., duration=6.):
        if not all(math.isfinite(v) for v in (distance, start, duration)) or start < 0 or duration <= 0:
            raise ValueError('Riferimento movimento non valido')
        self.distance, self.start, self.duration = distance, start, duration
        self.radius = design['wheel_radius_m']
        self.equilibrium = math.radians(design['equilibrium_pitch_deg'])
        self.a = np.array(design['Ad'])
        self.b = np.array(design['Bd_duty'])
        self.dt = design['dt_s']
        common = self.b.sum(axis=1)
        self.inverse = np.linalg.inv(np.column_stack((self.a[3:5, 1], common[3:5])))

    def profile(self, time):
        f = max(0., min(1., (time-self.start)/self.duration))
        s = self.distance*(10*f**3-15*f**4+6*f**5)
        v = self.distance/self.duration*(30*f**2-60*f**3+30*f**4)
        a = self.distance/self.duration**2*(60*f-180*f**2+120*f**3)
        j = self.distance/self.duration**3*(60-360*f+360*f**2) if 0 < f < 1 else 0.
        return s, v, a, j

    def sample(self, time, theta_origin, pitch_origin):
        s, v, acceleration, jerk = self.profile(time)
        rhs = np.array([-acceleration*self.dt/self.radius, 0.])
        rhs -= (self.a-np.eye(6))[3:5, 3]*(-v/self.radius)
        pitch_offset, duty = self.inverse @ rhs
        derivative_rhs = np.array([-jerk*self.dt/self.radius, 0.])
        derivative_rhs -= (self.a-np.eye(6))[3:5, 3]*(-acceleration/self.radius)
        pitch_rate = (self.inverse @ derivative_rhs)[0]
        pitch = self.equilibrium+pitch_offset
        x = np.array([theta_origin+pitch-pitch_origin-s/self.radius,
                      pitch, 0., pitch_rate-v/self.radius, pitch_rate, 0.])
        return x, np.array([duty, duty]), dict(distance_ref_m=s, velocity_ref_m_s=v,
            acceleration_ref_m_s2=acceleration, yaw_ref_rad=0.)


class RealPathReference(StraightReference):
    """Hardware timed 40 cm/pivot/40 cm schedule, with MuJoCo model feedforward.

    Rebase the longitudinal target on sensor estimates at phase entries, as
    LqrSimplePath does. No arrival gates or external XY pose feedback.
    """
    uses_pose = True

    @staticmethod
    def read_settings(path):
        values = {}
        for line in Path(path).read_text().splitlines():
            line = line.split('#', 1)[0].strip()
            if line:
                key, value = line.split()
                values[key] = value
        keys = ('path_distance_m', 'path_turn_deg', 'leg_duration_s',
                'turn_duration_s', 'initial_hold_s', 'between_hold_s',
                'final_hold_s', 'wheel_pwm_cap')
        result = {key: float(values[key]) for key in keys}
        if any(not math.isfinite(v) or v <= 0 for v in result.values()):
            raise ValueError('Distanze, angoli, tempi e limite PWM positivi e finiti richiesti')
        if result['wheel_pwm_cap'] > 100 or float(values.get('continuous_reference', 0)) != 0:
            raise ValueError('Richiesto profilo reale con riferimento riposizionato e PWM <=100')
        return result

    def __init__(self, design, settings):
        self.settings = dict(settings)
        super().__init__(design, settings['path_distance_m'], settings['initial_hold_s'],
                         settings['leg_duration_s'])
        self.turn_start = self.start+self.duration+settings['between_hold_s']
        self.turn_duration = settings['turn_duration_s']
        self.second_start = self.turn_start+self.turn_duration+settings['between_hold_s']
        self.schedule_duration = self.second_start+self.duration+settings['final_hold_s']
        self.turn = math.radians(settings['path_turn_deg'])
        self.origins = [None, None, None]
        differential = self.b[:, 1]-self.b[:, 0]
        if abs(differential[5]) < 1e-12:
            raise ValueError('Modello senza autorita yaw')
        self.yaw_velocity_ff = -(self.a[5, 5]-1)/differential[5]
        self.yaw_accel_ff = self.dt/differential[5]

    @staticmethod
    def quintic(time, start, duration, distance):
        f = max(0., min(1., (time-start)/duration))
        return (distance*(10*f**3-15*f**4+6*f**5),
                distance/duration*(30*f**2-60*f**3+30*f**4),
                distance/duration**2*(60*f-180*f**2+120*f**3),
                distance/duration**3*(60-360*f+360*f*f) if 0 < f < 1 else 0.)

    def sample(self, time, theta_origin, pitch_origin, pose, state):
        measured_s = self.radius*(state[1]-pitch_origin-state[0]+theta_origin)
        for index, start in enumerate((self.start, self.turn_start, self.second_start)):
            if time >= start and self.origins[index] is None:
                self.origins[index] = measured_s
        phase = sum(time >= boundary for boundary in (
            self.start, self.start+self.duration, self.turn_start,
            self.turn_start+self.turn_duration, self.second_start,
            self.second_start+self.duration))
        if time < self.turn_start:
            leg = self.quintic(time, self.start, self.duration, self.distance)
            s = 0. if time < self.start else self.origins[0]+leg[0]
        elif time < self.second_start:
            leg = (0., 0., 0., 0.)
            s = self.origins[1]
        else:
            leg = self.quintic(time, self.second_start, self.duration, self.distance)
            s = self.origins[2]+leg[0]
        _, v, acceleration, jerk = leg
        yaw, yaw_rate, yaw_accel, _ = self.quintic(
            time, self.turn_start, self.turn_duration, self.turn)
        rhs = np.array([-acceleration*self.dt/self.radius, 0.])
        rhs -= (self.a-np.eye(6))[3:5, 3]*(-v/self.radius)
        pitch_offset, common = self.inverse @ rhs
        derivative_rhs = np.array([-jerk*self.dt/self.radius, 0.])
        derivative_rhs -= (self.a-np.eye(6))[3:5, 3]*(-acceleration/self.radius)
        pitch_rate = (self.inverse @ derivative_rhs)[0]
        pitch = self.equilibrium+pitch_offset
        differential = self.yaw_velocity_ff*yaw_rate+self.yaw_accel_ff*yaw_accel
        xref = np.array([theta_origin+pitch-pitch_origin-s/self.radius, pitch,
                         yaw, pitch_rate-v/self.radius, pitch_rate, yaw_rate])
        planned_y = (self.quintic(time, self.start, self.duration, self.distance)[0]
                     -self.quintic(time, self.second_start, self.duration, self.distance)[0])
        return xref, np.array([common-differential, common+differential]), dict(
            distance_ref_m=s, velocity_ref_m_s=v, acceleration_ref_m_s2=acceleration,
            yaw_ref_rad=yaw, sequence_phase=int(phase),
            sequence_finished=int(time >= self.schedule_duration), planned_y_m=planned_y)


def allocate_pwm(requested, config):
    """Clip common command first, then differential to remaining wheel headroom."""
    common = float(np.mean(requested))
    differential = float((requested[1]-requested[0])/2)
    limited_common = float(np.clip(common, -config.max_pwm, config.max_pwm))
    headroom = config.max_pwm-abs(limited_common)
    limited_differential = float(np.clip(differential, -headroom, headroom))
    limited = np.array([limited_common-limited_differential, limited_common+limited_differential])
    applied = np.array([0. if abs(pwm) < config.pwm_deadband_percent else
        math.copysign(math.floor(config.pwm_period_ticks*abs(pwm)/100)*100/config.pwm_period_ticks, pwm)
        for pwm in limited])
    return limited, applied, common, differential


class MotionLqrController(BalanceController):
    """One MIMO LQR; feedback accepts only encoder/IMU measurements."""
    def __init__(self, config, design, reference, yaw_filter_tau_s=.5, feedback_delay_samples=0):
        super().__init__(config, design['equilibrium_pitch_deg'])
        if not math.isfinite(yaw_filter_tau_s) or yaw_filter_tau_s <= 0:
            raise ValueError('Costante filtro yaw positiva richiesta')
        self.design, self.reference = design, reference
        self.k = np.array(design['K_pwm_percent'])
        self.track = design['track_width_m']
        self.yaw_tau = yaw_filter_tau_s
        if feedback_delay_samples not in (0, 1):
            raise ValueError('Ritardo feedback di zero o un campione richiesto')
        self.feedback_delay_samples = feedback_delay_samples
        self.reset(0.)

    def reset(self, initial_pitch_deg):
        super().reset(initial_pitch_deg)
        self.x = np.array([0., math.radians(initial_pitch_deg), 0., 0., 0., 0.])
        self.origin = None
        self.previous_distance = 0.
        self.pose = np.zeros(2)
        self.previous_error = np.zeros(6)

    def command(self, error, dutyref, dt):
        """Control law separated from the common encoder/IMU estimator."""
        feedback = self.previous_error if self.feedback_delay_samples else error
        raw = 100*dutyref-self.k @ feedback
        self.previous_error = error.copy()
        return raw

    def update(self, accel_pitch_deg, gyro_deg_s, left_joint_velocity, right_joint_velocity,
               dt, ideal_pitch_deg=None, desired_velocity_m_s=0., wheel_positions=None,
               gyro_xyz_rad_s=None, free_time_s=0.):
        if wheel_positions is None or np.asarray(wheel_positions).shape != (2,) or not np.isfinite(wheel_positions).all():
            raise ValueError('Due posizioni encoder finite richieste')
        if gyro_xyz_rad_s is None or np.asarray(gyro_xyz_rad_s).shape != (3,) or not np.isfinite(gyro_xyz_rad_s).all():
            raise ValueError('Tre assi giroscopio calibrati richiesti')
        if not math.isfinite(free_time_s):
            raise ValueError('Tempo riferimento non finito')
        info = super().update(accel_pitch_deg, gyro_deg_s, left_joint_velocity,
            right_joint_velocity, dt, ideal_pitch_deg, 0.)
        pitch, pitch_rate = math.radians(self.pitch), math.radians(gyro_deg_s)
        angles = np.array(wheel_positions)
        theta = float(np.mean(angles))
        if self.origin is None:
            self.origin = (angles.copy(), pitch)
        angle_origin, pitch_origin = self.origin
        yaw_encoder = self.config.wheel_radius_m/self.track*float(
            (angles-angle_origin)[0]-(angles-angle_origin)[1])
        gx, _, gz = gyro_xyz_rad_s
        yaw_rate = -gx*math.sin(pitch)+gz*math.cos(pitch)
        predicted = self.x[2]+yaw_rate*dt
        alpha = math.exp(-dt/self.yaw_tau)
        yaw = predicted+(1-alpha)*wrap_angle(yaw_encoder-predicted)
        distance = self.config.wheel_radius_m*(pitch-pitch_origin-theta+float(np.mean(angle_origin)))
        delta = distance-self.previous_distance
        mid_yaw = (self.x[2]+yaw)/2
        self.pose += delta*np.array([-math.sin(mid_yaw), math.cos(mid_yaw)])
        self.previous_distance = distance
        self.position = distance
        self.x[:] = [theta, pitch, yaw, (left_joint_velocity+right_joint_velocity)/2,
                      pitch_rate, yaw_rate]
        extra = dict(pose=self.pose, state=self.x) if getattr(self.reference, 'uses_pose', False) else {}
        xref, dutyref, ref_info = self.reference.sample(free_time_s, float(np.mean(angle_origin)), pitch_origin, **extra)
        error = self.x-xref
        error[2] = wrap_angle(error[2])
        raw = self.command(error, dutyref, dt)
        limited, applied, common, differential = allocate_pwm(raw, self.config)
        info.update(pitch_target_deg=math.degrees(xref[1]), position_encoder_m=distance,
            pwm_requested_percent=float(np.mean(raw)), pwm_percent=float(np.mean(limited)),
            pwm_applied_percent=float(np.mean(applied)), saturated=bool(np.any(np.abs(raw-limited)>1e-9)),
            pwm_sx_requested_percent=float(raw[0]), pwm_dx_requested_percent=float(raw[1]),
            pwm_sx_applied_percent=float(applied[0]), pwm_dx_applied_percent=float(applied[1]),
            pwm_common_requested_percent=common, pwm_differential_requested_percent=differential,
            lqr_psi_error_rad=float(error[1]), lqr_theta_dot_error_rad_s=float(error[3]),
            lqr_psi_dot_rad_s=float(error[4]), yaw_valid=True, yaw_encoder_rad=yaw_encoder,
            pose_est_x_m=float(self.pose[0]), pose_est_y_m=float(self.pose[1]),
            distance_error_m=distance-ref_info['distance_ref_m'], yaw_error_rad=float(error[2]),
            reference_duty_sx=float(dutyref[0]), reference_duty_dx=float(dutyref[1]), **ref_info)
        info.update(zip(('x_theta_rad', 'x_psi_rad', 'x_rho_rad', 'x_theta_dot_rad_s',
                         'x_psi_dot_rad_s', 'x_rho_dot_rad_s'), self.x.tolist()))
        info.update(zip(('error_theta_rad', 'error_psi_rad', 'error_rho_rad', 'error_theta_dot_rad_s',
                         'error_psi_dot_rad_s', 'error_rho_dot_rad_s'), error.tolist()))
        return info
