"""Local MuJoCo longitudinal design; no Simscape or hardware gains imported."""
import math
import numpy as np
import mujoco
from balance_controller import BalanceController
from mujoco_robot import MujocoRobot, ROOT
import json

FULL_STATE_ORDER = ['theta', 'psi', 'rho', 'theta_dot', 'psi_dot', 'rho_dot']
LONGITUDINAL_INDICES = [1, 3, 4]  # psi, theta_dot, psi_dot (zero-based)


def solve_dlqr(a, b, q, r):
    p = q.copy()
    for _ in range(20000):
        k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
        new = q + a.T @ p @ a - a.T @ p @ b @ k
        new = (new + new.T) / 2
        if np.linalg.norm(new-p) <= 1e-11 * max(1, np.linalg.norm(new)):
            p = new
            break
        p = new
    else:
        raise ValueError('Riccati non convergente')
    k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
    poles = np.linalg.eigvals(a-b@k)
    if not np.all(np.abs(poles) < 1):
        raise ValueError('LQR discreto non stabile')
    return k, poles


def design_lqr(dt=0.01, epsilon=1e-4):
    if not math.isfinite(epsilon) or epsilon <= 0 or not math.isfinite(dt) or dt <= 0:
        raise ValueError('Periodo ed epsilon devono essere positivi e finiti')
    robot = MujocoRobot()  # independent design copy; original XML untouched
    m, d = robot.m, robot.d
    # Dry joint friction has no useful infinitesimal input derivative at rest.
    # Remove ONLY this term for design, retain it in the simulated plant.
    m.dof_frictionloss[:] = 0
    m.opt.iterations = 100
    m.opt.tolerance = 1e-12
    equilibrium = -math.degrees(math.atan2(m.body('struttura_centrale').ipos[1],
                                         m.body('struttura_centrale').ipos[2]))
    robot.start_support(equilibrium)
    for _ in range(round(2/m.opt.timestep)):
        mujoco.mj_step(m, d)
    robot.release()
    base = d.qpos.copy()
    wheel_v = [int(m.joint(name).dofadr[0]) for name in ('perno_sx','perno_dx')]
    steps = round(dt/m.opt.timestep)
    if not math.isclose(steps*m.opt.timestep, dt):
        raise ValueError('Il periodo LQR deve essere multiplo del passo fisico')

    def transition(z, duty):
        mujoco.mj_resetData(m, d)
        d.eq_active[:] = False
        d.qpos[:] = base
        angle = math.radians(equilibrium) + z[0]
        d.qpos[robot.root_q+3:robot.root_q+7] = [math.cos(angle/2), -math.sin(angle/2), 0, 0]
        d.qvel[:] = 0
        d.qvel[robot.root_v+3] = -z[2]
        d.qvel[wheel_v] = z[1]
        # Initial no-slip tangent; contact dynamics remain active thereafter.
        d.qvel[robot.root_v+1] = 0.032*(z[2]-z[1])
        d.ctrl[:] = duty
        for _ in range(steps):
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        return np.array([math.radians(robot.true_pitch_deg()-equilibrium),
                         np.mean(d.qvel[wheel_v]), -d.qvel[robot.root_v+3]])

    a = np.column_stack([(transition(np.eye(3)[j]*epsilon,0)-
                          transition(-np.eye(3)[j]*epsilon,0))/(2*epsilon)
                         for j in range(3)])
    b = ((transition(np.zeros(3),epsilon)-transition(np.zeros(3),-epsilon))/(2*epsilon))[:,None]
    residual = transition(np.zeros(3),0)
    if np.linalg.norm(residual, np.inf) > 1e-4:
        raise ValueError('La postura nominale non è sufficientemente vicina all’equilibrio')
    controllability = np.column_stack([b, a@b, a@a@b])
    if np.linalg.matrix_rank(controllability) != 3:
        raise ValueError('Modello longitudinale non controllabile')
    q = np.diag(1/np.array([math.radians(3),0.5,math.radians(30)])**2)
    r = np.array([[1/0.15**2]])
    k, poles = solve_dlqr(a,b,q,r)
    comparison = json.loads((ROOT/'models/lqr/matrices_common_pwm_corrected.json').read_text())
    sim_a = np.array(comparison['A'])
    return dict(full_state_order=FULL_STATE_ORDER,
                reduced_state_indices=LONGITUDINAL_INDICES,
                state_order=['psi_error_rad','theta_dot_rad_s','psi_dot_rad_s'],
                Ad=a.tolist(), Bd_duty=b.tolist(), Q=q.tolist(), R=r.tolist(),
                K_duty=k[0].tolist(), K_pwm_percent=(100*k[0]).tolist(),
                K_full_pwm_percent=[0.,float(100*k[0,0]),0.,float(100*k[0,1]),float(100*k[0,2]),0.],
                closed_loop_poles=[[float(v.real),float(v.imag)] for v in poles],
                equilibrium_pitch_deg=equilibrium, nominal_step_residual=residual.tolist(),
                controllability_rank=3,
                bryson_limits=dict(pitch_rad=math.radians(3),wheel_rate_rad_s=0.5,
                                   pitch_rate_rad_s=math.radians(30),duty=0.15),
                open_loop_poles=[[float(v.real),float(v.imag)] for v in np.linalg.eigvals(a)],
                equivalent_continuous_poles=[[float(v.real),float(v.imag)]
                    for v in np.log(np.linalg.eigvals(a).astype(complex))/dt],
                dt_s=dt, finite_difference_epsilon=epsilon,
                design_joint_frictionloss=0, plant_joint_frictionloss='unchanged',
                method='central differences of 10 ms MuJoCo rollout; no-slip initial tangent, 3-state projection',
                simscape_comparison=dict(source=comparison['matrix_source'],
                    used_for_gains=False, mapping_verified=False,
                    continuous_poles=[[float(v.real),float(v.imag)] for v in np.linalg.eigvals(sim_a)]))


class LqrController(BalanceController):
    """Three measured states; wheel position weight is zero, yaw uncontrolled."""
    def __init__(self, config, design):
        super().__init__(config, design['equilibrium_pitch_deg'])
        self.k = np.array(design['K_pwm_percent'])
        self.design = design
        self.x = np.array([0., 0., np.nan, 0., 0., np.nan])
        self.previous_wheel_rate = 0.

    def reset(self, initial_pitch_deg):
        super().reset(initial_pitch_deg)
        self.x = np.array([0., math.radians(initial_pitch_deg), np.nan, 0., 0., np.nan])
        self.previous_wheel_rate = 0.

    def update(self, accel_pitch_deg, gyro_deg_s, left_joint_velocity, right_joint_velocity,
               dt, ideal_pitch_deg=None, desired_velocity_m_s=0.0, wheel_positions=None):
        # Reuse existing sensor estimator/odometry; replace the PD command.
        info = super().update(accel_pitch_deg, gyro_deg_s, left_joint_velocity,
                              right_joint_velocity, dt, ideal_pitch_deg, desired_velocity_m_s)
        wheel_rate = (left_joint_velocity+right_joint_velocity)/2
        if wheel_positions is None:
            self.x[0] += (self.previous_wheel_rate+wheel_rate)*dt/2
        else:
            if not np.isfinite(wheel_positions).all() or len(wheel_positions) != 2:
                raise ValueError('Posizioni encoder non valide')
            self.x[0] = float(np.mean(wheel_positions))
        self.previous_wheel_rate = wheel_rate
        self.x[1] = math.radians(self.pitch)
        self.x[3] = wheel_rate
        self.x[4] = math.radians(gyro_deg_s)
        # rho and rho_dot remain unavailable until a yaw estimator is added.
        z = self.x[LONGITUDINAL_INDICES].copy()
        z[0] -= math.radians(self.equilibrium_pitch_deg)
        z[1] += desired_velocity_m_s/self.config.wheel_radius_m
        raw = float(-self.k @ z)
        c = self.config
        pwm = max(-c.max_pwm,min(c.max_pwm,raw))
        applied = 0.0 if abs(pwm)<c.pwm_deadband_percent else math.copysign(
            math.floor(c.pwm_period_ticks*abs(pwm)/100)*100/c.pwm_period_ticks,pwm)
        info.update(pitch_target_deg=self.equilibrium_pitch_deg, pwm_requested_percent=raw,
                    pwm_percent=pwm,pwm_applied_percent=applied,saturated=abs(raw)>c.max_pwm,
                    lqr_psi_error_rad=float(z[0]),
                    lqr_theta_dot_error_rad_s=float(z[1]),lqr_psi_dot_rad_s=float(z[2]))
        info.update(zip(('x_theta_rad','x_psi_rad','x_rho_rad','x_theta_dot_rad_s',
                         'x_psi_dot_rad_s','x_rho_dot_rad_s'), self.x.tolist()))
        info['yaw_valid'] = False
        return info
