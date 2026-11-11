"""Explicit experimental overrides; never writes the nominal model or gains."""
import math

import mujoco
from wheel_dynamics import configure_wheels, update_wheels


def apply_parameters(robot, parameters):
    allowed = {'armature_scale', 'damping_scale', 'friction_scale', 'torque_scale',
               'body_inertia_scale', 'imu', 'actuation_delay_ms', 'release_delay_ms', 'wheels'}
    if set(parameters)-allowed:
        raise ValueError('Unknown experimental parameters: '+str(set(parameters)-allowed))
    for key in allowed-{'imu', 'wheels'}:
        default = 0. if key.endswith('_ms') else 1.
        value = parameters.get(key, default)
        if not isinstance(value, (float, int)) or not math.isfinite(value) or value<0:
            raise ValueError('Invalid parameter '+key)
        if key in ('armature_scale', 'torque_scale', 'body_inertia_scale') and value<=0:
            raise ValueError('Positive parameter required: '+key)
        if key.endswith('_ms') and value>100:
            raise ValueError('Delay at most 100 ms')
    if parameters.get('imu', 'mpu6500') not in ('mpu6500', 'quantized'):
        raise ValueError('Unsupported experimental sensor mode')
    for name in ('perno_sx', 'perno_dx'):
        index = int(robot.m.joint(name).dofadr[0])
        robot.m.dof_armature[index] *= parameters.get('armature_scale', 1.)
        robot.m.dof_damping[index] *= parameters.get('damping_scale', 1.)
        robot.m.dof_frictionloss[index] *= parameters.get('friction_scale', 1.)
    robot.m.body_inertia[robot.body_id] *= parameters.get('body_inertia_scale', 1.)
    robot.torque_per_duty *= parameters.get('torque_scale', 1.)
    for name in ('motore_sx', 'motore_dx'):
        robot.m.actuator(name).gear[0] = -robot.torque_per_duty
    if 'wheels' in parameters:
        robot.wheel_dynamics = configure_wheels(robot.m, parameters['wheels'])
        update_wheels(robot.m, robot.d.ctrl*100, robot.wheel_dynamics)
    mujoco.mj_setConst(robot.m, robot.d)
    mujoco.mj_forward(robot.m, robot.d)
    return dict(requested=parameters,
                armature_kg_m2=[float(robot.m.dof_armature[int(robot.m.joint(n).dofadr[0])]) for n in ('perno_sx', 'perno_dx')],
                damping_Nm_s_rad=[float(robot.m.dof_damping[int(robot.m.joint(n).dofadr[0])]) for n in ('perno_sx', 'perno_dx')],
                frictionloss_Nm=[float(robot.m.dof_frictionloss[int(robot.m.joint(n).dofadr[0])]) for n in ('perno_sx', 'perno_dx')],
                torque_per_full_duty_Nm=robot.torque_per_duty,
                wheel_profiles=getattr(robot, 'wheel_dynamics', {}),
                physically_identified=False)
