"""Robot01 equivalent joint friction: distinct wheel/direction Stribeck branches.

The MuJoCo friction-loss constraint supplies the signed resisting torque,
including static holding. This module updates its magnitude before each physics
step; it never clamps wheel position, velocity, PWM or body motion. Effective
viscous damping already includes the calibrated motor velocity losses.
"""
import math


def configure(model,profile):
    if profile['robot']!='robot01' or set(profile['wheels'])!={'sx','dx'}:raise ValueError('Wrong motor hypothesis')
    validated=[]
    for wheel,p in profile['wheels'].items():
        values=[p['armature_kg_m2'],p['total_inertia_kg_m2'],p['off_damping_Nm_s_rad'],p['off_frictionloss_Nm']]
        if set(p['branches'])!={'forward','reverse'} or p['total_inertia_kg_m2']<=p['armature_kg_m2']:raise ValueError('Invalid wheel branches or total inertia')
        for branch in p['branches'].values():
            values += [branch['damping_Nm_s_rad'],branch['coulomb_Nm'],branch['breakaway_Nm'],branch['stribeck_velocity_rad_s']]
            if branch['stribeck_velocity_rad_s']<=0 or branch['breakaway_Nm']<branch['coulomb_Nm']:raise ValueError('Invalid static/kinetic friction ordering')
        if any(not math.isfinite(v) or v<0 for v in values):raise ValueError('Invalid nonnegative motor parameter')
        validated.append((int(model.joint('perno_'+wheel).dofadr[0]),p['armature_kg_m2']))
    for index,armature in validated:model.dof_armature[index]=armature


def friction_parameters(wheel_profile,physical_velocity,physical_pwm):
    if not math.isfinite(physical_velocity) or not math.isfinite(physical_pwm):raise ValueError('Nonfinite wheel state')
    if physical_pwm==0:return wheel_profile['off_damping_Nm_s_rad'],wheel_profile['off_frictionloss_Nm']
    direction='forward' if physical_velocity>.001 else 'reverse' if physical_velocity<-.001 else 'forward' if physical_pwm>0 else 'reverse'
    b=wheel_profile['branches'][direction]
    ratio=physical_velocity/b['stribeck_velocity_rad_s']
    dry=b['coulomb_Nm']+(b['breakaway_Nm']-b['coulomb_Nm'])*math.exp(-ratio*ratio)
    return b['damping_Nm_s_rad'],dry


def update(model,data,pwm,profile):
    for index,wheel in enumerate(['sx','dx']):
        adr=int(model.joint('perno_'+wheel).dofadr[0])
        damping,dry=friction_parameters(profile['wheels'][wheel],-float(data.qvel[adr]),float(pwm[index]))
        model.dof_damping[adr]=damping;model.dof_frictionloss[adr]=dry
