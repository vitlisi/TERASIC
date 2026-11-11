"""Explicit per-wheel equivalent dynamics; calibrated torque scale remains separate."""
import math

FIELDS = {'armature_kg_m2', 'damping_Nm_s_rad', 'frictionloss_Nm',
          'off_damping_Nm_s_rad', 'off_frictionloss_Nm'}


def configure_wheels(model, profiles):
    if not isinstance(profiles, dict) or set(profiles)-{'sx', 'dx'}:
        raise ValueError('Wheel profiles must use physical sx/dx')
    validated = {}
    for wheel, profile in profiles.items():
        if not isinstance(profile, dict) or set(profile)-FIELDS:
            raise ValueError('Unknown wheel dynamics field')
        i = int(model.joint('perno_'+wheel).dofadr[0])
        values = dict(armature_kg_m2=float(model.dof_armature[i]),
                      damping_Nm_s_rad=float(model.dof_damping[i]),
                      frictionloss_Nm=float(model.dof_frictionloss[i]))
        values.update(profile)
        values.setdefault('off_damping_Nm_s_rad', values['damping_Nm_s_rad'])
        values.setdefault('off_frictionloss_Nm', values['frictionloss_Nm'])
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or v < 0 for v in values.values()):
            raise ValueError('Finite nonnegative wheel dynamics required')
        validated[wheel] = values
    # Validate every profile before mutating the model.
    for wheel, values in validated.items():
        i = int(model.joint('perno_'+wheel).dofadr[0])
        model.dof_armature[i] = values['armature_kg_m2']
        model.dof_damping[i] = values['damping_Nm_s_rad']
        model.dof_frictionloss[i] = values['frictionloss_Nm']
    return validated


def update_wheels(model, pwm_percent, profiles):
    for wheel, values in profiles.items():
        i = int(model.joint('perno_'+wheel).dofadr[0])
        prefix = 'off_' if pwm_percent[0 if wheel == 'sx' else 1] == 0 else ''
        model.dof_damping[i] = values[prefix+'damping_Nm_s_rad']
        model.dof_frictionloss[i] = values[prefix+'frictionloss_Nm']
