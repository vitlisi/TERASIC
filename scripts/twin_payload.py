"""Aggregate a fixed centered point payload without editing the nominal XML."""
import math

import mujoco
import numpy as np


def body_inertia(model, body_id):
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.body_iquat[body_id])
    rotation = rotation.reshape(3, 3)
    return rotation@np.diag(model.body_inertia[body_id])@rotation.T


def add_centered_payload(robot, mass_kg=0., height_m=.10):
    """Point-payload COM at [0,0,height] from wheel-axis midpoint, body frame.

    The point approximation omits the payload's own centroidal inertia.
    Parallel-axis contributions and the combined COM are included exactly.
    """
    if (not math.isfinite(mass_kg) or not 0<=mass_kg<=.1 or
            not math.isfinite(height_m) or not 0<=height_m<=.3):
        raise ValueError('Payload mass [0,.1] kg and height [0,.3] m required')
    model = robot.m
    index = robot.body_id
    mass = float(model.body_mass[index])
    center = model.body_ipos[index].copy()
    inertia = body_inertia(model, index)
    point = np.array([0., 0., height_m])
    if mass_kg:
        total = mass+mass_kg
        delta = point-center
        combined_inertia = inertia+mass*mass_kg/total*(np.dot(delta, delta)*np.eye(3)-np.outer(delta, delta))
        eigenvalues, axes = np.linalg.eigh(combined_inertia)
        if eigenvalues.min()<=0:
            raise ValueError('Nonpositive payload aggregate inertia')
        if np.linalg.det(axes)<0:
            axes[:, 0] *= -1
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, axes.ravel())
        model.body_mass[index] = total
        model.body_ipos[index] = (mass*center+mass_kg*point)/total
        model.body_inertia[index] = eigenvalues
        model.body_iquat[index] = quat
        mujoco.mj_setConst(model, robot.d)
        mujoco.mj_forward(model, robot.d)
    return dict(mass_kg=mass_kg, position_body_m=point.tolist(),
                representation='fixed centered point payload; own centroidal inertia omitted',
                height_measured=False, nominal_body_mass_kg=mass,
                combined_body_mass_kg=float(model.body_mass[index]),
                nominal_com_body_m=center.tolist(), combined_com_body_m=model.body_ipos[index].tolist(),
                nominal_inertia_body_kg_m2=inertia.tolist(),
                combined_inertia_body_kg_m2=body_inertia(model, index).tolist(),
                reference_policy='retain nominal sensor frame, fixed pitch reference and initial pose')
