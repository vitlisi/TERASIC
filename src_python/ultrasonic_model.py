"""Sampled geometric HC-SR04 cone; datasheet bounds, explicit approximations."""
import json
import math
from pathlib import Path

import mujoco
import numpy as np

DEFAULT_CONFIG = Path(__file__).resolve().parents[1]/'config/mujoco_ultrasonic_hcsr04.json'


def draw_ultrasonic_cone(scene, model, data, cone, clear=False):
    """Decorative wire cone, up to 1 m; never part of sensor ray casting."""
    if clear:
        scene.ngeom = 0
    sid = cone.site_id
    origin = data.site_xpos[sid].copy()
    rotation = data.site_xmat[sid].reshape(3, 3)
    sample = cone.sample or {}
    raw = sample.get('ultrasonic_cone_distance_m', -1.)
    length = min(cone.maximum, 1., raw if raw > 0 else 1.)
    angle = math.radians(cone.half_angle_deg)
    phi = np.arange(32)*2*math.pi/32
    rim = np.column_stack((np.sin(angle)*np.cos(phi), np.sin(angle)*np.sin(phi),
                           np.full(32, math.cos(angle)))) @ rotation.T*length+origin

    def line(first, second, color, width=.001):
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                           np.zeros(3), np.zeros(3), np.eye(3).ravel(), color)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, first, second)
        geom.category = mujoco.mjtCatBit.mjCAT_DECOR
        scene.ngeom += 1

    for index in range(32):
        line(rim[index], rim[(index+1)%32], [0., .9, 1., .65])
        if index % 4 == 0:
            line(origin, rim[index], [0., .9, 1., .45])
    line(origin, origin+rotation[:, 2]*length, [0., .9, 1., .8])
    if raw > 0 and math.isfinite(sample.get('ultrasonic_hit_angle_deg', math.nan)):
        # Stored world point stays consistent with the sample/hold timestamp.
        if cone.hit_point is not None:
            line(origin, cone.hit_point, [1., .8, .05, 1.], .0015)


class UltrasonicCone:
    def __init__(self, model, path=DEFAULT_CONFIG):
        self.config = json.loads(Path(path).read_text())
        c = self.config
        self.minimum, self.maximum = float(c['min_range_m']), float(c['max_range_m'])
        self.period, self.speed = float(c['sample_period_s']), float(c['sound_speed_m_s'])
        angle = float(c['measuring_angle_deg'])
        interpretation = c['angle_interpretation']
        if interpretation not in ('full_aperture', 'half_angle'):
            raise ValueError('Interpretazione angolo ultrasuoni non valida')
        self.half_angle_deg = angle/2 if interpretation == 'full_aperture' else angle
        rings, azimuths = c['radial_rings'], c['azimuth_samples']
        if (not all(math.isfinite(v) for v in (self.minimum, self.maximum, self.period, self.speed, angle))
                or not 0 < self.minimum < self.maximum or self.period <= .06 or self.speed <= 0
                or not 0 < self.half_angle_deg < 90
                or type(rings) is not int or type(azimuths) is not int
                or not 1 <= rings <= 64 or not 4 <= azimuths <= 256):
            raise ValueError('Parametri cono ultrasuoni non validi')
        directions = [[0., 0., 1.]]
        for radius in range(1, rings+1):
            polar = math.radians(self.half_angle_deg)*radius/rings
            for azimuth in np.arange(azimuths)*2*math.pi/azimuths:
                directions.append([math.sin(polar)*math.cos(azimuth),
                                   math.sin(polar)*math.sin(azimuth), math.cos(polar)])
        self.directions = np.asarray(directions)
        self.site_id = model.site('ultrasonic_site').id
        self.body_id = int(model.site_bodyid[self.site_id])
        self.sample = None
        self.hit_point = None

    def read(self, model, data):
        now = float(data.time)
        previous = self.sample
        if previous is None or now < previous['ultrasonic_sample_time_s'] or (
                now-previous['ultrasonic_sample_time_s'] >= self.period-1e-9):
            rotation = data.site_xmat[self.site_id].reshape(3, 3)
            directions = np.ascontiguousarray(self.directions @ rotation.T).reshape(-1)
            count = len(self.directions)
            distances = np.full(count, -1., dtype=np.float64)
            ids = np.full(count, -1, dtype=np.int32)
            mujoco.mj_multiRay(model, data, data.site_xpos[self.site_id], directions,
                              None, True, self.body_id, ids, distances, None,
                              count, self.maximum+1e-9)
            hits = np.flatnonzero((distances >= 0) & (distances <= self.maximum+1e-12))
            index = int(hits[np.argmin(distances[hits])]) if len(hits) else -1
            raw = float(distances[index]) if index >= 0 else -1.
            self.hit_point = (data.site_xpos[self.site_id]+raw*directions.reshape(-1, 3)[index]
                              if index >= 0 else None)
            valid = self.minimum-1e-12 <= raw <= self.maximum+1e-12
            distance = min(self.maximum, max(self.minimum, raw)) if valid else math.nan
            self.sample = dict(ultrasonic_distance_m=distance, ultrasonic_valid=int(valid),
                ultrasonic_echo_us=2*distance/self.speed*1e6 if valid else math.nan,
                ultrasonic_sample_time_s=now, ultrasonic_cone_distance_m=raw,
                ultrasonic_hit_geom_id=int(ids[index]) if index >= 0 else -1,
                ultrasonic_hit_angle_deg=math.degrees(math.acos(float(np.clip(
                    self.directions[index, 2], -1, 1)))) if index >= 0 else math.nan,
                ultrasonic_hit_rays=len(hits))
        return dict(self.sample,
                    ultrasonic_ray_distance_m=float(data.sensor('ultrasonic_range').data[0]))
