"""Editable XY cubic-spline path, timed reference and visual-only ground trace."""
import json
import math
from pathlib import Path
import mujoco
import numpy as np
from motion_lqr import wrap_angle


class PathPlan:
    def __init__(self, config):
        self.config = config
        if not isinstance(config, dict) or any(not isinstance(config.get(key, {}), dict)
                for key in ('guidance', 'limits', 'visual')):
            raise ValueError('Path: oggetto JSON con guidance/limits/visual come oggetti richiesto')
        if config.get('schema_version') != 1 or config.get('units') != 'm' or config.get('frame') != 'local_to_release':
            raise ValueError('Path: richiesti schema_version=1, units=m, frame=local_to_release')
        self.points = np.asarray(config['waypoints_xy_m'], dtype=float)
        if (self.points.ndim != 2 or self.points.shape[1] != 2 or len(self.points) < 2
                or len(self.points) > 200 or not np.isfinite(self.points).all()):
            raise ValueError('Path: da 2 a 200 punti X/Y finiti richiesti')
        if np.linalg.norm(self.points[0]) > 1e-9:
            raise ValueError('Path: il punto iniziale deve essere [0,0]')
        h = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        if np.min(h) < .001:
            raise ValueError('Path: punti consecutivi distanti almeno 1 mm')
        self.start = float(config['start_s'])
        self.duration = float(config['travel_duration_s'])
        self.end_heading = math.radians(float(config.get('end_heading_deg', 0.)))
        guide = config.get('guidance', {})
        self.lateral_gain = float(guide.get('lateral_gain_rad_per_m', 4.))
        self.heading_limit = math.radians(float(guide.get('max_heading_correction_deg', 20.)))
        limits = config.get('limits', {})
        speed_limit = limits.get('max_speed_m_s')
        yaw_limit = limits.get('max_yaw_rate_deg_s')
        self.max_speed = None if speed_limit is None else float(speed_limit)
        self.max_yaw_rate = None if yaw_limit is None else math.radians(float(yaw_limit))
        values = [self.start, self.duration, self.end_heading, self.lateral_gain,
                  self.heading_limit]
        enabled_limits = [value for value in (self.max_speed,self.max_yaw_rate) if value is not None]
        if (not np.isfinite(values+enabled_limits).all() or self.start < 0 or self.duration <= 0
                or min(values[3:]) <= 0 or any(value <= 0 for value in enabled_limits)):
            raise ValueError('Path: tempi, guadagni e limiti non validi')
        self.knots = np.r_[0., np.cumsum(h)]
        count = len(self.points)
        matrix, rhs = np.zeros((count, count)), np.zeros((count, 2))
        matrix[0, :2] = [2*h[0], h[0]]
        rhs[0] = 6*((self.points[1]-self.points[0])/h[0]-[0., 1.])
        for i in range(1, count-1):
            matrix[i, i-1:i+2] = [h[i-1], 2*(h[i-1]+h[i]), h[i]]
            rhs[i] = 6*((self.points[i+1]-self.points[i])/h[i]-(self.points[i]-self.points[i-1])/h[i-1])
        matrix[-1, -2:] = [h[-1], 2*h[-1]]
        end_tangent = np.array([-math.sin(self.end_heading), math.cos(self.end_heading)])
        rhs[-1] = 6*(end_tangent-(self.points[-1]-self.points[-2])/h[-1])
        second = np.linalg.solve(matrix, rhs)
        self.a = self.points[:-1]
        self.b = np.diff(self.points, axis=0)/h[:, None]-h[:, None]*(2*second[:-1]+second[1:])/6
        self.c = second[:-1]/2
        self.d = (second[1:]-second[:-1])/(6*h[:, None])
        self.q_grid = np.linspace(0, self.knots[-1], 4001)
        xy, tangent, _ = self.geometry(self.q_grid)
        if np.min(np.linalg.norm(tangent, axis=1)) < .05:
            raise ValueError('Path: cuspide/quasi arresto geometrico; modificare i punti')
        self.arc_grid = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        self.length = float(self.arc_grid[-1])
        # Unwrap once geometrically; keep heading consistent even across +/-pi.
        self.heading_grid = np.unwrap(np.arctan2(-tangent[:, 0], tangent[:, 1]))
        self.line_width = float(config.get('visual', {}).get('line_width_m', .004))
        self.rgba = np.asarray(config.get('visual', {}).get('rgba', [1., .75, .05, .9]), dtype=float)
        if (not math.isfinite(self.line_width) or not 0 < self.line_width <= .02 or
                self.rgba.shape != (4,) or not np.isfinite(self.rgba).all() or
                np.any(self.rgba < 0) or np.any(self.rgba > 1)):
            raise ValueError('Path: colore RGBA 0..1 e larghezza 0..0.02 m richiesti')
        self.samples = self.trajectory(np.linspace(self.start, self.start+self.duration, 4001))
        if not all(np.isfinite(value).all() for value in self.samples.values()):
            raise ValueError('Path: riferimento non finito; durata/geometria numericamente non valida')
        if self.max_speed is not None and np.max(self.samples['speed']) > self.max_speed+1e-9:
            raise ValueError('Path: velocita eccessiva; aumentare travel_duration_s')
        if self.max_yaw_rate is not None and np.max(np.abs(self.samples['yaw_rate'])) > self.max_yaw_rate+1e-9:
            raise ValueError('Path: yaw rate eccessivo; allargare le curve o aumentare travel_duration_s')

    @classmethod
    def from_file(cls, filename, duration_override=None):
        config = json.loads(Path(filename).read_text())
        if duration_override is not None:
            if not isinstance(config,dict):
                raise ValueError('Path: oggetto JSON richiesto')
            config['travel_duration_s'] = duration_override
        return cls(config)

    def geometry(self, q):
        q = np.clip(np.asarray(q), 0., self.knots[-1])
        index = np.clip(np.searchsorted(self.knots, q, side='right')-1, 0, len(self.a)-1)
        s = (q-self.knots[index])[..., None]
        a, b, c, d = self.a[index], self.b[index], self.c[index], self.d[index]
        return a+s*(b+s*(c+s*d)), b+s*(2*c+3*s*d), 2*c+6*s*d

    def trajectory(self, time):
        f = np.clip((np.asarray(time)-self.start)/self.duration, 0., 1.)
        q = self.knots[-1]*(10*f**3-15*f**4+6*f**5)
        qdot = self.knots[-1]/self.duration*(30*f**2-60*f**3+30*f**4)
        qddot = self.knots[-1]/self.duration**2*(60*f-180*f**2+120*f**3)
        xy, tangent, second = self.geometry(q)
        norm = np.linalg.norm(tangent, axis=-1)
        speed = norm*qdot
        acceleration = norm*qddot+np.sum(tangent*second, axis=-1)/norm*qdot**2
        yaw_rate = (tangent[..., 0]*second[..., 1]-tangent[..., 1]*second[..., 0])/norm**2*qdot
        return dict(xy=xy, speed=speed, acceleration=acceleration,
            yaw=np.interp(q, self.q_grid, self.heading_grid), yaw_rate=yaw_rate,
            distance=np.interp(q, self.q_grid, self.arc_grid), q=q)

    def visual_points(self, spacing=.008):
        count = min(700, max(2, math.ceil(self.length/spacing)+1))
        q = np.interp(np.linspace(0, self.length, count), self.arc_grid, self.q_grid)
        return self.geometry(q)[0]

    def obstacle_pose(self, progress=.65, lateral_m=0.):
        """Pose at an arc-length fraction; positive lateral offset is left."""
        if not math.isfinite(progress) or not 0 < progress <= 1 or not math.isfinite(lateral_m):
            raise ValueError('Posizione ostacolo: progressione in (0,1] e offset finito')
        q = float(np.interp(progress*self.length, self.arc_grid, self.q_grid))
        xy, tangent, _ = self.geometry(q)
        yaw = math.atan2(-float(tangent[0]), float(tangent[1]))
        xy = xy+np.array([-math.cos(yaw), -math.sin(yaw)])*lateral_m
        return dict(local_xy_m=xy.tolist(), local_yaw_rad=yaw,
                    arc_progress=progress, lateral_offset_m=lateral_m)


class PathReference:
    uses_pose = True

    def __init__(self, design, plan):
        self.plan = plan
        self.stop_request = None
        self.stop_target = None
        self.radius = design['wheel_radius_m']
        self.equilibrium = math.radians(design['equilibrium_pitch_deg'])
        self.dt = design['dt_s']
        a, b = np.array(design['Ad']), np.array(design['Bd_duty'])
        inverse = np.linalg.inv(np.column_stack((a[3:, 1], b.sum(axis=1)[3:], (b[:, 1]-b[:, 0])[3:])))
        self.times = np.linspace(0., plan.start+plan.duration+1.,
            math.ceil((plan.start+plan.duration+1.)/self.dt)+1)
        nominal = plan.trajectory(self.times)
        yaw_acceleration = np.gradient(nominal['yaw_rate'], self.times)
        rhs = np.column_stack((-nominal['acceleration']*self.dt/self.radius,
                                np.zeros(len(self.times)), yaw_acceleration*self.dt))
        rhs -= np.outer(-nominal['speed']/self.radius, (a-np.eye(6))[3:, 3])
        rhs -= np.outer(nominal['yaw_rate'], (a-np.eye(6))[3:, 5])
        solution = rhs @ inverse.T
        self.pitch = self.equilibrium+solution[:, 0]
        self.pitch_rate = np.gradient(self.pitch, self.times)
        self.duty = np.column_stack((solution[:, 1]-solution[:, 2], solution[:, 1]+solution[:, 2]))

    def request_stop(self, time, distance):
        """Latch sensor-triggered stop; no automatic restart on missing echoes."""
        if self.stop_request is None:
            self.stop_request = dict(time_s=float(time), distance_m=float(distance))

    def sample(self, time, theta_origin, pitch_origin, pose, state):
        if self.stop_request is not None:
            if self.stop_target is None:
                nominal = self.plan.trajectory(time)
                distance = self.radius*(state[1]-pitch_origin-state[0]+theta_origin)
                xref = np.array([state[0]+self.equilibrium-state[1], self.equilibrium,
                                 state[2], 0., 0., 0.])
                self.stop_target = (xref, np.asarray(pose).copy(), distance,
                                    float(nominal['q']/self.plan.knots[-1]))
            xref, target_pose, distance, progress = self.stop_target
            displacement = target_pose-np.asarray(pose)
            yaw = float(xref[2])
            return xref.copy(), np.zeros(2), dict(distance_ref_m=distance,
                velocity_ref_m_s=0., acceleration_ref_m_s2=0., yaw_ref_rad=yaw,
                path_ref_x_m=float(target_pose[0]), path_ref_y_m=float(target_pose[1]),
                path_nominal_yaw_rad=yaw, path_yaw_rate_ref_rad_s=0.,
                path_error_forward_m=float(np.array([-math.sin(yaw), math.cos(yaw)]) @ displacement),
                path_error_lateral_m=float(np.array([-math.cos(yaw), -math.sin(yaw)]) @ displacement),
                path_pose_error_m=float(np.linalg.norm(displacement)),
                path_heading_correction_rad=0., path_distance_command_m=distance,
                path_progress=progress, ultrasonic_stop_active=1,
                path_reference_time_s=self.stop_request['time_s'])
        nominal = self.plan.trajectory(time)
        yaw = float(nominal['yaw'])
        displacement = nominal['xy']-np.asarray(pose)
        lateral = float(np.array([-math.cos(yaw), -math.sin(yaw)]) @ displacement)
        forward = float(np.array([-math.sin(yaw), math.cos(yaw)]) @ displacement)
        # Geometric lateral guidance. The inner LQR regulates projected forward
        # pose error, velocity and corrected heading together with balance.
        correction = float(np.clip(self.plan.lateral_gain*lateral,
                                     -self.plan.heading_limit, self.plan.heading_limit))
        pitch = float(np.interp(time, self.times, self.pitch))
        pitch_rate = float(np.interp(time, self.times, self.pitch_rate))
        v, distance, yaw_rate = (float(nominal[key]) for key in ('speed', 'distance', 'yaw_rate'))
        measured_distance = self.radius*(state[1]-pitch_origin-state[0]+theta_origin)
        commanded_distance = measured_distance+forward
        xref = np.array([theta_origin+pitch-pitch_origin-commanded_distance/self.radius,
                        pitch, yaw+correction, pitch_rate-v/self.radius, pitch_rate, yaw_rate])
        duty = np.array([np.interp(time, self.times, self.duty[:, i]) for i in range(2)])
        return xref, duty, dict(distance_ref_m=distance, velocity_ref_m_s=v,
            acceleration_ref_m_s2=float(nominal['acceleration']), yaw_ref_rad=yaw+correction,
            path_ref_x_m=float(nominal['xy'][0]), path_ref_y_m=float(nominal['xy'][1]),
            path_nominal_yaw_rad=yaw, path_yaw_rate_ref_rad_s=yaw_rate,
            path_error_forward_m=forward, path_error_lateral_m=lateral,
            path_pose_error_m=float(np.linalg.norm(displacement)),
            path_heading_correction_rad=correction,
            path_distance_command_m=float(commanded_distance),
            path_progress=float(nominal['q']/self.plan.knots[-1]),
            ultrasonic_stop_active=0, path_reference_time_s=time)


def draw_ground_path(scene, plan, origin_xy=(0., 0.), heading=0., free_time=0., clear=True):
    """Render geoms only: no mjModel bodies, contacts, mass, or forces."""
    local = plan.visual_points()
    rotation = np.array([[math.cos(heading), -math.sin(heading)],
                         [math.sin(heading), math.cos(heading)]])
    points = local @ rotation.T+np.asarray(origin_xy)
    if clear:
        scene.ngeom = 0
    for first, second in zip(points[:-1], points[1:]):
        if scene.ngeom >= scene.maxgeom-3:
            raise ValueError('Capacita geometrie grafiche insufficiente per il path')
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3), np.zeros(3), np.eye(3).ravel(), plan.rgba)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, plan.line_width/2,
            np.r_[first, .0025], np.r_[second, .0025])
        geom.category = mujoco.mjtCatBit.mjCAT_DECOR
        scene.ngeom += 1
    reference = plan.trajectory(free_time)['xy'] @ rotation.T+np.asarray(origin_xy)
    for position, color, label in [(points[0], [0., 1., .3, .95], 'A'),
                                    (points[-1], [1., .2, .2, .95], 'B'),
                                    (reference, [.1, .8, 1., .95], '')]:
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
            [.008, .008, .008], np.r_[position, .006], np.eye(3).ravel(), color)
        geom.category = mujoco.mjtCatBit.mjCAT_DECOR
        geom.label = label
        scene.ngeom += 1
