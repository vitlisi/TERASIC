"""Imported discrete Simscape Hinf, adapted to MuJoCo sensor coordinates.

No synthesis or gain tuning here. Same estimator and PWM allocation as MIMO LQR.
"""
import json
import math
from pathlib import Path
import numpy as np
from motion_lqr import MotionLqrController

# Native theta is mean relative wheel angle, with forward s=r*(psi-theta).
# Simscape analytic theta is forward travel/r. All entries are deviations
# from the hold reference, so the equilibrium pitch and encoder origins cancel.
STATE_TRANSFORM = np.eye(6)
STATE_TRANSFORM[0, :2] = [-1., 1.]
STATE_TRANSFORM[3, 3:5] = [-1., 1.]


class DiscreteHinf:
    """u(k)=C xc(k)+D y(k), then xc(k+1)=A xc(k)+B y(k)."""
    def __init__(self, path):
        self.export = json.loads(Path(path).read_text())
        e = self.export
        if (e.get('schema_version') != 1 or e.get('measurement_order') !=
                ['theta','psi','phi','theta_dot','psi_dot','phi_dot'] or
                e.get('measurement_units') != ['rad']*3+['rad/s']*3 or
                e.get('output_order') != ['PWM_R','PWM_L'] or
                e.get('output_unit') != 'normalized_duty' or
                e.get('input_convention') != 'estimated_state_minus_reference_at_equilibrium' or
                e.get('measurement_delay_samples') != 1):
            raise ValueError('Convenzioni del controllore Hinf non supportate')
        self.a, self.b, self.c, self.d = (np.asarray(e[k], dtype=float)
                                        for k in ('Ak','Bk','Ck','Dk'))
        n = self.a.shape[0] if self.a.ndim == 2 else 0
        if (n == 0 or self.a.shape != (n,n) or self.b.shape != (n,6) or
                self.c.shape != (2,n) or self.d.shape != (2,6) or
                not all(np.isfinite(v).all() for v in (self.a,self.b,self.c,self.d))):
            raise ValueError('Dimensioni o valori matrici Hinf non validi')
        self.dt = float(e['dt_s'])
        self.limit = float(e['output_limit'])
        if not math.isfinite(self.dt) or self.dt <= 0 or not 0 < self.limit <= 1:
            raise ValueError('Periodo o limite Hinf non valido')
        self.initial = np.asarray(e['controller_initial_state'],dtype=float)
        self.initial_delay = np.asarray(e['measurement_delay_initial_state'],dtype=float)
        if (self.initial.shape != (n,) or self.initial_delay.shape != (6,) or
                not np.isfinite(self.initial).all() or not np.isfinite(self.initial_delay).all()):
            raise ValueError('Inizializzazione Hinf non valida')
        self.reset()

    def reset(self):
        self.state = self.initial.copy()
        self.delayed = self.initial_delay.copy()
        self.last_feedback = self.initial_delay.copy()

    def step(self, error, dt):
        error = np.asarray(error,dtype=float)
        if error.shape != (6,) or not np.isfinite(error).all():
            raise ValueError('Richiesti sei errori di stato finiti')
        if not math.isclose(dt,self.dt,rel_tol=0,abs_tol=1e-12):
            raise ValueError('Periodo diverso da quello della sintesi Hinf')
        measurement = STATE_TRANSFORM @ error
        self.last_feedback = self.delayed.copy()
        duty_right_left = self.c @ self.state + self.d @ self.delayed
        state_next = self.a @ self.state + self.b @ self.delayed
        if not np.isfinite(duty_right_left).all() or not np.isfinite(state_next).all():
            raise ValueError('Stato o uscita Hinf non finiti')
        self.state = state_next
        self.delayed = measurement.copy()
        return duty_right_left[::-1].copy() # MuJoCo input order: SX, DX


class HinfController(MotionLqrController):
    def __init__(self, config, design, reference, controller_path):
        self.hinf = DiscreteHinf(controller_path)
        e = self.hinf.export
        for key,native in [('wheel_track_m','track_width_m'),('wheel_radius_m','wheel_radius_m')]:
            # Preserve original synthesized matrices/provenance. The confirmed
            # physical track is 164.44 mm, 0.04 mm above the old nominal export.
            original_track_correction = (key == 'wheel_track_m' and
                math.isclose(float(e[key]), .1644, rel_tol=0, abs_tol=1e-12) and
                math.isclose(design[native], .16444, rel_tol=0, abs_tol=1e-12))
            if not original_track_correction and not math.isclose(float(e[key]),design[native],rel_tol=0,abs_tol=1e-9):
                raise ValueError('Geometria Hinf diversa da MuJoCo: '+key)
        # The exported limit records the original experiment/synthesis scale.
        # Runtime saturation is explicitly chosen by config.max_pwm; matrices
        # and the original export remain unchanged for comparison experiments.
        super().__init__(config,design,reference)

    def reset(self, initial_pitch_deg):
        super().reset(initial_pitch_deg)
        self.hinf.reset()

    def command(self, error, dutyref, dt):
        if np.any(np.abs(dutyref)>1e-12):
            raise ValueError('Hinf supporta solo stabilizzazione senza feedforward')
        return 100*self.hinf.step(error,dt)

    def update(self, *args, **kwargs):
        info = super().update(*args, **kwargs)
        info.update(('hinf_feedback_'+str(i),float(v)) for i,v in enumerate(self.hinf.last_feedback))
        info.update(('hinf_xc_'+str(i),float(v)) for i,v in enumerate(self.hinf.state))
        return info
