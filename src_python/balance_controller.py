"""Controller esclusivo della simulazione. Nessuna dipendenza dal C++ reale.

PD interno: PWM = Kp*(pitch-target) + Kd*gyro.
Anello esterno: target = pitch_equilibrio - Kv*(v-v_des) - Kx*(x-x_des).
La posizione è odometria encoder, non posizione privilegiata MuJoCo.
"""
from dataclasses import dataclass, asdict
import math


@dataclass
class ControllerConfig:
    kp: float = 83.0  # punti percentuali PWM / grado
    kd: float = 3.74  # punti percentuali PWM / (grado/s)
    kv: float = 8.0  # gradi / (m/s)
    kx: float = 2.0  # gradi / m
    max_pwm: float = 15.0
    max_pitch_offset_deg: float = 3.0
    filter_tau_s: float = 0.5
    wheel_radius_m: float = 0.032
    pwm_deadband_percent: float = 0.5
    pwm_period_ticks: int = 50000

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('I parametri devono essere finiti')
        if not 0<=self.max_pwm<=100 or self.max_pitch_offset_deg<0 or self.filter_tau_s<0 or self.wheel_radius_m<=0 or self.pwm_period_ticks<=0 or self.pwm_deadband_percent<0:
            raise ValueError('Limite PWM 0..100 e parametri fisici validi richiesti')


class BalanceController:
    def __init__(self, config, equilibrium_pitch_deg):
        self.config=config
        self.equilibrium_pitch_deg=equilibrium_pitch_deg
        self.pitch=0.0
        self.position=0.0
        self.desired_position=0.0
        self.previous_velocity=0.0

    def reset(self, initial_pitch_deg):
        self.pitch=initial_pitch_deg
        self.position=0.0
        self.desired_position=0.0
        self.previous_velocity=0.0

    def update(self, accel_pitch_deg, gyro_deg_s, left_joint_velocity, right_joint_velocity,
               dt, ideal_pitch_deg=None, desired_velocity_m_s=0.0):
        c=self.config
        if dt<=0 or not math.isfinite(dt): raise ValueError('dt deve essere positivo e finito')
        if not all(math.isfinite(v) for v in (accel_pitch_deg,gyro_deg_s,left_joint_velocity,right_joint_velocity,desired_velocity_m_s)) or (ideal_pitch_deg is not None and not math.isfinite(ideal_pitch_deg)):
            raise ValueError('Sensori non finiti')
        if ideal_pitch_deg is not None:
            self.pitch=ideal_pitch_deg
        else:
            alpha=math.exp(-dt/c.filter_tau_s) if c.filter_tau_s else 0.0
            self.pitch=alpha*(self.pitch+gyro_deg_s*dt)+(1-alpha)*accel_pitch_deg
        # Encoder relativi al telaio: aggiunge la rotazione del telaio per
        # ricostruire la velocità di rotolamento al suolo. Asse IMU Y=-X corpo.
        velocity=c.wheel_radius_m*(math.radians(gyro_deg_s)-(left_joint_velocity+right_joint_velocity)/2)
        self.position+=(velocity+self.previous_velocity)*dt/2
        self.desired_position+=desired_velocity_m_s*dt
        self.previous_velocity=velocity
        offset=-c.kv*(velocity-desired_velocity_m_s)-c.kx*(self.position-self.desired_position)
        offset=max(-c.max_pitch_offset_deg,min(c.max_pitch_offset_deg,offset))
        target=self.equilibrium_pitch_deg+offset
        raw=c.kp*(self.pitch-target)+c.kd*gyro_deg_s
        pwm=max(-c.max_pwm,min(c.max_pwm,raw))
        applied=0.0 if abs(pwm)<c.pwm_deadband_percent else math.copysign(math.floor(c.pwm_period_ticks*abs(pwm)/100)*100/c.pwm_period_ticks,pwm)
        return dict(pitch_est_deg=self.pitch,pitch_target_deg=target,velocity_encoder_m_s=velocity,
                    position_encoder_m=self.position,pwm_requested_percent=raw,
                    pwm_percent=pwm,pwm_applied_percent=applied,
                    saturated=abs(raw)>c.max_pwm)
