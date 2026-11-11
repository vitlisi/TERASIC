"""Optional MPU-6500 register-output approximation and motor encoder chain."""
from collections import deque
import math
import numpy as np


class Mpu6500:
    def __init__(self,config,seed=0):
        self.config=config
        self.rng=np.random.default_rng(seed)
        self.time=None
        self.next_output=0.
        self.filtered=None
        self.history=deque()
        self.accel=np.zeros(3)
        self.gyro=np.zeros(3)
        self.accel_raw=np.zeros(3,dtype=int)
        self.gyro_raw=np.zeros(3,dtype=int)
        self.saturated=False

    def update(self,time,accel,gyro):
        if self.time is not None and time<=self.time+1e-12:
            return
        c=self.config
        measurement=np.r_[np.array(accel)/9.81,np.degrees(gyro)]
        measurement+=np.r_[c['accel_bias_g'],c['gyro_bias_deg_s']]
        dt=1/c['internal_rate_hz'] if self.time is None else time-self.time
        if not np.isfinite(measurement).all() or dt<=0:
            raise ValueError('Ingresso IMU non valido')
        densities=np.r_[np.full(3,c['accel_noise_density_g_sqrt_hz']),
                        np.full(3,c['gyro_noise_density_deg_s_sqrt_hz'])]
        noisy=measurement+self.rng.normal(size=6)*densities/math.sqrt(2*dt)
        fc=np.r_[np.full(3,c['accel_bandwidth_hz']),np.full(3,c['gyro_bandwidth_hz'])]
        alpha=np.exp(-2*np.pi*fc*dt)
        if self.filtered is None:
            self.filtered=measurement.copy()
        self.filtered=alpha*self.filtered+(1-alpha)*noisy
        self.history.append((time,self.filtered.copy()))
        self.time=time
        if time+1e-9>=self.next_output:
            times=np.array([t for t,_ in self.history])
            values=np.array([v for _,v in self.history])
            # Discrete one-pole delay at DC, plus transport delay = quoted total.
            pole_delay=dt*alpha/(1-alpha)
            total=np.r_[np.full(3,c['accel_group_delay_s']),np.full(3,c['gyro_group_delay_s'])]
            extra=np.maximum(0,total-pole_delay)
            delayed=np.array([np.interp(time-extra[j],times,values[:,j]) for j in range(6)])
            ranges=np.r_[np.full(3,c['accel_range_g']),np.full(3,c['gyro_range_deg_s'])]
            self.saturated=bool(np.any(np.abs(delayed)>ranges))
            delayed=np.clip(delayed,-ranges,ranges)
            self.accel_raw=np.clip(np.rint(delayed[:3]*c['accel_lsb_per_g']),-32768,32767).astype(int)
            self.gyro_raw=np.clip(np.rint(delayed[3:]*c['gyro_lsb_per_deg_s']),-32768,32767).astype(int)
            self.accel=self.accel_raw/c['accel_lsb_per_g']*9.81
            self.gyro=np.radians(self.gyro_raw/c['gyro_lsb_per_deg_s'])
            self.next_output=(math.floor((time+1e-9)*c['output_rate_hz'])+1)/c['output_rate_hz']
        while len(self.history)>2 and self.history[1][0]<time-.1:
            self.history.popleft()

    def read(self):
        return self.accel.copy(),self.gyro.copy()


class MotorEncoders:
    def __init__(self,config,counts_per_wheel_rev=None):
        self.config=config
        self.counts_per_rev=(counts_per_wheel_rev if counts_per_wheel_rev is not None else
            config['motor_pulses_per_rev_per_channel']*config['quadrature_multiplier']*config['gear_ratio'])
        if not math.isfinite(self.counts_per_rev) or self.counts_per_rev<=0:
            raise ValueError('Conteggi per giro ruota non validi')
        self.sign=np.array(config['counter_sign_from_joint'])
        self.counts=np.full(2,config['counter_origin'],dtype=int)
        self.phase=None
        self.time=None
        self.previous_angle=None
        self.velocity=np.zeros(2)

    def update_positions(self,angles):
        # Multi-edge changes counted analytically; not a sampled A/B waveform.
        if np.asarray(angles).shape!=(2,) or not np.isfinite(angles).all():
            raise ValueError('Posizioni encoder non finite')
        phase=np.rint(np.array(angles)*self.counts_per_rev/(2*np.pi)*self.sign).astype(np.int64)
        if self.phase is None:
            delta=phase
        else:
            delta=phase-self.phase
        self.counts=np.clip(self.counts+delta,0,2**self.config['counter_bits']-1).astype(int)
        self.phase=phase

    def reset(self,angles):
        self.phase=np.rint(np.array(angles)*self.counts_per_rev/(2*np.pi)*self.sign).astype(np.int64)
        self.counts[:]=self.config['counter_origin']
        self.time=None
        self.previous_angle=None
        self.velocity[:]=0

    def read(self,time):
        if np.any(self.counts==0) or np.any(self.counts==2**self.config['counter_bits']-1):
            raise ValueError('Contatore encoder simulato saturo; nessun rollover')
        angle=(self.counts-self.config['counter_origin'])*self.sign*2*np.pi/self.counts_per_rev
        if self.time is not None and time>self.time+1e-12:
            dt=time-self.time
            raw=(angle-self.previous_angle)/dt
            alpha=math.exp(-dt/self.config['velocity_filter_tau_s'])
            self.velocity=alpha*self.velocity+(1-alpha)*raw
        self.time=time
        self.previous_angle=angle.copy()
        return float(angle[0]),float(self.velocity[0]),float(angle[1]),float(self.velocity[1])
