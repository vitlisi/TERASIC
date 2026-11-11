"""Provisional SOC/OCV battery and power accounting for the existing plant.

The electrical equivalent matches the existing actuator + damping exactly;
it estimates currents without adding another braking term to MuJoCo.
"""
import json
import math
from pathlib import Path
import numpy as np


class BatteryDepleted(RuntimeError):
    pass


class BatteryPack:
    def __init__(self, config, initial_soc, torque_per_volt, electrical_damping):
        self.config = config
        ns, np_ = config['series_cells'], config['parallel_cells']
        if not isinstance(ns,int) or not isinstance(np_,int) or ns<1 or np_<1:
            raise ValueError('Numero celle non valido')
        self.capacity_ah = config['cell_capacity_ah']*np_
        self.resistance = config['cell_resistance_ohm']*ns/np_
        self.cutoff_v = config['cell_cutoff_v']*ns
        self.electronics_w = config['electronics_power_w']
        if not all(math.isfinite(v) for v in (self.capacity_ah,self.resistance,self.cutoff_v,self.electronics_w,initial_soc)):
            raise ValueError('Parametri batteria non finiti')
        if self.capacity_ah<=0 or self.resistance<0 or self.cutoff_v<=0 or self.electronics_w<0 or not 0<=initial_soc<=1:
            raise ValueError('Parametri batteria fuori intervallo')
        self.points=np.array(config['ocv_soc_points'],dtype=float)
        if self.points.ndim!=2 or self.points.shape[1]!=2 or len(self.points)<2 or not np.isfinite(self.points).all():
            raise ValueError('Curva OCV non valida')
        if (self.points[0,0]!=0 or self.points[-1,0]!=1 or
                not np.all(np.diff(self.points[:,0])>0) or
                not np.all(np.diff(self.points[:,1])>=0) or np.min(self.points[:,1])<=0):
            raise ValueError('Curva OCV deve coprire SOC 0..1 ed essere monotona')
        self.soc=float(initial_soc)
        c=np.broadcast_to(torque_per_volt,(2,)).astype(float)
        b=np.broadcast_to(electrical_damping,(2,)).astype(float)
        if not np.isfinite(np.r_[c,b]).all() or np.any(c<=0) or np.any(b<=0):
            raise ValueError('Equivalente motore richiede guadagno e damping positivi')
        # tau_em = c*d*V - b*w = K*i; K=Ke, R=K/c.
        self.motor_k=b/c
        self.motor_r=b/c**2
        self.used_ah=np.zeros(3)  # electronics, left battery branch, right branch
        self.used_wh=np.zeros(3)

    @classmethod
    def from_file(cls,path,initial_soc,torque_per_volt,electrical_damping,overrides=None):
        config=json.loads(Path(path).read_text())
        config.update(overrides or {})
        return cls(config,initial_soc,torque_per_volt,electrical_damping)

    def load(self, duty, wheel_rates):
        duty=np.asarray(duty,dtype=float);w=np.asarray(wheel_rates,dtype=float)
        if duty.shape!=(2,) or w.shape!=(2,) or not np.isfinite(np.r_[duty,w]).all() or np.any(np.abs(duty)>1):
            raise ValueError('Duty/velocità non validi per il modello elettrico')
        if self.soc<=0:
            raise BatteryDepleted('Carica batteria esaurita')
        voc=self.config['series_cells']*float(np.interp(self.soc,self.points[:,0],self.points[:,1]))
        # Positive consumption of each ideal bridge: max(0,d*i).
        alpha=duty**2/self.motor_r
        beta=-duty*self.motor_k*w/self.motor_r
        voltage=None
        for mask in range(4):
            active=np.array([bool(mask&1),bool(mask&2)])
            a=1+self.resistance*float(alpha[active].sum())
            b=self.resistance*float(beta[active].sum())-voc
            c=self.resistance*self.electronics_w
            discriminant=b*b-4*a*c
            if discriminant<0:
                continue
            candidate=(-b+math.sqrt(discriminant))/(2*a)
            branches=alpha*candidate+beta
            if candidate>0 and np.all(branches[active]>=-1e-10) and np.all(branches[~active]<=1e-10):
                voltage=candidate
                break
        if voltage is None or voltage<=self.cutoff_v:
            raise BatteryDepleted('Tensione sotto cutoff del modello o carico non sostenibile')
        winding=(duty*voltage-self.motor_k*w)/self.motor_r
        motors=np.maximum(0.,duty*winding)
        electronics=self.electronics_w/voltage
        return dict(battery_voltage_v=voltage,battery_ocv_v=voc,battery_soc=self.soc,
                    battery_consumed_mah=1000*float(self.used_ah.sum()),
                    battery_remaining_mah=1000*self.capacity_ah*self.soc,
                    battery_current_A=electronics+float(motors.sum()),
                    electronics_current_A=electronics,
                    motor_sx_winding_current_A=float(winding[0]),motor_dx_winding_current_A=float(winding[1]),
                    motor_sx_battery_current_A=float(motors[0]),motor_dx_battery_current_A=float(motors[1]))

    def advance(self,dt,load):
        if not math.isfinite(dt) or dt<=0:
            raise ValueError('Passo batteria non valido')
        currents=np.array([load['electronics_current_A'],load['motor_sx_battery_current_A'],load['motor_dx_battery_current_A']])
        if not np.isfinite(currents).all() or np.any(currents<0):
            raise ValueError('Correnti batteria non valide')
        dq=currents*dt/3600
        available=self.soc*self.capacity_ah
        if dq.sum()>available:
            dq*=available/dq.sum()
        self.used_ah+=dq
        self.used_wh+=dq*load['battery_voltage_v']
        self.soc=max(0.,self.soc-float(dq.sum())/self.capacity_ah)

    def summary(self):
        return dict(final_battery_soc=self.soc,consumed_ah=float(self.used_ah.sum()),
                    consumed_mah=1000*float(self.used_ah.sum()),remaining_mah=1000*self.capacity_ah*self.soc,
                    consumed_wh=float(self.used_wh.sum()),electronics_consumed_wh=float(self.used_wh[0]),
                    motor_sx_consumed_wh=float(self.used_wh[1]),motor_dx_consumed_wh=float(self.used_wh[2]))
