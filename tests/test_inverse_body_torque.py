"""Check inverse reconstruction derivatives and independently known MuJoCo torques."""
import sys,json,unittest
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'diagnostics'),str(ROOT/'scripts'),str(ROOT/'src_python')]
from inverse_body_torque import local_polynomial,body_torque,filtered_interval_width
from mujoco_robot import MujocoRobot
from twin_payload import add_centered_payload,body_inertia

class InverseTorqueTest(unittest.TestCase):
    def test_actual_timestamp_cubic_derivatives(self):
        t=np.arange(50)*.01+np.sin(np.arange(50))*.0001;y=1+2*t-3*t*t+4*t**3
        r=local_polynomial(t,y,11)
        np.testing.assert_allclose(r[:,0],y,atol=1e-12)
        np.testing.assert_allclose(r[:,1],2-6*t+12*t*t,atol=1e-10)
        np.testing.assert_allclose(r[:,2],-6+24*t,atol=1e-8)
    def test_static_friction_interval_through_filter(self):
        t=np.arange(40)*.01;width=np.where(np.arange(40)%3==0,.02,0.)
        bound=filtered_interval_width(t,width,11)
        for seed in range(5):
            unknown=np.random.default_rng(seed).uniform(-1,1,len(t))*width
            self.assertTrue(np.all(abs(local_polynomial(t,unknown,11)[:,0])<=bound+1e-12))
    def test_known_forward_dynamics_torque(self):
        r=MujocoRobot();add_centered_payload(r,.02,.1);r.configure_motor_friction(json.loads((ROOT/'config/mujoco_robot01_stribeck.json').read_text()));r.start_support(1)
        for _ in range(400):r.step()
        r.release();m,d=r.m,r.d;b=r.body_id;mass=m.body_mass[b];cy,cz=m.body_ipos[b,1:];ixx=body_inertia(m,b)[0,0];errors=[];omitted=[];truth=[]
        for i in range(400):
            r.set_motors(5*np.sin(i*.025),5*np.sin(i*.025));mujoco.mj_forward(m,d)
            theta=-2*np.arctan2(d.qpos[4],d.qpos[3]);inferred=body_torque(theta,-d.qacc[3],d.qacc[1],mass,cy,cz,ixx,d.qacc[2])
            friction=np.zeros(m.nv);mask=d.efc_type[:d.nefc]==mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF;mujoco.mj_mulJacTVec(m,d,friction,np.where(mask,d.efc_force[:d.nefc],0))
            actual=0.;without_armature=0.
            for wheel in ('sx','dx'):
                k=int(m.joint('perno_'+wheel).dofadr[0]);net=-(d.qfrc_actuator[k]+d.qfrc_passive[k]+friction[k]);without_armature+=net;actual+=net+m.dof_armature[k]*d.qacc[k]
            errors.append(inferred-actual);omitted.append(inferred-without_armature);truth.append(actual);mujoco.mj_step(m,d)
        self.assertLess(max(abs(np.array(errors))),2e-5) # residual small 3D motion/cross inertia
        self.assertGreater(np.sqrt(np.mean(np.array(truth)**2)),.005)
        self.assertGreater(np.sqrt(np.mean(np.array(omitted)**2)),10*np.sqrt(np.mean(np.array(errors)**2)))
if __name__=='__main__':unittest.main()
