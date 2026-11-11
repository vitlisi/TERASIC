// Offline bridge for MuJoCo screening of the actual C++ estimator/control core.
// No MMIO, /dev/mem, network or hardware entry points.
#include "../src_cpp/terasic_reale/controllers/balance_lqr_core.h"

struct CoreSimulation {
    LqrEncoderEstimator encoder;
    double pitch, k[4];
    CoreSimulation(double cpr, const double *gains, double initial_error)
        : encoder(cpr,-1,1,-1), pitch(initial_error)
    { for (int i=0;i<4;++i) k[i]=gains[i]; }
};

extern "C" {
void *lqr_sim_create(double cpr, const double *gains, double initial_error)
{ return new CoreSimulation(cpr,gains,initial_error); }

void lqr_sim_destroy(void *handle)
{ delete static_cast<CoreSimulation *>(handle); }

int lqr_sim_step(void *handle, unsigned sx, unsigned dx, double accel_error_deg,
                 double gyro_deg_s, double dt, double max_pwm, double *out)
{
    CoreSimulation &c=*static_cast<CoreSimulation *>(handle);
    c.pitch=lqr_fuse_pitch_deg(c.pitch,gyro_deg_s,dt,accel_error_deg);
    LqrState state;
    if (!c.encoder.update(sx,dx,dt,&state.theta,&state.theta_dot)) return 0;
    state.psi=c.pitch*3.141592653589793/180.0;
    state.psi_dot=gyro_deg_s*3.141592653589793/180.0;
    const LqrCommand command=lqr_command(state,c.k,1,max_pwm);
    if (!lqr_finite(command.raw_percent)) return 0;
    out[0]=c.pitch; out[1]=state.theta; out[2]=state.theta_dot;
    out[3]=command.raw_percent; out[4]=command.limited_percent;
    out[5]=command.applied_percent;
    return 1;
}
}
