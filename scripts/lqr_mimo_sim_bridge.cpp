#include "../src_cpp/terasic_reale/controllers/balance_lqr_mimo_core.h"
struct MimoSimulation {
    LqrMimoEstimator encoder;
    double pitch,k[2][6],limit,differential_limit;
    MimoSimulation(const double *gains,double initial_pitch,double pwm=15,double diff=3)
      : encoder(1325,.032,0.16444,initial_pitch*3.141592653589793/180.),pitch(initial_pitch),limit(pwm),differential_limit(diff)
    { for(int i=0;i<2;++i) for(int j=0;j<6;++j) k[i][j]=gains[6*i+j]; }
};
extern "C" {
void *mimo_create(const double *gains,double pitch) { return new MimoSimulation(gains,pitch); }
void *mimo_create_limited(const double *gains,double pitch,double pwm,double diff)
{
    if(!lqr_finite(pwm) || !lqr_finite(diff) || pwm<0 || pwm>100 || diff<0 || diff>pwm)
        return 0;
    return new MimoSimulation(gains,pitch,pwm,diff);
}
void mimo_destroy(void *p) { delete static_cast<MimoSimulation *>(p); }
int mimo_step(void *p,unsigned sx,unsigned dx,double accel,double rate,double *out)
{
    MimoSimulation &s=*static_cast<MimoSimulation *>(p);
    s.pitch=lqr_fuse_pitch_deg(s.pitch,rate,.01,accel);
    LqrMimoState state;
    if(!s.encoder.update(sx,dx,.01,s.pitch*3.141592653589793/180.,
                        rate*3.141592653589793/180.,&state)) return 0;
    const LqrMimoCommand c=lqr_mimo_command(state,s.k,1,s.limit,s.differential_limit);
    if(!lqr_finite(c.raw[0]) || !lqr_finite(c.raw[1])) return 0;
    for(int i=0;i<6;++i) out[i]=state.x[i];
    out[6]=c.raw[0];out[7]=c.raw[1];out[8]=c.applied[0];out[9]=c.applied[1];
    out[10]=c.common_raw;out[11]=c.differential_raw;
    return 1;
}
}
