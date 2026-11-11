// Offline only: same estimator, discrete Hinf and allocator as the real executable.
// The 50% ceiling is ONLY for simulation. Real executable has its own lower cap.
#include "../../src_cpp/terasic_reale/controllers/balance_hinf_core.h"
struct HinfSimulation {
    LqrMimoEstimator encoder;RealHinf controller;double pitch,cap,diff;
    HinfSimulation(double initial,double limit,double differential)
        :encoder(1325,.032,.16444,initial*3.141592653589793/180.),pitch(initial),cap(limit),diff(differential){}
};
extern "C" {
void *hinf_create(double initial,double cap,double diff) {
    if(!lqr_finite(initial) || !lqr_finite(cap) || !lqr_finite(diff) || cap<=0 || cap>50 || diff<0 || diff>cap)return 0;
    return new HinfSimulation(initial,cap,diff);
}
void hinf_destroy(void *p){delete static_cast<HinfSimulation *>(p);}
int hinf_measure(void *p,unsigned sx,unsigned dx,double accel,double rate,double *out){
    if(!p)return 0;
    HinfSimulation &s=*static_cast<HinfSimulation *>(p);
    s.pitch=lqr_fuse_pitch_deg(s.pitch,rate,.01,accel);
    LqrMimoState state;
    if(!s.encoder.update(sx,dx,.01,s.pitch*3.141592653589793/180.,rate*3.141592653589793/180.,&state))return 0;
    for(int i=0;i<6;++i)out[i]=state.x[i];
    return 1;
}
int hinf_allocate_only(double left,double right,double cap,double diff,double *out){
    if(!lqr_finite(left) || !lqr_finite(right) || !lqr_finite(cap) || !lqr_finite(diff) || cap<=0 || cap>50 || diff<0 || diff>cap)return 0;
    const LqrMimoCommand c=hinf_allocate(left,right,cap,diff);
    out[0]=c.applied[0];out[1]=c.applied[1];out[2]=c.common_raw;out[3]=c.differential_raw;
    return 1;
}
int hinf_step(void *p,unsigned sx,unsigned dx,double accel,double rate,double *out){
    if(!hinf_measure(p,sx,dx,accel,rate,out))return 0;
    HinfSimulation &s=*static_cast<HinfSimulation *>(p);
    LqrMimoState state;
    for(int i=0;i<6;++i)state.x[i]=out[i];
    double left=0,right=0;
    if(!s.controller.step(state,.032,&left,&right))return 0;
    const LqrMimoCommand c=hinf_allocate(left,right,s.cap,s.diff);
    if(!lqr_finite(c.raw[0]) || !lqr_finite(c.raw[1]))return 0;
    for(int i=0;i<6;++i){out[i]=state.x[i];out[12+i]=s.controller.feedback[i];}
    out[6]=c.raw[0];out[7]=c.raw[1];out[8]=c.applied[0];out[9]=c.applied[1];
    out[10]=c.common_raw;out[11]=c.differential_raw;
    for(int i=0;i<12;++i)out[18+i]=s.controller.xc[i];
    return 1;
}
}
