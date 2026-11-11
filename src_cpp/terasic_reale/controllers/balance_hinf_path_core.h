#ifndef TERASIC_BALANCE_HINF_PATH_CORE_H
#define TERASIC_BALANCE_HINF_PATH_CORE_H
#include "balance_hinf_core.h"
#include "balance_lqr_path_core.h"

// Same dynamic Hinf matrices, driven by state minus the shared path reference.
// Shared nominal feedforward is added before the existing PWM allocator.
inline bool hinf_path_command(RealHinf &controller,const LqrMimoState &state,
    const LqrPathReference &reference,double radius,double limit,double diff,LqrMimoCommand *out,
    double turn_yaw_reserve_percent=0)
{
    if(!lqr_finite(turn_yaw_reserve_percent) || turn_yaw_reserve_percent<0 || turn_yaw_reserve_percent>=limit)return false;
    LqrMimoState error;
    for(int i=0;i<6;++i)error.x[i]=state.x[i]-reference.x[i];
    double left=0,right=0;
    if(!controller.step(error,radius,&left,&right))return false;
    left+=reference.ff[0];right+=reference.ff[1];
    if(!lqr_finite(left) || !lqr_finite(right))return false;
    *out=hinf_allocate(left,right,limit,diff);
    if(turn_yaw_reserve_percent>0 && (reference.phase==3 || reference.phase==4)) {
        // Experimental yaw headroom during turn/settling; total wheel limit unchanged.
        // Keep unconstrained requests in telemetry, including commands rejected here.
        const double common=.5*(left+right),differential=.5*(right-left);
        const double allocated_common=lqr_mimo_clip(common,limit-turn_yaw_reserve_percent);
        *out=hinf_allocate(allocated_common-differential,allocated_common+differential,limit,diff);
        out->raw[0]=left;out->raw[1]=right;
        out->common_raw=common;out->differential_raw=differential;
    }
    return true;
}
#endif
