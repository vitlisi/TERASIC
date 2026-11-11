#ifndef DIAGNOSTICS_PWM_GAP_COMPENSATION_H
#define DIAGNOSTICS_PWM_GAP_COMPENSATION_H
#include "lqr_startup_assist.h" // Shared telemetry structure only.

// Research-only, memoryless four-branch command characteristic.
// For 0<|u|<8: |out|=floor+(8-floor)*|u|/8. Zero maps to zero.
// No persistence, encoder condition, release event or startup timeout.
class PwmGapCompensation {
public:
    static double map(double u,unsigned wheel,double cap) {
        if(!(cap>0) || u==0)return 0.;
        const double floors[2][2]={{5.,5.},{7.,7.}};
        const double mag=std::fabs(u);
        const double floor=floors[wheel][u<0 ? 1 : 0];
        double out=mag>=8. ? mag : floor+(8.-floor)*mag/8.;
        if(out>cap)out=cap;
        return u<0 ? -out : out;
    }
    LqrAssistOutput update(const double nominal[2],const unsigned *,double,
                           double cap,double differential_cap) {
        LqrAssistOutput r;
        for(unsigned i=0;i<2;++i) {
            r.nominal[i]=nominal[i];r.target[i]=map(nominal[i],i,cap);
        }
        const double d0=.5*(nominal[1]-nominal[0]);
        const double d1=.5*(r.target[1]-r.target[0]);
        r.pair_scale=1.;
        if(d1>differential_cap && d1>d0)
            r.pair_scale=(differential_cap-d0)/(d1-d0);
        else if(d1<-differential_cap && d1<d0)
            r.pair_scale=(-differential_cap-d0)/(d1-d0);
        if(r.pair_scale<0)r.pair_scale=0;
        if(r.pair_scale>1)r.pair_scale=1;
        for(unsigned i=0;i<2;++i) {
            r.applied[i]=nominal[i]+r.pair_scale*(r.target[i]-nominal[i]);
            r.active[i]=std::fabs(r.applied[i]-nominal[i])>1e-6;
        }
        return r;
    }
};
#endif
