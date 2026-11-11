#ifndef DIAGNOSTICS_LQR_STARTUP_ASSIST_H
#define DIAGNOSTICS_LQR_STARTUP_ASSIST_H
#include "startup_assist.h"
#include <cmath>

struct LqrAssistOutput {
    double nominal[2],target[2],applied[2],pair_scale;
    bool active[2];
};

class LqrStartupAssist {
    StartupAssist state_[2];
public:
    LqrAssistOutput update(const double nominal[2],const unsigned enc[2],double time,
                           double cap,double differential_cap) {
        LqrAssistOutput r;
        const double floors[2][2]={{5.,5.},{7.,7.}};
        for(unsigned i=0;i<2;++i) {
            r.nominal[i]=nominal[i];
            const double u=std::fabs(nominal[i])<.5 ? 0. : nominal[i];
            const double floor=floors[i][u<0 ? 1 : 0];
            double target=state_[i].update(u,time,enc[i],i==0 ? -1 : 1,i,true,floor);
            if(target>cap)target=cap;
            if(target<-cap)target=-cap;
            r.target[i]=target;
        }
        // Interpolate from the valid allocator command to the boosted target.
        // Preserve zeros and signs, the wheel cap and the differential cap.
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
