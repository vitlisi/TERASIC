// Offline only: exact original recurrence with structured input/output scaling.
#define RealHinf OriginalRealHinf
#include "../src_cpp/terasic_reale/controllers/balance_hinf_core.h"
#undef RealHinf
static double tuning_common=1.,tuning_pitch_rate=1.;
class RealHinf: public OriginalRealHinf {
public:
    bool step(const LqrMimoState &state,double radius,double *sx,double *dx) {
        LqrMimoState adjusted=state;adjusted.x[4]*=tuning_pitch_rate;
        double left=0,right=0;
        if(!OriginalRealHinf::step(adjusted,radius,&left,&right))return false;
        const double common=.5*(left+right),diff=.5*(right-left);
        *sx=tuning_common*common-diff;*dx=tuning_common*common+diff;
        return lqr_finite(*sx) && lqr_finite(*dx);
    }
};
#define TWIN_HINF
#include "../scripts/digital_twin_bridge.cpp"
extern "C" int twin_tuning_set(double common,double rate) {
    if(!lqr_finite(common) || !lqr_finite(rate) || common<=0 || common>1 || rate<=0 || rate>2)return 0;
    tuning_common=common;tuning_pitch_rate=rate;return 1;
}
