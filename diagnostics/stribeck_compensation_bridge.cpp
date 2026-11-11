#include "stribeck_lut_compensation.h"
extern "C" {
void *stribeck_comp_create() {return new StribeckLutCompensation;}
void stribeck_comp_destroy(void *p) {delete static_cast<StribeckLutCompensation*>(p);}
int stribeck_comp_step(void *p,double sx,double dx,unsigned es,unsigned ed,double t,double voltage,double *out) {
    StribeckLutCompensation &c=*static_cast<StribeckLutCompensation*>(p);
    c.set_voltage(voltage);const double u[]={sx,dx};const unsigned e[]={es,ed};
    const StribeckLutCompensation::Output r=c.update(u,e,t);
    for(unsigned w=0;w<2;++w) {
        out[w]=r.applied[w];out[2+w]=r.omega[w];out[4+w]=r.regime[w];
        out[6+w]=r.correction[w];out[8+w]=r.target[w];
    }
    out[10]=r.pair_scale;return r.valid ? 1:0;
}
}
