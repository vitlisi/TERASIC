#ifndef DIAGNOSTICS_PWM_INVERSE_FULL_H
#define DIAGNOSTICS_PWM_INVERSE_FULL_H
#include "lqr_startup_assist.h"
#include "inverse_lut_data.h"

// Professor-form static inverse: linear speed reference -> full real inverse.
// LQR PWM units are converted with the nominal *linear* steady gain (no
// Coulomb dead zone). Extensions are explicit hypotheses, not measured LUT.
class PwmInverseFull {
    static double slope(const InverseNode *p,unsigned start,unsigned count) {
        double x=0,y=0,xx=0,xy=0;
        for(unsigned i=start;i<start+count;++i) {
            x+=p[i].speed;y+=p[i].pwm;xx+=p[i].speed*p[i].speed;xy+=p[i].speed*p[i].pwm;
        }
        return (count*xy-x*y)/(count*xx-x*x);
    }
public:
    // region: 0 zero, 1 lower extension, 2 measured interpolation,
    // 3 upper extension; saturation is separately visible at the output cap.
    static double speed_map(double omega,unsigned wheel,double cap,unsigned *region=0) {
        if(region)*region=0;
        if(wheel>1 || !(cap>0) || !lqr_finite(omega) || omega==0)return 0.;
        const unsigned branch=2*wheel+(omega<0 ? 1 : 0),n=inverse_sizes[branch];
        const InverseNode *p=inverse_nodes[branch];
        const double speed=std::fabs(omega);double pwm=0.;
        if(speed<p[0].speed) {
            if(region)*region=1;
            pwm=p[0].pwm+slope(p,0,n<4 ? n : 4)*(speed-p[0].speed);
        } else if(speed>p[n-1].speed) {
            if(region)*region=3;
            const unsigned k=n<4 ? n : 4;
            pwm=p[n-1].pwm+slope(p,n-k,k)*(speed-p[n-1].speed);
        } else {
            if(region)*region=2;
            unsigned i=1;while(i<n-1 && p[i].speed<speed)++i;
            pwm=p[i-1].pwm+(speed-p[i-1].speed)/(p[i].speed-p[i-1].speed)*(p[i].pwm-p[i-1].pwm);
        }
        if(pwm<0)pwm=0;
        if(pwm>cap)pwm=cap;
        return omega<0 ? -pwm : pwm;
    }
    static double reference_speed(double u,unsigned wheel) {
        return u*(.441/12.*inverse_reference_voltage/100.)/inverse_damping[wheel];
    }
    static double map(double u,unsigned wheel,double cap,unsigned *region=0) {
        if(wheel>1 || !lqr_finite(u))return 0.;
        if(u>cap)u=cap;
        if(u<-cap)u=-cap;
        return speed_map(reference_speed(u,wheel),wheel,cap,region);
    }
    LqrAssistOutput update(const double nominal[2],const unsigned *,double,
                           double cap,double differential_cap) {
        LqrAssistOutput r;
        for(unsigned i=0;i<2;++i) {r.nominal[i]=nominal[i];r.target[i]=map(nominal[i],i,cap);}
        double common=.5*(r.target[0]+r.target[1]);
        if(common>cap)common=cap;
        if(common<-cap)common=-cap;
        double headroom=cap-std::fabs(common);
        if(headroom>differential_cap)headroom=differential_cap;
        const double raw_diff=.5*(r.target[1]-r.target[0]);double diff=raw_diff;
        if(diff>headroom)diff=headroom;
        if(diff<-headroom)diff=-headroom;
        r.pair_scale=std::fabs(raw_diff)>1e-12 ? std::fabs(diff/raw_diff) : 1.;
        r.applied[0]=common-diff;r.applied[1]=common+diff;
        for(unsigned i=0;i<2;++i)r.active[i]=std::fabs(r.applied[i]-nominal[i])>1e-6;
        return r;
    }
};
#endif
