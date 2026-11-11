#ifndef DIAGNOSTICS_PWM_INVERSE_COMPENSATION_H
#define DIAGNOSTICS_PWM_INVERSE_COMPENSATION_H
#include "lqr_startup_assist.h"
#include "inverse_lut_data.h"

// Experimental actuator model matching, NOT r=PWM or a speed control loop.
// Nominal PWM -> nominal steady speed -> inverse measured real characteristic.
// Identity outside measured support; smooth taper and correction cap for trial.
class PwmInverseCompensation {
public:
    static double map(double u,unsigned wheel,double cap,double *omega=0,bool *supported=0) {
        if(omega)*omega=0.;
        if(supported)*supported=false;
        if(wheel>1 || !(cap>0) || !lqr_finite(u))return 0.;
        if(u>cap)u=cap;
        if(u<-cap)u=-cap;
        if(u==0)return 0.;
        const double magnitude=std::fabs(u);
        const double torque=.441/12.*inverse_reference_voltage*magnitude/100.;
        const double speed=(torque-inverse_friction[wheel])/inverse_damping[wheel];
        if(omega)*omega=(u<0 ? -1. : 1.)*(speed>0 ? speed : 0.);
        const unsigned branch=2*wheel+(u<0 ? 1 : 0),n=inverse_sizes[branch];
        const InverseNode *nodes=inverse_nodes[branch];
        if(speed<=nodes[0].speed || speed>=nodes[n-1].speed)return u;
        if(supported)*supported=true;
        unsigned i=1;while(nodes[i].speed<speed)++i;
        const double fraction=(speed-nodes[i-1].speed)/(nodes[i].speed-nodes[i-1].speed);
        const double inverse=nodes[i-1].pwm+fraction*(nodes[i].pwm-nodes[i-1].pwm);
        const double t=(speed-nodes[0].speed)/(nodes[n-1].speed-nodes[0].speed);
        const double weight=16*t*t*(1-t)*(1-t); // C1 taper to identity at both boundaries.
        double correction=weight*(inverse-magnitude);
        if(correction>1.)correction=1.;
        if(correction<-1.)correction=-1.;
        double result=magnitude+correction;
        if(result>cap)result=cap;
        return u<0 ? -result : result;
    }
    LqrAssistOutput update(const double nominal[2],const unsigned *,double,
                           double cap,double differential_cap) {
        LqrAssistOutput r;
        for(unsigned i=0;i<2;++i) {r.nominal[i]=nominal[i];r.target[i]=map(nominal[i],i,cap);}
        const double d0=.5*(nominal[1]-nominal[0]),d1=.5*(r.target[1]-r.target[0]);
        r.pair_scale=1.;
        if(d1>differential_cap && d1>d0)r.pair_scale=(differential_cap-d0)/(d1-d0);
        else if(d1<-differential_cap && d1<d0)r.pair_scale=(-differential_cap-d0)/(d1-d0);
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
