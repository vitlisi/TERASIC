#ifndef PWM_LOW_COMMAND_COMPENSATION_H
#define PWM_LOW_COMMAND_COMPENSATION_H
#include "pwm_motion_smooth_compensation.h"
// Experimental low-command inverse + one-shot startup; frozen LUT nodes.
// Used by separate bench and LQR experiments, never by the default controller.
class PwmLowCommandCompensation {
public:
    typedef PwmMotionCompensationSmooth::Output Output;
    static double gain() {return .2044017950318064;}
    static double smooth(double t) {t=std::max(0.,std::min(1.,t));return t*t*(3.-2*t);}
    static double map(double u,unsigned wheel,int *support=0) {
        if(support)*support=0;
        if(wheel>1 || !lqr_finite(u) || std::fabs(u)<.5)return 0.;
        const double a=std::fabs(u),sgn=u<0 ? -1.:1.;
        if(a>=10.)return u; // identity at 10,15,20 and above
        const unsigned branch=2*wheel+(u<0),n=inverse_sizes[branch];
        const InverseNode *p=inverse_nodes[branch];const double v=gain()*a;
        double pwm;
        if(v<p[0].speed) {
            // Explicit unsupported low-speed regularization; not identified.
            pwm=p[0].pwm*v/p[0].speed;
        } else {
            if(support)*support=1;
            unsigned j=1;while(j<n-1 && p[j].speed<v)++j;
            pwm=p[j-1].pwm+(v-p[j-1].speed)*(p[j].pwm-p[j-1].pwm)/(p[j].speed-p[j-1].speed);
        }
        const double weight=1.-smooth((a-8.)/2.);
        return sgn*(a+weight*(pwm-a));
    }
private:
    struct State {
        unsigned enc,origin;double quiet,requested_since,pulse_since;int sign;
        bool armed,pulsing;
        State():enc(32768),origin(32768),quiet(0),requested_since(0),pulse_since(0),sign(0),armed(true),pulsing(false) {}
    } s_[2];
    bool initialized_;double previous_;
    static double clip(double u,double cap){return std::max(-cap,std::min(cap,u));}
public:
    PwmLowCommandCompensation():initialized_(false),previous_(0) {}
    Output update(const double input[2],const unsigned enc[2],double time,double cap=20.,double diffcap=3.) {
        Output r={{0,0},{0,0},{0,0},{0,0},{0,0},1.,{0,0},{0,0},false};
        const double dt=time-previous_;
        if(!lqr_finite(time) || !lqr_finite(cap) || cap<=0 || !lqr_finite(diffcap) || diffcap<0 ||
           (initialized_ && (!(dt>0) || dt>.05)))return r;
        for(unsigned w=0;w<2;++w)if(!lqr_finite(input[w]) || enc[w]==0 || enc[w]>=65535)return r;
        r.valid=true;
        for(unsigned w=0;w<2;++w) {
            State &s=s_[w];const double u=clip(input[w],cap),a=std::fabs(u);
            const int sign=a<.5 ? 0:(u<0 ? -1:1);
            const int delta=(static_cast<int>(enc[w])-static_cast<int>(s.enc))*(w==0 ? -1:1);
            if(initialized_) {s.quiet=delta==0 ? s.quiet+dt:0.;r.omega[w]=delta*2.*3.141592653589793/1325./dt;}
            if(sign!=s.sign) {s.requested_since=time;s.pulsing=false;}
            if(!sign) {
                s.pulsing=false;if(s.quiet>=.20)s.armed=true;
            }
            const bool stopped=s.quiet>=.10;
            double target=map(u,w,&r.support[w]);
            r.regime[w]=sign ? 1:0;
            if(sign && a<8. && stopped && s.armed && time-s.requested_since>=.03) {
                s.pulsing=true;s.armed=false;s.pulse_since=time;s.origin=enc[w];
            }
            if(s.pulsing) {
                const int directed=(static_cast<int>(enc[w])-static_cast<int>(s.origin))*(w==0 ? -1:1)*sign;
                if(!sign || a>=8. || directed>=3 || time-s.pulse_since>=.15)s.pulsing=false;
                else {
                    // Experimental amplitude from earlier successful cold commands, not guaranteed breakaway.
                    const double floor=w==0 ? 6.5:8.3;
                    target=sign*std::max(std::fabs(target),floor);r.regime[w]=3;
                }
            }
            r.nominal[w]=u;r.target[w]=clip(target,cap);s.enc=enc[w];s.sign=sign;
        }
        // Start from the valid LQR allocation and scale only the correction.
        // A high command with zero correction stays unchanged unless final driver
        // deadband quantization requires a tiny reduction to preserve the pair cap.
        const double common=clip(.5*(r.nominal[0]+r.nominal[1]),cap);
        const double room=std::min(diffcap,cap-std::fabs(common));
        const double d0=clip(.5*(r.nominal[1]-r.nominal[0]),room);
        r.nominal[0]=common-d0;r.nominal[1]=common+d0;
        const double d1=.5*(r.target[1]-r.target[0]);
        if(d1>diffcap && d1>d0)r.pair_scale=std::max(0.,std::min(1.,(diffcap-d0)/(d1-d0)));
        if(d1<-diffcap && d1<d0)r.pair_scale=std::max(0.,std::min(1.,(-diffcap-d0)/(d1-d0)));
        for(unsigned w=0;w<2;++w) {
            const double u=r.nominal[w]+r.pair_scale*(r.target[w]-r.nominal[w]);
            const unsigned high=std::fabs(u)<.5 ? 0u:static_cast<unsigned>(500.*std::fabs(u));
            r.applied[w]=(u<0 ? -1.:1.)*high/500.;
        }
        const double quantum_cap=std::floor(2.*diffcap*500.)/500.;
        if(std::fabs(r.applied[1]-r.applied[0])>quantum_cap) {
            const unsigned w=std::fabs(r.applied[0])>std::fabs(r.applied[1]) ? 0:1;
            const double q=r.applied[1-w]+(r.applied[w]>r.applied[1-w] ? quantum_cap:-quantum_cap);
            r.applied[w]=std::fabs(q)<.5 ? 0.:q;
        }
        for(unsigned w=0;w<2;++w)r.correction[w]=r.applied[w]-r.nominal[w];
        previous_=time;initialized_=true;return r;
    }
};
#endif
