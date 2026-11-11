#ifndef PWM_MOTION_COMPENSATION_H
#define PWM_MOTION_COMPENSATION_H
#include "../src_cpp/terasic_reale/controllers/balance_lqr_core.h"
#include "inverse_lut_data.h"
#include <algorithm>

// SOFTWARE CANDIDATE ONLY. No hardware integration or inferred startup threshold.
class PwmMotionCompensation {
public:
    enum Regime { STOPPED=0, DRIVING=1, BRAKING=2, STARTING=3, UNRESOLVED=4 };
    struct Config {
        double filter_s, stopped_s, request_s, pulse_s, neutral_rearm_s;
        double correction_rate, request_min;
        bool startup_enabled;
        Config():filter_s(.03),stopped_s(.10),request_s(.06),pulse_s(.15),
          neutral_rearm_s(.20),correction_rate(100.),request_min(1.),startup_enabled(false) {}
    };
    struct Output {
        double nominal[2],target[2],applied[2],omega[2],correction[2],pair_scale;
        int regime[2],support[2]; // support: 0 unmeasured low, 1 measured, 2 high
        bool valid;
    };
private:
    struct State {
        unsigned encoder;
        double omega,quiet,neutral,persistent,pulse,correction;
        int requested;
        bool armed,pulsing;
        State():encoder(32768),omega(0),quiet(0),neutral(0),persistent(0),
          pulse(0),correction(0),requested(0),armed(false),pulsing(false) {}
    } s_[2];
    Config cfg_;
    double previous_time_;
    bool initialized_;
    static double clip(double x,double a,double b) { return std::max(a,std::min(x,b)); }
    static double smooth(double x) { x=clip(x,0.,1.);return x*x*(3.-2.*x); }
public:
    explicit PwmMotionCompensation(const Config &c=Config()):cfg_(c),previous_time_(0),initialized_(false) {}
    // Inverse steady PWM minus the chosen linear viscous reference. Not a speed servo.
    // Voltage/torque conversion is an assumption, inherited from the nominal model.
    static double loss(double omega,unsigned wheel,int *support=0) {
        if(wheel>1 || !lqr_finite(omega))return 0.;
        const unsigned branch=2*wheel+(omega<0),n=inverse_sizes[branch];
        const InverseNode *p=inverse_nodes[branch];
        const double v=std::fabs(omega),g=.441/12.*inverse_reference_voltage/100.;
        double q=clip(v,p[0].speed,p[n-1].speed);
        unsigned j=1;while(j<n-1 && p[j].speed<q)++j;
        const double pwm=p[j-1].pwm+(q-p[j-1].speed)*(p[j].pwm-p[j-1].pwm)/(p[j].speed-p[j-1].speed);
        if(support)*support=v<p[0].speed ? 0 : (v>p[n-1].speed ? 2 : 1);
        // Outside measured range: explicit regularization, not identified inverse.
        const double taper=smooth((v-.08)/(p[0].speed-.08));
        return (omega<0 ? -1.:1.)*std::max(0.,pwm-inverse_damping[wheel]*q/g)*taper;
    }
    Output update(const double input[2],const unsigned enc[2],double time,double cap=20.,double diffcap=3.) {
        Output r={{0,0},{0,0},{0,0},{0,0},{0,0},1.,{0,0},{0,0},false};r.pair_scale=1.;r.valid=false;
        const double dt=time-previous_time_;
        bool good=lqr_finite(time) && lqr_finite(cap) && cap>0 && lqr_finite(diffcap) && diffcap>=0;
        for(unsigned w=0;w<2;++w)good=good && lqr_finite(input[w]) && enc[w]>0 && enc[w]<65535;
        if(!good || (initialized_ && (!(dt>0) || dt>.05))) {
            // Invalid data demands a stop from the caller; never emit a kick.
            *this=PwmMotionCompensation(cfg_);return r;
        }
        r.valid=true;
        for(unsigned w=0;w<2;++w) {
            State &s=s_[w];
            const double u=clip(input[w],-cap,cap);
            const int sign=u>=.5 ? 1 : (u<=-.5 ? -1 : 0);
            if(initialized_) {
                const int counts=(static_cast<int>(enc[w])-static_cast<int>(s.encoder))*(w==0 ? -1:1);
                const double a=std::exp(-dt/cfg_.filter_s);
                s.omega=a*s.omega+(1-a)*(counts*2.*3.141592653589793/1325./dt);
                s.quiet=counts==0 ? s.quiet+dt : 0.;
            }
            s.encoder=enc[w];
            const bool stopped=s.quiet>=cfg_.stopped_s && std::fabs(s.omega)<.08;
            s.neutral=sign==0 && stopped ? s.neutral+(initialized_ ? dt:0.) : 0.;
            if(s.neutral>=cfg_.neutral_rearm_s)s.armed=true;
            if(sign!=s.requested || sign==0) {s.persistent=0.;s.pulsing=false;}
            else if(initialized_)s.persistent+=dt;
            s.requested=sign;
            const bool moving=std::fabs(s.omega)>=.15 && s.quiet<cfg_.stopped_s;
            int regime=stopped ? STOPPED : UNRESOLVED;
            double desired=0.;
            if(moving && sign && u*s.omega<0)regime=BRAKING;
            if(moving && sign && u*s.omega>0) {
                regime=DRIVING;
                // Continuous request gate avoids a discontinuous relay around u=0.
                desired=loss(s.omega,w,&r.support[w])*smooth((std::fabs(u)-.5)/1.5);
            }
            if(cfg_.startup_enabled && s.armed && stopped && std::fabs(u)>=cfg_.request_min && s.persistent>=cfg_.request_s) {
                s.pulsing=true;s.armed=false;s.pulse=0.;
            }
            if(s.pulsing) {
                if(!sign || moving || s.pulse>=cfg_.pulse_s)s.pulsing=false;
                else {
                    regime=STARTING;
                    // Explicit unvalidated pulse parameters; NOT identified breakaway thresholds.
                    const double floor=w==0 ? 6.5:8.3;
                    desired=sign*std::max(0.,floor-std::fabs(u));
                    s.pulse+=initialized_ ? dt:0.;
                }
            }
            if(!sign || regime==BRAKING || regime==STOPPED || regime==UNRESOLVED)s.correction=0.;
            else {
                const double step=cfg_.correction_rate*(initialized_ ? dt:0.);
                // No stale correction may oppose the current request.
                if(s.correction*u<0)s.correction=0.;
                s.correction+=clip(desired-s.correction,-step,step);
                // Slew limits buildup only; residual boost must not survive gate closure.
                if(std::fabs(s.correction)>std::fabs(desired))s.correction=desired;
            }
            r.nominal[w]=u;r.omega[w]=s.omega;r.regime[w]=regime;
            r.target[w]=clip(u+s.correction,-cap,cap);
        }
        // Project baseline first, then interpolate correction within physical limits.
        // Keep the differential constraint without clipping only one wheel.
        const double common=clip(.5*(r.nominal[0]+r.nominal[1]),-cap,cap);
        const double room=std::min(diffcap,cap-std::fabs(common));
        const double d=clip(.5*(r.nominal[1]-r.nominal[0]),-room,room);
        r.nominal[0]=common-d;r.nominal[1]=common+d;
        const double d1=.5*(r.target[1]-r.target[0]);
        if(d1>diffcap && d1>d)r.pair_scale=clip((diffcap-d)/(d1-d),0.,1.);
        if(d1<-diffcap && d1<d)r.pair_scale=clip((-diffcap-d)/(d1-d),0.,1.);
        for(unsigned w=0;w<2;++w) {
            const double u=r.nominal[w]+r.pair_scale*(r.target[w]-r.nominal[w]);
            const unsigned high=std::fabs(u)<.5 ? 0u:static_cast<unsigned>(500.*std::fabs(u));
            r.applied[w]=(u<0 ? -1.:1.)*high/500.;
        }
        // Driver deadband can enlarge a differential after quantization (e.g. .49 -> 0).
        // Reduce the larger magnitude output; never add actuation to the other wheel.
        const double quantum_cap=std::floor(2.*diffcap*500.)/500.;
        if(std::fabs(r.applied[1]-r.applied[0])>quantum_cap) {
            const unsigned w=std::fabs(r.applied[0])>std::fabs(r.applied[1]) ? 0:1;
            const unsigned other=1-w;
            double q=r.applied[other]+(r.applied[w]>r.applied[other] ? quantum_cap:-quantum_cap);
            if(std::fabs(q)<.5)q=0.;
            r.applied[w]=q;
        }
        for(unsigned w=0;w<2;++w)r.correction[w]=r.applied[w]-r.nominal[w];
        previous_time_=time;initialized_=true;return r;
    }
};
#endif
