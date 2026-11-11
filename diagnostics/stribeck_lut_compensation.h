#ifndef ROBOT01_STRIBECK_LUT_COMPENSATION_H
#define ROBOT01_STRIBECK_LUT_COMPENSATION_H
#include "../src_cpp/terasic_reale/controllers/balance_lqr_core.h"
#include "stribeck_compensation_data.h"
#include <algorithm>

// Explicit bounded actuator-model matching experiment; not a speed servo.
// Same class is compiled into the MuJoCo bridge and the guarded ARM binary.
class StribeckLutCompensation {
public:
    struct Output {
        double nominal[2],target[2],applied[2],omega[2],correction[2],pair_scale;
        int regime[2],support[2];
        bool valid;
    };
private:
    unsigned previous_encoder_[2];
    double omega_[2],quiet_[2],request_[2],correction_[2],previous_,voltage_;
    int sign_[2];bool initialized_;
    static double clip(double x,double low,double high) {return std::max(low,std::min(high,x));}
    static double smooth(double x) {x=clip(x,0.,1.);return x*x*(3.-2*x);}
    static void driver_pair(const double input[2],double output[2],double diffcap) {
        for(unsigned w=0;w<2;++w) {
            const unsigned high=std::fabs(input[w])<.5 ? 0u:static_cast<unsigned>(500.*std::fabs(input[w]));
            output[w]=(input[w]<0 ? -1.:1.)*high/500.;
        }
        const double bound=std::floor(2.*diffcap*500.)/500.;
        if(std::fabs(output[1]-output[0])>bound) {
            const unsigned w=std::fabs(output[0])>std::fabs(output[1]) ? 0:1;
            const double q=output[1-w]+(output[w]>output[1-w] ? bound:-bound);
            output[w]=std::fabs(q)<.5 ? 0.:q;
        }
    }
public:
    static const char *lut_hash() {return stribeck_lut_sha256;}
    StribeckLutCompensation():previous_(0),voltage_(11.5),initialized_(false) {
        for(unsigned w=0;w<2;++w) {previous_encoder_[w]=32768;omega_[w]=quiet_[w]=request_[w]=correction_[w]=0;sign_[w]=0;}
    }
    void set_voltage(double v) {voltage_=v;}
    static double delta_torque(unsigned wheel,int sign,double speed) {
        if(wheel>1 || !lqr_finite(speed) || speed<0)return 0.;
        const StribeckLossNode *p=stribeck_loss[2*wheel+(sign<0)];
        if(speed<=p[0].speed)return p[0].delta_torque;
        if(speed>=p[stribeck_lut_size-1].speed)return p[stribeck_lut_size-1].delta_torque;
        unsigned i=1;while(p[i].speed<speed)++i;
        const double a=(speed-p[i-1].speed)/(p[i].speed-p[i-1].speed);
        return p[i-1].delta_torque+a*(p[i].delta_torque-p[i-1].delta_torque);
    }
    Output update(const double input[2],const unsigned enc[2],double time,double cap=20.,double diffcap=3.) {
        Output r={{0,0},{0,0},{0,0},{0,0},{0,0},1.,{0,0},{0,0},false};
        const double dt=time-previous_;
        if(!lqr_finite(time) || !lqr_finite(cap) || cap<=0 || cap>20 || !lqr_finite(diffcap) || diffcap<0 || diffcap>3 ||
           !lqr_finite(voltage_) || voltage_<11.2 || voltage_>12.1 ||
           (initialized_ && (!(dt>0) || dt>.025)))return r;
        for(unsigned w=0;w<2;++w)if(!lqr_finite(input[w]) || enc[w]==0 || enc[w]>=65535)return r;
        const double common=clip(.5*(input[0]+input[1]),-cap,cap);
        const double room=std::min(diffcap,cap-std::fabs(common));
        const double differential=clip(.5*(input[1]-input[0]),-room,room);
        r.nominal[0]=common-differential;r.nominal[1]=common+differential;
        for(unsigned w=0;w<2;++w) {
            const double u=r.nominal[w],a=std::fabs(u);const int sign=a<.5 ? 0:(u<0 ? -1:1);
            if(initialized_) {
                const int delta=(static_cast<int>(enc[w])-static_cast<int>(previous_encoder_[w]))*(w==0 ? -1:1);
                const double alpha=std::exp(-dt/.05);
                omega_[w]=alpha*omega_[w]+(1-alpha)*delta*2.*3.141592653589793/1325./dt;
                quiet_[w]=delta==0 ? quiet_[w]+dt:0.;
            }
            request_[w]=(sign && sign==sign_[w]) ? request_[w]+(initialized_ ? dt:0.):0.;
            const bool stopped=quiet_[w]>=.1 && std::fabs(omega_[w])<.08;
            const bool moving=std::fabs(omega_[w])>=.08 && quiet_[w]<.1;
            const bool braking=sign && moving && u*omega_[w]<0;
            const bool eligible=sign && !braking && request_[w]>=.03 && (stopped || (moving && u*omega_[w]>0));
            const double gate=smooth((a-.5)/1.)*(1-smooth((a-6.)/2.));
            double desired=0.;
            if(eligible && a<8.) {
                const double speed=stopped ? 0.:std::fabs(omega_[w]);
                desired=sign*delta_torque(w,sign,speed)*100./(.441/12.*voltage_)*gate;
                desired=clip(desired,-.5,.5);
            }
            if(!eligible || a>=8.)correction_[w]=0.;
            else correction_[w]+=clip(desired-correction_[w],-10.*dt,10.*dt);
            r.target[w]=clip(u+correction_[w],-cap,cap);
            // A correction cannot reverse the requested actuation or create PWM
            // from a neutral command; driver quantization is applied below.
            if(!sign)r.target[w]=0.;
            if(r.target[w]*u<0)r.target[w]=0.;
            r.omega[w]=omega_[w];r.regime[w]=!sign ? 0:(braking ? 2:(stopped ? 3:(moving ? 1:4)));
            r.support[w]=eligible ? (stopped ? 0:1):0;
            previous_encoder_[w]=enc[w];sign_[w]=sign;
        }
        const double d0=.5*(r.nominal[1]-r.nominal[0]),d1=.5*(r.target[1]-r.target[0]);
        if(d1>diffcap && d1>d0)r.pair_scale=clip((diffcap-d0)/(d1-d0),0.,1.);
        if(d1<-diffcap && d1<d0)r.pair_scale=clip((-diffcap-d0)/(d1-d0),0.,1.);
        double candidate[2],baseline[2];
        for(unsigned w=0;w<2;++w)candidate[w]=r.nominal[w]+r.pair_scale*(r.target[w]-r.nominal[w]);
        driver_pair(r.nominal,baseline,diffcap);driver_pair(candidate,r.applied,diffcap);
        // The .5% bound applies to the *physical* change after the driver's
        // .5% deadband and quantization, not just the floating-point target.
        // Discontinuous deadband crossings fall back to the baseline pair.
        if(std::fabs(r.applied[0]-baseline[0])>.500001 || std::fabs(r.applied[1]-baseline[1])>.500001) {
            r.applied[0]=baseline[0];r.applied[1]=baseline[1];r.pair_scale=0.;
        }
        for(unsigned w=0;w<2;++w)r.correction[w]=r.applied[w]-baseline[w];
        r.valid=true;previous_=time;initialized_=true;return r;
    }
};
#endif
