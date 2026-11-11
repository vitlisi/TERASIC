#ifndef TERASIC_LQR_ENDURANCE_CORE_H
#define TERASIC_LQR_ENDURANCE_CORE_H
#include "balance_lqr_path_simple_core.h"
#include <limits>

// Continuous software counts. FPGA counters saturate, so the caller must reload
// the vendor load registers before the rails and call recenter() after the write.
class LqrEnduranceEncoder {
    double cpr_,r_,track_,origin_,wheel_rate_,yaw_rate_,sx_sum_,dx_sum_;
    int prev_sx_,prev_dx_;bool initialized_;
public:
    LqrEnduranceEncoder(double cpr,double radius,double track,double origin)
      :cpr_(cpr),r_(radius),track_(track),origin_(origin),wheel_rate_(0),yaw_rate_(0),sx_sum_(0),dx_sum_(0),prev_sx_(32768),prev_dx_(32768),initialized_(false){}
    void recenter(){prev_sx_=prev_dx_=32768;}
    bool update(unsigned sx,unsigned dx,double dt,double pitch,double rate,LqrMimoState *out){
        if(sx==0 || sx>=65535 || dx==0 || dx>=65535 || !(cpr_>0) || !(r_>0) || !(track_>0) ||
           !lqr_finite(dt) || dt<0 || (initialized_ && dt<=0) || !lqr_finite(pitch) || !lqr_finite(rate))return false;
        const int ds=static_cast<int>(sx)-prev_sx_,dd=static_cast<int>(dx)-prev_dx_;
        if(initialized_ && (std::abs(ds)>1000 || std::abs(dd)>1000))return false;
        sx_sum_+=ds;dx_sum_+=dd;prev_sx_=sx;prev_dx_=dx;
        const double scale=2*3.141592653589793/cpr_;
        if(initialized_){const double a=std::exp(-dt/.05);
            wheel_rate_=a*wheel_rate_+(1-a)*(scale*.5*(ds-dd))/dt;
            yaw_rate_=a*yaw_rate_+(1-a)*(r_/track_*scale*(ds+dd))/dt;}
        initialized_=true;
        out->x[0]=r_*(pitch-origin_-scale*.5*(sx_sum_-dx_sum_));out->x[1]=pitch;
        out->x[2]=r_/track_*scale*(sx_sum_+dx_sum_);out->x[3]=r_*(rate-wheel_rate_);
        out->x[4]=rate;out->x[5]=yaw_rate_;
        for(int j=0;j<6;++j)if(!lqr_finite(out->x[j]))return false;
        return true;
    }
};

class EnduranceBattery {
    double cutoff_,filtered_,last_,below_;bool low_;
public:
    explicit EnduranceBattery(double cutoff):cutoff_(cutoff),filtered_(std::numeric_limits<double>::quiet_NaN()),last_(-1),below_(-1),low_(false){}
    void update(double time,double voltage){
        if(!lqr_finite(voltage) || voltage<=0 || (last_>=0 && time<=last_))return;
        if(last_<0)filtered_=voltage;
        else {const double a=std::exp(-(time-last_)/2.);filtered_=a*filtered_+(1-a)*voltage;}
        last_=time;
        if(filtered_<=cutoff_){if(below_<0)below_=time;if(time-below_>=2)low_=true;}else below_=-1;
    }
    bool low()const{return low_;}
    double filtered()const{return filtered_;}
};

class LqrEndurancePath {
    LqrPathModel model_;LqrSimplePath path_;
    unsigned long cycle_;double cycle_t_,base_s_,base_yaw_,last_s_,last_yaw_,x_,y_,cycle_x_,cycle_y_,hold_s_;
    bool initialized_;LqrPathReference last_reference_;
public:
    explicit LqrEndurancePath(const LqrPathModel &model):model_(model),path_(28.5,.4,3.141592653589793,model,8,7,1.5,1,2),
      cycle_(0),cycle_t_(0),base_s_(0),base_yaw_(0),last_s_(0),last_yaw_(0),x_(0),y_(0),cycle_x_(0),cycle_y_(0),hold_s_(0),initialized_(false){}
    LqrPathReference update(double time,const LqrMimoState &state){
        if(initialized_){const double ds=state.x[0]-last_s_,mid=.5*(state.x[2]+last_yaw_);x_-=ds*std::sin(mid);y_+=ds*std::cos(mid);}
        last_s_=state.x[0];last_yaw_=state.x[2];
        const unsigned long next=static_cast<unsigned long>(std::floor(time/35.5));
        if(next!=cycle_){
            hold_s_=last_reference_.x[0];cycle_=next;base_s_=state.x[0];base_yaw_=cycle_*2*3.141592653589793;
            cycle_x_=x_;cycle_y_=y_;
            path_=LqrSimplePath(28.5,.4,3.141592653589793,model_,8,7,1.5,1,2);
        }
        cycle_t_=time-cycle_*35.5;
        LqrMimoState local=state;local.x[0]-=base_s_;local.x[2]-=base_yaw_;
        LqrPathReference ref=path_.update(cycle_t_,local);ref.x[0]+=base_s_;ref.x[2]+=base_yaw_;
        // Restore the starting heading at the origin before the next departure.
        // The two forward legs keep their original timing; append one7s pivot.
        if(cycle_t_>=28.5){
            const LqrPathProfile pivot=lqr_path_profile(cycle_t_,28.5,7,3.141592653589793);
            ref.phase=8;ref.x[2]=base_yaw_+3.141592653589793+pivot.s;ref.x[5]=pivot.v;
            const double differential=100*(model_.diff_w*pivot.v+model_.diff_alpha*pivot.a);
            ref.ff[0]=-differential;ref.ff[1]=differential;
        }
        if(cycle_t_<1.5 && cycle_>0)ref.x[0]=hold_s_;
        last_reference_=ref;initialized_=true;return ref;
    }
    unsigned long cycle()const{return cycle_+1;}
    double cycle_time()const{return cycle_t_;}
    double world_x()const{return x_;}double world_y()const{return y_;}
    double cycle_x()const{return cycle_x_;}double cycle_y()const{return cycle_y_;}
};
#endif
