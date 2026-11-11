#ifndef TERASIC_BALANCE_LQR_PATH_CORE_H
#define TERASIC_BALANCE_LQR_PATH_CORE_H
#include "balance_lqr_mimo_core.h"

struct LqrPathModel {
    double pitch_v,pitch_a,common_v,common_a,diff_w,diff_alpha;
};
struct LqrPathReference {
    double x[6],ff[2];
    int phase;
};
struct LqrPathProfile { double s,v,a,j; };

inline double lqr_path_yaw_rate_rad_s(double gx_deg_s,double gz_deg_s,
                                       double absolute_pitch_deg,int sign)
{
    const double pitch=absolute_pitch_deg*3.141592653589793/180.;
    return sign*(-gx_deg_s*std::sin(pitch)+gz_deg_s*std::cos(pitch))*3.141592653589793/180.;
}

class LqrPathYawFusion {
    double yaw_,tau_;
public:
    explicit LqrPathYawFusion(double tau):yaw_(0),tau_(tau){}
    double update(double encoder_yaw,double gyro_yaw_rate,double dt)
    {
        const double predicted=yaw_+gyro_yaw_rate*dt;
        const double alpha=std::exp(-dt/tau_);
        // Both estimates are unwrapped, necessary for a continuous 180deg turn.
        yaw_=alpha*predicted+(1-alpha)*encoder_yaw;
        return yaw_;
    }
};

inline LqrPathProfile lqr_path_profile(double time,double start,double duration,double distance)
{
    LqrPathProfile p;
    double f=(time-start)/duration;
    if(f<=0){p.s=p.v=p.a=p.j=0;return p;}
    if(f>=1){p.s=distance;p.v=p.a=p.j=0;return p;}
    p.s=distance*(10*f*f*f-15*f*f*f*f+6*f*f*f*f*f);
    p.v=distance/duration*(30*f*f-60*f*f*f+30*f*f*f*f);
    p.a=distance/(duration*duration)*(60*f-180*f*f+120*f*f*f);
    p.j=distance/(duration*duration*duration)*(60-360*f+360*f*f);
    return p;
}

inline LqrPathReference lqr_path_reference(double time,int mode,double distance,
                                            double turn_rad,const LqrPathModel &model)
{
    if(mode==0) {
        LqrPathReference r;
        for(int i=0;i<6;++i)r.x[i]=0;
        r.ff[0]=r.ff[1]=0;r.phase=0;return r;
    }
    // 0..3 hold, 3..8 advance, 8..11 hold, 11..36 turn,
    // 36..39 hold, 39..44 advance in new heading, 44..49 hold.
    const LqrPathProfile first=lqr_path_profile(time,3,5,distance);
    const LqrPathProfile turn=lqr_path_profile(time,11,25,mode==2 ? turn_rad : 0);
    const LqrPathProfile second=lqr_path_profile(time,39,5,mode==2 ? distance : 0);
    const double velocity=first.v+second.v,accel=first.a+second.a,jerk=first.j+second.j;
    LqrPathReference r;
    r.x[0]=first.s+second.s;
    r.x[1]=model.pitch_v*velocity+model.pitch_a*accel;
    r.x[2]=turn.s;
    r.x[3]=velocity;
    r.x[4]=model.pitch_v*accel+model.pitch_a*jerk;
    r.x[5]=turn.v;
    const double common=100*(model.common_v*velocity+model.common_a*accel);
    const double differential=100*(model.diff_w*turn.v+model.diff_alpha*turn.a);
    r.ff[0]=common-differential;r.ff[1]=common+differential;
    r.phase=time<3 ? 0 : (time<8 ? 1 : 2);
    if(mode==2 && time>=11)r.phase=time<36 ? 3 : (time<39 ? 4 : (time<44 ? 5 : 6));
    return r;
}

inline bool lqr_path_tracking_safe(const LqrMimoState &s,const LqrPathReference &r)
{
    // The 20-degree bound now applies to TRACKING ERROR, not total desired turn.
    // Keep an independent absolute envelope for this specific short sequence.
    return std::fabs(s.x[0]-r.x[0])<=.12 && std::fabs(s.x[3])<=.30 &&
        std::fabs(s.x[2]-r.x[2])<=.3490658504 && std::fabs(s.x[5])<=1.5707963268 &&
        std::fabs(s.x[0])<=.35 && std::fabs(s.x[2])<=3.926990817;
}

inline LqrMimoCommand lqr_path_command(const LqrMimoState &s,const LqrPathReference &r,
    const double k[2][6],int sign,double limit,double differential_limit)
{
    LqrMimoState error;
    for(int i=0;i<6;++i)error.x[i]=s.x[i]-r.x[i];
    LqrMimoCommand c=lqr_mimo_command(error,k,sign,limit,differential_limit);
    c.raw[0]+=sign*r.ff[0];c.raw[1]+=sign*r.ff[1];
    c.common_raw=.5*(c.raw[0]+c.raw[1]);
    c.differential_raw=.5*(c.raw[1]-c.raw[0]);
    if(!lqr_finite(c.raw[0]) || !lqr_finite(c.raw[1])) return c;
    const double common=lqr_mimo_clip(c.common_raw,limit);
    double headroom=limit-std::fabs(common);
    if(headroom>differential_limit)headroom=differential_limit;
    const double differential=lqr_mimo_clip(c.differential_raw,headroom);
    c.limited[0]=common-differential;c.limited[1]=common+differential;
    for(int i=0;i<2;++i) {
        const double value=c.limited[i];
        const unsigned high=std::fabs(value)<.5 ? 0u :
            static_cast<unsigned>(50000*std::fabs(value)/100.);
        c.applied[i]=(value<0 ? -1. : 1.)*high/500.;
    }
    return c;
}
#endif
