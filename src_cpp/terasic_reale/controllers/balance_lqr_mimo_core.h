#ifndef TERASIC_BALANCE_LQR_MIMO_CORE_H
#define TERASIC_BALANCE_LQR_MIMO_CORE_H
#include "balance_lqr_core.h"

// One six-state feedback, two wheel commands. Signed yaw is encoder odometry
// in the MuJoCo joint convention, NOT a measured absolute compass heading.
// States: [longitudinal distance m, pitch rad, yaw rad, m/s, rad/s, rad/s].
struct LqrMimoState { double x[6]; };
struct LqrMimoCommand {
    double raw[2], limited[2], applied[2], common_raw, differential_raw;
};

class LqrMimoEstimator {
    LqrEncoderEstimator common_;
    double cpr_, radius_, track_, pitch_origin_, previous_yaw_, yaw_rate_;
    bool initialized_;
public:
    LqrMimoEstimator(double cpr,double radius,double track,double pitch_origin)
      : common_(cpr,-1,1,-1),cpr_(cpr),radius_(radius),track_(track),
        pitch_origin_(pitch_origin),previous_yaw_(0),yaw_rate_(0),initialized_(false) {}
    bool update(unsigned sx,unsigned dx,double dt,double pitch,double pitch_rate,
                LqrMimoState *state)
    {
        double theta=0,wheel_rate=0;
        if (!common_.update(sx,dx,dt,&theta,&wheel_rate) ||
            !lqr_finite(pitch) || !lqr_finite(pitch_rate) ||
            !(radius_>0) || !(track_>0)) return false;
        const double scale=2*3.141592653589793/cpr_;
        const double yaw=radius_/track_*scale*
            ((static_cast<int>(sx)-32768)+(static_cast<int>(dx)-32768));
        if (initialized_) {
            const double alpha=std::exp(-dt/.05);
            yaw_rate_=alpha*yaw_rate_+(1-alpha)*(yaw-previous_yaw_)/dt;
        }
        previous_yaw_=yaw; initialized_=true;
        state->x[0]=radius_*((pitch-pitch_origin_)-theta);
        state->x[1]=pitch;
        state->x[2]=yaw;
        state->x[3]=radius_*(pitch_rate-wheel_rate);
        state->x[4]=pitch_rate;
        state->x[5]=yaw_rate_;
        for (int i=0;i<6;++i) if (!lqr_finite(state->x[i])) return false;
        return true;
    }
};

inline double lqr_mimo_clip(double value,double limit)
{ return value>limit ? limit : (value < -limit ? -limit : value); }

inline LqrMimoCommand lqr_mimo_command(const LqrMimoState &state,
    const double k[2][6],int motor_sign,double limit,double differential_limit)
{
    LqrMimoCommand command;
    for (int row=0;row<2;++row) {
        command.raw[row]=0;
        for (int col=0;col<6;++col)
            command.raw[row]-=motor_sign*k[row][col]*state.x[col];
        command.limited[row]=command.applied[row]=0;
    }
    command.common_raw=.5*(command.raw[0]+command.raw[1]);
    command.differential_raw=.5*(command.raw[1]-command.raw[0]);
    if (!lqr_finite(command.raw[0]) || !lqr_finite(command.raw[1])) return command;
    // Balance/longitudinal common command has priority over yaw authority.
    const double common=lqr_mimo_clip(command.common_raw,limit);
    double headroom=limit-std::fabs(common);
    if (headroom>differential_limit) headroom=differential_limit;
    const double differential=lqr_mimo_clip(command.differential_raw,headroom);
    command.limited[0]=common-differential;
    command.limited[1]=common+differential;
    for (int i=0;i<2;++i) {
        const double value=command.limited[i];
        const unsigned high=std::fabs(value)<.5 ? 0u :
            static_cast<unsigned>(50000*std::fabs(value)/100.);
        command.applied[i]=(value<0 ? -1. : 1.)*high/500.;
    }
    return command;
}
#endif
