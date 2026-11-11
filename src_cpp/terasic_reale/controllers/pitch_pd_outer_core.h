#ifndef TERASIC_PITCH_PD_OUTER_CORE_H
#define TERASIC_PITCH_PD_OUTER_CORE_H
#include "balance_lqr_core.h"

// The FPGA encoder convention and 50 ms speed filter match the real LQR.
// Position is relative to the release pose; the chassis pitch contribution
// converts wheel-relative encoder motion to longitudinal ground motion.
class PitchPdOuterEstimator {
    LqrEncoderEstimator wheels_;
    double origin_pitch_rad_;
public:
    explicit PitchPdOuterEstimator(double origin_pitch_deg)
      : wheels_(1325.0,-1,1,-1),
        origin_pitch_rad_(origin_pitch_deg*3.141592653589793/180.0) {}
    bool update(unsigned sx,unsigned dx,double dt,double pitch_deg,double gyro_deg_s,
                double *position_m,double *velocity_m_s)
    {
        double wheel_rad=0,wheel_rate=0;
        if(!wheels_.update(sx,dx,dt,&wheel_rad,&wheel_rate) ||
           !lqr_finite(pitch_deg) || !lqr_finite(gyro_deg_s)) return false;
        const double pitch_rad=pitch_deg*3.141592653589793/180.0;
        const double pitch_rate=gyro_deg_s*3.141592653589793/180.0;
        *position_m=.032*(pitch_rad-origin_pitch_rad_-wheel_rad);
        *velocity_m_s=.032*(pitch_rate-wheel_rate);
        return lqr_finite(*position_m) && lqr_finite(*velocity_m_s);
    }
};

inline double pitch_pd_outer_target(double position_m,double velocity_m_s,
                                    double kx,double kv)
{
    double offset=-kx*position_m-kv*velocity_m_s;
    if(offset>3.0)offset=3.0;
    if(offset< -3.0)offset=-3.0;
    return offset;
}
#endif
