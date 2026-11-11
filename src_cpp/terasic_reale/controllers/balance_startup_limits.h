#ifndef TERASIC_BALANCE_STARTUP_LIMITS_H
#define TERASIC_BALANCE_STARTUP_LIMITS_H
#include <cmath>
// Same initial deviation envelope as MuJoCo. Not a recovery guarantee.
// The supported initial tilt must NEVER become the equilibrium reference.
static const double BALANCE_STARTUP_MAX_ERROR_DEG = 5.0;
inline bool balance_startup_pitch_ok(double absolute_deg,double reference_deg,
                                     double limit_deg=BALANCE_STARTUP_MAX_ERROR_DEG)
{
    // Comparison also rejects NaN and infinity, without C++11 isfinite.
    return limit_deg>0 && limit_deg<=BALANCE_STARTUP_MAX_ERROR_DEG &&
           std::fabs(absolute_deg-reference_deg)<=limit_deg;
}
#endif
