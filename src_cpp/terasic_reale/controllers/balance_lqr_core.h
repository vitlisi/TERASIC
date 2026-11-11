#ifndef TERASIC_BALANCE_LQR_CORE_H
#define TERASIC_BALANCE_LQR_CORE_H

#include <cmath>
#include <limits>

// Independent of pitch_pd. Full SI state x=[theta,psi,rho,theta_dot,psi_dot,rho_dot].
// psi=pitch, rho=yaw. Unestimated yaw is NaN, not an assumed zero.
// theta is mean wheel rotation RELATIVE TO THE CHASSIS, not world distance.
// K produces PWM percentage points, NOT normalized duty, torque or degrees.
struct LqrState {
    double theta, psi, rho, theta_dot, psi_dot, rho_dot;
    LqrState() : theta(0), psi(0), rho(std::numeric_limits<double>::quiet_NaN()),
                 theta_dot(0), psi_dot(0), rho_dot(std::numeric_limits<double>::quiet_NaN()) {}
    void vector(double x[6]) const {
        x[0]=theta; x[1]=psi; x[2]=rho;
        x[3]=theta_dot; x[4]=psi_dot; x[5]=rho_dot;
    }
};

struct LqrCommand {
    double raw_percent, limited_percent, applied_percent;
    unsigned high_ticks;
};

inline bool lqr_finite(double x)
{
    return x == x && x >= -1.7976931348623157e308 && x <= 1.7976931348623157e308;
}

inline double lqr_fuse_pitch_deg(double previous, double gyro, double dt,
                                 double accel_angle)
{
    return 0.98 * (previous + gyro * dt) + 0.02 * accel_angle;
}

inline LqrCommand lqr_command(const LqrState &x, const double k[4],
                              int motor_sign, double max_pwm)
{
    LqrCommand c;
    // Reduced K order [theta,psi,theta_dot,psi_dot]; explicitly omit yaw.
    // Do not multiply zero gains by NaN yaw estimates.
    c.raw_percent = -motor_sign * (k[0]*x.theta + k[1]*x.psi +
                                  k[2]*x.theta_dot + k[3]*x.psi_dot);
    c.limited_percent = c.applied_percent = 0;
    c.high_ticks = 0;
    if (!lqr_finite(c.raw_percent)) return c; // caller must stop on this fault
    c.limited_percent = c.raw_percent;
    if (c.limited_percent > max_pwm) c.limited_percent = max_pwm;
    if (c.limited_percent < -max_pwm) c.limited_percent = -max_pwm;
    c.high_ticks = std::fabs(c.limited_percent) < 0.5 ? 0u :
        static_cast<unsigned>(50000.0 * std::fabs(c.limited_percent) / 100.0);
    c.applied_percent = (c.limited_percent < 0 ? -1.0 : 1.0) *
                         c.high_ticks / 500.0;
    return c;
}

class LqrEncoderEstimator {
    double counts_per_rev_, previous_, velocity_;
    int sx_sign_, dx_sign_, model_sign_;
    bool initialized_;
public:
    LqrEncoderEstimator(double cpr, int sx_sign, int dx_sign, int model_sign)
      : counts_per_rev_(cpr), previous_(0), velocity_(0),
        sx_sign_(sx_sign), dx_sign_(dx_sign), model_sign_(model_sign),
        initialized_(false) {}

    bool update(unsigned sx, unsigned dx, double dt,
                double *angle, double *velocity)
    {
        // Actual FPGA counters saturate; do NOT apply a rollover correction.
        if (!(counts_per_rev_ > 0) || !lqr_finite(counts_per_rev_) ||
            sx == 0 || sx >= 65535 || dx == 0 || dx >= 65535 ||
            !lqr_finite(dt) || dt < 0 || (initialized_ && !(dt > 0))) return false;
        const double scale = 2.0 * 3.141592653589793 / counts_per_rev_;
        const double q = model_sign_ * scale * 0.5 *
            (sx_sign_ * (static_cast<int>(sx) - 32768) +
             dx_sign_ * (static_cast<int>(dx) - 32768));
        if (initialized_) {
            const double raw_velocity = (q - previous_) / dt;
            // 50 ms low-pass; its delay must be included in controller validation.
            const double alpha = std::exp(-dt / 0.05);
            velocity_ = alpha * velocity_ + (1.0-alpha) * raw_velocity;
        }
        previous_ = q;
        initialized_ = true;
        *angle = q;
        *velocity = velocity_;
        return lqr_finite(q) && lqr_finite(velocity_);
    }
};

#endif
