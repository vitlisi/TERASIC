#ifndef TERASIC_PITCH_PD_CORE_H
#define TERASIC_PITCH_PD_CORE_H

#include <cmath>

// Unita': gradi, gradi/s, secondi e punti percentuali PWM.
// Nucleo senza hardware, condiviso fra DE10-Nano e MuJoCo.
inline double pitch_pd_fuse(double theta, double gyro, double dt, double theta_acc)
{
    return 0.98 * (theta + gyro * dt) + 0.02 * theta_acc;
}

inline double pitch_pd_command(double theta, double gyro, double kp, double kd,
                               int motor_sign, double max_pwm)
{
    double duty = motor_sign * (kp * theta + kd * gyro);
    if (duty > max_pwm) duty = max_pwm;
    if (duty < -max_pwm) duty = -max_pwm;
    return duty;
}

inline unsigned pitch_pd_high_ticks(double signed_percent)
{
    return static_cast<unsigned>(50000 * std::fabs(signed_percent) / 100.0);
}

#endif
