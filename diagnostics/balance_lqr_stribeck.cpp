// Separate guarded experimental binary; baseline controller remains unchanged.
#define LQR_STARTUP_ASSIST_EXPERIMENT
#define PWM_GAP_COMPENSATION_EXPERIMENT
#define PWM_MOTION_COMPENSATION_EXPERIMENT
#define PWM_STRIBECK_LUT_EXPERIMENT
#include "balance_lqr_guarded.cpp"
