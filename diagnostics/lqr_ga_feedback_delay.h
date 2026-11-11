#ifndef TERASIC_LQR_GA_FEEDBACK_DELAY_H
#define TERASIC_LQR_GA_FEEDBACK_DELAY_H
// Experimental GA deployment: retain the one-sample error delay used in training.
// One controller per process; a fresh executable starts with zero delayed error.
#define lqr_mimo_command lqr_ga_command_without_delay
#include "../src_cpp/terasic_reale/controllers/balance_lqr_mimo_core.h"
#undef lqr_mimo_command
inline LqrMimoCommand lqr_mimo_command(const LqrMimoState &state,
    const double k[2][6],int sign,double limit,double differential_limit)
{
    static LqrMimoState previous={{0,0,0,0,0,0}};
    const LqrMimoCommand result=lqr_ga_command_without_delay(previous,k,sign,limit,differential_limit);
    previous=state;
    return result;
}
#endif
