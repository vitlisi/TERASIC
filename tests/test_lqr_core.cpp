#include "../src_cpp/terasic_reale/controllers/balance_lqr_core.h"
#include <cassert>
#include <limits>

int main()
{
    LqrEncoderEstimator e(1000,1,-1,1);
    double q=0,w=0;
    assert(e.update(32768,32768,0,&q,&w) && q==0 && w==0);
    assert(e.update(33768,31768,.01,&q,&w));
    assert(std::fabs(q-2*3.141592653589793)<1e-12 && w>0);
    assert(!e.update(65535,31768,.01,&q,&w));
    assert(!e.update(33768,31768,0,&q,&w));
    LqrEncoderEstimator uncalibrated(0,1,-1,1);
    assert(!uncalibrated.update(32768,32768,.01,&q,&w));
    LqrEncoderEstimator reverse(1000,1,-1,-1);
    assert(reverse.update(33768,31768,.01,&q,&w) && q<0 && w==0);
    const double k[4]={1,-10,2,-1};
    LqrState x; x.psi=.1;
    double full[6]; x.vector(full);
    assert(full[0]==0 && full[1]==.1 && !lqr_finite(full[2]));
    assert(full[3]==0 && full[4]==0 && !lqr_finite(full[5]));
    LqrCommand c=lqr_command(x,k,1,100);
    assert(std::fabs(c.raw_percent-1)<1e-12 && c.high_ticks==500);
    assert(lqr_command(x,k,-1,100).applied_percent==-1);
    assert(lqr_command(x,k,1,0).high_ticks==0);
    x.psi=20;
    c=lqr_command(x,k,1,100);
    assert(c.raw_percent==200 && c.applied_percent==100 && c.high_ticks==50000);
    c=lqr_command(x,k,1,15);
    assert(c.applied_percent==15 && c.high_ticks==7500);
    x.psi=.049;
    assert(lqr_command(x,k,1,100).high_ticks==0);
    x.psi=std::numeric_limits<double>::quiet_NaN();
    c=lqr_command(x,k,1,100);
    assert(!lqr_finite(c.raw_percent) && c.high_ticks==0);
    assert(lqr_finite(1) && !lqr_finite(std::numeric_limits<double>::infinity()));
    return 0;
}
