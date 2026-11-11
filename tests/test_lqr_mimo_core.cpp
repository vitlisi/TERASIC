#include "../src_cpp/terasic_reale/controllers/balance_lqr_mimo_core.h"
#include <cassert>
#include <cmath>
static bool near(double a,double b){return std::fabs(a-b)<1e-9;}
int main()
{
    LqrMimoEstimator estimator(1325,.032,0.16444,.02);LqrMimoState s;
    assert(estimator.update(32768,32768,0,.02,0,&s));
    assert(near(s.x[0],0) && near(s.x[2],0));
    assert(estimator.update(32668,32868,.01,.02,0,&s));
    assert(s.x[0]>0 && near(s.x[2],0)); // both forward, no yaw
    LqrMimoEstimator yaw(1325,.032,0.16444,0);
    assert(yaw.update(32768,32768,0,0,0,&s));
    assert(yaw.update(32868,32868,.01,0,0,&s));
    assert(near(s.x[0],0) && s.x[2]>0 && s.x[5]>0); // pure differential
    assert(!yaw.update(0,32768,.01,0,0,&s)); // saturated encoder, no rollover
    double k[2][6]={{0,0,-50,0,0,-10},{0,0,50,0,0,10}};
    for(int i=0;i<6;++i)s.x[i]=0;
    s.x[2]=.1;LqrMimoCommand c=lqr_mimo_command(s,k,1,15,3);
    assert(near(c.raw[0],5) && near(c.raw[1],-5));
    assert(near(c.applied[0],3) && near(c.applied[1],-3));
    k[0][0]=k[1][0]=-14;s.x[0]=1;
    c=lqr_mimo_command(s,k,1,15,3);
    assert(near(c.limited[0],15) && near(c.limited[1],13)); // balance priority
    k[0][0]=k[1][0]=-20;c=lqr_mimo_command(s,k,1,15,3);
    assert(near(c.applied[0],15) && near(c.applied[1],15));
    s.x[0]=0;s.x[2]=.001;c=lqr_mimo_command(s,k,1,15,3);
    assert(near(c.applied[0],0) && near(c.applied[1],0));
    return 0;
}
