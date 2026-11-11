#include "../src_cpp/terasic_reale/controllers/balance_lqr_path_core.h"
#include <cassert>
#include <cmath>
static bool near(double a,double b){return std::fabs(a-b)<1e-8;}
int main()
{
    const double pi=3.141592653589793;
    LqrPathModel m={0,0,0,0,0,0};
    LqrPathReference r=lqr_path_reference(0,2,.1,pi,m);
    assert(near(r.x[0],0) && near(r.x[2],0));
    r=lqr_path_reference(8,2,.1,pi,m);
    assert(near(r.x[0],.1) && near(r.x[3],0) && near(r.x[2],0));
    r=lqr_path_reference(23.5,2,.1,pi,m);
    assert(near(r.x[0],.1) && near(r.x[2],pi/2) && r.x[5]>0);
    r=lqr_path_reference(49,2,.1,pi,m);
    assert(near(r.x[0],.2) && near(r.x[2],pi) && near(r.x[3],0) && near(r.x[5],0));
    LqrMimoState state;
    for(int i=0;i<6;++i)state.x[i]=r.x[i];
    assert(lqr_path_tracking_safe(state,r)); // 180deg is valid at 180deg target
    state.x[2]+=.36;assert(!lqr_path_tracking_safe(state,r));
    r=lqr_path_reference(49,1,.03,0,m);
    assert(near(r.x[0],.03) && near(r.x[2],0));
    r=lqr_path_reference(49,0,.1,0,m);
    for(int i=0;i<6;++i)assert(near(r.x[i],0));
    for(int i=0;i<=4900;++i) {
        r=lqr_path_reference(i*.01,2,.1,pi,m);
        assert(r.x[3]>=-1e-10 && r.x[3]<.038);
        assert(r.x[5]>=-1e-10 && r.x[5]<.236);
    }
    double k[2][6]={{0,0,0,0,0,0},{0,0,0,0,0,0}};
    for(int i=0;i<6;++i)state.x[i]=0;
    r.ff[0]=20;r.ff[1]=10;
    LqrMimoCommand c=lqr_path_command(state,r,k,1,15,5);
    assert(near(c.applied[0],15) && near(c.applied[1],15));
    r.ff[0]=-4;r.ff[1]=4;c=lqr_path_command(state,r,k,1,15,5);
    assert(near(c.applied[0],-4) && near(c.applied[1],4));
    assert(near(lqr_path_yaw_rate_rad_s(0,10,0,-1),-10*pi/180));
    assert(near(lqr_path_yaw_rate_rad_s(10,0,90,-1),10*pi/180));
    LqrPathYawFusion fusion(60);
    double angle=0;
    for(int i=0;i<1000;++i)angle=fusion.update((i+1)*.2*pi/180,20*pi/180,.01);
    assert(near(angle,200*pi/180)); // heading is continuous across +/-180deg
    return 0;
}
