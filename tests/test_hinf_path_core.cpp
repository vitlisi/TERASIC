#include <cassert>
#include <cmath>
#include <limits>
#include "../src_cpp/terasic_reale/controllers/balance_hinf_path_core.h"
int main() {
    {
        RealHinf reserved,baseline,straight;
        LqrMimoState state={{0,0,0,0,0,0}};
        LqrPathReference ref;
        for(int i=0;i<6;++i)ref.x[i]=0;
        ref.ff[0]=20;ref.ff[1]=40;ref.phase=3;
        LqrMimoCommand r,b,s;
        assert(hinf_path_command(reserved,state,ref,.032,20,20,&r,8));
        assert(hinf_path_command(baseline,state,ref,.032,20,20,&b));
        assert(r.raw[0]==20 && r.raw[1]==40 && r.common_raw==30 && r.differential_raw==10);
        assert(r.applied[0]==4 && r.applied[1]==20);
        assert(b.applied[0]==20 && b.applied[1]==20);
        ref.phase=1;
        assert(hinf_path_command(straight,state,ref,.032,20,20,&s,8));
        assert(s.applied[0]==20 && s.applied[1]==20);
        assert(!hinf_path_command(straight,state,ref,.032,20,20,&s,20));
    }
    RealHinf path,expected;
    for(int n=0;n<500;++n) {
        LqrMimoState state,error;
        LqrPathReference ref;
        for(int i=0;i<6;++i){state.x[i]=.001*std::sin(n*.02+i);ref.x[i]=.0002*i;error.x[i]=state.x[i]-ref.x[i];}
        ref.ff[0]=2.;ref.ff[1]=3.;ref.phase=1;
        LqrMimoCommand command;
        assert(hinf_path_command(path,state,ref,.032,20,20,&command));
        double left=0,right=0;
        assert(expected.step(error,.032,&left,&right));
        LqrMimoCommand allocated=hinf_allocate(left+2,right+3,20,20);
        for(int i=0;i<2;++i){assert(std::fabs(command.raw[i]-allocated.raw[i])<1e-10);assert(command.applied[i]==allocated.applied[i]);}
        for(int i=0;i<12;++i)assert(path.xc[i]==expected.xc[i]);
        if(n==0){assert(command.raw[0]==2);assert(command.raw[1]==3);}
    }
    return 0;
}
