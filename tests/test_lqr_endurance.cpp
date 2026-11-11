#include "../src_cpp/terasic_reale/controllers/balance_lqr_endurance_core.h"
#include <cassert>
#include <cstdlib>
int main(){
    EnduranceBattery b(10.5);
    b.update(0,12);b.update(.1,10);assert(!b.low());
    for(int i=2;i<=120;++i)b.update(i*.1,10.4);
    assert(b.low());
    EnduranceBattery pulse(10.5);for(int i=0;i<100;++i)pulse.update(i*.1,i==50?8:12);
    assert(!pulse.low());
    LqrMimoEstimator old(1325,.032,0.16444,0);LqrEnduranceEncoder live(1325,.032,0.16444,0);
    LqrMimoState a,z;
    for(int i=0;i<200;++i){assert(old.update(32768-i,32768+i,.01,0,0,&a));assert(live.update(32768-i,32768+i,.01,0,0,&z));for(int j=0;j<6;++j)assert(std::fabs(a.x[j]-z.x[j])<1e-10);}
    const double before=z.x[0];live.recenter();assert(live.update(32768,32768,.01,0,0,&z));assert(std::fabs(z.x[0]-before)<1e-12);
    assert(!live.update(65535,32768,.01,0,0,&z));
    LqrEnduranceEncoder longrun(1325,.032,0.16444,0);
    assert(longrun.update(32768,32768,.01,0,0,&z));
    double expected=0;
    for(int round=0;round<200;++round){
        for(int count=1;count<=17000;++count){
            assert(longrun.update(32768-count,32768+count,.01,0,0,&z));
            expected+=.032*2*3.141592653589793/1325;
        }
        const double at_reset=z.x[0];longrun.recenter();
        assert(longrun.update(32768,32768,.01,0,0,&z));
        assert(std::fabs(z.x[0]-at_reset)<1e-8);
    }
    assert(std::fabs(z.x[0]-expected)<1e-5);

    LqrPathModel ff={0,0,0,0,0,0};LqrEndurancePath path(ff);
    double position=0;LqrPathReference previous;for(int j=0;j<6;++j)z.x[j]=0;
    // Ideal six cycles, including continuous heading, recenter and cycle-origin accounting.
    for(int i=0;i<=21300;++i){double t=i*.01;unsigned cycle=static_cast<unsigned>(t/35.5);double local=t-cycle*35.5;
        position=cycle*.8+lqr_path_profile(local,1.5,8,.4).s+lqr_path_profile(local,18.5,8,.4).s;
        z.x[0]=position;z.x[2]=cycle*2*3.141592653589793+lqr_path_profile(local,10.5,7,3.141592653589793).s+lqr_path_profile(local,28.5,7,3.141592653589793).s;
        LqrPathReference r=path.update(t,z);
        if(i>0 && i%3550==0){assert(std::fabs(r.x[0]-previous.x[0])<1e-8);assert(std::fabs(r.x[2]-previous.x[2])<1e-6);assert(::hypot(path.world_x(),path.world_y())<1e-7);}
        previous=r;
    }
    assert(path.cycle()==7);
    // At the start of every repeated departure the ideal heading is the same.
    assert(std::fabs(std::sin(previous.x[2]))<1e-8);
    LqrEndurancePath timeline(ff);
    for(int j=0;j<6;++j)z.x[j]=0;
    assert(timeline.update(28.5,z).phase==8);
    LqrPathReference pivot=timeline.update(32,z);
    assert(std::fabs(pivot.x[2]-1.5*3.141592653589793)<1e-8 && pivot.x[5]>0);
    pivot=timeline.update(35.5,z);
    assert(pivot.phase==0 && std::fabs(pivot.x[2]-2*3.141592653589793)<1e-8);

}
