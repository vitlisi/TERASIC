#include "../diagnostics/stribeck_lut_compensation.h"
#include <cassert>
#include <iostream>
#include <limits>
int main() {
    typedef StribeckLutCompensation C;C c;
    unsigned e[]={32768,32768};double u[]={3,3};C::Output r;
    for(int k=0;k<50;++k) {
        r=c.update(u,e,k*.01);assert(r.valid);
        assert(r.applied[0]>=3 && r.applied[0]<=3.5);
        assert(r.applied[1]>=3 && r.applied[1]<=3.5);
        if(k<10)assert(r.applied[0]==3);
    }
    assert(r.applied[0]==3.5 && r.applied[1]==3.5);
    // True neutral cannot emit a retained boost.
    u[0]=u[1]=0;r=c.update(u,e,.50);assert(r.valid && r.applied[0]==0 && r.applied[1]==0);
    // A request reversal must satisfy persistence again.
    u[0]=u[1]=-3;r=c.update(u,e,.51);assert(r.valid && r.applied[0]==-3 && r.applied[1]==-3);
    C moving;
    for(int k=0;k<100;++k) {e[0]-=2;e[1]+=2;u[0]=u[1]=3;r=moving.update(u,e,k*.01);assert(r.valid);}
    u[0]=u[1]=-3;r=moving.update(u,e,1.);assert(r.valid && r.applied[0]==-3 && r.applied[1]==-3);
    // The qualified LQR allocator supplies inputs within caps. Verify final
    // driver quantization and pair projection on varied asymmetric requests.
    C cap;unsigned enc[]={32768,32768};
    for(int k=0;k<3000;++k) {
        const double common=19*std::sin(k*.013),d=std::min(3.,20-std::fabs(common))*std::cos(k*.019);
        u[0]=common-d;u[1]=common+d;
        enc[0]+=k%5==0 ? 1:-1;enc[1]+=k%3==0 ? -1:1;
        r=cap.update(u,enc,k*.01);assert(r.valid);
        for(unsigned w=0;w<2;++w) {
            assert(std::fabs(r.applied[w])<=20.000001);
            assert(std::fabs(r.target[w]-r.nominal[w])<=.500001);
            assert(std::fabs(r.correction[w])<=.502001);
            if(std::fabs(r.nominal[w])>=8)assert(std::fabs(r.target[w]-r.nominal[w])<1e-12);
            assert(r.applied[w]*r.nominal[w]>=0);
        }
        assert(std::fabs(r.applied[1]-r.applied[0])<=6.000001);
    }
    C invalid;u[0]=u[1]=3;enc[0]=enc[1]=32768;
    assert(invalid.update(u,enc,0).valid);
    assert(!invalid.update(u,enc,0).valid);
    assert(!invalid.update(u,enc,.03).valid);
    invalid.set_voltage(11.19);assert(!invalid.update(u,enc,.01).valid);
    invalid.set_voltage(11.5);u[1]=std::numeric_limits<double>::quiet_NaN();assert(!invalid.update(u,enc,.01).valid);
    u[1]=3;enc[1]=65535;assert(!invalid.update(u,enc,.01).valid);
    assert(C::delta_torque(0,1,0)!=C::delta_torque(0,-1,0));
    assert(C::delta_torque(0,1,0)!=C::delta_torque(1,1,0));
    std::cout<<"PASS: bounded Stribeck LUT, neutral, persistence, braking, voltage, timing, encoder and physical pair caps\n";
}
