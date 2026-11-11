#include <cassert>
#include <limits>
#include "../src_cpp/terasic_reale/controllers/balance_hinf_core.h"
int main() {
    RealHinf h;
    LqrMimoState state={{0,.05,.01,0,0,0}};
    double sx=0,dx=0;
    assert(h.step(state,.032,&sx,&dx));assert(sx==0 && dx==0);
    assert(h.step(state,.032,&sx,&dx));assert(sx!=0 || dx!=0);
    h.reset();assert(h.step(state,.032,&sx,&dx));assert(sx==0 && dx==0);
    state.x[0]=std::numeric_limits<double>::infinity();
    assert(!h.step(state,.032,&sx,&dx));
    assert(!h.step(state,0,&sx,&dx));
    const LqrMimoCommand c=hinf_allocate(20,30,15,3);
    assert(c.raw[0]==20 && c.raw[1]==30);
    assert(c.limited[0]==15 && c.limited[1]==15);
    const LqrMimoCommand d=hinf_allocate(-5,5,15,3);
    assert(d.limited[0]==-3 && d.limited[1]==3);
    return 0;
}
