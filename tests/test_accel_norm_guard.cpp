#include "../src_cpp/terasic_reale/controllers/accel_norm_guard.h"
#include <cassert>
int main(){
    AccelNormGuard g;
    assert(g.update(.978686));
    assert(!g.update(1.403) && !g.failed() && g.bad_count()==1);
    assert(g.update(.99) && !g.failed() && g.bad_count()==0);
    assert(!g.update(1.41) && !g.failed());
    assert(!g.update(1.42) && !g.failed());
    assert(!g.update(1.43) && g.failed());
    AccelNormGuard low;assert(!low.update(.5) && !low.failed());
    assert(!low.update(.5) && !low.failed());assert(!low.update(.5) && low.failed());
    AccelNormGuard invalid;invalid.update(std::numeric_limits<double>::quiet_NaN());assert(invalid.failed());
    AccelNormGuard infinite;infinite.update(std::numeric_limits<double>::infinity());assert(infinite.failed());
    AccelNormGuard edge;assert(edge.update(.6) && edge.update(1.4));
}
