#include <cassert>
#include <limits>
#include "../src_cpp/terasic_reale/controllers/balance_startup_limits.h"
int main() {
    const double ref=.320;
    assert(balance_startup_pitch_ok(ref,ref));
    assert(balance_startup_pitch_ok(ref+3,ref));
    assert(balance_startup_pitch_ok(ref-3,ref));
    assert(balance_startup_pitch_ok(ref+5,ref));
    assert(balance_startup_pitch_ok(ref-5,ref));
    assert(!balance_startup_pitch_ok(ref+5.001,ref));
    assert(!balance_startup_pitch_ok(ref-5.001,ref));
    // Old absolute threshold wrongly rejected a valid 4.9-degree error.
    assert(balance_startup_pitch_ok(5.22,ref));
    assert(!balance_startup_pitch_ok(std::numeric_limits<double>::quiet_NaN(),ref));
    assert(!balance_startup_pitch_ok(std::numeric_limits<double>::infinity(),ref));
    assert(balance_startup_pitch_ok(ref+.49,ref,.5));
    assert(!balance_startup_pitch_ok(ref+.51,ref,.5));
    assert(!balance_startup_pitch_ok(ref,ref,6));
    assert(!balance_startup_pitch_ok(ref,ref,0));
    return 0;
}
