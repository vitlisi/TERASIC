#include <cassert>
#include <limits>
#include "../scripts/lqr_mimo_sim_bridge.cpp"
int main() {
    const double gains[12]={0,-736,0,0,0,0,0,-736,0,0,0,0};
    double out[12];
    void *low=mimo_create(gains,-3.32);
    void *high=mimo_create_limited(gains,-3.32,50,3);
    assert(low && high);
    assert(mimo_step(low,32768,32768,-3.32,0,out));
    assert(out[8]==-15 && out[9]==-15);
    assert(mimo_step(high,32768,32768,-3.32,0,out));
    assert(out[8]<-40 && out[8]>-50 && out[8]==out[9]);
    mimo_destroy(low);mimo_destroy(high);
    assert(!mimo_create_limited(gains,0,-1,3));
    assert(!mimo_create_limited(gains,0,15,16));
    assert(!mimo_create_limited(gains,0,std::numeric_limits<double>::quiet_NaN(),3));
    return 0;
}
