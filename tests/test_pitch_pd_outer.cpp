#include "../src_cpp/terasic_reale/controllers/pitch_pd_outer_core.h"
#include <cassert>
#include <cmath>

int main()
{
    PitchPdOuterEstimator estimator(0.0);
    double x=0,v=0;
    assert(estimator.update(32768,32768,0,0,0,&x,&v));
    assert(std::fabs(x)<1e-12 && std::fabs(v)<1e-12);
    // Opposite raw count signs are the same forward wheel motion.
    assert(estimator.update(32668,32868,.01,0,0,&x,&v));
    assert(x>0 && v>0);
    assert(!estimator.update(0,32868,.01,0,0,&x,&v));
    assert(std::fabs(pitch_pd_outer_target(0,0,2,8))<1e-12);
    assert(pitch_pd_outer_target(.1,.1,2,8)<0);
    assert(pitch_pd_outer_target(10,10,2,8)==-3.0);
    assert(pitch_pd_outer_target(-10,-10,2,8)==3.0);
}
