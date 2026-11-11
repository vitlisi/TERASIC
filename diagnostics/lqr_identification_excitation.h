#ifndef LQR_IDENTIFICATION_EXCITATION_H
#define LQR_IDENTIFICATION_EXCITATION_H
#include <cmath>

// Fixed deterministic probe: peak <=0.5 percentage points, active only2..8s.
// All frequencies complete integer cycles in the6s window before tapering.
inline double identification_probe(double seconds)
{
    if (!(seconds>=2. && seconds<8.)) return 0.;
    const double pi=3.14159265358979323846,t=seconds-2.;
    const double frequencies[6]={.5,1.,2.,4.,6.,8.};
    double sum=0.;
    for(unsigned i=0;i<6;++i)sum+=std::sin(2*pi*frequencies[i]*t);
    double envelope=1.;
    if(t<.5)envelope=.5*(1-std::cos(pi*t/.5));
    if(t>5.5)envelope=.5*(1-std::cos(pi*(6-t)/.5));
    return .5*envelope*sum/6.;
}

struct IdentificationCommand { double target[2],requested,allocated; bool valid; };

inline IdentificationCommand identification_command(const double base[2],double seconds,double cap)
{
    IdentificationCommand out={{0,0},0,0,false};
    if(!(cap>=0 && cap<=20 && seconds>=0 && seconds<=10) ||
       !(base[0]>=-cap && base[0]<=cap && base[1]>=-cap && base[1]<=cap))return out;
    out.requested=identification_probe(seconds);
    const double high=cap-(base[0]>base[1]?base[0]:base[1]);
    const double low=-cap-(base[0]<base[1]?base[0]:base[1]);
    out.allocated=out.requested>high?high:out.requested<low?low:out.requested;
    for(unsigned w=0;w<2;++w)out.target[w]=base[w]+out.allocated;
    out.valid=true;return out;
}
#endif
