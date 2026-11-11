#ifndef TERASIC_BALANCE_HINF_CORE_H
#define TERASIC_BALANCE_HINF_CORE_H
#include "balance_lqr_mimo_core.h"
#include "balance_hinf_matrices.h"
// Same discrete controller as Python: 100 Hz, one measurement delay, zero reset.
// Real state already contains distance s, not the relative wheel angle.
class RealHinf {
public:
    double xc[12], delayed[6], feedback[6];
    RealHinf() { reset(); }
    void reset() {
        for(int i=0;i<12;++i) xc[i]=0;
        for(int i=0;i<6;++i) delayed[i]=feedback[i]=0;
    }
    bool step(const LqrMimoState &s,double radius,double *sx,double *dx) {
        if (!(radius>0) || !lqr_finite(radius)) return false;
        double measurement[6],next[12],u[2]={0,0};
        for(int i=0;i<6;++i) {
            measurement[i]=s.x[i]/((i==0 || i==3) ? radius : 1.);
            if(!lqr_finite(measurement[i])) return false;
        }
        for(int i=0;i<2;++i) {
            for(int j=0;j<12;++j) u[i]+=HINF_Ck[i][j]*xc[j];
            for(int j=0;j<6;++j) u[i]+=HINF_Dk[i][j]*delayed[j];
            if(!lqr_finite(u[i])) return false;
        }
        for(int i=0;i<12;++i) {
            next[i]=0;
            for(int j=0;j<12;++j) next[i]+=HINF_Ak[i][j]*xc[j];
            for(int j=0;j<6;++j) next[i]+=HINF_Bk[i][j]*delayed[j];
            if(!lqr_finite(next[i])) return false;
        }
        for(int i=0;i<12;++i) xc[i]=next[i];
        for(int i=0;i<6;++i) { feedback[i]=delayed[i]; delayed[i]=measurement[i]; }
        *sx=100*u[1]; *dx=100*u[0]; // duty R,L -> percent SX,DX; no minus sign.
        return true;
    }
};
// Reuse exactly the tested common/differential allocator, without LQR feedback.
inline LqrMimoCommand hinf_allocate(double sx,double dx,double limit,double diff) {
    LqrMimoState s={{1,0,0,0,0,0}};
    double k[2][6]={{0,0,0,0,0,0},{0,0,0,0,0,0}};
    k[0][0]=-sx; k[1][0]=-dx;
    return lqr_mimo_command(s,k,1,limit,diff);
}
#endif
