#ifndef TERASIC_ACCEL_NORM_GUARD_H
#define TERASIC_ACCEL_NORM_GUARD_H
#include <limits>
// At100Hz, reject isolated norm outliers for attitude correction. Disarm on
// three consecutive outliers; independent pitch/gyro/az/deadline checks remain.
class AccelNormGuard {
    unsigned bad_;
    bool nonfinite_;
public:
    AccelNormGuard():bad_(0),nonfinite_(false){}
    bool update(double g){
        nonfinite_=!(g==g && g<=std::numeric_limits<double>::max() &&
                     g>=-std::numeric_limits<double>::max());
        const bool trusted=!nonfinite_ && g>=.6 && g<=1.4;
        if(trusted)bad_=0;else if(bad_<3)++bad_;
        return trusted;
    }
    bool failed()const{return nonfinite_ || bad_>=3;}
    unsigned bad_count()const{return bad_;}
};
#endif
