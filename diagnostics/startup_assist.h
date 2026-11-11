#ifndef DIAGNOSTICS_STARTUP_ASSIST_H
#define DIAGNOSTICS_STARTUP_ASSIST_H
// Research-only bounded startup assistance, no operational controller imports.
// Candidate floors from repeated 12.7 V tests: SX +/-5%, DX +/-6%.
// Floors do not guarantee startup. Persistence/duration are experiment design,
// not identified motor constants. One attempt per nonzero command episode.
struct StartupAssist {
    int sign;
    unsigned origin;
    double started;
    bool finished;
    StartupAssist():sign(0),origin(0),started(0),finished(false) {}
    double update(double requested,double t,unsigned encoder,int encoder_sign,
                  unsigned wheel,bool enabled,double floor_override=-1.) {
        if(requested==0) {sign=0;finished=false;return 0.;}
        const int s=requested>0 ? 1 : -1;
        if(s!=sign) {sign=s;origin=encoder;started=t;finished=false;}
        const double elapsed=t-started;
        const int directed=(static_cast<int>(encoder)-static_cast<int>(origin))*encoder_sign*s;
        if(directed>=3 || elapsed>=.23)finished=true;
        if(!enabled || finished || elapsed<.03)return requested;
        const double floors[2][2]={{5.,5.},{6.,6.}}; // [SX/DX][forward/reverse]
        const double floor=floor_override>0 ? floor_override : floors[wheel][s>0 ? 0 : 1];
        const double mag=requested>0 ? requested : -requested;
        return s*(mag<floor ? floor : mag);
    }
};
#endif
