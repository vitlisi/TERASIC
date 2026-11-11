// Offline adapter: compile the actual hardware parser and controller cores.
// No hardware entry point is called by this library.
#define main twin_unused_hardware_main
#ifdef TWIN_HINF_PATH
#define TWIN_HINF
#include "../src_cpp/terasic_reale/controllers/balance_hinf_path.cpp"
#elif defined(TWIN_PATH)
#include "../src_cpp/terasic_reale/controllers/balance_lqr_path_simple.cpp"
#elif defined(TWIN_HINF)
#include "../src_cpp/terasic_reale/controllers/balance_hinf.cpp"
typedef HinfConfig LqrConfig;
static bool read_lqr_config(const char *path,LqrConfig *config) { return read_hinf_config(path,config); }
#else
#include "../src_cpp/terasic_reale/controllers/balance_lqr_mimo.cpp"
#endif
#undef main

struct Twin {
    LqrConfig config;
    LqrMimoEstimator *encoder;
    double pitch, cap;
#ifdef TWIN_HINF
    RealHinf hinf;
#endif
#ifdef TWIN_PATH
    LqrSimplePath *path;
    LqrPathYawFusion *yaw;
    AccelNormGuard accel_guard;
#endif
    Twin():encoder(NULL),pitch(0),cap(0)
#ifdef TWIN_PATH
        ,path(NULL),yaw(NULL)
#endif
    {}
    ~Twin(){delete encoder;
#ifdef TWIN_PATH
        delete path;delete yaw;
#endif
    }
};

extern "C" {
void *twin_create(const char *file,double initial,double cap) {
    Twin *s=new Twin;
    if(!read_lqr_config(file,&s->config) || !lqr_finite(initial) ||
       !lqr_finite(cap) || cap<=0 || cap>100) {delete s;return NULL;}
#ifdef TWIN_HINF
    if(cap>25) {delete s;return NULL;}
#endif
    s->pitch=initial;s->cap=cap;
    const LqrConfig &c=s->config;
    s->encoder=new LqrMimoEstimator(c.cpr,c.radius,c.track,initial*3.141592653589793/180.);
#ifdef TWIN_PATH
    s->cap=pwm_limit(cap,c.turn_wheel_pwm);
#ifdef TWIN_HINF_PATH
    if(c.turn_yaw_reserve_percent>=s->cap){delete s;return NULL;}
#endif
    const double duration=2*c.leg_duration_s+c.turn_duration_s+c.initial_hold_s+2*c.between_hold_s+c.final_hold_s;
    s->path=new LqrSimplePath(duration,c.path_distance,c.path_turn,c.ff,c.leg_duration_s,
        c.turn_duration_s,c.initial_hold_s,c.between_hold_s,c.final_hold_s,c.continuous_reference);
    s->yaw=new LqrPathYawFusion(c.yaw_tau);
#endif
    return s;
}
void twin_destroy(void *p){delete static_cast<Twin *>(p);}
// Output: state6, reference6, raw2, limited2, applied2, ff2,
// phase, schedule_finished, world_x/y, fault, accel_trusted.
// TWIN_HINF requires 44 outputs: append feedback6 and controller state12.
int twin_step(void *p,unsigned sx,unsigned dx,double accel,double gy,double gx,double gz,
              double norm,double az,double dt,double time,double *out) {
    Twin &s=*static_cast<Twin *>(p);const LqrConfig &c=s.config;
    for(int i=0;i<26;++i)out[i]=0;
    bool trusted=true;
#ifdef TWIN_PATH
    trusted=s.accel_guard.update(norm);
#endif
    s.pitch=trusted ? lqr_fuse_pitch_deg(s.pitch,gy,dt,accel) : s.pitch+gy*dt;
    out[25]=trusted?1:0;
    bool bad_accel=norm<.6 || norm>1.4;
#ifdef TWIN_PATH
    bad_accel=s.accel_guard.failed();
#endif
    if(bad_accel || az<8000 || std::fabs(s.pitch)>12 || std::fabs(accel)>15 || std::fabs(gy)>180)
        {out[24]=1;return 0;}
    LqrMimoState state;
    if(!s.encoder->update(sx,dx,dt,s.pitch*3.141592653589793/180.,gy*3.141592653589793/180.,&state))
        {out[24]=2;return 0;}
    LqrMimoCommand command;
#ifdef TWIN_PATH
    const double rate=lqr_path_yaw_rate_rad_s(gx,gz,s.pitch+c.pitch_reference_deg,c.yaw_gyro_sign);
    state.x[2]=s.yaw->update(state.x[2],rate,dt);state.x[5]=rate;
    const LqrPathReference ref=s.path->update(time,state);
#ifdef TWIN_HINF_PATH
    if(!hinf_path_command(s.hinf,state,ref,c.radius,s.cap,s.cap,&command,c.turn_yaw_reserve_percent)){out[24]=4;return 0;}
    for(int i=0;i<6;++i)out[26+i]=s.hinf.feedback[i];
    for(int i=0;i<12;++i)out[32+i]=s.hinf.xc[i];
#else
    command=lqr_path_command(state,ref,c.k,c.motor_sign,s.cap,s.cap);
#endif
    for(int i=0;i<6;++i)out[6+i]=ref.x[i];
    out[18]=ref.ff[0];out[19]=ref.ff[1];out[20]=ref.phase;
    out[21]=s.path->schedule_finished(time)?1:0;
    out[22]=s.path->world_x();out[23]=s.path->world_y();
#else
    (void)gx;(void)gz;(void)time;
#ifdef TWIN_HINF
    double left=0,right=0;
    if(!s.hinf.step(state,c.radius,&left,&right)) {out[24]=4;return 0;}
    command=hinf_allocate(left,right,s.cap,c.max_diff);
    for(int i=0;i<6;++i)out[26+i]=s.hinf.feedback[i];
    for(int i=0;i<12;++i)out[32+i]=s.hinf.xc[i];
#else
    command=lqr_mimo_command(state,c.k,c.motor_sign,s.cap,c.max_diff);
#endif
    if(std::fabs(state.x[0])>.12 || std::fabs(state.x[2])>.3490658504)out[24]=3;
#endif
    if(std::fabs(state.x[3])>.30 || std::fabs(state.x[5])>1.5707963268)out[24]=3;
    for(int i=0;i<6;++i)out[i]=state.x[i];
    for(int i=0;i<2;++i){out[12+i]=command.raw[i];out[14+i]=command.limited[i];out[16+i]=command.applied[i];}
    if(!lqr_finite(command.raw[0]) || !lqr_finite(command.raw[1]))out[24]=4;
    return out[24]==0 ? 1:0;
}
}
