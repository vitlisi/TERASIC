#define main unused_path_program_main
#include "../src_cpp/terasic_reale/controllers/balance_lqr_path_simple.cpp"
#undef main
struct PathSimulation {
    LqrMimoEstimator encoder;
    double pitch,k[2][6],distance,turn,absolute_reference;
    LqrPathYawFusion yaw;
    int mode;
    LqrPathModel model;
    AccelNormGuard accel_guard;
    double accel_norm;
    LqrSimplePath *path;
    PathSimulation(const double *gains,const double *ff,double initial,int m,double d,double angle,double absolute,bool continuous=false)
      :encoder(1325,.032,0.16444,initial*3.141592653589793/180.),pitch(initial),
       distance(d),turn(angle),absolute_reference(absolute),yaw(60),mode(m),accel_norm(1)
    {
        for(int i=0;i<2;++i)for(int j=0;j<6;++j)k[i][j]=gains[6*i+j];
        model.pitch_v=ff[0];model.pitch_a=ff[1];model.common_v=ff[2];model.common_a=ff[3];
        model.diff_w=ff[4];model.diff_alpha=ff[5];
        path=new LqrSimplePath(28.5,distance,turn,model,8,7,1.5,1,2,continuous);
    }
    ~PathSimulation(){delete path;}
};
extern "C" {
void *path_create(const double *k,const double *ff,double initial,int mode,double d,double turn,double absolute)
{return new PathSimulation(k,ff,initial,mode,d,turn,absolute);}
void *path_create_continuous(const double *k,const double *ff,double initial,int mode,double d,double turn,double absolute)
{return new PathSimulation(k,ff,initial,mode,d,turn,absolute,true);}
void path_set_accel_norm(void *p,double g){static_cast<PathSimulation *>(p)->accel_norm=g;}
void path_destroy(void *p){delete static_cast<PathSimulation *>(p);}
int path_step(void *p,unsigned sx,unsigned dx,double accel,double rate,double gx,double gz,double time,double *out)
{
    PathSimulation &s=*static_cast<PathSimulation *>(p);
    const bool trusted=s.accel_guard.update(s.accel_norm);
    s.pitch=trusted ? lqr_fuse_pitch_deg(s.pitch,rate,.01,accel) : s.pitch+rate*.01;
    LqrMimoState state;
    if(!s.encoder.update(sx,dx,.01,s.pitch*3.141592653589793/180.,
                        rate*3.141592653589793/180.,&state))return 0;
    const double encoder_yaw=state.x[2];
    const double gyro_yaw=lqr_path_yaw_rate_rad_s(gx,gz,s.pitch+s.absolute_reference,1);
    state.x[2]=s.yaw.update(encoder_yaw,gyro_yaw,.01);state.x[5]=gyro_yaw;
    const LqrPathReference ref=s.path->update(time,state);
    const bool envelope=!s.accel_guard.failed() && std::fabs(state.x[3])<=.30 && std::fabs(state.x[5])<=1.5707963268;
    const double differential=50;
    const LqrMimoCommand c=lqr_path_command(state,ref,s.k,1,50,differential);
    if(!lqr_finite(c.raw[0]) || !lqr_finite(c.raw[1]))return 0;
    for(int i=0;i<6;++i){out[i]=state.x[i];out[12+i]=ref.x[i];}
    out[6]=c.raw[0];out[7]=c.raw[1];out[8]=c.applied[0];out[9]=c.applied[1];
    out[10]=c.common_raw;out[11]=c.differential_raw;
    out[18]=ref.phase;out[19]=envelope ? 1 : 0;
    out[20]=ref.ff[0];out[21]=ref.ff[1];out[22]=encoder_yaw;
    out[23]=s.path->world_x();out[24]=s.path->world_y();out[25]=std::sqrt(out[23]*out[23]+out[24]*out[24]);
    out[26]=s.path->schedule_finished(time)?1:0;out[27]=s.path->planned_y(time);out[28]=s.accel_guard.bad_count();out[29]=trusted?1:0;return 1;
}
}
