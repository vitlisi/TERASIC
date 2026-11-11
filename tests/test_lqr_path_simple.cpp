#define main simple_program_main
#include "../src_cpp/terasic_reale/controllers/balance_lqr_path_simple.cpp"
#undef main
#include <cassert>
int main(int argc,char **argv){
    assert(argc==2);LqrConfig cfg;assert(read_lqr_config(argv[1],&cfg));
    assert((cfg.path_distance==.2 || cfg.path_distance==.3 || cfg.path_distance==.4) && cfg.turn_wheel_pwm==50 && cfg.turn_diff==50);
    LqrSimplePath p(60,.2,3.141592653589793,cfg.ff);
    LqrMimoState state;for(int k=0;k<6;++k)state.x[k]=0;
    LqrPathReference r=p.update(0,state);assert(r.phase==0);
    r=p.update(3,state);assert(r.phase==1 && r.x[0]==0);
    // Deliberately miss every arrival: the clock must still run all phases.
    state.x[0]=.05;r=p.update(13,state);assert(r.phase==2 && std::fabs(r.x[0]-.2)<1e-9);
    r=p.update(15,state);assert(r.phase==3 && r.x[0]==.05);
    state.x[2]=1.;r=p.update(45,state);assert(r.phase==4);
    state.x[0]=.08;r=p.update(47,state);assert(r.phase==5 && r.x[0]==.08);
    r=p.update(57,state);assert(r.phase==6 && std::fabs(r.x[0]-.28)<1e-9);
    assert(!p.schedule_finished(59.99));assert(p.schedule_finished(60));
    r=p.update(80,state);assert(r.phase==6 && std::fabs(r.x[0]-.28)<1e-9);
    assert(std::fabs(p.planned_y(13)-.2)<1e-9 && std::fabs(p.planned_y(60))<1e-9);
    LqrSimplePath thirty(60,.3,3.141592653589793,cfg.ff);
    for(int k2=0;k2<6;++k2)state.x[k2]=0;
    thirty.update(0,state);thirty.update(3,state);
    r=thirty.update(8,state);assert(std::fabs(r.x[3]-.05625)<1e-9);
    r=thirty.update(13,state);assert(std::fabs(r.x[0]-.3)<1e-9);
    state.x[0]=.3;thirty.update(15,state);state.x[2]=3.141592653589793;
    r=thirty.update(47,state);assert(std::fabs(r.x[0]-.3)<1e-9);
    r=thirty.update(57,state);assert(std::fabs(r.x[0]-.6)<1e-9);
    assert(std::fabs(thirty.planned_y(13)-.3)<1e-9 && std::fabs(thirty.planned_y(60))<1e-9);
    LqrSimplePath timing(81,.3,3.141592653589793,cfg.ff,28,15);
    assert(timing.leg_duration()==28 && timing.turn_duration()==15);
    const double when[]={0,3,31,33,48,50,78,81};const int phases[]={0,1,2,3,4,5,6,6};
    for(int n=0;n<8;++n){r=timing.update(when[n],state);assert(r.phase==phases[n]);}
    assert(timing.schedule_finished(81) && !timing.schedule_finished(80.99));
    const LqrPathProfile pv=lqr_path_profile(17,3,28,.3),pw=lqr_path_profile(40.5,33,15,3.141592653589793);
    assert(std::fabs(pv.v-.0200892857142857)<1e-9);
    assert(std::fabs(pw.v-3.141592653589793/8)<1e-9);
    LqrSimplePath fast(34,.3,3.141592653589793,cfg.ff,8,10,2,1.5,3);
    const double fast_times[]={0,2,10,11.5,21.5,23,31,34};
    for(int n=0;n<8;++n){r=fast.update(fast_times[n],state);assert(r.phase==phases[n]);}
    assert(fast.schedule_finished(34) && !fast.schedule_finished(33.99));
    assert(std::fabs(lqr_path_profile(6,2,8,.3).v-.0703125)<1e-9);
    assert(std::fabs(lqr_path_profile(16.5,11.5,10,3.141592653589793).v-33.75*3.141592653589793/180)<1e-9);
    LqrSimplePath forty(28.5,.4,3.141592653589793,cfg.ff,8,7,1.5,1,2);
    const double forty_times[]={0,1.5,9.5,10.5,17.5,18.5,26.5,28.5};
    for(int n=0;n<8;++n){r=forty.update(forty_times[n],state);assert(r.phase==phases[n]);}
    assert(forty.schedule_finished(28.5) && !forty.schedule_finished(28.49));
    assert(path_control_sample_count(28.5,false)==2850);
    assert(path_control_sample_count(28.5,true)==3300);
    double duration=0;assert(parse_double("28.5",20,120,&duration) && duration==28.5);
    assert(!parse_double("nan",20,120,&duration));
    assert(std::fabs(lqr_path_profile(5.5,1.5,8,.4).v-.09375)<1e-9);
    assert(pwm_limit(50,50)==50 && pwm_limit(0,50)==0);
    double k[2][6]={{0,0,0,0,0,0},{0,0,0,0,0,0}};
    r.ff[0]=-80;r.ff[1]=80;
    LqrMimoCommand c=lqr_path_command(state,r,k,1,50,50);
    assert(c.applied[0]==-50 && c.applied[1]==50);
    uint32_t registers[1024]={0};fpga=registers;stop_requested=0;
    set_duties(-50,50);assert(reg_read(0x74)==25000 && reg_read(0x64)==25000);
    handle_stop(SIGTERM);set_duties(50,50);assert(!(reg_read(0x78)&1u) && !(reg_read(0x68)&1u));
    // Geometric odometry on an ideal20cm/pivot/20cm return.
    LqrSimplePath ideal(60,.2,3.141592653589793,cfg.ff);
    for(int i=0;i<=6000;++i){
        const double t=i*.01;
        state.x[0]=lqr_path_profile(t,3,10,.2).s+lqr_path_profile(t,47,10,.2).s;
        state.x[2]=lqr_path_profile(t,15,30,3.141592653589793).s;
        ideal.update(t,state);
    }
    assert(std::fabs(ideal.world_x())<1e-8 && std::fabs(ideal.world_y())<1e-8);
}
