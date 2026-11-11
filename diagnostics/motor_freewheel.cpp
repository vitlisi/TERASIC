// Shared MMIO map, process lock and stop-on-signal semantics with motor_pulse.
#define MOTOR_PULSE_HELPERS_ONLY
#include "motor_pulse.cpp"
#include "startup_assist.h"
#include "pwm_motion_smooth_compensation.h"
#ifdef LOW_COMMAND_CANDIDATE_STEPS
#include "pwm_low_command_compensation.h"
typedef PwmLowCommandCompensation CandidateStepMapper;
static const unsigned candidate_block_seconds=26;
#ifdef BASELINE_REVERSE_STEPS
static const double candidate_levels[]={-3.,-5.,-10.,-15.,-20.};
#else
static const double candidate_levels[]={3.,5.,10.,15.,20.};
#endif
#ifdef BASELINE_RECHECK_STEPS
#ifdef BASELINE_REVERSE_STEPS
static const char *candidate_protocol_id="freewheel_baseline_reverse_v1";
#else
static const char *candidate_protocol_id="freewheel_baseline_recheck_v1";
#endif
#else
static const char *candidate_protocol_id="freewheel_low_corrected_steps_v1";
#endif
#else
typedef PwmMotionCompensationSmooth CandidateStepMapper;
static const unsigned candidate_block_seconds=21;
static const double candidate_levels[]={3.,10.,15.,20.};
static const char *candidate_protocol_id="freewheel_candidate_steps_v2";
#endif
#ifdef BASELINE_RECHECK_STEPS
static const bool candidate_actuation_enabled=false;
#else
static const bool candidate_actuation_enabled=true;
#endif
static const unsigned candidate_level_count=sizeof(candidate_levels)/sizeof(candidate_levels[0]);
#include <sched.h>

static double duration_s=15.;
static const double sample_dt=.01;
static bool small_commands=false;
static bool startup_ramp=false;
static bool static_characteristic=false;
static bool holding_curve=false;
static bool startup_low=false;
static bool repeat_low=false;
static bool assist_comparison=false;
static bool branch_scan=false;
static int branch_sign=1;
static unsigned ramp_steps=32;
static unsigned ramp_wheel=0; // 0=SX, 1=DX; selected by protocol_pwm.
static const unsigned pwm_period=50000;
static bool inverse_mode=false;
static bool inverse_cold=false;
static bool inverse_identify=false;
static bool regime_comparison=false;
static bool candidate_step_comparison=false;
static const double identify_levels[]={10.,9.,8.,7.,6.,5.5,5.,4.5,4.,3.5,3.,2.5,2.};
struct InverseStep { unsigned wheel,on; double command,target,pwm; };
static InverseStep inverse_steps[6];

// Fixed, bounded diagnostic format generated from the research LUT on the PC.
static bool load_inverse_plan(const char *path)
{
    FILE *f=fopen(path,"r");
    if(!f)return false;
    bool valid=true;
    for(unsigned i=0;i<6;++i) {
        InverseStep &s=inverse_steps[i];
        if(fscanf(f,"%u %u %lf %lf %lf",&s.wheel,&s.on,&s.command,&s.target,&s.pwm)!=5) {valid=false;break;}
        const double sign=s.command>0 ? 1. : -1.;
        if(s.wheel>1 || s.on!=i%2 || !(sign*s.command>0 && sign*s.command<=10.) ||
           !(sign*s.target>0 && sign*s.target<=3.) ||
           !(sign*s.pwm>0 && sign*s.pwm<=10.) ||
           (!s.on && s.pwm!=s.command) ||
           (i && (s.wheel!=inverse_steps[0].wheel || s.command*inverse_steps[0].command<=0)) ||
           (i%2 && (s.command!=inverse_steps[i-1].command || s.target!=inverse_steps[i-1].target)) ||
           (i>=2 && i%2==0 && sign*s.command<=sign*inverse_steps[i-2].command))valid=false;
    }
    char extra;
    if(fscanf(f," %c",&extra)!=EOF)valid=false;
    fclose(f);
    if(valid) {
        inverse_mode=true;branch_scan=true;ramp_wheel=inverse_steps[0].wheel;
        branch_sign=inverse_steps[0].command>0 ? 1 : -1;duration_s=43.;
    }
    return valid;
}

static double protocol_pwm(double t, unsigned *phase)
{
    *phase=0;
    if(candidate_step_comparison) {
        const unsigned block=t>=candidate_block_seconds ? 1:0;
        const double local=t-candidate_block_seconds*block;
        for(unsigned i=0;i<candidate_level_count;++i)if(local>=1.+5.*i && local<3.+5.*i) {
            *phase=1+i+candidate_level_count*block;return candidate_levels[i];
        }
        return 0.;
    }
    if(regime_comparison) {
        // One acquisition, two wheels x two initial directions. Each block:
        // cold low2, OFF2, same-direction warmup1, hold low2, reverse low2, OFF2.
        for(unsigned b=0;b<4;++b) {
            const double local=t-(1.+11.*b);
            if(local>=0 && local<11.) {
                ramp_wheel=b/2;const double sign=b%2 ? -1. : 1.;
                const double low=ramp_wheel==0 ? 4. : 5.5;
                if(local<2.) {*phase=1+4*b;return sign*low;}
                if(local<4.)return 0.;
                if(local<5.) {*phase=2+4*b;return sign*10.;}
                if(local<7.) {*phase=3+4*b;return sign*low;}
                if(local<9.) {*phase=4+4*b;return -sign*low;}
                return 0.;
            }
        }
        return 0.;
    }
    if(inverse_identify) {
        for(unsigned block=0;block<2;++block) {
            const double local=t-(1.+42.*block);
            if(local>=0 && local<39.) {
                const unsigned step=static_cast<unsigned>(local/3.);
                *phase=1+13*block+step;return branch_sign*identify_levels[step];
            }
        }
        return 0.;
    }
    if(inverse_mode) {
        if(inverse_cold) {
            if(t<1. || t>=37.)return 0.;
            const unsigned i=static_cast<unsigned>((t-1.)/6.);
            if(t-1.-6.*i<3.) {*phase=2*i+2;return inverse_steps[i].pwm;}
            return 0.;
        }
        if(t<1. || t>=43.)return 0.;
        const unsigned i=static_cast<unsigned>((t-1.)/7.);
        const double local=t-1.-7.*i;
        if(local<2.) {*phase=2*i+1;return branch_sign*10.;}
        if(local<5.) {*phase=2*i+2;return inverse_steps[i].pwm;}
        return 0.;
    }
    if(branch_scan) {
        // Isolated cold starts, selected wheel only; 2s command + 2s OFF.
        for(unsigned i=0;i<5;++i) {
            const double start=1.+4.*i;
            if(t>=start && t<start+2.) {*phase=i+1;return branch_sign*(4.+i);}
        }
        return 0.;
    }
    if(assist_comparison) {
        // Two paired repetitions per direction: OFF/ON at nominal +/-4%.
        for(unsigned i=0;i<8;++i) {
            const double start=1.+4.*i;
            if(t>=start && t<start+2.) {*phase=i+1;return (i/2)%2 ? -4. : 4.;}
        }
        return 0.;
    }
    if(repeat_low) {
        const double levels[]={5.,-5.,6.,-6., -6.,6.,-5.,5.,
                               6.,5.,-6.,-5., -5.,-6.,5.,6., 5.,6.,-5.,-6.};
        for(unsigned i=0;i<20;++i) {
            const double start=1.+4.*i;
            if(t>=start && t<start+2.) {*phase=i+1;return levels[i];}
        }
        return 0.;
    }
    if(holding_curve) {
        const double levels[]={10.,8.,6.,5.,4.,3.,2.};
        for(unsigned block=0;block<4;++block) {
            const double local=t-(1.+18.*block);
            if(local>=0 && local<14.) {
                const unsigned step=static_cast<unsigned>(local/2.);
                *phase=1+7*block+step;
                return (block%2 ? -1. : 1.)*levels[step];
            }
        }
        return 0.;
    }
    if(static_characteristic) {
        const double levels[]={3.,-3.,4.,-4.,5.,-5.,6.,-6.,8.,-8.,
                               10.,-10.,15.,-15.,20.,-20.,-5.,5.,-6.,6.};
        for(unsigned i=0;i<(startup_low ? 8u : 20u);++i) {
            const double start=1.+4.*i;
            if(t>=start && t<start+2.) {*phase=i+1;return levels[i];}
        }
        return 0.;
    }
    if(startup_ramp) {
        // Four independent sweeps: SX+/SX-/DX+/DX-, 6 s OFF between ramps.
        for(unsigned block=0;block<4;++block) {
            const double local=t-(1.+(.5*ramp_steps+6.)*block);
            if(local>=0 && local<.5*ramp_steps) {
                ramp_wheel=block/2;
                const unsigned step=static_cast<unsigned>(local/.5);
                *phase=1+ramp_steps*block+step;
                return (block%2 ? -1. : 1.)*.25*(step+1);
            }
        }
        return 0.;
    }
    if(small_commands) {
        const double isolated[]={3.,-3.,5.,-5.,8.,-8.};
        for(unsigned i=0;i<6;++i) {
            const double start=1.+2.*i;
            if(t>=start && t<start+.5) {*phase=i+1;return isolated[i];}
        }
        const double levels[]={3.,5.,8.};
        for(unsigned block=0;block<3;++block) {
            const double start=14.+4.*block;
            if(t>=start && t<start+2.) {
                const unsigned step=static_cast<unsigned>((t-start)/.25);
                *phase=7+8*block+step;
                return step%2 ? -levels[block] : levels[block];
            }
        }
        return 0.;
    }
    const double starts[]={1.,6.,11.}, values[]={10.,15.,20.};
    for (unsigned i=0;i<3;++i) if(t>=starts[i] && t<starts[i]+1.) {
        *phase=i+1; return values[i];
    }
    return 0.;
}

// Block stop signals over the transaction so a stop cannot be overwritten.
static void freewheel_apply(double pwm)
{
    sigset_t set,old;
    sigemptyset(&set);
    sigaddset(&set,SIGINT);sigaddset(&set,SIGTERM);
    sigaddset(&set,SIGHUP);sigaddset(&set,SIGALRM);
    sigprocmask(SIG_BLOCK,&set,&old);
    if(stop_requested || pwm==0) stop_motors();
    else {
        const unsigned control=pwm>0 ? 0x07 : 0x05;
        // Drop RUN before changing direction, as in the real controller.
        if((((!startup_ramp && !branch_scan) || ramp_wheel==0) && (rd(robot_hw::PWM_SX_CONTROL)&2u)!=(control&2u)) ||
           (((!startup_ramp && !branch_scan) || ramp_wheel==1) && (rd(robot_hw::PWM_DX_CONTROL)&2u)!=(control&2u))) stop_motors();
        const double magnitude=pwm>0 ? pwm : -pwm;
        const unsigned high=static_cast<unsigned>(pwm_period*magnitude/100.);
        wr(robot_hw::PWM_SX_HIGH,(startup_ramp || branch_scan) && ramp_wheel!=0 ? 0 : high);
        wr(robot_hw::PWM_DX_HIGH,(startup_ramp || branch_scan) && ramp_wheel!=1 ? 0 : high);
        wr(robot_hw::PWM_SX_CONTROL,(startup_ramp || branch_scan) && ramp_wheel!=0 ? 6 : control);
        wr(robot_hw::PWM_DX_CONTROL,(startup_ramp || branch_scan) && ramp_wheel!=1 ? 6 : control);
    }
    sigprocmask(SIG_SETMASK,&old,NULL);
}

static void assist_apply(double sx,double dx,double cap=6.)
{
    sigset_t set,old;sigemptyset(&set);
    sigaddset(&set,SIGINT);sigaddset(&set,SIGTERM);
    sigaddset(&set,SIGHUP);sigaddset(&set,SIGALRM);
    sigprocmask(SIG_BLOCK,&set,&old);
    if(stop_requested)stop_motors();
    else {
        const double command[]={sx,dx};
        const unsigned controls[]={robot_hw::PWM_SX_CONTROL,robot_hw::PWM_DX_CONTROL};
        const unsigned highs[]={robot_hw::PWM_SX_HIGH,robot_hw::PWM_DX_HIGH};
        // Stop before reversal; both commands must stay inside bench cap.
        for(unsigned i=0;i<2;++i)
            if(!lqr_finite(command[i]) || command[i]>cap || command[i]<-cap)stop_requested=1;
        if(stop_requested)stop_motors();
        else {
            for(unsigned i=0;i<2;++i) {
                const unsigned control=command[i]>0 ? 7 : 5;
                if(command[i] && (rd(controls[i])&2u)!=(control&2u))stop_motors();
            }
            for(unsigned i=0;i<2;++i) {
                const double mag=command[i]>0 ? command[i] : -command[i];
                wr(highs[i],static_cast<unsigned>(pwm_period*mag/100.));
                wr(controls[i],command[i] ? (command[i]>0 ? 7 : 5) : 6);
            }
        }
    }
    sigprocmask(SIG_SETMASK,&old,NULL);
}

struct BatteryAcquisition {
    bool pending;
    double started,next,measured,voltage;
    unsigned raw;
    BatteryAcquisition():pending(false),started(0),next(0),measured(-1),voltage(0),raw(0) {}
    bool update(double t) {
        if(pending) {
            if(rd(0x90)&1u) {
                raw=rd(0x94);
                if(!raw || raw>=4095) return false;
                voltage=raw*18./4.7/1000.;measured=t;pending=false;next=t+.09;
                if(voltage<10.5 || voltage>13.) return false;
            } else if(t-started>.025) return false;
        }
        if(!pending && t>=next) {
            wr(0x94,1);wr(0x90,2);wr(0x90,3);wr(0x90,2);
            pending=true;started=t;
        }
        return true;
    }
};

struct FreewheelSample {
    PulseSample hw;
    double requested,battery,age;
    unsigned battery_raw;
    double candidate_target[2],candidate_omega[2];
    int candidate_regime[2],candidate_support[2];
    bool candidate_on;
};

static bool safe_counters(const PulseSample &s)
{
    // No assumption that the FPGA counter wraps rather than saturates.
    return s.sx>256 && s.sx<65279 && s.dx>256 && s.dx<65279;
}

int main(int argc,char **argv)
{
    if(argc>1 && !strcmp(argv[argc-1],"--candidate-step-comparison")) {
        candidate_step_comparison=true;duration_s=(candidate_actuation_enabled ? 2.:1.)*candidate_block_seconds;--argc;
    }
    if(argc>1 && !strcmp(argv[argc-1],"--regime-comparison")) {
        regime_comparison=true;branch_scan=true;duration_s=45.;--argc;
    }
    const char *identify_options[]={"--inverse-identify-sx-forward","--inverse-identify-sx-reverse","--inverse-identify-dx-forward","--inverse-identify-dx-reverse"};
    for(unsigned b=0;b<4;++b)if(argc>1 && !strcmp(argv[argc-1],identify_options[b])) {
        inverse_identify=true;branch_scan=true;ramp_wheel=b/2;branch_sign=b%2 ? -1 : 1;
        duration_s=85.;--argc;break;
    }
    if(argc>=3 && (!strcmp(argv[argc-2],"--inverse-plan") || !strcmp(argv[argc-2],"--inverse-cold-plan"))) {
        inverse_cold=!strcmp(argv[argc-2],"--inverse-cold-plan");
        if(!load_inverse_plan(argv[argc-1])) {
            fprintf(stderr,"Piano inversa non valido: nessun accesso hardware.\n");return 1;
        }
        argc-=2;
        if(inverse_cold)duration_s=37.;
    }
    if((inverse_mode && inverse_identify) || (regime_comparison && (inverse_mode || inverse_identify))) {
        fprintf(stderr,"Scegliere un solo protocollo inversa. Nessun accesso hardware.\n");return 1;
    }
    const char *branch_options[]={"--branch-sx-forward","--branch-sx-reverse","--branch-dx-forward","--branch-dx-reverse"};
    for(unsigned b=0;b<4;++b)if(argc>1 && !strcmp(argv[argc-1],branch_options[b])) {
        branch_scan=true;ramp_wheel=b/2;branch_sign=b%2 ? -1 : 1;duration_s=21.;--argc;break;
    }
    if(!branch_scan && argc>1 && !strcmp(argv[argc-1],"--startup-assist")) {
        assist_comparison=true;duration_s=33.;--argc;
    }
    else if(argc>1 && !strcmp(argv[argc-1],"--repeat-low")) {
        repeat_low=true;duration_s=81.;--argc;
    }
    else if(argc>1 && !strcmp(argv[argc-1],"--startup-low")) {
        startup_low=true;static_characteristic=true;duration_s=33.;--argc;
    }
    else if(argc>1 && !strcmp(argv[argc-1],"--holding-curve")) {
        holding_curve=true;duration_s=73.;--argc;
    }
    else if(argc>1 && !strcmp(argv[argc-1],"--static-characteristic")) {
        static_characteristic=true;duration_s=81.;--argc;
    }
    else if(argc>1 && !strcmp(argv[argc-1],"--startup-ramp-10")) {
        startup_ramp=true;ramp_steps=40;duration_s=105.;--argc;
    }
    else if(argc>1 && !strcmp(argv[argc-1],"--startup-ramp")) {
        startup_ramp=true;duration_s=89.;--argc;
    }
    else if(argc>1 && !strcmp(argv[argc-1],"--small-commands")) {
        small_commands=true;duration_s=27.;--argc;
    }
    if(argc==2 && !strcmp(argv[1],"plan")) {
        puts("t_s,phase,pwm_sx_requested_percent,pwm_dx_requested_percent");
        for(unsigned i=0;i<=static_cast<unsigned>(duration_s*100);++i) {
            unsigned phase;const double pwm=protocol_pwm(i*sample_dt,&phase);
            printf("%.3f,%u,%.3f,%.3f\n",i*sample_dt,phase,
                (startup_ramp || branch_scan) && ramp_wheel!=0 ? 0 : pwm,
                (startup_ramp || branch_scan) && ramp_wheel!=1 ? 0 : pwm);
        }
        return 0;
    }
    if(argc!=4 || strcmp(argv[1],"run") || strcmp(argv[2],"--wheels-lifted")) {
        fprintf(stderr,"Uso: %s plan [--small-commands|--startup-ramp|--startup-ramp-10|--static-characteristic|--holding-curve|--startup-low|--repeat-low] | run --wheels-lifted output.csv [--small-commands|--startup-ramp|--startup-ramp-10|--static-characteristic|--holding-curve|--startup-low|--repeat-low]\n",argv[0]);
        return 1;
    }
    if(inverse_mode || inverse_identify || regime_comparison || candidate_step_comparison) {
        struct sched_param priority;
        memset(&priority,0,sizeof(priority));priority.sched_priority=10;
        // Same priority as real controllers. Failure must not arm this test.
        if(sched_setscheduler(0,SCHED_FIFO,&priority)!=0) {
            perror("sched_setscheduler SCHED_FIFO (nessun accesso hardware)");return 1;
        }
        fprintf(stderr,"scheduling_policy=SCHED_FIFO scheduling_priority=10\n");
    }
    fprintf(stderr,"protocol_id=%s wheel_convention=%s\n"
        "Telaio da immobilizzare: due ruote sollevate e libere. Nessun bilanciamento.\n"
        "duration_s=%.0f sample_hz=100\n",
        candidate_step_comparison ? candidate_protocol_id : regime_comparison ? "freewheel_regimes_v1" : inverse_identify ? "freewheel_inverse_identify_v1" : inverse_cold ? "freewheel_inverse_cold_v1" : inverse_mode ? "freewheel_inverse_v1" : branch_scan ? "freewheel_branch_start_v1" : assist_comparison ? "freewheel_startup_assist_v1" : (repeat_low ? "freewheel_repeat_low_v1" : (startup_low ? "freewheel_startup_low_v1" : (holding_curve ? "freewheel_holding_v1" : (static_characteristic ? "freewheel_static_v1" : (startup_ramp ? (ramp_steps==40 ? "freewheel_startup_ramp10_v1" : "freewheel_startup_ramp_v1") : (small_commands ? "freewheel_small_v1" : "freewheel_v1")))))),robot_hw::wheel_convention(),duration_s);
    if(candidate_step_comparison) {
#ifdef BASELINE_RECHECK_STEPS
#ifdef BASELINE_REVERSE_STEPS
        fprintf(stderr,"levels=-3,-5,-10,-15,-20 pulse_s=2 pause_s=3 baseline_only=yes compensation=OFF cap=20 direction=reverse_both\n");
#else
        fprintf(stderr,"levels=3,5,10,15,20 pulse_s=2 pause_s=3 baseline_only=yes compensation=OFF cap=20 direction=forward_both\n");
#endif
#else
#ifdef LOW_COMMAND_CANDIDATE_STEPS
        fprintf(stderr,"levels=3,5,10,15,20 pulse_s=2 pause_s=3 block_s=26 baseline_then_corrected static_inverse_gain=.2044017950318064 transition=8..10 identity_from=10 startup=one_shot_150ms_release_3counts floors=6.5,8.3 cap=20 differential_cap=3 direction=forward_both\n");
#else
        fprintf(stderr,"levels=3,10,15,20 pulse_s=2 pause_s=3 block_s=21 baseline_then_V2 startup_enabled=no cap=20 differential_cap=3 direction=forward_both\n");
#endif
    #endif // BASELINE_RECHECK_STEPS
    }
    if(regime_comparison)fprintf(stderr,"blocks=sx_forward,sx_reverse,dx_forward,dx_reverse low_sx_percent=4 low_dx_percent=5.5 cold_s=2 off_s=2 warmup_s=1 warmup_percent=10 hold_s=2 reversal_s=2 compensation=OFF\n");
    else if(inverse_identify)fprintf(stderr,"selected_wheel=%s selected_sign=%d descents=2 levels_percent=10,9,8,7,6,5.5,5,4.5,4,3.5,3,2.5,2 plateau_s=3 pause_s=3 cap_percent=10\n",ramp_wheel==0 ? "sx" : "dx",branch_sign);
    else if(inverse_cold)fprintf(stderr,"selected_wheel=%s selected_sign=%d warmup_s=0 measurement_s=3 off_s=3 initial_off_s=1 cap_percent=10\n",ramp_wheel==0 ? "sx" : "dx",branch_sign);
    else if(inverse_mode)fprintf(stderr,"selected_wheel=%s selected_sign=%d warmup_s=2 measurement_s=3 off_s=2 initial_off_s=1 cap_percent=10\n",ramp_wheel==0 ? "sx" : "dx",branch_sign);
    else if(branch_scan)fprintf(stderr,"selected_wheel=%s selected_sign=%d levels_percent=4,5,6,7,8 pulse_s=2 pause_s=2 compensation=OFF\n",ramp_wheel==0 ? "sx" : "dx",branch_sign);
    if(assist_comparison)fprintf(stderr,"assist_floors_sx_forward_reverse=5,5 dx_forward_reverse=6,6 persistence_s=.03 max_boost_s=.20 directed_release_counts=3 paired_phases=odd_OFF_even_ON encoder_sx_sign=-1 encoder_dx_sign=1\n");
    const int output_fd=open(argv[3],O_WRONLY|O_CREAT|O_EXCL,0600);
    if(output_fd<0) {perror("output CSV (non sovrascrivere)");return 1;}
    FILE *output=fdopen(output_fd,"w");
    if(!output) {close(output_fd);return 1;}
    const int lock_fd=open("/tmp/terasic_balance_lqr.lock",O_RDWR|O_CREAT,0600);
    if(lock_fd<0 || flock(lock_fd,LOCK_EX|LOCK_NB)!=0 || hardware_process_conflict()) {
        fprintf(stderr,"Hardware occupato/non disponibile.\n");fclose(output);
        if(lock_fd>=0)close(lock_fd);
        return 1;
    }
    const int fd=open("/dev/mem",O_RDWR|O_SYNC);
    if(fd<0){perror("/dev/mem");fclose(output);close(lock_fd);return 1;}
    const size_t span=0x1000;
    void *base=mmap(NULL,span,PROT_READ|PROT_WRITE,MAP_SHARED,fd,
                    static_cast<off_t>(0xFF200000ULL));
    if(base==MAP_FAILED){perror("mmap");fclose(output);close(fd);close(lock_fd);return 1;}
    signal(SIGINT,request_stop);signal(SIGTERM,request_stop);
    signal(SIGHUP,request_stop);signal(SIGALRM,request_stop);
    regs=static_cast<volatile uint32_t *>(base);
    bool ok=!stop_requested;
    if((rd(robot_hw::PWM_SX_CONTROL)&1) || (rd(robot_hw::PWM_DX_CONTROL)&1)) {
        fprintf(stderr,"RUN gia' attivo: arresto e annullo.\n");ok=false;
    }
    stop_motors();
    wr(robot_hw::PWM_SX_PERIOD,pwm_period);wr(robot_hw::PWM_DX_PERIOD,pwm_period);
    wr(robot_hw::ENCODER_SX_ENABLE,0);wr(robot_hw::ENCODER_DX_ENABLE,0);
    wr(robot_hw::ENCODER_SX_LOAD,32768);wr(robot_hw::ENCODER_DX_LOAD,32768);
    wr(robot_hw::ENCODER_SX_ENABLE,1);wr(robot_hw::ENCODER_DX_ENABLE,1);
    static FreewheelSample samples[11000];unsigned count=0;
    BatteryAcquisition battery;
    StartupAssist assistance[2];
    CandidateStepMapper candidate;
    const double started=now_seconds();double next=started;
    if(!watchdog_set(100000))ok=false;
    const char *reason="none";unsigned last_progress=999;
    while(ok && !stop_requested && count<11000) {
        const double now=now_seconds();
        if(now<next){usleep(200);continue;}
        const double t=now-started;
        // Never catch up missed ticks by emitting a burst of duplicate samples.
        if(now-next>.005){reason="sample_overrun";ok=false;break;}
        if(!battery.update(t)){reason="battery_invalid_timeout_or_outside_10.5_13V";ok=false;break;}
        unsigned phase;const double pwm=protocol_pwm(t,&phase);
        if(pwm && (battery.measured<0 || t-battery.measured>.15)) {
            reason="battery_stale";ok=false;break;
        }
        if((candidate_step_comparison || branch_scan || assist_comparison || repeat_low || holding_curve || static_characteristic || (startup_ramp && phase && (phase-1)%ramp_steps==0)) && phase &&
           (count==0 || samples[count-1].hw.phase==0)) {
            if(count<50) {reason="insufficient_stationary_baseline";ok=false;break;}
            unsigned lo_sx=65535,hi_sx=0,lo_dx=65535,hi_dx=0;
            for(unsigned j=count-50;j<count;++j) {
                const PulseSample &h=samples[j].hw;
                if(h.sx<lo_sx)lo_sx=h.sx;
                if(h.sx>hi_sx)hi_sx=h.sx;
                if(h.dx<lo_dx)lo_dx=h.dx;
                if(h.dx>hi_dx)hi_dx=h.dx;
            }
            if(hi_sx-lo_sx>1 || hi_dx-lo_dx>1) {
                reason="wheels_not_stationary_before_pulse_or_ramp";ok=false;break;
            }
        }
        CandidateStepMapper::Output mapped={{0,0},{0,0},{0,0},{0,0},{0,0},1.,{0,0},{0,0},false};
        if(candidate_step_comparison) {
            const double nominal[]={pwm,pwm};
            const unsigned counters[]={rd(robot_hw::ENCODER_SX_VALUE)&0xffffu,rd(robot_hw::ENCODER_DX_VALUE)&0xffffu};
            mapped=candidate.update(nominal,counters,t,20.,3.);
            if(!mapped.valid){reason="candidate_invalid_encoder_or_timing";ok=false;break;}
            const bool enabled=candidate_actuation_enabled && t>=candidate_block_seconds;
            assist_apply(enabled ? mapped.applied[0]:pwm,enabled ? mapped.applied[1]:pwm,20.);
        } else if(assist_comparison) {
            const bool enabled=phase && phase%2==0;
            const double sx=assistance[0].update(pwm,t,rd(robot_hw::ENCODER_SX_VALUE)&0xffffu,-1,0,enabled);
            const double dx=assistance[1].update(pwm,t,rd(robot_hw::ENCODER_DX_VALUE)&0xffffu,1,1,enabled);
            assist_apply(sx,dx);
        } else freewheel_apply(pwm);
        FreewheelSample &s=samples[count++];
        // Timestamp after write: replay treats this as the observed command event.
        s.hw=pulse_sample(now_seconds()-started,phase);s.requested=pwm;
        s.battery=battery.measured<0 ? 0 : battery.voltage;
        s.battery_raw=battery.raw;s.age=battery.measured<0 ? -1 : t-battery.measured;
        if(candidate_step_comparison) {
            s.candidate_on=candidate_actuation_enabled && t>=candidate_block_seconds;
            for(unsigned w=0;w<2;++w) {
                s.candidate_target[w]=s.candidate_on ? mapped.target[w]:pwm;
                s.candidate_omega[w]=mapped.omega[w];s.candidate_regime[w]=mapped.regime[w];
                s.candidate_support[w]=mapped.support[w];
            }
        }
        if((candidate_step_comparison || regime_comparison || inverse_identify || inverse_mode || repeat_low || holding_curve || static_characteristic) && battery.measured>=0 && static_cast<unsigned>(t)!=last_progress) {
            last_progress=static_cast<unsigned>(t);
            fprintf(stderr,"progress t_s=%.2f phase=%u pwm=%.1f battery_v=%.3f\n",t,phase,pwm,battery.voltage);
            fflush(stderr);
        }
        if(!safe_counters(s.hw)){reason="encoder_near_limit";ok=false;break;}
        if(t>=duration_s)break;
        if(!watchdog_set(100000)){reason="watchdog_setup";ok=false;break;}
        next=started+count*sample_dt;
    }
    stop_motors();watchdog_set(0);
    wr(robot_hw::ENCODER_SX_ENABLE,0);wr(robot_hw::ENCODER_DX_ENABLE,0);
    const bool completed=ok && !stop_requested && count && samples[count-1].hw.time>=duration_s;
    fprintf(output,"t_s,phase,pwm_sx_requested_percent,pwm_dx_requested_percent,"
        "enc_sx_raw,enc_dx_raw,pwm_sx_applied_percent,pwm_dx_applied_percent,"
        "period_sx,period_dx,high_sx,high_dx,control_sx,control_dx,"
        "battery_raw,battery_v,battery_age_s");
    if(candidate_step_comparison)fprintf(output,",candidate_on,target_sx_percent,target_dx_percent,motion_sx_rad_s,motion_dx_rad_s,regime_sx,regime_dx,support_sx,support_dx");
    if(inverse_mode)fprintf(output,",sequence,lut_enabled,virtual_command,target_speed_rad_s,stage");
    if(regime_comparison)fprintf(output,",test_block,test_wheel,regime");
    fprintf(output,"\n");
    for(unsigned i=0;i<count;++i) {
        const FreewheelSample &s=samples[i];const PulseSample &h=s.hw;
        fprintf(output,"%.9f,%u,%.9f,%.9f,%u,%u,%.6f,%.6f,%u,%u,%u,%u,%u,%u,%u,%.6f,%.6f",
            h.time,h.phase,
            (regime_comparison ? h.phase>8 : branch_scan ? ramp_wheel!=0 : startup_ramp && h.phase>2*ramp_steps) ? 0 : s.requested,
            (regime_comparison ? h.phase<=8 : branch_scan ? ramp_wheel!=1 : startup_ramp && h.phase<=2*ramp_steps) ? 0 : s.requested,h.sx,h.dx,
            sampled_duty(h.high_sx,h.period_sx,h.control_sx),sampled_duty(h.high_dx,h.period_dx,h.control_dx),
            h.period_sx,h.period_dx,h.high_sx,h.high_dx,h.control_sx,h.control_dx,
            s.battery_raw,s.battery,s.age);
        if(candidate_step_comparison)fprintf(output,",%u,%.9f,%.9f,%.9f,%.9f,%d,%d,%d,%d",s.candidate_on ? 1:0,
            s.candidate_target[0],s.candidate_target[1],s.candidate_omega[0],s.candidate_omega[1],
            s.candidate_regime[0],s.candidate_regime[1],s.candidate_support[0],s.candidate_support[1]);
        if(inverse_mode) {
            if(h.phase) {
                const unsigned j=(h.phase-1)/2;const InverseStep &step=inverse_steps[j];
                fprintf(output,",%u,%u,%.9f,%.9f,%s",j+1,step.on,step.command,step.target,
                        inverse_cold ? "cold_start" : h.phase%2 ? "warmup" : "measurement");
            } else fprintf(output,",0,0,0,0,off");
        }
        if(regime_comparison) {
            if(h.phase) {
                const char *names[]={"cold_start","warmup","holding","reversal"};
                fprintf(output,",%u,%s,%s",1+(h.phase-1)/4,h.phase<=8 ? "sx" : "dx",names[(h.phase-1)%4]);
            } else fprintf(output,",0,none,off");
        }
        fprintf(output,"\n");
    }
    const bool saved=fflush(output)==0 && !ferror(output);
    const bool closed=fclose(output)==0;
    fprintf(stderr,"samples=%u final_control_sx=%u final_control_dx=%u reason=%s watchdog=%d\n"
        "result=%s\n",count,rd(robot_hw::PWM_SX_CONTROL),rd(robot_hw::PWM_DX_CONTROL),reason,
        static_cast<int>(watchdog_expired),completed && saved && closed ? "COMPLETED" : "ABORTED");
    regs=NULL;munmap(base,span);close(fd);close(lock_fd);
    return completed && saved && closed ? 0 : 1;
}
