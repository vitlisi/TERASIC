// Independent compensated moving-point acquisition. Historical raw driver unchanged.
// Reuse the verified MMIO map, signal stop, lock, ADC and channel driver helpers.
#define main static_point_legacy_freewheel_main
#include "motor_freewheel.cpp"
#undef main
#include <vector>
#include <cerrno>
#include "green_lut_frozen.h"

struct StaticPointConfig {
    unsigned wheel;
    int sign;
    bool moving,cycle;
    StaticPointConfig():wheel(0),sign(1),moving(false),cycle(false),magnitude(0),hold(3),virtual_magnitude(0) {}
    double magnitude,hold,virtual_magnitude;
    double warmup() const { return magnitude>0 ? 3. : 0.; }
    double duration() const { return cycle ? 1.+32.*hold+3. : 1.+warmup()+hold+3.; }
};
static bool point_number(const char *s,double *out) {
    char *end=0;errno=0;*out=strtod(s,&end);
    return !errno && end!=s && !*end && lqr_finite(*out);
}
static bool point_config(char **a,StaticPointConfig *c) {
    if(strcmp(a[0],"sx") && strcmp(a[0],"dx"))return false;
    if(strcmp(a[1],"forward") && strcmp(a[1],"reverse"))return false;
    if(strcmp(a[2],"moving"))return false;
    c->wheel=!strcmp(a[0],"dx");c->sign=!strcmp(a[1],"forward") ? 1:-1;
    c->moving=true;c->cycle=false;
    double mapped=0;
    if(!point_number(a[3],&c->virtual_magnitude) ||
       !green_map(c->wheel,c->sign,c->virtual_magnitude,&mapped) ||
       !point_number(a[4],&c->hold) || c->hold<3 || c->hold>10)return false;
    c->magnitude=std::fabs(mapped);return true;
}
static unsigned point_stage(double t,const StaticPointConfig &c) {
    if(t<1.)return 0;
    if(c.cycle)return t<1.+32.*c.hold ? 2:3;
    if(t<1.+c.warmup())return 1;
    if(t<1.+c.warmup()+c.hold)return 2;
    return 3;
}
static double point_pwm(double t,const StaticPointConfig &c) {
    const unsigned stage=point_stage(t,c);
    if(c.cycle) {
        if(stage!=2)return 0.;
        const unsigned segment=static_cast<unsigned>((t-1.)/c.hold);
        return c.sign*.5*(segment<16 ? segment+1:31-segment);
    }
    return stage==1 ? c.sign*8. : stage==2 ? c.sign*c.magnitude:0.;
}
struct StaticPointSample { PulseSample hw;double requested,voltage,age;unsigned raw,stage; };

static bool green_motion_verified(const std::vector<StaticPointSample> &samples,const StaticPointConfig &c) {
    if(samples.size()<50)return false;
    const unsigned first=static_cast<unsigned>(samples.size())-50;
    const double a=c.wheel ? samples[first].hw.dx:samples[first].hw.sx;
    const double b=c.wheel ? samples.back().hw.dx:samples.back().hw.sx;
    return (b-a)*(c.wheel ? 1:-1)*c.sign>=6;
}

// Live operating cutoff requested for future trials. It is checked at every
// valid ADC update (about 100 ms), before the next PWM application. A detected
// violation latches stop_requested; it is not merely a post-trial quality flag.
static bool point_apply_voltage_checked(double u,const BatteryAcquisition &battery) {
    const bool invalid=(u && battery.measured<0) ||
        (battery.measured>=0 && (!lqr_finite(battery.voltage) || battery.voltage<11.20 || battery.voltage>13.));
    if(invalid) { fprintf(stderr,"LIVE_VOLTAGE_STOP V=%.9f raw=%u measured=%.9f\n",battery.voltage,battery.raw,battery.measured);stop_requested=1;stop_motors();return false; }
    freewheel_apply(u);return true;
}

#ifndef STATIC_POINT_MAIN_NAME
#define STATIC_POINT_MAIN_NAME main
#endif
int STATIC_POINT_MAIN_NAME(int argc,char **argv) {
    const bool plan=argc==7 && !strcmp(argv[1],"plan");
    const bool run=argc==9 && !strcmp(argv[1],"run") && !strcmp(argv[2],"--wheels-lifted");
    StaticPointConfig c;
    if((!plan && !run) || !point_config(argv+(plan ? 2:3),&c)) {
        fprintf(stderr,"Usage: %s plan sx|dx forward|reverse moving virtual_0_or_supported_domain..20 hold_s_3..10\n"
            "       %s run --wheels-lifted sx|dx forward|reverse moving virtual_command hold_s NEW.csv\n",argv[0],argv[0]);return 2;
    }
    if(plan) {
        puts("t_s,stage,pwm_sx_requested_percent,pwm_dx_requested_percent");
        const unsigned n=static_cast<unsigned>(c.duration()*100.+.5);
        for(unsigned i=0;i<=n;++i) {
            const double t=i*.01,u=point_pwm(t,c);
            printf("%.2f,%u,%.9f,%.9f\n",t,point_stage(t,c),c.wheel==0 ? u:0.,c.wheel==1 ? u:0.);
        }
        return 0;
    }
    struct sched_param priority;memset(&priority,0,sizeof(priority));priority.sched_priority=10;
    if(sched_setscheduler(0,SCHED_FIFO,&priority)!=0){perror("SCHED_FIFO (no motor access)");return 1;}
    const int output_fd=open(argv[8],O_WRONLY|O_CREAT|O_EXCL,0600);
    if(output_fd<0){perror("New CSV only");return 1;}
    FILE *output=fdopen(output_fd,"w");if(!output){close(output_fd);return 1;}
    const int lock_fd=open("/tmp/terasic_balance_lqr.lock",O_RDWR|O_CREAT,0600);
    if(lock_fd<0 || flock(lock_fd,LOCK_EX|LOCK_NB)!=0 || hardware_process_conflict()) {
        fprintf(stderr,"Hardware busy\n");fclose(output);if(lock_fd>=0)close(lock_fd);return 1;
    }
    const int fd=open("/dev/mem",O_RDWR|O_SYNC);
    if(fd<0){perror("/dev/mem");fclose(output);close(lock_fd);return 1;}
    void *base=mmap(NULL,0x1000,PROT_READ|PROT_WRITE,MAP_SHARED,fd,static_cast<off_t>(0xFF200000ULL));
    if(base==MAP_FAILED){perror("mmap");fclose(output);close(fd);close(lock_fd);return 1;}
    regs=static_cast<volatile uint32_t *>(base);stop_requested=0;watchdog_expired=0;
    signal(SIGINT,request_stop);signal(SIGTERM,request_stop);signal(SIGHUP,request_stop);signal(SIGALRM,request_stop);
    const bool was_running=(rd(robot_hw::PWM_SX_CONTROL)&1u) || (rd(robot_hw::PWM_DX_CONTROL)&1u);
    stop_motors();
    bool ok=!was_running;const char *reason=was_running ? "already_running":"none";
    wr(robot_hw::PWM_SX_PERIOD,50000);wr(robot_hw::PWM_DX_PERIOD,50000);
    // One short point per process: counters reset once, with both motors OFF.
    wr(robot_hw::ENCODER_SX_ENABLE,0);wr(robot_hw::ENCODER_DX_ENABLE,0);
    wr(robot_hw::ENCODER_SX_LOAD,32768);wr(robot_hw::ENCODER_DX_LOAD,32768);
    wr(robot_hw::ENCODER_SX_ENABLE,1);wr(robot_hw::ENCODER_DX_ENABLE,1);
    branch_scan=true;ramp_wheel=c.wheel;startup_ramp=false;
    std::vector<StaticPointSample> samples;samples.reserve(c.cycle ? 10005:1705);
    BatteryAcquisition battery;const double started=now_seconds();double next=started;unsigned progress=999;
    if(!watchdog_set(100000)){ok=false;reason="watchdog_setup";}
    fprintf(stderr,"protocol_id=%s wheel=%s direction=%s mode=%s pwm=%.1f hold_s=%.2f "
        "warmup_pwm=%d warmup_s=%.1f duration_s=%.2f compensation=FROZEN_MOVING_LUT period=50000 sample_hz=100 live_adc_min_v=11.20\n",
        c.cycle ? "robot01_hysteresis_cycle_v1_guarded":"robot01_green_moving_v1_guarded",
        c.wheel ? "dx":"sx",c.sign>0 ? "forward":"reverse",c.cycle ? "cycle":c.moving ? "moving":"cold",c.sign*c.magnitude,
        c.hold,c.magnitude>0 ? c.sign*8:0,c.warmup(),c.duration());
    fprintf(stderr,"lut_sha256=%s virtual_command=%.9f target_velocity_rad_s=%.9f\n",
        green_lut_sha256,c.sign*c.virtual_magnitude,c.sign*c.virtual_magnitude*green_gain);
    while(ok && !stop_requested) {
        const double now=now_seconds();if(now<next){usleep(200);continue;}
        const double t=now-started;
        if(now-next>.005){ok=false;reason="sample_overrun";break;}
        if(!battery.update(t)){ok=false;reason="battery_invalid_timeout_or_outside_10.5_13V";break;}
        const unsigned stage=point_stage(t,c);const double u=point_pwm(t,c);
        if(u && (battery.measured<0 || t-battery.measured>.15)){ok=false;reason="battery_stale";break;}
        // Cold starts and pre-runs both require a quiet initial 0.5 s window.
        if(stage!=0 && !samples.empty() && samples.back().stage==0) {
            if(samples.size()<50){ok=false;reason="insufficient_quiet_baseline";break;}
            unsigned lo[2]={65535,65535},hi[2]={0,0};
            for(unsigned j=static_cast<unsigned>(samples.size())-50;j<samples.size();++j) {
                const unsigned enc[2]={samples[j].hw.sx,samples[j].hw.dx};
                for(unsigned w=0;w<2;++w){if(enc[w]<lo[w])lo[w]=enc[w];if(enc[w]>hi[w])hi[w]=enc[w];}
            }
            if(hi[0]-lo[0]>1 || hi[1]-lo[1]>1){ok=false;reason="not_stationary_before_trial";break;}
        }
        if(stage==2 && !samples.empty() && samples.back().stage==1) {
            if(!green_motion_verified(samples,c)){ok=false;reason="warmup_motion_not_verified";break;}
        }
        if(!point_apply_voltage_checked(u,battery)){ok=false;reason="battery_outside_live_11.20_13.00V_or_missing";break;}
        StaticPointSample s;s.hw=pulse_sample(now_seconds()-started,stage);s.requested=u;s.stage=stage;
        s.raw=battery.raw;s.voltage=battery.measured<0 ? 0.:battery.voltage;s.age=battery.measured<0 ? -1.:t-battery.measured;
        if(!safe_counters(s.hw)){ok=false;reason="encoder_near_limit";break;}
        const double own=c.wheel ? sampled_duty(s.hw.high_dx,s.hw.period_dx,s.hw.control_dx):sampled_duty(s.hw.high_sx,s.hw.period_sx,s.hw.control_sx);
        const unsigned other_high=c.wheel ? s.hw.high_sx:s.hw.high_dx,other_control=c.wheel ? s.hw.control_sx:s.hw.control_dx;
        if(std::fabs(own-u)>.0021 || other_high || (other_control&1u)){ok=false;reason="pwm_readback_mismatch";break;}
        samples.push_back(s);
        if(static_cast<unsigned>(t)!=progress){progress=static_cast<unsigned>(t);fprintf(stderr,"progress t=%.2f stage=%u pwm=%.1f V=%.3f\n",t,stage,u,s.voltage);}
        if(t>=c.duration())break;
        if(samples.size()>=(c.cycle ? 10002u:1702u)){ok=false;reason="sample_buffer_limit";break;}
        if(!watchdog_set(100000)){ok=false;reason="watchdog_setup";break;}
        next=started+samples.size()*.01;
    }
    stop_motors();watchdog_set(0);wr(robot_hw::ENCODER_SX_ENABLE,0);wr(robot_hw::ENCODER_DX_ENABLE,0);
    const unsigned final_sx=rd(robot_hw::PWM_SX_CONTROL),final_dx=rd(robot_hw::PWM_DX_CONTROL);
    const bool completed=ok && !stop_requested && !samples.empty() && samples.back().hw.time>=c.duration();
    fprintf(output,"t_s,stage,pwm_requested_percent,enc_sx_raw,enc_dx_raw,pwm_sx_applied_percent,pwm_dx_applied_percent,"
        "period_sx,period_dx,high_sx,high_dx,control_sx,control_dx,battery_raw,battery_v,battery_age_s,lut_virtual_command,target_velocity_rad_s\n");
    for(unsigned i=0;i<samples.size();++i) {
        const StaticPointSample &s=samples[i];const PulseSample &h=s.hw;
        fprintf(output,"%.9f,%u,%.6f,%u,%u,%.6f,%.6f,%u,%u,%u,%u,%u,%u,%u,%.6f,%.6f,%.9f,%.9f\n",
            h.time,s.stage,s.requested,h.sx,h.dx,sampled_duty(h.high_sx,h.period_sx,h.control_sx),
            sampled_duty(h.high_dx,h.period_dx,h.control_dx),h.period_sx,h.period_dx,h.high_sx,h.high_dx,
            h.control_sx,h.control_dx,s.raw,s.voltage,s.age,s.stage==2 ? c.sign*c.virtual_magnitude:0.,s.stage==2 ? c.sign*c.virtual_magnitude*green_gain:0.);
    }
    const bool saved=fflush(output)==0 && !ferror(output);const bool closed=fclose(output)==0;
    fprintf(stderr,"samples=%u final_control_sx=%u final_control_dx=%u reason=%s watchdog=%d result=%s\n",
        static_cast<unsigned>(samples.size()),final_sx,final_dx,reason,watchdog_expired,completed && saved && closed ? "COMPLETED":"ABORTED");
    regs=0;munmap(base,0x1000);close(fd);close(lock_fd);
    return completed && saved && closed ? 0:1;
}
