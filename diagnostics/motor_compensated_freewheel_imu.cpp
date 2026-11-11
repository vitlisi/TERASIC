// Isolated raised-wheel acquisition using the qualified bounded compensator.
// No balancing controller; both wheels receive the same nominal PWM profile.
#define main compensated_freewheel_unused_main
#include "motor_freewheel.cpp"
#undef main
#include "stribeck_lut_compensation.h"
#include <vector>
#include "freewheel_imu.h"

static double compensated_nominal(unsigned sample) {
    const double values[]={-4,0,-6,0,-8,0,4,0,6,0,8,0};
    if(sample<100 || sample>=2500)return 0.;
    return values[(sample-100)/200];
}

// The compensator already returns exact hardware PWM quanta. Encode those
// quanta without a second truncation, preserving its physical +/-0.5% bound.
static bool apply_compensated_pair(const double value[2]) {
    sigset_t set,old;sigemptyset(&set);
    sigaddset(&set,SIGINT);sigaddset(&set,SIGTERM);sigaddset(&set,SIGHUP);sigaddset(&set,SIGALRM);
    sigprocmask(SIG_BLOCK,&set,&old);
    unsigned high[2]={0,0};bool good=!stop_requested;
    for(unsigned w=0;w<2;++w) {
        if(!lqr_finite(value[w]) || std::fabs(value[w])>8.000001 || (value[w] && std::fabs(value[w])<.5))good=false;
        if(good) {
            const double quantized=std::floor(500.*std::fabs(value[w])+.5);
            if(std::fabs(500.*std::fabs(value[w])-quantized)>1e-6)good=false;
            else high[w]=static_cast<unsigned>(quantized);
        }
    }
    if(std::fabs(value[0]-value[1])>6.000001)good=false;
    if(!good) {stop_requested=1;stop_motors();}
    else {
        const unsigned controls[]={robot_hw::PWM_SX_CONTROL,robot_hw::PWM_DX_CONTROL};
        for(unsigned w=0;w<2;++w)if(value[w] && (rd(controls[w])&2u)!=(value[w]>0 ? 2u:0u))stop_motors();
        wr(robot_hw::PWM_SX_HIGH,high[0]);wr(robot_hw::PWM_DX_HIGH,high[1]);
        wr(robot_hw::PWM_SX_CONTROL,value[0] ? (value[0]>0 ? 7:5):6);
        wr(robot_hw::PWM_DX_CONTROL,value[1] ? (value[1]>0 ? 7:5):6);
    }
    sigprocmask(SIG_SETMASK,&old,NULL);return good;
}

struct CompensatedSample {
    Sample imu;double imu_time;PulseSample hw;double nominal,voltage,age,applied[2],target[2],correction[2],omega[2],time;
    unsigned raw;bool armed;
};

#ifndef COMPENSATED_FREEWHEEL_MAIN
#define COMPENSATED_FREEWHEEL_MAIN main
#endif
int COMPENSATED_FREEWHEEL_MAIN(int argc,char **argv) {
    const bool plan=argc==2 && !strcmp(argv[1],"plan");
    const bool check=argc==2 && !strcmp(argv[1],"check");
    const bool shadow=argc==5 && !strcmp(argv[1],"shadow") && !strcmp(argv[2],"--wheels-lifted") && !strcmp(argv[3],"--chassis-fixed");
    const bool armed=argc==5 && !strcmp(argv[1],"run") && !strcmp(argv[2],"--wheels-lifted") && !strcmp(argv[3],"--chassis-fixed");
    if(plan) {
        puts("t_s,nominal_sx_percent,nominal_dx_percent");
        for(unsigned i=0;i<=2600;++i)printf("%.2f,%.3f,%.3f\n",i*.01,compensated_nominal(i),compensated_nominal(i));
        return 0;
    }
    if(check) {printf("CONFIG_OK robot01_fixed_chassis compensation=bounded_stribeck cap8 sample_hz100 duration26 LUT_sha256=%s NO_HARDWARE_ACCESS\n",StribeckLutCompensation::lut_hash());return 0;}
    if(!shadow && !armed) {
        fprintf(stderr,"Usage: %s plan|check\n       %s shadow|run --wheels-lifted --chassis-fixed NEW.csv\n",argv[0],argv[0]);return 2;
    }
    struct sched_param priority;memset(&priority,0,sizeof(priority));priority.sched_priority=10;
    if(sched_setscheduler(0,SCHED_FIFO,&priority)!=0){perror("SCHED_FIFO (no motor access)");return 1;}
    const int output_fd=open(argv[4],O_WRONLY|O_CREAT|O_EXCL,0600);
    if(output_fd<0){perror("New CSV only");return 1;}
    FILE *output=fdopen(output_fd,"w");if(!output){close(output_fd);return 1;}
    const int lock_fd=open("/tmp/terasic_balance_lqr.lock",O_RDWR|O_CREAT,0600);
    if(lock_fd<0 || flock(lock_fd,LOCK_EX|LOCK_NB)!=0 || hardware_process_conflict()) {
        fprintf(stderr,"Hardware busy; no motor access\n");fclose(output);if(lock_fd>=0)close(lock_fd);return 1;
    }
    const int fd=open("/dev/mem",O_RDWR|O_SYNC);
    if(fd<0){perror("/dev/mem");fclose(output);close(lock_fd);return 1;}
    void *base=mmap(NULL,0x1000,PROT_READ|PROT_WRITE,MAP_SHARED,fd,static_cast<off_t>(0xFF200000ULL));
    if(base==MAP_FAILED){perror("mmap");fclose(output);close(fd);close(lock_fd);return 1;}
    regs=static_cast<volatile uint32_t *>(base);stop_requested=0;watchdog_expired=0;
    signal(SIGINT,request_stop);signal(SIGTERM,request_stop);signal(SIGHUP,request_stop);signal(SIGALRM,request_stop);
    const bool already=(rd(robot_hw::PWM_SX_CONTROL)&1u) || (rd(robot_hw::PWM_DX_CONTROL)&1u);
    stop_motors();bool ok=!already;const char *reason=already ? "already_running":"none";
    wr(robot_hw::PWM_SX_PERIOD,50000);wr(robot_hw::PWM_DX_PERIOD,50000);
    wr(robot_hw::ENCODER_SX_ENABLE,0);wr(robot_hw::ENCODER_DX_ENABLE,0);
    wr(robot_hw::ENCODER_SX_LOAD,32768);wr(robot_hw::ENCODER_DX_LOAD,32768);
    wr(robot_hw::ENCODER_SX_ENABLE,1);wr(robot_hw::ENCODER_DX_ENABLE,1);
    if(ok && !imu_setup()){ok=false;reason="IMU_setup_failed";}
    BatteryAcquisition battery;const double adc_started=now_seconds();
    while(ok && !stop_requested && battery.measured<0) {
        const double t=now_seconds()-adc_started;
        if(!battery.update(t) || t>.15){ok=false;reason="initial_ADC_missing_or_invalid";break;}
        if(battery.measured<0)usleep(1000);
    }
    // BatteryAcquisition timestamps must remain on its own monotonic epoch.
    const double started=now_seconds();const unsigned last=armed ? 2600:300;
    StribeckLutCompensation comp;std::vector<CompensatedSample> samples;samples.reserve(last+1);
    if(!watchdog_set(50000)){ok=false;reason="watchdog_setup";}
    fprintf(stderr,"protocol_id=robot01_compensated_freewheel_imu_v1 chassis=fixed wheels=both raised=yes armed=%d duration_s=%u sample_hz=100 cap_pwm=8 correction_cap=.5 live_adc=11.20..12.10 watchdog_us=50000 LUT_sha256=%s\n",armed?1:0,armed?26:3,StribeckLutCompensation::lut_hash());
    for(unsigned i=0;ok && !stop_requested && i<=last;++i) {
        const double due=started+i*.01;double now=now_seconds();
        while(now<due && !stop_requested){usleep(150);now=now_seconds();}
        if(stop_requested)break;
        if(now-due>.005){ok=false;reason="sample_overrun";break;}
        if(!battery.update(now-adc_started)){ok=false;reason="battery_invalid";break;}
        now=now_seconds();
        const double age=now-adc_started-battery.measured;
        if(!lqr_finite(battery.voltage) || battery.voltage<11.2 || battery.voltage>12.1 || age<0 || age>.15) {
            ok=false;reason="battery_outside_11.20_12.10_or_stale";break;
        }
        Sample imu;
        if(!get_sample(&imu)){ok=false;reason="IMU_read_failed";break;}
        now=now_seconds();
        if(now-due>.008){ok=false;reason="IMU_sample_overrun";break;}
        const double t=now-started;const double nominal=compensated_nominal(i);
        const PulseSample before=pulse_sample(t,i);
        if(!safe_counters(before)){ok=false;reason="encoder_near_limit";break;}
        if(i==100) {
            if(samples.size()<50){ok=false;reason="missing_quiet_baseline";break;}
            unsigned lo[2]={65535,65535},hi[2]={0,0};
            for(unsigned j=50;j<100;++j) {
                const unsigned enc[]={samples[j].hw.sx,samples[j].hw.dx};
                for(unsigned w=0;w<2;++w){lo[w]=std::min(lo[w],enc[w]);hi[w]=std::max(hi[w],enc[w]);}
            }
            if(hi[0]-lo[0]>1 || hi[1]-lo[1]>1){ok=false;reason="not_stationary_before_pulses";break;}
        }
        const double input[]={nominal,nominal};const unsigned enc[]={before.sx,before.dx};comp.set_voltage(battery.voltage);
        const StribeckLutCompensation::Output result=comp.update(input,enc,t);
        if(!result.valid){ok=false;reason="invalid_compensator_input";break;}
        const double physical[]={armed ? result.applied[0]:0.,armed ? result.applied[1]:0.};
        if(!apply_compensated_pair(physical)){ok=false;reason="physical_PWM_invalid";break;}
        PulseSample after=pulse_sample(t,i);after.sx=before.sx;after.dx=before.dx;
        const double actual[]={sampled_duty(after.high_sx,after.period_sx,after.control_sx),sampled_duty(after.high_dx,after.period_dx,after.control_dx)};
        if(std::fabs(actual[0]-physical[0])>.000001 || std::fabs(actual[1]-physical[1])>.000001) {
            ok=false;reason="PWM_readback_mismatch";break;
        }
        CompensatedSample sample;sample.imu=imu;sample.imu_time=t;sample.hw=after;sample.nominal=nominal;sample.voltage=battery.voltage;sample.raw=battery.raw;sample.age=age;sample.armed=armed;sample.time=t;
        for(unsigned w=0;w<2;++w){sample.applied[w]=actual[w];sample.target[w]=result.target[w];sample.correction[w]=result.correction[w];sample.omega[w]=result.omega[w];}
        samples.push_back(sample);
        if(i%100==0)fprintf(stderr,"progress t=%.3f nominal=%+.1f PWM_sx=%+.3f PWM_dx=%+.3f V=%.3f\n",t,nominal,actual[0],actual[1],battery.voltage);
        if(i<last && !watchdog_set(50000)){ok=false;reason="watchdog_setup";break;}
    }
    stop_motors();watchdog_set(0);i2c_write(2,0);wr(robot_hw::ENCODER_SX_ENABLE,0);wr(robot_hw::ENCODER_DX_ENABLE,0);
    const unsigned final_sx=rd(robot_hw::PWM_SX_CONTROL),final_dx=rd(robot_hw::PWM_DX_CONTROL);
    const bool complete=ok && !stop_requested && samples.size()==last+1;
    fprintf(output,"t_s,sample,armed,nominal_sx_percent,nominal_dx_percent,pwm_sx_applied_percent,pwm_dx_applied_percent,enc_sx_raw,enc_dx_raw,battery_v,battery_raw,battery_age_s,target_sx_percent,target_dx_percent,correction_sx_percent,correction_dx_percent,filtered_speed_sx_rad_s,filtered_speed_dx_rad_s,imu_t_s,gyro_pitch_deg_s,gyro_x_deg_s,gyro_z_deg_s,pitch_acc_deg,accel_g\n");
    for(unsigned i=0;i<samples.size();++i) {
        const CompensatedSample &s=samples[i];
        fprintf(output,"%.9f,%u,%u,%.9f,%.9f,%.9f,%.9f,%u,%u,%.9f,%u,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",s.time,i,s.armed?1:0,s.nominal,s.nominal,s.applied[0],s.applied[1],s.hw.sx,s.hw.dx,s.voltage,s.raw,s.age,s.target[0],s.target[1],s.correction[0],s.correction[1],s.omega[0],s.omega[1],s.imu_time,s.imu.gyro_deg_s,s.imu.gx/131.,s.imu.gz/131.,s.imu.angle_deg,s.imu.accel_g);
    }
    const bool saved=fflush(output)==0 && !ferror(output);const bool closed=fclose(output)==0;
    fprintf(stderr,"samples=%u final_control_sx=%u final_control_dx=%u reason=%s watchdog=%d result=%s\n",static_cast<unsigned>(samples.size()),final_sx,final_dx,reason,watchdog_expired,complete&&saved&&closed ? "COMPLETED":"ABORTED");
    regs=NULL;munmap(base,0x1000);close(fd);
    close(lock_fd);return complete&&saved&&closed ? 0:1;
}
