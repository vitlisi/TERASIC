#define _FILE_OFFSET_BITS 64
#include "../src_cpp/terasic_reale/hardware/robot_wheel_map.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/file.h>
#include <dirent.h>

static volatile uint32_t *regs;
static volatile sig_atomic_t stop_requested = 0;
static volatile sig_atomic_t watchdog_expired = 0;
static void stop_motors();

static void request_stop(int number)
{
    stop_requested = 1;
    if (number==SIGALRM) watchdog_expired=1;
    if (regs) stop_motors();
}

static bool hardware_process_conflict()
{
    DIR *directory=opendir("/proc");
    if (!directory) { perror("opendir /proc"); return true; }
    const char *names[]={"pitch_live","pitch_pd","pitch_pd_fixed","balance_lqr",
        "robot_test","balance_car","bal_car","DE10_Nano_bal","imu_measure",
        "imu_identity","motor_pulse","motor_response","encoder_manual","battery_status",
        "balance_lqr_2","balance_hinf","hinf_path","hinf_path_yaw","path_simple","lqr_endurance","motor_freewheel"};
    bool conflict=false;
    struct dirent *entry;
    while ((entry=readdir(directory))!=NULL) {
        char *end=NULL;
        const long pid=strtol(entry->d_name,&end,10);
        if (pid<=0 || *end || pid==static_cast<long>(getpid())) continue;
        char path[128],name[64]={0};
        std::snprintf(path,sizeof(path),"/proc/%ld/comm",pid);
        FILE *file=std::fopen(path,"r");
        if (!file) continue;
        const bool read_ok=std::fgets(name,sizeof(name),file)!=NULL;
        std::fclose(file);
        if (!read_ok) continue;
        name[std::strcspn(name,"\r\n")]=0;
        for (unsigned i=0;i<sizeof(names)/sizeof(names[0]);++i) {
            if (!std::strcmp(name,names[i])) {
                fprintf(stderr,"Programma hardware attivo: %s (PID %ld). Avvio rifiutato.\n",name,pid);
                conflict=true; break;
            }
        }
        if (conflict) break;
    }
    closedir(directory);
    return conflict;
}

static bool watchdog_set(unsigned microseconds)
{
    struct itimerval timer;
    std::memset(&timer,0,sizeof(timer));
    timer.it_value.tv_usec=microseconds;
    return setitimer(ITIMER_REAL,&timer,NULL)==0;
}

static uint32_t rd(unsigned offset)
{
    return regs[offset / 4];
}

static void wr(unsigned offset, uint32_t value)
{
    regs[offset / 4] = value;
}

static double now_seconds()
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t)!=0) {
        perror("clock_gettime");
        request_stop(0);
        return 0;
    }
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static void stop_motors()
{
    // FAST_DECAY=1, FORWARD=1, RUN=0.
    wr(robot_hw::PWM_SX_CONTROL, 0x06);
    wr(robot_hw::PWM_DX_CONTROL, 0x06);
    wr(robot_hw::PWM_SX_HIGH, 0);
    wr(robot_hw::PWM_DX_HIGH, 0);
}

struct PulseSample {
    double time;
    unsigned phase,sx,dx,high_sx,high_dx,period_sx,period_dx,control_sx,control_dx;
};

static PulseSample pulse_sample(double time,unsigned phase)
{
    PulseSample s;
    s.time=time;s.phase=phase;
    s.sx=rd(robot_hw::ENCODER_SX_VALUE)&0xffffu;
    s.dx=rd(robot_hw::ENCODER_DX_VALUE)&0xffffu;
    s.high_sx=rd(robot_hw::PWM_SX_HIGH);s.high_dx=rd(robot_hw::PWM_DX_HIGH);
    s.period_sx=rd(robot_hw::PWM_SX_PERIOD);s.period_dx=rd(robot_hw::PWM_DX_PERIOD);
    s.control_sx=rd(robot_hw::PWM_SX_CONTROL);s.control_dx=rd(robot_hw::PWM_DX_CONTROL);
    return s;
}

static double sampled_duty(unsigned high,unsigned period,unsigned control)
{
    return (control&1u) && period ? (control&2u ? 1. : -1.)*100.*high/period : 0.;
}

#ifndef MOTOR_PULSE_HELPERS_ONLY
int main(int argc, char **argv)
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    const bool csv=argc==5 && !strcmp(argv[4],"--csv");
    if ((argc != 4 && !csv) ||
        (strcmp(argv[1], "sx") && strcmp(argv[1], "dx")) ||
        (strcmp(argv[2], "0") && strcmp(argv[2], "1"))) {
        fprintf(stderr, "Uso: ./motor_pulse sx|dx 0|1 percentuale [--csv]\n");
        return 1;
    }

    char *end = NULL;
    long percent = strtol(argv[3], &end, 10);
    if (end == argv[3] || *end || percent < 1 || percent > 20) {
        fprintf(stderr, "Percentuale ammessa: da 1 a 20.\n");
        return 1;
    }

    unsigned motor = !strcmp(argv[1], "sx") ? robot_hw::PWM_SX_PERIOD : robot_hw::PWM_DX_PERIOD;
    unsigned direction = !strcmp(argv[2], "1") ? 1 : 0;
    const uint32_t period = 50000;
    const uint32_t high = period * percent / 100;

    const int lock_fd=open("/tmp/terasic_balance_lqr.lock",O_RDWR|O_CREAT,0600);
    if (lock_fd<0) { perror("lock impulso"); return 1; }
    if (flock(lock_fd,LOCK_EX|LOCK_NB)!=0) { perror("Hardware occupato"); close(lock_fd); return 1; }
    if (hardware_process_conflict()) { close(lock_fd); return 1; }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    const size_t span = 0x1000;
    void *base = mmap(NULL, span, PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd,
                      static_cast<off_t>(0xFF200000ULL));
    if (base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }
    regs = static_cast<volatile uint32_t *>(base);

    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);
    signal(SIGHUP, request_stop);
    signal(SIGALRM, request_stop);

    if ((rd(robot_hw::PWM_SX_CONTROL) & 1) || (rd(robot_hw::PWM_DX_CONTROL) & 1)) {
        fprintf(stderr, "Un canale era gia' attivo: arresto e annullo.\n");
        stop_motors();
        regs=NULL;
        munmap(base, span);
        close(fd);
        return 1;
    }

    stop_motors();

    wr(robot_hw::ENCODER_SX_ENABLE, 0);
    wr(robot_hw::ENCODER_DX_ENABLE, 0);
    wr(robot_hw::ENCODER_SX_LOAD, 32768);
    wr(robot_hw::ENCODER_DX_LOAD, 32768);
    wr(robot_hw::ENCODER_SX_ENABLE, 1);
    wr(robot_hw::ENCODER_DX_ENABLE, 1);

    unsigned control = 0x04 | (direction << 1);
    wr(motor, period);
    wr(motor + 8, control);
    wr(motor + 4, high);

    FILE *info=csv ? stderr : stdout;
    fprintf(info,"Canale=%s direzione_bit=%u PWM=%ld%% durata=300 ms\n",
           argv[1], direction, percent);
    fflush(info);
    PulseSample samples[128];unsigned sample_count=0;
    double started=now_seconds();

    if (!watchdog_set(500000)) {
        perror("watchdog impulso"); stop_requested=1;
    }
    if (!stop_requested) {
        started=now_seconds();
        const double deadline = started + 0.300;
        double next_sample=started;
        if (!stop_requested) wr(motor + 8, control | 1);
        if (stop_requested) stop_motors();

        while (!stop_requested && now_seconds() < deadline) {
            const double now=now_seconds();
            if(csv && now>=next_sample && sample_count<128) {
                samples[sample_count++]=pulse_sample(now-started,1);
                next_sample=now+.010;
                const PulseSample &s=samples[sample_count-1];
                if(s.sx==0 || s.sx==65535 || s.dx==0 || s.dx==65535)request_stop(0);
            }
            usleep(1000);
        }
    }

    stop_motors();
    watchdog_set(0);

    // Conta anche il breve movimento residuo dopo l'arresto.
    if(csv) {
        // Record exact first observed OFF transition and subsequent spin-down.
        const double stopped=now_seconds();
        double next_sample=stopped;
        while(!stop_requested && now_seconds()<stopped+.200) {
            const double now=now_seconds();
            if(now>=next_sample && sample_count<128) {
                samples[sample_count++]=pulse_sample(now-started,2);
                next_sample=now+.010;
            }
            usleep(1000);
        }
    } else usleep(200000);
    wr(robot_hw::ENCODER_SX_ENABLE, 0);
    wr(robot_hw::ENCODER_DX_ENABLE, 0);

    int left = static_cast<int>(rd(robot_hw::ENCODER_SX_VALUE) & 0xffff);
    int right = static_cast<int>(rd(robot_hw::ENCODER_DX_VALUE) & 0xffff);

    if(csv) {
        printf("t_s,phase,enc_sx_raw,enc_dx_raw,pwm_sx_applied_percent,pwm_dx_applied_percent,control_sx,control_dx\n");
        for(unsigned i=0;i<sample_count;++i) {
            const PulseSample &s=samples[i];
            printf("%.9f,%u,%u,%u,%.6f,%.6f,%u,%u\n",s.time,s.phase,s.sx,s.dx,
                sampled_duty(s.high_sx,s.period_sx,s.control_sx),
                sampled_duty(s.high_dx,s.period_dx,s.control_dx),s.control_sx,s.control_dx);
        }
        fflush(stdout);
    }
    fprintf(info,"Spostamento encoder: SX=%+d DX=%+d conteggi\n",
           left - 32768, right - 32768);
    fprintf(info,"Controllo finale: SX=0x%02x DX=0x%02x\n",
           static_cast<unsigned>(rd(robot_hw::PWM_SX_CONTROL)),
           static_cast<unsigned>(rd(robot_hw::PWM_DX_CONTROL)));
    fprintf(info,"Esito: %s\n",watchdog_expired ? "WATCHDOG" :
           (stop_requested ? "INTERROTTO/ERRORE" : "COMPLETATO"));

    regs=NULL;
    munmap(base, span);
    close(fd);
    close(lock_fd);
    return stop_requested || watchdog_expired ? 1 : 0;
}
#endif
