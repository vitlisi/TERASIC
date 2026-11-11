#define _FILE_OFFSET_BITS 64
#include "ultrasonic_core.h"
#include "robot_wheel_map.h"
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t interrupted=0;
static void stop(int) { interrupted=1; }
static bool now_ns(uint64_t *value)
{
    timespec ts;
    if(clock_gettime(CLOCK_MONOTONIC,&ts)) { perror("clock_gettime");return false; }
    *value=static_cast<uint64_t>(ts.tv_sec)*1000000000ULL+ts.tv_nsec;
    return true;
}
int main(int argc,char **argv)
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    if(argc==2 && !strcmp(argv[1],"check")) {
        fprintf(stderr,"Diagnostico sola lettura: FPGA0xFF200098, clock nominale50MHz; nessun accesso hardware.\n");
        return 0;
    }
    char *end=NULL;errno=0;
    const long seconds=argc==2 ? strtol(argv[1],&end,10) : 30;
    if(argc>2 || (argc==2 && (errno || end==argv[1] || *end)) || seconds<1 || seconds>300) {
        fprintf(stderr,"Uso: %s [secondi1..300 | check]\n",argv[0]);return 2;
    }
    // Same lock as all LQR/PD tools; never run a controller during this test.
    const int lock=open("/tmp/terasic_balance_lqr.lock",O_RDWR|O_CREAT,0600);
    if(lock<0) { perror("lock");return 1; }
    if(flock(lock,LOCK_EX|LOCK_NB)) { perror("Controllo gia attivo");close(lock);return 1; }
    const int fd=open("/dev/mem",O_RDONLY|O_SYNC);
    if(fd<0) { perror("open /dev/mem");close(lock);return 1; }
    void *mapped=mmap(NULL,0x1000,PROT_READ,MAP_SHARED,fd,static_cast<off_t>(0xFF200000ULL));
    if(mapped==MAP_FAILED) { perror("mmap");close(fd);close(lock);return 1; }
    volatile const uint32_t *regs=static_cast<volatile const uint32_t *>(mapped);
    signal(SIGINT,stop);signal(SIGTERM,stop);signal(SIGHUP,stop);
    uint64_t start=0,last_change=0;int result=0;
    if(!now_ns(&start))result=1;
    last_change=start;uint32_t previous=0;bool first=true;
    fprintf(stderr,"Ultrasuoni, sola lettura; robot sostenuto/appoggiato, motori spenti.\n"
        "Nessun flag di eco nuovo: un valore in portata puo essere vecchio.\n"
        "Spostare un bersaglio piano davanti al sensore, poi rimuoverlo. Ctrl+C termina.\n");
    printf("t_s,raw,echo_us,distance_cm,in_range,raw_changed,unchanged_s,motor_sx_control,motor_dx_control,status\n");
    while(!result && !interrupted) {
        uint64_t now=0;
        if(!now_ns(&now)) { result=1;break; }
        if(now-start>=static_cast<uint64_t>(seconds)*1000000000ULL)break;
        const uint32_t sx=regs[robot_hw::PWM_SX_CONTROL/4],dx=regs[robot_hw::PWM_DX_CONTROL/4];
        if((sx&1u) || (dx&1u)) {
            fprintf(stderr,"Test annullato: motore in RUN; nessuna scrittura hardware effettuata.\n");result=1;break;
        }
        const uint32_t raw=regs[0x98/4];
        const bool changed=first || raw!=previous;
        if(changed)last_change=now;
        const char *status=ultrasonic_status(raw);
        const bool in_range=ultrasonic_in_range(raw);
        const double elapsed=(now-start)*1e-9,unchanged=(now-last_change)*1e-9;
        printf("%.3f,%u,%.3f,",elapsed,static_cast<unsigned>(raw),ultrasonic_echo_us(raw));
        if(in_range)printf("%.3f",ultrasonic_distance_cm(raw));else printf("nan");
        printf(",%d,%d,%.3f,%u,%u,%s\n",in_range ? 1 : 0,(!first && changed) ? 1 : 0,
            unchanged,static_cast<unsigned>(sx),static_cast<unsigned>(dx),status);
        fflush(stdout);
        if(in_range)fprintf(stderr,"t=%5.1fs | distanza=%7.2f cm | raw=%u | invariato %.1fs | eco nuovo: NON CERTIFICATO\n",
            elapsed,ultrasonic_distance_cm(raw),static_cast<unsigned>(raw),unchanged);
        else fprintf(stderr,"t=%5.1fs | distanza=n/d | raw=%u | %s\n",elapsed,static_cast<unsigned>(raw),status);
        previous=raw;first=false;
        timespec delay={0,100000000};
        while(nanosleep(&delay,&delay) && errno==EINTR && !interrupted) {}
    }
    if(interrupted)result=130;
    fprintf(stderr,"Diagnostico terminato, exit_code=%d; nessuna scrittura hardware.\n",result);
    munmap(mapped,0x1000);close(fd);close(lock);return result;
}
