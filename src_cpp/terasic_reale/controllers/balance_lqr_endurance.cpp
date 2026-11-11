#define _FILE_OFFSET_BITS 64
#define __STDC_FORMAT_MACROS
#include "balance_startup_limits.h"
#include "../hardware/robot_wheel_map.h"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdint.h>
#include <inttypes.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/file.h>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include "balance_lqr_path_simple_core.h"
#include "accel_norm_guard.h"
#include "balance_lqr_endurance_core.h"
// Separate path reference executable. Same six-state feedback and two PWM.

// LQR sperimentale separato dal PD. Hardware derivato dal PD verificato.
// Nessun guadagno predefinito: richiede modello e calibrazione encoder validati.
static volatile uint32_t *fpga = NULL;
static volatile sig_atomic_t stop_requested = 0;
static volatile sig_atomic_t watchdog_expired = 0;

static bool hardware_process_conflict()
{
    DIR *directory = opendir("/proc");
    if (!directory) { perror("opendir /proc"); return true; }
    const char *names[] = {"hinf_path_yaw", "hinf_path", "balance_hinf", "pitch_live", "pitch_pd", "pitch_pd_fixed", "balance_lqr", "balance_lqr_pos", "balance_lqr_2", "balance_path", "balance_path_v2", "balance_path_v3", "balance_path_v4", "balance_path_v5", "balance_path_v6", "balance_path_v7", "path_simple", "lqr_endurance",
        "robot_test", "balance_car", "bal_car", "DE10_Nano_bal", "imu_measure",
        "imu_identity", "motor_pulse", "encoder_manual", "battery_status"};
    bool conflict = false;
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        char *end = NULL;
        const long pid = strtol(entry->d_name, &end, 10);
        if (pid <= 0 || *end || pid == static_cast<long>(getpid())) continue;
        char path[128], name[64] = {0};
        std::snprintf(path, sizeof(path), "/proc/%ld/comm", pid);
        FILE *file = std::fopen(path, "r");
        if (!file) continue;
        const bool read_ok = std::fgets(name, sizeof(name), file) != NULL;
        std::fclose(file);
        if (!read_ok) continue;
        name[std::strcspn(name, "\r\n")] = 0;
        for (unsigned i=0; i<sizeof(names)/sizeof(names[0]); ++i) {
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

static uint32_t reg_read(unsigned offset) { return fpga[offset / 4]; }
static void reg_write(unsigned offset, uint32_t value) { fpga[offset / 4] = value; }
static unsigned i2c_read(unsigned index) { return reg_read(index * 4) & 0xffu; }
static void i2c_write(unsigned index, unsigned value) { reg_write(index * 4, value); }

static void stop_motors()
{
    if (!fpga) return;
    reg_write(robot_hw::PWM_SX_CONTROL, 0x06); // SX: FAST_DECAY=1, FORWARD=1, RUN=0.
    reg_write(robot_hw::PWM_DX_CONTROL, 0x06); // DX.
    reg_write(robot_hw::PWM_SX_HIGH, 0);
    reg_write(robot_hw::PWM_DX_HIGH, 0);
}

static void handle_stop(int signal_number)
{
    stop_requested = 1;
    if (signal_number == SIGALRM) watchdog_expired = 1;
    stop_motors(); // Solo scritture MMIO: ferma i canali anche durante una lettura bloccata.
}

static void watchdog_set(unsigned microseconds)
{
    struct itimerval timer;
    std::memset(&timer, 0, sizeof(timer));
    timer.it_value.tv_sec = microseconds / 1000000u;
    timer.it_value.tv_usec = microseconds % 1000000u;
    setitimer(ITIMER_REAL, &timer, NULL);
}

static uint64_t now_ns()
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) {
        perror("clock_gettime");
        stop_requested = 1;
        return 0;
    }
    return static_cast<uint64_t>(t.tv_sec) * 1000000000u + t.tv_nsec;
}

static bool i2c_command(unsigned command, bool check_ack)
{
    i2c_write(4, command | 0x01); // IACK.
    const uint64_t deadline = now_ns() + 20000000u;
    do {
        const unsigned status = i2c_read(4);
        if (status & 0x20u) { fprintf(stderr, "I2C: arbitraggio perso\n"); return false; }
        if ((status & 0x01u) && !(status & 0x02u)) {
            if (check_ack && (status & 0x80u)) {
                fprintf(stderr, "I2C: NACK\n"); return false;
            }
            return true;
        }
        usleep(50);
    } while (!stop_requested && now_ns() < deadline);
    fprintf(stderr, "I2C: timeout/interruzione\n");
    return false;
}

static bool bus_free()
{
    const uint64_t deadline = now_ns() + 20000000u;
    do {
        if (!(i2c_read(4) & 0x40u)) return true;
        usleep(50);
    } while (!stop_requested && now_ns() < deadline);
    fprintf(stderr, "I2C: bus occupato\n");
    return false;
}

static bool imu_read(unsigned address, unsigned char *data, unsigned length)
{
    i2c_write(3, 0x68u << 1);
    if (!i2c_command(0x90, true)) return false;
    i2c_write(3, address);
    if (!i2c_command(0x10, true)) return false;
    i2c_write(3, (0x68u << 1) | 1u);
    if (!i2c_command(0x90, true)) return false;
    for (unsigned i = 0; i < length; ++i) {
        if (!i2c_command(i + 1u == length ? 0x68 : 0x20, false)) return false;
        data[i] = static_cast<unsigned char>(i2c_read(3));
    }
    return bus_free();
}

static bool imu_write(unsigned address, unsigned value)
{
    i2c_write(3, 0x68u << 1);
    if (!i2c_command(0x90, true)) return false;
    i2c_write(3, address);
    if (!i2c_command(0x10, true)) return false;
    i2c_write(3, value);
    return i2c_command(0x50, true) && bus_free();
}

static bool imu_setup()
{
    i2c_write(2, 0);
    i2c_write(4, 1);
    i2c_write(0, 99); // 100 kHz da clock FPGA di 50 MHz.
    i2c_write(1, 0);
    i2c_write(2, 0x80);
    unsigned char id = 0;
    if (!bus_free() || !imu_read(0x75, &id, 1) || id != 0x70u) {
        fprintf(stderr, "MPU-6500 WHO_AM_I inatteso: 0x%02x\n", id);
        return false;
    }
    if (!imu_write(0x6B, 0x80)) return false;
    usleep(100000);
    const unsigned addresses[] = {0x6B, 0x6C, 0x1B, 0x1C, 0x1A, 0x1D, 0x19};
    const unsigned values[] = {0x01, 0x00, 0x00, 0x00, 0x03, 0x03, 0x09};
    for (unsigned i = 0; i < 7; ++i)
        if (!imu_write(addresses[i], values[i])) return false;
    usleep(100000);
    for (unsigned i = 0; i < 7; ++i) {
        unsigned char value = 0;
        if (!imu_read(addresses[i], &value, 1) || value != values[i]) {
            fprintf(stderr, "Configurazione IMU non confermata: registro 0x%02x\n",
                    addresses[i]);
            return false;
        }
    }
    return true;
}

static int signed_word(const unsigned char *p)
{
    int value = (static_cast<int>(p[0]) << 8) | p[1];
    return value >= 32768 ? value - 65536 : value;
}

struct Sample {
    int ax, ay, az, gx, gy, gz;
    double angle_deg, gyro_deg_s, accel_g;
};

static bool get_sample(Sample *sample)
{
    unsigned char data[14];
    if (!imu_read(0x3B, data, 14)) return false;
    sample->ax = signed_word(data);
    sample->ay = signed_word(data + 2);
    sample->az = signed_word(data + 4);
    sample->gx = signed_word(data + 8);
    sample->gy = signed_word(data + 10);
    sample->gz = signed_word(data + 12);
    const double x = static_cast<double>(sample->ax);
    const double y = static_cast<double>(sample->ay);
    const double z = static_cast<double>(sample->az);
    sample->accel_g = std::sqrt(x*x + y*y + z*z) / 16384.0;
    // Con il robot verticale sul telaio attuale l'IMU misura az circa +1 g.
    sample->angle_deg = std::atan2(-x, z) * (180.0 / 3.141592653589793);
    sample->gyro_deg_s = sample->gy / 131.0;
    return true;
}

static bool motors_running()
{
    return (reg_read(robot_hw::PWM_SX_CONTROL) & 1u) || (reg_read(robot_hw::PWM_DX_CONTROL) & 1u);
}

static void encoders_start()
{
    reg_write(robot_hw::ENCODER_SX_ENABLE, 0);
    reg_write(robot_hw::ENCODER_DX_ENABLE, 0);
    reg_write(robot_hw::ENCODER_SX_LOAD, 32768);
    reg_write(robot_hw::ENCODER_DX_LOAD, 32768);
    reg_write(robot_hw::ENCODER_SX_ENABLE, 1);
    reg_write(robot_hw::ENCODER_DX_ENABLE, 1);
}

static void encoders_stop()
{
    reg_write(robot_hw::ENCODER_SX_ENABLE, 0);
    reg_write(robot_hw::ENCODER_DX_ENABLE, 0);
}

static void set_duties(double sx,double dx)
{
    if (stop_requested) { stop_motors(); return; }
    const double value[2]={sx,dx};
    uint32_t control[2], high[2];
    for (int i=0;i<2;++i) {
        high[i]=std::fabs(value[i])<.5 ? 0u :
            static_cast<unsigned>(50000*std::fabs(value[i])/100.);
        control[i]=high[i] ? (0x05u | (value[i]>0 ? 0x02u : 0u)) : 0x06u;
    }
    if ((reg_read(robot_hw::PWM_SX_CONTROL)&2u)!=(control[0]&2u) ||
        (reg_read(robot_hw::PWM_DX_CONTROL)&2u)!=(control[1]&2u)) stop_motors();
    reg_write(robot_hw::PWM_SX_PERIOD,50000);reg_write(robot_hw::PWM_DX_PERIOD,50000);
    reg_write(robot_hw::PWM_SX_HIGH,high[0]);reg_write(robot_hw::PWM_DX_HIGH,high[1]);
    reg_write(robot_hw::PWM_SX_CONTROL,control[0]);reg_write(robot_hw::PWM_DX_CONTROL,control[1]);
    if (stop_requested) stop_motors();
}

static bool parse_double(const char *s, double min_value, double max_value,
                         double *out)
{
    char *end = NULL;
    errno = 0;
    const double value = strtod(s, &end);
    if (errno || end == s || *end || !(value >= min_value && value <= max_value))
        return false;
    *out = value;
    return true;
}

struct LqrConfig {
    double k[2][6],cpr,pitch_reference_deg,radius,track,max_diff;
    bool shadow_only;
    int gyro_sign,motor_sign,pitch_sign;
    int path_mode;
    double path_distance,path_turn,turn_diff;
    LqrPathModel ff;
    int yaw_gyro_sign;
    double yaw_tau,turn_wheel_pwm,leg_duration_s,turn_duration_s,initial_hold_s,between_hold_s,final_hold_s;
    LqrConfig() : cpr(0),pitch_reference_deg(0),radius(0),track(0),max_diff(0),
        shadow_only(true),gyro_sign(1),motor_sign(1),pitch_sign(1),
        path_mode(0),path_distance(0),path_turn(0),turn_diff(0),yaw_gyro_sign(0),yaw_tau(0),turn_wheel_pwm(0),leg_duration_s(0),turn_duration_s(0),initial_hold_s(3),between_hold_s(2),final_hold_s(3)
    { for(int i=0;i<2;++i)for(int j=0;j<6;++j)k[i][j]=0; }
};

static bool read_lqr_config(const char *path,LqrConfig *c)
{
    FILE *f=fopen(path,"r");if(!f){perror(path);return false;}
    const char *keys[]={"counts_per_rev","gyro_sign","motor_sign","encoder_sx_sign",
        "encoder_dx_sign","wheel_model_sign","pitch_model_sign","pitch_reference_deg",
        "shadow_only","wheel_radius_m","wheel_track_m","max_differential_pwm",
        "k_sx_position","k_sx_pitch","k_sx_yaw","k_sx_velocity","k_sx_pitch_rate","k_sx_yaw_rate",
        "k_dx_position","k_dx_pitch","k_dx_yaw","k_dx_velocity","k_dx_pitch_rate","k_dx_yaw_rate",
        "path_mode","path_distance_m","path_turn_deg","turn_differential_pwm",
        "ff_pitch_v","ff_pitch_a","ff_common_v","ff_common_a","ff_diff_yaw_rate","ff_diff_yaw_accel",
        "yaw_gyro_sign","yaw_fusion_tau_s","wheel_pwm_cap","leg_duration_s","turn_duration_s","initial_hold_s","between_hold_s","final_hold_s"};
    bool seen[42]={false},ok=true;char line[512];
    while(fgets(line,sizeof(line),f)) {
        char *comment=strchr(line,'#');if(comment)*comment=0;
        char key[80],value[160],extra[2];
        const int fields=sscanf(line," %79s %159s %1s",key,value,extra);
        if(fields==EOF)continue;
        if(fields!=2){ok=false;break;}
        int index=-1;for(int i=0;i<42;++i)if(!strcmp(key,keys[i]))index=i;
        double number=0;const double largest=std::numeric_limits<double>::max();
        if(index<0 || seen[index] || !parse_double(value,-largest,largest,&number)){ok=false;break;}
        seen[index]=true;
        if(index==0){if(number<100 || number>1000000){ok=false;break;}c->cpr=number;}
        else if(index>=1 && index<=6){
            const int expected[]={1,1,-1,1,-1,1};
            if(number!=expected[index-1]){ok=false;break;}
        } else if(index==7){if(std::fabs(number)>5){ok=false;break;}c->pitch_reference_deg=number;}
        else if(index==8){if(number!=0 && number!=1){ok=false;break;}c->shadow_only=number==1;}
        else if(index==9){if(number<.02 || number>.05){ok=false;break;}c->radius=number;}
        else if(index==10){if(number<.10 || number>.30){ok=false;break;}c->track=number;}
        else if(index==11){if(number<0 || number>50){ok=false;break;}c->max_diff=number;}
        else if(index<24)c->k[(index-12)/6][(index-12)%6]=number;
        else if(index==24){if(number!=0 && number!=1 && number!=2){ok=false;break;}c->path_mode=static_cast<int>(number);}
        else if(index==25){if(number<=0 || number>.40){ok=false;break;}c->path_distance=number;}
        else if(index==26){if(std::fabs(number)>180){ok=false;break;}c->path_turn=number*3.141592653589793/180.;}
        else if(index==27){if(number<0 || number>50){ok=false;break;}c->turn_diff=number;}
        else if(index<34) {
            if(std::fabs(number)>1000){ok=false;break;}
            double *values[]={&c->ff.pitch_v,&c->ff.pitch_a,&c->ff.common_v,&c->ff.common_a,&c->ff.diff_w,&c->ff.diff_alpha};
            *values[index-28]=number;
        } else if(index==34) {
            if(number!=1 && number!=-1){ok=false;break;}c->yaw_gyro_sign=static_cast<int>(number);
        } else if(index==35) {
            if(number<30 || number>120){ok=false;break;}c->yaw_tau=number;
        } else if(index==36) {
            if(number<15 || number>50){ok=false;break;}c->turn_wheel_pwm=number;
        } else if(index<=38) {
            if(number<5 || number>60){ok=false;break;}
            if(index==37)c->leg_duration_s=number;else c->turn_duration_s=number;
        } else {
            if(number<(index==39 ? .5 : 0) || number>10){ok=false;break;}
            if(index==39)c->initial_hold_s=number;
            else if(index==40)c->between_hold_s=number;else c->final_hold_s=number;
        }
    }
    if(ferror(f))ok=false;
    fclose(f);
    for(int i=0;i<37;++i)if(!seen[i])ok=false;
    if(seen[37]!=seen[38])ok=false;
    if((seen[39] || seen[40] || seen[41]) && !(seen[39] && seen[40] && seen[41] && seen[37]))ok=false;
    const double total=2*c->leg_duration_s+c->turn_duration_s+c->initial_hold_s+2*c->between_hold_s+c->final_hold_s;
    if(seen[37] && (total<20 || total>120))ok=false;
    for(int i=0;i<2;++i)if(c->k[i][1]==0 && c->k[i][4]==0)ok=false;
    if(c->max_diff>50 || c->turn_diff<c->max_diff || c->turn_diff>c->turn_wheel_pwm)ok=false;
    if(c->path_mode!=2 || std::fabs(std::fabs(c->path_turn)-3.141592653589793)>1e-9)ok=false;
    if(!ok)fprintf(stderr,"Configurazione PATH incompleta/non valida: nessun avvio.\n");
    return ok;
}

static double pwm_limit(double requested,double configured)
{ return requested<configured ? requested : configured; }

static double cutoff_v=10.5,max_runtime_s=3600;
static int run(bool armed, bool evaluate, double seconds, const LqrConfig &config, double max_pwm)
{
    if (motors_running()) {
        fprintf(stderr, "Motore gia' in RUN: arresto e annullo.\n");
        stop_motors();
        return 1;
    }
    stop_motors();
    // Verify battery ADC while motors are OFF, before calibration/arming.
    reg_write(0x94,1);reg_write(0x90,2);reg_write(0x90,3);reg_write(0x90,2);
    const uint64_t adc_deadline=now_ns()+20000000ULL;
    while(!(reg_read(0x90)&1u) && !stop_requested && now_ns()<adc_deadline)usleep(50);
    const unsigned initial_raw=reg_read(0x94);
    const double initial_voltage=initial_raw*18./4.7/1000.;
    if(stop_requested || !(reg_read(0x90)&1u) || initial_raw==0 || initial_raw>=4095 || initial_voltage<cutoff_v+.2){
        fprintf(stderr,"ADC non valido o tensione iniziale %.3fV sotto margine %.3fV: motori OFF.\n",initial_voltage,cutoff_v+.2);return 1;
    }
    fprintf(stderr,"Batteria iniziale %.3fV; cutoff filtrato%.3fV, max durata%.0fs. Corrente NON misurata.\n",initial_voltage,cutoff_v,max_runtime_s);
    if (!imu_setup()) {
        i2c_write(4, 0x41);
        i2c_write(2, 0);
        return 1;
    }

    // Calibrazione a motori spenti: tenere il robot fermo e verticale.
    fprintf(stderr, "Calibrazione gyro: robot fermo 2 s; pitch entro +/-5 deg dal riferimento FISSO.\n");
    double gyro_x_sum=0,gyro_z_sum=0;
    double angle_sum = 0, gyro_sum = 0, angle_min = 1e9, angle_max = -1e9;
    bool ok = true;
    for (int i = 0; i < 200 && !stop_requested; ++i) {
        Sample s;
        if (!get_sample(&s)) {
            fprintf(stderr, "Calibrazione: lettura IMU fallita al campione %d.\n", i);
            ok = false; break;
        }
        if (s.accel_g < 0.75 || s.accel_g > 1.25 || s.az < 11000 ||
            !balance_startup_pitch_ok(s.angle_deg,config.pitch_reference_deg)) {
            fprintf(stderr, "Calibrazione: postura non valida al campione %d: "
                            "ax=%d ay=%d az=%d |a|=%.3f g pitch=%.2f deg.\n",
                    i, s.ax, s.ay, s.az, s.accel_g, s.angle_deg);
            ok = false; break;
        }
        angle_sum += s.angle_deg;
        gyro_sum += s.gyro_deg_s;
        gyro_x_sum+=s.gx/131.;gyro_z_sum+=s.gz/131.;
        if (s.angle_deg < angle_min) angle_min = s.angle_deg;
        if (s.angle_deg > angle_max) angle_max = s.angle_deg;
        usleep(10000);
    }
    const double initial_pitch = angle_sum / 200.0;
    const double angle_zero = config.pitch_reference_deg;
    const double initial_error = initial_pitch - angle_zero;
    const double gyro_bias = gyro_sum / 200.0;
    const double gyro_x_bias=gyro_x_sum/200.,gyro_z_bias=gyro_z_sum/200.;
    if(std::fabs(gyro_x_bias)>10 || std::fabs(gyro_z_bias)>10)ok=false;
    if (angle_max - angle_min > 3.0 || std::fabs(gyro_bias) > 10.0) {
        fprintf(stderr, "Calibrazione: variazione pitch=%.2f deg, "
                        "bias gyro Y=%.2f deg/s.\n",
                angle_max - angle_min, gyro_bias);
        ok = false;
    }
    if (stop_requested) ok = false;
    if (ok) fprintf(stderr,"Pitch iniziale assoluto=%.3f deg; riferimento FISSO=%.3f deg; errore iniziale=%+.3f deg.\n",
                    initial_pitch,angle_zero,initial_error);
    if (ok && armed && !balance_startup_pitch_ok(initial_pitch,angle_zero)) {
        fprintf(stderr,"Avvio annullato: errore iniziale oltre 5 deg dal riferimento fisso. Motori spenti.\n");
        ok=false;
    }
    if (!ok) {
        fprintf(stderr, "Calibrazione annullata: tenere il robot piu' fermo e verticale.\n");
        i2c_write(2, 0);
        return 1;
    }
    fprintf(stderr, "Riferimento pitch=%.3f deg; bias gyro Y=%.3f deg/s.\n",
            angle_zero, gyro_bias);
    fprintf(stderr,"Bias gyro X=%.6f Z=%.6f deg/s; yaw_sign=%d tau_fusione=%.1f s.\n",gyro_x_bias,gyro_z_bias,config.yaw_gyro_sign,config.yaw_tau);
    fprintf(stderr,"%s MIMO: unico K 2x6, due PWM; durata profilo=%.1f s limite=%.1f%% differenziale max=%.1f%%.\n",
            armed ? "ARMATO" : "SHADOW (motori spenti)",seconds,max_pwm,config.max_diff);
    fprintf(stderr,"Stati [s,psi,rho,v,psi_dot,rho_dot], SI; yaw gyro+encoder; zero dopo calibrazione.\n");
    for(int row=0;row<2;++row)
        fprintf(stderr,"K %s=[%.12g %.12g %.12g %.12g %.12g %.12g]\n",row==0 ? "SX" : "DX",
                config.k[row][0],config.k[row][1],config.k[row][2],config.k[row][3],config.k[row][4],config.k[row][5]);
    fprintf(stderr,"Raggio nominale=%.6f m carreggiata nominale=%.6f m CPR=%.3f\n",config.radius,config.track,config.cpr);
    fprintf(stderr,"PATH mode=%d distanza_per_tratto=%.3f m rotazione=%.1f deg; differenziale in svolta max=%.1f%%\n",
        config.path_mode,config.path_distance,config.path_turn*180/3.141592653589793,config.turn_diff);
    const double hold_total=config.initial_hold_s+2*config.between_hold_s+config.final_hold_s;
    const double leg_seconds=config.leg_duration_s>0 ? config.leg_duration_s : (seconds-hold_total)*.2;
    const double turn_seconds=config.turn_duration_s>0 ? config.turn_duration_s : (seconds-hold_total)*.6;
    fprintf(stderr,"PATH SEMPLICE: +%.1fcm, rotazione%+.1fdeg, +%.1fcm dalla ripartenza. Durata profilo%.1fs.\n",config.path_distance*100,config.path_turn*180/3.141592653589793,config.path_distance*100,seconds);
    fprintf(stderr,"Tempi: fermo%.1fs, avanti%.1fs, fermo%.1fs, svolta%.1fs, fermo%.1fs, avanti%.1fs, fermo%.1fs.\n",config.initial_hold_s,leg_seconds,config.between_hold_s,turn_seconds,config.between_hold_s,leg_seconds,config.final_hold_s);
    fprintf(stderr,"CICLICO40cm/180deg/40cm/180deg ogni35.5s. PWM max%.1f%%. Fine prova: allarme e hold10s poi OFF; sostenere prima.\n",pwm_limit(max_pwm,config.turn_wheel_pwm));
    fprintf(stderr,"Norma accelerazione: un outlier esclude solo la correzione accel,3 consecutivi fermano; altri arresti invariati.\n");
    encoders_start();
    std::printf("t_s,monotonic_ns,cycle,cycle_t_s,phase,voltage_v,voltage_filtered_v,adc_age_s,current_a,current_valid,position_m,position_ref_m,pitch_error_deg,yaw_rad,yaw_ref_rad,world_x_m,world_y_m,cycle_dx_m,cycle_dy_m,cycle_return_error_m,pwm_sx_percent,pwm_dx_percent,imu_us,late_us,encoder_recenters,parking,pitch_rms_window_deg,pwm_peak_window_percent,window_samples,pwm_clipped_samples\n");
    const uint64_t period = 10000000u; // 10 ms richiesti.
    const uint64_t start = now_ns();
    uint64_t previous = start;
    double psi = initial_error;
    const double origin_pitch=config.pitch_sign*initial_error*(3.141592653589793/180.0);
    LqrEnduranceEncoder encoder(config.cpr,config.radius,config.track,origin_pitch);
    double correlation = 0, previous_acc = 0;
    LqrPathYawFusion yaw_fusion(config.yaw_tau);
    double yaw_gyro_integral=0;
    int last_phase=-1;
    LqrEndurancePath path(config.ff);
    EnduranceBattery battery(cutoff_v);
    bool adc_pending=false;uint64_t adc_started=0,last_adc_start=0;
    unsigned recenter_count=0;double voltage=std::numeric_limits<double>::quiet_NaN(),last_voltage_time=-1;
    bool parking=false;double park_started=0,park_s=0,park_yaw=0,last_heartbeat=0;bool supervisor_eof=false;
    const char *end_reason="operator_or_fault";
    double block_pitch2=0,block_peak_pwm=0,block_late=0;unsigned block_count=0,block_clips=0;

    AccelNormGuard accel_guard;
    const long count = static_cast<long>(std::ceil(seconds * 100));
    for (long i = 0; !stop_requested && (armed || i<count); ++i) {
        const uint64_t due = start + static_cast<uint64_t>(i) * period;
        struct timespec wake;
        wake.tv_sec = static_cast<time_t>(due / 1000000000u);
        wake.tv_nsec = static_cast<long>(due % 1000000000u);
        int sleep_result;
        do {
            sleep_result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                           &wake, NULL);
        } while (sleep_result == EINTR && !stop_requested);
        if (stop_requested) break;
        if (sleep_result != 0) { fprintf(stderr, "Sleep fallito.\n"); ok = false; break; }
        const uint64_t t0 = now_ns();
        // In osservazione registra anche i ritardi: solo il controllo armato
        // deve fermarsi quando Linux salta una scadenza del ciclo.
        if (armed && (t0 < previous || (i && t0 - previous > 25000000u) ||
                      (t0 > due && t0 - due > 10000000u))) {
            fprintf(stderr, "Ritardo del ciclo: arresto.\n"); ok = false; break;
        }
        if (!armed && motors_running()) {
            fprintf(stderr, "Motore attivo durante osservazione: arresto.\n");
            ok = false; break;
        }
        if (armed) watchdog_set(50000); // 50 ms senza avanzamento: arresto.
        Sample s;
        if (!get_sample(&s)) { ok = false; break; }
        const uint64_t t1 = now_ns();
        if (stop_requested) { ok = false; break; }
        const double dt = (t0 - previous) * 1e-9;
        previous = t0;
        const double psi_acc = s.angle_deg - angle_zero;
        const double gyro = config.gyro_sign * (s.gyro_deg_s - gyro_bias);
        if (i) correlation += (psi_acc - previous_acc) *
                               (s.gyro_deg_s - gyro_bias);
        previous_acc = psi_acc;
        const bool accel_trusted=accel_guard.update(s.accel_g);
        psi = accel_trusted ? lqr_fuse_pitch_deg(psi,gyro,dt,psi_acc) : psi+gyro*dt;
        if(!accel_trusted && accel_guard.bad_count()==1)
            fprintf(stderr,"Accelerazione fuori intervallo a %.3fs: %.3fg; correzione accel ignorata, bilanciamento attivo salvo altri limiti.\n",(t0-start)*1e-9,s.accel_g);
        if (accel_guard.failed() || s.az < 8000 ||
            std::fabs(psi) > 12.0 || std::fabs(psi_acc) > 15.0 ||
            std::fabs(gyro) > 180.0 || t1 - t0 > 8000000u) {
            fprintf(stderr, "Limite assetto/IMU/tempo superato: "
                            "accel_bad=%u |a|=%.3f g az=%d psi=%.2f deg "
                            "psi_acc=%.2f deg gyro=%.2f deg/s "
                            "imu=%" PRIu64 " us. Arresto.\n",
                    accel_guard.bad_count(),s.accel_g, s.az, psi, psi_acc, gyro,
                    (t1 - t0) / 1000);
            ok = false; break;
        }
        const unsigned enc_sx = reg_read(robot_hw::ENCODER_SX_VALUE) & 0xffffu;
        const unsigned enc_dx = reg_read(robot_hw::ENCODER_DX_VALUE) & 0xffffu;
        LqrMimoState state;
        const bool encoder_valid=encoder.update(enc_sx,enc_dx,dt,
            psi*(3.141592653589793/180.),gyro*(3.141592653589793/180.),&state);
        if(!encoder_valid){fprintf(stderr,"Encoder MIMO non valido/saturo. Arresto.\n");ok=false;break;}
        if(enc_sx<16000 || enc_sx>49000 || enc_dx<16000 || enc_dx>49000){
            // Vendor load registers, counters remain enabled. No motor writes.
            reg_write(robot_hw::ENCODER_SX_LOAD,32768);reg_write(robot_hw::ENCODER_DX_LOAD,32768);
            encoder.recenter();++recenter_count;
            const unsigned sx_check=reg_read(robot_hw::ENCODER_SX_VALUE)&0xffffu,dx_check=reg_read(robot_hw::ENCODER_DX_VALUE)&0xffffu;
            if(std::abs(static_cast<int>(sx_check)-32768)>16 || std::abs(static_cast<int>(dx_check)-32768)>16){
                fprintf(stderr,"END_REASON=encoder_recenter_failed\n");ok=false;break;
            }
        }
        const double yaw_encoder=state.x[2];
        const double gyro_x=s.gx/131.-gyro_x_bias,gyro_z=s.gz/131.-gyro_z_bias;
        const double yaw_rate=lqr_path_yaw_rate_rad_s(gyro_x,gyro_z,psi+angle_zero,config.yaw_gyro_sign);
        yaw_gyro_integral+=yaw_rate*dt;
        state.x[2]=yaw_fusion.update(yaw_encoder,yaw_rate,dt);state.x[5]=yaw_rate;
        const double elapsed=(t0-start)*1e-9;
        LqrPathReference reference=path.update(elapsed,state);
        // ADC acquisition is a nonblocking state machine, independent of IMU I2C.
        // One conversion per100ms, never sleep or poll in the100Hz control loop.
        if(adc_pending && (reg_read(0x90)&1u)){
            const unsigned raw=reg_read(0x94);adc_pending=false;
            if(raw>0 && raw<4095){voltage=raw*18./4.7/1000.;last_voltage_time=elapsed;battery.update(elapsed,voltage);}
        }
        if(adc_pending && t0-adc_started>50000000ULL){
            adc_pending=false;fprintf(stderr,"ADC conversion timeout at %.2fs\n",elapsed);
        }
        if(!adc_pending && (!last_adc_start || t0-last_adc_start>=100000000ULL)){
            reg_write(0x94,1);reg_write(0x90,2);reg_write(0x90,3);reg_write(0x90,2);
            adc_pending=true;adc_started=last_adc_start=t0;
        }
        const bool adc_stale=elapsed>1 && (last_voltage_time<0 || elapsed-last_voltage_time>1);
        char heartbeat[256];const ssize_t received=::read(STDIN_FILENO,heartbeat,sizeof(heartbeat));
        if(received>0)last_heartbeat=elapsed;
        else if(received==0 || (received<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR))supervisor_eof=true;
        const bool supervisor_lost=supervisor_eof || elapsed-last_heartbeat>5;
        if(!parking && (battery.low() || adc_stale || elapsed>=max_runtime_s || supervisor_lost)){
            parking=true;park_started=elapsed;park_s=state.x[0];park_yaw=state.x[2];
            end_reason=battery.low()?"low_voltage":(adc_stale?"adc_stale":(supervisor_lost?"supervisor_lost":"maximum_duration"));
            fprintf(stderr,"END_REASON=%s t=%.2f V=%.3f; percorso sospeso, SOSTENERE IL ROBOT ORA: motori OFF fra10s.\n",end_reason,elapsed,battery.filtered());
        }
        if(parking){
            for(int j=0;j<6;++j)reference.x[j]=0;
            reference.x[0]=park_s;reference.x[2]=park_yaw;reference.ff[0]=reference.ff[1]=0;reference.phase=7;
            if(elapsed-park_started>=10){fprintf(stderr,"Parcheggio terminato; motori OFF.\n");break;}
        }
        if(armed && (std::fabs(state.x[3])>.30 || std::fabs(state.x[5])>1.5707963268)){
            end_reason="motion_envelope";fprintf(stderr,"END_REASON=motion_envelope v=%.4fm/s yaw_rate=%.2fdeg/s\n",state.x[3],state.x[5]*180/3.141592653589793);ok=false;break;
        }
        const double wheel_limit=pwm_limit(max_pwm,config.turn_wheel_pwm);
        const double differential_limit=wheel_limit;
        const LqrMimoCommand command=lqr_path_command(state,reference,config.k,config.motor_sign,wheel_limit,differential_limit);
        if(!lqr_finite(command.raw[0]) || !lqr_finite(command.raw[1])) {
            fprintf(stderr,"Comando MIMO non finito. Arresto.\n");ok=false;break;
        }
        double applied[2]={0,0};
        if(armed && evaluate) {
            applied[0]=command.applied[0];applied[1]=command.applied[1];
            set_duties(command.limited[0],command.limited[1]);
            if((std::fabs(applied[0])>=.5 && !(reg_read(robot_hw::PWM_SX_CONTROL)&1u)) ||
               (std::fabs(applied[1])>=.5 && !(reg_read(robot_hw::PWM_DX_CONTROL)&1u))) {
                fprintf(stderr,"RUN canale MIMO non confermato. Arresto.\n");ok=false;break;
            }
            watchdog_set(50000);
        }
        block_pitch2+=psi*psi;++block_count;
        if(command.raw[0]!=command.limited[0] || command.raw[1]!=command.limited[1])++block_clips;
        const double peak=::fmax(std::fabs(applied[0]),std::fabs(applied[1]));
        block_peak_pwm=::fmax(block_peak_pwm,peak);block_late=::fmax(block_late,t0>due?(t0-due)/1000.:0);
        if(i%10==0 || reference.phase!=last_phase){
            last_phase=reference.phase;
            const double dx=path.world_x()-path.cycle_x(),dy=path.world_y()-path.cycle_y();
            std::printf("%.6f,%" PRIu64 ",%lu,%.6f,%d,%.6f,%.6f,%.6f,nan,0,%.9f,%.9f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.3f,%.3f,%" PRIu64 ",%.1f,%u,%d,%.6f,%.3f,%u,%u\n",
                elapsed,t0,path.cycle(),path.cycle_time(),reference.phase,voltage,battery.filtered(),last_voltage_time<0?-1:elapsed-last_voltage_time,
                state.x[0],reference.x[0],psi,state.x[2],reference.x[2],path.world_x(),path.world_y(),dx,dy,std::sqrt(dx*dx+dy*dy),applied[0],applied[1],(t1-t0)/1000,block_late,recenter_count,parking?1:0,std::sqrt(block_pitch2/block_count),block_peak_pwm,block_count,block_clips);
            if(std::fflush(stdout)!=0){fprintf(stderr,"END_REASON=log_write_failed\n");ok=false;break;}
            block_pitch2=block_peak_pwm=block_late=0;block_count=block_clips=0;
        }
        if(i%100==0){
            fprintf(stderr,"t=%.1fs ciclo=%lu fase=%d V=%.3f filtrata=%.3f pitch=%.3f PWMpicco=%.1f%% %s\n",
                elapsed,path.cycle(),reference.phase,voltage,battery.filtered(),psi,peak,parking?"SOSTENERE: spegnimento imminente":"");
        }

    }
    stop_motors();
    encoders_stop();
    watchdog_set(0);
    if (!ok) i2c_write(4, 0x41);
    i2c_write(2, 0);
    fprintf(stderr, "LQR terminato; RUN SX=%u DX=%u.\n",
            static_cast<unsigned>(reg_read(robot_hw::PWM_SX_CONTROL) & 1u),
            static_cast<unsigned>(reg_read(robot_hw::PWM_DX_CONTROL) & 1u));
    fprintf(stderr,"END_REASON_FINAL=%s stop_signal=%d watchdog=%d ok=%d cicli_avviati=%lu XY=(%.4f,%.4f)m\n",end_reason,stop_requested?1:0,watchdog_expired?1:0,ok?1:0,path.cycle(),path.world_x(),path.world_y());
    fprintf(stderr, "Encoder delta SX=%+d DX=%+d.\n",
            static_cast<int>(reg_read(robot_hw::ENCODER_SX_VALUE) & 0xffffu) - 32768,
            static_cast<int>(reg_read(robot_hw::ENCODER_DX_VALUE) & 0xffffu) - 32768);
    if (!armed) fprintf(stderr, "Correlazione movimento angolo/gyro grezzo: %.3f "
                               "(serve movimento manuale; segno non validato se vicino a 0).\n",
                       correlation);
    return ok && !stop_requested && !watchdog_expired ? 0 : 1;
}

static void usage(const char *name)
{
    fprintf(stderr,"Uso: %s check config | shadow/run config 28.5 pwm_max_0..50 cutoff_10.5..12V max_durata_30..14400s\n",name);
}
int main(int argc,char **argv)
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    const bool armed=argc>1 && !strcmp(argv[1],"run");
    const bool shadow=argc>1 && !strcmp(argv[1],"shadow");
    const bool check=argc>1 && !strcmp(argv[1],"check");
    double seconds=0;double max_pwm=0;LqrConfig config;
    if((check && (argc!=3 || !read_lqr_config(argv[2],&config))) ||
       ((armed || shadow) && (argc!=7 || !read_lqr_config(argv[2],&config) ||
          !parse_double(argv[3],20,120,&seconds) || !parse_double(argv[4],0,50,&max_pwm) || !parse_double(argv[5],10.5,12,&cutoff_v) || !parse_double(argv[6],30,14400,&max_runtime_s))) ||
       (!armed && !shadow && !check)) { usage(argv[0]);return 2; }
    if(config.path_distance!=.4 || config.leg_duration_s!=8 || config.turn_duration_s!=7 || config.initial_hold_s!=1.5 || config.between_hold_s!=1 || config.final_hold_s!=2){fprintf(stderr,"Endurance richiede config40cm/28.5s verificata.\n");return 2;}
    if (check) {
        fprintf(stderr,"Struttura configurazione valida; modello e K richiedono validazione fisica. Nessun accesso hardware.\n");
        if (config.shadow_only) fprintf(stderr,"Configurazione riservata a SHADOW: run disabilitato.\n");
        return 0;
    }
    if((armed || shadow) && config.leg_duration_s>0 &&
       std::fabs(seconds-(2*config.leg_duration_s+config.turn_duration_s+config.initial_hold_s+2*config.between_hold_s+config.final_hold_s))>1e-9){
        fprintf(stderr,"Durata incoerente: con questi tempi usare %.1fs. Nessun accesso hardware.\n",2*config.leg_duration_s+config.turn_duration_s+config.initial_hold_s+2*config.between_hold_s+config.final_hold_s);return 2;
    }
    if (armed && config.shadow_only) {
        fprintf(stderr,"Configurazione riservata a SHADOW: run rifiutato prima dell'accesso hardware.\n");
        return 2;
    }
    // Lock closes the race between simultaneous LQR starts; the process check
    // also covers older hardware tools that do not take this lock.
    const int lock_fd=open("/tmp/terasic_balance_lqr.lock",O_RDWR|O_CREAT,0600);
    if (lock_fd<0) { perror("lock LQR"); return 1; }
    if (flock(lock_fd,LOCK_EX|LOCK_NB)!=0) { perror("LQR gia' attivo"); close(lock_fd); return 1; }
    if (hardware_process_conflict()) { close(lock_fd); return 1; }
    signal(SIGINT, handle_stop);
    signal(SIGTERM, handle_stop);
    signal(SIGHUP, handle_stop);
    signal(SIGALRM, handle_stop);
    struct sched_param priority;
    std::memset(&priority, 0, sizeof(priority));
    priority.sched_priority = 10;
    if (sched_setscheduler(0, SCHED_FIFO, &priority) != 0) {
        perror("sched_setscheduler SCHED_FIFO");
        if (armed) return 1;
    } else {
        fprintf(stderr, "Scheduling SCHED_FIFO, priorita' 10.\n");
    }
    const int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    const size_t span = 0x1000;
    void *mapped = mmap(NULL, span, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                        static_cast<off_t>(0xFF200000u));
    if (mapped == MAP_FAILED) { perror("mmap"); close(fd); return 1; }
    fpga = static_cast<volatile uint32_t *>(mapped);
    setvbuf(stdout, NULL, _IOFBF, 8192);
    const int input_flags=fcntl(STDIN_FILENO,F_GETFL,0);
    if(input_flags<0 || fcntl(STDIN_FILENO,F_SETFL,input_flags|O_NONBLOCK)<0){stop_motors();munmap(mapped,span);close(fd);close(lock_fd);return 1;}
    const int result = run(armed, armed || shadow, seconds, config, max_pwm);
    stop_motors();
    watchdog_set(0);
    fpga = NULL;
    munmap(mapped, span);
    close(fd);
    close(lock_fd);
    return result;
}
