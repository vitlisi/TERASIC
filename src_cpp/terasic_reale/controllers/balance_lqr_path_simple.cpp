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
#ifdef TERASIC_HINF_PATH
#include "balance_hinf_path_core.h"
#endif
#include "accel_norm_guard.h"
// Separate path reference executable. Same six-state feedback and two PWM.

// LQR sperimentale separato dal PD. Hardware derivato dal PD verificato.
// Nessun guadagno predefinito: richiede modello e calibrazione encoder validati.
static volatile uint32_t *fpga = NULL;
static volatile sig_atomic_t stop_requested = 0;
static volatile sig_atomic_t watchdog_expired = 0;

static long path_control_sample_count(double profile_seconds, bool armed)
{
    // LQR and Hinf keep balancing for 4.5 s after the timed path, then
    // stops its motors through the normal cleanup path. Shadow logs the path
    // only. Other controllers sharing this source retain their own protocol.
#if !defined(TERASIC_GA_PATH)
    return static_cast<long>(std::ceil((profile_seconds + (armed ? 4.5 : 0.0)) * 100.0));
#else
    (void)armed;
    return static_cast<long>(std::ceil(profile_seconds * 100.0));
#endif
}

static bool hardware_process_conflict()
{
    DIR *directory = opendir("/proc");
    if (!directory) { perror("opendir /proc"); return true; }
    const char *names[] = {"hinf_path_yaw", "hinf_path", "balance_hinf", "pitch_live", "pitch_pd", "pitch_pd_fixed", "balance_lqr", "balance_lqr_pos", "balance_lqr_2", "balance_path", "balance_path_v2", "balance_path_v3", "balance_path_v4", "balance_path_v5", "balance_path_v6", "balance_path_v7", "path_simple",
        "path_continuous", "robot_test", "balance_car", "bal_car", "DE10_Nano_bal", "imu_measure",
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
    bool continuous_reference;
    int gyro_sign,motor_sign,pitch_sign;
    int path_mode;
    double path_distance,path_turn,turn_diff,turn_yaw_reserve_percent;
    LqrPathModel ff;
    int yaw_gyro_sign;
    double yaw_tau,turn_wheel_pwm,leg_duration_s,turn_duration_s,initial_hold_s,between_hold_s,final_hold_s;
    LqrConfig() : cpr(0),pitch_reference_deg(0),radius(0),track(0),max_diff(0),
        shadow_only(true),continuous_reference(false),gyro_sign(1),motor_sign(1),pitch_sign(1),
        path_mode(0),path_distance(0),path_turn(0),turn_diff(0),turn_yaw_reserve_percent(0),yaw_gyro_sign(0),yaw_tau(0),turn_wheel_pwm(0),leg_duration_s(0),turn_duration_s(0),initial_hold_s(3),between_hold_s(2),final_hold_s(3)
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
        "yaw_gyro_sign","yaw_fusion_tau_s","wheel_pwm_cap","leg_duration_s","turn_duration_s","initial_hold_s","between_hold_s","final_hold_s","continuous_reference","turn_yaw_reserve_percent"};
    bool seen[44]={false},ok=true;char line[512];
    while(fgets(line,sizeof(line),f)) {
        char *comment=strchr(line,'#');if(comment)*comment=0;
        char key[80],value[160],extra[2];
        const int fields=sscanf(line," %79s %159s %1s",key,value,extra);
        if(fields==EOF)continue;
        if(fields!=2){ok=false;break;}
        int index=-1;for(int i=0;i<44;++i)if(!strcmp(key,keys[i]))index=i;
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
        else if(index<24){
#ifdef TERASIC_HINF_PATH
            ok=false;break; // A Hinf path configuration must not contain LQR gains.
#else
            c->k[(index-12)/6][(index-12)%6]=number;
#endif
        }
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
        } else if(index==43) {
#ifdef TERASIC_HINF_PATH
            if(number<0 || number>15){ok=false;break;}
            c->turn_yaw_reserve_percent=number;
#else
            ok=false;break;
#endif
        } else if(index==42) {
            if(number!=0 && number!=1){ok=false;break;}
            c->continuous_reference=number==1;
        } else {
            if(number<(index==39 ? .5 : 0) || number>10){ok=false;break;}
            if(index==39)c->initial_hold_s=number;
            else if(index==40)c->between_hold_s=number;else c->final_hold_s=number;
        }
    }
    if(ferror(f))ok=false;
    fclose(f);
    for(int i=0;i<37;++i){
#ifdef TERASIC_HINF_PATH
        if(i>=12 && i<24)continue;
#endif
        if(!seen[i])ok=false;
    }
    if(seen[37]!=seen[38])ok=false;
    if((seen[39] || seen[40] || seen[41]) && !(seen[39] && seen[40] && seen[41] && seen[37]))ok=false;
    const double total=2*c->leg_duration_s+c->turn_duration_s+c->initial_hold_s+2*c->between_hold_s+c->final_hold_s;
    if(seen[37] && (total<20 || total>120))ok=false;
#ifdef TERASIC_HINF_PATH
    if(std::fabs(c->radius-.032)>1e-12 || std::fabs(c->track-.16444)>1e-12 || c->turn_wheel_pwm>25 || c->turn_yaw_reserve_percent>=c->turn_wheel_pwm)ok=false;
#else
    for(int i=0;i<2;++i)if(c->k[i][1]==0 && c->k[i][4]==0)ok=false;
#endif
    if(c->max_diff>50 || c->turn_diff<c->max_diff || c->turn_diff>c->turn_wheel_pwm)ok=false;
    if(c->path_mode!=2 || std::fabs(std::fabs(c->path_turn)-3.141592653589793)>1e-9)ok=false;
    if(!ok)fprintf(stderr,"Configurazione PATH incompleta/non valida: nessun avvio.\n");
    return ok;
}

static double pwm_limit(double requested,double configured)
{ return requested<configured ? requested : configured; }

static int run(bool armed, bool evaluate, double seconds, const LqrConfig &config, double max_pwm)
{
    if (motors_running()) {
        fprintf(stderr, "Motore gia' in RUN: arresto e annullo.\n");
        stop_motors();
        return 1;
    }
    stop_motors();
    if (!imu_setup()) {
        i2c_write(4, 0x41);
        i2c_write(2, 0);
        return 1;
    }

    // Calibrazione a motori spenti: tenere il robot fermo e verticale.
    fprintf(stderr, "Avvio dal supporto iniziale: ruote a terra, recupero libero; il supporto non e' uno zero pitch.\n");
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
#ifdef TERASIC_HINF_PATH
    fprintf(stderr,"%s Hinf PATH: dinamico 12 stati; durata profilo=%.1f s limite=%.1f%% differenziale max=%.1f%%.\n",
            armed ? "ARMATO" : "SHADOW (motori spenti)",seconds,max_pwm,config.max_diff);
    fprintf(stderr,"Controller JSON SHA256=%s; tracking error x-reference, shared path feedforward.\n",HINF_EXPORT_SHA256);
    fprintf(stderr,"Riserva PWM yaw=%.3f%% nelle fasi3/4; limite totale per ruota invariato.\n",config.turn_yaw_reserve_percent);
#else
    fprintf(stderr,"%s MIMO: unico K 2x6, due PWM; durata profilo=%.1f s limite=%.1f%% differenziale max=%.1f%%.\n",
            armed ? "ARMATO" : "SHADOW (motori spenti)",seconds,max_pwm,config.max_diff);
    fprintf(stderr,"Stati [s,psi,rho,v,psi_dot,rho_dot], SI; yaw gyro+encoder; zero dopo calibrazione.\n");
    for(int row=0;row<2;++row)
        fprintf(stderr,"K %s=[%.12g %.12g %.12g %.12g %.12g %.12g]\n",row==0 ? "SX" : "DX",
                config.k[row][0],config.k[row][1],config.k[row][2],config.k[row][3],config.k[row][4],config.k[row][5]);
#endif
    fprintf(stderr,"Raggio nominale=%.6f m carreggiata nominale=%.6f m CPR=%.3f\n",config.radius,config.track,config.cpr);
    fprintf(stderr,"PATH mode=%d distanza_per_tratto=%.3f m rotazione=%.1f deg; differenziale in svolta max=%.1f%%\n",
        config.path_mode,config.path_distance,config.path_turn*180/3.141592653589793,config.turn_diff);
    const double hold_total=config.initial_hold_s+2*config.between_hold_s+config.final_hold_s;
    const double leg_seconds=config.leg_duration_s>0 ? config.leg_duration_s : (seconds-hold_total)*.2;
    const double turn_seconds=config.turn_duration_s>0 ? config.turn_duration_s : (seconds-hold_total)*.6;
    fprintf(stderr,"PATH SEMPLICE: +%.1fcm, rotazione%+.1fdeg, +%.1fcm dalla ripartenza. Durata profilo%.1fs.\n",config.path_distance*100,config.path_turn*180/3.141592653589793,config.path_distance*100,seconds);
    fprintf(stderr,"Tempi: fermo%.1fs, avanti%.1fs, fermo%.1fs, svolta%.1fs, fermo%.1fs, avanti%.1fs, fermo%.1fs.\n",config.initial_hold_s,leg_seconds,config.between_hold_s,turn_seconds,config.between_hold_s,leg_seconds,config.final_hold_s);
    fprintf(stderr,"Nessun gate di arrivo o pausa per tracking. PWM max%.1f%% per ruota.\n",pwm_limit(max_pwm,config.turn_wheel_pwm));
#if !defined(TERASIC_GA_PATH)
    if (armed) fprintf(stderr,"Profilo %.1f s; bilanciamento finale 4.5 s; arresto automatico motori a %.1f s dall'attivazione.\n",seconds,seconds+4.5);
#endif
    fprintf(stderr,"Norma accelerazione: un outlier esclude solo la correzione accel,3 consecutivi fermano; altri arresti invariati.\n");
    encoders_start();
    std::printf("sample,t_ns,accel_g,pitch_acc_deg,pitch_fused_deg,gyro_deg_s,imu_us,late_us,enc_sx_raw,enc_dx_raw,"
                "position_m,velocity_m_s,yaw_fused_rad,yaw_rate_gyro_rad_s,"
                "pwm_sx_raw_percent,pwm_dx_raw_percent,pwm_sx_limited_percent,pwm_dx_limited_percent,"
                "pwm_sx_applied_percent,pwm_dx_applied_percent,pwm_common_raw_percent,pwm_differential_raw_percent,"
                "armed,position_reference_m,yaw_reference_rad,encoder_valid,velocity_reference_m_s,yaw_rate_reference_rad_s,pitch_reference_offset_rad,phase,motion_safe,feedforward_sx_percent,feedforward_dx_percent,yaw_encoder_only_rad,yaw_encoder_rate_rad_s,yaw_gyro_integrated_rad,gyro_x_corrected_deg_s,gyro_z_corrected_deg_s,yaw_gyro_rate_rad_s,wheel_limit_percent,differential_limit_percent,world_x_est_m,world_y_est_m,return_error_est_m,schedule_finished,planned_x_m,planned_y_m,planned_leg_m,planned_turn_rad,accel_norm_bad_count,accel_correction_used");
#ifdef TERASIC_HINF_PATH
    for(int j=0;j<6;++j)std::printf(",hinf_feedback_%d",j);
    for(int j=0;j<12;++j)std::printf(",hinf_xc_%d",j);
    RealHinf hinf_controller;
#endif
    std::printf("\n");
    const uint64_t period = 10000000u; // 10 ms richiesti.
    const uint64_t start = now_ns();
    uint64_t previous = start;
    double psi = initial_error;
    const double origin_pitch=config.pitch_sign*initial_error*(3.141592653589793/180.0);
    LqrMimoEstimator encoder(config.cpr,config.radius,config.track,origin_pitch);
    double correlation = 0, previous_acc = 0;
    LqrPathYawFusion yaw_fusion(config.yaw_tau);
    double yaw_gyro_integral=0;
    int last_phase=-1;
    fprintf(stderr,"Riferimento longitudinale: %s.\n",config.continuous_reference ? "continuo nel frame odometrico iniziale" : "ripartenza dalla posizione misurata a ogni fase");
    LqrSimplePath path(seconds,config.path_distance,config.path_turn,config.ff,config.leg_duration_s,config.turn_duration_s,config.initial_hold_s,config.between_hold_s,config.final_hold_s,config.continuous_reference);
    bool completion_announced=false;
    AccelNormGuard accel_guard;
    const long count = path_control_sample_count(seconds,armed);
    long i=0;
    for (; !stop_requested &&
#if defined(TERASIC_GA_PATH)
         (armed || i<count);
#else
         i<count;
#endif
         ++i) {
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
        const double yaw_encoder=state.x[2],yaw_encoder_rate=state.x[5];
        const double gyro_x=s.gx/131.-gyro_x_bias,gyro_z=s.gz/131.-gyro_z_bias;
        const double yaw_rate=lqr_path_yaw_rate_rad_s(gyro_x,gyro_z,psi+angle_zero,config.yaw_gyro_sign);
        yaw_gyro_integral+=yaw_rate*dt;
        state.x[2]=yaw_fusion.update(yaw_encoder,yaw_rate,dt);state.x[5]=yaw_rate;
        const double elapsed=(t0-start)*1e-9;
        const LqrPathReference reference=path.update(elapsed,state);
        const bool motion_safe=std::fabs(state.x[3])<=.30 && std::fabs(state.x[5])<=1.5707963268;
        if(armed && !motion_safe){
            stop_motors();fprintf(stderr,"Velocita oltre inviluppo di sicurezza. Arresto.\n");ok=false;break;
        }
        if(armed && path.schedule_finished(elapsed) && !completion_announced){
            completion_announced=true;
#if !defined(TERASIC_GA_PATH)
            fprintf(stderr,"PROFILO TEMPORALE TERMINATO: XY stimato=(%+.3f,%+.3f)m; BILANCIAMENTO ATTIVO fino allo stop automatico a %.1f s. Arrivo geometrico non verificato automaticamente.\n",path.world_x(),path.world_y(),seconds+4.5);
#else
            fprintf(stderr,"PROFILO TEMPORALE TERMINATO: XY stimato=(%+.3f,%+.3f)m; BILANCIAMENTO ATTIVO. Sostenere il robot e poi Ctrl+C. Arrivo geometrico non verificato automaticamente.\n",path.world_x(),path.world_y());
#endif
        }
        const double wheel_limit=pwm_limit(max_pwm,config.turn_wheel_pwm);
        const double differential_limit=wheel_limit;
#ifdef TERASIC_HINF_PATH
        LqrMimoCommand command;
        if(!hinf_path_command(hinf_controller,state,reference,config.radius,wheel_limit,differential_limit,&command,config.turn_yaw_reserve_percent)) {
            stop_motors();fprintf(stderr,"Hinf path non finito. Arresto.\n");ok=false;break;
        }
#else
        const LqrMimoCommand command=lqr_path_command(state,reference,config.k,config.motor_sign,wheel_limit,differential_limit);
#endif
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
        if(i%100==0 || reference.phase!=last_phase) {
            last_phase=reference.phase;
            fprintf(stderr,"fase=%d target s=%.4f m yaw=%.1f deg | yaw_enc=%+.2f yaw_gyro=%+.2f deg | ",
                reference.phase,reference.x[0],reference.x[2]*180/3.141592653589793,
                yaw_encoder*180/3.141592653589793,yaw_gyro_integral*180/3.141592653589793);
            fprintf(stderr,"t=%.2f s | s=%+.4f m v=%+.4f m/s | pitch=%+.3f deg | yaw fuso=%+.2f deg | "
                "delta SX=%+d DX=%+d | PWM richiesta SX=%+.2f DX=%+.2f applicata SX=%+.2f DX=%+.2f\n",
                (t0-start)*1e-9,state.x[0],state.x[3],psi,state.x[2]*180/3.141592653589793,
                static_cast<int>(enc_sx)-32768,static_cast<int>(enc_dx)-32768,
                command.raw[0],command.raw[1],applied[0],applied[1]);
            fprintf(stderr,"XY stimato=(%+.1f,%+.1f)cm | XY ideale=(0,%+.1f)cm\n",100*path.world_x(),100*path.world_y(),100*path.planned_y(elapsed));
        }
        std::printf("%ld,%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,%" PRIu64 ",%" PRIu64 ",%u,%u,"
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%d,%.9f,%.9f,%.9f,%d,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f",
            i,t0,s.accel_g,psi_acc,psi,gyro,(t1-t0)/1000,
            t0>due ? (t0-due)/1000 : static_cast<uint64_t>(0),enc_sx,enc_dx,
            state.x[0],state.x[3],state.x[2],state.x[5],command.raw[0],command.raw[1],
            command.limited[0],command.limited[1],applied[0],applied[1],
            command.common_raw,command.differential_raw,armed?1:0,
            reference.x[0],reference.x[2],encoder_valid?1:0,reference.x[3],reference.x[5],reference.x[1],
            reference.phase,motion_safe?1:0,reference.ff[0],reference.ff[1],
            yaw_encoder,yaw_encoder_rate,yaw_gyro_integral,gyro_x,gyro_z,yaw_rate,wheel_limit,differential_limit);
        std::printf(",%.9f,%.9f,%.9f,%d,%.9f,%.9f,%.9f,%.9f",path.world_x(),path.world_y(),std::sqrt(path.world_x()*path.world_x()+path.world_y()*path.world_y()),path.schedule_finished(elapsed)?1:0,0.,path.planned_y(elapsed),config.path_distance,config.path_turn);
        std::printf(",%u,%d",accel_guard.bad_count(),accel_trusted?1:0);
#ifdef TERASIC_HINF_PATH
        for(int j=0;j<6;++j)std::printf(",%.17g",hinf_controller.feedback[j]);
        for(int j=0;j<12;++j)std::printf(",%.17g",hinf_controller.xc[j]);
#endif
        std::printf("\n");
    }
    stop_motors();
#if !defined(TERASIC_GA_PATH)
    if(armed && i==count && ok && !stop_requested)
        fprintf(stderr,"ARRESTO AUTOMATICO: %.1f s dall'attivazione, profilo completato; motori spenti.\n",seconds+4.5);
#endif
    encoders_stop();
    watchdog_set(0);
    if (!ok) i2c_write(4, 0x41);
    i2c_write(2, 0);
#ifdef TERASIC_HINF_PATH
    fprintf(stderr, "Hinf PATH terminato; RUN SX=%u DX=%u.\n",
#else
    fprintf(stderr, "LQR terminato; RUN SX=%u DX=%u.\n",
#endif
            static_cast<unsigned>(reg_read(robot_hw::PWM_SX_CONTROL) & 1u),
            static_cast<unsigned>(reg_read(robot_hw::PWM_DX_CONTROL) & 1u));
    fprintf(stderr,"ESITO: profilo_temporale_terminato=%d; XY stimato=(%+.4f,%+.4f)m.\n",completion_announced?1:0,path.world_x(),path.world_y());
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
#ifdef TERASIC_HINF_PATH
    fprintf(stderr,"Uso: %s check config | shadow/run config durata_profilo_20..120 pwm_max_0..25\n",name);
#else
    fprintf(stderr,"Uso: %s check config | shadow/run config durata_profilo_20..120 pwm_max_0..50\n",name);
#endif
}
int main(int argc,char **argv)
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    const bool armed=argc>1 && !strcmp(argv[1],"run");
    const bool shadow=argc>1 && !strcmp(argv[1],"shadow");
    const bool check=argc>1 && !strcmp(argv[1],"check");
    double seconds=0;double max_pwm=0;LqrConfig config;
    if((check && (argc!=3 || !read_lqr_config(argv[2],&config))) ||
       ((armed || shadow) && (argc!=5 || !read_lqr_config(argv[2],&config) ||
          !parse_double(argv[3],20,120,&seconds) || !parse_double(argv[4],0,50,&max_pwm))) ||
       (!armed && !shadow && !check)) { usage(argv[0]);return 2; }
#ifdef TERASIC_HINF_PATH
    if((armed || shadow) && max_pwm>25){fprintf(stderr,"Hinf path: limite massimo25%%. Nessun accesso hardware.\n");return 2;}
    if((armed || shadow) && config.turn_yaw_reserve_percent>=pwm_limit(max_pwm,config.turn_wheel_pwm)) {
        fprintf(stderr,"Riserva yaw incompatibile con il limite PWM. Nessun accesso hardware.\n");return 2;
    }
#endif
    if (check) {
#ifdef TERASIC_HINF_PATH
        fprintf(stderr,"Configurazione Hinf PATH valida; tracking sperimentale da verificare. Nessun accesso hardware.\n");
#else
        fprintf(stderr,"Struttura configurazione valida; modello e K richiedono validazione fisica. Nessun accesso hardware.\n");
#endif
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
    setvbuf(stdout, NULL, _IOFBF, 65536);
    const int result = run(armed, armed || shadow, seconds, config, max_pwm);
    stop_motors();
    watchdog_set(0);
    fpga = NULL;
    munmap(mapped, span);
    close(fd);
    close(lock_fd);
    return result;
}
