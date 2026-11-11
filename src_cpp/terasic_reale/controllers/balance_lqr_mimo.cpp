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
#include "balance_lqr_mimo_core.h"
#ifdef PWM_MOTION_COMPENSATION_EXPERIMENT
#ifdef PWM_LOW_COMMAND_EXPERIMENT
#include "../../../diagnostics/pwm_low_command_compensation.h"
typedef PwmLowCommandCompensation MotionExperiment;
#else
#ifdef PWM_MOTION_SMOOTH_EXPERIMENT
#include "../../../diagnostics/pwm_motion_smooth_compensation.h"
typedef PwmMotionCompensationSmooth MotionExperiment;
#else
#include "../../../diagnostics/pwm_motion_compensation.h"
typedef PwmMotionCompensation MotionExperiment;
#endif
#endif // PWM_LOW_COMMAND_EXPERIMENT
#include "../../../diagnostics/lqr_startup_assist.h"
#else
#ifdef LQR_STARTUP_ASSIST_EXPERIMENT
#ifdef PWM_GAP_COMPENSATION_EXPERIMENT
#ifdef PWM_INVERSE_COMPENSATION_EXPERIMENT
#ifdef PWM_INVERSE_FULL_EXPERIMENT
#include "../../../diagnostics/pwm_inverse_full.h"
#else
#include "../../../diagnostics/pwm_inverse_compensation.h"
#endif
#else
#include "../../../diagnostics/pwm_gap_compensation.h"
#endif
#else
#include "../../../diagnostics/lqr_startup_assist.h"
#endif
#endif
#endif // PWM_MOTION_COMPENSATION_EXPERIMENT
// Separate six-state/two-PWM executable. Legacy programs remain unchanged.

// LQR sperimentale separato dal PD. Hardware derivato dal PD verificato.
// Nessun guadagno predefinito: richiede modello e calibrazione encoder validati.
static volatile uint32_t *fpga = NULL;
static volatile sig_atomic_t stop_requested = 0;
static volatile sig_atomic_t watchdog_expired = 0;

static bool hardware_process_conflict()
{
    DIR *directory = opendir("/proc");
    if (!directory) { perror("opendir /proc"); return true; }
    const char *names[] = {"hinf_path_yaw", "hinf_path", "balance_hinf", "pitch_live", "pitch_pd", "pitch_pd_fixed", "balance_lqr", "balance_lqr_pos", "balance_lqr_2",
        "robot_test", "balance_car", "bal_car", "DE10_Nano_bal", "imu_measure",
        "imu_identity", "motor_pulse", "motor_freewheel", "lqr_assist", "hinf_assist", "lqr_gap", "hinf_gap", "path_simple", "lqr_endurance", "encoder_manual", "battery_status"};
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

static bool parse_long(const char *s, long min_value, long max_value, long *out)
{
    char *end = NULL;
    errno = 0;
    const long value = strtol(s, &end, 10);
    if (errno || end == s || *end || value < min_value || value > max_value)
        return false;
    *out = value;
    return true;
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
    double k[2][6],cpr,pitch_reference_deg,radius,track,max_diff,startup_limit;
    bool shadow_only;
    int gyro_sign,motor_sign,pitch_sign;
    LqrConfig() : cpr(0),pitch_reference_deg(0),radius(0),track(0),max_diff(0),startup_limit(5),
        shadow_only(true),gyro_sign(1),motor_sign(1),pitch_sign(1)
    { for(int i=0;i<2;++i)for(int j=0;j<6;++j)k[i][j]=0; }
};

static bool read_lqr_config(const char *path,LqrConfig *c)
{
    FILE *f=fopen(path,"r");if(!f){perror(path);return false;}
    const char *keys[]={"counts_per_rev","gyro_sign","motor_sign","encoder_sx_sign",
        "encoder_dx_sign","wheel_model_sign","pitch_model_sign","pitch_reference_deg",
        "shadow_only","wheel_radius_m","wheel_track_m","max_differential_pwm",
        "k_sx_position","k_sx_pitch","k_sx_yaw","k_sx_velocity","k_sx_pitch_rate","k_sx_yaw_rate",
        "k_dx_position","k_dx_pitch","k_dx_yaw","k_dx_velocity","k_dx_pitch_rate","k_dx_yaw_rate"};
    bool seen[24]={false},ok=true,startup_seen=false;char line[512];
    while(fgets(line,sizeof(line),f)) {
        char *comment=strchr(line,'#');if(comment)*comment=0;
        char key[80],value[160],extra[2];
        const int fields=sscanf(line," %79s %159s %1s",key,value,extra);
        if(fields==EOF)continue;
        if(fields!=2){ok=false;break;}
        if(!strcmp(key,"startup_max_error_deg")) {
            double limit=0;
            if(startup_seen || !parse_double(value,.01,BALANCE_STARTUP_MAX_ERROR_DEG,&limit)) {ok=false;break;}
            c->startup_limit=limit;startup_seen=true;continue;
        }
        int index=-1;for(int i=0;i<24;++i)if(!strcmp(key,keys[i]))index=i;
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
        else if(index==11){if(number<0 || number>10){ok=false;break;}c->max_diff=number;}
        else c->k[(index-12)/6][(index-12)%6]=number;
    }
    if(ferror(f))ok=false;
    fclose(f);
    for(int i=0;i<24;++i)if(!seen[i])ok=false;
    for(int i=0;i<2;++i)if(c->k[i][1]==0 && c->k[i][4]==0)ok=false;
    if(!ok)fprintf(stderr,"Configurazione MIMO incompleta/non valida: nessun avvio.\n");
    return ok;
}

static int run(bool armed, bool evaluate, long seconds, const LqrConfig &config, double max_pwm)
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
        if (s.angle_deg < angle_min) angle_min = s.angle_deg;
        if (s.angle_deg > angle_max) angle_max = s.angle_deg;
        usleep(10000);
    }
    const double initial_pitch = angle_sum / 200.0;
    const double angle_zero = config.pitch_reference_deg;
    const double initial_error = initial_pitch - angle_zero;
    const double gyro_bias = gyro_sum / 200.0;
    if (angle_max - angle_min > 3.0 || std::fabs(gyro_bias) > 10.0) {
        fprintf(stderr, "Calibrazione: variazione pitch=%.2f deg, "
                        "bias gyro Y=%.2f deg/s.\n",
                angle_max - angle_min, gyro_bias);
        ok = false;
    }
    if (stop_requested) ok = false;
    if (ok) fprintf(stderr,"Pitch iniziale assoluto=%.3f deg; riferimento FISSO=%.3f deg; errore iniziale=%+.3f deg.\n",
                    initial_pitch,angle_zero,initial_error);
    if (ok && armed && !balance_startup_pitch_ok(initial_pitch,angle_zero,config.startup_limit)) {
        fprintf(stderr,"Avvio annullato: errore iniziale oltre %.3f deg dal riferimento fisso. Motori spenti.\n",config.startup_limit);
        ok=false;
    }
    if (!ok) {
        fprintf(stderr, "Calibrazione annullata: tenere il robot piu' fermo e verticale.\n");
        i2c_write(2, 0);
        return 1;
    }
    fprintf(stderr, "Riferimento pitch=%.3f deg; bias gyro Y=%.3f deg/s.\n",
            angle_zero, gyro_bias);
    fprintf(stderr,"%s MIMO: unico K 2x6, due PWM; durata=%ld s limite=%.1f%% differenziale max=%.1f%%.\n",
            armed ? "ARMATO" : "SHADOW (motori spenti)",seconds,max_pwm,config.max_diff);
    fprintf(stderr,"Stati [s,psi,rho,v,psi_dot,rho_dot], SI; yaw solo encoder; zero dopo calibrazione.\n");
    for(int row=0;row<2;++row)
        fprintf(stderr,"K %s=[%.12g %.12g %.12g %.12g %.12g %.12g]\n",row==0 ? "SX" : "DX",
                config.k[row][0],config.k[row][1],config.k[row][2],config.k[row][3],config.k[row][4],config.k[row][5]);
    fprintf(stderr,"Raggio nominale=%.6f m carreggiata nominale=%.6f m CPR=%.3f\n",config.radius,config.track,config.cpr);
    fprintf(stderr,"Limite errore iniziale configurato=%.3f deg.\n",config.startup_limit);
    encoders_start();
    std::printf("sample,t_ns,accel_g,pitch_acc_deg,pitch_fused_deg,gyro_deg_s,imu_us,late_us,enc_sx_raw,enc_dx_raw,"
                "position_m,velocity_m_s,yaw_encoder_rad,yaw_rate_encoder_rad_s,"
                "pwm_sx_raw_percent,pwm_dx_raw_percent,pwm_sx_limited_percent,pwm_dx_limited_percent,"
                "pwm_sx_applied_percent,pwm_dx_applied_percent,pwm_common_raw_percent,pwm_differential_raw_percent,"
                "armed,position_reference_m,yaw_reference_rad,encoder_valid"
#ifdef LQR_STARTUP_ASSIST_EXPERIMENT
#ifdef PWM_GAP_COMPENSATION_EXPERIMENT
                ",pwm_sx_before_compensation_percent,pwm_dx_before_compensation_percent,pwm_sx_compensation_target_percent,pwm_dx_compensation_target_percent,compensation_sx_active,compensation_dx_active,compensation_pair_scale"
#else
                ",pwm_sx_before_assist_percent,pwm_dx_before_assist_percent,pwm_sx_assist_target_percent,pwm_dx_assist_target_percent,assist_sx_active,assist_dx_active,assist_pair_scale"
#endif
#endif
#ifdef PWM_MOTION_COMPENSATION_EXPERIMENT
                ",motion_sx_rad_s,motion_dx_rad_s,motion_sx_regime,motion_dx_regime,motion_sx_support,motion_dx_support"
#endif
                "\n");
#ifdef PWM_MOTION_COMPENSATION_EXPERIMENT
    MotionExperiment startup_assist;
#ifdef PWM_LOW_COMMAND_EXPERIMENT
    fprintf(stderr,"EXPERIMENT=low_command_inverse_startup gain=.2044017950318064 transition=8..10 identity_from=10 startup_max_s=.15 persistence_s=.03 release_counts=3 rearm=stationary_neutral floors_sx_dx=6.5,8.3 pair_correction_scaled=yes\n");
#else
#ifdef PWM_MOTION_SMOOTH_EXPERIMENT
    fprintf(stderr,"EXPERIMENT=motion_loss_smooth_v2 startup_enabled=no filter_s=.03 motion_enter=.18 motion_exit=.06 confidence=continuous correction_rate_up_down=100 LUT_unchanged=yes\n");
#else
    fprintf(stderr,"EXPERIMENT=motion_loss_compensation startup_enabled=no filter_s=.03 reference_voltage=11.2 correction_rate=100 delta_cap=none low_regularization=smoothstep high=constant_loss\n");
#endif // PWM_MOTION_SMOOTH_EXPERIMENT
#endif // PWM_LOW_COMMAND_EXPERIMENT
#else
#ifdef LQR_STARTUP_ASSIST_EXPERIMENT
#ifdef PWM_GAP_COMPENSATION_EXPERIMENT
#ifdef PWM_INVERSE_COMPENSATION_EXPERIMENT
#ifdef PWM_INVERSE_FULL_EXPERIMENT
    PwmInverseFull startup_assist;
    fprintf(stderr,"EXPERIMENT=full_inverse_linear_reference no_taper=yes no_delta_cap=yes reference_voltage=11.2 lower_upper_extensions=anchored_four_node_linear_fit zero=exact_zero physical_wheel_and_differential_caps_preserved=yes\n");
#else
    PwmInverseCompensation startup_assist;
    fprintf(stderr,"EXPERIMENT=inverse_static_model_matching reference_voltage=11.2 correction_cap_percent=1 smooth_taper=yes unsupported=identity zero=zero differential_allocator_cap_preserved=yes\n");
#endif
#else
    PwmGapCompensation startup_assist;
    fprintf(stderr,"EXPERIMENT=instantaneous_pwm_gap floors_sx_forward_reverse=5,5 floors_dx_forward_reverse=7,7 identity_from_percent=8 zero_maps_zero=yes persistence_s=0 timeout=none encoder_gate=none differential_allocator_cap_preserved=yes\n");
#endif
#else
    LqrStartupAssist startup_assist;
    fprintf(stderr,"EXPERIMENT=startup_assist floors_sx_forward_reverse=5,5 floors_dx_forward_reverse=7,7 persistence_s=.03 max_boost_s=.20 release_directed_counts=3 differential_cap_preserved=yes rearm=zero_or_direction_change\n");
#endif
#endif
#endif // PWM_MOTION_COMPENSATION_EXPERIMENT
    const uint64_t period = 10000000u; // 10 ms richiesti.
    const uint64_t start = now_ns();
    uint64_t previous = start;
    double psi = initial_error;
    const double origin_pitch=config.pitch_sign*initial_error*(3.141592653589793/180.0);
    LqrMimoEstimator encoder(config.cpr,config.radius,config.track,origin_pitch);
    double correlation = 0, previous_acc = 0;
    const long count = seconds * 100;
    for (long i = 0; i < count && !stop_requested; ++i) {
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
        psi = lqr_fuse_pitch_deg(psi, gyro, dt, psi_acc);
        if (s.accel_g < 0.6 || s.accel_g > 1.4 || s.az < 8000 ||
            std::fabs(psi) > 12.0 || std::fabs(psi_acc) > 15.0 ||
            std::fabs(gyro) > 180.0 || t1 - t0 > 8000000u) {
            fprintf(stderr, "Limite assetto/IMU/tempo superato: "
                            "|a|=%.3f g az=%d psi=%.2f deg "
                            "psi_acc=%.2f deg gyro=%.2f deg/s "
                            "imu=%" PRIu64 " us. Arresto.\n",
                    s.accel_g, s.az, psi, psi_acc, gyro,
                    (t1 - t0) / 1000);
            ok = false; break;
        }
        const unsigned enc_sx = reg_read(robot_hw::ENCODER_SX_VALUE) & 0xffffu;
        const unsigned enc_dx = reg_read(robot_hw::ENCODER_DX_VALUE) & 0xffffu;
        LqrMimoState state;
        const bool encoder_valid=encoder.update(enc_sx,enc_dx,dt,
            psi*(3.141592653589793/180.),gyro*(3.141592653589793/180.),&state);
        if(!encoder_valid){fprintf(stderr,"Encoder MIMO non valido/saturo. Arresto.\n");ok=false;break;}
        if(armed && (std::fabs(state.x[0])>.12 || std::fabs(state.x[3])>.30 ||
                     std::fabs(state.x[2])>.3490658504 || std::fabs(state.x[5])>1.5707963268)) {
            fprintf(stderr,"Limite posizione/velocita/direzione: s=%+.4f v=%+.4f rho=%+.3f rho_dot=%+.3f. Arresto.\n",
                    state.x[0],state.x[3],state.x[2],state.x[5]);ok=false;break;
        }
        const LqrMimoCommand command=lqr_mimo_command(state,config.k,config.motor_sign,max_pwm,config.max_diff);
        if(!lqr_finite(command.raw[0]) || !lqr_finite(command.raw[1])) {
            fprintf(stderr,"Comando MIMO non finito. Arresto.\n");ok=false;break;
        }
        double applied[2]={0,0};
#ifdef LQR_STARTUP_ASSIST_EXPERIMENT
#ifdef PWM_MOTION_COMPENSATION_EXPERIMENT
        MotionExperiment::Output motion_output={{0,0},{0,0},{0,0},{0,0},{0,0},1.,{0,0},{0,0},false};
#endif
        LqrAssistOutput assistance={{0,0},{0,0},{0,0},1.,{false,false}};
#endif
        if(armed && evaluate) {
            applied[0]=command.applied[0];applied[1]=command.applied[1];
#ifdef LQR_STARTUP_ASSIST_EXPERIMENT
            const unsigned counters[2]={enc_sx,enc_dx};
#ifdef PWM_MOTION_COMPENSATION_EXPERIMENT
            motion_output=startup_assist.update(command.limited,counters,(t0-start)*1e-9,max_pwm,config.max_diff);
            if(!motion_output.valid) {
                fprintf(stderr,"Motion compensator invalid encoder/timing; arresto.\n");ok=false;break;
            }
            for(unsigned w=0;w<2;++w) {
                assistance.nominal[w]=motion_output.nominal[w];assistance.target[w]=motion_output.target[w];
                assistance.applied[w]=motion_output.applied[w];assistance.active[w]=std::fabs(motion_output.correction[w])>1e-6;
            }
            assistance.pair_scale=motion_output.pair_scale;
#else
#ifdef PWM_GAP_COMPENSATION_EXPERIMENT
            // Use the unquantized allocator output: even requests below .5%
            // enter the new characteristic, rather than being lost beforehand.
            assistance=startup_assist.update(command.limited,counters,(t0-start)*1e-9,max_pwm,config.max_diff);
#else
            assistance=startup_assist.update(applied,counters,(t0-start)*1e-9,max_pwm,config.max_diff);
#endif
#endif // PWM_MOTION_COMPENSATION_EXPERIMENT
            applied[0]=assistance.applied[0];applied[1]=assistance.applied[1];
            set_duties(applied[0],applied[1]);
            const unsigned control_sx=reg_read(robot_hw::PWM_SX_CONTROL),control_dx=reg_read(robot_hw::PWM_DX_CONTROL);
            applied[0]=(control_sx&1u) ? (control_sx&2u ? 1. : -1.)*reg_read(robot_hw::PWM_SX_HIGH)/500. : 0.;
            applied[1]=(control_dx&1u) ? (control_dx&2u ? 1. : -1.)*reg_read(robot_hw::PWM_DX_HIGH)/500. : 0.;
#else
            set_duties(command.limited[0],command.limited[1]);
#endif
            if((std::fabs(applied[0])>=.5 && !(reg_read(robot_hw::PWM_SX_CONTROL)&1u)) ||
               (std::fabs(applied[1])>=.5 && !(reg_read(robot_hw::PWM_DX_CONTROL)&1u))) {
                fprintf(stderr,"RUN canale MIMO non confermato. Arresto.\n");ok=false;break;
            }
            watchdog_set(50000);
        }
        if(i%100==0)
            fprintf(stderr,"t=%.2f s | s=%+.4f m v=%+.4f m/s | pitch=%+.3f deg | yaw enc=%+.2f deg | "
                "delta SX=%+d DX=%+d | PWM richiesta SX=%+.2f DX=%+.2f applicata SX=%+.2f DX=%+.2f\n",
                (t0-start)*1e-9,state.x[0],state.x[3],psi,state.x[2]*180/3.141592653589793,
                static_cast<int>(enc_sx)-32768,static_cast<int>(enc_dx)-32768,
                command.raw[0],command.raw[1],applied[0],applied[1]);
        std::printf("%ld,%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,%" PRIu64 ",%" PRIu64 ",%u,%u,"
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,0,0,%d",
            i,t0,s.accel_g,psi_acc,psi,gyro,(t1-t0)/1000,
            t0>due ? (t0-due)/1000 : static_cast<uint64_t>(0),enc_sx,enc_dx,
            state.x[0],state.x[3],state.x[2],state.x[5],command.raw[0],command.raw[1],
            command.limited[0],command.limited[1],applied[0],applied[1],
            command.common_raw,command.differential_raw,armed?1:0,encoder_valid?1:0);
#ifdef LQR_STARTUP_ASSIST_EXPERIMENT
        std::printf(",%.9f,%.9f,%.9f,%.9f,%d,%d,%.9f",assistance.nominal[0],assistance.nominal[1],
            assistance.target[0],assistance.target[1],assistance.active[0]?1:0,assistance.active[1]?1:0,assistance.pair_scale);
#endif
#ifdef PWM_MOTION_COMPENSATION_EXPERIMENT
        std::printf(",%.9f,%.9f,%d,%d,%d,%d",motion_output.omega[0],motion_output.omega[1],
            motion_output.regime[0],motion_output.regime[1],motion_output.support[0],motion_output.support[1]);
#endif
        std::printf("\n");
    }
    stop_motors();
    encoders_stop();
    watchdog_set(0);
    if (!ok) i2c_write(4, 0x41);
    i2c_write(2, 0);
    fprintf(stderr, "LQR terminato; RUN SX=%u DX=%u.\n",
            static_cast<unsigned>(reg_read(robot_hw::PWM_SX_CONTROL) & 1u),
            static_cast<unsigned>(reg_read(robot_hw::PWM_DX_CONTROL) & 1u));
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
    fprintf(stderr,"Uso: %s check config | shadow/run config secondi_1..30 max_pwm_0..100\n",name);
}
int main(int argc,char **argv)
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    const bool armed=argc>1 && !strcmp(argv[1],"run");
    const bool shadow=argc>1 && !strcmp(argv[1],"shadow");
    const bool check=argc>1 && !strcmp(argv[1],"check");
    long seconds=0;double max_pwm=0;LqrConfig config;
    if((check && (argc!=3 || !read_lqr_config(argv[2],&config))) ||
       ((armed || shadow) && (argc!=5 || !read_lqr_config(argv[2],&config) ||
          !parse_long(argv[3],1,30,&seconds) || !parse_double(argv[4],0,100,&max_pwm))) ||
       (!armed && !shadow && !check)) { usage(argv[0]);return 2; }
    if (check) {
        fprintf(stderr,"Limite errore iniziale configurato=%.3f deg.\n",config.startup_limit);
        fprintf(stderr,"Struttura configurazione valida; modello e K richiedono validazione fisica. Nessun accesso hardware.\n");
        if (config.shadow_only) fprintf(stderr,"Configurazione riservata a SHADOW: run disabilitato.\n");
        return 0;
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
