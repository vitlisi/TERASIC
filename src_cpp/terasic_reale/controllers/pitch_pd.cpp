#define _FILE_OFFSET_BITS 64
#define __STDC_FORMAT_MACROS
#include "balance_startup_limits.h"
#include "../hardware/robot_wheel_map.h"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#include <inttypes.h>
#include <fcntl.h>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include "pitch_pd_core.h"
#ifdef TERASIC_PD_OUTER_LOOP
#include "pitch_pd_outer_core.h"
static double outer_kx=0,outer_kv=0;
#endif
#ifdef TERASIC_PD_FIXED_REFERENCE
#include <dirent.h>
static double fixed_pitch_reference_deg = 0;
static bool fixed_reference_process_conflict()
{
    DIR *directory = opendir("/proc");
    if (!directory) { perror("opendir /proc"); return true; }
    const char *names[] = {"hinf_path_yaw", "hinf_path", "balance_hinf", "pitch_live", "pitch_pd", "pitch_pd_fixed", "pitch_pd_outer", "balance_lqr",
                          "robot_test", "balance_car", "bal_car", "DE10_Nano_bal",
                          "imu_measure", "imu_identity", "motor_pulse"};
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
        for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
            if (std::strcmp(name, names[i]) == 0) {
                fprintf(stderr, "Programma hardware attivo: %s (PID %ld). Avvio rifiutato.\n", name, pid);
                conflict = true; break;
            }
        }
        if (conflict) break;
    }
    closedir(directory);
    return conflict;
}
#endif

// Primo PD sperimentale del pitch per il bitstream usato da diagnostics/.
// Nessun servizio di rete, controllo velocita' o riavvio automatico dei motori.
static volatile uint32_t *fpga = NULL;
static volatile sig_atomic_t stop_requested = 0;
static volatile sig_atomic_t watchdog_expired = 0;

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
    int ax, ay, az, gy;
    double angle_deg, gyro_deg_s, accel_g;
};

static bool get_sample(Sample *sample)
{
    unsigned char data[14];
    if (!imu_read(0x3B, data, 14)) return false;
    sample->ax = signed_word(data);
    sample->ay = signed_word(data + 2);
    sample->az = signed_word(data + 4);
    sample->gy = signed_word(data + 10);
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

static void set_duty(double signed_percent)
{
    if (stop_requested) { stop_motors(); return; }
    if (std::fabs(signed_percent) < 0.5) { stop_motors(); return; }
    const uint32_t period = 50000;
    const uint32_t high = pitch_pd_high_ticks(signed_percent);
    const uint32_t control = 0x05u | (signed_percent > 0 ? 0x02u : 0u);
    // Interrompe RUN prima di cambiare direzione; i due canali sono speculari.
    if ((reg_read(robot_hw::PWM_SX_CONTROL) & 0x02u) != (control & 0x02u) ||
        (reg_read(robot_hw::PWM_DX_CONTROL) & 0x02u) != (control & 0x02u)) stop_motors();
    reg_write(robot_hw::PWM_SX_PERIOD, period);
    reg_write(robot_hw::PWM_DX_PERIOD, period);
    reg_write(robot_hw::PWM_SX_HIGH, high);
    reg_write(robot_hw::PWM_DX_HIGH, high);
    reg_write(robot_hw::PWM_SX_CONTROL, control);
    reg_write(robot_hw::PWM_DX_CONTROL, control);
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

static int run(bool armed, long seconds, int gyro_sign, int motor_sign,
               double kp, double kd, double max_pwm)
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

    // Fixed-reference variant must accept the supported tilt, not zero it.
#ifdef TERASIC_PD_FIXED_REFERENCE
    const double calibration_reference = fixed_pitch_reference_deg;
#else
    const double calibration_reference = 0; // Legacy PD intentionally zeros pose.
#endif
    // Calibrazione a motori spenti: tenere il robot fermo e verticale.
    fprintf(stderr, "Calibrazione IMU: tenere il robot fermo e verticale per 2 s.\n");
    double angle_sum = 0, gyro_sum = 0, angle_min = 1e9, angle_max = -1e9;
    bool ok = true;
    for (int i = 0; i < 200 && !stop_requested; ++i) {
        Sample s;
        if (!get_sample(&s)) {
            fprintf(stderr, "Calibrazione: lettura IMU fallita al campione %d.\n", i);
            ok = false; break;
        }
        if (s.accel_g < 0.75 || s.accel_g > 1.25 || s.az < 11000 ||
            !balance_startup_pitch_ok(s.angle_deg,calibration_reference)) {
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
#ifdef TERASIC_PD_FIXED_REFERENCE
    const double initial_pitch_deg = angle_sum / 200.0;
    const double angle_zero = fixed_pitch_reference_deg;
#else
    const double angle_zero = angle_sum / 200.0;
#endif
    const double gyro_bias = gyro_sum / 200.0;
    if (angle_max - angle_min > 3.0 || std::fabs(gyro_bias) > 10.0) {
        fprintf(stderr, "Calibrazione: variazione pitch=%.2f deg, "
                        "bias gyro Y=%.2f deg/s.\n",
                angle_max - angle_min, gyro_bias);
        ok = false;
    }
    if (stop_requested) ok = false;
    if (!ok) {
        fprintf(stderr, "Calibrazione annullata: tenere il robot piu' fermo e verticale.\n");
        i2c_write(2, 0);
        return 1;
    }
#ifdef TERASIC_PD_FIXED_REFERENCE
    fprintf(stderr, "Pitch iniziale assoluto=%.3f deg; riferimento FISSO=%.3f deg; errore iniziale=%+.3f deg.\n",
            initial_pitch_deg, angle_zero, initial_pitch_deg - angle_zero);
    if (armed && !balance_startup_pitch_ok(initial_pitch_deg,angle_zero)) {
        fprintf(stderr, "Avvio annullato: errore iniziale oltre 5 deg dal riferimento fisso. Motori spenti.\n");
        i2c_write(2, 0);
        return 1;
    }
#endif
    fprintf(stderr, "Riferimento pitch=%.3f deg; bias gyro Y=%.3f deg/s.\n",
            angle_zero, gyro_bias);
    if (armed) fprintf(stderr, "PD ARMATO: sign_gyro=%+d sign_motore=%+d "
                              "Kp=%.3f Kd=%.3f limite=%.1f%% durata=%ld s.\n",
                       gyro_sign, motor_sign, kp, kd, max_pwm, seconds);
    else fprintf(stderr, "Solo osservazione: i motori restano spenti.\n");

    encoders_start();
    std::printf("sample,t_ns,ax_raw,ay_raw,az_raw,gy_raw,accel_g,"
                "theta_acc_deg,theta_fused_deg,gyro_deg_s,pwm_percent,late_us,imu_us,"
                "enc_sx_raw,enc_dx_raw"
#ifdef TERASIC_PD_OUTER_LOOP
                ",position_m,velocity_m_s,pitch_target_deg,outer_offset_deg"
#endif
                "\n");
    const uint64_t period = 10000000u; // 10 ms richiesti.
    const uint64_t start = now_ns();
    uint64_t previous = start;
#ifdef TERASIC_PD_FIXED_REFERENCE
    double theta = initial_pitch_deg - angle_zero;
#else
    double theta = 0;
#endif
    double correlation = 0, previous_acc = 0;
#ifdef TERASIC_PD_OUTER_LOOP
    PitchPdOuterEstimator outer_estimator(initial_pitch_deg-angle_zero);
    fprintf(stderr,"PD outer: Kx=%.3f deg/m Kv=%.3f deg/(m/s); target offset clipped to +/-3 deg; CPR=1325 radius=0.032 m.\n",outer_kx,outer_kv);
#endif
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
        const double theta_acc = s.angle_deg - angle_zero;
        const double gyro = gyro_sign * (s.gyro_deg_s - gyro_bias);
        if (i) correlation += (theta_acc - previous_acc) *
                               (s.gyro_deg_s - gyro_bias);
        previous_acc = theta_acc;
        theta = pitch_pd_fuse(theta, gyro, dt, theta_acc);
        if (s.accel_g < 0.6 || s.accel_g > 1.4 || s.az < 8000 ||
            std::fabs(theta) > 12.0 || std::fabs(theta_acc) > 15.0 ||
            std::fabs(gyro) > 180.0 || t1 - t0 > 8000000u) {
            fprintf(stderr, "Limite assetto/IMU/tempo superato: "
                            "|a|=%.3f g az=%d theta=%.2f deg "
                            "theta_acc=%.2f deg gyro=%.2f deg/s "
                            "imu=%" PRIu64 " us. Arresto.\n",
                    s.accel_g, s.az, theta, theta_acc, gyro,
                    (t1 - t0) / 1000);
            ok = false; break;
        }
        double duty = 0;
#ifdef TERASIC_PD_OUTER_LOOP
        const unsigned enc_sx = reg_read(robot_hw::ENCODER_SX_VALUE) & 0xffffu;
        const unsigned enc_dx = reg_read(robot_hw::ENCODER_DX_VALUE) & 0xffffu;
        double position_m=0,velocity_m_s=0;
        if(!outer_estimator.update(enc_sx,enc_dx,dt,theta,gyro,&position_m,&velocity_m_s)) {
            fprintf(stderr,"Encoder/odometria non validi: arresto.\n"); ok=false; break;
        }
        const double pitch_target_deg=pitch_pd_outer_target(position_m,velocity_m_s,outer_kx,outer_kv);
#endif
        if (armed) {
#ifdef TERASIC_PD_OUTER_LOOP
            duty = pitch_pd_command(theta-pitch_target_deg, gyro, kp, kd, motor_sign, max_pwm);
#else
            duty = pitch_pd_command(theta, gyro, kp, kd, motor_sign, max_pwm);
#endif
            set_duty(duty);
            if (!motors_running() && std::fabs(duty) >= 0.5) {
                fprintf(stderr, "RUN motore non confermato: arresto.\n");
                ok = false; break;
            }
            watchdog_set(50000);
        }
#ifdef TERASIC_PD_OUTER_LOOP
        const unsigned enc_sx_log = enc_sx, enc_dx_log = enc_dx;
#else
        const unsigned enc_sx_log = reg_read(robot_hw::ENCODER_SX_VALUE) & 0xffffu;
        const unsigned enc_dx_log = reg_read(robot_hw::ENCODER_DX_VALUE) & 0xffffu;
#endif
        std::printf("%ld,%" PRIu64 ",%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.3f,%" PRIu64 ",%" PRIu64 ",%u,%u",
                    i, t0, s.ax, s.ay, s.az, s.gy, s.accel_g,
                    theta_acc, theta, gyro, duty,
                    t0 > due ? (t0 - due) / 1000 : static_cast<uint64_t>(0),
                    (t1 - t0) / 1000, enc_sx_log, enc_dx_log);
#ifdef TERASIC_PD_OUTER_LOOP
        std::printf(",%.6f,%.6f,%.6f,%.6f",position_m,velocity_m_s,angle_zero+pitch_target_deg,pitch_target_deg);
#endif
        std::printf("\n");
    }
    stop_motors();
    encoders_stop();
    watchdog_set(0);
    if (!ok) i2c_write(4, 0x41);
    i2c_write(2, 0);
    fprintf(stderr, "PD terminato; RUN SX=%u DX=%u.\n",
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
#ifdef TERASIC_PD_FIXED_REFERENCE
#ifdef TERASIC_PD_OUTER_LOOP
    fprintf(stderr, "Uso PD outer (riferimento fisso; Kx deg/m, Kv deg/(m/s)):\n"
                    "  %s observe secondi_1..30 riferimento_-5..5\n"
                    "  %s run gyro_sign_-1|1 motor_sign_-1|1 "
                    "kp_0..5 kd_0..1 max_pwm_1..35 secondi_1..30 riferimento_-5..5 kx_0..10 kv_0..20\n",
            name, name);
#else
    fprintf(stderr, "Uso (riferimento assoluto in gradi, obbligatorio):\n"
                    "  %s observe secondi_1..30 riferimento_-5..5\n"
                    "  %s run gyro_sign_-1|1 motor_sign_-1|1 "
                    "kp_0..5 kd_0..1 max_pwm_1..35 secondi_1..30 riferimento_-5..5\n",
            name, name);
#endif
#else
    fprintf(stderr, "Uso:\n  %s observe secondi_1..30\n"
                    "  %s run gyro_sign_-1|1 motor_sign_-1|1 "
                    "kp_0..5 kd_0..1 max_pwm_1..35 secondi_1..30\n",
            name, name);
#endif
}

int main(int argc, char **argv)
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
#ifdef TERASIC_PD_FIXED_REFERENCE
    if (argc == 2 && std::strcmp(argv[1], "--help") == 0) { usage(argv[0]); return 0; }
#endif
    bool armed = argc > 1 && std::strcmp(argv[1], "run") == 0;
    bool observe = argc > 1 && std::strcmp(argv[1], "observe") == 0;
    long seconds = 0, gyro_sign = 1, motor_sign = 1;
    double kp = 0, kd = 0, max_pwm = 0;
    if ((observe && (
#ifdef TERASIC_PD_FIXED_REFERENCE
                    argc != 4 || !parse_double(argv[3], -5, 5, &fixed_pitch_reference_deg) ||
#else
                    argc != 3 ||
#endif
                    !parse_long(argv[2], 1, 30, &seconds))) ||
        (armed && (
#ifdef TERASIC_PD_FIXED_REFERENCE
                   argc !=
#ifdef TERASIC_PD_OUTER_LOOP
                   11 || !parse_double(argv[9], 0, 10, &outer_kx) ||
                   !parse_double(argv[10], 0, 20, &outer_kv) ||
#else
                   9 ||
#endif
                   !parse_double(argv[8], -5, 5, &fixed_pitch_reference_deg) ||
#else
                   argc != 8 ||
#endif
                   !parse_long(argv[2], -1, 1, &gyro_sign) ||
                   !parse_long(argv[3], -1, 1, &motor_sign) ||
                   (gyro_sign != -1 && gyro_sign != 1) ||
                   (motor_sign != -1 && motor_sign != 1) ||
                   !parse_double(argv[4], 0, 5, &kp) ||
                   !parse_double(argv[5], 0, 1, &kd) ||
                   !parse_double(argv[6], 1, 35, &max_pwm) ||
                   !parse_long(argv[7], 1, 30, &seconds))) ||
        (!observe && !armed)) {
        usage(argv[0]); return 2;
    }
#ifdef TERASIC_PD_FIXED_REFERENCE
    if (fixed_reference_process_conflict()) return 1;
#endif
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
    const int result = run(armed, seconds, static_cast<int>(gyro_sign),
                           static_cast<int>(motor_sign), kp, kd, max_pwm);
    stop_motors();
    watchdog_set(0);
    fpga = NULL;
    munmap(mapped, span);
    close(fd);
    return result;
}
