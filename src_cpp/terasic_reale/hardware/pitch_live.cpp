#define _FILE_OFFSET_BITS 64
#include "robot_wheel_map.h"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>

// Absolute pitch display ONLY. Never writes motor or encoder registers.
// Sensor setup and I2C transactions match the existing pitch_pd diagnostic.
static volatile uint32_t *fpga = NULL;
static volatile sig_atomic_t stop_requested = 0;
static void handle_stop(int) { stop_requested = 1; }
static uint32_t reg_read(unsigned offset) { return fpga[offset / 4]; }
static unsigned i2c_read(unsigned index) { return fpga[index] & 0xffu; }
static void i2c_write(unsigned index, unsigned value) { fpga[index] = value; }

// I2C/MPU-6500 sequence copied from the validated PD; no control code.
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

static bool conflicting_process()
{
    DIR *directory = opendir("/proc");
    if (!directory) { perror("opendir /proc"); return true; }
    const char *names[] = {"pitch_pd", "pitch_pd_fixed", "balance_lqr_2", "balance_hinf",
                          "path_simple", "lqr_endurance", "ultrasonic_live", "battery_status", "balance_lqr", "robot_test", "balance_car",
                          "bal_car", "DE10_Nano_bal", "imu_measure", "imu_identity",
                          "motor_pulse", "pitch_live"};
    struct dirent *entry;
    bool conflict = false;
    while ((entry = readdir(directory)) != NULL) {
        char *end = NULL;
        const long pid = strtol(entry->d_name, &end, 10);
        if (pid <= 0 || *end || pid == static_cast<long>(getpid())) continue;
        char path[128];
        std::snprintf(path, sizeof(path), "/proc/%ld/comm", pid);
        FILE *file = std::fopen(path, "r");
        if (!file) continue;
        char name[64] = {0};
        const bool read_ok = std::fgets(name, sizeof(name), file) != NULL;
        std::fclose(file);
        if (!read_ok) continue;
        name[std::strcspn(name, "\r\n")] = 0;
        for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
            if (std::strcmp(name, names[i]) == 0) {
                fprintf(stderr, "Programma hardware attivo: %s (PID %ld). Chiuderlo prima.\n", name, pid);
                conflict = true;
                break;
            }
        }
        if (conflict) break;
    }
    closedir(directory);
    return conflict;
}

struct Window {
    double angle[100], gyro[100], norm[100];
    unsigned count, next;
    double mean, deviation, span, gyro_rms;
    bool steady;
    Window() : count(0), next(0), mean(0), deviation(0), span(0), gyro_rms(0), steady(false) {}
    void update(double a, double g, double n) {
        angle[next] = a; gyro[next] = g; norm[next] = n;
        next = (next + 1) % 100;
        if (count < 100) ++count;
        double sum = 0, sum_g2 = 0, low = angle[0], high = angle[0];
        bool plausible = true;
        for (unsigned i = 0; i < count; ++i) {
            sum += angle[i]; sum_g2 += gyro[i] * gyro[i];
            if (angle[i] < low) low = angle[i];
            if (angle[i] > high) high = angle[i];
            if (norm[i] < 0.85 || norm[i] > 1.15) plausible = false;
        }
        mean = sum / count;
        double variance = 0;
        for (unsigned i = 0; i < count; ++i) {
            const double delta = angle[i] - mean;
            variance += delta * delta;
        }
        deviation = std::sqrt(variance / count);
        span = high - low;
        gyro_rms = std::sqrt(sum_g2 / count);
        steady = count == 100 && plausible && deviation <= 0.25 && span <= 1.0 && gyro_rms <= 1.5;
    }
};

static int display(long seconds, FILE *csv)
{
    if (motors_running()) { fprintf(stderr, "Motore in RUN: lettura rifiutata.\n"); return 1; }
    if (!imu_setup()) return 1;
    printf("Calibro SOLO il bias gyro per circa 2 s: tenere il corpo fermo.\n"
           "Il pitch assoluto NON viene azzerato. Motori non comandati.\n");
    fflush(stdout);
    double pitch_sum = 0;
    double sum = 0, low_a = 1e9, high_a = -1e9, low_g = 1e9, high_g = -1e9;
    for (unsigned i = 0; i < 100; ++i) {
        Sample s;
        if (stop_requested || motors_running() || !get_sample(&s)) return 1;
        if (s.accel_g < 0.85 || s.accel_g > 1.15 || s.az <= 0) {
            fprintf(stderr, "Assetto/accelerazione non plausibile per calibrare il gyro.\n"); return 1;
        }
        pitch_sum += s.angle_deg;
        sum += s.gyro_deg_s;
        if (s.angle_deg < low_a) low_a = s.angle_deg;
        if (s.angle_deg > high_a) high_a = s.angle_deg;
        if (s.gyro_deg_s < low_g) low_g = s.gyro_deg_s;
        if (s.gyro_deg_s > high_g) high_g = s.gyro_deg_s;
        usleep(20000);
    }
    const double bias = sum / 100;
    printf("Calibrazione: pitch medio assoluto=%+.4f deg; min=%+.4f max=%+.4f; span=%.4f deg; gyro bias=%+.4f deg/s span=%.4f deg/s.\n",
           pitch_sum/100,low_a,high_a,high_a-low_a,bias,high_g-low_g);
    fprintf(csv,"# calibration_pitch_mean_deg=%.6f; min_deg=%.6f; max_deg=%.6f; gyro_bias_deg_s=%.6f; gyro_span_deg_s=%.6f\n",
            pitch_sum/100,low_a,high_a,bias,high_g-low_g);
    if (high_a - low_a > 1.0 || high_g - low_g > 3.0 || std::fabs(bias) > 10.0) {
        fprintf(stderr, "Corpo mosso durante calibrazione gyro: riprovare piu' fermi.\n"); return 1;
    }
    printf("Bias gyro Y=%+.4f deg/s. Campionamento 50 Hz, display 5 Hz.\n"
           "FERMO indica solo quiete del sensore, NON equilibrio meccanico.\n"
           "Tenere il corpo sempre sostenuto. Ctrl+C per terminare.\n", bias);
    fflush(stdout);
    fprintf(csv, "# gyro_bias_deg_s=%.6f; pitch_abs=atan2(-ax,az); no_pitch_zero\n", bias);
    fprintf(csv, "t_s,pitch_abs_deg,gyro_corrected_deg_s,accel_g,mean_2s_deg,std_2s_deg,span_2s_deg,steady\n");
    const bool terminal = isatty(STDOUT_FILENO);
    const uint64_t start = now_ns();
    uint64_t deadline = start;
    Window window;
    unsigned sample = 0;
    int result = 0;
    while (!stop_requested) {
        const uint64_t now = now_ns();
        if (stop_requested || (seconds && now - start >= static_cast<uint64_t>(seconds) * 1000000000ULL)) break;
        if (motors_running() || (sample % 50 == 0 && conflicting_process())) {
            fprintf(stderr, "Programma concorrente o motore in RUN: lettura interrotta.\n"); result = 1; break;
        }
        Sample s;
        if (!get_sample(&s) || s.az <= 0 || s.accel_g < 0.6 || s.accel_g > 1.4) {
            fprintf(stderr, "IMU non plausibile/lettura fallita: diagnostico terminato.\n"); result = 1; break;
        }
        const double gyro = s.gyro_deg_s - bias;
        window.update(s.angle_deg, gyro, s.accel_g);
        fprintf(csv, "%.6f,%.6f,%.6f,%.5f,%.6f,%.6f,%.6f,%u\n",
                (now - start) * 1e-9, s.angle_deg, gyro, s.accel_g,
                window.mean, window.deviation, window.span, window.steady ? 1u : 0u);
        if (sample % 10 == 0) {
            printf("%sPitch=%+7.3f deg | MEDIA ~2s=%+7.3f | std=%.3f | gyro=%+6.2f deg/s | %s   %s",
                   terminal ? "\r" : "", s.angle_deg, window.mean, window.deviation, gyro,
                   window.count < 100 ? "ATTENDERE" : window.steady ? "FERMO" : "MOVIMENTO",
                   terminal ? "" : "\n");
            fflush(stdout); fflush(csv);
        }
        ++sample;
        deadline += 20000000ULL;
        const uint64_t after = now_ns();
        // Reset a missed deadline: the display never runs a burst to catch up.
        if (after > deadline) deadline = after + 20000000ULL;
        struct timespec target;
        target.tv_sec = deadline / 1000000000ULL;
        target.tv_nsec = deadline % 1000000000ULL;
        const int sleep_result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, NULL);
        if (sleep_result != 0 && sleep_result != EINTR) {
            fprintf(stderr, "clock_nanosleep: %s\n", std::strerror(sleep_result)); result = 1; break;
        }
    }
    printf("\nDiagnostico terminato.\n");
    if (result == 0 && window.steady)
        printf("Ultima finestra FERMA: media pitch assoluto=%+.4f deg; std=%.4f deg.\n"
               "Riferimento candidato scelto dall'operatore; equilibrio da verificare.\n", window.mean, window.deviation);
    else printf("Nessun riferimento finale suggerito: finestra incompleta, movimento o errore.\n");
    return result;
}

int main(int argc, char **argv)
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    long seconds = 0;
    if (argc == 2 && std::strcmp(argv[1], "--help") == 0) {
        printf("Uso: pitch_live [secondi_0..3600]; 0 = fino a Ctrl+C. Nessun comando motori.\n"); return 0;
    }
    if (argc > 2) { fprintf(stderr, "Uso: pitch_live [secondi_0..3600]\n"); return 2; }
    if (argc == 2) {
        char *end = NULL;
        errno = 0;
        seconds = strtol(argv[1], &end, 10);
        if (errno || end == argv[1] || *end || seconds < 0 || seconds > 3600) return 2;
    }
    if (conflicting_process()) return 1;
    char path[256];
    std::snprintf(path, sizeof(path), "logs/terasic_reale/runs/diagnostics/pitch_live_%ld_%ld.csv",
                  static_cast<long>(time(NULL)), static_cast<long>(getpid()));
    const int log_fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (log_fd < 0) { perror("Creazione CSV (avviare dalla cartella del progetto)"); return 1; }
    FILE *csv = fdopen(log_fd, "w");
    if (!csv) { perror("fdopen"); close(log_fd); return 1; }
    printf("CSV: %s\n", path); fflush(stdout);
    const int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); fclose(csv); return 1; }
    const size_t span = 0x1000;
    void *mapped = mmap(NULL, span, PROT_READ | PROT_WRITE, MAP_SHARED,
                        fd, static_cast<off_t>(0xFF200000ULL));
    if (mapped == MAP_FAILED) { perror("mmap"); close(fd); fclose(csv); return 1; }
    fpga = static_cast<volatile uint32_t *>(mapped);
    signal(SIGINT, handle_stop); signal(SIGTERM, handle_stop); signal(SIGHUP, handle_stop);
    const int result = display(seconds, csv);
    // I2C only; no motor/encoder control writes, including signal cleanup.
    if (!motors_running()) { i2c_write(4, 0x41); i2c_write(2, 0); }
    munmap(mapped, span); close(fd);
    const int closed = fclose(csv);
    if (closed != 0) perror("Chiusura CSV");
    return closed != 0 ? 1 : result;
}
