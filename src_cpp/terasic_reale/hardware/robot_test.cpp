#define _FILE_OFFSET_BITS 64
#include "robot_wheel_map.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

// Registri e sequenze derivati dai diagnostici in diagnostics/.
// Questo programma non implementa alcun controllo del robot.
static volatile uint32_t *fpga = NULL;
static volatile sig_atomic_t stop_requested = 0;

static void on_signal(int) { stop_requested = 1; }

static uint32_t read_reg(unsigned offset) { return fpga[offset / 4]; }
static void write_reg(unsigned offset, uint32_t value) { fpga[offset / 4] = value; }
static unsigned i2c_read(unsigned index) { return read_reg(index * 4) & 0xffu; }
static void i2c_write(unsigned index, unsigned value) { write_reg(index * 4, value); }

static uint64_t monotonic_ns()
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) {
        perror("clock_gettime");
        return 0;
    }
    return static_cast<uint64_t>(t.tv_sec) * 1000000000u + t.tv_nsec;
}

static bool i2c_command(unsigned command, bool check_ack, const char *stage)
{
    i2c_write(4, command | 0x01); // IACK cancella il flag precedente.
    const uint64_t deadline = monotonic_ns() + 20000000u;
    do {
        unsigned status = i2c_read(4);
        if (status & 0x20) {
            fprintf(stderr, "%s: arbitraggio perso, SR=0x%02x\n", stage, status);
            return false;
        }
        if ((status & 0x01) && !(status & 0x02)) {
            if (check_ack && (status & 0x80)) {
                fprintf(stderr, "%s: NACK, SR=0x%02x\n", stage, status);
                return false;
            }
            return true;
        }
        usleep(50);
    } while (!stop_requested && monotonic_ns() < deadline);
    fprintf(stderr, "%s: timeout/interruzione, SR=0x%02x\n", stage, i2c_read(4));
    return false;
}

static bool bus_free()
{
    const uint64_t deadline = monotonic_ns() + 20000000u;
    do {
        if (!(i2c_read(4) & 0x40)) return true;
        usleep(50);
    } while (!stop_requested && monotonic_ns() < deadline);
    fprintf(stderr, "Bus I2C occupato, SR=0x%02x\n", i2c_read(4));
    return false;
}

static bool imu_read(unsigned address, unsigned char *data, unsigned length)
{
    i2c_write(3, 0x68 << 1);
    if (!i2c_command(0x90, true, "Indirizzo I2C scrittura")) return false;
    i2c_write(3, address);
    if (!i2c_command(0x10, true, "Registro IMU")) return false;
    i2c_write(3, (0x68 << 1) | 1);
    if (!i2c_command(0x90, true, "Indirizzo I2C lettura")) return false;
    for (unsigned i = 0; i < length; ++i) {
        if (!i2c_command(i + 1 == length ? 0x68 : 0x20, false, "Dato IMU"))
            return false;
        data[i] = static_cast<unsigned char>(i2c_read(3));
    }
    return bus_free();
}

static bool imu_write(unsigned address, unsigned value)
{
    i2c_write(3, 0x68 << 1);
    if (!i2c_command(0x90, true, "Indirizzo I2C scrittura")) return false;
    i2c_write(3, address);
    if (!i2c_command(0x10, true, "Registro IMU")) return false;
    i2c_write(3, value);
    return i2c_command(0x50, true, "Scrittura IMU") && bus_free();
}

static bool imu_setup()
{
    // Stessa configurazione verificata da imu_measure.cpp.
    i2c_write(2, 0);
    i2c_write(4, 1);
    i2c_write(0, 99); // 50 MHz / (5 * 100 kHz) - 1.
    i2c_write(1, 0);
    i2c_write(2, 0x80);

    unsigned char id = 0;
    if (!bus_free() || !imu_read(0x75, &id, 1) || id != 0x70) {
        fprintf(stderr, "MPU-6500 WHO_AM_I inatteso: 0x%02x\n", id);
        return false;
    }
    if (!imu_write(0x6B, 0x80)) return false; // Reset del solo sensore.
    usleep(100000);

    const unsigned addresses[] = {0x6B, 0x6C, 0x1B, 0x1C, 0x1A, 0x1D, 0x19};
    const unsigned values[] = {0x01, 0x00, 0x00, 0x00, 0x03, 0x03, 0x09};
    for (unsigned i = 0; i < 7; ++i)
        if (!imu_write(addresses[i], values[i])) return false;
    usleep(100000);
    for (unsigned i = 0; i < 7; ++i) {
        unsigned char value = 0;
        if (!imu_read(addresses[i], &value, 1) || value != values[i]) {
            fprintf(stderr, "Configurazione IMU non confermata: reg 0x%02x\n",
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

static bool motors_running()
{
    return (read_reg(robot_hw::PWM_SX_CONTROL) & 1u) || (read_reg(robot_hw::PWM_DX_CONTROL) & 1u);
}

static void encoders_disable()
{
    write_reg(robot_hw::ENCODER_SX_ENABLE, 0);
    write_reg(robot_hw::ENCODER_DX_ENABLE, 0);
}

static void encoders_start()
{
    encoders_disable();
    write_reg(robot_hw::ENCODER_SX_LOAD, 32768);
    write_reg(robot_hw::ENCODER_DX_LOAD, 32768);
    write_reg(robot_hw::ENCODER_SX_ENABLE, 1);
    write_reg(robot_hw::ENCODER_DX_ENABLE, 1);
}

static void motors_stop()
{
    // FAST_DECAY=1, FORWARD=1, RUN=0; come in motor_pulse.cpp.
    write_reg(robot_hw::PWM_SX_CONTROL, 0x06);
    write_reg(robot_hw::PWM_DX_CONTROL, 0x06);
    write_reg(robot_hw::PWM_SX_HIGH, 0);
    write_reg(robot_hw::PWM_DX_HIGH, 0);
}

static int test_sensors(long samples, long period_ms)
{
    if (motors_running()) {
        fprintf(stderr, "Acquisizione annullata: motore gia' in RUN.\n");
        return 1;
    }
    if (!imu_setup()) {
        i2c_write(4, 0x41); // Tenta STOP dopo transazione incompleta.
        i2c_write(2, 0);
        return 1;
    }
    encoders_start();
    fprintf(stderr, "IMU: raw accel +/-2g, gyro +/-250 deg/s; encoder raw 16 bit.\n");
    printf("sample,t_start_ns,t_imu_end_ns,t_enc_end_ns,late_us,imu_us,encoder_us,"
           "ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,enc_sx_raw,enc_dx_raw\n");
    fflush(stdout);

    bool ok = true;
    const uint64_t start = monotonic_ns();
    const uint64_t period = static_cast<uint64_t>(period_ms) * 1000000u;
    for (long i = 0; i < samples && !stop_requested; ++i) {
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
        if (sleep_result != 0) {
            fprintf(stderr, "clock_nanosleep: %s\n", strerror(sleep_result));
            ok = false;
            break;
        }
        if (motors_running()) {
            fprintf(stderr, "Motore entrato in RUN: acquisizione interrotta.\n");
            ok = false;
            break;
        }
        const uint64_t t0 = monotonic_ns();
        unsigned char data[14];
        if (!imu_read(0x3B, data, 14)) {
            ok = false;
            break;
        }
        const uint64_t t1 = monotonic_ns();
        const unsigned left = read_reg(robot_hw::ENCODER_SX_VALUE) & 0xffffu;
        const unsigned right = read_reg(robot_hw::ENCODER_DX_VALUE) & 0xffffu;
        const uint64_t t2 = monotonic_ns();
        const uint64_t late = t0 > due ? (t0 - due) / 1000 : 0;
        printf("%ld,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64
               ",%" PRIu64 ",%" PRIu64 ",%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
               i,
               t0, t1, t2, late, (t1 - t0) / 1000, (t2 - t1) / 1000,
               signed_word(data), signed_word(data + 2),
               signed_word(data + 4), signed_word(data + 6),
               signed_word(data + 8), signed_word(data + 10),
               signed_word(data + 12), left, right);
        fflush(stdout);
    }
    encoders_disable();
    if (!ok) i2c_write(4, 0x41);
    i2c_write(2, 0);
    fprintf(stderr, "Acquisizione terminata; encoder e controller I2C disabilitati.\n");
    return ok ? 0 : 1;
}

static int test_motor(const char *side, int direction, long percent)
{
    if (motors_running()) {
        fprintf(stderr, "Canale gia' in RUN: arresto e test annullato.\n");
        motors_stop();
        return 1;
    }
    motors_stop();
    encoders_start();
    const unsigned motor = strcmp(side, "sx") == 0 ? robot_hw::PWM_SX_PERIOD : robot_hw::PWM_DX_PERIOD;
    const uint32_t period = 50000;
    const uint32_t high = period * static_cast<unsigned>(percent) / 100;
    const unsigned control = 0x04 | (static_cast<unsigned>(direction) << 1);
    write_reg(motor, period);
    write_reg(motor + 8, control);
    write_reg(motor + 4, high);
    fprintf(stderr, "Impulso %s, direzione_bit=%d, PWM=%ld%%, massimo 300 ms.\n",
            side, direction, percent);
    if (!stop_requested) {
        const uint64_t deadline = monotonic_ns() + 300000000u;
        write_reg(motor + 8, control | 1);
        while (!stop_requested && monotonic_ns() < deadline) usleep(1000);
    }
    motors_stop();
    usleep(200000); // Include eventuale movimento residuo.
    encoders_disable();
    const int left = static_cast<int>(read_reg(robot_hw::ENCODER_SX_VALUE) & 0xffffu) - 32768;
    const int right = static_cast<int>(read_reg(robot_hw::ENCODER_DX_VALUE) & 0xffffu) - 32768;
    printf("encoder_delta_sx=%+d encoder_delta_dx=%+d control_sx=0x%02x control_dx=0x%02x\n",
           left, right, static_cast<unsigned>(read_reg(robot_hw::PWM_SX_CONTROL)),
           static_cast<unsigned>(read_reg(robot_hw::PWM_DX_CONTROL)));
    fflush(stdout);
    return (read_reg(robot_hw::PWM_SX_CONTROL) & 1u) || (read_reg(robot_hw::PWM_DX_CONTROL) & 1u) ? 1 : 0;
}

static bool parse_long(const char *text, long minimum, long maximum, long *value)
{
    char *end = NULL;
    errno = 0;
    long n = strtol(text, &end, 10);
    if (errno || end == text || *end || n < minimum || n > maximum) return false;
    *value = n;
    return true;
}

static void usage(const char *program)
{
    fprintf(stderr, "Uso: %s sensors [campioni 1..30000] [periodo_ms 1..1000]\n"
                    "     %s motor sx|dx 0|1 percentuale_1..20\n",
            program, program);
}

int main(int argc, char **argv)
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    bool sensors = argc >= 2 && strcmp(argv[1], "sensors") == 0;
    bool motor = argc >= 2 && strcmp(argv[1], "motor") == 0;
    long samples = 3000, period_ms = 10, direction = 0, percent = 0;
    if (sensors) {
        if (argc > 4 || (argc >= 3 && !parse_long(argv[2], 1, 30000, &samples)) ||
            (argc == 4 && !parse_long(argv[3], 1, 1000, &period_ms))) {
            usage(argv[0]); return 2;
        }
    } else if (motor) {
        if (argc != 5 || (strcmp(argv[2], "sx") && strcmp(argv[2], "dx")) ||
            !parse_long(argv[3], 0, 1, &direction) ||
            !parse_long(argv[4], 1, 20, &percent)) {
            usage(argv[0]); return 2;
        }
    } else { usage(argv[0]); return 2; }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    const size_t span = 0x1000;
    void *base = mmap(NULL, span, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                      static_cast<off_t>(0xFF200000u));
    if (base == MAP_FAILED) { perror("mmap"); close(fd); return 1; }
    fpga = static_cast<volatile uint32_t *>(base);
    int result = sensors ? test_sensors(samples, period_ms) :
                           test_motor(argv[2], static_cast<int>(direction), percent);
    munmap(base, span);
    close(fd);
    return result;
}
