#define _FILE_OFFSET_BITS 64
#include "robot_wheel_map.h"
#include <cstdio>
#include <stdint.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

// Standalone diagnostic, motors OFF. Does not write motor/IMU registers.
// Vendor references: BAL_CAR_HPS_CODE/Software/lib/fpga.cpp ADC_Ltc2308_Read,
// lib/car.cpp get_power_value; Hardware/soc_system.qsys: ADC offset 0x90.
// ADC channel 1, scale V_battery = raw * 18 / 4.7 / 1000.
// This is the nominal vendor scale, NOT a calibrated state-of-charge meter.
static volatile sig_atomic_t interrupted = 0;
static void stop(int) { interrupted = 1; }

static bool clock_ns(uint64_t *value)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        perror("clock_gettime");
        return false;
    }
    *value = static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
    return true;
}

static bool motors_off(volatile uint32_t *regs)
{
    return !(regs[robot_hw::PWM_SX_CONTROL/4] & 1u) && !(regs[robot_hw::PWM_DX_CONTROL/4] & 1u);
}

int main()
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    signal(SIGINT, stop);
    signal(SIGTERM, stop);
    signal(SIGHUP, stop);
    const size_t span = 0x1000;
    const int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    void *mapped = mmap(NULL, span, PROT_READ | PROT_WRITE, MAP_SHARED,
                        fd, static_cast<off_t>(0xFF200000ULL));
    if (mapped == MAP_FAILED) { perror("mmap"); close(fd); return 1; }
    volatile uint32_t *regs = static_cast<volatile uint32_t *>(mapped);
    int result = 0;
    unsigned low = 4095, high = 0, count = 0;
    double total = 0;
    for (unsigned sample = 0; sample < 16; ++sample) {
        if (interrupted || !motors_off(regs)) {
            fprintf(stderr, "Lettura annullata: interruzione o motore in RUN.\n");
            result = 1;
            break;
        }
        uint64_t start, now;
        if (!clock_ns(&start)) { result = 1; break; }
        regs[0x94 / 4] = 1; // one conversion, vendor sequence
        regs[0x90 / 4] = 2; // channel 1, start low
        regs[0x90 / 4] = 3; // start rising edge
        regs[0x90 / 4] = 2;
        bool ready = false;
        do {
            usleep(50);
            if (interrupted || !motors_off(regs)) break;
            if (regs[0x90 / 4] & 1u) { ready = true; break; }
            if (!clock_ns(&now)) break;
        } while (now - start < 20000000ULL);
        if (!ready) {
            fprintf(stderr, "ADC: timeout (20 ms) o acquisizione interrotta.\n");
            result = 1;
            break;
        }
        const unsigned raw = regs[0x94 / 4];
        if (raw == 0 || raw >= 4095) {
            fprintf(stderr, "ADC non valido/saturo: %u; tensione non disponibile.\n", raw);
            result = 1;
            break;
        }
        if (raw < low) low = raw;
        if (raw > high) high = raw;
        total += raw;
        ++count;
        usleep(20000);
    }
    if (interrupted || !motors_off(regs)) result = 1;
    munmap(mapped, span);
    close(fd);
    if (result != 0 || count != 16) return 1;
    const double factor = 18.0 / 4.7 / 1000.0;
    const double mean = total / count * factor;
    printf("ADC batteria: canale=1 campioni=%u raw_medio=%.2f raw_min=%u raw_max=%u\n",
           count, total / count, low, high);
    printf("Tensione pacco (scala nominale Terasic): %.3f V [min %.3f, max %.3f]\n",
           mean, low * factor, high * factor);
    printf("Percentuale di carica: non disponibile (nessun fuel gauge calibrato).\n");
    printf("Misura a motori spenti; confrontare una volta con un multimetro.\n");
    if (mean < 10.5) {
        printf("Tensione sotto 10.5 V: soglia di segnalazione della demo Terasic; sospendere prove motori e verificare il pacco.\n");
        return 2;
    }
    return 0;
}
