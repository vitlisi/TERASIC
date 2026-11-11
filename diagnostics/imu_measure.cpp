#include "imu_bus.h"
#include <cmath>
#include <signal.h>

static volatile sig_atomic_t stop_requested = 0;

static void request_stop(int)
{
    stop_requested = 1;
}

static bool sensor_write(unsigned reg, unsigned value)
{
    wr(3, 0x68 << 1);
    if (!command(0x90, true, "Indirizzo scrittura"))
        return false;

    wr(3, reg);
    if (!command(0x10, true, "Selezione registro"))
        return false;

    wr(3, value);
    if (!command(0x50, true, "Scrittura e STOP"))
        return false;

    return wait_bus_free();
}

static bool sensor_read(unsigned reg, unsigned char *data,
                        unsigned length)
{
    wr(3, 0x68 << 1);
    if (!command(0x90, true, "Indirizzo scrittura"))
        return false;

    wr(3, reg);
    if (!command(0x10, true, "Selezione registro"))
        return false;

    wr(3, (0x68 << 1) | 1);
    if (!command(0x90, true, "Indirizzo lettura"))
        return false;

    for (unsigned i = 0; i < length; ++i) {
        unsigned cmd = (i + 1 == length) ? 0x68 : 0x20;
        if (!command(cmd, false, "Lettura dati"))
            return false;
        data[i] = static_cast<unsigned char>(rd(3));
    }

    return wait_bus_free();
}

static int signed_word(const unsigned char *p)
{
    int value = (static_cast<int>(p[0]) << 8) | p[1];
    return (value >= 32768) ? value - 65536 : value;
}

int main()
{
    const size_t span = 0x1000;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

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

    // Controlla RUN senza modificare i registri dei motori.
    if ((rd(0x78 / 4) & 1) || (rd(0x68 / 4) & 1)) {
        fprintf(stderr, "Test annullato: un motore ha RUN=1.\n");
        munmap(base, span);
        close(fd);
        return 1;
    }

    wr(2, 0);
    wr(4, 1);
    wr(0, 99);
    wr(1, 0);
    wr(2, 0x80);

    unsigned identity = 0;
    bool ok = wait_bus_free() && read_identity(0x68, &identity);
    if (ok && identity != 0x70) {
        fprintf(stderr, "WHO_AM_I inatteso: 0x%02x\n", identity);
        ok = false;
    }

    // Reset soltanto del sensore.
    if (ok)
        ok = sensor_write(0x6B, 0x80);
    usleep(100000);

    const unsigned addresses[] = {
        0x6B, 0x6C, 0x1B, 0x1C, 0x1A, 0x1D, 0x19
    };
    const unsigned values[] = {
        0x01, 0x00, 0x00, 0x00, 0x03, 0x03, 0x09
    };

    for (unsigned i = 0; i < 7 && ok; ++i)
        ok = sensor_write(addresses[i], values[i]);

    usleep(100000);

    // Verifica le impostazioni rileggendole.
    for (unsigned i = 0; i < 7 && ok; ++i) {
        unsigned char value = 0;
        ok = sensor_read(addresses[i], &value, 1);
        if (ok && value != values[i]) {
            fprintf(stderr,
                    "Registro 0x%02x: atteso 0x%02x, letto 0x%02x\n",
                    addresses[i], values[i], value);
            ok = false;
        }
    }

    if (ok) {
        printf("Configurazione verificata: accel +/-2g, gyro +/-250 deg/s.\n");
        printf("Tenere il robot fermo. Lettura per circa 6 secondi.\n");
        fflush(stdout);
    }

    for (int i = 0; i < 30 && ok && !stop_requested; ++i) {
        unsigned char data[14];
        ok = sensor_read(0x3B, data, 14);
        if (!ok)
            break;

        double ax = signed_word(data) / 16384.0;
        double ay = signed_word(data + 2) / 16384.0;
        double az = signed_word(data + 4) / 16384.0;
        // Byte 6 e 7: temperatura, qui non utilizzata.
        double gx = signed_word(data + 8) / 131.0;
        double gy = signed_word(data + 10) / 131.0;
        double gz = signed_word(data + 12) / 131.0;

        printf("a[g]=%+.3f %+.3f %+.3f |a|=%.3f "
               "gyro[deg/s]=%+.2f %+.2f %+.2f\n",
               ax, ay, az, std::sqrt(ax*ax + ay*ay + az*az),
               gx, gy, gz);
        fflush(stdout);
        usleep(200000);
    }

    if (!ok) {
        wr(4, 0x41);
        wait_bus_free();
    }

    wr(2, 0);
    printf("Test terminato; controller I2C disabilitato.\n");
    munmap(base, span);
    close(fd);
    return ok ? 0 : 1;
}
