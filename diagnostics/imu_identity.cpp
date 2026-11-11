#define _FILE_OFFSET_BITS 64
#include <cstdio>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>

static volatile uint32_t *regs;

static unsigned rd(unsigned index)
{
    return regs[index] & 0xffu;
}

static void wr(unsigned index, unsigned value)
{
    regs[index] = value;
}

static double now_seconds()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static bool command(unsigned cmd, bool check_ack,
                    const char *stage)
{
    // IACK cancella il flag della precedente operazione.
    wr(4, cmd | 0x01);
    const double deadline = now_seconds() + 0.020;

    do {
        unsigned status = rd(4);

        if (status & 0x20) {
            fprintf(stderr, "%s: arbitraggio perso, SR=0x%02x\n",
                    stage, status);
            return false;
        }

        // IF=1 indica completamento; TIP=0 indica trasferimento finito.
        if ((status & 0x01) && !(status & 0x02)) {
            if (check_ack && (status & 0x80)) {
                fprintf(stderr, "%s: NACK, SR=0x%02x\n",
                        stage, status);
                return false;
            }
            return true;
        }

        usleep(50);
    } while (now_seconds() < deadline);

    fprintf(stderr, "%s: timeout, SR=0x%02x\n", stage, rd(4));
    return false;
}

static bool wait_bus_free()
{
    const double deadline = now_seconds() + 0.020;
    do {
        if (!(rd(4) & 0x40))
            return true;
        usleep(50);
    } while (now_seconds() < deadline);

    fprintf(stderr, "Bus ancora occupato, SR=0x%02x\n", rd(4));
    return false;
}

static bool read_identity(unsigned address, unsigned *value)
{
    // START e invio dell'indirizzo con bit di scrittura.
    wr(3, address << 1);
    if (!command(0x90, true, "Indirizzo in scrittura"))
        return false;

    // Seleziona WHO_AM_I; non scrive un valore nel sensore.
    wr(3, 0x75);
    if (!command(0x10, true, "Selezione WHO_AM_I"))
        return false;

    // Repeated START e indirizzo con bit di lettura.
    wr(3, (address << 1) | 1);
    if (!command(0x90, true, "Indirizzo in lettura"))
        return false;

    // Legge l'ultimo byte, invia NACK e genera STOP.
    if (!command(0x68, false, "Lettura WHO_AM_I"))
        return false;

    *value = rd(3);
    return wait_bus_free();
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

    // Il controller I2C si trova all'offset FPGA 0x00.
    // Prescaler = 50 MHz / (5 * 100 kHz) - 1 = 99.
    wr(2, 0x00);
    wr(4, 0x01);
    wr(0, 99);
    wr(1, 0);
    wr(2, 0x80);

    printf("I2C configurato a 100 kHz con clock di 50 MHz.\n");
    printf("Test all'indirizzo I2C 0x68.\n");
    fflush(stdout);

    bool ok = wait_bus_free();

    for (int i = 0; i < 3 && ok; ++i) {
        unsigned identity = 0;
        ok = read_identity(0x68, &identity);

        if (ok) {
            printf("Lettura %d: WHO_AM_I=0x%02x\n", i + 1, identity);
            if (identity != 0x70) {
                fprintf(stderr, "Identificativo diverso da MPU-6500.\n");
                ok = false;
            }
        }
        usleep(100000);
    }

    if (!ok) {
        // Tenta di chiudere una transazione eventualmente incompleta.
        wr(4, 0x41);
        wait_bus_free();
    }

    wr(2, 0x00);
    printf("Test terminato; controller I2C disabilitato.\n");
    munmap(base, span);
    close(fd);
    return ok ? 0 : 1;
}
