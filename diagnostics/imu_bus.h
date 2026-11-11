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

