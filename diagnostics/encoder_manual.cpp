#define _FILE_OFFSET_BITS 64
#include "../src_cpp/terasic_reale/hardware/robot_wheel_map.h"
#include <cstdio>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <signal.h>
#include <sys/file.h>
#include <dirent.h>
#include <cstdlib>
#include <cstring>

static bool hardware_process_conflict()
{
    DIR *directory = opendir("/proc");
    if (!directory) { perror("opendir /proc"); return true; }
    const char *names[] = {"pitch_live", "pitch_pd", "pitch_pd_fixed", "balance_lqr",
        "robot_test", "balance_car", "bal_car", "DE10_Nano_bal", "imu_measure",
        "imu_identity", "motor_pulse", "encoder_manual", "battery_status"};
    bool conflict = false;
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        char *end = NULL;
        const long pid = strtol(entry->d_name, &end, 10);
        if (pid <= 0 || *end || pid == static_cast<long>(getpid())) continue;
        char path[128], name[64] = {0};
        std::snprintf(path,sizeof(path),"/proc/%ld/comm",pid);
        FILE *file = std::fopen(path,"r");
        if (!file) continue;
        const bool read_ok = std::fgets(name,sizeof(name),file) != NULL;
        std::fclose(file);
        if (!read_ok) continue;
        name[std::strcspn(name,"\r\n")] = 0;
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

static volatile sig_atomic_t stop_requested = 0;

static void request_stop(int)
{
    stop_requested = 1;
}

static uint32_t read_reg(void *base, unsigned offset)
{
    return *reinterpret_cast<volatile uint32_t *>(
        static_cast<unsigned char *>(base) + offset);
}

static void write_reg(void *base, unsigned offset, uint32_t value)
{
    *reinterpret_cast<volatile uint32_t *>(
        static_cast<unsigned char *>(base) + offset) = value;
}

int main()
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    const int lock_fd=open("/tmp/terasic_balance_lqr.lock",O_RDWR|O_CREAT,0600);
    if (lock_fd<0) { perror("lock encoder"); return 1; }
    if (flock(lock_fd,LOCK_EX|LOCK_NB)!=0) { perror("Hardware occupato"); close(lock_fd); return 1; }
    if (hardware_process_conflict()) { close(lock_fd); return 1; }
    const size_t span = 0x1000;
    const off_t physical_base = static_cast<off_t>(0xFF200000ULL);

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    void *base = mmap(NULL, span, PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, physical_base);
    if (base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);
    signal(SIGHUP, request_stop);

    if ((read_reg(base, robot_hw::PWM_SX_CONTROL) & 1u) ||
        (read_reg(base, robot_hw::PWM_DX_CONTROL) & 1u)) {
        fprintf(stderr, "Test annullato: un motore ha RUN=1.\n");
        munmap(base, span);
        close(fd);
        return 1;
    }

    // Disabilita il conteggio prima di inizializzare i contatori.
    write_reg(base, robot_hw::ENCODER_SX_ENABLE, 0);
    write_reg(base, robot_hw::ENCODER_DX_ENABLE, 0);

    // Registro indice 2: carica il valore iniziale.
    write_reg(base, robot_hw::ENCODER_SX_LOAD, 32768);
    write_reg(base, robot_hw::ENCODER_DX_LOAD, 32768);

    // Registro indice 1: abilita soltanto il conteggio.
    write_reg(base, robot_hw::ENCODER_SX_ENABLE, 1);
    write_reg(base, robot_hw::ENCODER_DX_ENABLE, 1);

    printf("PRONTO: test manuale di 30 secondi. Motori spenti; nessuna calibrazione IMU.\n");
    printf("Sostenere il telaio fermo con ruote libere. Girare A MANO una sola ruota di UN giro.\n");
    printf("SX e DX sono i lati FISICI nel verso di avanzamento (physical_sx_dx_v2).\n");
    printf("Conteggi iniziali impostati a 32768.\n");
    fflush(stdout);

    int previous_left = -1;
    int previous_right = -1;
    bool ok = true;

    for (int sample = 0; sample < 150 && !stop_requested; ++sample) {
        if ((read_reg(base,robot_hw::PWM_SX_CONTROL)&1u) || (read_reg(base,robot_hw::PWM_DX_CONTROL)&1u)) {
            fprintf(stderr,"Motore attivo durante test: acquisizione annullata.\n");
            ok=false; break;
        }
        int left = static_cast<int>(read_reg(base, robot_hw::ENCODER_SX_VALUE) & 0xffffu);
        int right = static_cast<int>(read_reg(base, robot_hw::ENCODER_DX_VALUE) & 0xffffu);
        if (left==0 || left==65535 || right==0 || right==65535) {
            fprintf(stderr,"Contatore saturo: acquisizione annullata.\n");
            ok=false; break;
        }

        if (left != previous_left || right != previous_right || sample%5==0) {
            printf("t~%4.1f s | SX=%5d DX=%5d | delta SX=%+6d DX=%+6d\n",
                   sample*.2,left, right, left - 32768, right - 32768);
            fflush(stdout);
            previous_left = left;
            previous_right = right;
        }

        usleep(200000);
    }

    write_reg(base, robot_hw::ENCODER_SX_ENABLE, 0);
    write_reg(base, robot_hw::ENCODER_DX_ENABLE, 0);
    printf("FINALE: delta SX=%+d DX=%+d | RUN SX=%u DX=%u | esito=%s\n",
           static_cast<int>(read_reg(base,robot_hw::ENCODER_SX_VALUE)&0xffffu)-32768,
           static_cast<int>(read_reg(base,robot_hw::ENCODER_DX_VALUE)&0xffffu)-32768,
           static_cast<unsigned>(read_reg(base,robot_hw::PWM_SX_CONTROL)&1u),
           static_cast<unsigned>(read_reg(base,robot_hw::PWM_DX_CONTROL)&1u),
           !ok ? "ERRORE" : (stop_requested ? "INTERROTTO" : "COMPLETATO"));
    printf("Test terminato: conteggio disabilitato.\n");

    munmap(base, span);
    close(fd);
    close(lock_fd);
    return ok && !stop_requested ? 0 : 1;
}
