#define _FILE_OFFSET_BITS 64
#include "../src_cpp/terasic_reale/hardware/robot_wheel_map.h"
#include <cstdio>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

static uint32_t read_reg(void *base, unsigned offset)
{
    volatile uint32_t *reg =
        reinterpret_cast<volatile uint32_t *>(
            static_cast<unsigned char *>(base) + offset);
    return *reg;
}

int main()
{
    fprintf(stderr,"wheel_convention=%s\n",robot_hw::wheel_convention());
    const off_t physical_base = static_cast<off_t>(0xFF200000ULL);
    const size_t span = 0x11000;

    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    void *base = mmap(NULL, span, PROT_READ, MAP_SHARED,
                      fd, physical_base);
    if (base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    printf("SW raw:       0x%08x\n",
           static_cast<unsigned>(read_reg(base, 0x10080)));

    printf("Encoder SX:   %u\n",
           static_cast<unsigned>(read_reg(base, robot_hw::ENCODER_SX_VALUE) & 0xffff));
    printf("Encoder DX:   %u\n",
           static_cast<unsigned>(read_reg(base, robot_hw::ENCODER_DX_VALUE) & 0xffff));

    printf("Motore SX:    periodo=%u alto=%u controllo=0x%08x\n",
           static_cast<unsigned>(read_reg(base, robot_hw::PWM_SX_PERIOD)),
           static_cast<unsigned>(read_reg(base, robot_hw::PWM_SX_HIGH)),
           static_cast<unsigned>(read_reg(base, robot_hw::PWM_SX_CONTROL)));
    printf("Motore DX:    periodo=%u alto=%u controllo=0x%08x\n",
           static_cast<unsigned>(read_reg(base, robot_hw::PWM_DX_PERIOD)),
           static_cast<unsigned>(read_reg(base, robot_hw::PWM_DX_HIGH)),
           static_cast<unsigned>(read_reg(base, robot_hw::PWM_DX_CONTROL)));

    munmap(base, span);
    close(fd);
    return 0;
}
