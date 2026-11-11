#ifndef TERASIC_ROBOT_WHEEL_MAP_H
#define TERASIC_ROBOT_WHEEL_MAP_H
// v2: SX/DX are PHYSICAL left/right viewed in the forward travel direction.
// Verified 2026-10-28: legacy FPGA-labelled SX drives physical RIGHT.
// MMIO offsets from 0xFF200000. Never apply this permutation to I2C values.
namespace robot_hw {
    enum {
        PWM_SX_PERIOD=0x60, PWM_SX_HIGH=0x64, PWM_SX_CONTROL=0x68,
        PWM_DX_PERIOD=0x70, PWM_DX_HIGH=0x74, PWM_DX_CONTROL=0x78,
        ENCODER_SX_VALUE=0x20, ENCODER_SX_ENABLE=0x24, ENCODER_SX_LOAD=0x28,
        ENCODER_DX_VALUE=0x40, ENCODER_DX_ENABLE=0x44, ENCODER_DX_LOAD=0x48
    };
    inline const char *wheel_convention() { return "physical_sx_dx_v2"; }
}
#endif
