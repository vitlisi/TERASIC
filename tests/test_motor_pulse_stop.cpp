// Test signal stops using memory-backed registers, never /dev/mem.
#define main motor_pulse_program_main
#include "../diagnostics/motor_pulse.cpp"
#undef main
#include <cassert>

int main()
{
    uint32_t memory[1024]={0};
    regs=memory;
    memory[robot_hw::ENCODER_SX_VALUE/4]=32760;
    memory[robot_hw::ENCODER_DX_VALUE/4]=32768;
    memory[robot_hw::PWM_SX_PERIOD/4]=50000;
    memory[robot_hw::PWM_SX_HIGH/4]=4000;
    memory[robot_hw::PWM_SX_CONTROL/4]=0x07;
    PulseSample captured=pulse_sample(.02,1);
    assert(captured.sx==32760 && captured.dx==32768 && captured.phase==1);
    assert(sampled_duty(captured.high_sx,captured.period_sx,captured.control_sx)==8);
    assert(sampled_duty(4000,50000,0x05)==-8);
    assert(sampled_duty(4000,50000,0x06)==0);
    assert(sampled_duty(4000,0,0x07)==0);
    memory[0x78/4]=memory[0x68/4]=0x07;
    memory[0x74/4]=memory[0x64/4]=4000;
    request_stop(SIGHUP);
    assert(stop_requested && !watchdog_expired);
    assert(!(memory[0x78/4]&1) && !(memory[0x68/4]&1));
    assert(memory[0x74/4]==0 && memory[0x64/4]==0);
    stop_requested=0;
    memory[0x78/4]=memory[0x68/4]=0x07;
    request_stop(SIGALRM);
    assert(stop_requested && watchdog_expired);
    assert(!(memory[0x78/4]&1) && !(memory[0x68/4]&1));
    regs=NULL;
    request_stop(SIGTERM); // Signal before/after mapping must not dereference it.
    return 0;
}
