// MMIO simulated in ordinary memory: no hardware accesses.
#define main freewheel_program_main
#include "../diagnostics/motor_freewheel.cpp"
#undef main
#include <cassert>
#include <cmath>

int main()
{
    unsigned phase=9;
    assert(protocol_pwm(0,&phase)==0 && phase==0);
    assert(protocol_pwm(1,&phase)==10 && phase==1);
    assert(protocol_pwm(2,&phase)==0 && phase==0);
    assert(protocol_pwm(6,&phase)==15 && phase==2);
    assert(protocol_pwm(11,&phase)==20 && phase==3);
    assert(protocol_pwm(12,&phase)==0 && protocol_pwm(15,&phase)==0);
    uint32_t memory[1024]={0};regs=memory;
    wr(robot_hw::PWM_SX_PERIOD,50000);wr(robot_hw::PWM_DX_PERIOD,50000);
    wr(robot_hw::ENCODER_SX_VALUE,32768);wr(robot_hw::ENCODER_DX_VALUE,32768);
    freewheel_apply(15);
    PulseSample s=pulse_sample(6,2);
    assert(s.control_sx==7 && s.control_dx==7);
    assert(sampled_duty(s.high_sx,s.period_sx,s.control_sx)==15);
    assert(s.high_dx==7500 && safe_counters(s));
    freewheel_apply(-5);
    s=pulse_sample(0,0);
    assert(s.control_sx==5 && s.control_dx==5);
    assert(sampled_duty(s.high_sx,s.period_sx,s.control_sx)==-5);
    freewheel_apply(0);
    assert(rd(robot_hw::PWM_SX_CONTROL)==6 && rd(robot_hw::PWM_SX_HIGH)==0);
    small_commands=true;
    assert(protocol_pwm(1,&phase)==3 && phase==1);
    assert(protocol_pwm(3,&phase)==-3 && phase==2);
    assert(protocol_pwm(14,&phase)==3 && phase==7);
    assert(protocol_pwm(14.25,&phase)==-3 && phase==8);
    assert(protocol_pwm(23.75,&phase)==-8 && phase==30);
    assert(protocol_pwm(24,&phase)==0 && phase==0);
    small_commands=false;startup_ramp=true;
    for(unsigned variant=0;variant<2;++variant) {
    ramp_steps=variant ? 40 : 32;
    for(unsigned block=0;block<4;++block) {
        const double start=1.+(.5*ramp_steps+6.)*block;
        assert(protocol_pwm(start-.01,&phase)==0);
        for(unsigned step=0;step<ramp_steps;++step) {
            const double pwm=protocol_pwm(start+.5*step,&phase);
            assert(pwm==(block%2 ? -1. : 1.)*.25*(step+1));
            assert(phase==1+ramp_steps*block+step && ramp_wheel==block/2);
            freewheel_apply(pwm);
            const PulseSample h=pulse_sample(0,phase);
            assert(sampled_duty(h.high_sx,h.period_sx,h.control_sx)==(block<2 ? pwm : 0));
            assert(sampled_duty(h.high_dx,h.period_dx,h.control_dx)==(block<2 ? 0 : pwm));
            assert((block<2 ? h.control_dx : h.control_sx)==6);
        }
        assert(protocol_pwm(start+.5*ramp_steps,&phase)==0);
    }
    }
    request_stop(SIGALRM);
    freewheel_apply(20); // Cannot restart after a watchdog or stop signal.
    assert(watchdog_expired && rd(robot_hw::PWM_SX_CONTROL)==6);
    assert(rd(robot_hw::PWM_DX_CONTROL)==6 && rd(robot_hw::PWM_DX_HIGH)==0);
    stop_requested=watchdog_expired=0;
    BatteryAcquisition battery;
    assert(battery.update(0) && battery.pending);
    wr(0x90,3);wr(0x94,2821);
    assert(battery.update(.01) && !battery.pending);
    assert(std::fabs(battery.voltage-2821*18./4.7/1000.)<1e-9);
    assert(battery.update(.11));
    assert(!battery.update(.14)); // Ready bit low after start: conversion timeout.
    BatteryAcquisition low;
    assert(low.update(0));wr(0x90,3);wr(0x94,2600);
    assert(!low.update(.01));
    s.sx=256;assert(!safe_counters(s));
    regs=NULL;
    return 0;
}
