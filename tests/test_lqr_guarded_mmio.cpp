#define main guarded_lqr_hardware_main
#include "../diagnostics/balance_lqr_guarded.cpp"
#undef main
#include <cassert>
int main() {
    uint32_t memory[1024]={0};fpga=memory;stop_requested=0;
    double expected[]={8.,-6.123},actual[2]={0,0};set_duties(expected[0],expected[1]);
    assert(guarded_readback(expected,actual));assert(actual[0]==8. && actual[1]<-6.12 && actual[1]>-6.124);
    LqrLiveVoltageGuard b;assert(b.accept(3000,1.));assert(b.ready(1.1));
    assert(!b.accept(2924,1.2));assert(stop_requested);
    assert(reg_read(robot_hw::PWM_SX_HIGH)==0 && reg_read(robot_hw::PWM_DX_HIGH)==0);
    assert(!(reg_read(robot_hw::PWM_SX_CONTROL)&1u) && !(reg_read(robot_hw::PWM_DX_CONTROL)&1u));
    assert(!b.update(1.3));set_duties(8,8);assert(reg_read(robot_hw::PWM_SX_HIGH)==0);
    stop_requested=0;set_duties(8,-8);reg_write(robot_hw::PWM_DX_HIGH,1);expected[1]=-8;
    assert(!guarded_readback(expected,actual));assert(stop_requested);
    assert(reg_read(robot_hw::PWM_SX_HIGH)==0 && reg_read(robot_hw::PWM_DX_HIGH)==0);
    fpga=NULL;return 0;
}
