#define COMPENSATED_FREEWHEEL_MAIN freewheel_hardware_main_unused
#include "../diagnostics/motor_compensated_freewheel_imu.cpp"
#include <cassert>
int main() {
    for(unsigned i=0;i<=2600;++i) {
        const double u=compensated_nominal(i);
        assert(std::fabs(u)<=8);
        if(i<100 || i>=2500)assert(u==0);
    }
    assert(compensated_nominal(100)==-4 && compensated_nominal(300)==0);
    assert(compensated_nominal(500)==-6 && compensated_nominal(900)==-8);
    assert(compensated_nominal(1300)==4 && compensated_nominal(1700)==6 && compensated_nominal(2100)==8);
    uint32_t memory[1024]={0};regs=memory;stop_requested=0;
    wr(robot_hw::PWM_SX_PERIOD,50000);wr(robot_hw::PWM_DX_PERIOD,50000);
    const double good[]={-4.5,-4.002};assert(apply_compensated_pair(good));
    assert(rd(robot_hw::PWM_SX_HIGH)==2250 && rd(robot_hw::PWM_DX_HIGH)==2001);
    assert(rd(robot_hw::PWM_SX_CONTROL)==5 && rd(robot_hw::PWM_DX_CONTROL)==5);
    const double reverse[]={4.,4.5};assert(apply_compensated_pair(reverse));
    assert(rd(robot_hw::PWM_SX_CONTROL)==7 && rd(robot_hw::PWM_DX_CONTROL)==7);
    stop_requested=1;assert(!apply_compensated_pair(reverse));assert(rd(robot_hw::PWM_SX_HIGH)==0);
    stop_requested=0;const double bad[]={8.002,8.};assert(!apply_compensated_pair(bad));assert(stop_requested);
    assert(!(rd(robot_hw::PWM_SX_CONTROL)&1u) && !(rd(robot_hw::PWM_DX_CONTROL)&1u));
    regs=NULL;return 0;
}
