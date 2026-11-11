#include <cassert>
#include "../src_cpp/terasic_reale/hardware/robot_wheel_map.h"
#include "../src_cpp/terasic_reale/controllers/balance_lqr_mimo_core.h"
int main() {
    // Physical identity confirmed by the operator, not derived from enum aliases.
    assert(robot_hw::PWM_SX_PERIOD==0x60 && robot_hw::PWM_DX_PERIOD==0x70);
    assert(robot_hw::PWM_SX_HIGH==0x64 && robot_hw::PWM_DX_HIGH==0x74);
    assert(robot_hw::PWM_SX_CONTROL==0x68 && robot_hw::PWM_DX_CONTROL==0x78);
    assert(robot_hw::ENCODER_SX_VALUE==0x20 && robot_hw::ENCODER_DX_VALUE==0x40);
    assert(robot_hw::ENCODER_SX_LOAD==0x28 && robot_hw::ENCODER_DX_LOAD==0x48);
    LqrMimoEstimator e(1325,.032,.16444,0);LqrMimoState s;
    assert(e.update(32768,32768,0,0,0,&s));
    assert(e.update(32768-100,32768+100,.01,0,0,&s));
    assert(s.x[0]>0 && s.x[3]>0 && s.x[2]==0);
    // Right wheel advances alone -> positive yaw, agreeing with MuJoCo lift.
    LqrMimoEstimator turn(1325,.032,.16444,0);
    assert(turn.update(32768,32768,0,0,0,&s));
    assert(turn.update(32768,32868,.01,0,0,&s));
    assert(s.x[0]>0 && s.x[2]>0 && s.x[5]>0);
    // For the existing symmetric K, coordinate migration preserves hold commands:
    // K_v2 = swap_rows(K_v1) * diag(1,1,-1,1,1,-1) == K_v1.
    const double k[2][6]={{-268,-736,-56,-363,-22,-12},{-268,-736,56,-363,-22,12}};
    const double signs[6]={1,1,-1,1,1,-1};
    for(int row=0;row<2;++row)for(int col=0;col<6;++col)
        assert(k[row][col]==k[1-row][col]*signs[col]);
    return 0;
}
