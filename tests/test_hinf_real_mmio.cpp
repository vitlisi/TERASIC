// In-memory register test: never opens /dev/mem or starts a real motor.
#define main mimo_program_main
#include "../src_cpp/terasic_reale/controllers/balance_hinf.cpp"
#undef main
#include <cassert>
int main()
{
    uint32_t registers[1024]={0};fpga=registers;stop_requested=0;
    set_duties(8,-8);
    assert(reg_read(0x64)==4000 && reg_read(0x74)==4000);
    assert(reg_read(0x68)==7 && reg_read(0x78)==5);
    set_duties(0,8);
    assert(reg_read(0x68)==6 && reg_read(0x64)==0);
    assert(reg_read(0x78)==7 && reg_read(0x74)==4000);
    handle_stop(SIGTERM);
    assert(reg_read(0x68)==6 && reg_read(0x78)==6);
    assert(reg_read(0x64)==0 && reg_read(0x74)==0);
    set_duties(8,8); // pending stop may never re-arm either channel
    assert(reg_read(0x68)==6 && reg_read(0x78)==6);
    fpga=NULL;return 0;
}
