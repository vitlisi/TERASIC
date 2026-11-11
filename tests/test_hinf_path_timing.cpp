#define main hinf_program_main
#include "../src_cpp/terasic_reale/controllers/balance_hinf_path.cpp"
#undef main
#include <cassert>

int main()
{
    assert(path_control_sample_count(28.5, false) == 2850);
    assert(path_control_sample_count(28.5, true) == 3300);
    return 0;
}
