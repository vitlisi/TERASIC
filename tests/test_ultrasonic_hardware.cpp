#include "../src_cpp/terasic_reale/hardware/ultrasonic_core.h"
#include <cassert>
#include <cmath>
#include <cstring>
int main()
{
    assert(std::fabs(ultrasonic_echo_us(50000)-1000)<1e-9);
    assert(std::fabs(ultrasonic_distance_cm(50000)-17)<1e-9);
    assert(!strcmp(ultrasonic_status(0),"ZERO"));
    assert(!strcmp(ultrasonic_status(0x98),"ERRORE_VENDOR"));
    assert(!strcmp(ultrasonic_status(0xffffffff),"REGISTRO_NON_VALIDO"));
    assert(!strcmp(ultrasonic_status(5882),"FUORI_PORTATA"));
    assert(!strcmp(ultrasonic_status(5883),"IN_PORTATA_NON_CERTIFICA_ECO_NUOVO"));
    assert(!strcmp(ultrasonic_status(1176470),"IN_PORTATA_NON_CERTIFICA_ECO_NUOVO"));
    assert(!strcmp(ultrasonic_status(1176471),"FUORI_PORTATA"));
}
