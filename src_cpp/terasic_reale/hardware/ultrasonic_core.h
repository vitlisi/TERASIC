#ifndef TERASIC_ULTRASONIC_CORE_H
#define TERASIC_ULTRASONIC_CORE_H
#include <stdint.h>
// Vendor Qsys: sonic_distance at offset0x98, clk_0=50MHz.
// RTL counts full echo width; distance=width*340m/s/2.
// No sequence/valid/timeout register exists: range alone cannot prove freshness.
inline double ultrasonic_echo_us(uint32_t raw) { return raw/50.0; }
inline double ultrasonic_distance_cm(uint32_t raw) { return raw*0.00034; }
inline bool ultrasonic_in_range(uint32_t raw)
{
    const double cm=ultrasonic_distance_cm(raw);
    return raw!=0 && raw!=0x98 && raw<=0x3fffff && cm>=2 && cm<=400;
}
inline const char *ultrasonic_status(uint32_t raw)
{
    if(raw==0)return "ZERO";
    // Historical vendor software treats this value as a failed bus read.
    if(raw==0x98)return "ERRORE_VENDOR";
    if(raw>0x3fffff)return "REGISTRO_NON_VALIDO";
    const double cm=ultrasonic_distance_cm(raw);
    if(cm<2 || cm>400)return "FUORI_PORTATA";
    return "IN_PORTATA_NON_CERTIFICA_ECO_NUOVO";
}
#endif
