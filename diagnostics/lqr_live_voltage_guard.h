#ifndef ROBOT01_LQR_LIVE_VOLTAGE_GUARD_H
#define ROBOT01_LQR_LIVE_VOLTAGE_GUARD_H
// Included after the guarded driver's register/stop helpers. ADC only, no PWM command.
struct LqrLiveVoltageGuard {
    bool pending,fault;double started,next,measured,voltage;unsigned raw;const char *reason;
    LqrLiveVoltageGuard():pending(false),fault(false),started(0),next(0),measured(-1),voltage(0),raw(0),reason("none") {}
    bool fail(const char *why) { fault=true;reason=why;stop_requested=1;stop_motors();return false; }
    bool accept(unsigned value,double t) {
        raw=value;pending=false;
        if(!raw || raw>=4095)return fail("invalid_adc");
        voltage=raw*18./4.7/1000.;measured=t;next=t+.09;
        if(voltage<11.20 || voltage>12.10)return fail("outside_live_11.20_12.10V");
        return true;
    }
    bool update(double t) {
        if(fault)return fail(reason);
        if(pending) {
            if(reg_read(0x90)&1u){if(!accept(reg_read(0x94),t))return false;}
            else if(t-started>.025)return fail("adc_timeout");
        }
        if(!pending && t>=next){reg_write(0x94,1);reg_write(0x90,2);reg_write(0x90,3);reg_write(0x90,2);pending=true;started=t;}
        if(measured>=0 && t-measured>.15)return fail("stale_adc");
        return true;
    }
    bool ready(double t) const {return !fault && measured>=0 && t-measured>=0 && t-measured<=.15;}
};
#endif
