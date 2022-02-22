#include <AP_HAL/AP_HAL.h>

#include "RPM_Analog.h"

#include <stdio.h>
extern const AP_HAL::HAL &hal;

/* 
   open the sensor in constructor
*/
AP_RPM_Analog::AP_RPM_Analog(AP_RPM &_ap_rpm, uint8_t _instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, _instance, _state)
{
   _rpm_pin_analog_source = hal.analogin->channel(get_pin());
}

void AP_RPM_Analog::update(void)
{
    // this copes with changing the pin at runtime
    if(!_rpm_pin_analog_source->set_pin(get_pin())){
        return;
    }

    state.rate_rpm = _rpm_pin_analog_source->read_average();
    state.rate_rpm *= ap_rpm._params[state.instance].scaling;
    state.signal_quality = 0.5f;
    state.last_reading_ms = AP_HAL::millis();
}