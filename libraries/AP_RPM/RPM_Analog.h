#pragma once

#include "AP_RPM.h"

#include "RPM_Backend.h"
#include <Filter/Filter.h>
#include <AP_Math/AP_Math.h>

class AP_RPM_Analog: public AP_RPM_Backend{
    public:
        // constructor
        AP_RPM_Analog(AP_RPM &ranger, uint8_t instance, AP_RPM::RPM_State &_state);

        // update state
        void update(void) override;
    
    private:
        ModeFilterFloat_Size5 signal_quality_filter {3};
       AP_HAL::AnalogSource *_rpm_pin_analog_source;

};