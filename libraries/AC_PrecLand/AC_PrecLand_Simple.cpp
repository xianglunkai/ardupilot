#include "AC_PrecLand_config.h"

#if AC_PRECLAND_SIMPLE_ENABLED

#include "AC_PrecLand_Simple.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_AHRS/AP_AHRS.h"
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>


// init - perform initialisation of this backend
void AC_PrecLand_Simple::init()
{
    // set healthy
    _state.healthy = true;
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_Simple::update()
{
    // update health
    _state.healthy =  AP::ahrs().healthy();

    // get radio
    uint16_t cvalue = 1500;
    RC_Channel *c = rc().find_channel_for_option(RC_Channel::AUX_FUNC::SCRIPTING_2);
    if (c != nullptr && rc().has_valid_input()) {
        // get control channel
        cvalue = c->get_radio_in();

        if (cvalue >= RC_Channel::AUX_PWM_TRIGGER_HIGH) {
            _dock_pos_valid = AP::ahrs().get_relative_position_NED_origin(_dock_pos);
            // send message to notify user we have set dock position
            Location loc{};
            if (AP::ahrs().get_location(loc)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Set up dock pos: lat=%d lon=%d", int(loc.lat), int(loc.lng));
            }
        }
    }
    
    // get new sensor data
    if (_state.healthy && _dock_pos_valid) {
        Vector3f curr_pos_NED;
        if (!AP::ahrs().get_relative_position_NED_origin(curr_pos_NED)) {
            return;
        }
        auto && _distance_to_target_vec = _dock_pos - curr_pos_NED;

        _distance_to_target = _distance_to_target_vec.length();
       
        _los_meas_body = (_distance_to_target_vec / (_distance_to_target + 1e-6f));
     
        _have_los_meas = true;
        _los_meas_time_ms = AP_HAL::millis();
    } else {
        _have_los_meas = false;
    }

    _have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
}

#endif // AC_PRECLAND_IRLOCK_ENABLED
