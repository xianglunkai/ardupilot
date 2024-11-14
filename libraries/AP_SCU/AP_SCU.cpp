#include "AP_SCU.h"
#if AP_SCU_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Relay/AP_Relay.h>
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SCU/AP_CANIO.h>

extern const AP_HAL::HAL& hal;


// parameters
const AP_Param::GroupInfo AP_SCU::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Engine control
    // @Description: This enables internal combustion engine control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_SCU, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: 
    // @DisplayName: 
    // @Description: 
    // @User: 
    // @Units: 
    // @Range: 
    AP_GROUPINFO("CCW_ON", 1, AP_SCU, steering_relay_on, 1),

    // @Param: 
    // @DisplayName: 
    // @Description: 
    // @User: 
    // @Units: 
    // @Range: 
    AP_GROUPINFO("STR_FREQ", 2, AP_SCU, steering_pwm_freq_khz, 33.0f),


    // @Param: 
    // @DisplayName: 
    // @Description: 
    // @User: 
    // @Units: 
    // @Range: 
    AP_GROUPINFO("THR_FREQ", 3, AP_SCU, throttle_pwm_freq_khz, 1.0f),

    AP_GROUPEND
};



// singleton instance
AP_SCU *AP_SCU::_singleton;

AP_SCU::AP_SCU():   
    _throttle_idx(SRV_Channel::k_none),
    _steering_idx(SRV_Channel::k_none)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}


// init - performs any required initialisation for this instance
void AP_SCU::init()
{
    _steering_idx = SRV_Channel::k_motor1;
    _throttle_idx = SRV_Channel::k_motor3;

    _engine.init();
    _steering.init();

    // setup servo output
    setup_servo_output();

    // setup pwm type
    setup_pwm_type();

    // set safety output
    setup_safety_output();
}

// update position - should be called periodically
void AP_SCU::update()
{
    uint32_t now = AP_HAL::millis();

    // exit if not enabled
    if (!enable || _first_update || !hal.util->get_soft_armed())  {
        _first_update = false;
        _last_update_ms = now;
        return;
    }

    int16_t dt_ms = now - _last_update_ms;
    if (dt_ms >= 500) {
        dt_ms = 50;
    }

    // 1. engine start and stop control
    // DISARM and ARMED
    uint16_t cvalue = 1500;
    uint16_t sts_cmd = 0;
    // start command
    RC_Channel *c_start = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARMDISARM);
    if (c_start != nullptr && rc().has_valid_input()) {
        // get starter control channel
        cvalue = c_start->get_radio_in();

        if (cvalue >= RC_Channel::AUX_PWM_TRIGGER_HIGH) {
            sts_cmd = 1;
        } 
    }

    // stop command
    RC_Channel *c_stop = rc().find_channel_for_option(RC_Channel::AUX_FUNC::DISARM);
    if (c_stop != nullptr && rc().has_valid_input()) {
        // get starter control channel
        cvalue = c_stop->get_radio_in();

        if (cvalue >= RC_Channel::AUX_PWM_TRIGGER_HIGH) {
            sts_cmd = 2;
        } 
    }

    if (AP_Notify::flags.failsafe_radio) {
        // user has requested ignition kill on RC failsafe
        sts_cmd = 0;
    }

    // start and stop control
    _engine.eng_armed_control(sts_cmd, 0, dt_ms);
    

    // 2. engine throttle and gear control
    // read k_throttle from AP_Motor
    const float throttle = constrain_float(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_throttle), -1.0f, 1.0f);
    const int16_t throttle_out = _engine.eng_throttle_control(throttle); 
    move_servo(_throttle_idx, throttle_out, -1000, +1000);
    // set frequency
    SRV_Channels::set_rc_frequency(_throttle_idx, throttle_pwm_freq_khz * 1000);

    // 3. steering control
    const float steering = constrain_float(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_steering), -1.0f, 1.0f);
    float steering_pos = 0;
    AP_CANIO *can_io = AP::can_io();
    if (can_io == nullptr) {
        SRV_Channels::set_rc_frequency(_steering_idx, 0);
        return;
    }
    if(!can_io->read(AP_CANIO_Params::FUNCTION::STEER_ANG, steering_pos)) {
        SRV_Channels::set_rc_frequency(_steering_idx, 0);
        return;
    }
    const float steering_out = _steering.steering_angle_control(steering, steering_pos);
    // ouput PWM duty = 50%
    move_servo(_steering_idx, 0, -1000, 1000);
    // set frequency
    SRV_Channels::set_rc_frequency(_steering_idx, steering_pwm_freq_khz * 1000 * abs(steering_out));
    // control steering direction
    AP_Relay *ap_relay = AP::relay();
    if (ap_relay == nullptr) {
        return;
    }
    bool on = is_positive(steering_out) ? steering_relay_on : !steering_relay_on;
    ap_relay->set(AP_Relay_Params::FUNCTION::STERT_DIR, on);
}


void AP_SCU::setup_servo_output()
{
    // throttle are in poweer percent so -1000 ... +1000
    SRV_Channels::set_angle(_throttle_idx, 1000);

    // steering are limited to 
    SRV_Channels::set_angle(_steering_idx, 1000);
}

void AP_SCU::setup_pwm_type()
{
    
}

void AP_SCU::setup_safety_output()
{
    // stop sending pwm if main CPU fails
    SRV_Channels::set_failsafe_limit(_throttle_idx, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_rc_frequency(_throttle_idx, 0);

    // steering control with direction
    SRV_Channels::set_trim_to_min_for(_steering_idx,true);
    SRV_Channels::set_failsafe_limit(_steering_idx, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_rc_frequency(_steering_idx, 0);
}


// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
void AP_SCU::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	SRV_Channels::move_servo((SRV_Channel::Aux_servo_function_t)function_idx, angle, angle_min, angle_max);
}


namespace AP {
    AP_SCU *scu()
    {
        return AP_SCU::get_singleton();
    }
};

#endif