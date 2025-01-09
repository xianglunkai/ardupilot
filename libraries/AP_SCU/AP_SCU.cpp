#include "AP_SCU.h"
#if AP_SCU_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
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
    AP_GROUPINFO("CCW_ON", 1, AP_SCU, steering_relay_on, 0),

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

    // @Param: 
    // @DisplayName: 
    // @Description: 
    // @User: 
    // @Units: 
    // @Range: 
    AP_SUBGROUPINFO(_engine, "ENG_", 4,  AP_SCU, AP_SCU_Engine),

    // @Param: 
    // @DisplayName: 
    // @Description: 
    // @User: 
    // @Units: 
    // @Range: 
    AP_SUBGROUPINFO(_steering, "STR_", 5,  AP_SCU, AP_SCU_Steering),

    // @Param: _steering_idx
    // @DisplayName: steering Servo Output Function
    // @Description: steering Servo Output Function
    // @Values: SRV_Channel::k_motor1
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("STR_FN", 6, AP_SCU, _steering_idx, SRV_Channel::k_motor1),

    // @Param: _throttle_idx
    // @DisplayName: throttle Servo Output Function
    // @Description: throttle Servo Output Function
    // @Values: SRV_Channel::k_motor3
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("THR_FN", 6, AP_SCU, _throttle_idx, SRV_Channel::k_motor3),


    AP_GROUPEND
};



// singleton instance
AP_SCU *AP_SCU::_singleton;

AP_SCU::AP_SCU()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}


// init - performs any required initialisation for this instance
void AP_SCU::init()
{
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
    _last_update_ms = now;

    //1. engine start and stop control
 
    uint16_t cvalue = 1500;
    uint16_t sts_cmd = 0;

    // start command
    RC_Channel *c_start = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARMDISARM);
    if (c_start != nullptr && rc().has_valid_input()) {
        // get starter control channel
        cvalue = c_start->get_radio_in();
     
        if (cvalue >= RC_Channel::AUX_PWM_TRIGGER_HIGH) {
            sts_cmd = static_cast<uint16_t>(Engine_Cmd::REQ_START);
                    
              GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Staring Enginer!");

        } else if (cvalue <= RC_Channel::AUX_PWM_TRIGGER_LOW) {
           sts_cmd = static_cast<uint16_t>(Engine_Cmd::REQ_STOP);
              GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Stoping Enginer!");
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
    move_servo((SRV_Channel::Aux_servo_function_t)_throttle_idx.get(), throttle_out, -1000, +1000);
    // set frequency
    SRV_Channels::set_rc_frequency((SRV_Channel::Aux_servo_function_t)_throttle_idx.get(), throttle_pwm_freq_khz * 1000);

    // 3. steering control
    const float steering = constrain_float(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_steering), -1.0f, 1.0f);
    uint16_t steering_pos = 0;
    bool steerig_pos_valid = true;

#if AP_CANIO_ENABLE
    AP_CANIO *can_io = AP::can_io();
    if (can_io == nullptr) {
        steerig_pos_valid = false;
    }
    if(!can_io->read(AP_CANIO_Params::FUNCTION::STEER_ANG, steering_pos)) {
        steerig_pos_valid = false;
    }

#endif

    if (steerig_pos_valid == false) {
        SRV_Channels::set_rc_frequency((SRV_Channel::Aux_servo_function_t)_steering_idx.get(), 0);
        return;
    }

    const float steering_out = _steering.steering_angle_control(steering, steering_pos);
    // control steering direction
    AP_Relay *ap_relay = AP::relay();
    if (ap_relay == nullptr) {
        return;
    }
    bool on = is_positive(steering_out) ? steering_relay_on : !steering_relay_on;
    ap_relay->set(AP_Relay_Params::FUNCTION::STERT_DIR, on);

    // ouput PWM duty = 50%
    move_servo((SRV_Channel::Aux_servo_function_t)_steering_idx.get(), 0, -1000, 1000);
    // set frequency
    uint16_t freq = steering_pwm_freq_khz * 1000 * abs(steering_out);
    SRV_Channels::set_rc_frequency((SRV_Channel::Aux_servo_function_t)_steering_idx.get(), freq);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "steering pos:%d, freq:%d", steering_pos, freq);
}


void AP_SCU::setup_servo_output()
{
    // throttle are in poweer percent so -1000 ... +1000
    SRV_Channels::set_angle((SRV_Channel::Aux_servo_function_t)_throttle_idx.get(), 1000);

    // steering are limited to 
    SRV_Channels::set_angle((SRV_Channel::Aux_servo_function_t)_steering_idx.get(), 1000);
}

void AP_SCU::setup_pwm_type()
{
    
}

void AP_SCU::setup_safety_output()
{
    // stop sending pwm if main CPU fails
    SRV_Channels::set_failsafe_limit((SRV_Channel::Aux_servo_function_t)_throttle_idx.get(), SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_rc_frequency((SRV_Channel::Aux_servo_function_t)_throttle_idx.get(), 0);

    // steering control with direction
    SRV_Channels::set_trim_to_min_for((SRV_Channel::Aux_servo_function_t)_steering_idx.get(),true);
    SRV_Channels::set_failsafe_limit((SRV_Channel::Aux_servo_function_t)_steering_idx.get(), SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_rc_frequency((SRV_Channel::Aux_servo_function_t)_steering_idx.get(), 0);
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