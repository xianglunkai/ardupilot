/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * AP_ESSA.cpp
 *
 *      Author: xiang lunkai
 */

#include "AP_ESSA.h"

#if AP_ESSA_ENABLED
#include <stdio.h>
#include <string.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/AP_Math.h>    
#include <RC_Channel/RC_Channel.h>
#include <AP_Relay/AP_Relay.h>

#if 0
  #include <GCS_MAVLink/GCS.h>
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

extern const AP_HAL::HAL & hal;


// parameters
const AP_Param::GroupInfo AP_ESSA::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Engine control
    // @Description: This enables internal combustion engine control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_ESSA, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CTL_CLASS
    // @DisplayName: propeller control class
    // @Description: 0 : diff 1: sync 2: debug 
    AP_GROUPINFO("CTL_CLASS", 1, AP_ESSA, _control_class, CTL_CLASS_DIFF),

    // @Param: GEAR_DZ
    // @DisplayName: gear control deadzone, range[0, 100]
    // @Description: protect motor gear switch from F/R to R/F
    AP_GROUPINFO("GEAR_DZ", 2, AP_ESSA, _gear_dz, 5),

    // @Param: STARTER_TIME
    // @DisplayName: Time to run starter
    // @Description: This is the number of seconds to run the starter when trying to start the engine
    // @User: Standard
    // @Units: s
    // @Range: 0.1 5
    AP_GROUPINFO("START_TIME", 3, AP_ESSA, _starter_time, 3),

    // @Param: START_DELAY
    // @DisplayName: Time to wait between starts
    // @Description: Delay between start attempts
    // @User: Standard
    // @Units: s
    // @Range: 1 10
    AP_GROUPINFO("START_DELAY", 4, AP_ESSA, _starter_delay, 2),


    AP_GROUPEND
};

// singleton instance
AP_ESSA *AP_ESSA::_singleton;

AP_ESSA::AP_ESSA()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ESSA must be singleton");
    }
#endif
    _singleton = this;
}


void AP_ESSA::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::ESSA) {
            _driver = NEW_NOTHROW AP_ESSA_Driver();
            return;
        }
    }

}


void AP_ESSA::update()
{
    // exit if not enabled
    if (_driver == nullptr || !_enable) {
        return;
    }

   const uint64_t now = AP_HAL::millis64();

    AP_Relay *relay = AP::relay();
    if (relay == nullptr) {
       return;
    }

    // delay start enginer
    if (!_delay_start) {
        _delay_start = true;
        _starter_delay_run_ms = now;
    }
    if (now - _starter_delay_run_ms <= _starter_delay * 1000) {
        relay->set(AP_Relay_Params::FUNCTION::RELAY, false);
        return;
    }

    // start enginer 
    if (!_initialized ) {
        _starter_last_run_ms = now;
        _initialized = true;
    }

    if ((now  - _starter_last_run_ms) <= _starter_time * 1000) {
        relay->set(AP_Relay_Params::FUNCTION::RELAY, true);
    } else {
        relay->set(AP_Relay_Params::FUNCTION::RELAY, false);
    }

    // throttle ,steering control
    _driver->update(_control_class, _gear_dz);
}



// get latest battery status info.  returns true on success and populates arguments
bool AP_ESSA::get_batt_info(uint8_t instance, float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const
{

    // use battery info from display_system_state if available 
    if ((AP_HAL::millis() - _driver->_bms_last_update_ms[instance]) <= ESSA_BATT_TIMEOUT_MS) {

        voltage = _driver->_bms_msg_raw[instance].data.bms_voltage * 0.1f;
        current_amps = _driver->_bms_msg_raw[instance].data.bms_current * 0.1f - 3000;
        temp_C = 0;
        pct_remaining = _driver->_bms_msg_raw[instance].data.bms_soc;
        return true;
    }

    return false;
}

bool AP_ESSA::get_batt_capacity_Ah(uint8_t instance, uint16_t &amp_hours) const
{
    return false;
}


namespace AP {
    AP_ESSA *essa()
    {
        return AP_ESSA::get_singleton();
    }
};


AP_ESSA_Driver::AP_ESSA_Driver() : CANSensor("ESSA")
{
    register_driver(AP_CAN::Protocol::ESSA);

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ESSA_Driver::loop, void), "ESSA", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}


// parse inbound frames
void AP_ESSA_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    switch (CAN_ID(id)) {

        case CAN_ID::PROP_FDB_PORT:
        {
            memcpy(_prop_fdb_msg_raw[0].raw_data, frame.data, frame.dlc);
            break;
        }
        case CAN_ID::PROP_FDB_STBD:
        {
            memcpy(_prop_fdb_msg_raw[1].raw_data, frame.data, frame.dlc);
            break;
        }

        case CAN_ID::STEER_FDB_ANG:
        {
            memcpy(_steer_fdb_msg_raw.raw_data, frame.data, frame.dlc);
            break;
        }

        case CAN_ID::MOTOR_DRIVER_PORT:
        {
            const uint8_t idx = 0;
            memcpy(_motor_driver_msg_raw[idx].raw_data, frame.data, frame.dlc);

            const float esc_tempC = _motor_driver_msg_raw[idx].data.motor_mos_temp - 40.0f;
            const float voltage = _motor_driver_msg_raw[idx].data.motor_bus_voltage * 0.1f;
            const float current_amps = _motor_driver_msg_raw[idx].data.motor_bus_current * 0.1f - 500;
            const float motor_tempC = _motor_driver_msg_raw[idx].data.motor_temp - 40.0f;

            update_esc_telem(idx, voltage, current_amps, esc_tempC, motor_tempC);

            break;
        }
        case CAN_ID::MOTOR_RPM_PORT:
        {
            const uint8_t idx = 0;
            memcpy(_motor_speed_msg_raw[idx].raw_data, frame.data, frame.dlc);

            const float rpm = _motor_speed_msg_raw[idx].data.motor_rpm - 32768;

            update_esc_rpm(idx, rpm);

            break;
        }

        case CAN_ID::MOTOR_DRIVER_STBD:
        {
            const uint8_t idx = 1;
            memcpy(_motor_driver_msg_raw[idx].raw_data, frame.data, frame.dlc);

            const float esc_tempC = _motor_driver_msg_raw[idx].data.motor_mos_temp - 40.0f;
            const float voltage = _motor_driver_msg_raw[idx].data.motor_bus_voltage * 0.1f;
            const float current_amps = _motor_driver_msg_raw[idx].data.motor_bus_current * 0.1f - 500;
            const float motor_tempC = _motor_driver_msg_raw[idx].data.motor_temp - 40.0f;

            update_esc_telem(idx, voltage, current_amps, esc_tempC, motor_tempC);

            break;
        }

        case CAN_ID::MOTOR_RPM_STABD:
        {
            const uint8_t idx = 1;
            memcpy(_motor_speed_msg_raw[idx].raw_data, frame.data, frame.dlc);

            const float rpm = _motor_speed_msg_raw[idx].data.motor_rpm - 32768;

            update_esc_rpm(idx, rpm);

            break;
        }

        case CAN_ID::STEER_POS_PORT:
        {
            break;
        }
        case CAN_ID::STEER_POS_STBD:
        {
            break;
        }


        case CAN_ID::LIFT_POS_PORT:
        {
            const uint8_t idx = 0;
            memcpy(_lift_fdb_msg_raw[idx].raw_data, frame.data, frame.dlc);

            break;
        }
        case CAN_ID::LIFT_POS_STBD:
        {
            const uint8_t idx = 1;
            memcpy(_lift_fdb_msg_raw[idx].raw_data, frame.data, frame.dlc);

            break;
        }

        case CAN_ID::BMS_STATUS_PORT:
        {
            _bms_last_update_ms[0] = AP_HAL::millis();
            memcpy(_bms_msg_raw[0].raw_data, frame.data, frame.dlc);

            break;
        }
        case CAN_ID::BMS_STATUS_STBD:
        {
            _bms_last_update_ms[1] = AP_HAL::millis();
            memcpy(_bms_msg_raw[1].raw_data, frame.data, frame.dlc);

            break;
        }

        case CAN_ID::ESSA_CTL_STATUS:
        {
            memcpy(_essa_sys_msg_raw.raw_data, frame.data, frame.dlc);
            break;
        }

        case CAN_ID::PROP_CTL_CMD:
        {
            break;
        }

        case CAN_ID::PROP_CTL_MODE:
        {
            break;
        }

    }

}

// send ESC telemetry
void AP_ESSA_Driver::update_esc_telem(uint8_t telem_esc_index, float voltage, float current_amps, float esc_tempC, float motor_tempC)
{
#if HAL_WITH_ESC_TELEM

    // fill in telemetry data structure
    AP_ESC_Telem_Backend::TelemetryData telem_dat {};
    telem_dat.temperature_cdeg = esc_tempC * 100;   // temperature in centi-degrees
    telem_dat.voltage = voltage;                    // voltage in volts
    telem_dat.current = current_amps;               // current in amps
    telem_dat.motor_temp_cdeg = motor_tempC * 100;  // motor temperature in centi-degrees

    // send telem and rpm data
    update_telem_data(telem_esc_index, telem_dat, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                                                  AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE |
                                                  AP_ESC_Telem_Backend::TelemetryType::CURRENT |
                                                  AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);
#endif
}

void AP_ESSA_Driver::update_esc_rpm(uint8_t telem_esc_index, float rpm)
{
#if HAL_WITH_ESC_TELEM

    update_rpm(telem_esc_index, rpm);

#endif
}


void AP_ESSA_Driver::update(const int8_t control_class, const int16_t gear_dz)
{
    // set default control mode
    prop_mode_cmd_t mode_select;
    reset_control_mode(mode_select);

    // set default control commands
    prop_ctl_cmd_t control_cmd;
    reset_control_cmd(control_cmd);
  
    uint16_t lift_open = 0;

    if(!hal.util->get_soft_armed()) {
        const int16_t lift = SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_rcin4_mapped) * 1000.0;
        if (lift > 1000 || lift < -1000)
            lift_open = 0;
        else 
            lift_open = lift > gear_dz ? 1: ((lift < -gear_dz ) ? 2 : 0);
            
    }

    if (control_class == CTL_CLASS_DIFF) {

        const int16_t throttle_port = constrain_int16(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_throttleLeft) * 1000.0, -1000, 1000);
        const int16_t throttle_stbd = constrain_int16(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_throttleRight) * 1000.0, -1000, 1000);
       
        const uint16_t throttle_open_port = (uint32_t)(abs(throttle_port)) * 65535 / 1000 + 0;
        const uint16_t throttle_open_stbd = (uint32_t)(abs(throttle_stbd)) * 65535 / 1000 + 0;

        const uint16_t gear_open_port = throttle_port > gear_dz ? 1: ((throttle_port < -gear_dz) ? 2 : 0);
        const uint16_t gear_open_stbd = throttle_stbd > gear_dz ? 1: ((throttle_stbd < -gear_dz) ? 2 : 0);

        const uint16_t steering_open = 1024;

        control_cmd.data.throttle_open_port = throttle_open_port;
        control_cmd.data.throttle_open_stbd = throttle_open_stbd;
        control_cmd.data.steer_open = steering_open;
        control_cmd.data.gear_open_port = gear_open_port;
        control_cmd.data.gear_open_stbd = gear_open_stbd;
        control_cmd.data.lift_open_port = lift_open;
        control_cmd.data.lift_open_stbd = lift_open;

    } else if (control_class == CTL_CLASS_SYNC) {

        const int16_t throttle = constrain_int16(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_throttle) * 1000.0, -1000, 1000);
        const uint16_t throttle_open = (uint32_t)(abs(throttle)) * 65535 / 1000 + 0;

        const uint16_t gear_open = throttle > gear_dz ? 1: ((throttle < -gear_dz ) ? 2 : 0);

        const int16_t steering = constrain_int16(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_steering) * 1000.0, -1000, 1000);
        const uint16_t steering_open = (uint32_t)(steering) * 1024 / 1000 + 1024;

        control_cmd.data.throttle_open_port = throttle_open;
        control_cmd.data.throttle_open_stbd = throttle_open;
        control_cmd.data.steer_open = steering_open;
        control_cmd.data.gear_open_port = gear_open;
        control_cmd.data.gear_open_stbd = gear_open;
        control_cmd.data.lift_open_port = lift_open;
        control_cmd.data.lift_open_stbd = lift_open;

    } else if (control_class == CTL_CLASS_DEBUG) {
        control_cmd.data.throttle_open_port = 0x0200;
        control_cmd.data.throttle_open_stbd = 0;
        control_cmd.data.steer_open = 0x0800;
        control_cmd.data.gear_open_port = 0x01;
        control_cmd.data.gear_open_stbd = 0;
        control_cmd.data.lift_open_port = 0x01;
        control_cmd.data.lift_open_stbd = 0;
    }

    // move commands
    _control_cmd = std::move(control_cmd);
    _mode_select = std::move(mode_select);
    _new_command = true;

}

void AP_ESSA_Driver::reset_control_mode(prop_mode_cmd_t & mode)
{
    // enable auto control
    mode.data.ctl_mode_select = 1;
    // disable turbo mode
    mode.data.turbo_mode_select = 0;
}

void AP_ESSA_Driver::reset_control_cmd(prop_ctl_cmd_t & cmd)
{
    cmd.data.throttle_open_port = 0;
    cmd.data.throttle_open_stbd = 0;

    cmd.data.gear_open_port =  static_cast<uint64_t>(Gear_Pos::N);
    cmd.data.gear_open_stbd =  static_cast<uint64_t>(Gear_Pos::N);

    cmd.data.steer_open = 1024;

    cmd.data.lift_open_port = 0;
    cmd.data.lift_open_stbd = 0;
}

void AP_ESSA_Driver::loop()
{
    uint16_t mode_tx_counter = 0;
    uint16_t cmd_tx_counter = 0;

    const uint16_t cmd_tx_counter_limit = CTL_CMD_INTERVAL_US / SCHED_LOOP_US;
    const uint16_t mode_tx_counter_limit = CTL_MODE_INTERVAL_US / SCHED_LOOP_US;

    while (true) { 
      
       // 50ms loop delay
        hal.scheduler->delay_microseconds(SCHED_LOOP_US); 
    
        if (_new_command ) {
            _new_command = false;

            // Transmit engine control mode commands at regular intervals
            if (++mode_tx_counter >= mode_tx_counter_limit) {
                mode_tx_counter = 0;
                send_mode_select_messages(_mode_select);
            }

            // Transmit engine control commands at regular intervals
            if (++cmd_tx_counter >= cmd_tx_counter_limit) {
                cmd_tx_counter = 0;
                send_control_cmd_messages(_control_cmd);
            }
        }
    }
}

// send control mode over CAN
void AP_ESSA_Driver::send_mode_select_messages(const prop_mode_cmd_t & mode_select)
{
    AP_HAL::CANFrame txFrame;
    uint32_t id = (uint32_t)(CAN_ID::PROP_CTL_MODE);
    txFrame = {(id |= AP_HAL::CANFrame::FlagEFF), mode_select.raw_data, sizeof(mode_select.raw_data)};
    write_frame(txFrame, AP_HAL::micros64() + 1000ULL);
}

// send control mode over CAN
void AP_ESSA_Driver::send_control_cmd_messages(const prop_ctl_cmd_t & control_cmd)
{
    AP_HAL::CANFrame txFrame;
    uint32_t id = (uint32_t)(CAN_ID::PROP_CTL_CMD);
    txFrame = {(id |= AP_HAL::CANFrame::FlagEFF), control_cmd.raw_data, sizeof(control_cmd.raw_data)};
    write_frame(txFrame, AP_HAL::micros64() + 1000ULL);
}


#endif