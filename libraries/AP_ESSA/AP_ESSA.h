/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: xiang lunkai
 */

#pragma once

#include <AP_ESSA/AP_ESSA_config.h>

#if AP_ESSA_ENABLED
#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

// constansts
static constexpr uint8_t ESSA_MAX_NUM = 2;
static constexpr uint32_t SCHED_LOOP_US = 50000U;
static constexpr uint32_t CTL_CMD_INTERVAL_US = 50000U;
static constexpr uint32_t CTL_MODE_INTERVAL_US = 500000U;

static constexpr uint8_t CTL_CLASS_DIFF = 0;
static constexpr uint8_t CTL_CLASS_SYNC = 1;
static constexpr uint8_t CTL_CLASS_DEBUG = 2;
static constexpr uint32_t ESSA_BATT_TIMEOUT_MS = 2500; 

class AP_ESSA_Driver : public CANSensor, public AP_ESC_Telem_Backend
{
public:
    
    AP_ESSA_Driver();

    // called from SRV_Channels
    void update(const int8_t control_class, const int16_t gear_dz);

    // message CAN ID
    enum class CAN_ID : uint32_t {
        
        PROP_FDB_PORT       = 0x15000020,
        PROP_FDB_STBD       = 0x15000040,

        STEER_FDB_ANG       = 0x16000020,

        MOTOR_DRIVER_PORT   = 0x11000030,
        MOTOR_RPM_PORT      = 0x11000031,

        MOTOR_DRIVER_STBD   = 0x11000060,
        MOTOR_RPM_STABD     = 0x11000061,

        STEER_POS_PORT      = 0x12000032,
        STEER_POS_STBD      = 0x12000062,

        LIFT_POS_PORT       = 0x13000031,
        LIFT_POS_STBD       = 0x13000061,

        BMS_STATUS_PORT     = 0x14000020,
        BMS_STATUS_STBD     = 0x14000040,

        PROP_CTL_CMD        = 0x1AF00000,

        PROP_CTL_MODE       = 0x1AF00001,

        ESSA_CTL_STATUS     = 0x10000030,
    };

    // gear open position
    enum class Gear_Pos {
        N = 0,
        F = 1,
        R = 2,
    };

    // CAN matrix table
    union prop_fdb_msg_t {
        struct {
            uint64_t throttle_open:16;          // 0-65535 --> 0-100
            uint64_t gear_open:2;               // 0: N; 1: F 2: R
            uint64_t throttle_iszero:1;         // 0: invalid, 1: valid
            uint64_t reserved:5;
        } data = {};

        uint8_t raw_data[3];
    };

    union steer_fdb_msg_t
    {
        struct {
            uint64_t steer_open:16;             // x := (x - 0x400(1024)) * 450 / (1024 * 10)
        } data = {};

        uint8_t raw_data[2];
    };

    union motor_drive_msg_t
    {
        struct {
            uint64_t motor_mos_temp:8;          // x := (x - 40) deg
            uint64_t reserved:8;
            uint64_t motor_temp:8;              // x := (x - 40) deg
            uint64_t motor_bus_voltage:16;      // x := x * 0.1   V
            uint64_t motor_bus_current:16;      // x := (x * 0.1 - 500) A
            uint64_t motor_aux_bus_voltage:8;   // x := x * 0.1 V
        } data = {};

        uint8_t raw_data[8];
    };

    union motor_speed_msg_t
    {
        struct {
            uint64_t motor_rpm:16;              // x := (x - 32768) rpm
            uint64_t motor_dc_power:16;         // x := x w
        } data = {};

        uint8_t raw_data[4];
    };

    union steer_driver_msg_t
    {
        struct {
            uint64_t steer_cmd:16;             // x : = (x * 0.1 - 1800) deg
            uint64_t steer_fdb:16;             // x : = (x * 0.1- 1800) deg
        } data = {};

        uint8_t raw_data[4];
    };
    

    union lift_fdb_msg_t
    {
        struct {
            uint64_t lift_angle:8;              // x := (x - 4)
        } data = {};

        uint8_t raw_data[1];
        
    };
    
    union bms_msg_t
    {
        struct {
            uint64_t bms_voltage:16;            // x := x * 0.1 V
            uint64_t bms_current:16;            // x := (x * 0.1 - 3000)A
            uint64_t bms_soc:16;                // x := (x) * 0.1  %
            uint64_t bms_soh:16;                // x := (x) * 0.1  %
        } data = {};

        uint8_t raw_data[8];
    };
    
    union prop_ctl_cmd_t
    {
        struct {
            uint64_t throttle_open_port:16;
            uint64_t throttle_open_stbd:16;   
            uint64_t gear_open_port:2;       // 0: N 1:F 2:R
            uint64_t reserved1:2;              
            uint64_t gear_open_stbd:2;       // 0: N 1:F 2:R
            uint64_t reserved2:2;
            uint64_t steer_open:16;          // x := (x - 1024) * 450 / (1024 * 10) [-1024 1024]
            uint64_t lift_open_port:2;       // 0: invalid 1: up 2: down
            uint64_t reserved3:2;
            uint64_t lift_open_stbd:2;       // 0: invalid 1: up 2: down
            uint64_t reserved4:2;
        } data = {};

        uint8_t raw_data[8];
    };
    
    union prop_mode_cmd_t
    {
        struct {
            uint64_t ctl_mode_select:1;     // 0: manual 1: auto
            uint64_t turbo_mode_select:1;   // 0: disable 1: enable 
            uint64_t reserved:62;
        } data = {};

        uint8_t raw_data[8];
    };


    union essa_sys_msg_t
    {
       struct {
            uint64_t reserved:18;
            uint64_t sys_status:2;
            uint64_t reserved1:26;
            uint64_t throttle_iszero:1;
            uint64_t reserved2:1;
       } data = {};

       uint8_t raw_data[6];
    };
    

    // system time that system state was last updated
    uint32_t _bms_last_update_ms[ESSA_MAX_NUM];        

    // Propeller feedback message [PORT, STBD]
    prop_fdb_msg_t _prop_fdb_msg_raw[ESSA_MAX_NUM]; 

    // Steer  feedback message
    steer_fdb_msg_t _steer_fdb_msg_raw;

    // Motor diriver message[PORT, STBD]
    motor_drive_msg_t _motor_driver_msg_raw[ESSA_MAX_NUM];

    // Motor speed meesage [PORT STBD]
    motor_speed_msg_t _motor_speed_msg_raw[ESSA_MAX_NUM];

    // Steer driver message [PORT STBD]
    steer_driver_msg_t _steer_driver_msg_raw[ESSA_MAX_NUM];

    // lift feedback message [PORT STBD]
    lift_fdb_msg_t _lift_fdb_msg_raw[ESSA_MAX_NUM];

    // BMS feedback message [PORT STBD]
    bms_msg_t _bms_msg_raw[ESSA_MAX_NUM];

    // Essa system status;
    essa_sys_msg_t _essa_sys_msg_raw;

    prop_mode_cmd_t _mode_select;
    bool _new_command{false};

    prop_ctl_cmd_t _control_cmd;

private:

    // handle for incoming frames
    void handle_frame(AP_HAL::CANFrame & frame) override;

   // loop to send output to ESCs in background thread
    void loop();

    // update engine electric status
    void update_esc_telem(uint8_t telem_esc_index, float voltage, float current_amps, float esc_tempC, float motor_tempC);
    
    // update engine speed
    void update_esc_rpm(uint8_t telem_esc_index, float rpm);

    void reset_control_cmd(prop_ctl_cmd_t & cmd);

    void reset_control_mode(prop_mode_cmd_t & mode);

    // send control mode over CAN
    void send_mode_select_messages(const prop_mode_cmd_t & mode_select);

    // send an engine control commands over can
    void send_control_cmd_messages(const prop_ctl_cmd_t & control_cmd);

}; // class AP_ESSA_Driver


class AP_ESSA {
public:

    AP_ESSA();


    /* Do not allow copies */
    CLASS_NO_COPY(AP_ESSA);

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    // get latest battery status info.  returns true on success and populates arguments
    bool get_batt_info(uint8_t instance, float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const WARN_IF_UNUSED;
   
    bool get_batt_capacity_Ah(uint8_t instance, uint16_t &amp_hours) const;

    static AP_ESSA *get_singleton() { return _singleton; }

private:

    static AP_ESSA *_singleton;

    bool _initialized {false};
    bool _delay_start {false};

    // time when we control relay
    uint64_t _starter_last_run_ms {0};
    uint64_t _starter_delay_run_ms {0};

    // communication driver
    AP_ESSA_Driver *_driver;

    // enable library
    AP_Int8 _enable;

    // time to run starter for seconds
    AP_Float _starter_time;

    // delay between start attempts seconds
    AP_Float _starter_delay;

    // dynamics system allocation type
    AP_Int8 _control_class;

    // dynamics system control deadzone
    AP_Int16 _gear_dz;

};


namespace AP {
    AP_ESSA *essa();
};

#endif