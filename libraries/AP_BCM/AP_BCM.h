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

#include <AP_BCM/AP_BCM_config.h>

#if AP_BCM_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

class AP_BCM_Driver : public CANSensor, public AP_ESC_Telem_Backend
{
public:
    AP_BCM_Driver();

    void update(const int16_t gear_dz);

    // message CAN ID
    enum class CAN_ID : uint32_t {
        BCM_TO_ECU_CMD       = 0x1e0, // BCM to ECU command
        BCM_TO_ECU_ACK       = 0x7fe, // BCM to ECU ack
        ECU_TO_BCM_STS       = 0x303, // ECU to BCM status
        ECU_TO_BCM_REQ       = 0x7e3, // ECU to BCM request
    };

    // gear open position
    enum class Gear_Pos: uint16_t {
        N = 0,
        F = 1,
        R = 2,
    };
    
    // engine start/stop control command
    enum class Engine_Cmd: uint16_t {
        NO_OPS    = 0,
        REQ_START = 1,
        REQ_STOP  = 2
    };

    // lift down position
    enum class Lift_Pos: uint16_t {
        NORMAL = 0,
        UP     = 1,
        DOWN   = 2,
    };

    // some constants
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME1_HEAD = 0x00;
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME2_HEAD = 0x04;
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME3_HEAD = 0x08;
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME1_LOOP_TIME_MS = 5; // 5t
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME2_LOOP_TIME_MS = 5; // 5t + 1
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME3_LOOP_TIME_MS = 20; // 20t + 4
    static constexpr int16_t BCM_TO_ECM_CMD_FRAME1_THR_FORWARD_MAX = 0x32f;
    static constexpr int16_t BCM_TO_ECM_CMD_FRAME1_THR_BACKWARD_MAX = 0x267;
    static constexpr int16_t BCM_TO_ECM_CMD_FRAME1_THR_IDEL = 0x0cd;
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME3_GEAR_IDEL = 0x6e91;
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME3_GEAR_FORWARD = 0x8878;
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME3_THR_FORWARD_MAX = 0xe61a;
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME3_GEAR_BACKWARD = 0x56a9;
    static constexpr uint16_t BCM_TO_ECU_CMD_FRAME3_THR_BACKWARD_MAX = 0x19e7;

    // message CAN data using union and struct with packed attribute
    union PACKED BCM_TO_ECU_CMD_Data {
        struct {
            uint64_t frame1_head:8; // default 0x00

            // control: {
            // throttle_cmd:10; // default 0x0cd range [0x267, 0x32f]
            // gear_cmd:2; // default 0x00 
            // lift_cmd:2; // default 0x00
            // sts_cmd:2; // default 0x00
            //}
            uint64_t control:16;

            // control2: {
            // throttle_cmd:10; // default 0x0cd range [0x267, 0x32f]
            // gear_cmd:2; // default 0x00 
            // lift_cmd:2; // default 0x00
            // sts_cmd:2; // default 0x00
            //}
            uint64_t control2:16;

            // control3: {
            // throttle_cmd:10; // default 0x0cd range [0x267, 0x32f]
            // gear_cmd:2; // default 0x00 
            // lift_cmd:2; // default 0x00
            // sts_cmd:2; // default 0x00
            //}
            uint64_t control3:16;

            uint64_t byte7:8; // default 0x00
        } data1 = {
            .frame1_head = 0x00,
            .control = 0xcd00,
            .control2 = 0xcd00,
            .control3 = 0xcd00,
            .byte7 = 0x80,
        };

        struct {
            uint64_t frame2_head:8; // default 0x04

            // control: {
            // throttle_cmd:10; // default 0x0cd range [0x267, 0x32f]
            // gear_cmd:2; // default 0x00 
            // lift_cmd:2; // default 0x00
            // sts_cmd:2; // default 0x00
            //}
            uint64_t control:16;

            uint64_t reserved:40; // default 0x00
        } data2;

        struct {
            uint64_t frame3_head:8; // default 0x08
            uint64_t byte1_2:16; // default 0x0000
            uint64_t byte3:8; // default 0x00
            uint64_t byte4:8; // default 0x00
            uint64_t byte5:8; // default 0x00/0x004 circle runing
            uint64_t byte6:8; // default 0x61
            uint64_t byte7:8; // default 0x08
        } data3;

        uint8_t raw_data[8];
    };

    union PACKED ECU_TO_BCM_STS_Data {
        struct {
            uint64_t frame_head:8; // default 0x00
            uint64_t rpm:16; // default 0x00 rpm = x * 4
            uint64_t reserved1:8; // default 0x00
            uint64_t throttle_pos:8; // pos = x * 0.5 (%)
            uint64_t reserved2:16; // default 0x00
            uint64_t reserved3:6; // default 0x00
            uint64_t gear_pos:2; // default 0x00 0:N, 1:F, 2:R
        } data;

        uint8_t raw_data[8];
    };

    union PACKED ECU_TO_BCM_REQ_Data {
        struct {
            uint64_t byte0:8; // default 0x80
            uint64_t byte1:8; // default 0x27
            uint64_t byte2:8; // default 0x45
            uint64_t byte3:8; // default 0x63
        } data = {
            .byte0 = 0x80,
            .byte1 = 0x27,
            .byte2 = 0x45,
            .byte3 = 0x63,
        };

        uint8_t raw_data[4];
    };

    union PACKED BCM_TO_ECU_ACK_Data {
        struct {
            uint64_t byte0:8; // default 0x80
            uint64_t byte1:8; // default 0x04
            uint64_t byte2:8; // default 0x01

            uint64_t byte3:8; // byte3 = ECU_TO_BCM_REQ_Data.byte1
            uint64_t byte4:8; // byte4 = ECU_TO_BCM_REQ_Data.byte2
            uint64_t byte5:8; // byte5 = ECU_TO_BCM_REQ_Data.byte3
        }data = {
            .byte0 = 0x80,
            .byte1 = 0x04,
            .byte2 = 0x01,
            .byte3 = 0x27,
            .byte4 = 0x45,
            .byte5 = 0x63,
        };
        uint8_t raw_data[6];
    };

    // BCM to ECU command
    BCM_TO_ECU_CMD_Data _bcm_to_ecu_cmd; 
    BCM_TO_ECU_CMD_Data _bcm_to_ecu_cmd2;
    BCM_TO_ECU_CMD_Data _bcm_to_ecu_cmd3;

    // ECU to BCM status
    ECU_TO_BCM_STS_Data _ecu_to_bcm_sts;
    // ECU to BCM request
    ECU_TO_BCM_REQ_Data _ecu_to_bcm_req;
    // BCM to ECU ack
    BCM_TO_ECU_ACK_Data _bcm_to_ecu_ack;

    // handshake flag
    bool _handshake_success{false};

    bool _received_ecu_request{false};

    bool _received_ecu_status{false};
    uint32_t _update_ecu_status_ms {0};

    uint64_t _clock_counter {0};
    uint16_t _change_counter{0};
    uint16_t _counter{0};

    // allow threads to lock against baro update
    HAL_Semaphore &get_semaphore(void) {
        return _rsem;
    }

private:

    HAL_Semaphore _rsem;            // semaphore for multi-thread 

    // handle for incoming frames
    void handle_frame(AP_HAL::CANFrame & frame) override;

    // loop to send output to ESCs in background thread
    void loop();

    void timer_update();
        
    // update engine speed
    void update_esc_rpm(uint8_t telem_esc_index, float rpm);

    // send BCM to ECU ACK
    void send_handshake_ack_messages(const BCM_TO_ECU_ACK_Data & mode_select);

    // send an engine control commands over can
    void send_control_cmd_messages(const BCM_TO_ECU_CMD_Data & control_cmd);
};

class AP_BCM {
public:

    AP_BCM();


    /* Do not allow copies */
    CLASS_NO_COPY(AP_BCM);

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    // add interface for AP_SCU
    void set_control_commands(const uint16_t throttle, const uint8_t gear, const uint8_t lift, const uint8_t sts);

    static AP_BCM *get_singleton() { return _singleton; }

private:

    static AP_BCM *_singleton;

    // communication driver
    AP_BCM_Driver *_driver;

    // enable library
    AP_Int8 _enable;

    // dynamics system control deadzone
    AP_Int16 _gear_dz;
};


namespace AP {
    AP_BCM *bcm();
};

#endif
