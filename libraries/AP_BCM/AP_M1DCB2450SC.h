#pragma once

#include "AP_BCM_config.h"

#if AP_M1DCB2450SC_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

class AP_M1DCB2450SC {
public:

    // constructor
    AP_M1DCB2450SC();

    CLASS_NO_COPY(AP_M1DCB2450SC);

    // initialise driver
    void init();

    // returns true if communicating with the motor
    bool healthy();

    // send target motor speed
    void send_motor_speed_cmd(const int16_t speed);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_M1DCB2450SC *get_singleton() { return _singleton; }

private:

    static AP_M1DCB2450SC *_singleton;

    // consume incoming messages from motor, reply with latest motor speed
    // runs in background thread
    void thread_main();

    // initialise serial port and gpio pins (run from background thread)
    // returns true on success
    bool init_internals();

    // members
    AP_HAL::UARTDriver *_uart;      // serial port to communicate with motor
    bool _initialised;              // true once driver has been initialised

    static constexpr uint32_t M1DCB2450SC_SERIAL_BAUD = 19200;
    static constexpr uint32_t M1DCB2450SC_ADDRESS = 0x01;
    static constexpr uint32_t M1DCB2450SC_SEND_MOTOR_SPEED_INTERVAL_MS = 300;
    static constexpr uint32_t M1DCB2450SC_SEND_MOTOR_STATUS_REQUEST_INTERVAL_MS = 1000;


    // Motor specific message ids
    enum class MotorMsgId : uint8_t {
        INFO = 0x00,
        STATUS = 0x01,
        CONFIG_MODE = 0x03,
        CONFIG_ADDR = 0x04,
        DRIVE = 0x82
    };

    // function codes
    enum class FunctionCode : uint8_t {
        READ_HOLDING_REGISTER = 0x03,
        READ_INPUT_REGISTER = 0x04,
        WRITE_HOLDING_REGISTER = 0x06,
        WRITE_MULTIPLE_REGISTER = 0x10,
        READ_WRITE_MULTIPLE_REGISTER = 0x17
    };

    // register address
    enum class RegisterAddress : uint16_t {
        WORK_MODE= 0x0000,    
        ADDRESS = 0x0001,
        BUADRATE = 0x0002,
        CONTROL_CODE = 0x0003,
        SET_MOTOR_SPEED = 0x0004,
        GET_MOTOR_SPEED = 0x0005,   
        SET_PWM_FREQ = 0x0006,  
        SET_SOFT_START_TIME = 0x0007,  
        SET_SOFT_STOP_TIME = 0x0008,
        READ_LIMITED_CODE = 0x000D    
    };


    enum class WorkMode : uint16_t {
        CONTROL = 0x00,
    };

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_ADDRESS,
        WAITING_FOR_FUNCTION_CODE,
        WAITING_FOR_PAYLOAD_LEN,
        WAITING_FOR_PAYLOAD,
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH,
    };


    // set target motor speed 01 06 00 04 xxH xxL CRCH CRCL
    void set_motor_speed();

    // configure address 01 06 00 01 00 02 CRCH CRCL
    void set_device_address();

    // set control mode 01 06 00 00 00 00 CRCH CRCL 
    void set_work_mode();

    void read_status();

    // process a single byte received on serial port
    // return true if a complete message has been received (the message will be held in _received_buff)
    bool parse_byte(uint8_t b);

    // process message held in _received_buff
    void parse_message();

    // returns true if it is safe to send a message
    bool safe_to_send() const { return ((_send_delay_us == 0) && (_reply_wait_start_ms == 0)); }

    // check for timeout after sending a message and unset pin if required
    void check_for_send_end();

    // calculate delay required to allow message to be completely sent
    uint32_t calc_send_delay_us(uint8_t num_bytes);

    // record msgid of message to wait for and set timer for reply timeout handling
    void set_expected_reply_msgid(uint8_t msg_id);

    // check for timeout waiting for reply
    void check_for_reply_timeout();

    // mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
    void set_reply_received();

    // structure holding latest message contents
    // the order of fields matches the incoming message so it can be used to calculate the crc
    struct PACKED {
        uint8_t address;                            // device address (required for calculating crc)
        uint8_t function_code;                      // function code (always 0x04 but required for calculating crc)
        uint8_t payload_len;                        // message payload length
        uint8_t payload[16*2];  // payload
        uint16_t crc;                               // latest message's crc
        uint16_t payload_recv;                      // number of message's payload bytes received so far
        ParseState state;                           // state of incoming message processing
    } parsed_msg;

    // message
    bool _left_limited{false};
    bool _right_limited{false};


     // health reporting
    HAL_Semaphore _last_healthy_sem;// semaphore protecting reading and updating of _last_send_motor_ms and _last_received_ms
    uint32_t _last_received_ms;     // system time (in millis) that a message was successfully parsed (for health reporting)
    uint32_t _last_send_motor_ms;
    uint32_t _last_send_motor_status_request_ms;

    int16_t _motor_speed_cmd {0};

    uint32_t _send_start_us;        // system time (in micros) when last message started being sent (used for timing to unset DE pin)
    uint32_t _send_delay_us;        // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying
    // reply message handling
    uint8_t _reply_msgid;           // replies expected msgid (reply often does not specify the msgid so we must record it)
    uint32_t _reply_wait_start_ms;  // system time that we started waiting for a reply message
    bool _send_motor_speed {false};         // true if motor speed should be sent at next opportunity
    bool _command_changed {false};


    // enable library
    AP_Int8 _enable;
    // device address
    AP_Int16 _address;
    // limited
    AP_Int16 _limit;

};

namespace AP {
    AP_M1DCB2450SC *m1dcb2450sc();
};



#endif