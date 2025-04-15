#include "AP_M1DCB2450SC.h"

#if AP_M1DCB2450SC_ENABLED
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;


// parameters
const AP_Param::GroupInfo AP_M1DCB2450SC::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: AP_M1DCB2450SC ENABLE 
    // @Description: This enables AP_M1DCB2450SC driver
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_M1DCB2450SC, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ADDR
    // @DisplayName: device address
    // @Description: device address
    AP_GROUPINFO("ADDR", 1, AP_M1DCB2450SC, _address, (int16_t)RegisterAddress::ADDRESS),

    // @Param: LIMT
    // @DisplayName: device address
    // @Description: device address
    AP_GROUPINFO("LIMT", 2, AP_M1DCB2450SC, _limit, 60),


    AP_GROUPEND
};


// singleton instance
AP_M1DCB2450SC *AP_M1DCB2450SC::_singleton;

AP_M1DCB2450SC::AP_M1DCB2450SC()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_BCM must be singleton");
    }
#endif
    _singleton = this;
}


// initialise driver
void AP_M1DCB2450SC::init()
{
    // only init once
    // Note: a race condition exists here if init is called multiple times quickly before thread_main has a chance to set _initialise
    if (_initialised || !_enable) {
        return;
    }

    // create background thread to process serial input and output
    char thread_name[15];
    hal.util->snprintf(thread_name, sizeof(thread_name), "AP_M1DCB%u", (unsigned)0);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_M1DCB2450SC::thread_main, void), thread_name, 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        return;
    }
}

// returns true if communicating with the motor
bool AP_M1DCB2450SC::healthy()
{
    if (!_initialised || !_enable) {
        return false;
    }
    {
        // healthy if both receive and send have occurred in the last 3 seconds
        WITH_SEMAPHORE(_last_healthy_sem);
        const uint32_t now_ms = AP_HAL::millis();
        return ((now_ms - _last_received_ms < 3000) && (now_ms - _last_send_motor_ms < 3000));
    }
}


// initialise serial port and gpio pins (run from background thread)
bool AP_M1DCB2450SC::init_internals()
{
    // find serial driver and initialise
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_M1DCB2450SC, 0);
    if (_uart == nullptr) {
        return false;
    }
    _uart->begin(M1DCB2450SC_SERIAL_BAUD);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);
    
    return true;
}

// send target motor speed
void AP_M1DCB2450SC::send_motor_speed_cmd(const int16_t speed)
{
    _motor_speed_cmd = constrain_int16(speed, -_limit.get(), _limit.get());
    if (_left_limited || _right_limited) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Over Limited, Stopping!");
    }
}


// check for timeout waiting for reply message
void AP_M1DCB2450SC::check_for_reply_timeout()
{
    // return immediately if not waiting for reply
    if (_reply_wait_start_ms == 0) {
        return;
    }
    if (AP_HAL::millis() - _reply_wait_start_ms > 100) {
        _reply_wait_start_ms = 0;
    }
}

// consume incoming messages from motor, reply with latest motor speed
// runs in background thread
void AP_M1DCB2450SC::thread_main()
{
    // initialisation
    if (!init_internals()) {
        return;
    }
    _initialised = true;

    // configure address 00 06 00 01 00 02 58 1A
    set_device_address();
    hal.scheduler->delay(1000);
    uint32_t read_bytes = MIN(_uart->available(), 1024U);
    while (read_bytes-- > 0) {
        _uart->read();
    }
    

    // set control mode
    set_work_mode();
    hal.scheduler->delay(1000);
    read_bytes = MIN(_uart->available(), 1024U);
    while (read_bytes-- > 0) {
        _uart->read();
    }

    // motor speed control
    while (true) {
        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // check if transmit pin should be unset
        check_for_send_end();

        // check for timeout waiting for reply
        check_for_reply_timeout();

        // parse incoming characters
        uint32_t nbytes = MIN(_uart->available(), 1024U);
        while (nbytes-- > 0) {
            int16_t b = _uart->read();
            if (b >= 0 ) {
                if (parse_byte((uint8_t)b)) {
                    // complete message received, parse it!
                    parse_message();
                    // clear wait-for-reply because if we are waiting for a reply, this message must be it
                    set_reply_received();
                }
            }
        }

        // send motor speed
        if (safe_to_send()) {
            uint32_t now_ms = AP_HAL::millis();

            if (now_ms - _last_send_motor_ms >= M1DCB2450SC_SEND_MOTOR_SPEED_INTERVAL_MS) {
                // send motor speed every 0.1sec
                _send_motor_speed = true;
            }  else if (now_ms - _last_send_motor_status_request_ms >= M1DCB2450SC_SEND_MOTOR_STATUS_REQUEST_INTERVAL_MS) {
                // send request for motor status
                read_status();
                _last_send_motor_status_request_ms = now_ms;
            }

            // send motor speed
            if (_send_motor_speed) {
                set_motor_speed();
                _send_motor_speed = false;
            }
        }

    }

}


// process one byte received on serial port
// returns true if successfully parsed a message
// if distances are valid, valid_readings is set to true and distance is stored in reading_cm
bool AP_M1DCB2450SC::parse_byte(uint8_t b)
{
    uint8_t write_command_response_buf[8];

    // process byte depending upon current state
    switch (parsed_msg.state) {

    case ParseState::WAITING_FOR_ADDRESS: {
        if (b == _address.get()) {
            parsed_msg.address = b;
            parsed_msg.state = ParseState::WAITING_FOR_FUNCTION_CODE;
            write_command_response_buf[0] = b;
        }
        break;
    }

    case ParseState::WAITING_FOR_FUNCTION_CODE:
        if (b == (uint8_t)FunctionCode::READ_HOLDING_REGISTER ||
            b == (uint8_t)FunctionCode::READ_INPUT_REGISTER) {
            parsed_msg.function_code = b;
            parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD_LEN;
        } else {
            parsed_msg.state = ParseState::WAITING_FOR_ADDRESS;
        }

        if (b == (uint8_t)FunctionCode::WRITE_HOLDING_REGISTER) {
            write_command_response_buf[1] = b;
            parsed_msg.function_code = b;
            parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD_LEN;
        }
        break;

    case ParseState::WAITING_FOR_PAYLOAD_LEN:
        // only parse messages of the expected length
        parsed_msg.payload_len =  parsed_msg.function_code == (uint8_t)FunctionCode::WRITE_HOLDING_REGISTER ? 4 : b;
        parsed_msg.payload_recv = 0;
        parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD;
        break;

    case ParseState::WAITING_FOR_PAYLOAD:
        if (parsed_msg.payload_recv < parsed_msg.payload_len) {
            if (parsed_msg.payload_recv < ARRAY_SIZE(parsed_msg.payload)) {
                parsed_msg.payload[parsed_msg.payload_recv] = b;
                write_command_response_buf[2 + parsed_msg.payload_recv] = b;
            }
            parsed_msg.payload_recv++;
        }
        if (parsed_msg.payload_recv == parsed_msg.payload_len) {
            parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
        }
        break;

    case ParseState::WAITING_FOR_CRC_LOW:
        parsed_msg.crc = b;
        parsed_msg.state = ParseState::WAITING_FOR_CRC_HIGH;
        break;

    case ParseState::WAITING_FOR_CRC_HIGH: {
            parsed_msg.crc |= ((uint16_t)b << 8);
            parsed_msg.state = ParseState::WAITING_FOR_ADDRESS;

            // check crc
            uint16_t expected_crc = calc_crc_modbus(&parsed_msg.address, 3+parsed_msg.payload_recv);
            uint16_t write_command_response_crc = calc_crc_modbus(write_command_response_buf, 8);
            
            if (expected_crc == parsed_msg.crc || write_command_response_crc == parsed_msg.crc) {
                {
                    // record time of successful receive for health reporting
                    WITH_SEMAPHORE(_last_healthy_sem);
                    _last_received_ms = AP_HAL::millis();
                }
                return true;
            }
            break;
        }
    }

    return false;
}

// process message held in _received_buff
void AP_M1DCB2450SC::parse_message()
{
    // replies strangely do not return the msgid so we must have stored it
    MotorMsgId msg_id = (MotorMsgId)_reply_msgid;

    if  (parsed_msg.function_code == (uint8_t)FunctionCode::WRITE_HOLDING_REGISTER&& 
        ((parsed_msg.payload[0] << 8 | parsed_msg.payload[1]) == (uint16_t)RegisterAddress::SET_MOTOR_SPEED) &&
        msg_id == MotorMsgId::DRIVE) {
        
        // request received to send updated motor speed
        _send_motor_speed = true;
        return;
    }

    if (msg_id == MotorMsgId::STATUS &&
        parsed_msg.function_code == (uint8_t)FunctionCode::READ_HOLDING_REGISTER) {
        _left_limited = UINT16_VALUE(parsed_msg.payload[0], parsed_msg.payload[1]);
        _right_limited = UINT16_VALUE(parsed_msg.payload[2], parsed_msg.payload[3]);
    }

}

// mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
void AP_M1DCB2450SC::set_reply_received()
{
    _reply_wait_start_ms = 0;
}


// check for timeout after sending and unset pin if required
void AP_M1DCB2450SC::check_for_send_end()
{
    if (_send_delay_us == 0) {
        // not sending
        return;
    }

    if (AP_HAL::micros() - _send_start_us < _send_delay_us) {
        // return if delay has not yet elapsed
        return;
    }
    _send_delay_us = 0;

}

// calculate delay require to allow bytes to be sent
uint32_t AP_M1DCB2450SC::calc_send_delay_us(uint8_t num_bytes)
{
    // baud rate of 19200 bits/sec
    // total number of bits = 10 x num_bytes
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    const uint32_t delay_us = 1e6 * num_bytes * 10 / M1DCB2450SC_SERIAL_BAUD + 300;
    return delay_us;
}

// record msgid of message to wait for and set timer for timeout handling
void AP_M1DCB2450SC::set_expected_reply_msgid(uint8_t msg_id)
{
    _reply_msgid = msg_id;
    _reply_wait_start_ms = AP_HAL::millis();
}


// send motor speed
void AP_M1DCB2450SC::set_motor_speed()
{
    // set expected reply message id
    set_expected_reply_msgid((uint8_t)MotorMsgId::DRIVE);

    uint8_t req_buf[] = {
            (uint8_t)_address.get(),                       // address
            (uint8_t)FunctionCode::WRITE_HOLDING_REGISTER, // function code low
            (uint8_t)HIGHBYTE(RegisterAddress::SET_MOTOR_SPEED),                                          // function code high
            (uint8_t)LOWBYTE(RegisterAddress::SET_MOTOR_SPEED), 
            (uint8_t)HIGHBYTE(_motor_speed_cmd),                                          // function code high
            (uint8_t)LOWBYTE(_motor_speed_cmd), 
            0,                                          // crc low
            0                                           // crc high
    };
    const uint8_t req_buf_len = sizeof(req_buf);

    // fill in crc bytes
    uint16_t crc = calc_crc_modbus(req_buf, req_buf_len - 2);
    req_buf[req_buf_len - 2] = LOWBYTE(crc);
    req_buf[req_buf_len - 1] = HIGHBYTE(crc);

    // send request to device
    _uart->write(req_buf, req_buf_len);

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(req_buf));

    // record time of send for health reporting
    WITH_SEMAPHORE(_last_healthy_sem);
    _last_send_motor_ms = AP_HAL::millis();

}

// configure address 01 06 00 01 00 02 CRCH CRCL
void AP_M1DCB2450SC::set_device_address()
{
    uint8_t req_buf[] = {
            00,                       // address
            (uint8_t)FunctionCode::WRITE_HOLDING_REGISTER, // function code low
            (uint8_t)HIGHBYTE(RegisterAddress::ADDRESS),                                          // function code high
            (uint8_t)LOWBYTE(RegisterAddress::ADDRESS), 
            (uint8_t)HIGHBYTE(_address.get()),                                          // function code high
            (uint8_t)LOWBYTE(_address.get()), 
            0,                                          // crc low
            0                                           // crc high
    };
    const uint8_t req_buf_len = sizeof(req_buf);

    // fill in crc bytes
    uint16_t crc = calc_crc_modbus(req_buf, req_buf_len - 2);
    req_buf[req_buf_len - 2] = LOWBYTE(crc);
    req_buf[req_buf_len - 1] = HIGHBYTE(crc);

    // send request to device
    _uart->write(req_buf, req_buf_len);

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(req_buf));
}

// set control mode 01 06 00 00 00 00 CRCH CRCL 
void AP_M1DCB2450SC::set_work_mode()
{
    uint8_t req_buf[] = {
            (uint8_t)_address.get(),                       // address
            (uint8_t)FunctionCode::WRITE_HOLDING_REGISTER, // function code low
            (uint8_t)HIGHBYTE(RegisterAddress::WORK_MODE),                                          // function code high
            (uint8_t)LOWBYTE(RegisterAddress::WORK_MODE), 
            (uint8_t)HIGHBYTE(WorkMode::CONTROL),                                          // function code high
            (uint8_t)LOWBYTE(WorkMode::CONTROL), 
            0,                                          // crc low
            0                                           // crc high
    };
    const uint8_t req_buf_len = sizeof(req_buf);

    // fill in crc bytes
    uint16_t crc = calc_crc_modbus(req_buf, req_buf_len - 2);
    req_buf[req_buf_len - 2] = LOWBYTE(crc);
    req_buf[req_buf_len - 1] = HIGHBYTE(crc);

    // send request to device
    _uart->write(req_buf, req_buf_len);

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(req_buf));
}


// set control mode 01 06 00 00 00 00 CRCH CRCL 
void AP_M1DCB2450SC::read_status()
{
    // set expected reply message id
    set_expected_reply_msgid((uint8_t)MotorMsgId::STATUS);

    uint8_t req_buf[] = {
            (uint8_t)_address.get(),                       // address
            (uint8_t)FunctionCode::READ_HOLDING_REGISTER, // function code low
            (uint8_t)HIGHBYTE(RegisterAddress::READ_LIMITED_CODE),                                          // function code high
            (uint8_t)LOWBYTE(RegisterAddress::READ_LIMITED_CODE), 
            (uint8_t)HIGHBYTE(0x0002),                                          // function code high
            (uint8_t)LOWBYTE(0x0002), 
            0,                                          // crc low
            0                                           // crc high
    };
    const uint8_t req_buf_len = sizeof(req_buf);

    // fill in crc bytes
    uint16_t crc = calc_crc_modbus(req_buf, req_buf_len - 2);
    req_buf[req_buf_len - 2] = LOWBYTE(crc);
    req_buf[req_buf_len - 1] = HIGHBYTE(crc);

    // send request to device
    _uart->write(req_buf, req_buf_len);

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(req_buf));
}

namespace AP {
    AP_M1DCB2450SC *m1dcb2450sc()
    {
        return AP_M1DCB2450SC::get_singleton();
    }
};

#endif
