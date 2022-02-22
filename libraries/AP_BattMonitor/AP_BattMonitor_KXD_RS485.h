#pragma once
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_Backend.h"

#ifndef HAL_KXD_RS485_ENABLED
#define HAL_KXD_RS485_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_KXD_RS485_ENABLED

#define BATT_KXD_PAYLOAD_LENGTH 255

class AP_BattMonitor_KXD_RS485 : public AP_BattMonitor_Backend{
    public:
        AP_BattMonitor_KXD_RS485(AP_BattMonitor &mon,
            AP_BattMonitor::BattMonitor_State &mon_state,
            AP_BattMonitor_Params &params);
        
    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    virtual void init(void) override;

    virtual void read() override;

    private:

       // the value 0 is special to the UARTDriver - it's "use default"
        virtual uint16_t rx_bufsize() const { return 0; }
        virtual uint16_t tx_bufsize() const { return 0; }

        AP_HAL::UARTDriver *_uart = nullptr;

         // parsing state
        enum class ParseState : uint8_t {
            WAITING_FOR_START_BYTE,
            WAITING_FOR_FUNCTION_CODE,
            WAITING_FOR_STATE_BYTE,
            WAITING_FOR_PAYLOAD_LEN,
            WAITING_FOR_PAYLOAD,
            WAITING_FOR_CRC_HIGH,
            WAITING_FOR_CRC_LOW,
            WAITING_FOR_END_BYTE,
        };

        // process one byte received on serial port
        // returns true if successfully parsed a message
        // if distances are valid, valid_readings is set to true and distance is stored in reading_cm
        bool parse_byte(uint8_t b, bool &valid_reading);


        //calc crc of parsed_msg
        uint16_t calc_crc();
        // structure holding latest message contents
        // the order of fields matches the incoming message so it can be used to calculate the crc
        struct PACKED {
            uint8_t start_byte;
            uint8_t function_code;
            uint8_t state_byte;
            uint8_t payload_len;
            uint8_t payload[BATT_KXD_PAYLOAD_LENGTH];
            uint16_t crc;
            uint8_t  end_byte;
            uint16_t payload_recv;
            ParseState state;
        } parsed_msg;

        // Send request buffer
        // read state feedback
        const uint8_t send_request_buffer[7] = {
            0xDD,  // Start byte
            0xA5,  // Read
            0x03,  // Register address
            0x00,
            0xFF, // Checksum Lo
            0xFD, // Checksum Hi
            0x77  // End byte
        };

        uint32_t last_request_ms; // system time of last request to sensor to send 
        uint32_t last_read_ms;    // system time of last successful read

        struct Battery_message{
            uint16_t voltage;  // 10mV
            int16_t  current;  // 10mA,+: charge -:discharge
            uint16_t remain_pct; // 10mAh
            uint16_t total_pct;  // 10mAh
        }battery_message;
};

#endif