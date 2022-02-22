#include "AP_BattMonitor_KXD_RS485.h"

#if HAL_KXD_RS485_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

#define BATT_KXD_TIMEOUT_MS                500         // timeout in milliseconds if no distance messages received

AP_BattMonitor_KXD_RS485::AP_BattMonitor_KXD_RS485(AP_BattMonitor &mon,
            AP_BattMonitor::BattMonitor_State &mon_state,
            AP_BattMonitor_Params &params):
            AP_BattMonitor_Backend(mon, mon_state, params)
{
    
}


void AP_BattMonitor_KXD_RS485::init(void)
{
    _uart  = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Batt_KXD_RS485, 0);
    if (_uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"KXD uart error");
        return;
    }
    _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Batt_KXD_RS485, 0),rx_bufsize(), tx_bufsize());
}


void AP_BattMonitor_KXD_RS485::read()
{
    if(_uart == nullptr){
        return;
    }

    // check for timeout receiving messages
    uint32_t now_ms = AP_HAL::millis();
    if(((now_ms - last_read_ms) > BATT_KXD_TIMEOUT_MS) && ((now_ms - last_request_ms) > BATT_KXD_TIMEOUT_MS)){
         _uart->write(send_request_buffer, sizeof(send_request_buffer));
         // record time of request
        last_request_ms = AP_HAL::millis();
    }

    bool latest_read_valid = false;

    // read any available characters
    int16_t nbytes = _uart->available();
    while(nbytes-- > 0){
        int16_t r = _uart->read();
        if(r  < 0){
            continue;
        }
        if(parse_byte((uint8_t)r,latest_read_valid)){

            // set parameters
            if(!_state.healthy){
                uint16_t _full_charge_capacity = battery_message.total_pct * 10;
                if (_full_charge_capacity != _params._pack_capacity) {
                    _params._pack_capacity.set_and_notify(_full_charge_capacity);
                }
            }

            // update battery status
            _state.voltage = 0.01f * battery_message.voltage;
            _state.current_amps = 0.01f * battery_message.current;
            _state.consumed_mah = 10.0f * (battery_message.total_pct - battery_message.remain_pct);

            // printf("voltage = %f,current = %f\n", _state.voltage,_state.current_amps);

            last_read_ms = AP_HAL::millis();
            _state.last_time_micros = AP_HAL::micros();
            _state.healthy = true;
            return;
        }
    }

    // check unhealthy
    if((now_ms - last_read_ms) > 2* BATT_KXD_TIMEOUT_MS && last_read_ms > 0 ){
        memset(&battery_message,0,sizeof(battery_message));
        memset(&parsed_msg,0,sizeof(parsed_msg));
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"Battery unhealthy");
        _state.voltage = 0;
        _state.current_amps = 0;
        _state.healthy = false;
    }

}

// process one byte received on serial port
// returns true if successfully parsed a message
bool AP_BattMonitor_KXD_RS485::parse_byte(uint8_t b, bool &valid_reading)
{
    // process byte depending upon current state
    switch (parsed_msg.state) {

    case ParseState::WAITING_FOR_START_BYTE: {
        if (b == 0xDD) {
            parsed_msg.start_byte = b;
            parsed_msg.state = ParseState::WAITING_FOR_FUNCTION_CODE;
        }
        break;
    }

    case ParseState::WAITING_FOR_FUNCTION_CODE:
        if (b == 0x03) {
            parsed_msg.function_code = b;
            parsed_msg.state = ParseState::WAITING_FOR_STATE_BYTE;
        } else {
            parsed_msg.state = ParseState::WAITING_FOR_START_BYTE;
        }
        break;
    case ParseState::WAITING_FOR_STATE_BYTE:
        parsed_msg.state_byte = b;
        if(b == 0x00){
            parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD_LEN;
        }else{
            parsed_msg.state = ParseState::WAITING_FOR_START_BYTE;
        }

        break;

    case ParseState::WAITING_FOR_PAYLOAD_LEN:
        // only parse messages of the expected length
        if (b > 0) {
            parsed_msg.payload_len = b;
            parsed_msg.payload_recv = 0;
            parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD;
        } else {
            parsed_msg.state = ParseState::WAITING_FOR_START_BYTE;
        }
        break;

    case ParseState::WAITING_FOR_PAYLOAD:
        if (parsed_msg.payload_recv < parsed_msg.payload_len) {
            if (parsed_msg.payload_recv < ARRAY_SIZE(parsed_msg.payload)) {
                parsed_msg.payload[parsed_msg.payload_recv] = b;
            }
            parsed_msg.payload_recv++;
        }
        if (parsed_msg.payload_recv == parsed_msg.payload_len) {
            parsed_msg.state = ParseState::WAITING_FOR_CRC_HIGH;
        }
        break;

    case ParseState::WAITING_FOR_CRC_HIGH:
        parsed_msg.crc = (uint16_t)b << 8;
        parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
        break;

    case ParseState::WAITING_FOR_CRC_LOW: 
        parsed_msg.crc |= ((uint16_t)b);
        parsed_msg.state = ParseState::WAITING_FOR_END_BYTE;
        break;
     case ParseState::WAITING_FOR_END_BYTE:
        if(b == 0x77){
            uint16_t expected_crc=calc_crc();
            if(expected_crc==parsed_msg.crc){
                battery_message.voltage    = UINT16_VALUE(parsed_msg.payload[0],parsed_msg.payload[1]);
                battery_message.current    = UINT16_VALUE(parsed_msg.payload[2],parsed_msg.payload[3]);;
                battery_message.remain_pct = UINT16_VALUE(parsed_msg.payload[4],parsed_msg.payload[5]);;
                battery_message.total_pct  = UINT16_VALUE(parsed_msg.payload[6],parsed_msg.payload[7]);;
                valid_reading = true;
                return true;
            }
        }
        parsed_msg.state = ParseState::WAITING_FOR_START_BYTE;
        break;
    }
    

    valid_reading = false;
    return false;
}

uint16_t AP_BattMonitor_KXD_RS485::calc_crc()
{
    uint16_t crc=0;
    crc += parsed_msg.state_byte;
    crc += parsed_msg.payload_len;
    for(uint16_t index=0;index<parsed_msg.payload_len;index++){
        crc += parsed_msg.payload[index];
    }
    crc = ~crc+1;
    return crc;
}





#endif