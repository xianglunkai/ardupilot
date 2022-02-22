#include "AP_BattMonitor_KXD_CAN.h"

#if HAL_KXD_CAN_ENABLE
#include <AP_CANManager/AP_CANManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_KXD_CAN::read()
{
    WITH_SEMAPHORE(_sem_static);

    // get output voltage and current but allow for getting input if serial number is negative
    // Using _params._serial_number == 0 will give you the average output of all devices
    if (get_voltage_and_current_and_temp(_params._serial_number, _state.voltage, _state.current_amps, _state.consumed_mah)) {

        // set parameters
        if(!_state.healthy){
            uint16_t _full_charge_capacity = BMS_devices[_params._serial_number].capacity;
            if (_full_charge_capacity != _params._pack_capacity) {
                _params._pack_capacity.set_and_notify(_full_charge_capacity);
            }
        }

        _state.last_time_micros = AP_HAL::micros();
        _state.healthy = true;
        return;
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"Battery unhealthy");
    _state.voltage = 0;
    _state.current_amps = 0;
    _state.healthy = false;
}

// parse inbound frames
void AP_BattMonitor_KXD_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem_static);
    if ((frame.id & AP_HAL::CANFrame::FlagEFF) == 0) {
        return;
    }

    switch ((PacketType)frame.id)
    {
    case PacketType::PORT_BATT:
         BMS_devices[0].capacity = 100.0f* (((uint16_t)frame.data[6] << 8) | ((uint16_t)frame.data[7]));
         BMS_devices[0].remain   = 100.0* (((uint16_t)frame.data[4] << 8) | ((uint16_t)frame.data[5]));
         BMS_devices[0].voltage  = 0.1f * (((uint16_t)frame.data[0])<<8 | ((uint16_t)frame.data[1]));
         BMS_devices[0].current  = 0.1f * static_cast<int16_t>(((uint16_t)frame.data[2])<<8 | ((uint16_t)frame.data[3]));
         BMS_devices[0].timestamp_ms = AP_HAL::millis();
   
        break;
    case PacketType::STBD_BATT:
         BMS_devices[1].capacity = 100.0f * (((uint16_t)frame.data[6] << 8) | ((uint16_t)frame.data[7]));
         BMS_devices[1].remain   = 100.0f * (((uint16_t)frame.data[4] << 8) | ((uint16_t)frame.data[5]));
         BMS_devices[1].voltage  = 0.1f * (((uint16_t)frame.data[0])<<8 | ((uint16_t)frame.data[1]));
         BMS_devices[1].current  = 0.1f * static_cast<int16_t>(((uint16_t)frame.data[2])<<8 | ((uint16_t)frame.data[3]));
         BMS_devices[1].timestamp_ms = AP_HAL::millis();

        break;
    default:
        break;
    }

}

// get the voltage and current and temp of the input or the output MPPT device when returning true
// when returning false, no values were changed.
bool AP_BattMonitor_KXD_CAN::get_voltage_and_current_and_temp(const int32_t serialnumber, float &voltage, float &current, float &consumed_mah) const
{
    if (static_cast<uint16_t>(serialnumber) >= ARRAY_SIZE(BMS_devices) || 
       !BMS_devices[serialnumber].is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"Battery unhealthy");
        return false;
    }
    
    // we only report output energy
    voltage = BMS_devices[serialnumber].voltage;
    current = std::fabs(BMS_devices[serialnumber].current);
    consumed_mah = BMS_devices[serialnumber].capacity - BMS_devices[serialnumber].remain;
    return true;
}

#endif