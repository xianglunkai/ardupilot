#pragma once
#include "AP_BattMonitor_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#ifndef HAL_KXD_CAN_ENABLE
#define HAL_KXD_CAN_ENABLE (HAL_NUM_CAN_IFACES && !HAL_MINIMIZE_FEATURES && HAL_MAX_CAN_PROTOCOL_DRIVERS)
#endif

#if HAL_KXD_CAN_ENABLE

#ifndef KXD_CAN_DEVICE_COUNT_MAX
    #define KXD_CAN_DEVICE_COUNT_MAX 2
#endif

class AP_BattMonitor_KXD_CAN : public CANSensor, public AP_BattMonitor_Backend {
public:
     // construct the CAN Sensor
    AP_BattMonitor_KXD_CAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params):
        AP_BattMonitor_Backend(mon, mon_state, params),
        CANSensor("BMS")
    {
        register_driver(AP_CANManager::Driver_Type_KXD_CAN);
    }

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

protected:
    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    enum class PacketType: uint32_t{
        PORT_BATT    = 0x8a6d0d09, // Messages *from* PORT battery
        STBD_BATT    = 0x8ad10d09, // Messages *from* STBD battery
    };

    bool get_voltage_and_current_and_temp(const int32_t serialnumber, float &voltage, float &current, float &consumed_mah) const;

    struct BMS_device{
        float voltage;
        float current;
        float remain;
        float capacity;

        uint32_t timestamp_ms;
        bool is_healthy() const {
            return ((timestamp_ms > 0) && ((AP_HAL::millis() - timestamp_ms) < 10000));
        }

    }BMS_devices[KXD_CAN_DEVICE_COUNT_MAX];

    HAL_Semaphore _sem_static;

};


#endif
