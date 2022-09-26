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
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_TEMPERATURE_SENSOR_ENABLED
#define AP_TEMPERATURE_SENSOR_ENABLED (!HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024)
#endif

#if AP_TEMPERATURE_SENSOR_ENABLED
#include "AP_TemperatureSensor_Params.h"

// maximum number of Temperature Sensors
#ifndef AP_TEMPERATURE_SENSOR_MAX_INSTANCES
#define AP_TEMPERATURE_SENSOR_MAX_INSTANCES             3
#endif

// first sensor is always the primary sensor
#ifndef AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE
#define AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE          0
#endif

// declare backend class
class AP_TemperatureSensor_Backend;
class AP_TemperatureSensor_TSYS01;

class AP_TemperatureSensor
{
    friend class AP_TemperatureSensor_Backend;
    friend class AP_TemperatureSensor_TSYS01;

public:

    // Constructor
    AP_TemperatureSensor();

    CLASS_NO_COPY(AP_TemperatureSensor);

    static AP_TemperatureSensor *get_singleton() { return _singleton; }

    // temperature sensor types
    enum class Type : uint8_t {
        NONE                        = 0,
        TSYS01                      = 1,
    };

    // option to map to another system component
    enum class Source : uint8_t {
        None                        = 0,
        ESC                         = 1,
        Motor                       = 2,
        Battery_Index               = 3,
        Battery_ID_SerialNumber     = 4,
    };

    // The TemperatureSensor_State structure is filled in by the backend driver
    struct TemperatureSensor_State {
        const struct AP_Param::GroupInfo *var_info;
        uint32_t    last_time_ms;              // time when the sensor was last read in milliseconds
        float       temperature;               // temperature (deg C)
        uint8_t     instance;                  // instance number
    };

    // Return the number of temperature sensors instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available temperature sensors
    void init();

    // Update the temperature for all temperature sensors
    void update();

    bool get_temperature(float &temp, const uint8_t instance = AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE) const;

    bool healthy(const uint8_t instance = AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE) const;

    // accessors to params
    Type get_type(const uint8_t instance = AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE) const;
    Source get_source(const uint8_t instance = AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE) const;
    int32_t get_source_id(const uint8_t instance = AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE) const;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // parameters
    AP_TemperatureSensor_Params _params[AP_TEMPERATURE_SENSOR_MAX_INSTANCES];

private:
    static AP_TemperatureSensor *_singleton;

    TemperatureSensor_State _state[AP_TEMPERATURE_SENSOR_MAX_INSTANCES];
    AP_TemperatureSensor_Backend *drivers[AP_TEMPERATURE_SENSOR_MAX_INSTANCES];

    uint8_t     _num_instances;         // number of temperature sensors

    // Parameters
    AP_Int8 _log_flag;                  // log_flag: true if we should log all sensors data
};

namespace AP {
    AP_TemperatureSensor &temperature_sensor();
};

#endif // AP_TEMPERATURE_SENSOR_ENABLED
