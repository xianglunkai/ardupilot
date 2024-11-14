#include "AP_CANIO.h"
#if AP_CANIO_ENABLE
#include <stdio.h>
#include <string.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>    
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL & hal;

const AP_Param::GroupInfo AP_CANIO_Params::var_info[] = {

    // @User: Standard
    AP_GROUPINFO_FLAGS("FUNCTION", 1, AP_CANIO_Params, function, (float)FUNCTION::NONE, AP_PARAM_FLAG_ENABLE),

    AP_GROUPINFO("PIN", 2, AP_CANIO_Params, pin, -1),


    // @Param: DEFAULT
    // @DisplayName: Relay default state
    // @Description: Should the relay default to on or off, this only applies to RELAYx_FUNC "Relay" (1). All other uses will pick the appropriate default output state from within the controlling function's parameters. Note that if INVERTED is set then the default is inverted.
    // @Values: 0: Off,1:On,2:NoChange
    // @User: Standard
    AP_GROUPINFO("DEFAULT", 3, AP_CANIO_Params, default_state, (float)DefaultState::OFF),

    // @Param: INVERTED
    // @DisplayName: Relay invert output signal
    // @Description: Should the relay output signal be inverted. If enabled, relay on would be pin low and relay off would be pin high. NOTE: this impact's DEFAULT.
    // @Values: 0:Normal,1:Inverted
    // @User: Standard
    AP_GROUPINFO("INVERTED", 4, AP_CANIO_Params, inverted, false),

};


AP_CANIO_Params::AP_CANIO_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

const AP_Param::GroupInfo AP_CANIO::var_info[] = {

    // @Param: STARTER_TIME
    // @DisplayName: Time to run starter
    // @Description: This is the number of seconds to run the starter when trying to start the engine
    // @User: Standard
    // @Units: s
    // @Range: 0.1 5
    AP_GROUPINFO("STARTER_TIME", 0, AP_CANIO, debug, 0),

    // @Group: 1_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 7, AP_CANIO, AP_CANIO_Params),

#if AP_CANIO_NUM_RELAYS > 1
    // @Group: 2_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 8, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 2
    // @Group: 3_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 9, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 3
    // @Group: 4_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 10, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 4
    // @Group: 5_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[4], "5_", 11, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 5
    // @Group: 6_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[5], "6_", 12, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 6
    // @Group: 7_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[6], "7_", 13, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 7
    // @Group: 8_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[7], "8_", 14, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 8
    // @Group: 9_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[8], "9_", 15, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 9
    // @Group: 10_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[9], "10_", 16, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 10
    // @Group: 11_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[10], "11_", 17, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 11
    // @Group: 12_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[11], "12_", 18, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 12
    // @Group: 13_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[12], "13_", 19, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 13
    // @Group: 14_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[13], "14_", 20, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 14
    // @Group: 15_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[14], "15_", 21, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_RELAYS > 15
    // @Group: 16_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_params[15], "16_", 22, AP_CANIO, AP_CANIO_Params),
#endif

};


// singleton instance
AP_CANIO *AP_CANIO::_singleton;

AP_CANIO::AP_CANIO() : CANSensor("CANIO")
{
    AP_Param::setup_object_defaults(this, var_info);

    register_driver(AP_CAN::Protocol::CANIO);

    _singleton = this;

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CANIO::loop, void), "CANIO", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}


void AP_CANIO::init()
{
    set_defaults();

    // setup the actual default values of all the pins
    for (uint8_t instance = 0; instance < ARRAY_SIZE(_params); instance++) {
        const int16_t pin = _params[instance].pin;
        if (pin == -1) {
            // no valid pin to set it on, skip it
            continue;
        }

        const AP_CANIO_Params::FUNCTION function = _params[instance].function;
        if (!function_valid(function)) {
            // invalid function, skip it
            continue;
        }

        if (function == AP_CANIO_Params::FUNCTION::RELAY) {
            // relay by instance number, set the state to match our output
            const AP_CANIO_Params::DefaultState default_state = _params[instance].default_state;
            if ((default_state == AP_CANIO_Params::DefaultState::OFF) ||
                (default_state == AP_CANIO_Params::DefaultState::ON)) {

                set_pin_by_instance(instance, (bool)default_state);
            }
        } else {
            // all functions are supposed to be off by default
            // this will need revisiting when we support inversion
            set_pin_by_instance(instance, false);
        }

    }
}


void AP_CANIO::set(const AP_CANIO_Params::FUNCTION function, const bool value) {
    if (!function_valid(function)) {
        // invalid function
        return;
    }

    for (uint8_t instance = 0; instance < ARRAY_SIZE(_params); instance++) {
        if (function != _params[instance].function) {
            continue;
        }

        set_pin_by_instance(instance, value);
    }
}

// set a pins output state by instance and log if required
// this is an internal helper, instance must have already been validated to be in range
void AP_CANIO::set_pin_by_instance(uint8_t instance, bool value)
{
    const int16_t pin = _params[instance].pin;
    if (pin == -1) {
        // no valid pin to set it on, skip it
        return;
    }

    if (_params[instance].inverted > 0) {
        value = !value;
    }

    set_pin(pin, value);
    
}

void AP_CANIO::set(const uint8_t instance, const bool value)
{
    if (instance >= ARRAY_SIZE(_params)) {
        return;
    }

    if (_params[instance].function != AP_CANIO_Params::FUNCTION::RELAY) {
        return;
    }

    set_pin_by_instance(instance, value);
}

void AP_CANIO::toggle(uint8_t instance)
{
    if (instance < ARRAY_SIZE(_params)) {
        set(instance, !get(instance));
    }
}

bool AP_CANIO::get(uint8_t instance) const
{
    if (instance >= ARRAY_SIZE(_params)) {
        // invalid instance
        return false;
    }

    if (_params[instance].inverted > 0) {
        return !get_pin(_params[instance].pin.get());
    }

    return get_pin(_params[instance].pin.get());
}

// Get relay state from pin number
bool AP_CANIO::get_pin(const int16_t pin) const
{
    if (pin < 0) {
        // invalid pin
        return false;
    }

    // Read pins
    const uint64_t data = _read_pins.all & ((uint64_t)(1) << (pin-1));
    const uint16_t pins = (data >> (pin - 1)) & 0x01;
    return (bool)pins;
}


// Set relay state from pin number
void AP_CANIO::set_pin(const int16_t pin, const bool value)
{
    if (pin < 0) {
        // invalid pin
        return;
    }

    // Real GPIO pin
    {
        // WITH_SEMAPHORE(sem);
        if (pin == 1) {
            _write_pins.all |= ((uint64_t)(1)<<(pin-1));
        } else {
            _write_pins.all &= (0xffff - ((uint64_t)(1)<<(pin-1)));      
        }

        _pins_is_new = true;
    }

}

// see if the relay is enabled
bool AP_CANIO::enabled(AP_CANIO_Params::FUNCTION function) const
{
    for (uint8_t instance = 0; instance < ARRAY_SIZE(_params); instance++) {
        if ((_params[instance].function == function) && (_params[instance].pin != -1)) {
            return true;
        }
    }
    return false;
}


bool AP_CANIO::read(AP_CANIO_Params::FUNCTION function, float &value)
{
    if (function == AP_CANIO_Params::FUNCTION::THR_ANG && _ai_is_new) {
        value = _throttle_pos;
        return true;
    }

    if (function == AP_CANIO_Params::FUNCTION::THR_ANG && _ai_is_new) {
        value = _steering_pos;
        return true;
    }

    return false;
}


void AP_CANIO::loop()
{
    uint16_t read_io_counter = 0;
    uint16_t write_io_counter = 0;
    uint16_t read_ai_counter = 0;
    uint8_t temp_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    while (true) { 
      
        // 10ms loop delay
        hal.scheduler->delay_microseconds(10000); 
        {

            // WITH_SEMAPHORE(sem);
            if (_pins_is_new ) {
                _pins_is_new = false;

                // write IO channel 50ms
                if (++write_io_counter >= 5) {
                    write_io_counter = 0;
                    send_messages((uint32_t)(CAN_ID::WRITE_IO), (uint8_t*)_write_pins.raw_data, 8);
                }
            }
        }

        // read AI channel 20ms
        if (++read_ai_counter >= 2) {
            read_ai_counter = 0;
            send_messages((uint32_t)(CAN_ID::READ_AI), (uint8_t*)temp_data, 8);
        }

        // read IO channle 1000ms
        if (++read_io_counter >= 100) {
            read_io_counter = 0;
            send_messages((uint32_t)(CAN_ID::READ_IO), (uint8_t*)temp_data, 8);
        }
        

    }
}

// send control mode over CAN
void AP_CANIO::send_messages(const uint32_t cand_id, const uint8_t *data, const uint8_t data_len)
{
    AP_HAL::CANFrame txFrame;
    uint32_t id = cand_id;
    txFrame = {(id |= AP_HAL::CANFrame::FlagEFF), data, data_len};
    write_frame(txFrame, AP_HAL::micros64() + 1000ULL);
}


// parse inbound frames
void AP_CANIO::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    switch (CAN_ID(id)) {

        case CAN_ID::READ_IO:
        {
            memcpy(&_read_pins.all, frame.data, frame.dlc);
            break;
        }

        case CAN_ID::READ_AI:
        {
            AIChannel ai_data;
            memcpy(&ai_data.raw_data, frame.data, frame.dlc);

            // translate into real unit
            _throttle_pos = ai_data.throttle_pos;
            _steering_pos = ai_data.steering_pos;
            _ai_is_new = true;

            if (debug) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "steering: %0.1f, throttle: %0.1f", (float)_steering_pos, (float)_throttle_pos);
            }
            break;
        }

        case CAN_ID::WRITE_IO:
        {
            break;
        }
    }

}

void AP_CANIO::set_defaults() {

    memset(&_write_pins.data, 0, sizeof(_write_pins.data));
    memset(&_read_pins.data, 0, sizeof(_read_pins.data));

    for (uint8_t i = 0; i < ARRAY_SIZE(_params); i++) {
        // set the default
        _params[i].pin.set_default(-1);
    }
}


// Return true is function is valid
bool AP_CANIO::function_valid(AP_CANIO_Params::FUNCTION function) const
{
    return (function > AP_CANIO_Params::FUNCTION::NONE) && (function < AP_CANIO_Params::FUNCTION::NUM_FUNCTIONS);
}

namespace AP {
    AP_CANIO *can_io()
    {
        return AP_CANIO::get_singleton();
    }
};

#endif