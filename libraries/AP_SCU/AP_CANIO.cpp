#include "AP_CANIO.h"
#if AP_CANIO_ENABLE
#include <stdio.h>
#include <string.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>    
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>


#define DEBUG 1

extern const AP_HAL::HAL & hal;

const AP_Param::GroupInfo AP_CANIO_Params::var_info[] = {

    // @User: Standard
    AP_GROUPINFO_FLAGS("FUNC", 1, AP_CANIO_Params, function, (float)FUNCTION::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: DEFAULT
    // @DisplayName: Pins
    // @Description: Should the relay default to on or off, this only applies to RELAYx_FUNC "Relay" (1). All other uses will pick the appropriate default output state from within the controlling function's parameters. Note that if INVERTED is set then the default is inverted.
    // @Values: 0~63 / 0~18
    // @User: Standard
    AP_GROUPINFO("PIN", 2, AP_CANIO_Params, pin, -1),


    // @Param: DEFAULT
    // @DisplayName: Relay default state
    // @Description: Should the relay default to on or off, this only applies to RELAYx_FUNC "Relay" (1). All other uses will pick the appropriate default output state from within the controlling function's parameters. Note that if INVERTED is set then the default is inverted.
    // @Values: 0: Off,1:On,2:NoChange
    // @User: Standard
    AP_GROUPINFO("DEFT", 3, AP_CANIO_Params, default_state, (float)DefaultState::OFF),

    // @Param: INVERTED
    // @DisplayName: Relay invert output signal
    // @Description: Should the relay output signal be inverted. If enabled, relay on would be pin low and relay off would be pin high. NOTE: this impact's DEFAULT.
    // @Values: 0:Normal,1:Inverted
    // @User: Standard
    AP_GROUPINFO("INVT", 4, AP_CANIO_Params, inverted, false),

    AP_GROUPEND

};

AP_CANIO_Params::AP_CANIO_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

const AP_Param::GroupInfo AP_CANIO::var_info[] = {

    // @Group: 1_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[0], "1_", 1, AP_CANIO, AP_CANIO_Params),

#if AP_CANIO_NUM_IO > 1
    // @Group: 2_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[1], "2_", 2, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 2
    // @Group: 3_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[2], "3_", 3, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 3
    // @Group: 4_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[3], "4_", 4, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 4
    // @Group: 5_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[4], "5_", 5, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 5
    // @Group: 6_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[5], "6_", 6, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 6
    // @Group: 7_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[6], "7_", 7, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 7
    // @Group: 8_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[7], "8_", 8, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 8
    // @Group: 9_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[8], "9_", 9, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 9
    // @Group: 10_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[9], "10_", 10, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 10
    // @Group: 11_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[10], "11_", 11, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_IO > 11
    // @Group: 12_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_ioparams[11], "12_", 12, AP_CANIO, AP_CANIO_Params),
#endif


    // @Group: 1_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_aiparams[0], "AI1_", 13, AP_CANIO, AP_CANIO_Params),

#if AP_CANIO_NUM_AI > 1
    // @Group: 12_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_aiparams[1], "AI2_", 14, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_AI > 2
    // @Group: 12_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_aiparams[2], "AI3_", 15, AP_CANIO, AP_CANIO_Params),
#endif

#if AP_CANIO_NUM_AI > 3
    // @Group: 12_
    // @Path: AP_CANIO_Params.cpp
    AP_SUBGROUPINFO(_aiparams[3], "AI4_", 16, AP_CANIO, AP_CANIO_Params),
#endif

    AP_GROUPEND

};


// singleton instance
AP_CANIO *AP_CANIO::_singleton;


AP_CANIO::AP_CANIO()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_CANIO must be singleton");
    }
#endif
    _singleton = this;
}


void AP_CANIO::init()
{
    for (uint8_t i = 0; i < AP_CANIO_NUM_IO; i++) {
        // set the default
        _ioparams[i].pin.set_default(-1);
    }

    for (uint8_t i = 0; i < AP_CANIO_NUM_AI; i++) {
        // set the default
        _aiparams[i].pin.set_default(-1);
    }

    if (_driver != nullptr) {
            // only allow one instance
            return;
    }
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::CANIO) {
            _driver = NEW_NOTHROW AP_CANIO_Driver(_ioparams, _aiparams);
            return;
        }
    }
}

void AP_CANIO::set(AP_CANIO_Params::FUNCTION function, bool value) 
{ 
    if (_driver == nullptr) {
        return;
    }
    _driver->set(function, value);
}


bool AP_CANIO::read(AP_CANIO_Params::FUNCTION function, uint16_t & value) 
{ 
    if (_driver == nullptr) {
        return false;
    }
    return _driver->read(function, value); 
}


namespace AP {
    AP_CANIO *can_io()
    {
        return AP_CANIO::get_singleton();
    }
};


AP_CANIO_Driver::AP_CANIO_Driver(const AP_CANIO_Params *ioparams, const AP_CANIO_Params *aiparams) : 
                                CANSensor("CANIO"),
                                _ioparams(ioparams),
                                _aiparams(aiparams)
{
  
    register_driver(AP_CAN::Protocol::CANIO);
    init();

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CANIO_Driver::loop, void), "CANIO", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}


void AP_CANIO_Driver::init()
{
    set_defaults();

    // setup the actual default values of all the pins
    for (uint8_t instance = 0; instance < AP_CANIO_NUM_IO; instance++) {
        const int16_t pin = _ioparams[instance].pin;
        if (pin == -1) {
            // no valid pin to set it on, skip it
            continue;
        }

        const AP_CANIO_Params::FUNCTION function = _ioparams[instance].function;
        if (!function_valid(function)) {
            // invalid function, skip it
            continue;
        }

        // set the state to match our output
        const AP_CANIO_Params::DefaultState default_state = _ioparams[instance].default_state;
        if ((default_state == AP_CANIO_Params::DefaultState::OFF) ||
            (default_state == AP_CANIO_Params::DefaultState::ON)) {

            set_pin_by_instance(instance, (bool)default_state);
        }

    }
}


void AP_CANIO_Driver::set_defaults() {

    memset(&_write_io_group.all, 0, sizeof(_write_io_group.raw_data));
    memset(&_read_io_group.all, 0, sizeof(_read_io_group.raw_data));

    memset(_ai_groups, 0, sizeof(_ai_groups));
}


// Return true is function is valid
bool AP_CANIO_Driver::function_valid(AP_CANIO_Params::FUNCTION function) const
{
    return (function > AP_CANIO_Params::FUNCTION::NONE) && (function < AP_CANIO_Params::FUNCTION::NUM_FUNCTIONS);
}

void AP_CANIO_Driver::set(const AP_CANIO_Params::FUNCTION function, const bool value) {
    if (!function_valid(function)) {
        // invalid function
        return;
    }

    for (uint8_t instance = 0; instance < AP_CANIO_NUM_IO; instance++) {
        if (function != _ioparams[instance].function) {
            continue;
        }

        set_pin_by_instance(instance, value);
    }
}

// set a pins output state by instance and log if required
// this is an internal helper, instance must have already been validated to be in range
void AP_CANIO_Driver::set_pin_by_instance(uint8_t instance, bool value)
{
    const int16_t pin = _ioparams[instance].pin;
    if (pin == -1) {
        // no valid pin to set it on, skip it
        return;
    }

    if (_ioparams[instance].inverted > 0) {
        value = !value;
    }

    set_pin(pin, value);
}

 void AP_CANIO_Driver::get_pin_by_instance(uint8_t instance, uint16_t & value)
 {
    const int16_t pin = _ioparams[instance].pin;
    if (pin == -1) {
        // no valid pin to set it on, skip it
        return;
    }

    value = get_pin(pin);
 }


 void AP_CANIO_Driver::get_channel_by_instance(uint8_t instance, uint16_t & value)
 {
    const int16_t channel = _aiparams[instance].pin;
    if (channel == -1) {
        // no valid pin to set it on, skip it
        return;
    }

    const uint16_t group = channel / 4;
    const uint16_t index = channel - 4 * group;
    const uint16_t tmp_value = _ai_groups[group].data[index];
    value = ((tmp_value & 0xff) << 8) |((tmp_value & 0xff00) >> 8);
 }


// Get relay state from pin number
bool AP_CANIO_Driver::get_pin(const uint64_t pin) const
{
    // Read pins
    const uint64_t data = _read_io_group.all & ((uint64_t)(1) << (pin));
    const uint16_t pins = (data >> (pin)) & 0x01;
    return (bool)pins;
}


// Set relay state from pin number
void AP_CANIO_Driver::set_pin(const uint64_t pin, const bool value)
{
  
    // Real GPIO pin
    {
        // WITH_SEMAPHORE(sem);
        if (value == true) {
            _write_io_group.all |= ((uint64_t)(1)<<(pin));
        } else {
            _write_io_group.all &= (0xffffffff - ((uint64_t)(1)<<(pin)));      
        }
    }

}

// see if the relay is enabled
bool AP_CANIO_Driver::enabled(AP_CANIO_Params::FUNCTION function) const
{
    for (uint8_t instance = 0; instance < AP_CANIO_NUM_IO; instance++) {
        if ((_ioparams[instance].function == function) && (_ioparams[instance].pin != -1)) {
            return true;
        }
    }

    for (uint8_t instance = 0; instance < AP_CANIO_NUM_AI; instance++) {
        if ((_aiparams[instance].function == function) && (_aiparams[instance].pin != -1)) {
            return true;
        }
    }
    return false;
}


bool AP_CANIO_Driver::read(AP_CANIO_Params::FUNCTION function, uint16_t &value)
{

    if (!function_valid(function)) {
        // invalid function
        return false;
    }

    for (uint8_t instance = 0; instance < AP_CANIO_NUM_IO; instance++) {
        if (function != _ioparams[instance].function) {
            continue;
        }

        get_pin_by_instance(instance, value);
        return true;
        
 
    }

    for (uint8_t channel = 0; channel < AP_CANIO_NUM_AI; channel++) {
        if (function != _aiparams[channel].function) {
            continue;
        }

        get_channel_by_instance(channel, value);
        return true;
    
    }

    return false;
}


void AP_CANIO_Driver::loop()
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
 
            // write IO channel 50ms
            if (++write_io_counter >= 5) {
                write_io_counter = 0;

                send_messages((uint32_t)(CAN_ID::WRITE_IO), (uint8_t*)_write_io_group.raw_data, 8);
            }
            

            // read AI channel 20ms
            if (++read_ai_counter >= 2) {
                read_ai_counter = 0;
                send_messages((uint32_t)(CAN_ID::READ_AI), temp_data, 8);
            }

            // read IO channle 1000ms
            if (++read_io_counter >= 100) {
                read_io_counter = 0;
                send_messages((uint32_t)(CAN_ID::READ_IO), temp_data, 8);
            }
        }
        
    }
}

// send control mode over CAN
void AP_CANIO_Driver::send_messages(const uint32_t cand_id, const uint8_t *data, const uint8_t data_len)
{
    AP_HAL::CANFrame txFrame;
    uint32_t id = cand_id;

    txFrame = {(id |= AP_HAL::CANFrame::FlagEFF), data, data_len};
    write_frame(txFrame, 10000ULL);
}


// parse inbound frames
void AP_CANIO_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    switch (CAN_ID(id)) {

        case CAN_ID::READ_IO:
        {
            memcpy(&_read_io_group.all, frame.data, frame.dlc);
            break;
        }

        case CAN_ID::READ_AI:
        {
            memcpy(&_ai_groups[0].all, frame.data, frame.dlc);

            break;
        }

        case CAN_ID::WRITE_IO:
        {
            break;
        }
    }

}

#endif