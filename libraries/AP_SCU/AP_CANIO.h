#pragma once

#include "AP_SCU_Config.h"

#if AP_CANIO_ENABLE
#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>


class AP_CANIO_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_CANIO_Params();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_CANIO_Params);

    enum class DefaultState : uint8_t {
        OFF = 0,
        ON = 1,
        NO_CHANGE = 2,
    };
    
    enum class FUNCTION : uint8_t {
        NONE = 0,
        ENG_ACC = 2,
        ENG_START = 3,
        ENG_STOP = 4,
        LED_RED = 5,
        LED_YELLOW = 6,
        LED_GREEN = 7,
        SPEAKER = 8,
        STEER_ANG = 9,
        THR_ANG = 10,
        NUM_FUNCTIONS // must be the last entry
    };

    AP_Enum<FUNCTION> function;
    AP_Int16 pin;
    AP_Enum<DefaultState> default_state;  // default state
    AP_Int8 inverted;                     // inverted signal
};


#ifndef AP_CANIO_NUM_IO
  #define AP_CANIO_NUM_IO 12
#endif

#if AP_CANIO_NUM_IO < 1
  #error There must be at least one relay instance if using AP_CANIO
#endif

#ifndef AP_CANIO_NUM_AI
  #define AP_CANIO_NUM_AI 4
#endif

#if AP_CANIO_NUM_AI < 1
  #error There must be at least one relay instance if using AP_CANIO
#endif

static constexpr uint16_t AP_CANIO_NUM_AI_GROUP = 5;


/// @class	AP_CANIO
/// @brief	Class to manage the ArduPilot relay
class AP_CANIO_Driver : public CANSensor
{
public:
    AP_CANIO_Driver(const AP_CANIO_Params *_ioparams, const AP_CANIO_Params *_aiparams);

    enum class CAN_ID : uint32_t {
        WRITE_IO = 0xAA0101,
        READ_IO  = 0xAA0201,
        READ_AI  = 0xAA0303,
    };

    union PACKED IOGroup {
        uint64_t all;
        uint8_t raw_data[8];
    };

    union PACKED AIGroup {
      uint8_t raw_data[8];
      uint16_t data[4];
      uint64_t all;
    };
    

    // setup the relay pin
    void        init();

    void set(AP_CANIO_Params::FUNCTION function, bool value);

    bool read(AP_CANIO_Params::FUNCTION function, uint16_t & value);

    // see if the relay is enabled
    bool enabled(AP_CANIO_Params::FUNCTION function) const;


private:

    // Return true is function is valid
    bool function_valid(AP_CANIO_Params::FUNCTION function) const;

    void set_defaults();

    void set_pin_by_instance(uint8_t instance, bool value);

    void get_pin_by_instance(uint8_t instance, uint16_t & value);

    void get_channel_by_instance(uint8_t instance, uint16_t & value);

    // Set relay state from pin number
    void set_pin(const uint64_t pin, const bool value);

    // Get relay state from pin number
    bool get_pin(const uint64_t pin) const;

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    void send_messages(const uint32_t cand_id, const uint8_t *data, const uint8_t data_len);

    void loop();

private:

    const AP_CANIO_Params *_ioparams;

    const AP_CANIO_Params *_aiparams;

    // HAL_Semaphore sem;
    IOGroup _write_io_group;

    IOGroup _read_io_group;
 
    AIGroup _ai_groups[AP_CANIO_NUM_AI_GROUP];
};

class AP_CANIO {
public:
    AP_CANIO();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_CANIO);

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    void set(AP_CANIO_Params::FUNCTION function, bool value);
    

    bool read(AP_CANIO_Params::FUNCTION function, uint16_t & value);


    static AP_CANIO *get_singleton() { return _singleton; }

private:
    static AP_CANIO *_singleton;

    AP_CANIO_Driver *_driver;

    AP_CANIO_Params _ioparams[AP_CANIO_NUM_IO];

    AP_CANIO_Params _aiparams[AP_CANIO_NUM_AI];
};


namespace AP {
    AP_CANIO *can_io();
};

#endif