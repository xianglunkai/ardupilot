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
        RELAY = 1,
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
    AP_Int8 inverted;                       // inverted signal
};


#ifndef AP_CANIO_NUM_RELAYS
  #define AP_CANIO_NUM_RELAYS 16
#endif

#if AP_CANIO_NUM_RELAYS < 1
  #error There must be at least one relay instance if using AP_CANIO
#endif

/// @class	AP_CANIO
/// @brief	Class to manage the ArduPilot relay
class AP_CANIO : public CANSensor
{
public:
    AP_CANIO();

    // /* Do not allow copies */
    CLASS_NO_COPY(AP_CANIO);

    enum class CAN_ID : uint32_t {
        WRITE_IO = 0xAA0101,
        READ_IO  = 0xAA0201,
        READ_AI  = 0xAA0303,
    };


    union PACKED KOPins {
        struct {
            uint64_t   KO1:1;          /*   bit:0     */
            uint64_t   KO2:1;          /*   bit:1     */
            uint64_t   KO3:1;          /*   bit:2     */
            uint64_t   KO4:1;          /*   bit:3     */
            uint64_t   KO5:1;          /*   bit:4     */
            uint64_t   KO6:1;          /*   bit:5     */
            uint64_t   KO7:1;          /*   bit:0     */
            uint64_t   KO8:1;          /*   bit:1     */
            uint64_t   rsvd:56;
        } data = {};

        uint64_t all;

        uint8_t raw_data[8];
    };

    union PACKED AIChannel {
      struct {
        uint64_t steering_pos:16;
        uint64_t throttle_pos:16;
        uint64_t rsvd:32;
      };
      uint8_t raw_data[8];
    };
    

    // setup the relay pin
    void        init();

    // activate the IO
    void        on(uint8_t instance) { set(instance, true); }

    // de-activate the IO
    void        off(uint8_t instance) { set(instance, false); }

    // get state of IO
    bool        get(uint8_t instance) const;

    // toggle the IO status
    void        toggle(uint8_t instance);

    void set(AP_CANIO_Params::FUNCTION function, bool value);

    bool read(AP_CANIO_Params::FUNCTION function, float & value);

    // see if the relay is enabled
    bool enabled(AP_CANIO_Params::FUNCTION function) const;

    static const struct AP_Param::GroupInfo var_info[];

    static AP_CANIO *get_singleton() { return _singleton; }

private:
    static AP_CANIO *_singleton;

    AP_CANIO_Params _params[AP_CANIO_NUM_RELAYS];

    AP_Int8 debug;

    // Return true is function is valid
    bool function_valid(AP_CANIO_Params::FUNCTION function) const;

    void set(uint8_t instance, bool value);

    void set_defaults();

    void set_pin_by_instance(uint8_t instance, bool value);

    // Set relay state from pin number
    void set_pin(const int16_t pin, const bool value);

    // Get relay state from pin number
    bool get_pin(const int16_t pin) const;

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    void send_messages(const uint32_t cand_id, const uint8_t *data, const uint8_t data_len);

    void loop();

private:
    // HAL_Semaphore sem;
    KOPins _write_pins;
    KOPins _read_pins;
    bool _pins_is_new {false};

    float _throttle_pos;
    bool _steering_pos;
    bool _ai_is_new {false};
};

namespace AP {
    AP_CANIO *can_io();
};

#endif