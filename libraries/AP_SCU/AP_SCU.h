#pragma once

#include "AP_SCU_Config.h"

#if AP_SCU_ENABLED
#include "engine.h"
#include "steering.h"
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_SCU {
public:
    // constructor
    AP_SCU();
 
    /* Do not allow copies */
    CLASS_NO_COPY(AP_SCU);


   // engine start/stop control command
    enum class Engine_Cmd: uint16_t {
        NO_OPS    = 0,
        REQ_START = 1,
        REQ_STOP  = 2
    };

    // lift down position
    enum class Lift_Pos: uint16_t {
        NORMAL = 0,
        UP     = 1,
        DOWN   = 2,
    };


    // init - performs any required initialisation for this instance
    void init();

    // update position - should be called periodically
    void update();

    static AP_SCU *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    ///  moves servo with the given function id to the specified angle.  all angles are in body-frame and degrees * 10
    void move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    void setup_servo_output();

    void setup_pwm_type();

    void setup_safety_output();

    void generate_commands();

    AP_SCU_Engine   _engine;
    AP_SCU_Steering _steering;

    uint32_t _last_update_ms;
    bool _first_update {true};

    // control commands
    float _throttle_cmd; // [-1 1]
    float _steering_cmd; // [-1 1]
    uint8_t _lift_cmd;   // [0 1 2]
    uint8_t _sts_cmd;    // [0 1 2]


    static AP_SCU *_singleton;

private:
    // SRV_Channel - different id numbers are used depending upon the instance number
    AP_Int16    _throttle_idx;  // SRV_Channel throttle function index
    AP_Int16   _steering_idx;  // SRV_Channel steering function index

    AP_Int8 enable;
    AP_Float steering_pwm_freq_khz;
    AP_Float throttle_pwm_freq_khz;
    AP_Int8  steering_relay_on;           // relay value

};


namespace AP {
    AP_SCU *scu();
};

#endif