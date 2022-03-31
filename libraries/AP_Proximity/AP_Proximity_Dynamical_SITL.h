#pragma once


#include "AP_Proximity_Backend.h"

#if HAL_PROXIMITY_ENABLED

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#include <AP_Common/Location.h>
#include <algorithm>

class AP_Proximity_Dynamical_SITL : public AP_Proximity_Backend
{

public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

private:

    enum class Run_State{
        INIT_MODE = 0,
        CIRCLE_MODE = 1,
        START_MODE = 2,
        PP_MODE  = 3,
    };

    // initial state
    Run_State _state{Run_State::INIT_MODE};
    // mode center postion
    Vector2f _center_loc;
    // last running time
    uint32_t _last_update_ms[3];

};
#endif // CONFIG_HAL_BOARD

#endif // HAL_PROXIMITY_ENABLED