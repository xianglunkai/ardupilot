#pragma once

#include "AP_Proximity.h"

#if HAL_PROXIMITY_ENABLED
#include "AP_Proximity_Backend.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#include <AP_Common/Location.h>
#include <algorithm>

class AP_Proximity_Dynamical_SITL : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_Dynamical_SITL(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

private:

    enum class Run_State{
        GET_CENTER = 0,
        CIRCLE_MODE = 1,
        START_MODE = 2,
    };

    // initial state
    Run_State _state{Run_State::GET_CENTER};

    const float _mode_run_time = 10;
    const uint8_t _object_num = 8;

    // mode center postion
    Vector2f _center_loc;

    // objects postion
    std::vector<Vector2f> _objects_loc;

    // last running time
    uint32_t _last_update_ms{0};

};
#endif // CONFIG_HAL_BOARD

#endif // HAL_PROXIMITY_ENABLED