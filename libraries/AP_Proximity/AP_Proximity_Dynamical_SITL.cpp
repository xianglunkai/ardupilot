#include "AP_Proximity_Dynamical_SITL.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAX_RANGE 200.0f
#define PROXIMITY_ACCURACY 0.1f
#define PROXIMITY_OFFSET_DIST 50

/* 
   The constructor also initialises the proximity sensor. 
*/
AP_Proximity_Dynamical_SITL::AP_Proximity_Dynamical_SITL(AP_Proximity &_frontend,
                                     AP_Proximity::Proximity_State &_state):
    AP_Proximity_Backend(_frontend, _state)
{
 
}

// update the state of the sensor
void AP_Proximity_Dynamical_SITL::update(void)
{
    // get current vehicle postion
    Vector2f current_loc;
    if(!AP::ahrs().get_relative_position_NE_origin(current_loc)){
        _state = Run_State::GET_CENTER;
        return;
    }
    const float current_bearing = AP::ahrs().yaw_sensor * 0.01f;

    switch (_state)
    {
    case Run_State::GET_CENTER:
        _center_loc =  current_loc;
        _center_loc.offset_bearing(current_bearing,PROXIMITY_OFFSET_DIST);
        _state = Run_State::CIRCLE_MODE;
        break;
    case Run_State::CIRCLE_MODE:

        break;
    case Run_State::START_MODE:
        break;
    default:
        break;
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_Dynamical_SITL::distance_max() const
{
    return PROXIMITY_MAX_RANGE;
}

float AP_Proximity_Dynamical_SITL::distance_min() const
{
    return 0.0f;
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_Dynamical_SITL::get_upward_distance(float &distance) const
{
    // return distance to fence altitude
   return false;
}



#endif // CONFIG_HAL_BOARD

#endif // HAL_PROXIMITY_ENABLED