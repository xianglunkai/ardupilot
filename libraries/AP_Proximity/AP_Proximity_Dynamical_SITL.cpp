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
#define PROXIMITY_CENTER_OFFSET_DIST 30
#define PROXIMITY_OBJECT_MAX_RANGE   30.0f
#define PROXIMITY_OBJECT_MAX_VEL     2.0f
#define PROXIMITY_OBJECT_RADIUS      3.0f


const uint32_t mode_run_time_ms = 30000;
const uint8_t  object_num = 4;

// update the state of the sensor
void AP_Proximity_Dynamical_SITL::update(void)
{
    // get current vehicle postion
    Vector2f current_loc;
    if(!AP::ahrs().get_relative_position_NE_origin(current_loc)){
        set_status(AP_Proximity::Status::NoData);
        _state = Run_State::GET_CENTER;
        _last_state = Run_State::CIRCLE_MODE;
        return;
    }
    const float current_bearing = AP::ahrs().yaw_sensor * 0.01f;

    // objects postion,velocity
    std::vector<Vector2f> _objects_loc(object_num);
    std::vector<Vector2f> _objects_vel(object_num);
    const float del_ang = 2* M_PI / object_num;

    switch (_state)
    {
    case Run_State::GET_CENTER:
        _center_loc =  current_loc;
        _center_loc.offset_bearing(current_bearing,PROXIMITY_CENTER_OFFSET_DIST);
        _state = (_last_state == Run_State::CIRCLE_MODE) ?(Run_State::START_MODE):(Run_State::CIRCLE_MODE);
        _last_update_ms = AP_HAL::millis();
        _last_state = Run_State::GET_CENTER;
        break;
    case Run_State::CIRCLE_MODE:
    {
         const uint32_t eplase_time_circle = AP_HAL::millis() - _last_update_ms ;
         _last_state = Run_State::CIRCLE_MODE;
        if(eplase_time_circle > mode_run_time_ms){
            _state = Run_State::GET_CENTER;
            return;
        }
        for(size_t i = 0; i < object_num; i++){
           
            _objects_vel[i].x = -PROXIMITY_OBJECT_MAX_RANGE * 
                                sinf(i * del_ang +  PROXIMITY_OBJECT_MAX_VEL / PROXIMITY_OBJECT_MAX_RANGE * eplase_time_circle * 0.001f) * 
                                PROXIMITY_OBJECT_MAX_VEL / PROXIMITY_OBJECT_MAX_RANGE;

            _objects_vel[i].y = PROXIMITY_OBJECT_MAX_RANGE *
                                 cosf(i * del_ang +  PROXIMITY_OBJECT_MAX_VEL / PROXIMITY_OBJECT_MAX_RANGE * eplase_time_circle * 0.001f) * 
                                PROXIMITY_OBJECT_MAX_VEL / PROXIMITY_OBJECT_MAX_RANGE;   

            _objects_loc[i].x = _center_loc.x +  PROXIMITY_OBJECT_MAX_RANGE *
                                cosf( i * del_ang +  PROXIMITY_OBJECT_MAX_VEL / PROXIMITY_OBJECT_MAX_RANGE * eplase_time_circle * 0.001f) ;

            _objects_loc[i].y = _center_loc.y + PROXIMITY_OBJECT_MAX_RANGE * 
                                sinf( i * del_ang + PROXIMITY_OBJECT_MAX_VEL / PROXIMITY_OBJECT_MAX_RANGE * eplase_time_circle * 0.001f) ;
        }
    }
        break;
    case Run_State::START_MODE:
    {
        const uint32_t eplase_time_start = AP_HAL::millis() - _last_update_ms ;
        // update last state
        _last_state = Run_State::START_MODE;
        if(eplase_time_start> mode_run_time_ms){
            _state = Run_State::GET_CENTER;
            return;
        }
        for(size_t i = 0; i < object_num; i++){
            const float vel_dir = wrap_2PI(i * del_ang);

            _objects_vel[i].x =  PROXIMITY_OBJECT_MAX_VEL * cosf(vel_dir);

            _objects_vel[i].y =  PROXIMITY_OBJECT_MAX_VEL * sinf(vel_dir);

            _objects_loc[i].x =  _center_loc.x +  _objects_vel[i].x * eplase_time_start * 0.001f;

            _objects_loc[i].y =  _center_loc.y  +  _objects_vel[i].y * eplase_time_start * 0.001f;
        }
    }
        break;
    default:
        break;
    }

    // send proximity data into OA 
    if(_last_state != Run_State::GET_CENTER){
        set_status(AP_Proximity::Status::Good);
        for(size_t i = 0; i< object_num; i++){
            const float angle_deg   = wrap_360(degrees(_objects_loc[i].angle()));
            const float distance_m  = _objects_loc[i].length();
            const float vel_mag     = _objects_vel[i].length();
            const float vel_ang     =  wrap_360(degrees(_objects_vel[i].angle()));

            const AP_Proximity_Boundary_3D::Face face = boundary.get_face(angle_deg);
            const float distance_to_vehicle = (_objects_loc[i] - current_loc).length();
            const float relative_to_angle   = wrap_360(angle_deg - current_bearing);

            if (!ignore_reading(relative_to_angle, distance_to_vehicle)) {
                if ((distance_to_vehicle <= distance_max()) && (distance_to_vehicle >= distance_min())) {
                    boundary.set_face_attributes(face, relative_to_angle, distance_to_vehicle);
                    // update OA database
                    database_push(angle_deg,0.0f,distance_m,vel_mag,vel_ang,PROXIMITY_OBJECT_RADIUS,false);
                } else {
                    // invalidate distance of face
                    boundary.reset_face(face);
                }
            }
        }
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