#include "AP_Proximity_Dynamical_SITL.h"

#if HAL_PROXIMITY_ENABLED

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAX_RANGE 200.0f
#define PROXIMITY_ACCURACY 0.1f
#define PROXIMITY_OBJECT_MAX_RANGE   35.0f
#define PROXIMITY_OBJECT_MAX_VEL     2.0f
#define PROXIMITY_OBJECT_RADIUS      3.0f

const uint32_t mode_run_time_ms = 25000;
const uint8_t  object_num = 6;

// update the state of the sensor
void AP_Proximity_Dynamical_SITL::update(void)
{
    // get current vehicle postion
    Vector2f current_loc;
    if(!AP::ahrs().get_relative_position_NE_origin(current_loc)){
        set_status(AP_Proximity::Status::NoData);
        _state = Run_State::INIT_MODE;
        return;
    }
    const float current_bearing = AP::ahrs().yaw_sensor * 0.01f;

    // objects postion,velocity
    std::vector<Vector2f> _objects_loc(object_num);
    std::vector<Vector2f> _objects_vel(object_num);
    const float del_ang = 2* M_PI / object_num;
    switch (_state)
    {
        case Run_State::INIT_MODE:
            _center_loc = current_loc;
            _last_update_ms[0] = AP_HAL::millis();
            _state = Run_State::CIRCLE_MODE;
            return;
            break;
        case Run_State::CIRCLE_MODE:
        {
            const uint32_t eplase_time_circle = AP_HAL::millis() - _last_update_ms[0];
            if(eplase_time_circle > mode_run_time_ms){
                _center_loc = current_loc;
                _last_update_ms[1] = AP_HAL::millis();
                _state = Run_State::START_MODE;
                return;
            }
            for (size_t i = 0; i < object_num; i++) {
                const float omega = PROXIMITY_OBJECT_MAX_VEL / PROXIMITY_OBJECT_MAX_RANGE;
                const float pos_dir =  i * del_ang;
                const float time = eplase_time_circle * 0.001f;
                _objects_vel[i] = Vector2f{-sinf(pos_dir + omega * time), cosf(pos_dir + omega * time)} * PROXIMITY_OBJECT_MAX_RANGE * omega;
                _objects_loc[i] = _center_loc + Vector2f{cosf(pos_dir + omega * time), sinf(pos_dir + omega * time)} * PROXIMITY_OBJECT_MAX_RANGE;
            }
            break;
        }
        case Run_State::START_MODE:
        {
            const uint32_t eplase_time_start = AP_HAL::millis() - _last_update_ms[1] ;
            // update last state
            if (eplase_time_start> mode_run_time_ms) {
                _center_loc = current_loc;
                _last_update_ms[2] = AP_HAL::millis();
                _state = Run_State::PP_MODE;
                return;
            }
            for (size_t i = 0; i < object_num; i++) {
                const float pos_dir = i * del_ang;
                _objects_vel[i] = Vector2f{cosf(pos_dir + M_PI), sinf(pos_dir + M_PI)} * PROXIMITY_OBJECT_MAX_VEL * 0.5f;
                _objects_loc[i] = _center_loc + Vector2f{cosf(pos_dir), sinf(pos_dir)} * PROXIMITY_OBJECT_MAX_RANGE * 1.5f + _objects_vel[i] * eplase_time_start * 0.001f;
            }
            break;
        }
        case Run_State::PP_MODE:
        {
            const uint32_t eplase_time_start = AP_HAL::millis() - _last_update_ms[2] ;
            if (eplase_time_start> mode_run_time_ms) {
                _center_loc = current_loc;
                _last_update_ms[0] = AP_HAL::millis();
                _state = Run_State::CIRCLE_MODE;
                return;
            }
            for (size_t i = 0; i < object_num; i++) {
                const float    sign = (i % 2 == 0) ?(1.0f):(-1.0f);
                _objects_vel[i] = Vector2f{0, 1} * sign * PROXIMITY_OBJECT_MAX_VEL;
                _objects_loc[i] = _center_loc + Vector2f{0.5f * PROXIMITY_OBJECT_MAX_RANGE * (i+1) * sign, -PROXIMITY_OBJECT_MAX_RANGE * sign} + _objects_vel[i] * eplase_time_start * 0.001f;
            }
            break;
        }
        default:
            break;
    }

    // variables to calculate closest angle and distance for each face
    AP_Proximity_Boundary_3D::Face face;
    float face_distance = FLT_MAX;
    float face_yaw_deg = 0.0f;
    bool face_distance_valid = false;
    if (_state != Run_State::INIT_MODE) {
        // reset this  boundary to fill with new data
        boundary.reset();
        set_status(AP_Proximity::Status::Good);
        for (size_t i = 0; i< object_num; i++) {

            // get absolute postion,velocity and heading
            const float angle_deg   = wrap_360(degrees(_objects_loc[i].angle()));
            const float distance_m  = _objects_loc[i].length();
            const float vel_mag     = _objects_vel[i].length();
            const float vel_ang     =  wrap_360(degrees(_objects_vel[i].angle()));

            // get relative distance and heading
            const float distance_to_vehicle = (_objects_loc[i] - current_loc).length();
            const float direction_to_obstacle = degrees((_objects_loc[i] - current_loc).angle());
            const float relative_to_angle   = wrap_360(direction_to_obstacle - current_bearing);

            // check validation
            const bool range_check = distance_to_vehicle > distance_max() || distance_to_vehicle < distance_min();
            if (range_check || ignore_reading(relative_to_angle, distance_to_vehicle,false)) {
                continue;
            }

            // get face for this latest reading
            const AP_Proximity_Boundary_3D::Face latest_face = boundary.get_face(relative_to_angle);
            if (latest_face != face) {
                // store previous face
                if (face_distance_valid) {
                    boundary.set_face_attributes(face, face_yaw_deg, face_distance);
                } else {
                    boundary.reset_face(face);
                }
                // init for latest face
                face = latest_face;
                face_distance_valid = false;
            }

            // update minimum distance found so far
            if (!face_distance_valid || (distance_to_vehicle < face_distance)) {
                face_yaw_deg = relative_to_angle;
                face_distance = distance_to_vehicle;
                face_distance_valid = true;
            }

            // update object avoidance database with warth-frame point
            database_push(angle_deg, 0.0f, distance_m, vel_mag, vel_ang, PROXIMITY_OBJECT_RADIUS, false);
        }
        // process the last face
        if (face_distance_valid) {
            boundary.set_face_attributes(face, face_yaw_deg, face_distance);
        } else {
            boundary.reset_face(face);
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