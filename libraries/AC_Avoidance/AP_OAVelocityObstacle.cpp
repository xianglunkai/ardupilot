/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_OAVelocityObstacle.h"
#include <AC_Avoidance/AP_OADatabase.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

// parameter defaults
const float OA_VO_LOW_SPEED_SQUARED = (0.2f * 0.2f);    // when ground course is below this speed squared, vehicle's heading will be used
const float OA_VO_BEARING_INC_XY = 1;                 // check every 5 degrees around vehicle
const float OA_VO_SPEED_INC_XY = 0.2;                 // check every 0.2 m/s around vehicle

const AP_Param::GroupInfo AP_OAVelocityObstacle::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Type of VO
    // @Description: VO will search for clear path along the direction defined by this parameter
    // @Values: 0:disable 1:noly speed regulator 2: only heading regulator 3: speed and heading regulator at same time
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_OAVelocityObstacle, _vo_type, 3),

    // @Param: LOOKAHEAD
    // @DisplayName: Object Avoidance look ahead distance maximum
    // @Description: Object Avoidance will look this many meters ahead of vehicle
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOOKAHEAD", 2, AP_OAVelocityObstacle, _lookahead, 15.0f),

    // @Param: BR_DEV_MAX
    // @DisplayName: Object Avoidance deviate angle maximum
    // @Description: Object Avoidance will look this many degrees deviate current groundcrouse
    // @Units: degrees
    // @Range: 0 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("BRDEV_MAX", 3, AP_OAVelocityObstacle, _bearing_deviate_max, 20.0f),

    // @Param: SP_DEV_MAX
    // @DisplayName: Object Avoidance deviate speed maximum
    // @Description: Object Avoidance will look this many m/s deviate current speed
    // @Units: m/s
    // @Range: 0 1
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SPDEV_MAX", 4, AP_OAVelocityObstacle, _speed_deviate_max, 1.0f),

    AP_GROUPEND
};

AP_OAVelocityObstacle::AP_OAVelocityObstacle()
{
    AP_Param::setup_object_defaults(this,var_info);
    _bearing_prev = FLT_MAX;
    _speed_prev = FLT_MAX;
}

// run background task to find best path and update avoidance_results
// returns true and populates origin_new and destination_new if OA is required.  returns false if OA is not required
bool AP_OAVelocityObstacle::update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, float &desired_speed_new, bool proximity_only)
{
    // VO always sets origin to current_loc
    origin_new = current_loc;

    // calculate bearing and distance to final destination
    //const float bearing_to_dest = current_loc.get_bearing_to(destination) * 0.01f;
    const float distance_to_dest = current_loc.get_distance(destination);

    // make sure user has set a meaningful value for _lookahead
    _lookahead = MAX(_lookahead,1.0f);

    // get ground course
    float ground_course_deg;
    Vector3f speed_vec{ground_speed_vec,0.0f};
    if (ground_speed_vec.length_squared() < OA_VO_LOW_SPEED_SQUARED) {
        // with zero ground speed use vehicle's heading
        ground_course_deg = AP::ahrs().yaw_sensor * 0.01f;
        speed_vec = Vector3f(cosf(AP::ahrs().yaw), sinf(AP::ahrs().yaw),0.0f) * sqrtf(OA_VO_LOW_SPEED_SQUARED);
    } else {
        ground_course_deg = degrees(ground_speed_vec.angle());
    }

    // convert current location to offsets (in cm) from EKF origin
    Vector3f vehicle_pos{};
    if (!current_loc.get_vector_from_origin_NEU(vehicle_pos)) {
        return false;
    }
    vehicle_pos = vehicle_pos * 0.01f;

    float desired_bearing = 0;
    bool ret = true;
    switch ((VO_TYPE)_vo_type.get())
    {
        case VO_TYPE::NONE:
        // todo: use AP_Avoidance as a report class to tell GCS threat level around vehicle
        case VO_TYPE::SPEED_REGULATOR:
        // only regulator desired speed for avodiance,but still change bearing when encounter static obstacles
            break;
        case VO_TYPE::BEARING_REGULATOR:
        // only regulator bearing
            break;
        case VO_TYPE::BEARING_SPEED_REGULATOR:
        // regulator speed and bearing at same time
            ret = search_xy_path(vehicle_pos,speed_vec,ground_course_deg,desired_bearing,desired_speed_new,proximity_only);
            destination_new = current_loc;
            destination_new.offset_bearing(desired_bearing,MIN(_lookahead,distance_to_dest));
            break;
        default:
            break;
    }

    return ret;
}


bool AP_OAVelocityObstacle::search_xy_path(const Vector3f &vehicle_pos, const Vector3f &vehicle_speed, float ground_course_deg,float &desired_bearing, float &desired_speed, bool proximity_only)
{
    // search in OA_BENDYRULER_BEARING_INC degree increments around the vehicle alternating left
    // and right. For each direction check if vehicle would avoid all obstacles
    
    for (uint16_t i = 0; i <= (_bearing_deviate_max / OA_VO_BEARING_INC_XY); i++) {
        for (uint16_t bdir = 0; bdir <= 1; bdir++) {
            // skip duplicate check of bearing straight towards destination
            if ((i==0) && (bdir > 0)) {
                continue;
            }
            // bearing that we are probing
            const float bearing_delta = i * OA_VO_BEARING_INC_XY * (bdir == 0 ? -1.0f : 1.0f);

            // probing speed change 
            for (uint16_t j = 0; j <= (_speed_deviate_max / OA_VO_SPEED_INC_XY); j++) {
                for (uint16_t sdir = 0; sdir <= 1; sdir++) {
                    // skip j == 0 && sdir > 0
                    if ((j == 0) && (sdir > 0)) {
                        continue;
                    }
                    const float speed_delta = j * OA_VO_SPEED_INC_XY * (sdir == 0 ? -1.0f : 1.0f);

                    // calculate margin from obstacle for this scenario
                    float margin = calc_avoidance_margin(vehicle_pos,vehicle_speed,bearing_delta,speed_delta,proximity_only);
                    const float test_speed = speed_delta + vehicle_speed.length();

                    if (margin > 0.0f && test_speed > 1.0f) {
                        const bool active = (!i == 0 || !j ==0);
                        desired_bearing = wrap_360(ground_course_deg + bearing_delta);
                        desired_speed = test_speed;
                        return active;
                    }
                }
            } 
        }
    }

    desired_speed = 0.0f;
    desired_bearing = ground_course_deg;

    return true;
}

// calculate minimum distance between a path and any obstacle
float AP_OAVelocityObstacle::calc_avoidance_margin(const Vector3f &vehicle_pos, const Vector3f &vehicle_speed, const float &delta_bearing, const float &delta_speed, bool proximity_only) const
{
    float margin_min = FLT_MAX;
    
    float latest_margin;

    if (calc_margin_from_object_database(vehicle_pos, vehicle_speed, delta_bearing, delta_speed, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }
    
    if (proximity_only) {
        // only need margin from proximity data
        return margin_min;
    }

    return margin_min;
}

// calculate minimum angle between a path and proximity sensor obstacles
// on success returns true and updates margin
 bool AP_OAVelocityObstacle::calc_margin_from_object_database(const Vector3f &vehicle_pos, const Vector3f &vehicle_speed, const float &delta_bearing, const float &delta_speed, float &margin) const
 {
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    float smallest_margin = FLT_MAX;
    for (uint16_t i = 0; i< oaDb->database_count();i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const Vector3f rel_pos = item.pos - vehicle_pos;
        const Vector3f rel_vel = vehicle_speed - item.vel;
        // only obstacles in the designated area shall be considered
        if(rel_pos.length() > _lookahead + item.radius || rel_pos.is_zero()){
            continue;
        }
        // determine whether they will meet in the furture
        const float mu = asinf((item.radius + _margin_max)/rel_pos.length());
        const float gama = wrap_PI(rel_vel.xy().angle() - rel_pos.xy().angle());
        const float fai = wrap_PI(vehicle_speed.xy().angle() - rel_vel.xy().angle());
        const float m = fabsf((-sinf(fai) * delta_speed + vehicle_speed.length() * cosf(fai) * radians(delta_bearing))/(rel_vel.length() + FLT_EPSILON) +
                    gama) - mu;
        if(m < smallest_margin){
            smallest_margin = m;
        }
    }

     // return smallest margin
    if (smallest_margin < FLT_MAX) {
        margin = smallest_margin;
        return true;
    }

    return false;
 }