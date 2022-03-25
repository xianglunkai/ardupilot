#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/hysteresis.h>

/*
 * Velocity obstacle avoidance for avoiding the polygon and circular fence and dynamical objects detected by the proximity sensor
*/

class AP_OAVelocityObstacle{
public:
    AP_OAVelocityObstacle();

    /* Do not allow copies */
    AP_OAVelocityObstacle(const AP_OAVelocityObstacle &other) = delete;
    AP_OAVelocityObstacle &operator=(const AP_OAVelocityObstacle) = delete;

    // send configuration info stored in front end parameters
    void set_config(float margin_max) { _margin_max = MAX(margin_max,0.0f); }

    // run background task to find best path and update avoidance_results
    // returns true and populates origin_new and destination_new if OA is required.  returns false if OA is not required
    bool update(const Location& current_loc, const Location& destination, const float &desired_speed,const Vector2f &ground_speed_vec, Location &origin_new,Location &destination_new, float &desired_speed_new,bool proximity_only);

    bool abandon_waypoint() const { return _abandon_wp; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    // OA common parameters
    float _margin_max;              // object avoidance will ignore objects more than this many meters from vehicle

    // algorithm parameters
    AP_Float _lookahead;            // object avoidance will look this many meters ahead of vehicle
    

    // internal variables used by background thread
    bool  _abandon_wp{false};       // give up current destination or not
};