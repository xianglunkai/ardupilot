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

#include "AP_OAPathPlanner.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Planning/trajectory_stitcher.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_OABendyRuler.h"
#include "AP_OADijkstra.h"

extern const AP_HAL::HAL &hal;

// parameter defaults
const float OA_MARGIN_MAX_DEFAULT = 5;
const int16_t OA_OPTIONS_DEFAULT = 1;

const int16_t OA_UPDATE_MS  = 1000;     // path planning updates run at 1hz
const int16_t OA_TIMEOUT_MS = 3000;     // results over 3 seconds old are ignored
const float   OA_LOOKAHEAD_M = 15.0f;   // minimal track line length
const size_t  FLAGS_trajectory_stitching_preserved_length = 20;
const bool    FLAGS_enable_trajectory_stitcher = true;
const bool    FLAGS_replan_by_offset = true;

const AP_Param::GroupInfo AP_OAPathPlanner::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Object Avoidance Path Planning algorithm to use
    // @Description: Enabled/disable path planning around obstacles
    // @Values: 0:Disabled,1:BendyRuler,2:Dijkstra,3:Dijkstra with BendyRuler
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1,  AP_OAPathPlanner, _type, OA_PATHPLAN_DISABLED, AP_PARAM_FLAG_ENABLE),

    // Note: Do not use Index "2" for any new parameter
    //       It was being used by _LOOKAHEAD which was later moved to AP_OABendyRuler 

    // @Param: MARGIN_MAX
    // @DisplayName: Object Avoidance wide margin distance
    // @Description: Object Avoidance will ignore objects more than this many meters from vehicle
    // @Units: m
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MARGIN_MAX", 3, AP_OAPathPlanner, _margin_max, OA_MARGIN_MAX_DEFAULT),

    // @Group: DB_
    // @Path: AP_OADatabase.cpp
    AP_SUBGROUPINFO(_oadatabase, "DB_", 4, AP_OAPathPlanner, AP_OADatabase),

    // @Param: OPTIONS
    // @DisplayName: Options while recovering from Object Avoidance
    // @Description: Bitmask which will govern vehicles behaviour while recovering from Obstacle Avoidance (i.e Avoidance is turned off after the path ahead is clear).   
    // @Bitmask{Rover}: 0: Reset the origin of the waypoint to the present location, 1: log Dijkstra points
    // @Bitmask{Copter}: 1: log Dijkstra points
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 5, AP_OAPathPlanner, _options, OA_OPTIONS_DEFAULT),

    // @Group: BR_
    // @Path: AP_OABendyRuler.cpp
    AP_SUBGROUPPTR(_oabendyruler, "BR_", 6, AP_OAPathPlanner, AP_OABendyRuler),

    // @Group: SP_
    // @Path: AP_SpeedDecider.cpp
    AP_SUBGROUPPTR(_speed_decider, "SP_", 7, AP_OAPathPlanner, AP_SpeedDecider),

#if APM_BUILD_TYPE(APM_BUILD_Rover)
    // @Group: SL_
    // @Path: AP_ShorelineAvoid.cpp
    AP_SUBGROUPINFO(_oashoreline, "SL_", 8, AP_OAPathPlanner, AP_ShorelineAvoid),

    // @Group: SO_
    // @Path: AP_ShallowAvoid.cpp
    AP_SUBGROUPINFO(_oashallow, "SO_", 9, AP_OAPathPlanner, AP_ShallowAvoid),
#endif

    // @Group: DP_
    // @Path: AP_SLTPlanner.cpp
    AP_SUBGROUPPTR(_slt_planner, "DP_", 10, AP_OAPathPlanner, AP_SLTPlanner),

    AP_GROUPEND
};

/// Constructor
AP_OAPathPlanner::AP_OAPathPlanner()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// perform any required initialisation
void AP_OAPathPlanner::init()
{
    // run background task looking for best alternative destination
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        return;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            _oabendyruler = new AP_OABendyRuler();
            AP_Param::load_object_from_eeprom(_oabendyruler, AP_OABendyRuler::var_info);
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
#if AP_FENCE_ENABLED
        if (_oadijkstra == nullptr) {
            _oadijkstra = new AP_OADijkstra(_options);
        }
#endif
        break;
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
#if AP_FENCE_ENABLED
        if (_oadijkstra == nullptr) {
            _oadijkstra = new AP_OADijkstra(_options);
        }
#endif
        if (_oabendyruler == nullptr) {
            _oabendyruler = new AP_OABendyRuler();
            AP_Param::load_object_from_eeprom(_oabendyruler, AP_OABendyRuler::var_info);
        }
        break;

    case OA_EM:
        if (_speed_decider == nullptr) {
            _speed_decider = new AP_SpeedDecider();
            AP_Param::load_object_from_eeprom(_speed_decider, AP_SpeedDecider::var_info);
        }
        break;
    case OA_SLT:
        if (_slt_planner == nullptr) {
            _slt_planner = new AP_SLTPlanner();
            AP_Param::load_object_from_eeprom(_slt_planner, AP_SLTPlanner::var_info);
        }
        break;
    }

    _oadatabase.init();
    start_thread();
}

// pre-arm checks that algorithms have been initialised successfully
bool AP_OAPathPlanner::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // check if initialisation has succeeded
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        break;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "BendyRuler OA requires reboot");
            return false;
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
        if (_oadijkstra == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Dijkstra OA requires reboot");
            return false;
        }
        break;
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
        if(_oadijkstra == nullptr || _oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "OA requires reboot");
            return false;
        }
        break;
    case OA_EM:
        if (_speed_decider == nullptr || _oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "OA requires reboot");
        }
        break;
    case OA_SLT:
        if (_slt_planner == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "OA requires reboot");
        }
        break;
    }

    return true;
}

bool AP_OAPathPlanner::start_thread()
{
    WITH_SEMAPHORE(_rsem);

    if (_thread_created) {
        return true;
    }
    if (_type == OA_PATHPLAN_DISABLED) {
        return false;
    }

    // create the avoidance thread as low priority. It should soak
    // up spare CPU cycles to fill in the avoidance_result structure based
    // on requests in avoidance_request
    uint32_t stack_size = 8192;
    #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_IMX_K60 || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_IMX
        stack_size = 1024 * 1024;
    #endif
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OAPathPlanner::avoidance_thread, void),
                                    "avoidance",
                                    stack_size, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
     return false;
    }
    _thread_created = true;
    return true;
}

// helper function to map OABendyType to OAPathPlannerUsed
AP_OAPathPlanner::OAPathPlannerUsed AP_OAPathPlanner::map_bendytype_to_pathplannerused(AP_OABendyRuler::OABendyType bendy_type)
{
    switch (bendy_type) {
    case AP_OABendyRuler::OABendyType::OA_BENDY_HORIZONTAL:
        return OAPathPlannerUsed::BendyRulerHorizontal;

    case AP_OABendyRuler::OABendyType::OA_BENDY_VERTICAL:
        return OAPathPlannerUsed::BendyRulerVertical;

    default:
    case AP_OABendyRuler::OABendyType::OA_BENDY_DISABLED:
        return OAPathPlannerUsed::None;
    }
}

// provides an alternative target location if path planning around obstacles is required
// returns true and updates result_loc with an intermediate location
AP_OAPathPlanner::OA_RetState AP_OAPathPlanner::mission_avoidance(const Location &current_loc,
                                         const Location &origin,
                                         const Location &destination,
                                         const float &desired_speed,
                                         Location &result_origin,
                                         Location &result_destination,
                                         float &result_desired_speed,
                                         OAPathPlannerUsed &path_planner_used)
{
    // exit immediately if disabled or thread is not running from a failed init
    if (_type == OA_PATHPLAN_DISABLED || !_thread_created) {
        return OA_NOT_REQUIRED;
    }

    const uint32_t now = AP_HAL::millis();
    WITH_SEMAPHORE(_rsem);

    // place new request for the thread to work on
    avoidance_request.current_loc = current_loc;
    avoidance_request.origin = origin;
    avoidance_request.destination = destination;
    avoidance_request.desired_speed = desired_speed;
    avoidance_request.ground_speed_vec = AP::ahrs().groundspeed_vector();
    avoidance_request.request_time_ms = now;

    // check result's destination matches our request
    const bool destination_matches = (destination.lat == avoidance_result.destination.lat) && (destination.lng == avoidance_result.destination.lng);

    // check results have not timed out
    const bool timed_out = now - avoidance_result.result_time_ms > OA_TIMEOUT_MS;

    // return results from background thread's latest checks
    if (destination_matches && !timed_out) {
        // we have a result from the thread
        result_origin = avoidance_result.origin_new;
        result_destination = avoidance_result.destination_new;
        result_desired_speed = avoidance_result.desired_speed_new;
        path_planner_used = avoidance_result.path_planner_used;

        // override result_origin, result_destination, result_desired_speed
        if (path_planner_used == OAPathPlannerUsed::SLT &&
            avoidance_result.last_publishable_trajectory.size() > 0) {
            result_origin = current_loc;
            const float relative_time = (now - avoidance_result.last_publishable_trajectory.header_time()) * 0.001f;
            planning::TrajectoryPoint track_point = avoidance_result.last_publishable_trajectory.Evaluate(relative_time); 

            // update result_desired_speed
            result_desired_speed = track_point.v;

            // do not consider stop trajectory
            if (result_desired_speed > 0) {
                // translate track point in X-Y into LAT-LNG
                Location temp_loc(Vector3f{track_point.path_point.x * 100, 
                                        track_point.path_point.y * 100, 0.0},
                                        Location::AltFrame::ABOVE_ORIGIN);

                // ensure track line minimal length is grater OA_LOOKAHEAD_M
                result_destination = temp_loc;
                const float desired_bearing = result_origin.get_bearing_to(result_destination) * 0.01f;
                const float desired_distance = result_origin.get_distance(result_destination);
                if (desired_distance < OA_LOOKAHEAD_M) {
                    result_destination = current_loc;
                    result_destination.offset_bearing(desired_bearing, OA_LOOKAHEAD_M);
                }
            }
        }

        return avoidance_result.ret_state;
    }

    // if timeout then path planner is taking too long to respond
    if (timed_out) {
        avoidance_result.last_publishable_trajectory.clear();
        return OA_ERROR;
    }

    // background thread is working on a new destination
    return OA_PROCESSING;
}

// avoidance thread that continually updates the avoidance_result structure based on avoidance_request
void AP_OAPathPlanner::avoidance_thread()
{
    // require ekf origin to have been set
    bool origin_set = false;
    while (!origin_set) {
        hal.scheduler->delay(500);
        struct Location ekf_origin {};
        {
            WITH_SEMAPHORE(AP::ahrs().get_semaphore());
            origin_set = AP::ahrs().get_origin(ekf_origin);    
        }
    }

    while (true) {

        // if database queue needs attention, service it faster
        if (_oadatabase.process_queue()) {
            hal.scheduler->delay(1);
        } else {
            hal.scheduler->delay(20);
        }

        const uint32_t now = AP_HAL::millis();
        if (now - avoidance_latest_ms < OA_UPDATE_MS) {
            continue;
        }
        avoidance_latest_ms = now;

        _oadatabase.update();

        Location origin_new;
        Location destination_new;
        float desired_speed_new;
        {
            WITH_SEMAPHORE(_rsem);
            if (now - avoidance_request.request_time_ms > OA_TIMEOUT_MS) {
                // this is a very old request, don't process it
                avoidance_result.ret_state = OA_NOT_REQUIRED;
                continue;
            }

            // copy request to avoid conflict with main thread
            avoidance_request2 = avoidance_request;

            // store passed in origin and destination so we can return it if object avoidance is not required
            origin_new = avoidance_request.origin;
            destination_new = avoidance_request.destination;
            desired_speed_new = avoidance_request.desired_speed;
        }

        // planning is triggered by prediction data, but we can still use an estimated
        // cycle time for stitching
         const float planning_cycle_time = 0.001f * OA_UPDATE_MS;
        
        // current planning start clock time
        const float start_timestamp = AP_HAL::millis();

        // run background task looking for best alternative destination
        OA_RetState res = OA_NOT_REQUIRED;
        OAPathPlannerUsed path_planner_used = OAPathPlannerUsed::None;

        // check goal and current location near obstacles
        _abandon_wp = false;
        _margin_min = FLT_MAX;
        const float check_margin = 2.0f * _margin_max;
        
        if (check_unreachable_from_object_database(avoidance_request2.current_loc, avoidance_request2.destination, check_margin)) {
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"目标航点靠近障碍物");
            _abandon_wp = true;
        }

        if (check_unreachable_from_home_fence(avoidance_request2.current_loc, avoidance_request2.destination, check_margin)) {
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"目标航点靠近家围栏");
            _abandon_wp = true;
        }

        if (check_unreachable_from_inclusion_and_exclusion_polygons(avoidance_request2.current_loc, avoidance_request2.destination, check_margin)) {
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"目标航点靠近多边形围栏");
            _abandon_wp = true;
        }

        if (check_unreachable_from_inclusion_and_exclusion_circles(avoidance_request2.current_loc, avoidance_request2.destination, check_margin)) {
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"目标航点靠近圆形围栏");
            _abandon_wp = true;
        }
        
        // 1. update vehicle state
        planning::VehicleState vehicle_state;
        Vector2f current_pos;
        if (!avoidance_request2.current_loc.get_vector_xy_from_origin_NE(current_pos)) {
            continue;
        }
        vehicle_state.x = current_pos.x / 100;
        vehicle_state.y = current_pos.y / 100;
        vehicle_state.z = 0;
        vehicle_state.timestamp = AP_HAL::millis();
        vehicle_state.heading = AP::ahrs().yaw;
        vehicle_state.kappa = 0.0f;
        vehicle_state.linear_velocity = AP::ahrs().groundspeed();
        vehicle_state.angular_velocity = AP::ahrs().get_yaw_rate_earth();
        vehicle_state.linear_acceleration = AP::ahrs().get_accel().xy().length();

        // 2. called ComputeStitchingTrajectory to get stiching_trajectory
        std::vector<planning::TrajectoryPoint> stitching_trajectory = 
        planning::TrajectoryStitcher::ComputeStitchingTrajectory(
            vehicle_state, start_timestamp, planning_cycle_time,
            FLAGS_trajectory_stitching_preserved_length, FLAGS_replan_by_offset,
            &avoidance_result.last_publishable_trajectory, FLAGS_enable_trajectory_stitcher);

        // 3. update planning start point
        auto planning_start_point_xy = stitching_trajectory.back();
        Location planning_start_point(Vector3f{planning_start_point_xy.path_point.x * 100, 
                                               planning_start_point_xy.path_point.y * 100, 0}, 
                                               Location::AltFrame::ABOVE_ORIGIN);
        
        // 4. define planning trajectory output
        planning::DiscretizedTrajectory planned_trajectory_pb;

        switch (_type) {
        case OA_PATHPLAN_DISABLED:
            continue;
        case OA_PATHPLAN_BENDYRULER: {
            if (_oabendyruler == nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            }
            _oabendyruler->set_config(_margin_max);
            _oabendyruler->set_margin_min(_margin_min);

            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, false)) {
                res = OA_SUCCESS;
            }
            path_planner_used = map_bendytype_to_pathplannerused(bendy_type);
            break;
        }

        case OA_PATHPLAN_DIJKSTRA: {
#if AP_FENCE_ENABLED
            if (_oadijkstra == nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            }
            _oadijkstra->set_fence_margin(_margin_max);
            const AP_OADijkstra::AP_OADijkstra_State dijkstra_state = _oadijkstra->update(avoidance_request2.current_loc, avoidance_request2.destination, origin_new, destination_new);
            switch (dijkstra_state) {
            case AP_OADijkstra::DIJKSTRA_STATE_NOT_REQUIRED:
                res = OA_NOT_REQUIRED;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_ERROR:
                res = OA_ERROR;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_SUCCESS:
                res = OA_SUCCESS;
                break;
            }
            path_planner_used = OAPathPlannerUsed::Dijkstras;
#endif
            break;
        }

        case OA_PATHPLAN_DJIKSTRA_BENDYRULER: {
            if ((_oabendyruler == nullptr) || _oadijkstra == nullptr) {
                 GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            } 
            _oabendyruler->set_config(_margin_max);
            _oabendyruler->set_margin_min(_margin_min);
            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, proximity_only)) {
                // detected a obstacle by vehicle's proximity sensor. Switch avoidance to BendyRuler till obstacle is out of the way
                proximity_only = false;
                res = OA_SUCCESS;
                path_planner_used = map_bendytype_to_pathplannerused(bendy_type);
                break;
            } else {
                // cleared all obstacles, trigger Dijkstra's to calculate path based on current deviated position  
#if AP_FENCE_ENABLED
                if (proximity_only == false) {
                    _oadijkstra->recalculate_path();
                }
#endif
                // only use proximity avoidance now for BendyRuler
                proximity_only = true;
            }
#if AP_FENCE_ENABLED
            _oadijkstra->set_fence_margin(_margin_max);
            const AP_OADijkstra::AP_OADijkstra_State dijkstra_state = _oadijkstra->update(avoidance_request2.current_loc, avoidance_request2.destination, origin_new, destination_new);
            switch (dijkstra_state) {
            case AP_OADijkstra::DIJKSTRA_STATE_NOT_REQUIRED:
                res = OA_NOT_REQUIRED;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_ERROR:
                res = OA_ERROR;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_SUCCESS:
                res = OA_SUCCESS;
                break;
            }
            path_planner_used = OAPathPlannerUsed::Dijkstras;
#endif
            break;
        }
        
        case OA_EM: {
            if ((_oabendyruler == nullptr) || (_speed_decider == nullptr)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            }
             path_planner_used = OAPathPlannerUsed::EM;
             
            _speed_decider->set_config(_margin_max);
            if (_speed_decider->update(avoidance_request2.current_loc, origin_new, destination_new, avoidance_request2.ground_speed_vec, desired_speed_new, planning_cycle_time)) {
                res = OA_SUCCESS;
            }
           
            break;
        }

        case OA_SLT: {
            if (_slt_planner == nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            }
            path_planner_used = OAPathPlannerUsed::SLT;

            _slt_planner->set_config(_margin_max);
            // use planning start point from sitching trajectory
            if (_slt_planner->update(planning_start_point, avoidance_request2.origin, avoidance_request2.destination,
                                    avoidance_request2.desired_speed, 
                                    planned_trajectory_pb, planning_cycle_time)) {
                res = OA_SUCCESS;
            }
            break;
        }

        } // switch

#if APM_BUILD_TYPE(APM_BUILD_Rover)
        // shoreline avoidance
        bool shoreline_detect = _oashoreline.update(avoidance_request2.current_loc,
                                                    avoidance_request2.origin, avoidance_request2.destination);
        // shallow avoidance
        bool shallow_detect   = _oashallow.update(avoidance_request2.current_loc,
                                                  avoidance_request2.origin, avoidance_request2.destination,
                                                  avoidance_request2.ground_speed_vec,
                                                  planning_cycle_time);
        if (shoreline_detect || shallow_detect || _abandon_wp) {
            res = OA_ABANDON;
        }
#endif


        {
            // give the main thread the avoidance result
            WITH_SEMAPHORE(_rsem);
            avoidance_result.destination = avoidance_request2.destination;
            avoidance_result.origin_new = (res == OA_SUCCESS) ? origin_new : avoidance_result.origin_new;
            avoidance_result.destination_new = (res == OA_SUCCESS) ? destination_new : avoidance_result.destination;
            avoidance_result.desired_speed_new = (res == OA_SUCCESS) ? desired_speed_new : avoidance_request2.desired_speed;
            avoidance_result.result_time_ms = AP_HAL::millis();
            avoidance_result.path_planner_used = path_planner_used;
            avoidance_result.ret_state = res;

            // 4. prepend stitching trajectory points
            if (planned_trajectory_pb.size() > 0) {
                avoidance_result.last_publishable_trajectory = std::move(planning::PublishableTrajectory(
                    start_timestamp, planned_trajectory_pb));
                
                // apend last stitching trajectory
                avoidance_result.last_publishable_trajectory.PrependTrajectoryPoints(
                    std::vector<planning::TrajectoryPoint>(stitching_trajectory.begin(),
                                                           stitching_trajectory.end() - 1));
            }
        }
    }
}

// returns the number of points in the current solution path
uint16_t AP_OAPathPlanner::get_path_count() const
{
    switch (_type) {
    case OA_PATHPLAN_DIJKSTRA:
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
        if (_oadijkstra == nullptr) {
            return 0;
        }
        return _oadijkstra->get_path_length();
        break;
        
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            return 0;
        }
        return _oabendyruler->get_path_length();
        break;

    case OA_SLT:
        if (_slt_planner == nullptr) {
            return 0;
        }
        return _slt_planner->get_path_length();
        break;

    default:
        return 0;
    }
}

// returns a location in the current solution path
bool AP_OAPathPlanner::get_path_point(uint8_t point_num, Location& Loc) const
{
    switch (_type) {
    case OA_PATHPLAN_DIJKSTRA:
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
        if (_oadijkstra == nullptr || !_oadijkstra->get_shortest_path_location(point_num, Loc)) {
            return false;
        }
        // note that altitude in Location is wrong, it is not calculated as part of the Dijkstra's solution.
        // Copter interpolates the altitude and rover does not. Possibly in the future altitude will be included in the solution
        return true;

    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr || !_oabendyruler->get_shortest_path_location(point_num, Loc)) {
            return false;
        }
        return true;
        break;
        
    case OA_SLT:
        if (_slt_planner == nullptr || !_slt_planner->get_shortest_path_location(point_num, Loc)) {
            return false;
        }
        return true;
        break;

    default:
        return false;
    }
}


bool AP_OAPathPlanner::check_unreachable_from_object_database(const Location &curr, const Location &end, const float margin)
{
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if(oaDb == nullptr || !oaDb->healthy()){
        return false;
    }

    // convert start and end to offsets (in cm) from EKF origin
    Vector3f cur_NEU,end_NEU;
    if (!curr.get_vector_from_origin_NEU(cur_NEU) || !end.get_vector_from_origin_NEU(end_NEU)) {
        return false;
    }
    
    // check each obstacle's distance from goal
    for(uint16_t i = 0; i < oaDb ->database_count(); i++){
        const AP_OADatabase::OA_DbItem& item = oaDb ->get_item(i);
        const Vector3f point_cm = item.pos * 100.0f;
        // margin is distance between goal point and obstacle edge
        const float m = (point_cm - end_NEU).length() * 0.01f - item.radius;
        const float n = (point_cm - cur_NEU).length() * 0.01f - item.radius;
        _margin_min = MIN(_margin_min, n);
        if(m < margin && n < margin){
            return true;
        }
    }

    return false;
}

bool AP_OAPathPlanner::check_unreachable_from_home_fence(const Location &cur, const Location &end, const float margin)
{
    // exit immediately if polygon fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if(fence == nullptr){
        return false;
    }
    if(fence->get_enabled_fences() & AC_FENCE_TYPE_CIRCLE){
        return false;
    }

    // calculate goal point's distance from home
    const Location &ahrs_home = AP::ahrs().get_home();
    const float end_dist_sq = ahrs_home.get_distance_NE(end).length_squared();
    const float cur_dist_sq = ahrs_home.get_distance_NE(cur).length_squared();

    // margin is fence radius minus the longer of start or end distance
    const float m = fence->get_radius() - sqrtf(end_dist_sq);
    const float n = fence->get_radius() - sqrtf(cur_dist_sq);
    _margin_min = MIN(_margin_min, n);
    if (m < margin && n < margin) {
        return true;
    }

    return true;
}

bool AP_OAPathPlanner::check_unreachable_from_inclusion_and_exclusion_polygons(const Location &cur, const Location &end, const float margin)
{
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // exclusion polygons enabled along with polygon fences
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return false;
    }

    // return immediately if no inclusion nor exclusion polygons
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();
    if ((num_inclusion_polygons == 0) && (num_exclusion_polygons == 0)) {
        return false;
    }

    // convert start and end to offsets from EKF origin
    Vector2f cur_NE, end_NE;
    if (!cur.get_vector_xy_from_origin_NE(cur_NE) || !end.get_vector_xy_from_origin_NE(end_NE)) {
        return false;
    }

    // iterate through inclusion polygons and calculate minimum distance
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);

        // if outside the fence margin is the closest distance but with negative sign
        const float end_sign = Polygon_outside(end_NE, boundary, num_points) ? -1.0f : 1.0f;
        const float cur_sign = Polygon_outside(cur_NE, boundary, num_points) ? -1.0f : 1.0f;

        // calculate min distance (in meters) from line to polygon
        const float m = end_sign * Polygon_closest_distance_point(boundary,num_points,end_NE) * 0.01f;
        const float n = cur_sign * Polygon_closest_distance_point(boundary,num_points,cur_NE) * 0.01f;
        _margin_min = MIN(_margin_min, n);
        if ( m < margin && n < margin) {
            return true;
        }
       
    }

    // iterate through exclusion polygons and calculate minimum margin
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);

        // if outside the fence margin is the closest distance but with negative sign
        const float end_sign = Polygon_outside(end_NE, boundary, num_points) ? 1.0f : -1.0f;
        const float cur_sign = Polygon_outside(cur_NE, boundary,num_points) ? 1.0f : -1.0f;

        // calculate min distance (in meters) from line to polygon
        const float m = end_sign * Polygon_closest_distance_point(boundary,num_points,end_NE) * 0.01f;
        const float n = cur_sign * Polygon_closest_distance_point(boundary,num_points,cur_NE) * 0.01f;
        _margin_min = MIN(_margin_min, n);
        if ( m < margin && n < margin) {
            return true;
        }
    }

     return false;
}

bool AP_OAPathPlanner::check_unreachable_from_inclusion_and_exclusion_circles(const Location &cur, const Location &end, const float margin) 
{
     // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // inclusion/exclusion circles enabled along with polygon fences
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return false;
    }

    // return immediately if no inclusion nor exclusion circles
    const uint8_t num_inclusion_circles = fence->polyfence().get_inclusion_circle_count();
    const uint8_t num_exclusion_circles = fence->polyfence().get_exclusion_circle_count();
    if ((num_inclusion_circles == 0) && (num_exclusion_circles == 0)) {
        return false;
    }

    // convert start and end to offsets from EKF origin
    Vector2f cur_NE, end_NE;
    if (!cur.get_vector_xy_from_origin_NE(cur_NE) || !end.get_vector_xy_from_origin_NE(end_NE)) {
        return false;
    }

    // iterate through inclusion circles and calculate minimum margin
    for (uint8_t i = 0; i < num_inclusion_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_cm, radius)) {

            const float end_dist_sq = (end_NE - center_pos_cm).length_squared();
            const float cur_dist_sq = (cur_NE - center_pos_cm).length_squared();

            // margin is fence radius minus the longer of start or end distance
            const float m = radius - sqrtf(end_dist_sq) * 0.01f;
            const float n = radius - sqrtf(cur_dist_sq) * 0.01f;
            _margin_min = MIN(_margin_min, n);
            // update margin with lowest value so far
            if (m < margin && n < margin) {
                return true;
            }
        }
    }

    // iterate through exclusion circles and calculate minimum margin
    for (uint8_t i = 0; i < num_exclusion_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {

            const float end_dist_sq = (end_NE - center_pos_cm).length_squared();
            const float cur_dist_sq = (cur_NE - center_pos_cm).length_squared();

            // margin is fence radius minus the longer of start or end distance
            const float m = sqrtf(end_dist_sq) * 0.01f - radius;
            const float n = sqrtf(cur_dist_sq) * 0.01f - radius;
            _margin_min = MIN(_margin_min, n);
            // update margin with lowest value so far
            if (m < margin && n < margin) {
                return true;
            }
        }
    }

    return false;
}

// singleton instance
AP_OAPathPlanner *AP_OAPathPlanner::_singleton;

namespace AP {

AP_OAPathPlanner *ap_oapathplanner()
{
    return AP_OAPathPlanner::get_singleton();
}

}
