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
#include "AP_OABendyRuler.h"
#include "AP_OADijkstra.h"

extern const AP_HAL::HAL &hal;

// parameter defaults
const float OA_MARGIN_MAX_DEFAULT = 5;
const int16_t OA_OPTIONS_DEFAULT = 1;

const int16_t OA_UPDATE_MS  = 1000;      // path planning updates run at 1hz
const int16_t OA_TIMEOUT_MS = 3000;     // results over 3 seconds old are ignored

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
    // @Path: AP_DPPlanner.cpp
    AP_SUBGROUPPTR(_dp_planner, "DP_", 10, AP_OAPathPlanner, AP_DPPlanner),

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
        if (_oadijkstra == nullptr) {
            _oadijkstra = new AP_OADijkstra(_options);
        }
        break;
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
        if (_oadijkstra == nullptr) {
            _oadijkstra = new AP_OADijkstra(_options);
        }
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
        if (_oabendyruler == nullptr) {
            _oabendyruler = new AP_OABendyRuler();
            AP_Param::load_object_from_eeprom(_oabendyruler, AP_OABendyRuler::var_info);
        }
        break;
    case OA_DP:
        if (_dp_planner == nullptr) {
            _dp_planner = new AP_DPPlanner();
            AP_Param::load_object_from_eeprom(_dp_planner, AP_DPPlanner::var_info);
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
    case OA_DP:
        if (_dp_planner == nullptr) {
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
        return avoidance_result.ret_state;
    }

    // if timeout then path planner is taking too long to respond
    if (timed_out) {
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
                continue;
            }

            // copy request to avoid conflict with main thread
            avoidance_request2 = avoidance_request;

            // store passed in origin and destination so we can return it if object avoidance is not required
            origin_new = avoidance_request.origin;
            destination_new = avoidance_request.destination;
            desired_speed_new = avoidance_request.desired_speed;
        }

        // run background task looking for best alternative destination
        OA_RetState res = OA_NOT_REQUIRED;
        OAPathPlannerUsed path_planner_used = OAPathPlannerUsed::None;
        switch (_type) {
        case OA_PATHPLAN_DISABLED:
            continue;
        case OA_PATHPLAN_BENDYRULER: {
            if (_oabendyruler == nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            }
            _oabendyruler->set_config(_margin_max);

            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, false)) {
                res = OA_SUCCESS;
            }else if(_oabendyruler->abandon_waypoint()){
                res = OA_ABANDON;
            }
            path_planner_used = map_bendytype_to_pathplannerused(bendy_type);
            break;
        }

        case OA_PATHPLAN_DIJKSTRA: {
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
            break;
        }

        case OA_PATHPLAN_DJIKSTRA_BENDYRULER: {
            if ((_oabendyruler == nullptr) || _oadijkstra == nullptr) {
                 GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            } 
            _oabendyruler->set_config(_margin_max);
            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, proximity_only)) {
                // detected a obstacle by vehicle's proximity sensor. Switch avoidance to BendyRuler till obstacle is out of the way
                proximity_only = false;
                res = OA_SUCCESS;
                path_planner_used = map_bendytype_to_pathplannerused(bendy_type);
                break;
            } else {
                // cleared all obstacles, trigger Dijkstra's to calculate path based on current deviated position  
                if (proximity_only == false) {
                    _oadijkstra->recalculate_path();
                }
                // only use proximity avoidance now for BendyRuler
                proximity_only = true;
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
            break;
        }
        
        case OA_EM: {
            if ((_oabendyruler == nullptr) || (_speed_decider == nullptr)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            }
             path_planner_used = OAPathPlannerUsed::EM;
             
             _oabendyruler->set_config(_margin_max);
            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, false)) {
                res = OA_SUCCESS;
            }else if(_oabendyruler->abandon_waypoint()){
                res = OA_ABANDON;
                break;
            }

            _speed_decider->set_config(_margin_max);
            if (_speed_decider->update(avoidance_request2.current_loc, avoidance_request2.origin, avoidance_request2.destination, avoidance_request2.ground_speed_vec, desired_speed_new, 0.001f * OA_UPDATE_MS)) {
                res = OA_SUCCESS;
            }
           
            break;
        }

        case OA_DP: {
            if (_dp_planner == nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"OAPathPlanner need reboot");
                continue;
            }
            path_planner_used = OAPathPlannerUsed::DP;

            _dp_planner->set_config(_margin_max);
            if (_dp_planner->update(avoidance_request2.current_loc, avoidance_request2.origin, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, desired_speed_new, false)) {
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
                                                  0.001f * OA_UPDATE_MS);
        if (shoreline_detect || shallow_detect) {
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
        }
    }
}

// singleton instance
AP_OAPathPlanner *AP_OAPathPlanner::_singleton;

namespace AP {

AP_OAPathPlanner *ap_oapathplanner()
{
    return AP_OAPathPlanner::get_singleton();
}

}
