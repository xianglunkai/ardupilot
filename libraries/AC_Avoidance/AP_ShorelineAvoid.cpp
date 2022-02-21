#include "AP_ShorelineAvoid.h"
#include "AP_OADatabase.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>


// singleton instance
AP_ShorelineAvoid *AP_ShorelineAvoid::_singleton;
namespace AP {
    AP_ShorelineAvoid *shoreline_avoid()
    {
        return AP_ShorelineAvoid::get_singleton();
    }
}

#define SHORE_SONAR_AVOID_ENABLE 1
const float OA_DETECT_BEARING_INC_XY   = 1;      // deg

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_ShorelineAvoid::var_info[] = {

    // @Param: LIK_DST
    // @DisplayName: Shoreline Avoidance link distance 
    // @Description: Longest distance between adjacent points of shoreline
    // @Units: m
    // @Range: 0.001 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LIK_DST", 1, AP_ShorelineAvoid, _shoreline_link_dist, 3),

    // @Param: LEN_MIN
    // @DisplayName: Shoreline Avoidance shoreline length minimum
    // @Description: Shortest shoreline length
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LEN_MIN", 2, AP_ShorelineAvoid, _shoreline_min_length, 30),

    // @Param: SAFE_DST
    // @DisplayName: Shoreline Avoidance safe distance minimum
    // @Description: Safe distance from shoreline
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_DST", 3, AP_ShorelineAvoid, _shoreline_safe_dist, 20),

    // @Param: RADIUS_MIN
    // @DisplayName: Shoreline Avoidance item radius minmum
    // @Description: Minimum radius of shoreline point cloud
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS_MIN", 4, AP_ShorelineAvoid, _min_radius, 3),

    // @Param: SCAN_ANG
    // @DisplayName: Shoreline Avoidance scan angle maximum
    // @Description: Set the maximum scan angle of shoreline
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SCAN_ANG", 5, AP_ShorelineAvoid, _shoreline_scan_max_angle, 180),

    // @Param: DPT_MIN
    // @DisplayName: Water depth minimum
    // @Description: Shallowest water depth for avoidance
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DPT_MIN", 6, AP_ShorelineAvoid, _shollow_min_depth, 0.8),

    // @Param: DELAY_DST
    // @DisplayName: Delay many meters for shallowest water depth
    // @Description: Delay many meters for ensure shallow water depth
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DELAY_DST", 7, AP_ShorelineAvoid, _shollow_move_dist, 5.0),

    AP_GROUPEND
    };

AP_ShorelineAvoid::AP_ShorelineAvoid()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OADetect must be singleton");
    }
    _singleton = this;
}


// true if update has been called recently
bool AP_ShorelineAvoid::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < 1000);
}


// Run background task to find shoreline and update avoidance result
// Return true if shoreline detected and can't reach destination 
// Return false if destination could arrivalable temporarily.
bool AP_ShorelineAvoid::update(const Location &current_loc,const Location& origin,const Location& destination)
{
    // convert location with lat-lng to offsets from ekf orgin
    Vector2f current_ne,origin_ne,destination_ne;
    if (!current_loc.get_vector_xy_from_origin_NE(current_ne) ||
        !origin.get_vector_xy_from_origin_NE(origin_ne) ||
        !destination.get_vector_xy_from_origin_NE(destination_ne)) {
        // OA is not required
        return false;
    }

    // vehicle already reach destination,return true
    float mission_distance = origin.get_distance(destination);
    if (current_ne == destination_ne || mission_distance < 1e-3f) {
        return true;
    }

    // translate units cm to m
    current_ne *= 0.01f;
    origin_ne *= 0.01f;
    destination_ne *= 0.01f;

   // check result's destination matches our request ,and update current mission 
    const bool destination_matches = ((destination.lat == _destination_prev.lat) && 
                                     (destination.lng  == _destination_prev.lng));
   if(!destination_matches){
       _origin_prev = origin;
       _destination_prev = destination;
       _intersect_point.zero();

 #if SHORE_SONAR_AVOID_ENABLE == 1
       _sonar_avoid_loc = current_ne;
       _close_to_shoreline = false;
#endif
       _mission_lock = true;
   }

   _last_update_ms = AP_HAL::millis();
   if(!is_active()){
       _last_avoid_flag = false;
   }

    // special consideration if two waypoints close 
    const float wp_distance = (destination_ne - origin_ne).length();
    if(wp_distance <= _shoreline_safe_dist && _last_avoid_flag == true){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "contious shoreline avoidance");
        _last_avoid_flag = false;
        return true;
    }

    // return true if exist shoreline
    if(!update_shoreline(current_ne,destination_ne)){ return false;}
    
    // return true if destination with back of shoreline
    const bool res = update_avoidance(current_ne,origin_ne,destination_ne);
    if(res == true){_last_avoid_flag = true;}

    return res;
}

// Return true if shoreline detected and can not reach destination 
bool AP_ShorelineAvoid::update_avoidance(const Vector2f& current_ne,const Vector2f& origin_ne,const Vector2f& destination_ne)
{
    // check mission lock
    if(_mission_lock == false){
        return false;
    }

#if SHORE_SONAR_AVOID_ENABLE == 1
    // sonar low water depth avoidance
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder != nullptr && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
        const bool sensor_healthy = (rangefinder->status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good);
        const float water_depth_m = rangefinder->distance_orient(ROTATION_PITCH_270,true);

        if(sensor_healthy && water_depth_m <=_shollow_min_depth && _close_to_shoreline){

            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "shallow water alarm:%f",water_depth_m);
            
            const float move_distance = (current_ne - _sonar_avoid_loc).length();
            if(move_distance > _shollow_move_dist) {
                _mission_lock = false;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "shallow water avoidance");
                return true;
            }
        }else{
            _sonar_avoid_loc  = current_ne;
        }
    }
#endif


    // if vehicle already close intersection point ,return true
   if(!_intersect_point.is_zero()){
        float distance_to_intersection = (_intersect_point - current_ne).length();
        if(distance_to_intersection < _shoreline_safe_dist){
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "shoreline avoidance");
            _mission_lock = false;
            return true;
        }
   }

    // check shorelines empty
    Vector2f pb_sl_cross,ab_sl_cross,closest_psl,closest_bsl;
    if(_shoreline_set.empty()){return false;}

    // check every shoreline
    for(auto item:_shoreline_set){
        auto sl = item.second;
        float shoreline_len = calc_shoreline_len(sl);
        if( shoreline_len < _shoreline_min_length){
            continue;
        }

        bool pb_sl = Vector2f::intersection_between_segment_and_lines(current_ne,destination_ne,sl,pb_sl_cross);
        bool ab_sl = Vector2f::intersection_between_segment_and_lines(origin_ne,destination_ne,sl,ab_sl_cross);
        
        float distance_psl = Vector2f::distance_between_point_and_lines(current_ne,sl,closest_psl);
        float distance_bsl  = Vector2f::distance_between_point_and_lines(destination_ne,sl,closest_bsl);

        // update intersection with current line
        if(pb_sl == true && ab_sl == true && current_ne != origin_ne){         
            _intersect_point    = ab_sl_cross;
        }

        // store close to shoreline state
#if SHORE_SONAR_AVOID_ENABLE == 1
        if(pb_sl == true){  _close_to_shoreline = true; }
#endif

        // current line is back of shoreline,return true if project distance close to total distance
        if(pb_sl == true && ab_sl == false){
            const float total_dist = (destination_ne - origin_ne).length();
            const float proj_dist  = (current_ne - origin_ne) * (destination_ne - origin_ne) / total_dist;
            const float distance_to_closestp2 = (closest_bsl - current_ne).length();
            if((total_dist - proj_dist) <= _shoreline_safe_dist || (distance_to_closestp2 <= _shoreline_safe_dist)){
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "mission edging shoreline");
                _mission_lock = false;
                return true;
            }
        }
        
        // destination near shoreline
        if(distance_psl < _shoreline_safe_dist && 
           distance_bsl < _shoreline_safe_dist &&
           (closest_psl - closest_bsl).length() < _shoreline_safe_dist){
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "goal near shoreline");
            _mission_lock = false;
            return true;
        }
    }

   return false;
}

// return true if exist shoreline
bool AP_ShorelineAvoid::update_shoreline(const Vector2f& current_ne,const Vector2f& destination_ne)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        // Exit immediately if db is empty
        return false;
    }

    const float bearing_to_destination = (destination_ne - current_ne).angle();
    float boundary_th_min = FLT_MAX;
    float boundary_th_max = -FLT_MAX;

    // Collect obstacles from DB
    _obstacle_set.clear();
    for(uint16_t i = 0; i< oaDb->database_count(); i++){
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const float radius = MAX(_min_radius,item.radius);
        const Vector2f position(item.pos.xy());
        const float bearing_to_obstacle = (position - current_ne).angle();
        const float test_bearing = wrap_PI(bearing_to_obstacle - bearing_to_destination);
        if(std::abs(test_bearing) < radians(0.5f * _shoreline_scan_max_angle)){
             _obstacle_set.emplace_back(position,radius);
             boundary_th_min = std::min(boundary_th_min,test_bearing);
             boundary_th_max = std::max(boundary_th_max,test_bearing);
        }
    }

    // extract boundary from obstacles
    const int32_t resolution = degrees(boundary_th_max - boundary_th_min)/OA_DETECT_BEARING_INC_XY;
    if(_obstacle_set.empty() || resolution <= 3 || resolution > _shoreline_scan_max_angle/OA_DETECT_BEARING_INC_XY)
    {return false;}

    std::vector<uint32_t> indexs(resolution,INT32_MAX);
    std::vector<float> rou(resolution,FLT_MAX);
    for(uint16_t i = 0; i< _obstacle_set.size(); i++){
        const float bearing =  wrap_PI((_obstacle_set[i].xy() - current_ne).angle() - bearing_to_destination) ;
        const float rel_bearing = bearing - boundary_th_min;
        const float rel_dist    = (_obstacle_set[i].xy() - current_ne).length();

        int32_t id = degrees(rel_bearing)/OA_DETECT_BEARING_INC_XY;
        if(id < 0 || id >= resolution){
        //  GCS_SEND_TEXT(MAV_SEVERITY_INFO, "id:%d,resolution:%d",id,resolution);
         //   return false; 
         continue;
        }

        if(rel_dist < rou[id]){
            rou[id]    = rel_dist;
            indexs[id] = i;
        }
    }

    // update boundary
    _boundary_set.clear();
    for(int32_t i = 0; i< resolution; i++){
        if(indexs[i] < _obstacle_set.size()){
            _boundary_set.emplace_back(_obstacle_set.at(indexs[i]));
        }
    }
    if(_boundary_set.empty()){
        return false;
    }
    
    // update shoreline set
    _shoreline_set.clear();
    std::pair<uint32_t,std::vector<Vector2f>> shoreline_item;
    uint32_t seq = 1;
    shoreline_item.first = seq;
    shoreline_item.second.emplace_back(_boundary_set[0].xy());
    for(uint32_t i = 1; i< _boundary_set.size(); i++){
        auto point_cur = _boundary_set[i].xy();
        auto point_prev = _boundary_set[i - 1].xy();
        const float point_cur_radius = _boundary_set[i].z;
        const float point_prev_radius = _boundary_set[i-1].z;

        const float clozet_distance = (point_cur - point_prev).length() - point_cur_radius - point_prev_radius;
        if(clozet_distance <= _shoreline_link_dist){
            shoreline_item.second.emplace_back(point_cur);
        }else{
            if(!shoreline_item.second.empty()){
                _shoreline_set.emplace_back(shoreline_item);
            }
            shoreline_item.second.clear();
            shoreline_item.first = ++seq;
            shoreline_item.second.emplace_back(point_cur);
        }
    }

    // remember the last shoreline item
    if(!shoreline_item.second.empty()){
        _shoreline_set.emplace_back(shoreline_item);
    }

    if(_shoreline_set.empty()){
        return false;
    }
    return true;

}


float AP_ShorelineAvoid::calc_shoreline_len(const std::vector<Vector2f> &lines) const
{
    if(lines.size() == 0 || lines.size() == 1){
        return 0;
    }

    float len = 0;
    for(uint16_t i = 0; i < lines.size() - 1; i++)
    {
      len += (lines.at(i+1) - lines.at(i)).length();
    }

    return len;
}

