#include "AP_RoutePlanner.h"
#include "AP_OADatabase.h"

#include <algorithm>
#include <list>
#include <utility>
#include <memory>
#include <functional>

#include <AP_Planning/math_utils.h>

#include <AC_Fence/AC_Fence.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>


const AP_Param::GroupInfo AP_RoutePlanner::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_RoutePlanner, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ROB_SIZE
    // @Description: vehicle radius
    // @Units: m
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("ROB_SIZE", 1, AP_RoutePlanner, _vehicle_radius, 0.5f),


    // @Param: REF_LB
    // @Description: reference center line left range
    // @Units: m
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("REF_LB", 2, AP_RoutePlanner, _reference_left_bound, 20.0f),

    // @Param: REF_RB
    // @Description: reference center line right range
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("REF_RB", 3, AP_RoutePlanner, _reference_right_bound, 20.0f),

    // @Param: REF_RES
    // @Description: reference center line resolution
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("REF_RES", 4, AP_RoutePlanner, _path_resolution, 0.5f),

    // @Param: SPL_NUM
    // @Description: sample points number  of each level
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("SPL_NUM", 4, AP_RoutePlanner, _sample_points_num_each_level, 10),

    // @Param: STL_MIN
    // @Description: sample minimal lookahead length
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("STL_MIN", 5, AP_RoutePlanner, _step_length_min, 5),

    // @Param: STL_MAX
    // @Description: sample maximual lookahead length
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("STL_MAX", 6, AP_RoutePlanner, _step_length_max, 50),

    // @Param: TIME_DLT
    // @Description: delta time for evaluation dynamical collision
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("TIME_DLT", 7, AP_RoutePlanner, _eval_time_interval, 0.1),

    // @Param: WGT_L
    // @Description: path l cost
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("WGT_L", 8, AP_RoutePlanner, _path_l_cost, 1),  

    // @Param: DL_WGT
    // @Description: path dl cost
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("WGT_DL", 9, AP_RoutePlanner, _path_dl_cost, 1),  

    // @Param: DDL_WGT
    // @Description: path ddl cost
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("WGT_DDL", 10, AP_RoutePlanner, _path_ddl_cost, 1),  

    // @Param: OBS_WGT
    // @Description: obstacle collision cost
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("WGT_OBS", 11, AP_RoutePlanner, _obstacle_collision_cost, 10), 

  AP_GROUPEND


  
};

 AP_RoutePlanner::AP_RoutePlanner()
 {
    AP_Param::setup_object_defaults(this, var_info);
 }

// run background task to find best cruise speed and update avoidance results
// returns false if obstacle avoidance is not required
bool AP_RoutePlanner::update(const Location &current_loc, 
            const Location& origin, 
            const Location& destination, 
            const planning::SpeedData &speed_data,
            const float planning_cycle_time,
            planning::PathData *const path_data)
{
    // 1. convert location with lat-lng to offsets from ekf orgin
    Vector2f current_ne, origin_ne, destination_ne;
    if (!current_loc.get_vector_xy_from_origin_NE(current_ne) ||
        !origin.get_vector_xy_from_origin_NE(origin_ne) ||
        !destination.get_vector_xy_from_origin_NE(destination_ne)) {
        // OA is not required
        return false;
    }
    // translate units from cm to m
    current_ne *= 0.01f;
    origin_ne *= 0.01f;
    destination_ne *= 0.01f;

    // 2. create reference line
    const bool reference_matches = (_start == origin_ne) && (_end == destination_ne);
    if (!reference_matches) {
        _start = origin_ne;
        _end = destination_ne;
        // calculate bearing and distance to final destination
        const float reference_bearing = origin.get_bearing_to(destination) * 0.01f;
        const float reference_length = origin.get_distance(destination);
        const Vector2f unit_ref = (_end - _start).normalized();
        planning::Trajectory traj;
        float s = 0;
        while ( s < reference_length + _path_resolution) {
            const Vector2f pt = _start + unit_ref * s;
            planning::TrajPoint tp;
            tp.s = s;
            tp.x = pt.x;
            tp.y = pt.y;
            tp.theta = radians(reference_bearing);
            tp.kappa = 0;
            tp.velocity = 0;
            tp.left_bound = _reference_left_bound;
            tp.right_bound = _reference_right_bound;
            traj.emplace_back(tp);
            s += _path_resolution;
        }
        set_reference(planning::DiscretizedTraj(traj));
    }

    // 3. get enviroment obstacles
    update_obstacles();

    if (_static_obstacles.empty() && _dynamic_obstacles.empty()) {
        return false;
    }

    // 4. get planning start SL point from reference line
    _heuristic_speed_data = std::move(speed_data);
    _init_point_sl = _reference.get_projection({current_ne.x, current_ne.y});

    float groundSpeed = AP::ahrs().groundspeed();
    Vector2f groundspeed_vector = AP::ahrs().groundspeed_vector();
    if (groundSpeed < 0.1f) {
      // use a small ground speed vector in the right direction,
      // allowing us to use the compass heading at zero GPS velocity
      groundSpeed = 0.1f;
      groundspeed_vector = Vector2f(cosf(AP::ahrs().yaw), sinf(AP::ahrs().yaw)) * groundSpeed;
    }
    auto projected_line_unit = (_end - _start).normalized();
    _init_v = MAX(groundspeed_vector * projected_line_unit, 0.0f);

    // 5. dp planning
    std::vector<DpRoadGraphNode> min_cost_path;
    if (!generate_min_cost_path(&min_cost_path)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AP_RoutePlanner: fail to generate graph!");
        return false;
    }

    // 6. interpolation and return path
    std::vector<planning::PathPoint> path_profile;
    float accumulated_s = min_cost_path.front().sl_point.x();
    for (size_t i = 1; i < min_cost_path.size(); i++) {
        const auto &prev_node = min_cost_path[i-1];
        const auto &cur_node = min_cost_path[i];

        const float path_length = cur_node.sl_point.x() - prev_node.sl_point.x();
        float current_s = 0;
        const auto &curve = cur_node.min_cost_curve;
        while (current_s + _path_resolution / 2.0  < path_length) {
            planning::PathPoint pt;
           
            pt.s = accumulated_s + current_s;
            // get l in SL 
            const float l = curve.Evaluate(0, current_s);
            // [s,l] -> [x, y]
            auto xy = _reference.get_cartesian(pt.s, l);
            // set x,y
            pt.x = xy.x();
            pt.y = xy.y();
            current_s += _path_resolution;
            path_profile.emplace_back(pt);
        }

        if (i == min_cost_path.size() - 1) {
            accumulated_s += current_s;
        } else {
            accumulated_s += path_length;
        }
    }

    *path_data = planning::PathData(path_profile);

    return true;
}

 bool AP_RoutePlanner::generate_min_cost_path(std::vector<DpRoadGraphNode> *min_cost_path)
 {
    if (min_cost_path == nullptr) {
        return false;
    }

    // 1. sample path waypoints
    std::vector<std::vector<planning::Vec2d>> path_waypoints;
    if (!SamplePathWaypoints(&path_waypoints) || path_waypoints.size() < 1) {
        return false;
    }

    // 2. find the first node for searching
    std::list<std::list<DpRoadGraphNode>> graph_nodes;
    const auto &first_row = path_waypoints.front();
    size_t nearest_i = 0;
    for (size_t i  = 1; i < first_row.size(); ++i) {
        if (std::abs(first_row[i].y() - _init_point_sl.y()) < 
            std::abs(first_row[nearest_i].y() - _init_point_sl.y())) {
                nearest_i = i;
            }
    }
    
    // add a list
    graph_nodes.emplace_back();
    // add a item into a list
    graph_nodes.back().emplace_back(first_row[nearest_i], nullptr, ComparableCost());

    auto &front = graph_nodes.front().front();
    size_t total_level = path_waypoints.size();

    // 3. outer loop: level, inner loop: row
    for (size_t level = 1; level < total_level; ++level) {
        // record and store current level nodes
        const auto &prev_dp_nodes = graph_nodes.back();
        const auto &level_points = path_waypoints[level];

        // add a list
        graph_nodes.emplace_back();

        for (size_t i = 0; i < level_points.size(); ++i) {
            // [level, i]
            const auto &cur_point = level_points[i];
            graph_nodes.back().emplace_back(cur_point, nullptr);
            
            for (const auto &prev_dp_node : prev_dp_nodes) {
                const auto &prev_sl_point = prev_dp_node.sl_point;
                
                float init_dl = 0.0;
                float init_ddl = 0.0;
                if (level == 1) {
                    init_dl = 0;
                    init_ddl = 0;
                }

                planning::QuinticPolynomialCurve1d curve(prev_sl_point.y(), init_dl, init_ddl,
                                                         cur_point.y(), 0.0, 0.0,
                                                         cur_point.x() - prev_sl_point.x());
                // if dl/ds is too larger, continue!
                if (!is_valid_curve(curve)) {
                    continue;
                }

                // compute cost
                const auto cost = Calculate(curve, prev_sl_point.x(), cur_point.x(), level, total_level) + prev_dp_node.min_cost;
                
                // update node cost
                graph_nodes.back().back().UpdateCost(&prev_dp_node, curve, cost);
            }
        }
    }

    // 4. find the minimal cost node in the last level
    DpRoadGraphNode fake_head;
    for (const auto &cur_dp_node : graph_nodes.back()) {
        fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve, cur_dp_node.min_cost);
    }

    // 5. backup
    const auto *min_cost_node = &fake_head;
    while (min_cost_node->min_cost_prev_node) {
        min_cost_node = min_cost_node->min_cost_prev_node;
        min_cost_path->push_back(*min_cost_node);
    }
    if (min_cost_node != &graph_nodes.front().front()) {
        return false;
    }

    std::reverse(min_cost_path->begin(), min_cost_path->end());

    return true;
 }

// TODO(All): optimize obstacle cost calculation time
ComparableCost AP_RoutePlanner::Calculate(const planning::QuinticPolynomialCurve1d &curve,
                                         const float start_s,
                                         const float end_s,
                                         const uint32_t curr_level,
                                         const uint32_t total_level) {
  ComparableCost total_cost;
  // path cost
  total_cost +=
      CalculatePathCost(curve, start_s, end_s, curr_level, total_level);

  // static obstacle cost
  total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);

  // dynamic obstacle cost
  total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
  return total_cost;
}


ComparableCost AP_RoutePlanner::CalculatePathCost(const planning::QuinticPolynomialCurve1d &curve,
                                                  const float start_s, const float end_s,
                                                  const uint32_t curr_level,
                                                  const uint32_t total_level)
{
    ComparableCost cost;
    float path_cost = 0.0;
    std::function<float(const float)> quasi_softmax = [this](const float x) {
        const float l0 = 1.0f;
        const float b =  2.0f;
        const float k =  1.0f;
        return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
    };

    for (float curve_s = 0.0f; curve_s < (end_s - start_s); curve_s += _path_resolution) {
    
        // _path_l_cost * l * l  * quasi_softmax(l)
        const float l = curve.Evaluate(0, curve_s);
        path_cost += l * l * _path_l_cost * quasi_softmax(std::abs(l));

        // _path_dl_cost * dl * dl 
        const float dl = std::abs(curve.Evaluate(1, curve_s));
        path_cost += dl * dl * _path_dl_cost;

        if (IsOffRoad(curve_s + start_s, l, dl)) {
            cost.cost_items[ComparableCost::OUT_OF_BOUNDARY] = true;
        }

        // _path_ddl_cost * ddl * ddl 
        const float ddl = std::abs(curve.Evaluate(2, curve_s));
        path_cost += ddl  * ddl * _path_ddl_cost;
    }

    path_cost *= _path_resolution;

    cost.smoothness_cost = path_cost;
    return cost;

}

bool AP_RoutePlanner::IsOffRoad(const float ref_s, const float l, const float dl)
{
    const float buffer = 0.1f;
    const float left_bound = std::max(_init_point_sl.y() + _vehicle_radius.get() + buffer, _reference_left_bound.get());
    const float right_bound = std::min(_init_point_sl.y() - _vehicle_radius.get() - buffer, -_reference_right_bound.get()); 

    if (l + _vehicle_radius + buffer / 2.0f > left_bound ||
        l - _vehicle_radius - buffer / 2.0f < right_bound) {
            return true;
        } 
    
    return false;
}


ComparableCost AP_RoutePlanner::CalculateStaticObstacleCost(const planning::QuinticPolynomialCurve1d &curve, const float start_s, const float end_s)
{
    ComparableCost obstacle_cost;
    if(_static_obstacles.empty()) {
        return obstacle_cost;
    }

    for (float curr_s = start_s; curr_s <= end_s; curr_s += _path_resolution) {
        const float curr_l = curve.Evaluate(0, curr_s - start_s);
        for (const auto &obs : _static_obstacles) {
            obstacle_cost += GetCostFromObsSL(curr_s, curr_l, obs);
        }
    }

    obstacle_cost.safety_cost *= _path_resolution;
    return obstacle_cost;
}

ComparableCost AP_RoutePlanner::GetCostFromObsSL(const float adc_s, const float adc_l,
                                  const object & obstacle)
{
    ComparableCost obstacle_cost;
    const float adc_front_s = adc_s + _vehicle_radius;
    const float adc_end_s = adc_s - _vehicle_radius;
    const float adc_left_l = adc_l + _vehicle_radius;
    const float adc_right_l = adc_l - _vehicle_radius;

    const float FLAGS_lateral_ignore_buffer = 20.0f;
    const auto obstacle_sl   = _reference.get_projection({obstacle.pos.x, obstacle.pos.y}); 
    const float obs_start_s  = obstacle_sl.x() - obstacle.radius;
    const float obs_end_s    = obstacle_sl.x() + obstacle.radius;
    const float obs_start_l  = obstacle_sl.y() - obstacle.radius;
    const float obs_end_l  = obstacle_sl.y() + obstacle.radius;


    if (adc_left_l + FLAGS_lateral_ignore_buffer < obs_start_l ||
        adc_right_l -  FLAGS_lateral_ignore_buffer > obs_end_l) {
            return obstacle_cost;
        }
    
    bool no_overlap = ((adc_front_s < obs_start_s || 
                        adc_end_s > obs_end_s) ||  // longitudinal
                        (adc_left_l + 0.1 < obs_start_l ||
                        adc_right_l - 0.1 > obs_end_l));  // lateral 
    
    if (!no_overlap) {
        obstacle_cost.cost_items[ComparableCost::HAS_COLLISION] = true; 
    }

    // if obstacle is behind ADC, ignore its cost contribution
    if (adc_front_s > obs_end_s) {
        return obstacle_cost;
    }

    const float delta_l = std::max(adc_right_l - obs_end_l, obs_start_l - adc_left_l);

    if (delta_l < 2 * _safe_margin) {
        // using sigmoid
        obstacle_cost.safety_cost += _obstacle_collision_cost * planning::Sigmoid(_safe_margin - delta_l);
    }

    return obstacle_cost;
    
}
      
ComparableCost AP_RoutePlanner::CalculateDynamicObstacleCost(const planning::QuinticPolynomialCurve1d &curve, const float start_s,
                                                             const float end_s)
{
    ComparableCost obstacle_cost;
    if (_dynamic_obstacles.empty()) {
        return obstacle_cost;
    }

    const float total_time = std::min(_heuristic_speed_data.TotalTime(), 3.0f);
    const float num_of_time_stamps = static_cast<uint32_t>(std::floor(total_time / _eval_time_interval));

    float time_stamp = 0.0f;
    for (size_t index = 0; index < num_of_time_stamps; ++index, time_stamp += _eval_time_interval) {
        planning::SpeedPoint speed_point;
        _heuristic_speed_data.EvaluateByTime(time_stamp, &speed_point);

        const float ref_s = speed_point.s() + _init_point_sl.x();

        if (ref_s < start_s) {
            continue;
        }

        if (ref_s > end_s) {
            break;
        }

        const float s = ref_s - start_s;
        const float l = curve.Evaluate(0, s);
        const float dl = curve.Evaluate(1, s);

        for (const auto &obs : _dynamic_obstacles) {
            obstacle_cost += GetCostBetweenObsBoxes(ref_s, l, obs, time_stamp);
        }
    }

    static constexpr float kDynamicObsWeight = 1e-6;
    obstacle_cost.safety_cost *= (_eval_time_interval * kDynamicObsWeight);
    return obstacle_cost;
}

ComparableCost AP_RoutePlanner::GetCostBetweenObsBoxes(const float adc_s, const float adc_l, const object &obstacle, const float time_stamp)
{
    ComparableCost obstacle_cost;

    planning::Vec2d ego_pos = _reference.get_cartesian(adc_s, adc_l);
    planning::Vec2d obs_pos{obstacle.pos.x + obstacle.vel.x * time_stamp, obstacle.pos.y + obstacle.vel.y * time_stamp};
    const float distance =  obs_pos.DistanceTo(ego_pos) - obstacle.radius - _vehicle_radius;

    if (distance  > 10.0f * _safe_margin) {
        return obstacle_cost;
    }

    obstacle_cost.safety_cost += _obstacle_collision_cost * planning::Sigmoid(_safe_margin - distance);

    obstacle_cost.safety_cost += 20.0f * planning::Sigmoid(2.0f * _safe_margin - distance);

    return obstacle_cost;
}

bool AP_RoutePlanner::is_valid_curve(const planning::QuinticPolynomialCurve1d &curve) const {
  static constexpr float kMaxLateralDistance = 20.0;
  for (float s = 0.0; s < curve.ParamLength(); s += 2.0) {
    const float l = curve.Evaluate(0, s);
    if (std::fabs(l) > kMaxLateralDistance) {
      return false;
    }
  }
  return true;
}




bool AP_RoutePlanner::SamplePathWaypoints(std::vector<std::vector<planning::Vec2d>> *const points)
{
    if (points == nullptr) {
        return false;
    }

    // 1. push init point into waypoint list
    points->clear();
    points->insert(points->begin(), std::vector<planning::Vec2d>{_init_point_sl});

    // 2. compute sample distance
    const float kMinSampleDistance = 40.0;
    const float total_length = std::min(_init_point_sl.x() + std::max(_init_v * 8.0f, kMinSampleDistance), 
                                       _reference.length());
    
    const float half_adc_width = _vehicle_radius;

    // 3. get number of sample per level
    const float num_sample_per_level = _sample_points_num_each_level;

    // 4. compute longitude sample gap
    static constexpr float kSamplePointLookForwardTime = 4.0f;
    const float level_distance = planning::Clamp(_init_v * kSamplePointLookForwardTime, _step_length_min.get(), _step_length_max.get());

    // 5. sample point
    float accumulated_s = _init_point_sl.x();
    float prev_s = accumulated_s;

    static constexpr size_t kNumLevel = 3;
    for (size_t i = 0; i < kNumLevel && accumulated_s < total_length; ++i) {
        accumulated_s += level_distance;
        if(accumulated_s + level_distance / 2.0f > total_length) {
            accumulated_s = total_length;
        }

        const float s = std::min(accumulated_s, total_length);
        static constexpr float kMinAllowSampleStep = 1.0f;
        if (std::abs(s - prev_s) < kMinAllowSampleStep) {
            continue;
        }
        prev_s = s;

        // 5.1 get left and right boundary of road, left(+), right(-)
        float left_width = _reference_left_bound;
        float right_width = _reference_right_bound;

        static constexpr float kBoundaryBuff = 0.20;
        const float eff_right_width = right_width - half_adc_width - kBoundaryBuff;
        const float eff_left_width = left_width - half_adc_width - kBoundaryBuff;
        const float sample_l_range = eff_right_width + eff_left_width;
        float sample_right_boundary = -eff_right_width;
        float sample_left_boundary = eff_left_width;
        
        // 5.2 shift raod boundary
        if (_init_point_sl.y() > sample_left_boundary || _init_point_sl.y() < sample_right_boundary) {
            sample_right_boundary = std::min(-eff_right_width, _init_point_sl.y());
            sample_left_boundary = std::max(eff_left_width, _init_point_sl.y());

            if (_init_point_sl.y() > eff_left_width) {
                sample_right_boundary = std::max(sample_right_boundary, _init_point_sl.y() - sample_l_range);
            }
            if (_init_point_sl.y() < eff_right_width) {
                sample_left_boundary = std::min(sample_left_boundary, _init_point_sl.y() + sample_l_range);
            }
        }

        // 5.3 slice get points
        std::vector<float> sample_l;
        planning::uniform_slice(sample_right_boundary, sample_left_boundary, static_cast<uint32_t>(num_sample_per_level - 1), &sample_l);

        // 5.3 push back every level points into vector
        std::vector<planning::Vec2d> level_points;
        for (size_t j = 0; j < sample_l.size(); j++) {
            planning::Vec2d sl{s, sample_l[j]};
            level_points.push_back(std::move(sl));
        }
        if (!level_points.empty()) {
            points->emplace_back(level_points);
        }
    }

    return true;
}

// set reference line
void AP_RoutePlanner::set_reference(const planning::DiscretizedTraj &reference)
{
    _reference = reference;
}

void AP_RoutePlanner::update_obstacles()
{
    // reset list
    _dynamic_obstacles.clear();
    _static_obstacles.clear();
    
    // update proximity 
    update_proximity_obstacles();
  
    update_fence_obstacles();
}

void AP_RoutePlanner::update_proximity_obstacles()
{
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }

    // proximity obstacles 
    for (uint16_t i=0; i<oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const Vector2f vel = item.vel.xy();
        struct object obj;
        obj.pos = item.pos.xy();
        obj.vel = item.vel.xy();
        obj.radius = item.radius;

        if (vel.length_squared() <= sq(0.3f)) {
            _static_obstacles.emplace_back(obj);
        } else {
            _dynamic_obstacles.emplace_back(obj);
        }
    }
}

 void AP_RoutePlanner::update_fence_obstacles()
 {
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return;
    }
    // exclusion circles enabled along with polygon fences
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }
    // return immediately if no inclusion nor exclusion circles
    const uint8_t num_exclusion_circles = fence->polyfence().get_exclusion_circle_count();
    if (num_exclusion_circles == 0) {
        return;
    }

    // iterate through exclusion circles
    for (uint8_t i = 0; i < num_exclusion_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {
            struct object obj;
            obj.pos.x = center_pos_cm.x * 0.01f;
            obj.pos.y = center_pos_cm.y * 0.01f;
            obj.vel.zero();
            obj.radius = radius;;
            _static_obstacles.emplace_back(obj);
        }
    }
}