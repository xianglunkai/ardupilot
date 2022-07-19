#include "AP_DPPlanner.h"
#include "AP_OADatabase.h"
#include <AP_Planning/linear_interpolation.h>
#include <AP_AHRS/AP_AHRS.h>
#include <algorithm>


#define debug 1
#if debug
#include <iostream>
#endif

constexpr float DPPLANNER_LOOKAHEAD_TIME = 1.5f;

const AP_Param::GroupInfo AP_DPPlanner::var_info[] = {

  // @Param: ENABLE
  // @DisplayName: Enable
  // @Description: Enable SLT planner
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
  // @RebootRequired: True
  AP_GROUPINFO_FLAGS("ENABLE", 0, AP_DPPlanner, _enable, 0, AP_PARAM_FLAG_ENABLE),

  // @Param: NFE
  // @Description: number of finite elements used to descretize an OCP
  // @Range: 1 320
  // @User: Standard
  AP_GROUPINFO("NFE", 1, AP_DPPlanner, _nfe, 20),

  // @Param: TF
  // @Description: time horizon length
  // @Range: 1 100
  // @User: Standard
  AP_GROUPINFO("TF", 2, AP_DPPlanner, _tf, 10),

  // @Param: SPEED_MAX
  // @Description: max speed
  // @User: Standard
  AP_GROUPINFO("SPEED_MAX", 3, AP_DPPlanner, _speed_max, 3.0f),

  // @Param: DCC_MAX
  // @Description: Maximum vehicle acceleration
  // @Units: m/s^2
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("DCC_MAX", 4, AP_DPPlanner, _dcceleration_max, 0.3f),

  // @Param: ACC_MAX
  // @Description: Maximum vehicle acceleration
  // @Units: m/s^2
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("ACC_MAX", 5, AP_DPPlanner, _acceleration_max, 0.3f),

  // @Param: JERK_MAX
  // @Description: Maximum vehicle acceleration
  // @Units: m/s^2
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("JERK_MAX", 6, AP_DPPlanner, _jerk_max, 10.0f),

  // @Param: ROB_WID
  // @Description: vehicle width
  // @Units: m
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("ROB_WID", 7, AP_DPPlanner, _vehicle_width, 0.25f),

  // @Param: ROB_LEN
  // @Description: vehicle length
  // @Units: m
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("ROB_LEN", 8, AP_DPPlanner, _vehicle_length, 1.0f),

  // @Param: OBS_WGT
  // @Description: cost of obstacles
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("OBS_WGH", 9, AP_DPPlanner, _obstacle_weight, 1000.0f),

  // @Param: LAT_WGT
  // @Description: lateral cost
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("LAT_WGT", 10, AP_DPPlanner, _lateral_weight, 0.1f),

  // @Param: SLAT_WGT
  // @Description: lateral change cost dl/ds
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("SLAT_WGT", 11, AP_DPPlanner, _lateral_change_weight, 0.5f),

  // @Param: TLAT_WGT
  // @Description: lateral change cost, dl/dt
  // @Units: m/s^2
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("TLAT_WGT", 12, AP_DPPlanner, _lateral_vel_change_weight, 1.0f),

  // @Param: LON_WGT
  // @Description: longitudinal velocity cost
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("LON_WGT", 13, AP_DPPlanner, _longitudinal_vel_weight, 10.0f),

  // @Param: TLON_WGT
  // @Description: Cost of longitudinal velocity change, ds/dt
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("TLON_WGT", 14, AP_DPPlanner, _longitudinal_vel_change_weight, 1.0f),

  // @Param: REF_LB
  // @Description: reference center line left range
  // @Units: m
  // @Range: 0 100
  // @User: Standard
  AP_GROUPINFO("REF_LB", 15, AP_DPPlanner, _reference_left_bound, 100.0f),

  // @Param: REF_RB
  // @Description: reference center line right range
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("REF_RB", 16, AP_DPPlanner, _reference_right_bound, 100.0f),

  // @Param: REF_RES
  // @Description: reference center line resolution
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("REF_RES", 17, AP_DPPlanner, _reference_resolution, 0.1f),

  AP_GROUPEND
};

AP_DPPlanner::AP_DPPlanner() 
{
    AP_Param::setup_object_defaults(this, var_info);
}

// run background task to find best cruise speed and update avoidance results
// returns false if obstacle avoidance is not required
bool AP_DPPlanner::update(const Location &current_loc, const Location& origin, const Location& destination, const Vector2f &ground_speed_vec, 
            Location &origin_new, Location &destination_new, float &desired_speed_new, 
            bool proximity_only)
{
    if (!_enable) {
     return false;
    }

    // initial 
    _nseg = _nfe / NT;
    _unit_time = _tf / NT;
    _time = planning::LinSpaced<NT>(_unit_time, _tf);
    _station = planning::LinSpaced<NS>(0.0f, _unit_time * _speed_max);
    _lateral = planning::LinSpaced<NL-1>(0.0f, 1.0f);

    Vector2f current_ne, origin_ne, destination_ne;
    // convert location with lat-lng to offsets from ekf orgin
    if (!current_loc.get_vector_xy_from_origin_NE(current_ne) ||
        !origin.get_vector_xy_from_origin_NE(origin_ne) ||
        !destination.get_vector_xy_from_origin_NE(destination_ne)) {
        // OA is not required
        return false;
    }
    // translate units cm to m
    current_ne *= 0.01f;
    origin_ne *= 0.01f;
    destination_ne *= 0.01f;

    // get ground course
    float ground_course_deg;
    float groundSpeed = ground_speed_vec.length();
    if (ground_speed_vec.length_squared() < sq(0.2f)) {
        // with zero ground speed use vehicle's heading
        ground_course_deg = AP::ahrs().yaw_sensor * 0.01f;
        groundSpeed = 0.0f;
    } else {
        ground_course_deg = degrees(ground_speed_vec.angle());
    }

    // generate reference
    _desired_speed_input = desired_speed_new;
    const bool reference_matches = (_start == origin_ne) && (_end == destination_ne);
    if (!reference_matches) {
        // update reference line
        _start = origin_ne;
        _end = destination_ne;
        // calculate bearing and distance to final destination
        const float reference_bearing = origin.get_bearing_to(destination) * 0.01f;
        const float reference_length = origin.get_distance(destination);
        const Vector2f unit_ref = (_end - _start).normalized();
        planning::Trajectory traj;
        float s = 0;
        while ( s < reference_length + _reference_resolution) {
            const Vector2f pt = _start + unit_ref * s;
            planning::TrajPoint tp;
            tp.s = s;
            tp.x = pt.x;
            tp.y = pt.y;
            tp.theta = radians(reference_bearing);
            tp.kappa = 0;
            tp.velocity = _desired_speed_input;
            tp.left_bound = _reference_left_bound;
            tp.right_bound = _reference_right_bound;
            traj.emplace_back(tp);
            s += _reference_resolution;
        }
        set_reference(planning::DiscretizedTraj(traj));
    }
    
    // update obstacle list
    update_obstacles(proximity_only);

    // update planning start point
    auto sl = _reference.get_projection({current_ne.x, current_ne.y});
    _state.start_s = sl.x();
    _state.start_l = sl.y();
    _state.start_theta = radians(ground_course_deg);

    // reset state space
    for (int i = 0; i < NT; i++) {
        for (int j = 0; j < NS; j++) {
            for (int k = 0; k < NL; k++) {
                _state_space[i][j][k] = StateCell();
            }
        }
    }

    // evaluate first layer
    for (int i = 0; i < NS; i++) {
        for (int j = 0; j < NL; j++) {
            auto tup = get_total_cost(StateIndex(-1, -1, -1), StateIndex(0, i, j));
            _state_space[0][i][j].current_s = tup.first;
            _state_space[0][i][j].cost = tup.second;
        }
    }

    // dynamical programming
    for (int i = 0; i < NT - 1; i++) {
        for (int j = 0; j < NS; j++) {
            for (int k = 0; k < NL; k++) {
                StateIndex parent_ind(i, j, k);

                for (int m = 0; m < NS; m++) {
                    for (int n = 0; n < NL; n++) {
                        StateIndex current_ind(i + 1, m , n);
                        auto tup = get_total_cost(parent_ind, current_ind);
                        const float delta_cost = tup.second;
                        const float cur_s = tup.first;

                        const float cur_cost = _state_space[i][j][k].cost + delta_cost;
                        if (cur_cost < _state_space[i+1][m][n].cost) {
                            _state_space[i+1][m][n] = StateCell(cur_cost, cur_s, j, k);
                        }
                    }
                }
            }
        }
    }

    // find the least cost in final layer
    float min_cost = std::numeric_limits<float>::max();
    int16_t min_s_ind = 0, min_l_ind = 0;
    for (int16_t i = 0; i < NS; i++) {
        for (int16_t j = 0; j < NL; j++) {
            const float cost = _state_space[NT - 1][i][j].cost;
            if (cost < min_cost) {
                min_s_ind = i;
                min_l_ind = j;
                min_cost = cost;
            }
        }
    }

    // trace back layers to find optimum traj
    std::vector<std::pair<StateIndex, StateCell>> waypoints(NT);
    for (int16_t i = NT - 1; i >= 0; i--) {
        auto &cell  = _state_space[i][min_s_ind][min_l_ind];
        waypoints[i] = std::make_pair(StateIndex(i, min_s_ind, min_l_ind), cell);
        min_s_ind = cell.parent_s_ind;
        min_l_ind = cell.parent_l_ind;
    }

    // interpolation
    const float dt = _tf / (_nfe - 1);
    planning::Trajectory data(_nfe);
    float last_l = _state.start_l, last_s = _state.start_s;
    for (int16_t i = 0; i < NT; i++) {
        float parent_s = i > 0 ? waypoints[i-1].second.current_s : _state.start_s;
        auto segment = interpolate_linearly(parent_s, waypoints[i].second.parent_l_ind, waypoints[i].first.s, waypoints[i].first.l);

        for (int j = 0; j < _nseg; j++) {
            auto dl = segment[j].y() - last_l;
            auto ds = std::max(segment[j].x() - last_s, planning::kMathEpsilon);
            last_l = segment[j].y();
            last_s = segment[j].x();

            auto xy = _reference.get_cartesian(segment[j].x(), segment[j].y());
            auto tp = _reference.evaluate_station(segment[j].x());
            int n = i * _nseg + j;
            data[n].s = segment[j].x();
            data[n].x = xy.x();
            data[n].y = xy.y();
            data[n].velocity = (n == 0) ? groundSpeed : (data[n].s - data[n-1].s) / dt;
            data[n].theta = (n  == 0) ? _state.start_theta : tp.theta + atanf((dl / ds) / (1 - tp.kappa * segment[j].y()));
        }
    }
   
    // construct planning trajectory
    auto result = planning::DiscretizedTraj(data);

    #if debug
    int16_t i = 0;
    for (auto res:result.data()) {
        printf("t[%f], s[%f], vel[%f]\n", dt * i, res.s, res.velocity);
        i++;
    }
    #endif

    // get next desired location and velocity
    uint16_t next_track_ind = 0;
    for (uint16_t ind = 0; ind < result.data().size(); ind++) {
        next_track_ind =  ind;
        const Vector2f q(result.data().at(ind).x, result.data().at(ind).y);
        if ((q - current_ne).length() > 10.0f) {
            break;
        }
    }

    auto p = result.data().at(next_track_ind);
    desired_speed_new = p.velocity;
    if (desired_speed_new > 0.3f) {
        destination_new = current_loc;
        const Vector2f vec_p{p.x, p.y};
        const float final_bearing = degrees((vec_p - current_ne).angle());
        const float final_distance = (vec_p - current_ne).length();
        destination_new.offset_bearing(final_bearing, final_distance);
    }
    return true;
}

// set reference line
void AP_DPPlanner::set_reference(const planning::DiscretizedTraj &reference)
{
    _reference = reference;
    _road_barrier.clear();
    const float start_s = _reference.data().front().s;
    const float back_s = _reference.data().back().s;
    int16_t sample_point = (back_s - start_s) / _reference_resolution;
    for (int16_t i = 0; i <= sample_point; i++) {
        const float s = start_s + i * _reference_resolution;
        auto ref = _reference.evaluate_station(s);
        _road_barrier.push_back(_reference.get_cartesian(s, ref.left_bound));
        _road_barrier.push_back(_reference.get_cartesian(s, -ref.right_bound));
    }

    std::sort(_road_barrier.begin(), _road_barrier.end(), [](const planning::Vec2d &a, const planning::Vec2d &b) {
    return a.x() < b.x();
    });
}


// get collision cost
float AP_DPPlanner::get_collision_cost(StateIndex parent_ind, StateIndex current_ind)
{
    float parent_s = _state.start_s, grandparent_s = _state.start_s;
    float last_l = _state.start_l, last_s = _state.start_s;
    if (parent_ind.t >= 0 ) {
        auto &cell = _state_space[parent_ind.t][parent_ind.s][parent_ind.l];
        parent_s = cell.current_s;

        if (parent_ind.t > 0) {
            auto &parent_cell = _state_space[parent_ind.t - 1][cell.parent_s_ind][cell.parent_l_ind];
            grandparent_s = parent_cell.current_s;
        }
        
        // get previous path
        auto prev_path = interpolate_linearly(grandparent_s, cell.parent_l_ind, parent_ind.s, parent_ind.l);
        last_l = prev_path.back().y();
        last_s = prev_path.back().x();
    }

    auto path = interpolate_linearly(parent_s, parent_ind.l, current_ind.s, current_ind.l);

    for (uint16_t i = 0; i < path.size(); i++) {
        auto &pt = path[i];
        float dl = pt.y() - last_l;
        float ds = MAX(pt.x() - last_s, planning::kMathEpsilon);
        last_l = pt.y();
        last_s = pt.x();

        auto cart = _reference.get_cartesian(pt.x(), pt.y());
        auto ref  = _reference.evaluate_station(pt.x());
        const float margin = _vehicle_width / 2 * 1.5f;
        const float lb = std::min(0.0f, -ref.right_bound + margin);
        const float ub = std::max(0.0f,  ref.left_bound - margin);
        if (pt.y() < lb - planning::kMathEpsilon || pt.y() > ub + planning::kMathEpsilon) {
            return _obstacle_weight;
        }
        const float heading = ref.theta + atan((dl/ds) / (1 - ref.kappa * pt.y()));
        planning::Pose pose(cart.x(), cart.y(), heading);

        const float parent_time = parent_ind.t < 0 ? 0.0f : _time[parent_ind.t];
        const float time = parent_time + i * (_unit_time / _nseg);

        if (check_optimization_collision(time, pose)) {
            return _obstacle_weight;
        }
    }
    return 0;

}

// get total cost
std::pair<float, float> AP_DPPlanner::get_total_cost(StateIndex parent_ind, StateIndex cur_ind)
{
    float parent_s = _state.start_s, grandparent_s = _state.start_s;
    float parent_l = _state.start_l, grandparent_l = _state.start_l;

    if (parent_ind.t >= 0) {
        auto &cell = _state_space[parent_ind.t][parent_ind.s][parent_ind.l];
        int grandparent_s_ind = cell.parent_s_ind;
        int grandparent_l_ind = cell.parent_l_ind;
        parent_s = cell.current_s;
        parent_l = get_lateral_offset(parent_s, parent_ind.l);

        if (parent_ind.t >= 1) {
            grandparent_s = _state_space[parent_ind.t - 1][grandparent_s_ind][grandparent_l_ind].current_s;
            grandparent_l = get_lateral_offset(grandparent_s, grandparent_l_ind);
        }
    }

    float cur_s = parent_s + _station[cur_ind.s];
    float cur_l = get_lateral_offset(cur_s, cur_ind.l);

    float ds1 = cur_s - parent_s;
    float dl1 = cur_l - parent_l;

    float ds0 = parent_s - grandparent_s;
    float dl0 = parent_l - grandparent_l;

    float cost_obstacle = get_collision_cost(parent_ind, cur_ind);
    if (cost_obstacle >= _obstacle_weight) {
        return std::make_pair(cur_s, _obstacle_weight);
    }

    float cost_lateral = fabs(cur_l);
    float cost_lateral_change = fabs(parent_l - cur_l) / (_station[cur_ind.s] + planning::kMathEpsilon);
    float cost_lateral_change_t = fabs(dl1 - dl0) / _unit_time;
    float cost_longitudinal_velocity = fabs(ds1 / _unit_time - _desired_speed_input);
    float cost_longitudinal_velocity_change = fabs((ds1 - ds0) / _unit_time);

    float delta_cost = (
        _lateral_weight * cost_lateral +
        _lateral_change_weight * cost_lateral_change +
        _lateral_vel_change_weight * cost_lateral_change_t +
        _longitudinal_vel_weight * cost_longitudinal_velocity +
        _longitudinal_vel_change_weight * cost_longitudinal_velocity_change);

    return std::make_pair(cur_s, delta_cost);
}

// get lateral offset
float AP_DPPlanner::get_lateral_offset(const float s, const int16_t l_ind)
{
    if (l_ind == NL - 1) {
        return 0.0f;
    }
    auto ref = _reference.evaluate_station(s);
    const float margin = _vehicle_width / 2 * 1.5f;
    const float lb = -ref.right_bound + margin;
    const float ub = ref.left_bound - margin;
    return lb + (ub - lb) * _lateral[l_ind];
}

// linear polate
std::vector<planning::Vec2d> AP_DPPlanner::interpolate_linearly(const float parent_s, const int16_t parent_l_ind, const int16_t current_s_ind, const int16_t current_l_ind)
{
    std::vector<planning::Vec2d> result(_nseg);
    float pl = _state.start_l;
    float ps = _state.start_s;
    if (parent_l_ind >= 0) {
        ps = parent_s;
        pl = get_lateral_offset(ps, parent_l_ind);
    }

    const float cur_s = ps + _station[current_s_ind];
    const float cur_l = get_lateral_offset(cur_s, current_l_ind);

    const float s_step = _station[current_s_ind] / _nseg;
    const float l_step = (cur_l - pl) / _nseg;

    for (int16_t i = 0; i < _nseg; i++) {
        result[i].set_x(ps + i * s_step);
        result[i].set_y(pl + i * l_step);
    }

    return result;
}


template<class T>
std::tuple<T, T, T, T> AP_DPPlanner::get_disc_positions(const T &x, const T &y, const T &theta) const
{
    const float r2x = 0.25f * _vehicle_length;
    const float f2x = 0.75f * _vehicle_length;
    auto xf = x + f2x * cosf(theta);
    auto xr = x + r2x * cosf(theta);
    auto yf = y + f2x * sinf(theta);
    auto yr = y + r2x * sinf(theta);
    return std::make_tuple(xf, yf, xr, yr);
}

bool AP_DPPlanner::check_station_collision(const planning::Box2d &rect)
{
    for (auto &obstacle: _static_obstacles) {
        if (obstacle.HasOverlap(rect)) {
            return true;
        }
    }
    
    if (_road_barrier.empty()) {
        return false;
    }

    if (rect.max_x() < _road_barrier.front().x() || rect.min_x() > _road_barrier.back().x()) {
        return false;
    }

    auto comp = [](float val, const planning::Vec2d &a) {
        return val < a.x();
    };

    // binary search
    auto check_start = std::upper_bound(_road_barrier.begin(), _road_barrier.end(), rect.min_x(), comp);
    auto check_end   = std::upper_bound(_road_barrier.begin(), _road_barrier.end(), rect.max_x(), comp);

    if (check_start > _road_barrier.begin()) {
        std::advance(check_start, -1);
    }

    for (auto iter = check_start; iter != check_end; iter++) {
        if (rect.IsPointIn(*iter)) {
        return true;
        }
    }

    return false;
}

bool AP_DPPlanner::check_dynamical_collision(const float time, const planning::Box2d &rect)
{
    for (auto &obstacle: _dynamic_obstacles) {
        auto obs_box_t = obstacle.second.AABoundingBox();
        obs_box_t.Shift(obstacle.first * time);
        if (planning::Box2d(obs_box_t).HasOverlap(rect)) {
            return true;
        }
    }
    return false;
}

bool AP_DPPlanner::check_optimization_collision(const float time, const planning::Pose &pose)
{
    const float radius = std::hypot(0.25f * _vehicle_length, 0.5f * _vehicle_width);
    planning::AABox2d initial_box(
        {-radius - _safe_margin, -radius - _safe_margin},
        {+radius + _safe_margin, +radius + _safe_margin}
    );
    float xr, yr, xf, yf;
    std::tie(xr, yr, xf, yf) = get_disc_positions(pose.x(), pose.y(), pose.theta());
    auto f_box = initial_box, r_box = initial_box;
    f_box.Shift({xf, yf});
    r_box.Shift({xr, yr});
  if (check_station_collision(planning::Box2d(f_box)) || check_station_collision(planning::Box2d(r_box)) ||
      check_dynamical_collision(time, planning::Box2d(f_box)) || check_dynamical_collision(time, planning::Box2d(r_box))) {
    return true;
  }
  return false;
}

void AP_DPPlanner::update_obstacles(bool proximity_only)
{
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }

    _dynamic_obstacles.clear();
    _static_obstacles.clear();
    for (uint16_t i=0; i<oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const Vector2f vel = item.vel.xy();

        planning::AABox2d initial_box(
            {-item.radius , -item.radius},
            {+item.radius , +item.radius}
            );
        initial_box.Shift({item.pos.x,item.pos.y});
        auto obs_box = planning::Box2d(initial_box);

        if (vel.length_squared() <= sq(0.2f)) {
            _static_obstacles.emplace_back(planning::Polygon2d(obs_box));
        } else {
            _dynamic_obstacles.emplace_back(std::make_pair(planning::Vec2d{vel.x, vel.y}, planning::Polygon2d(obs_box)));
        }
    }
    if (proximity_only) {
        return;
    }
    
}