#include "AP_DPPlanner.h"
#include <AP_Planning/linear_interpolation.h>

const AP_Param::GroupInfo AP_DPPlanner::var_info[] = {


  AP_GROUPINFO_FLAGS("ENABLE", 0, AP_DPPlanner, _enable, 0, AP_PARAM_FLAG_ENABLE),

  AP_GROUPINFO("ACC_MAX", 1, AP_DPPlanner, _nfe, 80),

  AP_GROUPINFO("DEC_MAX", 2, AP_DPPlanner, _tf, 16),

  AP_GROUPINFO("SPEED_MAX", 3, AP_DPPlanner, _cruise_speed, 2.0f),

  AP_GROUPINFO("ROB_RADIUS", 4, AP_DPPlanner, _speed_max, 3.0f),

  AP_GROUPINFO("CRUISE_SPD", 5, AP_DPPlanner, _dcceleration_max, 0.3f),

  AP_GROUPINFO("PLN_TIME", 6, AP_DPPlanner, _acceleration_max, 0.3f),

  AP_GROUPINFO("PLN_DIST", 7, AP_DPPlanner, _jerk_max, 10.0f),

  AP_GROUPINFO("OBS_TIME", 8, AP_DPPlanner, _vehicle_radius, 1.0f),

  AP_GROUPINFO("OBS_WGH", 9, AP_DPPlanner, _obstacle_weight, 1.0f),

  AP_GROUPINFO("ACC_WGH", 10, AP_DPPlanner, _obstacle_weight, 1000),

  AP_GROUPINFO("DEC_WGH", 11, AP_DPPlanner, _lateral_weight, 0.1f),

  AP_GROUPINFO("ACC_WGH", 10, AP_DPPlanner, _lateral_change_weight, 0.5f),

  AP_GROUPINFO("DEC_WGH", 11, AP_DPPlanner, _lateral_vel_change_weight, 1.0f),

  AP_GROUPINFO("ACC_WGH", 10, AP_DPPlanner, _longitudinal_vel_weight, 10.0f),

  AP_GROUPINFO("DEC_WGH", 11, AP_DPPlanner, _longitudinal_vel_change_weight, 1.0f),

  AP_GROUPEND
};

AP_DPPlanner::AP_DPPlanner() 
{
    AP_Param::setup_object_defaults(this, var_info);

    _nseg = _nfe / NT;
    _unit_time = _tf / NT;
    _time = planning::linspace<NT>(_unit_time, _tf);
    _station = planning::linspace<NS>(0.0f, _unit_time * _speed_max);
    _lateral = planning::linspace<NL-1>(0.0f, 1.0f);
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

    Vector2f current_ne, origin_ne, destination_ne, projected_line_unit;
    // convert location with lat-lng to offsets from ekf orgin
    if (!current_loc.get_vector_xy_from_origin_NE(current_ne) ||
        !origin.get_vector_xy_from_origin_NE(origin_ne) ||
        !destination.get_vector_xy_from_origin_NE(destination_ne)) {
        // OA is not required
        return false;
    }
    return true;
}

// get collision cost
float AP_DPPlanner::get_collision_cost(StateIndex parent_ind, StateIndex current_ind)
{
    return 0.0f;
}

// get total cost
std::pair<float, float> AP_DPPlanner::get_total_cost(StateIndex parent_ind, StateIndex cur_ind)
{
    return std::make_pair(0,0);
}

// get lateral offset
float AP_DPPlanner::get_lateral_offset(const float s, const int16_t l_ind)
{
    if (l_ind == NL - 1) {
        return 0.0f;
    }
    const float lb = -_safe_margin;
    const float ub = _safe_margin;
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