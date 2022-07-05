#include "AP_SpeedDecider.h"
#include "AP_OADatabase.h"

#include <limits>
#include <algorithm>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

namespace {
    static constexpr float unit_t = 3.0f;
    static constexpr float dense_unit_s = 0.1f;
    static constexpr float sparse_unit_s = 1.0f;

    // obstacle cost config
    static constexpr float default_obstacle_cost  =1.0e4f;

    // speed cost config
    static constexpr float default_speed_cost = 1.0e3f;
    static constexpr float exceed_speed_penalty = 1.0e3f;
    static constexpr float low_speed_penalty = 10.0f; 
    static constexpr float reference_speed_penalty = 10.0f;

    // accel cost config
    static constexpr float accel_penalty = 1.0f;
    static constexpr float decel_penalty = 1.0f;

    // jerk cost config
    static constexpr float positive_jerk_coeff = 1.0f;
    static constexpr float negative_jerk_coeff = 1.0f;

    // other constraint
    static constexpr float max_acceleration = 0.5f;
    static constexpr float max_deceleration = 0.5f;

    // spatial potential cost config for minimal time traversal
    static constexpr float spatial_potential_penalty = 1.0e2f;
   
    // other constant parameters
    static constexpr float kfloatEpsilon = 1.0e-6f;
    static constexpr float kInf = std::numeric_limits<float>::infinity();
    static constexpr float track_err_max = 2.0f;
    static constexpr float planning_length_min = 15.0f;
}  // namespace

#define debug 1
#if debug
#include <iostream>
#endif

const AP_Param::GroupInfo AP_SpeedDecider::var_info[] = {

  // @Param: ENABLE
  // @DisplayName: Enable
  // @Description: Enable SpeedDecider
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
  // @RebootRequired: True
  AP_GROUPINFO_FLAGS("ENABLE", 0, AP_SpeedDecider, _enable, 0, AP_PARAM_FLAG_ENABLE),

  // @Param: ACC_MAX
  // @Description: Maximum vehicle acceleration
  // @Units: m/s^2
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("ACC_MAX", 1, AP_SpeedDecider, _acc_max, max_acceleration),

  // @Param: DEC_MAX
  // @Description: Maximum deceleration of vehicle
  // @Units: m/s^2
  // @Range: 0 10
  // @User: Standard
  AP_GROUPINFO("DEC_MAX", 2, AP_SpeedDecider, _dec_max, max_deceleration),

  // @Param: SPEED_MAX
  // @Description: Maximum vehicle speed
  // @Units: m/s
  // @Range: 1 10
  // @User: Standard
  AP_GROUPINFO("SPEED_MAX", 3, AP_SpeedDecider, _speed_max, 3.0f),

  // @Param: ROB_RADIUS
  // @Description: Maximum radius of vehicle shape
  // @Units: m
  // @Range: 0.1 10
  // @User: Standard
  AP_GROUPINFO("ROB_RADIUS", 4, AP_SpeedDecider, _vehicle_radius, 1.0f),

  // @Param: CRUISE_SPD
  // @Description: Vehicle cruise speed
  // @Units: m/s
  // @Range: 1 10
  // @User: Standard
  AP_GROUPINFO("CRUISE_SPD", 5, AP_SpeedDecider, _cruise_speed, 2.0f),

  // @Param: PLN_TIME
  // @Description: Maximum time of speed planning curve
  // @Units: s
  // @Range: 1 30
  // @User: Standard
  AP_GROUPINFO("PLN_TIME", 6, AP_SpeedDecider, _total_t, 10.0f),

  // @Param: PLN_DIST
  // @Description: Maximum distance of speed planning curve
  // @Units: m
  // @Range: 1 150
  // @User: Standard
  AP_GROUPINFO("PLN_DIST", 7, AP_SpeedDecider, _total_s, 30.0f),

  // @Param: OBS_TIME
  // @Description: Obstacle path prediction time
  // @Units: s
  // @Range: 1 10
  // @User: Standard
  AP_GROUPINFO("OBS_TIME", 8, AP_SpeedDecider, _obs_pred_t, 8.0f),

  // @Param: OBS_WGH
  // @Description: Obstacle avoidance penalty factor
  // @Range: 1 1e10
  // @User: Standard
  AP_GROUPINFO("OBS_WGH", 9, AP_SpeedDecider, _obstacle_weight, 1.0f),

  // @Param: ACC_WGH
  // @Description: Acceleration penalty factor
  // @Range: 1 1e10
  // @User: Standard
  AP_GROUPINFO("ACC_WGH", 10, AP_SpeedDecider, _accel_penalty, accel_penalty),

  // @Param: DEC_WGH
  // @Description: Dcceleration penalty factor
  // @Units: m
  // @Range: 1 10
  // @User: Standard
  AP_GROUPINFO("DEC_WGH", 11, AP_SpeedDecider, _decel_penalty, decel_penalty),

  AP_GROUPEND
};

AP_SpeedDecider::AP_SpeedDecider()
{
    _accel_cost.fill(-1.0);
    _jerk_cost.fill(-1.0);
    AP_Param::setup_object_defaults(this,var_info);
}

// run background task to find best cruise speed and update avoidance results
// returns false if obstacle avoidance is not required
bool AP_SpeedDecider::update(const Location &current_loc, const Location& origin, const Location& destination,
                             const Vector2f &ground_speed_vec, float &desired_speed_new, const float dt) 
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

    // translate units cm to m
    current_ne *= 0.01f;
    origin_ne *= 0.01f;
    destination_ne *= 0.01f;
    projected_line_unit = (destination_ne - origin_ne).normalized();

    // projecting current speed into AB 
    float groundSpeed = ground_speed_vec.length();
    float ground_course_deg = degrees(ground_speed_vec.angle());
    Vector2f groundspeed_vector = ground_speed_vec;
    if (groundSpeed < 0.1f) {
      // use a small ground speed vector in the right direction,
      // allowing us to use the compass heading at zero GPS velocity
      groundSpeed = 0.1f;
      groundspeed_vector = Vector2f(cosf(AP::ahrs().yaw), sinf(AP::ahrs().yaw)) * groundSpeed;
      // with zero ground speed use vehicle's heading
      ground_course_deg = AP::ahrs().yaw_sensor * 0.01f;
    }

    // get current speed 
    _curr_speed = MAX(groundspeed_vector * projected_line_unit, 0.0f);
    _planning_period = dt;
    Vector2f mp = current_ne + ground_speed_vec * dt;

    // determine planning start position
    _curr_start = Vector2f::closest_point(mp, origin_ne, destination_ne);
   // const float distance_to_line = (_curr_start - mp).length();
    const float distance_to_end  = MIN((_curr_start - destination_ne).length(), _total_s);
    _planning_length = distance_to_end;

    // determine planning end position
    _curr_end = _curr_start +  projected_line_unit * distance_to_end;
   // const float angle_to_line = wrap_180(ground_course_deg - degrees(projected_line_unit.angle()));

    // could not speed planning, then return not required
    if (/*distance_to_line > track_err_max ||*/ distance_to_end  < planning_length_min  /*|| fabsf(angle_to_line) > 30.0f*/) {
      return false;
    }

    // update obstacle st boundary
    update_obstacle_st_boundary();

    #if debug
    for (auto sl: _st_boundaries) {
      std::cout << "tmin:" << sl.min_t() << " tmax:" << sl.max_t() << " smin:" << sl.min_s() << " smax:" << sl.max_s() << std::endl;
    }
    #endif

    // search speed time graph
    planning::SpeedData speed_data;
    if (!search(&speed_data)) {
      GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to search graph with dynamic programming");
      desired_speed_new = 0.0f;
      return true;
    }
    
    planning::SpeedPoint sp;
    if (!speed_data.EvaluateByTime(dt, &sp)) {
      GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to get speed by time");
      desired_speed_new = 0.0f;
      return true;
    }

    desired_speed_new = sp.v();
    return true;
}


// given time t, calculate the driving limits in s due to vehicle's dynamics
void AP_SpeedDecider::get_vehicle_dynamics_limits(const float t, const float speed, float& low_st, float& upper_st)
{
  // Process lower bound: (constant deceleration)
  float dec_time = speed / _dec_max;
  if (t  < dec_time) {
    low_st = (speed - _dec_max * (t - 0) + speed) * (t - 0) * 0.5;
  } else {
    low_st = (speed * dec_time) * 0.5;
  }

  // Process upper bound: (constant acceleration)
  float acc_time = (_speed_max - speed) / _acc_max;
  if (t  < acc_time) {
    upper_st = (speed + _acc_max * (t - 0) + speed) *
                        (t - 0) * 0.5f;
  } else {
    upper_st = (speed + _speed_max) * acc_time * 0.5f +
                            (t - 0 - acc_time) * _speed_max;
  }
}

// calculate reference line t-s point
float AP_SpeedDecider::get_reference_s(const float t, const float speed, const float desired_speed)
{
  if (speed < desired_speed) {
    float acc_time = (desired_speed - speed) / _acc_max;
    if (t < acc_time) {
      return (speed * t + 0.5f * _acc_max * sq(t));
    } else {
      return (speed + desired_speed) * acc_time * 0.5f + (t - acc_time) * desired_speed;
    }
  } else {
    float dec_time = (speed - desired_speed) /_dec_max;
    if (t < dec_time) {
      return speed * t  - 0.5f * _dec_max * sq(t);
    } else {
      return  (speed + desired_speed) * dec_time * 0.5f + (t - dec_time) * desired_speed;
    }
  }
}

// geneate drivable st boundary
void AP_SpeedDecider::genate_drivable_boundary(const float current_speed, STBound& st_bound)
{
  // initialize st-boundary
  if (!st_bound.empty()) {
    st_bound.clear();
  }
  for (float curr_t = 0.0f; curr_t <= _total_t; curr_t += unit_t) {
    st_bound.emplace_back(curr_t, std::numeric_limits<float>::lowest(),
                          std::numeric_limits<float>::max());
  }

  // sweep-line to get detailed ST-boundary
  for (size_t i = 0; i < st_bound.size(); ++i) {
    float t, s_lower, s_upper;
    std::tie(t, s_lower, s_upper) = st_bound.at(i);

    // get boudary due to driving limits
    get_vehicle_dynamics_limits(t, current_speed, s_lower, s_upper);

    // update into st_bound
    st_bound.at(i) = std::make_tuple(t, s_lower, s_upper);
  }
}

bool AP_SpeedDecider::compute_obstacle_st_boundary(const Vector2f& vehicle_start, const Vector2f& vehicle_end,
                                                   const Vector2f& obs_pos, const Vector2f& obs_vel, const float radius,
                                                   std::vector<planning::STPoint>& lower_points,
                                                   std::vector<planning::STPoint>& upper_points)
{
   // predict obstacle position with current velocity and time t, and mapping ST boundary lower points and upper points
  for (float curr_t = _planning_period; curr_t <= _obs_pred_t; curr_t += _planning_period) {
    const auto obstalce_pos_t = obs_pos + obs_vel * curr_t;
    // get distance of obstacle from reference line
    const auto ref_unit = (vehicle_end - vehicle_start).normalized();
    const float distance_to_ref = std::abs((obstalce_pos_t - vehicle_start) % ref_unit);
    const float projected_to_ref = (obstalce_pos_t - vehicle_start) * ref_unit;
    // determine whether the obstalce is outside both ends of the reference line
    if (distance_to_ref <= _vehicle_radius + radius) {
      // determine ST-boundary
      const float lower_s = projected_to_ref - radius - _vehicle_radius;
      const float upper_s = projected_to_ref + radius + _vehicle_radius;
      if ((lower_s < 0 && upper_s < 0) || (lower_s > _planning_length && upper_s > _planning_length)) {
        continue;
      }
      lower_points.emplace_back(lower_s, curr_t);
      upper_points.emplace_back(upper_s, curr_t);
    }
  }
  return lower_points.size() > 1;
}

void AP_SpeedDecider::update_obstacle_st_boundary()
{
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }

    float smallest_margin = FLT_MAX;
    _st_boundaries.clear();
    for (uint16_t i=0; i<oaDb->database_count(); i++) {
      const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
      std::vector<planning::STPoint> lower_points;
      std::vector<planning::STPoint> upper_points;

      // compute obstacle st boundary
      if (compute_obstacle_st_boundary(_curr_start, _curr_end,
                                  item.pos.xy(), item.vel.xy(), item.radius,
                                  lower_points, upper_points)) {
       planning::STBoundary boundary = planning::STBoundary::create_instance_accurate(lower_points, upper_points);
      _st_boundaries.emplace_back(boundary);
      }

      const float m = (item.pos.xy() - _curr_start).length() - item.radius;
      if (m < smallest_margin) {
        smallest_margin = m;
      }
    }

    #if debug
    if (smallest_margin < _margin_max) {
      std::cout << "smallest margin: " << smallest_margin << std::endl;
    }
    #endif

}



float AP_SpeedDecider::get_obstacle_cost(const planning::StGraphPoint& point)
{
  const float s = point.point().s();
  const float t = point.point().t();

  float cost = 0.0f;
  
  if (_use_st_drivable_boundary) {
    STBound st_bound;
    genate_drivable_boundary(_curr_speed, st_bound);

    int index = static_cast<int>(t / unit_t);
    float ts, lower_bound, upper_bound;
    std::tie(ts, lower_bound, upper_bound) = st_bound.at(index);

    if (s > upper_bound || s < lower_bound) {
      return kInf;
    }
  }

  for (auto& boundary : _st_boundaries) {
    // check invalid
    if (boundary.min_s() > _planning_length) {
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }
    if (boundary.is_point_in_boundary(point.point())) {
      return kInf;
    }

    // compute obstacle cost
    float s_upper = 0.0f;
    float s_lower = 0.0f;
    boundary.get_boundary_srange(t, &s_upper, &s_lower);
    if (s < s_lower) {
      if (s + _margin_max < s_lower) {
        continue;
      } else {
        auto s_diff = _margin_max - s_lower + s;
        cost += _obstacle_weight * default_obstacle_cost *
                s_diff * s_diff;
      }
    } else if (s > s_upper) {
      if (s > s_upper + _margin_max) {  // or calculated from velocity
        continue;
      } else {
        auto s_diff = _margin_max + s_upper - s;
        cost += _obstacle_weight * default_obstacle_cost *
                s_diff * s_diff;
      }
    }
  }
  return cost * unit_t;
}

float AP_SpeedDecider::get_spatial_potential_cost(const planning::StGraphPoint& point)
{
  return ((_curr_end - _curr_start).length() - point.point().s()) * spatial_potential_penalty;
}

float AP_SpeedDecider::get_reference_cost(const planning::STPoint& point, const planning::STPoint& reference_point) const
{
   return 0.0f;
}

float AP_SpeedDecider::get_speed_cost(const planning::STPoint& first, const planning::STPoint& second,
                                      const float speed_limit,
                                      const float cruise_speed) const
{
  float cost = 0.0f;
  const float speed = (second.s() - first.s()) / unit_t;
  if (speed < 0.0f) {
    return kInf;
  }

  float det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0) {
    cost += exceed_speed_penalty * default_speed_cost * (det_speed * det_speed) * unit_t;
  } else if (det_speed < 0) {
    cost += low_speed_penalty * default_speed_cost * -det_speed * unit_t;
  }

  if (_enable_dp_reference_speed) {
    float diff_speed = speed - cruise_speed;
    cost += reference_speed_penalty * default_speed_cost * fabs(diff_speed) * unit_t;
  }

  return cost;
}

float AP_SpeedDecider::get_accel_cost_by_two_points(const float pre_speed,
                                                    const planning::STPoint& pre_point,
                                                    const planning::STPoint& curr_point)
{
  float current_speed = (curr_point.s() - pre_point.s()) / unit_t;
  float accel = (current_speed - pre_speed) / unit_t;
  return get_accel_cost(accel);
}

float AP_SpeedDecider::get_accel_cost_by_three_points(const planning::STPoint& first, 
                                                      const planning::STPoint& second,
                                                      const planning::STPoint& third)
{
  float accel = (first.s() + third.s() - 2 * second.s()) / (unit_t * unit_t);
  return get_accel_cost(accel);
}

float AP_SpeedDecider::get_jerk_cost_by_two_points(const float pre_speed, 
                                                  const float pre_acc,
                                                  const planning::STPoint& pre_point,
                                                  const planning::STPoint& curr_point)
{
  const float curr_speed = (curr_point.s() - pre_point.s()) / unit_t;
  const float curr_accel = (curr_speed - pre_speed) / unit_t;
  const float jerk = (curr_accel - pre_acc) / unit_t;
  return jerk_cost(jerk);
}

float AP_SpeedDecider::get_jerk_cost_by_three_points(const float first_speed,
                                                     const planning::STPoint& first,
                                                     const planning::STPoint& second,
                                                     const planning::STPoint& third)
{
  const float pre_speed = (second.s() - first.s()) / unit_t;
  const float pre_acc = (pre_speed - first_speed) / unit_t;
  const float curr_speed = (third.s() - second.s()) / unit_t;
  const float curr_acc = (curr_speed - pre_speed) / unit_t;
  const float jerk = (curr_acc - pre_acc) / unit_t;
  return jerk_cost(jerk);
}

float AP_SpeedDecider::get_jerk_cost_by_four_points(const planning::STPoint& first, 
                                                    const planning::STPoint& second,
                                                    const planning::STPoint& third,
                                                    const planning::STPoint& fourth)
{
    float jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t * unit_t * unit_t);
  return jerk_cost(jerk);
}

float AP_SpeedDecider::get_accel_cost(const float accel)
{
  float cost = 0.0;
  static constexpr float kEpsilon = 0.1;
  static constexpr size_t kShift = 100;
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
  if (accel_key >= _accel_cost.size()) {
    return kInf;
  }

  if (_accel_cost.at(accel_key) < 0.0) {
    const float accel_sq = accel * accel;
    float max_acc = _acc_max;
    float max_dec = -_dec_max;
    float accel_penalty = _accel_penalty;
    float decel_penalty = _decel_penalty;

    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0f * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0f * (accel - max_acc)));
    _accel_cost.at(accel_key) = cost;
  } else {
    cost = _accel_cost.at(accel_key);
  }
  return cost * unit_t;
}

float AP_SpeedDecider::jerk_cost(const float jerk)
{
  float cost = 0.0;
  static constexpr float kEpsilon = 0.1;
  static constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= _jerk_cost.size()) {
    return kInf;
  }

  if (_jerk_cost.at(jerk_key) < 0.0) {
    float jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = positive_jerk_coeff * jerk_sq * unit_t;
    } else {
      cost = negative_jerk_coeff * jerk_sq * unit_t;
    }
    _jerk_cost.at(jerk_key) = cost;
  } else {
    cost = _jerk_cost.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

// gridded s-t graph optimizers
bool AP_SpeedDecider::search(planning::SpeedData* const speed_data)
{
  static constexpr float kBounadryEpsilon = 1e-2f;
  for (auto& boundary : _st_boundaries) {
   // If init point in collision with obstacle, return speed fallback
    if (boundary.is_point_in_boundary({0.0, 0.0}) ||
        (std::fabs(boundary.min_t()) < kBounadryEpsilon &&
         std::fabs(boundary.min_s()) < kBounadryEpsilon)) {
      _dimension_t = static_cast<uint32_t>(std::ceil(_total_t / static_cast<float>(unit_t))) + 1;
      std::vector<planning::SpeedPoint> speed_profile;
      float t = 0.0f;
      for (uint32_t i = 0; i < _dimension_t; ++i, t += unit_t) {
        speed_profile.push_back(planning::SpeedPoint::ToSpeedPoint(0, t));
      }
      *speed_data = planning::SpeedData(speed_profile);
      return true;
    }
  }


  if (!init_cost_table()) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "initialize cost table failed");
    return false;
  }

  if (!init_speed_limit_lookup()) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "initialize speed limit lookup table failed");
    return false;
  }

  if (!calculate_total_cost()) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "calculate total cost failed");
    return false;
  }

  if (!retrieve_speed_profile(speed_data)) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "retrieve best speed profile failed");
    return false;
  }

  return true;
}

bool AP_SpeedDecider::init_cost_table()
{
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // sanity check for numberical stability
  if (unit_t < kfloatEpsilon) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "unit_t is smaller than kfloatEpsilon");
    return false;
  }
 
  // sanity check on s dimension setting
  if (_dense_dimension_s < 1) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "dense_dimension_s is at least 1");
    return false;
  }

  _dimension_t = static_cast<uint32_t>(std::ceil(_total_t / static_cast<float>(unit_t))) + 1;

   float sparse_length_s =
      _planning_length - static_cast<float>(_dense_dimension_s - 1) * dense_unit_s;

  _sparse_dimension_s =
      sparse_length_s > std::numeric_limits<float>::epsilon()
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s))
          : 0;
  _dense_dimension_s =
      sparse_length_s > std::numeric_limits<float>::epsilon()
          ? _dense_dimension_s
          : static_cast<uint32_t>(std::ceil(_planning_length / dense_unit_s)) +
                1;
  _dimension_s = _dense_dimension_s + _sparse_dimension_s;

  // sanity check 
  if (_dimension_t < 1 || _dimension_s < 1) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Dp st cost table size incorrect");
    return false;
  }

  // reinitialisze
  _accel_cost.fill(-1.0);
  _jerk_cost.fill(-1.0);
  _cost_table.clear();
  _cost_table = std::vector<std::vector<planning::StGraphPoint>>(_dimension_t, std::vector<planning::StGraphPoint>(_dimension_s, planning::StGraphPoint()));

  float curr_t = 0.0f;
  for (uint32_t i = 0; i < _cost_table.size(); ++i, curr_t += unit_t) {
    auto& cost_table_i = _cost_table[i];
    float curr_s = 0.0f;
    for (uint32_t j = 0; j < _dense_dimension_s; ++j, curr_s += dense_unit_s) {
      cost_table_i[j].init(i, j, planning::STPoint(curr_s, curr_t));
    }
    curr_s = static_cast<float>(_dense_dimension_s - 1) * dense_unit_s + sparse_unit_s;
    for (uint32_t j = _dense_dimension_s; j < cost_table_i.size(); ++j, curr_s += sparse_unit_s) {
      cost_table_i[j].init(i, j, planning::STPoint(curr_s, curr_t));
    }
  }

  const auto& cost_table_0 = _cost_table[0];
  _spatial_distance_by_index = std::vector<float>(cost_table_0.size(), 0.0f);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    _spatial_distance_by_index[i] = cost_table_0[i].point().s();
  }

  return true;
}

bool AP_SpeedDecider::init_speed_limit_lookup()
{
  _speed_limit_by_index.clear();
  _speed_limit_by_index.resize(_dimension_s);
  
  for (uint32_t i = 0; i < _dimension_s; i++) {
    _speed_limit_by_index[i] = _speed_max;
  }

  return true;
}

bool AP_SpeedDecider::retrieve_speed_profile(planning::SpeedData* const speed_data)
{
    float min_cost = std::numeric_limits<float>::infinity();
    const planning::StGraphPoint* best_end_point = nullptr;
    for (const planning::StGraphPoint& cur_point : _cost_table.back()) {
      if (!std::isinf(cur_point.total_cost()) &&
          cur_point.total_cost() < min_cost) {
        best_end_point = &cur_point;
        min_cost = cur_point.total_cost();
      }
    }

    for (const auto& row : _cost_table) {
      const planning::StGraphPoint& cur_point = row.back();
      if (!std::isinf(cur_point.total_cost()) &&
          cur_point.total_cost() < min_cost) {
        best_end_point = &cur_point;
        min_cost = cur_point.total_cost();
      }
    }

    if (best_end_point == nullptr) {
      GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Fail to find the best feasible trajectory");
      return false;
    }

    std::vector<planning::SpeedPoint> speed_profile;
    const planning::StGraphPoint* cur_point = best_end_point;
    while (cur_point != nullptr) {
      planning::SpeedPoint speed_point;
      speed_point.set_s(cur_point->point().s());
      speed_point.set_t(cur_point->point().t());
      speed_profile.push_back(speed_point);
      cur_point = cur_point->pre_point();
    }
    std::reverse(speed_profile.begin(), speed_profile.end());

    static constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
    if (speed_profile.front().t() > kEpsilon ||
        speed_profile.front().s() > kEpsilon) {
      GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Fail to retrieve speed profile");
      return false;
    }

    for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
      const float v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                      (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
      speed_profile[i].set_v(v);
    }

    *speed_data = planning::SpeedData(speed_profile);
    return true;
}

bool AP_SpeedDecider::calculate_total_cost()
{
  // col and row are STGraph
  // t corresponding to col
  // s corresponding to row
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;

  for (size_t c = 0; c < _cost_table.size(); ++c) {
    // the hightest row is the maximum distance samping value allowed under the condition of maximum acceleration
    size_t highest_row = 0;
    // the lowest row is the minimum distance samping value allowed value under the condition of maximum deceleration
    size_t lowest_row = _cost_table.back().size() - 1;

    int count = static_cast<int>(next_highest_row) - static_cast<int>(next_lowest_row) + 1;
    if (count > 0) {
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);
        calculate_cost_at(msg);
      }
    }

    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = _cost_table[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<float>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        get_row_range(cost_cr, &h_r, &l_r);
        highest_row = MAX(highest_row, h_r);
        lowest_row = MIN(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }


  return true;
}

void AP_SpeedDecider::calculate_cost_at(const std::shared_ptr<StGraphMessage>& msg)
{
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto& cost_cr = _cost_table[c][r];

  cost_cr.set_obstacle_cost(get_obstacle_cost(cost_cr));
  if (cost_cr.obstacle_cost() > std::numeric_limits<float>::max()) {
    return;
  }

  cost_cr.set_spatial_potential_cost(get_spatial_potential_cost(cost_cr));

  const auto& cost_init = _cost_table[0][0];
  if (c == 0) {
    if (r != 0) {
       GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "incorrect. row should be 0 with col 0");
    }
    cost_cr.set_total_cost(0.0f);
    cost_cr.set_optimal_speed(_curr_speed);
    return;
  }

  const float speed_limit = _speed_limit_by_index[r];
  const float cruise_speed = _cruise_speed;
  // the minimal s to model as constant acceleration formula
  // default: 0.25 * 7 = 1.75m
  const float min_s_consider_speed = dense_unit_s * _dimension_t;

  if (c == 1) {
    const float acc =
        2 * (cost_cr.point().s() / unit_t - _curr_speed) / unit_t;
    if (acc < -_dec_max || acc > _acc_max) {
      return;
    }

    if (_curr_speed + acc * unit_t < -kfloatEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      return;
    }

    if (check_overlap_on_dp_st_graph(_st_boundaries, cost_cr, cost_init)) {
      return;
    }

    cost_cr.set_total_cost(
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
        cost_init.total_cost() +
        calculate_edge_cost_for_second_col(r, speed_limit, cruise_speed));
    cost_cr.set_pre_point(cost_init);
    cost_cr.set_optimal_speed(_curr_speed + acc * unit_t);
    return;
  }

  static constexpr float kSpeedRangeBuffer = 0.20f;
  const float pre_lowest_s =
      cost_cr.point().s() -
      _speed_max * (1 + kSpeedRangeBuffer) * unit_t;
  const auto pre_lowest_itr =
      std::lower_bound(_spatial_distance_by_index.begin(),
                       _spatial_distance_by_index.end(), pre_lowest_s);
  uint32_t r_low = 0;
  if (pre_lowest_itr == _spatial_distance_by_index.end()) {
    r_low = _dimension_s - 1;
  } else {
    r_low = static_cast<uint32_t>(
        std::distance(_spatial_distance_by_index.begin(), pre_lowest_itr));
  }
  const uint32_t r_pre_size = r - r_low + 1;
  const auto& pre_col = _cost_table[c - 1];
  float curr_speed_limit = speed_limit;

  if (c == 2) {
    for (uint32_t i = 0; i < r_pre_size; ++i) {
      uint32_t r_pre = r - i;
      if (std::isinf(pre_col[r_pre].total_cost()) ||
          pre_col[r_pre].pre_point() == nullptr) {
        continue;
      }
      // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
      // data in ST point.
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
      const float curr_a =
          2 *
          ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t -
           pre_col[r_pre].get_optimal_speed()) /
          unit_t;
      if (curr_a < -_dec_max || curr_a > _acc_max) {
        continue;
      }

      if (pre_col[r_pre].get_optimal_speed() + curr_a * unit_t <
              -kfloatEpsilon &&
          cost_cr.point().s() > min_s_consider_speed) {
        continue;
      }

      // Filter out continuous-time node connection which is in collision with
      // obstacle
      if (check_overlap_on_dp_st_graph(_st_boundaries, cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }
      curr_speed_limit =
          std::fmin(curr_speed_limit, _speed_limit_by_index[r_pre]);
      const float cost = cost_cr.obstacle_cost() +
                          cost_cr.spatial_potential_cost() +
                          pre_col[r_pre].total_cost() +
                          calculate_edge_cost_for_third_col(
                              r, r_pre, curr_speed_limit, cruise_speed);

      if (cost < cost_cr.total_cost()) {
        cost_cr.set_total_cost(cost);
        cost_cr.set_pre_point(pre_col[r_pre]);
        cost_cr.set_optimal_speed(pre_col[r_pre].get_optimal_speed() +
                                curr_a * unit_t);
      }
    }
    return;
  }

  for (uint32_t i = 0; i < r_pre_size; ++i) {
    uint32_t r_pre = r - i;
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    const float curr_a =
        2 *
        ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t -
         pre_col[r_pre].get_optimal_speed()) /
        unit_t;
    if (curr_a > _acc_max || curr_a < -_dec_max) {
      continue;
    }

    if (pre_col[r_pre].get_optimal_speed() + curr_a * unit_t < -kfloatEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    if (check_overlap_on_dp_st_graph(_st_boundaries, cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const planning::StGraphPoint& prepre_graph_point = _cost_table[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }
    const planning::STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const planning::STPoint& prepre_point = prepre_graph_point.point();
    const planning::STPoint& pre_point = pre_col[r_pre].point();
    const planning::STPoint& curr_point = cost_cr.point();
    curr_speed_limit =
        std::fmin(curr_speed_limit, _speed_limit_by_index[r_pre]);
    float cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  calculate_edge_cost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);

    if (cost < cost_cr.total_cost()) {
      cost_cr.set_total_cost(cost);
      cost_cr.set_pre_point(pre_col[r_pre]);
      cost_cr.set_optimal_speed(pre_col[r_pre].get_optimal_speed() +
                              curr_a * unit_t);
    }
  }
}

float AP_SpeedDecider::calculate_edge_cost(const planning::STPoint& first, const planning::STPoint& second,
                                           const planning::STPoint& third, const planning::STPoint& forth,
                                           const float speed_limit, const float cruise_speed)
{
  return get_speed_cost(third, forth, speed_limit, cruise_speed) +
         get_accel_cost_by_three_points(second, third, forth) +
         get_jerk_cost_by_four_points(first, second, third, forth);
}

float AP_SpeedDecider::calculate_edge_cost_for_second_col(const uint32_t row,
                                          const float speed_limit,
                                          const float cruise_speed)
{
  float init_speed = _curr_speed;
  float init_acc = 0.0f;
  const planning::STPoint& pre_point = _cost_table[0][0].point();
  const planning::STPoint& curr_point = _cost_table[1][row].point();
  return get_speed_cost(pre_point, curr_point, speed_limit, cruise_speed) +
         get_accel_cost_by_two_points(init_speed, pre_point, curr_point) +
         get_jerk_cost_by_two_points(init_speed, init_acc, pre_point, curr_point);
}

float AP_SpeedDecider::calculate_edge_cost_for_third_col(const uint32_t curr_row,
                                        const uint32_t pre_row,
                                        const float speed_limit,
                                        const float cruise_speed)
{
  float init_speed = _curr_speed;
  const planning::STPoint& first = _cost_table[0][0].point();
  const planning::STPoint& second = _cost_table[1][pre_row].point();
  const planning::STPoint& third = _cost_table[2][curr_row].point();
  return get_speed_cost(second, third, speed_limit, cruise_speed) +
         get_accel_cost_by_three_points(first, second, third) +
         get_jerk_cost_by_three_points(init_speed, first, second, third);
}

// get the row-range of next time step
void AP_SpeedDecider::get_row_range(const planning::StGraphPoint& point, 
                                    size_t* next_highest_row,
                                    size_t* next_lowest_row)
{
    // TODO(all): Record speed information in StGraphPoint and deprecate this.
    // A scaling parameter for DP range search due to the lack of accurate
    // information of the current velocity (set to 1 by default since we use
    // past 1 second's average v as approximation)
    float v0 = 0.0f;
    float acc_coeff = 0.5;
    if (!point.pre_point()) {
      v0 = _curr_speed;
    } else {
      v0 = point.get_optimal_speed();
    }

    const auto max_s_size = _dimension_s - 1;
    const float t_squared = unit_t * unit_t;
    const float s_upper_bound = v0 * unit_t +
                                acc_coeff * _acc_max * t_squared +
                                point.point().s();
    const auto next_highest_itr =
        std::lower_bound(_spatial_distance_by_index.begin(),
                        _spatial_distance_by_index.end(), s_upper_bound);
    if (next_highest_itr == _spatial_distance_by_index.end()) {
      *next_highest_row = max_s_size;
    } else {
      *next_highest_row =
          std::distance(_spatial_distance_by_index.begin(), next_highest_itr);
    }

    const float s_lower_bound =
        std::fmax(0.0, v0 * unit_t - acc_coeff * _dec_max * t_squared) +
        point.point().s();
    const auto next_lowest_itr =
        std::lower_bound(_spatial_distance_by_index.begin(),
                        _spatial_distance_by_index.end(), s_lower_bound);
    if (next_lowest_itr == _spatial_distance_by_index.end()) {
      *next_lowest_row = max_s_size;
    } else {
      *next_lowest_row =
          std::distance(_spatial_distance_by_index.begin(), next_lowest_itr);
    }
}

// Continuous-time collision check using linear interpolation as closed-loop
// dynamics
bool AP_SpeedDecider::check_overlap_on_dp_st_graph(const std::vector<planning::STBoundary>& boundaries,
                                                   const planning::StGraphPoint& p1, 
                                                   const planning::StGraphPoint& p2)
{

  for (const auto& boundary : boundaries) {
    // Check collision between a polygon and a line segment
    if (boundary.HasOverlap({p1.point(), p2.point()})) {
      return true;
    }
  }

  return false;
}