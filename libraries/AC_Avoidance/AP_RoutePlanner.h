#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Planning/path_data.h>
#include <AP_Planning/speed_data.h>
#include <AP_Planning/discretized_traj.h>
#include <AP_Planning/discretized_trajectory.h>
#include <AP_Planning/quintic_polynomial_curve1d.h>

#include <array>
#include <cmath>


class ComparableCost {
public:
    ComparableCost() = default;
    ComparableCost(const bool has_collision, const bool out_of_boundary,
                  const bool out_of_lane, const float safety_cost_,
                  const float smoothness_cost_)
        : safety_cost(safety_cost_), smoothness_cost(smoothness_cost_) {
      cost_items[HAS_COLLISION] = has_collision;
      cost_items[OUT_OF_BOUNDARY] = out_of_boundary;
      cost_items[OUT_OF_LANE] = out_of_lane;
    }
    ComparableCost(const ComparableCost &) = default;

    int CompareTo(const ComparableCost &other) const {
      for (size_t i = 0; i < cost_items.size(); ++i) {
        if (cost_items[i]) {
          if (other.cost_items[i]) {
            continue;
          } else {
            return 1;
          }
        } else {
          if (other.cost_items[i]) {
            return -1;
          } else {
            continue;
          }
        }
      }

      static constexpr float kEpsilon = 1e-12;
      const float diff = safety_cost + smoothness_cost - other.safety_cost -
                          other.smoothness_cost;
      if (std::abs(diff) < kEpsilon) {
        return 0;
      } else if (diff > 0) {
        return 1;
      } else {
        return -1;
      }
    }
    ComparableCost operator+(const ComparableCost &other) {
      ComparableCost lhs = *this;
      lhs += other;
      return lhs;
    }
    ComparableCost &operator+=(const ComparableCost &other) {
      for (size_t i = 0; i < cost_items.size(); ++i) {
        cost_items[i] = (cost_items[i] || other.cost_items[i]);
      }
      safety_cost += other.safety_cost;
      smoothness_cost += other.smoothness_cost;
      return *this;
    }
    bool operator>(const ComparableCost &other) const {
      return this->CompareTo(other) > 0;
    }
    bool operator>=(const ComparableCost &other) const {
      return this->CompareTo(other) >= 0;
    }
    bool operator<(const ComparableCost &other) const {
      return this->CompareTo(other) < 0;
    }
    bool operator<=(const ComparableCost &other) const {
      return this->CompareTo(other) <= 0;
    }
    /*
    * cost_items represents an array of factors that affect the cost,
    * The level is from most critical to less critical.
    * It includes:
    * (0) has_collision or out_of_boundary
    * (1) out_of_lane
    *
    * NOTICE: Items could have same critical levels
    */
    static const size_t HAS_COLLISION = 0;
    static const size_t OUT_OF_BOUNDARY = 1;
    static const size_t OUT_OF_LANE = 2;
    std::array<bool, 3> cost_items = {{false, false, false}};

    // cost from distance to obstacles or boundaries
    float safety_cost = 0.0f;
    // cost from deviation from lane center, path curvature etc
    float smoothness_cost = 0.0f;
};


class AP_RoutePlanner {
public:
    AP_RoutePlanner();

    ~AP_RoutePlanner() = default;

    // do not allow copies
    CLASS_NO_COPY(AP_RoutePlanner);

    // send configuration info stored in front end parameters
    void set_config(float margin_max) { _safe_margin = MAX(margin_max, 0.0f); }


    // run background task to find best cruise speed and update avoidance results
    // returns false if obstacle avoidance is not required
    bool update(const Location &current_loc, 
                const Location& origin, 
                const Location& destination, 
                const planning::SpeedData &speed_data,
                const float planning_cycle_time,
                planning::PathData *const path_data);

    // paramters table
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Reference module
    planning::DiscretizedTraj _reference;
    Vector2f _start;
    Vector2f _end;
    void set_reference(const planning::DiscretizedTraj &reference);

private:
  // obstacle module
    struct object {
        Vector2f pos;           // position of the object as an offset in meters from the EKF origin
        Vector2f vel;           // velocity of object in meters from from EKF origin
        float radius;           // objects radius in meters
    };
    std::vector<object> _static_obstacles;
    std::vector<object> _dynamic_obstacles;
    void update_obstacles();
    void update_fence_obstacles();
    void update_proximity_obstacles();

private:
  // trajectory cost module
  ComparableCost Calculate(const planning::QuinticPolynomialCurve1d &curve,
                           const float start_s, const float end_s,
                           const uint32_t curr_level,
                           const uint32_t total_level);

  ComparableCost CalculatePathCost(const planning::QuinticPolynomialCurve1d &curve,
                                   const float start_s, const float end_s,
                                   const uint32_t curr_level,
                                   const uint32_t total_level);

  ComparableCost CalculateStaticObstacleCost(const planning::QuinticPolynomialCurve1d &curve, 
                                             const float start_s,
                                             const float end_s);
      
  ComparableCost CalculateDynamicObstacleCost(const planning::QuinticPolynomialCurve1d &curve, 
                                              const float start_s,
                                              const float end_s);
  
  bool IsOffRoad(const float ref_s, const float l, const float dl);

  ComparableCost GetCostFromObsSL(const float adc_s,
                                  const float adc_l,
                                  const object & obstacle);

  ComparableCost GetCostBetweenObsBoxes(const float adc_s, 
                                        const float adc_l, 
                                        const object &obstacle, 
                                        const float time_stamp);


      
private:
  // DP module
  struct DpRoadGraphNode {
    public:
      DpRoadGraphNode() = default;

      DpRoadGraphNode(const planning::Vec2d point_sl,
                      const DpRoadGraphNode *node_prev)
          : sl_point(point_sl), min_cost_prev_node(node_prev) {}

      DpRoadGraphNode(const planning::Vec2d point_sl,
                      const DpRoadGraphNode *node_prev,
                      const ComparableCost &cost)
          : sl_point(point_sl), min_cost_prev_node(node_prev), min_cost(cost) {}

      void UpdateCost(const DpRoadGraphNode *node_prev,
                      const planning::QuinticPolynomialCurve1d &curve,
                      const ComparableCost &cost) {
        if (cost <= min_cost) {
          min_cost = cost;
          min_cost_prev_node = node_prev;
          min_cost_curve = curve;
        }
    }

    // node{sl_point, min_cost, curve, prev_node_ptr}
    planning::Vec2d sl_point;
    const DpRoadGraphNode *min_cost_prev_node = nullptr;
    ComparableCost min_cost = {true, true, true,
                               std::numeric_limits<float>::infinity(),
                               std::numeric_limits<float>::infinity()};
    planning::QuinticPolynomialCurve1d min_cost_curve;
  };

  bool generate_min_cost_path(std::vector<DpRoadGraphNode> *min_cost_path);


  bool is_valid_curve(const planning::QuinticPolynomialCurve1d &curve) const;

private:
  // Waypoint Sampler
  bool SamplePathWaypoints(std::vector<std::vector<planning::Vec2d>> *const points);

private:
    // common parameters
    float _safe_margin;  
    
    /// vehicle parameters
    AP_Int8 _enable;                                         // enable or disable algorithm
    AP_Float _vehicle_radius;                                // vehicle radius

    /// reference parameters
    AP_Float _reference_left_bound;                          // reference center line left range
    AP_Float _reference_right_bound;                         // reference center line right range
    AP_Float _path_resolution;                               // path resolution

    /// waypoint sampler
    AP_Float _sample_points_num_each_level;                  // default 32
    AP_Float _step_length_min;
    AP_Float _step_length_max;

    /// cost computing
    AP_Float _eval_time_interval;
    AP_Float _path_l_cost;
    AP_Float _path_dl_cost;
    AP_Float _path_ddl_cost;
    AP_Float _obstacle_collision_cost;                     // obstacle collision cost 
   

private:
  // useful variables
  planning::Vec2d _init_point_sl;
  float _init_v;
  planning::SpeedData _heuristic_speed_data;

};





