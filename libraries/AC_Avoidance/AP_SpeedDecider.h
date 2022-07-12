#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include <tuple>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>
#include <memory>

#include <AP_Planning/st_graph_point.h>
#include <AP_Planning/st_point.h>
#include <AP_Planning/st_boundary.h>
#include <AP_Planning/speed_data.h>

// An algorithm for dynamic obstacle avoidance by adjusting the speed in straight-line cruise. 
// Its biggest advantage is that it does not allow reversing and considers kinematic constraints. 
// More generally, it can be extended to EM path planning algorithm. 
// Its disadvantage is that it is unable to plan the dynamic obstacle avoidance problem when turning.
// Input: start point,goal point,current vehicle state
// Output: desired cruise speed modified,obstacle avoidance requirements 

namespace {
    using STBoundPoint = std::tuple<float, float, float>;
    using STBound = std::vector<STBoundPoint>;
}

class AP_SpeedDecider{
public:
    AP_SpeedDecider();

    /* Do not allow copies */
    AP_SpeedDecider(const AP_SpeedDecider &other) = delete;
    AP_SpeedDecider &operator=(const AP_SpeedDecider) = delete;

    // send configuration info stored in front end parameters
    void set_config(float margin_max) { _margin_max = MAX(margin_max, 0.0f); }

    // run background task to find best cruise speed and update avoidance results
    // returns false if obstacle avoidance is not required
    bool update(const Location &current_loc, const Location& origin, const Location& destination,
                const Vector2f &ground_speed_vec, float &desired_speed_new, const float dt);

    // paramters table
    static const struct AP_Param::GroupInfo var_info[];

private:
    // given time t and speed, calculate the driving limits in s due to vehicle's dynamics
    void get_vehicle_dynamics_limits(const float t, const float speed, float& low_st, float& upper_st);

    // calculate reference line t-s point
    float get_reference_s(const float t, const float speed, const float desired_speed);
    
    // geneate drivable st boundary
    void genate_drivable_boundary(const float current_speed, STBound& st_bound);

    // generate obstacle st boundary
    bool compute_obstacle_st_boundary(const Vector2f& vehicle_start, const Vector2f& vehicle_end,
                                      const Vector2f& obs_pos, const Vector2f& obs_vel, const float radius,
                                      std::vector<planning::STPoint>& lower_points,
                                      std::vector<planning::STPoint>& upper_points);
    void update_obstacle_st_boundary();

private:
    // DP cost functions
    float get_obstacle_cost(const planning::StGraphPoint& point);

    float get_spatial_potential_cost(const planning::StGraphPoint& point);

    float get_reference_cost(const planning::STPoint& point, const planning::STPoint& reference_point) const;

    float get_speed_cost(const planning::STPoint& first, const planning::STPoint& second,
                         const float speed_limit,
                         const float cruise_speed) const;
    
    float get_accel_cost_by_two_points(const float pre_speed,
                                       const planning::STPoint& first,
                                       const planning::STPoint& second);
    
    float get_accel_cost_by_three_points(const planning::STPoint& first, const planning::STPoint& second,
                                         const planning::STPoint& third);

    float get_jerk_cost_by_two_points(const float pre_speed, const float pre_acc,
                                      const planning::STPoint& pre_point,
                                      const planning::STPoint& curr_point);

    float get_jerk_cost_by_three_points(const float first_speed,
                                        const planning::STPoint& first_point,
                                        const planning::STPoint& second_point,
                                        const planning::STPoint& third_point);

    float get_jerk_cost_by_four_points(const planning::STPoint& first, const planning::STPoint& second,
                                       const planning::STPoint& third, const planning::STPoint& fourth);
    
    float get_accel_cost(const float accel);

    float jerk_cost(const float jerk);
    
private:
    // gridded s-t graph optimizers
    bool search(planning::SpeedData* const speed_data);

    bool init_cost_table();

    bool init_speed_limit_lookup();

    bool retrieve_speed_profile(planning::SpeedData* const speed_data);

    bool calculate_total_cost();

    // defined for  task
    struct StGraphMessage {
        StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
        uint32_t c;
        uint32_t r;
    };

    void calculate_cost_at(const std::shared_ptr<StGraphMessage>& msg);

    float calculate_edge_cost(const planning::STPoint& first, const planning::STPoint& second,
                              const planning::STPoint& third, const planning::STPoint& forth,
                              const float speed_limit, const float cruise_speed);

    float calculate_edge_cost_for_second_col(const uint32_t row,
                                             const float speed_limit,
                                             const float cruise_speed);

    float calculate_edge_cost_for_third_col(const uint32_t curr_row,
                                            const uint32_t pre_row,
                                            const float speed_limit,
                                            const float cruise_speed);

    // get the row-range of next time step
    void get_row_range(const planning::StGraphPoint& point, 
                       size_t* next_highest_row,
                       size_t* next_lowest_row);

    // Continuous-time collision check using linear interpolation as closed-loop
    // dynamics
    bool check_overlap_on_dp_st_graph(const std::vector<planning::STBoundary>& boundaries,
                                      const planning::StGraphPoint& p1, 
                                      const planning::StGraphPoint& p2);

private:
    // parameters
    AP_Int8 _enable;             // enable or disable alogorithm
    AP_Float _acc_max;           // acceleration maximumum value ,set must >= 0
    AP_Float _dec_max;           // deceleration maximum value, set must >= 0
    AP_Float _speed_max;         // speed maximum value
    AP_Float _vehicle_radius;    // vehicle shape size
    AP_Float _cruise_speed;      // vehicle cruise speed
    AP_Float _total_t;           // planning time with unit seconds
    AP_Float _total_s;           // planning distance with unit meters
    AP_Float _obs_pred_t;        // obstacle trajectory prediction time with seconds
    AP_Float _obstacle_weight;   // avoidance obstacle collision penalty factor
    AP_Float _accel_penalty;     // acceleration variation penalty
    AP_Float _decel_penalty;     // deceleration variation penalty
    AP_Float _unit_t;            // time scale
    AP_Float _dense_unit_s;      // dense distance scale
    AP_Float _sparse_unit_s;     // sparse distance scale
    AP_Float _planning_length_min; 

    float _planning_length;
    float _margin_max;           // avoidance margin with meters
    bool _use_st_drivable_boundary{false};  // use drivable boundary 
    bool _enable_dp_reference_speed{true};  // enable reference speed deviation penalty 
private:
    std::vector<planning::STBoundary> _st_boundaries;  // ST graph 
    std::array<float, 200> _accel_cost;                // acceleration cost list
    std::array<float, 400> _jerk_cost;                 // deceleration cost list
    
    std::vector<float> _speed_limit_by_index;
    std::vector<float> _spatial_distance_by_index;

    // ST graph dimension
    uint32_t _dimension_t{0};
    uint32_t _dense_dimension_s{101};
    uint32_t _sparse_dimension_s{0};
    uint32_t _dimension_s{0};

    // cost_table_[t][s]
    // row: s, col: t --- NOTICE: Please do NOT change.
    std::vector<std::vector<planning::StGraphPoint>> _cost_table;
private:
    // store current state
    float    _planning_period;  // planing module period with seconds
    float    _curr_speed;       // planning module init speed projected reference
    Vector2f _curr_start;       // planning module start position projected reference
    Vector2f _curr_end;         // planning module end position projected reference
};