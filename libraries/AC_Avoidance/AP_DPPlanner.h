#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <tuple>
#include <limits>
#include <cmath>
#include <memory>
#include <AP_Planning/vec2d.h>
#include <AP_Planning/discretized_traj.h>
#include <AP_Planning/box2d.h>
#include <AP_Planning/pose.h>
#include <AP_Planning/polygon2d.h>

class AP_DPPlanner {
public:
    AP_DPPlanner();

    CLASS_NO_COPY(AP_DPPlanner);  /* Do not allow copies */

    // send configuration info stored in front end parameters
    void set_config(float margin_max) { _safe_margin = MAX(margin_max, 0.0f); }

    // run background task to find best cruise speed and update avoidance results
    // returns false if obstacle avoidance is not required
    bool update(const Location &current_loc, const Location& origin, const Location& destination, const Vector2f &ground_speed_vec, 
                Location &origin_new, Location &destination_new, float &desired_speed_new, 
                bool proximity_only);

    // paramters table
    static const struct AP_Param::GroupInfo var_info[];

private:
    // define some constance
    static const int NT = 5;
    static const int NS = 7;
    static const int NL = 10;

    // state node content
    struct StateCell {
        float cost = std::numeric_limits<float>::max();
        float current_s = std::numeric_limits<float>::min();
        int16_t parent_s_ind = -1;
        int16_t parent_l_ind = -1;

        StateCell() = default;

        StateCell(double cost0, double cur_s0, int parent_s_ind0, int parent_l_ind0): 
                 cost(cost0), current_s(cur_s0),
                 parent_s_ind(parent_s_ind0), parent_l_ind(parent_l_ind0) {}
    };

    // state node index
    struct StateIndex {
        int16_t t = -1, s = -1, l = -1;
        
        StateIndex() = default;

        StateIndex(int tt, int ss, int ll) : t(tt), s(ss), l(ll) {}
    };

    // planning start node
    struct StartState {
        float start_s = 0;
        float start_l = 0;
        float start_theta = 0;
    };

    // segment number
    int16_t _nseg;
    
    // segment unit (s)
    float _unit_time;

    // SLT search nodes
    std::array<float, NT> _time; 
    std::array<float, NS> _station;
    std::array<float, NL-1> _lateral;

    // planning initial node
    StartState _state;
    StateCell _state_space[NT][NS][NL];

    // get collision cost
    float get_collision_cost(StateIndex parent_ind, StateIndex current_ind);

    // get total cost
    std::pair<float, float> get_total_cost(StateIndex parent_ind, StateIndex cur_ind);

    // get lateral offset
    float get_lateral_offset(const float s, const int16_t l_ind);

    // linear polate
    std::vector<planning::Vec2d> interpolate_linearly(const float parent_s, const int16_t parent_l_ind, const int16_t current_s_ind, const int16_t current_l_ind);

private:

    // OA common parameters
    float _safe_margin;                                     // object avoidance will ignore objects more than this many meters from vehicle
       
    // DP parameters
    AP_Int8 _enable;                                         // enable or disable algorithm
    AP_Float _nfe;                                           // number of finite elements used to descretize an OCP
    AP_Float _tf;                                            // time horizon length
    AP_Float _speed_max;                                     // max speed
    AP_Float _dcceleration_max;                              // must set > 0
    AP_Float _acceleration_max;                              // must set > 0
    AP_Float _jerk_max;                                      // M/S^3
    AP_Float _vehicle_length;                                // vehicle length
    AP_Float _vehicle_width;                                 // vehicle width
    AP_Float _obstacle_weight;                               // cost of obstacles, should be set larger to avoid collision with obstacles
    AP_Float _lateral_weight;                                // lateral cost, the larger the trajectory the closer to reference line
    AP_Float _lateral_change_weight;                         // lateral change cost dl/ds, penalty for lateral change
    AP_Float _lateral_vel_change_weight;                     // lateral change cost, dl/dt, penalty for sudden lateral change
    AP_Float _longitudinal_vel_weight;                       // longitudinal velocity cost, velocity to the nominal velocity
    AP_Float _longitudinal_vel_change_weight;                // Cost of longitudinal velocity change, ds/dt
    AP_Float _reference_left_bound;                          // reference center line left range
    AP_Float _reference_right_bound;                         // reference center line right range
    AP_Float _reference_resolution;                          // reference center line resolution with meter

    // internal variables used by background thread

private:
    // reference line
    planning::DiscretizedTraj _reference;
    std::vector<planning::Vec2d> _road_barrier;
    Vector2f _start;
    Vector2f _end;
    float _desired_speed_input;
    void set_reference(const planning::DiscretizedTraj &reference);

private:
    // enviroment
    std::vector<planning::Polygon2d> _static_obstacles;
    std::vector<std::pair<planning::Vec2d, planning::Polygon2d>> _dynamic_obstacles;
    void update_obstacles(bool proximity_only);

    template<class T>
    std::tuple<T, T, T, T> get_disc_positions(const T &x, const T &y, const T &theta) const;
    bool check_station_collision(const planning::Box2d &rect);
    bool check_dynamical_collision(const float time, const planning::Box2d &rect);
    bool check_optimization_collision(const float time, const planning::Pose &pose);
};