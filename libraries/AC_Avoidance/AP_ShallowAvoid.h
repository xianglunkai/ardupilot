#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <deque>
#include <algorithm>

class AP_ShallowAvoid{
    public:
        AP_ShallowAvoid();
        virtual ~AP_ShallowAvoid() = default;

        /* Do not allow copies */
        CLASS_NO_COPY(AP_ShallowAvoid);

        // Get singleton instance
        static AP_ShallowAvoid *get_singleton(){
            return _singleton;
        }

        // Paramter group info
        static const struct AP_Param::GroupInfo var_info[];

        // Return true if shallows detected and can not reach destination 
        // Return false if destination could arrivalable temporarily.
        bool update(const Location &current_loc, const Location& origin, const Location& destination, const Vector2f &ground_speed_vec, const float dt) WARN_IF_UNUSED;
    
    private:
        static AP_ShallowAvoid *_singleton;

        // true if update has been called recently
        bool is_active() const;

        // Common paramters
        AP_Int16 _sample_distance;
        AP_Int16 _predict_distance;
        AP_Float _max_lean_angle;
        AP_Float _min_water_depth;
        AP_Float _min_water_radius;
        AP_Float _min_water_slope;

        // Common variables
        std::deque<float> _sample_points;
        uint32_t _last_update_ms;       // system time of last call to update
        bool _last_avoid_flag{false};
};

namespace AP {
    AP_ShallowAvoid *shallow_avoid();
};