
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Param/AP_Param.h>
#include <vector>

#define OA_SHORELINE_DEBUG_ENABLE 0

class AP_ShorelineAvoid{
    public:
        AP_ShorelineAvoid();

        virtual ~AP_ShorelineAvoid() = default;

         /* Do not allow copies */
        CLASS_NO_COPY(AP_ShorelineAvoid);

        // Get singleton instance
        static AP_ShorelineAvoid *get_singleton(){
            return _singleton;
        }

        // Paramter group info
        static const struct AP_Param::GroupInfo var_info[];

        // Run background task to find shoreline and update avoidance result
        // Return true if shoreline detected and can not reach destination 
        // Return false if destination could arrivalable temporarily.
        bool update(const Location &current_loc,const Location& origin,const Location& destination);

#if OA_SHORELINE_DEBUG_ENABLE
         void send_debug_info(mavlink_channel_t chan, uint16_t interval_ms);
#endif

    private:
         static AP_ShorelineAvoid *_singleton;

         // Parameters
         AP_Float _shoreline_link_dist;
         AP_Float _shoreline_min_length;
         AP_Float _shoreline_safe_dist;
         AP_Float _min_radius;
         AP_Float _shoreline_scan_max_angle;
         AP_Float _shollow_min_depth;
         AP_Float _shollow_move_dist;

         // Common variables
         Location _origin_prev;
         Location _destination_prev;
         Vector2f _intersect_point;
         bool     _mission_lock{false};
         std::vector<Vector3f> _obstacle_set;
         std::vector<Vector3f> _boundary_set;
         std::vector<std::pair<uint32_t, std::vector<Vector2f>>> _shoreline_set;

         Vector2f _sonar_avoid_loc;
         bool _close_to_shoreline{false};
         uint32_t _last_update_ms;       // system time of last call to update
         bool     _last_avoid_flag{false};

         // Common functions
        bool is_active() const;
        float calc_shoreline_len(const std::vector<Vector2f> &lines) const;
        bool update_avoidance(const Vector2f& current_ne,const Vector2f& origin_ne,const Vector2f& destination_ne) WARN_IF_UNUSED;
        bool update_shoreline(const Vector2f& current_ne,const Vector2f& destination_ne) WARN_IF_UNUSED;
};

namespace AP {
    AP_ShorelineAvoid *shoreline_avoid();
};