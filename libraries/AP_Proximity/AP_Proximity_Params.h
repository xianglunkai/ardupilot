#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#define PROXIMITY_MAX_IGNORE                6   // up to six areas can be ignored

class AP_Proximity_Params{
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Proximity_Params(void);

    /* Do not allow copies */
    AP_Proximity_Params(const AP_Proximity_Params &other) = delete;
    AP_Proximity_Params &operator=(const AP_Proximity_Params&) = delete;

    AP_Int8  _type;
    AP_Int8  _orientation;
    AP_Int16 _yaw_correction;
    AP_Int16 _ignore_angle_deg[PROXIMITY_MAX_IGNORE];   // angle (in degrees) of area that should be ignored by sensor (i.e. leg shows up)
    AP_Int8 _ignore_width_deg[PROXIMITY_MAX_IGNORE];    // width of beam (in degrees) that should be ignored
    AP_Int8 _raw_log_enable;                            // enable logging raw distances
    AP_Int8 _ign_gnd_enable;                            // true if land detection should be enabled
    AP_Float _filt_freq;                                // cutoff frequency for low pass filter
    AP_Float _max_m;                                   // Proximity maximum range
    AP_Float _min_m;                                   // Proximity minimum range
};

