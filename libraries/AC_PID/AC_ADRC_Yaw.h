#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AP_Logger/AP_Logger.h>

class AC_ADRC_YAW{
    public:
        // Constructor for ADRC
        AC_ADRC_YAW();
        virtual ~AC_ADRC_YAW() = default;

        CLASS_NO_COPY(AC_ADRC_YAW);

        //  update_all - set target and measured inputs to ADRC controller and calculate outputs
        //  target and error are filtered
        float update_all(float target,float measurement);

        // set time step in seconds
        void set_dt(float dt);

        // Reset ESO
        void reset_eso(float measurement,float measurement_rate = 0.0f);

        // Reset filter
        void reset_filter(){
            flags_.reset_filter_ = true;
        }

        const AP_Logger::PID_Info& get_debug_info(void) const { return _debug_info; }

        // parameter var table
         static const struct AP_Param::GroupInfo var_info[];
    
    private:
        // Parameters
        AP_Float gama_;
        AP_Float wc_;
        AP_Float wo_;
        AP_Float b0_;
        AP_Float delta_;
        AP_Float bound_command_;
        AP_Float kesai_;
        AP_Float error_max_;

        // flags
        struct ar_adrc_flags {
            bool reset_filter_ :1; // true when input filter should be reset during next call to set_input
        } flags_;

        // internal variables
        float dt_{0.02f};
        float z1_{0.0f};
        float z2_{0.0f};
        float z3_{0.0f};

        float v1_{0.0f};
        float v2_{0.0f};


        AP_Logger::PID_Info _debug_info;
};