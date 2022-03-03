#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AP_Logger/AP_Logger.h>

class AC_ADRC{
    public:
       // Constructor for ADRC
        AC_ADRC(float initial_wc,float initial_wo,float initial_b0,float initial_delta,float intiial_order,float dt);
        virtual ~AC_ADRC() = default;

        CLASS_NO_COPY(AC_ADRC);

        //  update_all - set target and measured inputs to ADRC controller and calculate outputs
        //  target and error are filtered
        float update_all(float target,float measurement);

        // set time step in seconds
        void set_dt(float dt);

        // reset ESO
        void reset_eso(float measurement);

        const AP_Logger::PID_Info& get_debug_info(void) const { return _debug_info; }

        // parameter var table
         static const struct AP_Param::GroupInfo var_info[];

    private:

        // parameters
        AP_Float wc_;          // Response bandwidth in rad/s
        AP_Float wo_;          // State estimation bandwidth in rad/s
        AP_Float b0_;          // Control gain
        AP_Float limit_;
        AP_Float delta_;
        AP_Int8  order_;
  
        // internal varibales
        float dt_;                // timestep in seconds

        // ESO interal variables
        float z1_;
        float z2_;
        float z3_;
     
        AP_Logger::PID_Info _debug_info;
  
};