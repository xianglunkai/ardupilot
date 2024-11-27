#pragma once

#include "AP_SCU_Config.h"

#if AP_SCU_ENABLED
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_P_1D.h>


class AP_SCU_Steering {
public:
    // constructor
    AP_SCU_Steering();

    static const struct AP_Param::GroupInfo var_info[];

    // Engine start/stop control data
    enum SteeringCTLType {
        STEER_BANG_BANG_CTL = 0,
        STEER_OPEN_CTL,
        STEER_PID_CTL,
    };

  
    /* steer control run state */
    struct SteeringRunState{
        float steer_angle_deg;   /* current deg,resolution from acutor_cfg.h */
        float steer_angle_reg;   /* read from AD or uart etc */

        float steer_cmd_in_log;	
        float steer_cmd_out_log;	
    };

    // one time init call
    void init();

    /**
     * @breif: Steer control,steer_cmd with [-1,+1];
     * @return servo motor output
     **/
    float steering_angle_control(const float steer_cmd, const float steer_pos_ad);

    /*
    * @brief: compute current steer angle value
    * @retval: return  current steer angle and register value
    **/
    void steering_update_angle(const float steer_pos_ad);

    // get engine control state
    const SteeringRunState & steering_state(void) { return _steering_run_state; }

private:

    SteeringRunState _steering_run_state;

private:
    // parameters
    AP_Int8 control_type;

    AP_Int8 steer_ctl_sign;

    AP_Float steer_max_angle;
	AP_Float steer_L_sensor_fdb;
	AP_Float steer_R_sensor_fdb;
	AP_Float steer_N_sensor_fdb;

	AP_Float steer_ctl_dz;

    AP_Float kp;

};



#endif

