#include "steering.h"

#if AP_SCU_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AP_SCU_Steering::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Engine control
    // @Description: This enables internal combustion engine control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_SCU_Steering, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _POSZ
    // @DisplayName: Position controller P gain
    // @Description: Positioncontroller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POSZ", 1, AP_SCU_Steering, AC_P_1D),

    // @Param: CTL_TYPE
    // @DisplayName: 
    // @Description: 
    // @Range: 
    // @User: Standard
    AP_GROUPINFO("CTL_TYPE", 2, AP_SCU_Steering, control_type, STEER_BANG_BANG_CTL),


	// @Param: CTL_SGN
    // @DisplayName: 
    // @Description: 
    // @Range: 
    // @User: Standard
    AP_GROUPINFO("CTL_SGN", 3, AP_SCU_Steering, steer_ctl_sign, 1),


	// @Param: ANG_MAX
    // @DisplayName: 
    // @Description: 
    // @Range:
    // @User: Standard
	AP_GROUPINFO("ANG_MAX", 4, AP_SCU_Steering, steer_max_angle, 35),


	// @Param: LEF_POS
    // @DisplayName: 
    // @Description: 
    // @Range: 
    // @User: Standard
	AP_GROUPINFO("LEF_POS", 5, AP_SCU_Steering, steer_L_sensor_fdb, 1465.0f),


	// @Param: RIG_POS
    // @DisplayName: 
    // @Description: 
    // @Range: 
    // @User: Standard
	AP_GROUPINFO("RIG_POS", 6, AP_SCU_Steering, steer_R_sensor_fdb, 235.0f),


	// @Param: MID_POS
    // @DisplayName: 
    // @Description: 
    // @Range: 
    // @User: Standard
	AP_GROUPINFO("MID_POS", 7, AP_SCU_Steering, steer_N_sensor_fdb, 890.0f),


    // @Param: CTL_DZ
    // @DisplayName: 
    // @Description: 
    // @Range: 
    // @User: Standard
	AP_GROUPINFO("CTL_DZ", 8, AP_SCU_Steering, steer_ctl_dz, 30.0f),

    // @Param: VEL_MAX
    // @DisplayName:
    // @Description:
    // @User: Standard
	AP_GROUPINFO("VEL_MAX", 9, AP_SCU_Steering, steer_out_vel_max, 0.5f),

    // @Param: ACC_MAX
    // @DisplayName: 
    // @Description: 
    // @Range: 
    // @User: Standard
	AP_GROUPINFO("ACC_MAX", 10, AP_SCU_Steering, steer_out_acc_max, 0.5f),

    AP_GROUPEND
};

// constructor
AP_SCU_Steering::AP_SCU_Steering():_p_pos(1.0)
{
    AP_Param::setup_object_defaults(this, var_info);
    init();
	
}

// one time init call
void AP_SCU_Steering::init()
{
    memset(&_steering_run_state, 0, sizeof(_steering_run_state));

    // define maximum position error and maximum first and second differential limits
    _p_pos.set_limits(-fabsf(steer_out_vel_max), steer_out_vel_max, steer_out_acc_max, 0.0f);
}

/**
 * @breif: Steer control,steer_cmd with [-1,+1];
 * @return servo motor output
 **/
float AP_SCU_Steering::steering_angle_control(const float steer_cmd, const float steer_pos_ad)
{
    /* get motor output */
	float servo_cmd = (steer_cmd >= 0)?
                        (linear_interpolate(steer_N_sensor_fdb,
                                            steer_R_sensor_fdb,
		                                    steer_cmd,
											0.0f,
                                            1.0f)):
			            (linear_interpolate(steer_L_sensor_fdb,
                                            steer_N_sensor_fdb,
                                            steer_cmd,
											-1.0f,
                                            0.0f));

	float servo_out = 0;
	float ctl_err = 0;

	float steer_sensor_min =  MIN(steer_L_sensor_fdb, steer_R_sensor_fdb) - 2.0f * steer_ctl_dz;
	float steer_sensor_max =  MAX(steer_L_sensor_fdb, steer_R_sensor_fdb) + 2.0f * steer_ctl_dz;

	bool ret = (steer_pos_ad < steer_sensor_min && steer_pos_ad > steer_sensor_max)?(true):(false);
	if (ret) {
		GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_SCU-Steering: steering position overlimit to %.1f", steer_pos_ad);
	}
 
	switch(control_type)
	{
		case STEER_BANG_BANG_CTL:
        {
			ctl_err = steer_ctl_sign * (servo_cmd - steer_pos_ad);
			if(abs(ctl_err) <= steer_ctl_dz)
			{
				servo_out = 0;
			} else {
			  servo_out = (ctl_err < 0)?(-1.0f):(1.0f);
			}
        }
		break;

		case STEER_OPEN_CTL:
        {
            ctl_err = sign(steer_cmd);
            if(is_zero(ctl_err))
            {
                servo_out = 0.0f;
            } else {
                servo_out = (ctl_err < 0)?(-1):(1);
            }
        }

		break;
		case STEER_PID_CTL:
        {
            servo_out = steer_ctl_sign * _p_pos.update_all(servo_cmd, steer_pos_ad);
            servo_out = constrain_float(servo_out, -1.0f, 1.0f);
            if(abs(ctl_err) <= steer_ctl_dz)
			{
				servo_out = 0.0f;
			} 
        }
        
		break;
		default:
			servo_out = 0.0f;
		break;
	}


	/* update steer angle */
	steering_update_angle(steer_pos_ad);

	/* log */
	_steering_run_state.steer_cmd_in_log = steer_cmd;
	_steering_run_state.steer_cmd_out_log = servo_out;

	/* return motor input value */
	return servo_out;
}

/*
* @brief: compute current steer angle value
* @retval: return  current steer angle and register value
**/
void AP_SCU_Steering::steering_update_angle(const float steer_pos_ad)
{
    float steer_angle_deg = 0;
	const float steer_left_max_angle = -(steer_max_angle);
	const float steer_right_max_angle = +(steer_max_angle);

	if(steer_L_sensor_fdb < steer_R_sensor_fdb) {

		if(steer_pos_ad <= steer_N_sensor_fdb) {
            steer_angle_deg = linear_interpolate(steer_left_max_angle,
                                                    0.0f,
                                                steer_pos_ad,
                                                steer_L_sensor_fdb,
                                                steer_N_sensor_fdb);
		} else {
		    steer_angle_deg = linear_interpolate(0.0f,
                                                steer_right_max_angle,
						                        steer_pos_ad,
												steer_N_sensor_fdb,
                                                steer_R_sensor_fdb);
		}
	} else {
		if(steer_pos_ad <= steer_N_sensor_fdb) {
		    steer_angle_deg = linear_interpolate(steer_right_max_angle,
                                                0.0f,
											    steer_pos_ad,
											    steer_R_sensor_fdb,
                                                steer_N_sensor_fdb);
		} else {
		    steer_angle_deg = linear_interpolate(0.0f,
                                                steer_left_max_angle,
												steer_pos_ad,
												steer_N_sensor_fdb,
                                                steer_L_sensor_fdb);
		}
	}

	/* update current steer angle */
	_steering_run_state.steer_angle_deg = steer_angle_deg;
	_steering_run_state.steer_angle_reg = steer_pos_ad;
}



#endif