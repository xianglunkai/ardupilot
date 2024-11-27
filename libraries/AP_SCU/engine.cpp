#include "engine.h"

#if AP_SCU_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_SCU/AP_CANIO.h>
#include <AP_RPM/AP_RPM.h>

const AP_Param::GroupInfo AP_SCU_Engine::var_info[] = {


    // @Param: STARTER_TIME
    // @DisplayName: Time to run starter
    // @Description: This is the number of seconds to run the starter when trying to start the engine
    // @User: Standard
    // @Units: s
    // @Range: 0.1 5
    AP_GROUPINFO("ST_TM", 1, AP_SCU_Engine, starter_time, 3),

    // @Param: START_DELAY
    // @DisplayName: Time to wait between starts
    // @Description: Delay between start attempts
    // @User: Standard
    // @Units: s
    // @Range: 1 10
    AP_GROUPINFO("ST_DEY", 2, AP_SCU_Engine, starter_delay, 2),


#if AP_RPM_ENABLED
    // @Param: RPM_THRESH
    // @DisplayName: RPM threshold
    // @Description: This is the measured RPM above which the engine is considered to be running
    // @User: Standard
    // @Range: 100 100000
    AP_GROUPINFO("RPM_THR", 3, AP_SCU_Engine, gear_shift_engine_speed, 100),

    // @Param: RPM_CHAN
    // @DisplayName: RPM instance channel to use
    // @Description: This is which of the RPM instances to use for detecting the RPM of the engine
    // @User: Standard
    // @Values: 0:None,1:RPM1,2:RPM2
    AP_GROUPINFO("RPM_CHN",  4, AP_SCU_Engine, rpm_instance, 0),
#endif

    // @Param: IDLE_PCT
    // @DisplayName: Throttle percentage for engine idle
    // @Description: This is the minimum percentage throttle output while running, this includes being disarmed, but not safe
    // @User: Standard
    // @Range: 0 100
    AP_GROUPINFO("IDL_PCT", 5, AP_SCU_Engine, idle_percent, 10),


    // @Param: 
    // @DisplayName:
    // @Description: 
    // @User: Standard
    AP_GROUPINFO("FWD_POS", 6, AP_SCU_Engine, eng_forward_max_thrust_out, 1000),


    // @Param: 
    // @DisplayName:
    // @Description: 
    // @User: Standard
    AP_GROUPINFO("BCK_POS", 7, AP_SCU_Engine, eng_backward_max_thrust_out, -1000),

    // @Param: 
    // @DisplayName:
    // @Description: 
    // @User: Standard
    AP_GROUPINFO("IDL_POS", 8, AP_SCU_Engine, eng_idle_thrust_out, 0),


    // @Param: 
    // @DisplayName:
    // @Description: 
    // @User: Standard
    AP_GROUPINFO("HAT_POS", 9, AP_SCU_Engine, eng_halt_thrust_out, 0),

    // @Param: 
    // @DisplayName:
    // @Description: 
    // @User: Standard
    AP_GROUPINFO("GEAR_D", 10, AP_SCU_Engine, gear_D_thrust_out, 300),

    // @Param: 
    // @DisplayName:
    // @Description: 
    // @User: Standard
    AP_GROUPINFO("GEAR_N", 11, AP_SCU_Engine, gear_N_thrust_out, 0),

    // @Param: 
    // @DisplayName:
    // @Description: 
    // @User: Standard
    AP_GROUPINFO("GEAR_R", 12, AP_SCU_Engine, gear_R_thrust_out, -300),

    // @Param: 
    // @DisplayName:
    // @Description: 
    // @User: Standard
    AP_GROUPINFO("GEAR_TM", 13, AP_SCU_Engine, gear_shift_block_ms, 200),

    AP_GROUPEND
};


AP_SCU_Engine::AP_SCU_Engine()
{
    AP_Param::setup_object_defaults(this, var_info);

    #if AP_RPM_ENABLED
        // Engine runs at 10Hz
        _rpm_filter.set_cutoff_frequency(10, 0.5f);
    #endif
}

void AP_SCU_Engine::set_acc(bool on)
{
#if AP_RELAY_ENABLED
    AP_Relay *relay = AP::relay();
    if (relay != nullptr) {
        relay->set(AP_Relay_Params::FUNCTION::ENG_ACC, on);
    }
#endif // AP_RELAY_ENABLED


#if AP_CANIO_ENABLE
    AP_CANIO *can_io = AP::can_io();
    if (can_io != nullptr) {
        can_io->set(AP_CANIO_Params::FUNCTION::ENG_ACC, on);
    }
#endif // AP_RELAY_ENABLED

}

void AP_SCU_Engine::set_start(bool on)
{
#if AP_RELAY_ENABLED
    AP_Relay *relay = AP::relay();
    if (relay != nullptr) {
        relay->set(AP_Relay_Params::FUNCTION::ENG_START, on);
    }
#endif // AP_RELAY_ENABLED


#if AP_CANIO_ENABLE
    AP_CANIO *can_io = AP::can_io();
    if (can_io != nullptr) {
        can_io->set(AP_CANIO_Params::FUNCTION::ENG_START, on);
    }
#endif // AP_CANIO_ENABLED
}

void AP_SCU_Engine::set_stop(bool on)
{
#if AP_RELAY_ENABLED
    AP_Relay *relay = AP::relay();
    if (relay != nullptr) {
        relay->set(AP_Relay_Params::FUNCTION::ENG_STOP, on);
    }
#endif // AP_CANIO_ENABLED


#if AP_CANIO_ENABLE
    AP_CANIO *can_io = AP::can_io();
    if (can_io != nullptr) {
        can_io->set(AP_CANIO_Params::FUNCTION::ENG_STOP, on);
    }
#endif // AP_RELAY_ENABLED

}

// one time init call
void AP_SCU_Engine::init()
{
    memset(&_eng_run_state, 0, sizeof(_eng_run_state));
    memset(&_eng_sts_ctrl, 0, sizeof(_eng_sts_ctrl));

    set_acc(true);
    set_start(false);
    set_stop(false);
}

// STS control
void AP_SCU_Engine::eng_armed_control(const uint16_t sts_cmd, const uint16_t ems, const uint16_t dt_ms)
{
    _eng_run_state.ems_state = false;
    if (ems == 1) {
        _eng_run_state.ems_state = true;
        return;
    }

    _eng_sts_ctrl.sts_cmd = sts_cmd;
	if(_eng_sts_ctrl.last_sts_cmd == 0&& _eng_sts_ctrl.sts_cmd == 1) {
		_eng_sts_ctrl.sts_event_cmd = REQ_START;
	}
	else if (_eng_sts_ctrl.last_sts_cmd == 0&& _eng_sts_ctrl.sts_cmd == 2) {
		_eng_sts_ctrl.sts_event_cmd = REQ_STOP;
	}
	else{};
	_eng_sts_ctrl.last_sts_cmd  = 	_eng_sts_ctrl.sts_cmd;


	if(	_eng_sts_ctrl.sts_lock_flag == 0){
		if(_eng_sts_ctrl.sts_event_cmd == REQ_START){
			if(_eng_sts_ctrl.sts_delay_cnt >= starter_time/dt_ms){
				// The motor have been armed!
			    _eng_run_state.armed_state = true;
				// Store current state
				_eng_sts_ctrl.sts_ctrl_state = START_RESET;
				_eng_sts_ctrl.sts_delay_cnt = 0;
				// LOCK command execute for a time
				_eng_sts_ctrl.sts_lock_flag = 1;
				// Reset operation
				_eng_sts_ctrl.sts_event_cmd     = NO_OPS;
			}
			else{
				_eng_sts_ctrl.sts_ctrl_state = STARTING;
				_eng_sts_ctrl.sts_delay_cnt++;
			}
		}
		else if(_eng_sts_ctrl.sts_event_cmd == REQ_STOP){
			if(_eng_sts_ctrl.sts_delay_cnt >= starter_time/dt_ms){

				// motor have been closed!
			    _eng_run_state.armed_state = false;;

				_eng_sts_ctrl.sts_ctrl_state = STOP_RESET;
				_eng_sts_ctrl.sts_delay_cnt = 0;
				// LOCK command execute for a time
				_eng_sts_ctrl.sts_lock_flag = 1;
				// Reset operation
				_eng_sts_ctrl.sts_event_cmd     = NO_OPS;
			}
			else{
				_eng_sts_ctrl.sts_ctrl_state = STOPING;
				_eng_sts_ctrl.sts_delay_cnt++;
			}
		}
		else{
			_eng_sts_ctrl.sts_event_cmd     = NO_OPS;
		}
	}
	else{
		if(++_eng_sts_ctrl.sts_delay_cnt >= starter_delay/dt_ms){
			_eng_sts_ctrl.sts_delay_cnt = 0;
			_eng_sts_ctrl.sts_lock_flag = 0;
		}
	}


    #if AP_RPM_ENABLED
        // get speed
        AP_RPM *ap_rpm = AP::rpm();
        if (!ap_rpm || rpm_instance == 0 || !ap_rpm->healthy(rpm_instance-1)) {
            return;
        }

        // get current RPM feedback
        float rpm_value;

        // Double Check to make sure engine is really running
        if (!ap_rpm->get_rpm(rpm_instance-1, rpm_value) || rpm_value < 1) {
            // Reset idle point to the default value when the engine is stopped
            rpm_value = 0.0f;
        }

        filtered_rpm_value =  _rpm_filter.apply(rpm_value);
    #endif
}


// throttle control
int16_t AP_SCU_Engine::eng_throttle_control(const float thr_cmd)
{
    float throttle     = thr_cmd;

    /* Initialize throttle output ensure halt engine */
	int16_t throttle_out =  eng_halt_thrust_out;
    const float idle_trottle = idle_percent * 0.01f;

	/* According to engine control state */
	switch (_eng_sts_ctrl.sts_ctrl_state) {
	case DEFAULT_RESET:
        {
            throttle  = 0;
            throttle_out = eng_halt_thrust_out;
       
            set_start(false);
            set_stop(false);
        }

		break;

	case STOP_RESET:
        {
            throttle =0;
            throttle_out = eng_halt_thrust_out;

            set_start(false);
            set_stop(false);
        }

		break;

	case START_RESET:
        {
            if (_eng_run_state.armed_state == false) {
                    throttle_out =  eng_halt_thrust_out;
                } else {
                        if(throttle >= 0)
                        {
                            if(throttle <= idle_trottle)
                            {
                                throttle_out = linear_interpolate(eng_idle_thrust_out,
                                                                  gear_D_thrust_out,
                                                                  throttle,
                                                                  0.0f,
                                                                  idle_trottle);
                            }
                            else
                            {
                                throttle_out = linear_interpolate(gear_D_thrust_out,
                                                                  eng_forward_max_thrust_out,
                                                                  throttle,
                                                                  idle_trottle,
                                                                  1.0f);
                            }

                        }
                        else
                        {
                            if(throttle >= -idle_trottle)
                            {
                                throttle_out = linear_interpolate(gear_R_thrust_out,
                                                                  eng_idle_thrust_out,
                                                                  throttle,
                                                                  -idle_trottle,
                                                                  0.0f);
                            }
                            else
                            {
                                throttle_out = linear_interpolate(eng_backward_max_thrust_out,
                                                                  gear_R_thrust_out,
                                                                  throttle,
                                                                  -1.0f,
                                                                  -idle_trottle);
                            }

                        }
                
                }

                set_start(false);
                set_stop(false);
        }
	
		break;

	case STOPING:
		throttle =0;
		throttle_out = eng_halt_thrust_out;

        set_start(false);
        set_stop(true);

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_SCU-Engine: Stopping!");
		break;

	case STARTING:
		throttle =0;
		throttle_out =  eng_idle_thrust_out;

        set_start(true);
        set_stop(false);

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_SCU-Engine: Starting!");
		break;

	 default:
		break;
	}


	if(_eng_run_state.ems_state == true)
	{
		throttle =0;
	    throttle_out =  eng_idle_thrust_out;
	}

	/* Log engine throttle output */
	_eng_run_state.cmd_in_log = (int16_t)(throttle*1000);
	_eng_run_state.cmd_out_log = (int16_t)(throttle_out);

	/* return control out to motor */
	return throttle_out;
}


#endif