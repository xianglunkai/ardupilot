#pragma once

#include "AP_SCU_Config.h"

#if AP_SCU_ENABLED
#include <AP_Param/AP_Param.h>
#include <AP_Relay/AP_Relay_config.h>
#include <Filter/LowPassFilter.h>
#include <AP_RPM/AP_RPM_config.h>


class AP_SCU_Engine {
public:
    // constructor
    AP_SCU_Engine();

    static const struct AP_Param::GroupInfo var_info[];

    // Engine start/stop control data
    enum EngineSTSState {
        DEFAULT_RESET = 0,   // default
        STOP_RESET    = 1,   // has been stoped
        START_RESET   = 2,   // has been started
        STOPING       = 3,   // in the process of being stoped
        STARTING      = 4,   // in the process of being started
    };

    enum EngineSTSCmd {
        NO_OPS    = 0,
        REQ_START = 1,
        REQ_STOP  = 2
    };


    // engine running state 
    struct EngineRunState {
        bool armed_state;
        bool ems_state;
        int16_t  speed;
        int16_t  cmd_in_log;
        int16_t  cmd_out_log;
    };


    // Engine start/stop control data type
    struct EngineSTSCtrl {
        uint16_t sts_cmd;
        uint16_t last_sts_cmd;
        uint16_t sts_event_cmd;

        EngineSTSState sts_ctrl_state;
        uint16_t sts_delay_cnt;
        uint16_t sts_lock_flag;
    };


    // one time init call
    void init();

    // get engine speed
    virtual bool eng_get_speed(uint16_t & speed) { return false; }

    // STS control
    virtual void eng_armed_control(const uint16_t sts_cmd, const uint16_t ems, const uint16_t dt_ms);

    // throttle control
    virtual int16_t eng_throttle_control(const float thr_cmd);

    // get engine control state
    const EngineRunState & eng_state(void) { return _eng_run_state; }


private:
#if AP_RPM_ENABLED
    // filter for RPM value
    LowPassFilterConstDtFloat _rpm_filter;
    float filtered_rpm_value;
#endif

    EngineRunState _eng_run_state;

    EngineSTSCtrl _eng_sts_ctrl;


    void set_acc(bool on);

    void set_start(bool on);
    
    void set_stop(bool on);

private:
    // parameters


    // time to run starter for (seconds)
    AP_Float starter_time;

    // delay between start attempts (seconds)
    AP_Float starter_delay;

    AP_Int16 idle_percent;

    AP_Int16 rpm_instance;

   	AP_Int16 eng_forward_max_thrust_out;            // The output for Engine forward running maximum speed
	AP_Int16 eng_backward_max_thrust_out;           // The output for Engine backward running maximum speed
	AP_Int16 eng_idle_thrust_out;                   // The output for Engine idle running
	AP_Int16 eng_halt_thrust_out;                   // The output for Engine shutdown
    
    AP_Int16 gear_D_thrust_out;
	AP_Int16 gear_N_thrust_out;
	AP_Int16 gear_R_thrust_out;
	AP_Int16 gear_shift_engine_speed;                /* Must ensure current engine speed is low,uint: rpm,resolution: 1rpm */
	AP_Int16 gear_shift_block_ms;                  /* shift gear block time to ensure shiftting is OK,uints: ms,resolution: 1ms */
};



#endif

