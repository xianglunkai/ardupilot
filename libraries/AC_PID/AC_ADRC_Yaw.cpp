#include "AC_ADRC_Yaw.h"
#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AC_ADRC_YAW::var_info[] = {

    // @Param: GAMA
    // @Description: Disturb composent factor
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GAMA",0,AC_ADRC_YAW,gama_,1),

    // @Param: WC
    // @Description: Response bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WC",1,AC_ADRC_YAW,wc_,10),

    // @Param: WO
    // @Description: ESO bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WO",2,AC_ADRC_YAW,wo_,15),

    // @Param: B0
    // @Description: Control input gain
    // @User: Standard
    AP_GROUPINFO("B0",3,AC_ADRC_YAW,b0_,10),

    // @Param: DELTA
    // @Description: Linear deadzone
    // @User: Standard
    AP_GROUPINFO("DELTA",4,AC_ADRC_YAW,delta_,50),

    // @Param: KESAI
    // @Description: Response damping,should set zero initially
    // @User: Standard
    AP_GROUPINFO("KESAI",5,AC_ADRC_YAW,kesai_,0.0f),

    // @Param: ERMAX
    // @Description: Control error maximum
    // @User: Standard
    AP_GROUPINFO("ERMAX",6,AC_ADRC_YAW,error_max_,60),

    // @Param: LM
    // @Description: Control output bound
    // @User: Standard
    AP_GROUPINFO("LM",7,AC_ADRC_YAW,bound_command_,1.0f),

    AP_GROUPEND
};

AC_ADRC_YAW::AC_ADRC_YAW(float initial_wc,float initial_wo,float initial_b0,float intial_delta,
                         float dt,float initial_gama,float initial_kesai,float initial_error_max):
    dt_(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this,var_info);

    wc_ = initial_wc;
    wo_ = initial_wo;
    b0_ = initial_b0;
    delta_ = intial_delta;
    gama_ = initial_gama;
    kesai_ = initial_kesai;
    error_max_ = initial_error_max;

    // reset input filter to first value received
    flags_.reset_filter_ = true;

    memset(&_debug_info, 0, sizeof(_debug_info));
}

float AC_ADRC_YAW::update_all(float target,float measurement)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement) || is_zero(dt_)) {
        return 0.0f;
    }

    if(flags_.reset_filter_){
        flags_.reset_filter_ = false;
        v1_ = measurement;
        v2_ = 0;
    }

    // observation error
    float est_error = wrap_180(measurement - z1_);
    if(isnan(est_error)){est_error = 0;}
 
    // Smooth reference signal and calculate and filter derivative
    float error = wrap_180(target - z1_);
    float derivative = 0 - z2_;

    error = constrain_float(error,-error_max_,error_max_);

    // update control output
    float kp = wc_ * wc_,kd = 2 * wc_ * kesai_;
    float control_unbounded = (kp * fal(error,0.5f,delta_) + kd *fal(derivative,0.25,delta_) - gama_ * z3_)/b0_;
   
    // Anti saturation
    float control = 0.0f;
    if(is_zero(bound_command_.get())){
        control = control_unbounded;
    }else{
        control = constrain_float(control_unbounded, -bound_command_,+bound_command_);
    }
     
    // ESO update
    float l1 = 3 * wo_;
	float l2 = 3 * wo_ * wo_;
	float l3 = wo_ * wo_ * wo_;
    float fe  = fal(est_error,0.5,delta_);
    float fe1 = fal(est_error,0.25,delta_);

    z1_ += (z2_ + l1 * est_error) * dt_;
    z1_ = wrap_360(z1_);
    z2_ += (z3_ + b0_ * control + l2 * fe) * dt_;
    z3_ += (l3 * fe1) * dt_;

     // For loggers
    _debug_info.target = target;
    _debug_info.actual = measurement;
    _debug_info.error  = error ;
  
    _debug_info.P = z1_;
    _debug_info.I = z2_;
    _debug_info.D = z3_;
    _debug_info.FF = control;

    return control;
}

void AC_ADRC_YAW::set_dt(float dt)
{
    dt_ = dt;
}

void AC_ADRC_YAW::reset_eso(float measurement,float measurement_rate)
{
    z1_ = measurement;
    z2_ = measurement_rate;
    z3_ = 0.0f;
    memset(&_debug_info, 0, sizeof(_debug_info));
}
