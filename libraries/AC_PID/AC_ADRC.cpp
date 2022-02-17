#include "AC_ADRC.h"
#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AC_ADRC::var_info[] = {

    // @Param: WC
    // @Description: Response bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WC",2,AC_ADRC,wc_,10),

    // @Param: WO
    // @Description: ESO bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WO",3,AC_ADRC,wo_,15),
    
    // @Param: B0
    // @Description: Control input gain
    // @User: Standard
    AP_GROUPINFO("B0",4,AC_ADRC,b0_,10),

    // @Param: DELTA
    // @Description: Linear deadzone
    // @User: Standard
    AP_GROUPINFO("DELTA",5,AC_ADRC,delta_,0.0),

    // @Param: ORDER
    // @Description: Model order
    // @User: Standard
    AP_GROUPINFO("ORDER",6,AC_ADRC,order_,1),

    // @Param: LM
    // @Description: Control output bound
    // @User: Standard
    AP_GROUPINFO("LM",7,AC_ADRC,limit_,1.0f),

    AP_GROUPEND

};

AC_ADRC::AC_ADRC()
{
    AP_Param::setup_object_defaults(this,var_info);

    memset(&_debug_info, 0, sizeof(_debug_info));
}

float AC_ADRC::update_all(float target,float measurement)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // get controller error
    float e1 = target - z1_;
    
    // control derivation error
    float e2 = -z2_;      

    // state estimation error                
    float e  = z1_ - measurement;         

    float output = 0.0f;
    float output_limited = 0;
    float sigma = 1.0f/(sq(e) + 1.0f);

    switch (order_)
    {
    case 1:
        {
            // nonlinear control law
            output = (wc_ * fal(e1,0.5f,delta_)  - sigma * z2_)/b0_;

            // limit output 
            if(is_zero(limit_.get())){
                output_limited = output;
            }else{
                output_limited = constrain_float(output,-limit_,limit_);
            }
            
            // state estimation
            float fe = fal(e,0.5,delta_);
            float beta1 = 2 * wo_;
            float beta2 = wo_ * wo_;
            z1_ = z1_ + dt_ * (z2_ - beta1*e + b0_ * output_limited);
            z2_ = z2_ + dt_ * (-beta2 * fe);

            _debug_info.P      = z1_;
            _debug_info.I      = z2_;
            _debug_info.D      = z3_;
            _debug_info.FF     = output_limited;
        }
        break;
    case 2:
        {
            float kp  = sq(wc_);
            float kd  = 2*wc_;

            // nonlinear control law
            output = (kp * fal(e1,0.5f,delta_) + kd * fal(e2,0.25,delta_) - sigma * z3_)/b0_;

            // limit output 
            if(is_zero(limit_.get())){
                output_limited = output;
            }else{
                output_limited = constrain_float(output,-limit_,limit_);
            }
            
            // state estimation
            float beta1 = 3 * wo_;
            float beta2 = 3 * wo_ * wo_;
            float beta3 = wo_ * wo_ * wo_;
            float fe  = fal(e,0.5,delta_);
            float fe1 = fal(e,0.25,delta_);
            z1_  = z1_ + dt_ * (z2_ - beta1 * e);
            z2_  = z2_ + dt_ * (z3_ - beta2 * fe + b0_ * output_limited);
            z3_  = z3_ + dt_ * (- beta3 * fe1);

            _debug_info.P      = z1_;
            _debug_info.I      = z2_;
            _debug_info.D      = z3_;
            _debug_info.FF     = output_limited;
        }
        break;
    default:
        output_limited = 0.0f;
        break;
    }


    // For loggers
    _debug_info.target = target;
    _debug_info.actual = measurement;
    _debug_info.error  = target - measurement;

    return output_limited;
}



void AC_ADRC::set_dt(float dt)
{
    dt_ = dt;
}

void AC_ADRC::reset_eso(float measurement)
{
    z1_ = measurement;
    z2_ = 0.0f;
    z3_ = 0.0f;
}