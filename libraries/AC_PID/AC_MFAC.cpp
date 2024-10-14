#include "AC_MFAC.h"

#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AC_MFAC::var_info[] = {

    // @Param: LAMDA
    // @DisplayName: MFAC penalty facor for controller output change
    // @Description: Penalty factor for controller output change
    AP_GROUPINFO("LAMDA",    0, AC_MFAC, _lamada, 10),

    // @Param: MU
    // @DisplayName: MFAC penalty factor for adaptive parameters change
    // @Description: Penalty factor for adaptive parameters change
    AP_GROUPINFO("MU",    1, AC_MFAC, _mu, 100),

    // @Param: YITA
    // @DisplayName:MFAC adaptive learning rate
    // @Description: Adaptive learning rate
    AP_GROUPINFO("YITA",    2, AC_MFAC, _yita, 1.0),

    // @Param: EPLSE
    // @DisplayName: MFAC PPD reset deadzone
    // @Description: PPD reset deadzone
    AP_GROUPINFO("EPLSE",    3, AC_MFAC, _eplise, 0.01),

    // @Param: KR
    // @DisplayName: MFAC penalty factor for control system damping
    // @Description: Penalty factor for control system damping
    AP_GROUPINFO("KR",    4, AC_MFAC, _kr, 1.0),

    // @Param: ROU1
    // @DisplayName: PI Gain
    // @Description: PI gain produce an ouput that is proportional to the rate of change of the error
    AP_GROUPINFO("ROU1",    5, AC_MFAC,_rou[0],1.0),

    // @Param: ROU2
    // @DisplayName: D Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("ROU2",    6, AC_MFAC, _rou[1], 1.0),

    // @Param: FAI1
    // @DisplayName: MFAC PPD parameter
    // @Description: PPD parameter intial value
    AP_GROUPINFO("FAI1",    7, AC_MFAC,_fai[0],0.5),

    // @Param: FAI2
    // @DisplayName: MFAC PPD parameter
    // @Description:  PPD parameter initial value
    AP_GROUPINFO("FAI2",    8, AC_MFAC, _fai[1], 0.5),

    // @Param: LM
    // @Description: Control output bound
    // @User: Standard
    AP_GROUPINFO("LM",    9, AC_MFAC, _limit,1.0f),

    AP_GROUPEND
};


// Constructor
AC_MFAC::AC_MFAC(float initial_lamada,float initial_kr,float initial_eplise,float dt):
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this,var_info);

    _lamada.set_and_default(initial_lamada);
    _kr.set_and_default(initial_kr);
    _eplise.set_and_default(initial_eplise);

    memset(&_debug_info, 0, sizeof(_debug_info));

}

// update_all  - set target and measured inputs to MFAC controller and calculate output
float AC_MFAC::update_all(const float target,const float measurement,const bool wrap2pi)
{
   // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // updtate measurement
    const float del_measurement = (wrap2pi)?wrap_PI(measurement - _measurement):(measurement - _measurement);
    _measurement = measurement;

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha() * (target - _target);
        _error += get_filt_E_alpha() * ((_target - measurement) - _error);
    }


    // MFAC controller calculation
    const float error = (wrap2pi)?wrap_PI(_error) : (_error;
    float control_cmd = _control_cmd
                      + _rou[1] * _vec_fai.y * (error - _kr * del_measurement / _dt) / (_lamada + sq(_vec_fai.y))
                      - _rou[0] * _vec_fai.x * _vec_fai.y * del_measurement / (_lamada + sq(_vec_fai.y));
    
    // Limit output 
    if(!is_zero(_limit.get())){
        control_cmd = constrain_float(control_cmd ,-_limit,_limit);
    }

    // update PPD 
    _vec_delu.x = del_measurement;
    _vec_delu.y = control_cmd - _control_cmd;
    _vec_fai +=  _vec_delu *  (del_measurement - _vec_fai * _vec_delu) * _yita / ( _mu + _vec_delu.length_squared());

    // reset PPD 
    if(_vec_fai.length() <= _eplise || _vec_delu.length() <= _eplise || !is_zero(sign(_vec_fai.x) - sign(_vec_fai.y)))
    {
        _vec_fai.x = _fai[0];
        _vec_fai.y = _fai[1];
    }

    // return controller output
    _control_cmd = control_cmd;

    // log debug message
    _debug_info.target = target;
    _debug_info.actual = measurement;
    _debug_info.error  = target - measurement;
    _debug_info.P      = _vec_fai.x;
    _debug_info.I      = 0;
    _debug_info.D      = _vec_fai.y;
    _debug_info.FF     = _control_cmd;

    return control_cmd;
}


// reset PPD and state variables
void AC_MFAC::reset(const float measurement)
{
    _vec_fai.x = _fai[0];
    _vec_fai.y = _fai[1];

    _vec_delu.zero();

    _control_cmd = 0.0f;
    _measurement = measurement;
    memset(&_debug_info, 0, sizeof(_debug_info));
}

// get_filt_T_alpha - get the target filter alpha
float AC_MFAC::get_filt_T_alpha() const
{
    return get_filt_alpha(_filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_MFAC::get_filt_E_alpha() const
{
    return get_filt_alpha(_filt_E_hz);
}

// get_filt_alpha - calculate a filter alpha
float AC_MFAC::get_filt_alpha(float filt_hz) const
{
    return calc_lowpass_alpha_dt(_dt, filt_hz);
}