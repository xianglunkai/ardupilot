#pragma once 

#include <AC_PID/AC_ADRC.h>
#include <AC_PID/AC_P.h>

#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_ADRC_ENABLED
    #define CUSTOMCONTROL_ADRC_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_ADRC_ENABLED

class AC_CustomControl_ADRC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);

    // run latest level body-frame rate controller and send outputs to the motors
    Vector3f update() override;
    void reset() override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // put controller related variables here
    float _dt;

    // angle P controller objects
    AC_P                _p_angle_roll2;
    AC_P                _p_angle_pitch2;
    AC_P                _p_angle_yaw2;

    // rate PID controller objects
    AC_ADRC _adrc_atti_rate_roll;
    AC_ADRC _adrc_atti_rate_pitch;
    AC_ADRC _adrc_atti_rate_yaw;
};
#endif