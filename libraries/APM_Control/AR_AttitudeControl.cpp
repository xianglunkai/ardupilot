/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_AttitudeControl.h"
#include <AP_GPS/AP_GPS.h>

// attitude control default definition
#define AR_ATTCONTROL_STEER_ANG_P       2.00f
#define AR_ATTCONTROL_STEER_RATE_FF     0.20f
#define AR_ATTCONTROL_STEER_RATE_P      0.20f
#define AR_ATTCONTROL_STEER_RATE_I      0.20f
#define AR_ATTCONTROL_STEER_RATE_IMAX   1.00f
#define AR_ATTCONTROL_STEER_RATE_D      0.00f
#define AR_ATTCONTROL_STEER_RATE_FILT   10.00f
#define AR_ATTCONTROL_STEER_RATE_MAX    120.0f
#define AR_ATTCONTROL_STEER_ACCEL_MAX   120.0f
#define AR_ATTCONTROL_THR_SPEED_P       0.20f
#define AR_ATTCONTROL_THR_SPEED_I       0.20f
#define AR_ATTCONTROL_THR_SPEED_IMAX    1.00f
#define AR_ATTCONTROL_THR_SPEED_D       0.00f
#define AR_ATTCONTROL_THR_SPEED_FILT    10.00f
#define AR_ATTCONTROL_PITCH_THR_P       1.80f
#define AR_ATTCONTROL_PITCH_THR_I       1.50f
#define AR_ATTCONTROL_PITCH_THR_D       0.03f
#define AR_ATTCONTROL_PITCH_THR_IMAX    1.0f
#define AR_ATTCONTROL_PITCH_THR_FILT    10.0f
#define AR_ATTCONTROL_BAL_SPEED_FF      1.0f
#define AR_ATTCONTROL_DT                0.02f
#define AR_ATTCONTROL_TIMEOUT_MS        200
#define AR_ATTCONTROL_HEEL_SAIL_P       1.0f
#define AR_ATTCONTROL_HEEL_SAIL_I       0.1f
#define AR_ATTCONTROL_HEEL_SAIL_D       0.0f
#define AR_ATTCONTROL_HEEL_SAIL_IMAX    1.0f
#define AR_ATTCONTROL_HEEL_SAIL_FILT    10.0f
#define AR_ATTCONTROL_DT                0.02f

// ADRC controller default parameters:
#define AR_ATTCONTROL_STEER_ANG_WC      10.0f
#define AR_ATTCONTROL_STEER_ANG_WO      13.0f
#define AR_ATTCONTROL_STEER_ANG_B0      80.0f
#define AR_ATTCONTROL_STEER_ANG_DELTA   50.0f

#define AR_ATTCONTROL_STEER_RATE_WC     45.0f
#define AR_ATTCONTROL_STEER_RATE_WO     12.0f
#define AR_ATTCONTROL_STEER_RATE_B0     10.0f
#define AR_ATTCONTROL_STEER_RATE_DELTA   1.0f
#define AR_ATTCONTROL_STEER_RATE_ORDER   1

#define AR_ATTCONTROL_THR_SPEED_WC      5.0f
#define AR_ATTCONTROL_THR_SPEED_WO      5.0f
#define AR_ATTCONTROL_THR_SPEED_B0      10.0f
#define AR_ATTCONTROL_THR_SPEED_DELTA   1.0f
#define AR_ATTCONTROL_THR_SPEED_ORDER   1

//MFAC controller default parameters:
#define AR_ATTCONTROL_STEER_ANG_LAMADA  1.0f
#define AR_ATTCONTROL_STEER_ANG_KR      0.3f
#define AR_ATTCONTROL_STEER_ANG_EPLISE  0.01f

#define AR_ATTCONTROL_STEER_RATE_LAMADA  1.0f
#define AR_ATTCONTROL_STEER_RATE_KR      0.1f
#define AR_ATTCONTROL_STEER_RATE_EPLISE  0.01f

#define AR_ATTCONTROL_THR_SPEED_LAMADA  10.0f
#define AR_ATTCONTROL_THR_SPEED_KR      1.0f
#define AR_ATTCONTROL_THR_SPEED_EPLISE  0.01f



// throttle/speed control maximum acceleration/deceleration (in m/s) (_ACCEL_MAX parameter default)
#define AR_ATTCONTROL_THR_ACCEL_MAX     1.00f

// minimum speed in m/s
#define AR_ATTCONTROL_STEER_SPEED_MIN   1.0f

// speed (in m/s) at or below which vehicle is considered stopped (_STOP_SPEED parameter default)
#define AR_ATTCONTROL_STOP_SPEED_DEFAULT    0.1f

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AR_AttitudeControl::var_info[] = {

    // @Param: _STR_RAT_P
    // @DisplayName: Steering control rate P gain
    // @Description: Steering control rate P gain.  Converts the turn rate error (in radians/sec) to a steering control output (in the range -1 to +1)
    // @Range: 0.000 2.000
    // @Increment: 0.001
    // @User: Standard

    // @Param: _STR_RAT_I
    // @DisplayName: Steering control I gain
    // @Description: Steering control I gain.  Corrects long term error between the desired turn rate (in rad/s) and actual
    // @Range: 0.000 2.000
    // @Increment: 0.001
    // @User: Standard

    // @Param: _STR_RAT_IMAX
    // @DisplayName: Steering control I gain maximum
    // @Description: Steering control I gain maximum.  Constrains the steering output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _STR_RAT_D
    // @DisplayName: Steering control D gain
    // @Description: Steering control D gain.  Compensates for short-term change in desired turn rate vs actual
    // @Range: 0.000 0.400
    // @Increment: 0.001
    // @User: Standard

    // @Param: _STR_RAT_FF
    // @DisplayName: Steering control feed forward
    // @Description: Steering control feed forward
    // @Range: 0.000 3.000
    // @Increment: 0.001
    // @User: Standard

    // @Param: _STR_RAT_FILT
    // @DisplayName: Steering control filter frequency
    // @Description: Steering control input filter.  Lower values reduce noise but add delay.
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _STR_RAT_FLTT
    // @DisplayName: Steering control Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _STR_RAT_FLTE
    // @DisplayName: Steering control Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _STR_RAT_FLTD
    // @DisplayName: Steering control Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _STR_RAT_SMAX
    // @DisplayName: Steering slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_steer_rate_pid, "_STR_RAT_", 1, AR_AttitudeControl, AC_PID),

    // @Param: _SPEED_P
    // @DisplayName: Speed control P gain
    // @Description: Speed control P gain.  Converts the error between the desired speed (in m/s) and actual speed to a motor output (in the range -1 to +1)
    // @Range: 0.010 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _SPEED_I
    // @DisplayName: Speed control I gain
    // @Description: Speed control I gain.  Corrects long term error between the desired speed (in m/s) and actual speed
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _SPEED_IMAX
    // @DisplayName: Speed control I gain maximum
    // @Description: Speed control I gain maximum.  Constrains the maximum motor output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _SPEED_D
    // @DisplayName: Speed control D gain
    // @Description: Speed control D gain.  Compensates for short-term change in desired speed vs actual
    // @Range: 0.000 0.400
    // @Increment: 0.001
    // @User: Standard

    // @Param: _SPEED_FF
    // @DisplayName: Speed control feed forward
    // @Description: Speed control feed forward
    // @Range: 0.000 0.500
    // @Increment: 0.001
    // @User: Standard

    // @Param: _SPEED_FILT
    // @DisplayName: Speed control filter frequency
    // @Description: Speed control input filter.  Lower values reduce noise but add delay.
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _SPEED_FLTT
    // @DisplayName: Speed control Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _SPEED_FLTE
    // @DisplayName: Speed control Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _SPEED_FLTD
    // @DisplayName: Speed control Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _SPEED_SMAX
    // @DisplayName: Speed control slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_throttle_speed_pid, "_SPEED_", 2, AR_AttitudeControl, AC_PID),

    // @Param: _ACCEL_MAX
    // @DisplayName: Speed control acceleration (and deceleration) maximum in m/s/s
    // @Description: Speed control acceleration (and deceleration) maximum in m/s/s.  0 to disable acceleration limiting
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @Units: m/s/s
    // @User: Standard
    AP_GROUPINFO("_ACCEL_MAX", 3, AR_AttitudeControl, _throttle_accel_max, AR_ATTCONTROL_THR_ACCEL_MAX),

    // @Param: _BRAKE
    // @DisplayName: Speed control brake enable/disable
    // @Description: Speed control brake enable/disable. Allows sending a reversed output to the motors to slow the vehicle.
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO("_BRAKE", 4, AR_AttitudeControl, _brake_enable, 1),

    // @Param: _STOP_SPEED
    // @DisplayName: Speed control stop speed
    // @Description: Speed control stop speed.  Motor outputs to zero once vehicle speed falls below this value
    // @Range: 0.00 0.50
    // @Increment: 0.01
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_STOP_SPEED", 5, AR_AttitudeControl, _stop_speed, AR_ATTCONTROL_STOP_SPEED_DEFAULT),

    // @Param: _STR_ANG_P
    // @DisplayName: Steering control angle P gain
    // @Description: Steering control angle P gain.  Converts the error between the desired heading/yaw (in radians) and actual heading/yaw to a desired turn rate (in rad/sec)
    // @Range: 1.000 10.000
    // @Increment: 0.1
    // @User: Standard
    AP_SUBGROUPINFO(_steer_angle_p, "_STR_ANG_", 6, AR_AttitudeControl, AC_P),

    // @Param: _STR_ACC_MAX
    // @DisplayName: Steering control angular acceleration maximum
    // @Description: Steering control angular acceleration maximum (in deg/s/s).  0 to disable acceleration limiting
    // @Range: 0 1000
    // @Increment: 0.1
    // @Units: deg/s/s
    // @User: Standard
    AP_GROUPINFO("_STR_ACC_MAX", 7, AR_AttitudeControl, _steer_accel_max, AR_ATTCONTROL_STEER_ACCEL_MAX),

    // @Param: _STR_RAT_MAX
    // @DisplayName: Steering control rotation rate maximum
    // @Description: Steering control rotation rate maximum in deg/s.  0 to remove rate limiting
    // @Range: 0 1000
    // @Increment: 0.1
    // @Units: deg/s
    // @User: Standard
    AP_GROUPINFO("_STR_RAT_MAX", 8, AR_AttitudeControl, _steer_rate_max, AR_ATTCONTROL_STEER_RATE_MAX),

    // @Param: _DECEL_MAX
    // @DisplayName: Speed control deceleration maximum in m/s/s
    // @Description: Speed control and deceleration maximum in m/s/s.  0 to use ATC_ACCEL_MAX for deceleration
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @Units: m/s/s
    // @User: Standard
    AP_GROUPINFO("_DECEL_MAX", 9, AR_AttitudeControl, _throttle_decel_max, 0.00f),

    // @Param: _BAL_P
    // @DisplayName: Pitch control P gain
    // @Description: Pitch control P gain for BalanceBots.  Converts the error between the desired pitch (in radians) and actual pitch to a motor output (in the range -1 to +1)
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _BAL_I
    // @DisplayName: Pitch control I gain
    // @Description: Pitch control I gain for BalanceBots.  Corrects long term error between the desired pitch (in radians) and actual pitch
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _BAL_IMAX
    // @DisplayName: Pitch control I gain maximum
    // @Description: Pitch control I gain maximum.  Constrains the maximum motor output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _BAL_D
    // @DisplayName: Pitch control D gain
    // @Description: Pitch control D gain.  Compensates for short-term change in desired pitch vs actual
    // @Range: 0.000 0.100
    // @Increment: 0.001
    // @User: Standard

    // @Param: _BAL_FF
    // @DisplayName: Pitch control feed forward
    // @Description: Pitch control feed forward
    // @Range: 0.000 0.500
    // @Increment: 0.001
    // @User: Standard

    // @Param: _BAL_FILT
    // @DisplayName: Pitch control filter frequency
    // @Description: Pitch control input filter.  Lower values reduce noise but add delay.
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _BAL_FLTT
    // @DisplayName: Pitch control Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _BAL_FLTE
    // @DisplayName: Pitch control Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _BAL_FLTD
    // @DisplayName: Pitch control Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _BAL_SMAX
    // @DisplayName: Pitch control slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pitch_to_throttle_pid, "_BAL_", 10, AR_AttitudeControl, AC_PID),

    // @Param: _BAL_SPD_FF
    // @DisplayName: Pitch control feed forward from speed
    // @Description: Pitch control feed forward from speed
    // @Range: 0.0 10.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_BAL_SPD_FF", 11, AR_AttitudeControl, _pitch_to_throttle_speed_ff, AR_ATTCONTROL_BAL_SPEED_FF),

    // @Param: _SAIL_P
    // @DisplayName: Sail Heel control P gain
    // @Description: Sail Heel control P gain for sailboats.  Converts the error between the desired heel angle (in radians) and actual heel to a main sail output (in the range -1 to +1)
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _SAIL_I
    // @DisplayName: Sail Heel control I gain
    // @Description: Sail Heel control I gain for sailboats.  Corrects long term error between the desired heel angle (in radians) and actual
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _SAIL_IMAX
    // @DisplayName: Sail Heel control I gain maximum
    // @Description: Sail Heel control I gain maximum.  Constrains the maximum I term contribution to the main sail output (range -1 to +1)
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _SAIL_D
    // @DisplayName: Sail Heel control D gain
    // @Description: Sail Heel control D gain.  Compensates for short-term change in desired heel angle vs actual
    // @Range: 0.000 0.100
    // @Increment: 0.001
    // @User: Standard

    // @Param: _SAIL_FF
    // @DisplayName: Sail Heel control feed forward
    // @Description: Sail Heel control feed forward
    // @Range: 0.000 0.500
    // @Increment: 0.001
    // @User: Standard

    // @Param: _SAIL_FILT
    // @DisplayName: Sail Heel control filter frequency
    // @Description: Sail Heel control input filter.  Lower values reduce noise but add delay.
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _SAIL_FLTT
    // @DisplayName: Sail Heel Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _SAIL_FLTE
    // @DisplayName: Sail Heel Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _SAIL_FLTD
    // @DisplayName: Sail Heel Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard

    // @Param: _SAIL_SMAX
    // @DisplayName: Sail heel slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_sailboat_heel_pid, "_SAIL_", 12, AR_AttitudeControl, AC_PID),

    // @Param: _TURN_MAX_G
    // @DisplayName: Turning maximum G force
    // @Description: The maximum turning acceleration (in units of gravities) that the rover can handle while remaining stable. The navigation code will keep the lateral acceleration below this level to avoid rolling over or slipping the wheels in turns
    // @Units: gravities
    // @Range: 0.1 10
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_TURN_MAX_G", 13, AR_AttitudeControl, _turn_lateral_G_max, 0.6f),


    // @Param: _CTL_TYP_STR
    // @DisplayName: Steering rate controller type
    // @Range: 0 PID 1 ADRC 2 ADPC
    // @User: Standard
    AP_GROUPINFO("_CTL_TYP_STR", 20, AR_AttitudeControl, _steer_rate_ctl_type, 0),

    // @Param: _CTL_TYP_SPD
    // @DisplayName: Speed controller type
    // @Range: 0 PID 1 ADRC 2 ADP
    // @User: Standard
    AP_GROUPINFO("_CTL_TYP_SPD", 21, AR_AttitudeControl, _throttle_speed_ctl_type, 0),

    // @Param: _CTL_TYP_ANG
    // @DisplayName: Steering angle controller type 
    // @Range: 0 PID 1 ADRC 2 ADP
    // @User: Standard
    AP_GROUPINFO("_CTL_TYP_ANG", 22, AR_AttitudeControl, _steer_angle_ctl_type, 0),

    // @Param: STR_AR_WC
    // @Description: Response bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard

    // @Param: STR_AR_WO
    // @Description: ESO bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    
    // @Param: STR_AR_B0
    // @Description: Control input gain
    // @User: Standard

    // @Param: STR_AR_DELTA
    // @Description: Linear deadzone
    // @User: Standard
 
    // @Param: STR_AR_ORDER
    // @Description: Model order
    // @User: Standard

    // @Param: STR_AR_LM
    // @Description: Control output bound
    // @User: Standard
    AP_SUBGROUPINFO(_steer_rate_adrc,"_STR_AR_", 23, AR_AttitudeControl, AC_ADRC),

    // @Param: SPD_AR_WC
    // @Description: Response bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard

    // @Param: SPD_AR_WO
    // @Description: ESO bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    
    // @Param: SPD_AR_B0
    // @Description: Control input gain
    // @User: Standard

    // @Param: SPD_AR_DELTA
    // @Description: Linear deadzone
    // @User: Standard
 
    // @Param: SPD_AR_ORDER
    // @Description: Model order
    // @User: Standard

    // @Param: SPD_AR_LM
    // @Description: Control output bound
    // @User: Standard
    AP_SUBGROUPINFO(_throttle_speed_adrc,"_SPD_AR_", 24, AR_AttitudeControl, AC_ADRC),

    // @Param: ANG_AR_WC
    // @Description: Response bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard

    // @Param: ANG_AR_WO
    // @Description: ESO bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard

    // @Param: ANG_AR_B0
    // @Description: Control input gain
    // @User: Standard

    // @Param: ANG_AR_DELTA
    // @Description: Linear deadzone
    // @User: Standard

    // @Param: ANG_AR_KESAI
    // @Description: Response damping
    // @User: Standard

    // @Param: ANG_AR_ERRMAX
    // @Description: Control error maximum
    // @User: Standard,

    // @Param: ANG_AR_LM
    // @Description: Control output bound
    // @User: Standard
    AP_SUBGROUPINFO(_steer_angle_adrc,"_ANG_AR_", 25, AR_AttitudeControl, AC_ADRC_YAW),

    // @Param: STR_MA_LAMDA
    // @DisplayName: MFAC penalty facor for controller output change
    // @Description: Penalty factor for controller output change

    // @Param: STR_MA_MU
    // @DisplayName: MFAC penalty factor for adaptive parameters change
    // @Description: Penalty factor for adaptive parameters change

    // @Param: STA_MA_YITA
    // @DisplayName:MFAC adaptive learning rate
    // @Description: Adaptive learning rate

    // @Param: STR_MA_EPLSE
    // @DisplayName: MFAC PPD reset deadzone
    // @Description: PPD reset deadzone

    // @Param: STR_MA_KR
    // @DisplayName: MFAC penalty factor for control system damping
    // @Description: Penalty factor for control system damping

    // @Param: STR_MA_ROU1
    // @DisplayName: PI Gain
    // @Description: PI gain produce an ouput that is proportional to the rate of change of the error

    // @Param: STR_MA_ROU2
    // @DisplayName: D Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error

    // @Param: STR_MA_FAI1
    // @DisplayName: MFAC PPD parameter
    // @Description: PPD parameter intial value

    // @Param: STR_MA_FAI2
    // @DisplayName: MFAC PPD parameter
    // @Description:  PPD parameter initial value

    // @Param: STR_MA_LM
    // @Description: Control output bound
    // @User: Standard
    AP_SUBGROUPINFO(_steer_rate_mfac,"_STR_MA_",26,AR_AttitudeControl,AC_MFAC),

    // @Param: SPD_MA_LAMDA
    // @DisplayName: MFAC penalty facor for controller output change
    // @Description: Penalty factor for controller output change

    // @Param: SPA_MA_MU
    // @DisplayName: MFAC penalty factor for adaptive parameters change
    // @Description: Penalty factor for adaptive parameters change

    // @Param: SPD_MA_YITA
    // @DisplayName:MFAC adaptive learning rate
    // @Description: Adaptive learning rate

    // @Param: SPD_MA_EPLSE
    // @DisplayName: MFAC PPD reset deadzone
    // @Description: PPD reset deadzone

    // @Param: SPD_MA_KR
    // @DisplayName: MFAC penalty factor for control system damping
    // @Description: Penalty factor for control system damping

    // @Param: SPD_MA_ROU1
    // @DisplayName: PI Gain
    // @Description: PI gain produce an ouput that is proportional to the rate of change of the error

    // @Param: SPD_MA_ROU2
    // @DisplayName: D Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error

    // @Param: SPD_MA_FAI1
    // @DisplayName: MFAC PPD parameter
    // @Description: PPD parameter intial value

    // @Param: SPD_MA_FAI2
    // @DisplayName: MFAC PPD parameter
    // @Description:  PPD parameter initial value

    // @Param: SPD_MA_LM
    // @Description: Control output bound
    // @User: Standard
    AP_SUBGROUPINFO(_throttle_speed_mfac,"_SPD_MA_",27,AR_AttitudeControl,AC_MFAC),


    // @Param: ANG_MA_LAMDA
    // @DisplayName: MFAC penalty facor for controller output change
    // @Description: Penalty factor for controller output change

    // @Param: ANG_MA_MU
    // @DisplayName: MFAC penalty factor for adaptive parameters change
    // @Description: Penalty factor for adaptive parameters change

    // @Param: ANG_MA_YITA
    // @DisplayName:MFAC adaptive learning rate
    // @Description: Adaptive learning rate

    // @Param: ANG_MA_EPLSE
    // @DisplayName: MFAC PPD reset deadzone
    // @Description: PPD reset deadzone

    // @Param: ANG_MA_KR
    // @DisplayName: MFAC penalty factor for control system damping
    // @Description: Penalty factor for control system damping

    // @Param: ANG_MA_ROU1
    // @DisplayName: PI Gain
    // @Description: PI gain produce an ouput that is proportional to the rate of change of the error

    // @Param: ANG_MA_ROU2
    // @DisplayName: D Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error

    // @Param: ANG_MA_FAI1
    // @DisplayName: MFAC PPD parameter
    // @Description: PPD parameter intial value

    // @Param: ANG_MA_FAI2
    // @DisplayName: MFAC PPD parameter
    // @Description:  PPD parameter initial value

    // @Param: ANG_MA_LM
    // @Description: Control output bound
    // @User: Standard
    AP_SUBGROUPINFO(_steer_angle_mfac,"_ANG_MA_",28,AR_AttitudeControl,AC_MFAC),

    AP_GROUPEND
};

AR_AttitudeControl::AR_AttitudeControl() :
    _steer_angle_p(AR_ATTCONTROL_STEER_ANG_P),
    _steer_rate_pid(AR_ATTCONTROL_STEER_RATE_P, AR_ATTCONTROL_STEER_RATE_I, AR_ATTCONTROL_STEER_RATE_D, AR_ATTCONTROL_STEER_RATE_FF, AR_ATTCONTROL_STEER_RATE_IMAX, 0.0f, AR_ATTCONTROL_STEER_RATE_FILT, 0.0f, AR_ATTCONTROL_DT),
    _throttle_speed_pid(AR_ATTCONTROL_THR_SPEED_P, AR_ATTCONTROL_THR_SPEED_I, AR_ATTCONTROL_THR_SPEED_D, 0.0f, AR_ATTCONTROL_THR_SPEED_IMAX, 0.0f, AR_ATTCONTROL_THR_SPEED_FILT, 0.0f, AR_ATTCONTROL_DT),
    _pitch_to_throttle_pid(AR_ATTCONTROL_PITCH_THR_P, AR_ATTCONTROL_PITCH_THR_I, AR_ATTCONTROL_PITCH_THR_D, 0.0f, AR_ATTCONTROL_PITCH_THR_IMAX, 0.0f, AR_ATTCONTROL_PITCH_THR_FILT, 0.0f, AR_ATTCONTROL_DT),
    _sailboat_heel_pid(AR_ATTCONTROL_HEEL_SAIL_P, AR_ATTCONTROL_HEEL_SAIL_I, AR_ATTCONTROL_HEEL_SAIL_D, 0.0f, AR_ATTCONTROL_HEEL_SAIL_IMAX, 0.0f, AR_ATTCONTROL_HEEL_SAIL_FILT, 0.0f, AR_ATTCONTROL_DT),
    _steer_angle_adrc(AR_ATTCONTROL_STEER_ANG_WC,AR_ATTCONTROL_STEER_ANG_WO,AR_ATTCONTROL_STEER_ANG_B0,AR_ATTCONTROL_STEER_ANG_DELTA,AR_ATTCONTROL_DT),
    _steer_rate_adrc(AR_ATTCONTROL_STEER_RATE_WC,AR_ATTCONTROL_STEER_RATE_WO,AR_ATTCONTROL_STEER_RATE_B0,AR_ATTCONTROL_STEER_RATE_DELTA,AR_ATTCONTROL_STEER_RATE_ORDER,AR_ATTCONTROL_DT),
    _throttle_speed_adrc(AR_ATTCONTROL_THR_SPEED_WC,AR_ATTCONTROL_THR_SPEED_WO,AR_ATTCONTROL_THR_SPEED_B0,AR_ATTCONTROL_THR_SPEED_DELTA,AR_ATTCONTROL_THR_SPEED_ORDER,AR_ATTCONTROL_DT),
    _steer_angle_mfac(AR_ATTCONTROL_STEER_ANG_LAMADA,AR_ATTCONTROL_STEER_ANG_KR,AR_ATTCONTROL_STEER_ANG_EPLISE,AR_ATTCONTROL_DT),
    _steer_rate_mfac(AR_ATTCONTROL_STEER_RATE_LAMADA,AR_ATTCONTROL_STEER_RATE_KR,AR_ATTCONTROL_STEER_RATE_EPLISE,AR_ATTCONTROL_DT),
    _throttle_speed_mfac(AR_ATTCONTROL_THR_SPEED_LAMADA,AR_ATTCONTROL_THR_SPEED_KR,AR_ATTCONTROL_THR_SPEED_EPLISE,AR_ATTCONTROL_DT)
    {
    AP_Param::setup_object_defaults(this, var_info);
}

// return a steering servo output from -1.0 to +1.0 given a desired lateral acceleration rate in m/s/s.
// positive lateral acceleration is to the right.
float AR_AttitudeControl::get_steering_out_lat_accel(float desired_accel, bool motor_limit_left, bool motor_limit_right, float dt)
{
    // record desired accel for reporting purposes
    _steer_lat_accel_last_ms = AP_HAL::millis();
    _desired_lat_accel = desired_accel;

    // get speed forward
    float speed;
    if (!get_forward_speed(speed)) {
        // we expect caller will not try to control heading using rate control without a valid speed estimate
        // on failure to get speed we do not attempt to steer
        return 0.0f;
    }

    const float desired_rate = get_turn_rate_from_lat_accel(desired_accel, speed);

    return get_steering_out_rate(desired_rate, motor_limit_left, motor_limit_right, dt);
}

// return a steering servo output from -1 to +1 given a heading in radians
// set rate_max_rads to a non-zero number to apply a limit on the desired turn rate
// return value is normally in range -1.0 to +1.0 but can be higher or lower
float AR_AttitudeControl::get_steering_out_heading(float heading_rad, float rate_max_rads, bool motor_limit_left, bool motor_limit_right, float dt)
{
    // calculate the desired turn rate (in radians) from the angle error (also in radians)
    float desired_rate = get_turn_rate_from_heading(heading_rad, AP::ahrs().yaw,rate_max_rads);

    return get_steering_out_rate(desired_rate, motor_limit_left, motor_limit_right, dt);
}

// return a steering servo ouput from -1 to +1 give a course heading in radians
// set rate_max_rads to a non-zero number to apply a limit on the desired turn rate
// return value is normally in range -1.0 to +1.0 but can be higher or lower
float AR_AttitudeControl::get_steering_out_course(float heading_rad,float rate_max_rads,bool motor_limit_left, bool motor_limit_right,float dt)
{
    // get ground course
    float ground_course_deg;
    Vector2f ground_speed_vec = AP::ahrs().groundspeed_vector();
    const float ground_speed_dir = degrees(ground_speed_vec.angle());
    if (ground_speed_vec.length() < AR_ATTCONTROL_STEER_SPEED_MIN) {
        // with zero ground speed use vehicle's heading
        ground_course_deg = AP::ahrs().yaw_sensor * 0.01f;
    } else {
        ground_course_deg = ground_speed_dir;
    }
    ground_course_deg =  wrap_360(ground_course_deg);

    // select controller
    if(_steer_angle_ctl_type == Controller_type::ADRC){
        return get_steering_out_course_adrc(degrees(heading_rad),ground_course_deg,dt);
    }else if(_steer_angle_ctl_type == Controller_type::MFAC){
         return get_steering_out_course_mfac(heading_rad,radians(ground_course_deg),dt);
    }else{}

   // calculate the desired turn rate (in radians) from the angle error (also in radians)
    float desired_rate = get_turn_rate_from_heading(heading_rad, radians(ground_course_deg),rate_max_rads);

    return get_steering_out_rate(desired_rate, motor_limit_left, motor_limit_right, dt);
}

float AR_AttitudeControl::get_steering_out_course_adrc(float heading_deg,float current_heading,float dt)
{
    const float angular_velocity = degrees(AP::ahrs().get_yaw_rate_earth());
    // if not called recently, reset input filter and desired heading to actual heading 
    const uint32_t now = AP_HAL::millis();
    if(_steer_angle_last_ms == 0 || (now - _steer_angle_last_ms) >= AR_ATTCONTROL_TIMEOUT_MS){
        _steer_angle_adrc.reset_eso(current_heading,angular_velocity);
        _steer_angle_adrc.reset_filter();
    } 
    _steer_angle_last_ms = now;

    // set ADRC's dt
    _steer_angle_adrc.set_dt(dt);

    float output = _steer_angle_adrc.update_all(heading_deg, current_heading);
    // constrain and return final output
    return output;
}

float AR_AttitudeControl::get_steering_out_course_mfac(float heading_rad,float current_heading,float dt)
{
    // if not called recently, reset input filter and desired heading to actual heading 
    const uint32_t now = AP_HAL::millis();
    if(_steer_angle_last_ms == 0 || (now - _steer_angle_last_ms) >= AR_ATTCONTROL_TIMEOUT_MS){
        _steer_angle_mfac.reset(current_heading);
    } 
    _steer_angle_last_ms = now;

    // set MFAC's dt
    _steer_angle_mfac.set_dt(dt);

    float output = _steer_angle_mfac.update_all(heading_rad, current_heading,true);
    // constrain and return final output
    return output;
}


// return a desired turn-rate given a desired heading in radians
float AR_AttitudeControl::get_turn_rate_from_heading(float heading_rad, float current_heading,float rate_max_rads) const
{
    const float yaw_error = wrap_PI(heading_rad - current_heading);

    // Calculate the desired turn rate (in radians) from the angle error (also in radians)
    float desired_rate = _steer_angle_p.get_p(yaw_error);

    // limit desired_rate if a custom pivot turn rate is selected, otherwise use ATC_STR_RAT_MAX
    if (is_positive(rate_max_rads)) {
        desired_rate = constrain_float(desired_rate, -rate_max_rads, rate_max_rads);
    }

    // if acceleration limit is provided, ensure rate can be slowed to zero in time to stop at heading_rad (i.e. avoid overshoot)
    if (is_positive(_steer_accel_max)) {
        const float steer_accel_rate_max_rads = safe_sqrt(2.0 * fabsf(yaw_error) * radians(_steer_accel_max));
        desired_rate = constrain_float(desired_rate, -steer_accel_rate_max_rads, steer_accel_rate_max_rads);
    }

    return desired_rate;
}

// return a steering servo output from -1 to +1 given a
// desired yaw rate in radians/sec. Positive yaw is to the right.
float AR_AttitudeControl::get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right, float dt)
{
    // select controller
    if(_steer_rate_ctl_type == Controller_type::ADRC){
        return get_steering_out_rate_adrc(desired_rate,dt);
    }else if(_steer_rate_ctl_type == Controller_type::MFAC){
        return get_steering_out_rate_mfac(desired_rate,dt);
    }else{}

    // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // if not called recently, reset input filter and desired turn rate to actual turn rate (used for accel limiting)
    const uint32_t now = AP_HAL::millis();
    if ((_steer_turn_last_ms == 0) || ((now - _steer_turn_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        _steer_rate_pid.reset_filter();
        _steer_rate_pid.reset_I();
        _desired_turn_rate = AP::ahrs().get_yaw_rate_earth();
    }
    _steer_turn_last_ms = now;

    // acceleration limit desired turn rate
    if (is_positive(_steer_accel_max)) {
        const float change_max = radians(_steer_accel_max) * dt;
        desired_rate = constrain_float(desired_rate, _desired_turn_rate - change_max, _desired_turn_rate + change_max);
    }
    _desired_turn_rate = desired_rate;

    // rate limit desired turn rate
    if (is_positive(_steer_rate_max)) {
        const float steer_rate_max_rad = radians(_steer_rate_max);
        _desired_turn_rate = constrain_float(_desired_turn_rate, -steer_rate_max_rad, steer_rate_max_rad);
    }

    // G limit based on speed
    float speed;
    if (get_forward_speed(speed)) {
        // do not limit to less than 1 deg/s
        const float turn_rate_max = MAX(get_turn_rate_from_lat_accel(get_turn_lat_accel_max(), fabsf(speed)), radians(1.0f));
        _desired_turn_rate = constrain_float(_desired_turn_rate, -turn_rate_max, turn_rate_max);
    }

    // set PID's dt
    _steer_rate_pid.set_dt(dt);

    float output = _steer_rate_pid.update_all(_desired_turn_rate, AP::ahrs().get_yaw_rate_earth(), (motor_limit_left || motor_limit_right));
    output += _steer_rate_pid.get_ff();
    // constrain and return final output
    return output;
}

float AR_AttitudeControl::get_steering_out_rate_adrc(float desired_rate,float dt)
{
    // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // if not called recently, reset input filter and desired turn rate to actual turn rate (used for accel limiting)
    const uint32_t now = AP_HAL::millis();
    if ((_steer_turn_last_ms == 0) || ((now - _steer_turn_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        _desired_turn_rate = AP::ahrs().get_yaw_rate_earth();
        _steer_rate_adrc.reset_eso(_desired_turn_rate);
    }
    _steer_turn_last_ms = now;

    // acceleration limit desired turn rate
    if (is_positive(_steer_accel_max)) {
        const float change_max = radians(_steer_accel_max) * dt;
        desired_rate = constrain_float(desired_rate, _desired_turn_rate - change_max, _desired_turn_rate + change_max);
    }
    _desired_turn_rate = desired_rate;

    // rate limit desired turn rate
    if (is_positive(_steer_rate_max)) {
        const float steer_rate_max_rad = radians(_steer_rate_max);
        _desired_turn_rate = constrain_float(_desired_turn_rate, -steer_rate_max_rad, steer_rate_max_rad);
    }

    // G limit based on speed
    float speed;
    if (get_forward_speed(speed)) {
        // do not limit to less than 1 deg/s
        const float turn_rate_max = MAX(get_turn_rate_from_lat_accel(get_turn_lat_accel_max(), fabsf(speed)), radians(1.0f));
        _desired_turn_rate = constrain_float(_desired_turn_rate, -turn_rate_max, turn_rate_max);
    }

    // set ADRC's dt
    _steer_rate_adrc.set_dt(dt);

    float output = _steer_rate_adrc.update_all(_desired_turn_rate, AP::ahrs().get_yaw_rate_earth());
    // constrain and return final output
    return output;
}

float AR_AttitudeControl::get_steering_out_rate_mfac(float desired_rate,float dt)
{
    // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // if not called recently, reset input filter and desired turn rate to actual turn rate (used for accel limiting)
    const uint32_t now = AP_HAL::millis();
    if ((_steer_turn_last_ms == 0) || ((now - _steer_turn_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        _desired_turn_rate = AP::ahrs().get_yaw_rate_earth();
        _steer_rate_mfac.reset(_desired_turn_rate);
    }
    _steer_turn_last_ms = now;

    // acceleration limit desired turn rate
    if (is_positive(_steer_accel_max)) {
        const float change_max = radians(_steer_accel_max) * dt;
        desired_rate = constrain_float(desired_rate, _desired_turn_rate - change_max, _desired_turn_rate + change_max);
    }
    _desired_turn_rate = desired_rate;

    // rate limit desired turn rate
    if (is_positive(_steer_rate_max)) {
        const float steer_rate_max_rad = radians(_steer_rate_max);
        _desired_turn_rate = constrain_float(_desired_turn_rate, -steer_rate_max_rad, steer_rate_max_rad);
    }

    // G limit based on speed
    float speed;
    if (get_forward_speed(speed)) {
        // do not limit to less than 1 deg/s
        const float turn_rate_max = MAX(get_turn_rate_from_lat_accel(get_turn_lat_accel_max(), fabsf(speed)), radians(1.0f));
        _desired_turn_rate = constrain_float(_desired_turn_rate, -turn_rate_max, turn_rate_max);
    }

    // set MFAC's dt
    _steer_rate_mfac.set_dt(dt);
    float output = _steer_rate_mfac.update_all(_desired_turn_rate,AP::ahrs().get_yaw_rate_earth(),false);

    // constrain and return final output
    return output;
}

// get latest desired turn rate in rad/sec (recorded during calls to get_steering_out_rate)
float AR_AttitudeControl::get_desired_turn_rate() const
{
    // return zero if no recent calls to turn rate controller
    if ((_steer_turn_last_ms == 0) || ((AP_HAL::millis() - _steer_turn_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }
    return _desired_turn_rate;
}

// get latest desired lateral acceleration in m/s/s (recorded during calls to get_steering_out_lat_accel)
float AR_AttitudeControl::get_desired_lat_accel() const
{
    // return zero if no recent calls to lateral acceleration controller
    if ((_steer_lat_accel_last_ms == 0) || ((AP_HAL::millis() - _steer_lat_accel_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }
    return _desired_lat_accel;
}

// get actual lateral acceleration in m/s/s.  returns true on success
bool AR_AttitudeControl::get_lat_accel(float &lat_accel) const
{
    float speed;
    if (!get_forward_speed(speed)) {
        return false;
    }
    lat_accel = speed * AP::ahrs().get_yaw_rate_earth();
    return true;
}

// calculate the turn rate in rad/sec given a lateral acceleration (in m/s/s) and speed (in m/s)
float AR_AttitudeControl::get_turn_rate_from_lat_accel(float lat_accel, float speed) const
{
    // enforce minimum speed to stop oscillations when first starting to move
    if (fabsf(speed) < AR_ATTCONTROL_STEER_SPEED_MIN) {
        if (is_negative(speed)) {
            speed = -AR_ATTCONTROL_STEER_SPEED_MIN;
        } else {
            speed = AR_ATTCONTROL_STEER_SPEED_MIN;
        }
    }

    return lat_accel / speed;
}

// return a throttle output from -1 to +1 given a desired speed in m/s (use negative speeds to travel backwards)
//   motor_limit should be true if motors have hit their upper or lower limits
//   cruise speed should be in m/s, cruise throttle should be a number from -1 to +1
float AR_AttitudeControl::get_throttle_out_speed(float desired_speed, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt)
{
    // select controller
    if(_throttle_speed_ctl_type == Controller_type::ADRC){
        return get_throttle_out_speed_adrc(desired_speed,dt);
    }else if(_throttle_speed_ctl_type == Controller_type::MFAC){
        return get_throttle_out_speed_mfac(desired_speed,dt);
    }else{
    }

    // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // get speed forward
    float speed;
    if (!get_forward_speed(speed)) {
        // we expect caller will not try to control heading using rate control without a valid speed estimate
        // on failure to get speed we do not attempt to steer
        return 0.0f;
    }

    // if not called recently, reset input filter and desired speed to actual speed (used for accel limiting)
    if (!speed_control_active()) {
        _throttle_speed_pid.reset_filter();
        _throttle_speed_pid.reset_I();
        _desired_speed = speed;
    }
    _speed_last_ms = AP_HAL::millis();

    // acceleration limit desired speed
    _desired_speed = get_desired_speed_accel_limited(desired_speed, dt);

    // set PID's dt
    _throttle_speed_pid.set_dt(dt);

    // calculate base throttle (protect against divide by zero)
    float throttle_base = 0.0f;
    if (is_positive(cruise_speed) && is_positive(cruise_throttle)) {
        throttle_base = _desired_speed * (cruise_throttle / cruise_speed);
    }

    // calculate final output
    float throttle_out = _throttle_speed_pid.update_all(_desired_speed, speed, (motor_limit_low || motor_limit_high || _throttle_limit_low || _throttle_limit_high));
    throttle_out += _throttle_speed_pid.get_ff();
    throttle_out += throttle_base;

    // update PID info for reporting purposes
    _throttle_speed_pid_info = _throttle_speed_pid.get_pid_info();
    _throttle_speed_pid_info.FF += throttle_base;

    // clear local limit flags used to stop i-term build-up as we stop reversed outputs going to motors
    _throttle_limit_low = false;
    _throttle_limit_high = false;

    // protect against reverse output being sent to the motors unless braking has been enabled
    if (!_brake_enable) {
        // if both desired speed and actual speed are positive, do not allow negative values
        if ((_desired_speed >= 0.0f) && (throttle_out <= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_low = true;
        } else if ((_desired_speed <= 0.0f) && (throttle_out >= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_high = true;
        }
    }

    // final output throttle in range -1 to 1
    return throttle_out;
}


float AR_AttitudeControl::get_throttle_out_speed_adrc(float desired_speed,float dt)
{
        // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // get speed forward
    float speed;
    if (!get_forward_speed(speed)) {
        // we expect caller will not try to control heading using rate control without a valid speed estimate
        // on failure to get speed we do not attempt to steer
        return 0.0f;
    }

    // if not called recently, reset input filter and desired speed to actual speed (used for accel limiting)
    if (!speed_control_active()) {
        _throttle_speed_adrc.reset_eso(speed);
        _desired_speed = speed;
    }
    _speed_last_ms = AP_HAL::millis();

    // acceleration limit desired speed
    _desired_speed = get_desired_speed_accel_limited(desired_speed, dt);

    // set ADRC's dt
    _throttle_speed_adrc.set_dt(dt);

    // calculate final output
    float throttle_out = _throttle_speed_adrc.update_all(desired_speed, speed);
 
    // clear local limit flags used to stop i-term build-up as we stop reversed outputs going to motors
    _throttle_limit_low = false;
    _throttle_limit_high = false;

    // protect against reverse output being sent to the motors unless braking has been enabled
    if (!_brake_enable) {
        // if both desired speed and actual speed are positive, do not allow negative values
        if ((desired_speed >= 0.0f) && (throttle_out <= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_low = true;
        } else if ((desired_speed <= 0.0f) && (throttle_out >= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_high = true;
        }
    }

    // final output throttle in range -1 to 1
    return throttle_out;
}


float AR_AttitudeControl::get_throttle_out_speed_mfac(float desired_speed,float dt)
{
    // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // get speed forward
    float speed;
    if (!get_forward_speed(speed)) {
        // we expect caller will not try to control heading using rate control without a valid speed estimate
        // on failure to get speed we do not attempt to steer
        return 0.0f;
    }

    // if not called recently, reset input filter and desired speed to actual speed (used for accel limiting)
    if (!speed_control_active()) {
        _throttle_speed_mfac.reset(speed);
        _desired_speed = speed;
    }
    _speed_last_ms = AP_HAL::millis();

    // acceleration limit desired speed
    _desired_speed = get_desired_speed_accel_limited(desired_speed, dt);

    // set MFAC's dt
    _throttle_speed_mfac.set_dt(dt);

    // calculate final output
    float throttle_out = _throttle_speed_mfac.update_all(desired_speed, speed,false);
 
    // clear local limit flags used to stop i-term build-up as we stop reversed outputs going to motors
    _throttle_limit_low = false;
    _throttle_limit_high = false;

    // protect against reverse output being sent to the motors unless braking has been enabled
    if (!_brake_enable) {
        // if both desired speed and actual speed are positive, do not allow negative values
        if ((desired_speed >= 0.0f) && (throttle_out <= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_low = true;
        } else if ((desired_speed <= 0.0f) && (throttle_out >= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_high = true;
        }
    }

    // final output throttle in range -1 to 1
    return throttle_out;
}

// return a throttle output from -1 to +1 to perform a controlled stop.  returns true once the vehicle has stopped
float AR_AttitudeControl::get_throttle_out_stop(bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt, bool &stopped)
{
    // get current system time
    const uint32_t now = AP_HAL::millis();

    // if we were stopped in the last 300ms, assume we are still stopped
    bool _stopped = (_stop_last_ms != 0) && (now - _stop_last_ms) < 300;

    // get deceleration limited speed
    float desired_speed_limited = get_desired_speed_accel_limited(0.0f, dt);

    // get speed forward
    float speed = 0.0f;
    if (!get_forward_speed(speed)) {
        // could not get speed so assume stopped
        _stopped = true;
    } else {
        // if desired speed is zero and vehicle drops below _stop_speed consider it stopped
        if (is_zero(desired_speed_limited) && fabsf(speed) <= fabsf(_stop_speed)) {
            _stopped = true;
        }
    }

    // set stopped status for caller
    stopped = _stopped;

    // if stopped return zero
    if (stopped) {
        // update last time we thought we were stopped
        _stop_last_ms = now;
        // set last time speed controller was run so accelerations are limited
        _speed_last_ms = now;
        // reset filters and I-term
        _throttle_speed_pid.reset_filter();
        _throttle_speed_pid.reset_I();
        _throttle_speed_adrc.reset_eso(speed);
        _throttle_speed_mfac.reset(speed);
        // ensure desired speed is zero
        _desired_speed = 0.0f;
        return 0.0f;
    }

    // clear stopped system time
    _stop_last_ms = 0;
    // run speed controller to bring vehicle to stop
    return get_throttle_out_speed(desired_speed_limited, motor_limit_low, motor_limit_high, cruise_speed, cruise_throttle, dt);
}

// balancebot pitch to throttle controller
// returns a throttle output from -100 to +100 given a desired pitch angle and vehicle's current speed (from wheel encoders)
// desired_pitch is in radians, veh_speed_pct is supplied as a percentage (-100 to +100) of vehicle's top speed
float AR_AttitudeControl::get_throttle_out_from_pitch(float desired_pitch, float vehicle_speed_pct, bool motor_limit_low, bool motor_limit_high, float dt)
{
    // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // if not called recently, reset input filter
    const uint32_t now = AP_HAL::millis();
    if ((_balance_last_ms == 0) || ((now - _balance_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        _pitch_to_throttle_pid.reset_filter();
        _pitch_to_throttle_pid.reset_I();
    }
    _balance_last_ms = now;

    // set PID's dt
    _pitch_to_throttle_pid.set_dt(dt);

    // add feed forward from speed
    float output = vehicle_speed_pct * 0.01f * _pitch_to_throttle_speed_ff;
    output += _pitch_to_throttle_pid.update_all(desired_pitch, AP::ahrs().pitch, (motor_limit_low || motor_limit_high));
    output += _pitch_to_throttle_pid.get_ff();

    // constrain and return final output
    return output;
}

// get latest desired pitch in radians for reporting purposes
float AR_AttitudeControl::get_desired_pitch() const
{
    // if not called recently, return 0
    if ((_balance_last_ms == 0) || ((AP_HAL::millis() - _balance_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }

    return _pitch_to_throttle_pid.get_pid_info().target;
}

// Sailboat heel(roll) angle controller releases sail to keep at maximum heel angle
// but does not attempt to reach maximum heel angle, ie only lets sails out, does not pull them in
float AR_AttitudeControl::get_sail_out_from_heel(float desired_heel, float dt)
{
    // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // if not called recently, reset input filter
    const uint32_t now = AP_HAL::millis();
    if ((_heel_controller_last_ms == 0) || ((now - _heel_controller_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        _sailboat_heel_pid.reset_filter();
        _sailboat_heel_pid.reset_I();
    }
    _heel_controller_last_ms = now;

    // set PID's dt
    _sailboat_heel_pid.set_dt(dt);

    _sailboat_heel_pid.update_all(desired_heel, fabsf(AP::ahrs().roll));

    // get feed-forward
    const float ff = _sailboat_heel_pid.get_ff();

    // get p, constrain to be zero or negative
    float p = _sailboat_heel_pid.get_p();
    if (is_positive(p)) {
        p = 0.0f;
    }

    // get i, constrain to be zero or negative
    float i = _sailboat_heel_pid.get_i();
    if (is_positive(i)) {
        i = 0.0f;
        _sailboat_heel_pid.reset_I();
    }

    // get d
    const float d = _sailboat_heel_pid.get_d();

    // constrain and return final output
    return (ff + p + i + d) * -1.0f;
}

// get forward speed in m/s (earth-frame horizontal velocity but only along vehicle x-axis).  returns true on success
bool AR_AttitudeControl::get_forward_speed(float &speed) const
{
    Vector3f velocity;
    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.get_velocity_NED(velocity)) {
        // use less accurate GPS, assuming entire length is along forward/back axis of vehicle
        if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            if (abs(wrap_180_cd(_ahrs.yaw_sensor - AP::gps().ground_course_cd())) <= 9000) {
                speed = AP::gps().ground_speed();
            } else {
                speed = -AP::gps().ground_speed();
            }
            return true;
        } else {
            return false;
        }
    }
    // calculate forward speed velocity into body frame
    speed = velocity.x*_ahrs.cos_yaw() + velocity.y*_ahrs.sin_yaw();
    return true;
}

float AR_AttitudeControl::get_decel_max() const
{
    if (is_positive(_throttle_decel_max)) {
        return _throttle_decel_max;
    } else {
        return _throttle_accel_max;
    }
}

// check if speed controller active
bool AR_AttitudeControl::speed_control_active() const
{
    // active if there have been recent calls to speed controller
    if ((_speed_last_ms == 0) || ((AP_HAL::millis() - _speed_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

// get latest desired speed recorded during call to get_throttle_out_speed.  For reporting purposes only
float AR_AttitudeControl::get_desired_speed() const
{
    // return zero if no recent calls to speed controller
    if (!speed_control_active()) {
        return 0.0f;
    }
    return _desired_speed;
}

// get acceleration limited desired speed
float AR_AttitudeControl::get_desired_speed_accel_limited(float desired_speed, float dt) const
{
    // return input value if no recent calls to speed controller
    // apply no limiting when ATC_ACCEL_MAX is set to zero
    if (!speed_control_active() || !is_positive(_throttle_accel_max)) {
        return desired_speed;
    }

    // sanity check dt
    dt = constrain_float(dt, 0.0f, 1.0f);

    // use previous desired speed as basis for accel limiting
    float speed_prev = _desired_speed;

    // if no recent calls to speed controller limit based on current speed
    if (!speed_control_active()) {
        get_forward_speed(speed_prev);
    }

    // acceleration limit desired speed
    float speed_change_max;
    if (fabsf(desired_speed) < fabsf(_desired_speed) && is_positive(_throttle_decel_max)) {
        speed_change_max = _throttle_decel_max * dt;
    } else {
        speed_change_max = _throttle_accel_max * dt;
    }
    return constrain_float(desired_speed, speed_prev - speed_change_max, speed_prev + speed_change_max);
}

// get minimum stopping distance (in meters) given a speed (in m/s)
float AR_AttitudeControl::get_stopping_distance(float speed) const
{
    // get maximum vehicle deceleration
    const float accel_max = get_accel_max();

    // avoid divide by zero
    if ((accel_max <= 0.0f) || is_zero(speed)) {
        return 0.0f;
    }

    // assume linear deceleration
    return 0.5f * sq(speed) / accel_max;
}

// relax I terms of throttle and steering controllers
void AR_AttitudeControl::relax_I()
{
    _steer_rate_pid.reset_I();
    _throttle_speed_pid.reset_I();
    _pitch_to_throttle_pid.reset_I();
}
