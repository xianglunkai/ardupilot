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
/*
 * AP_BCM.cpp
 *
 *      Author: xiang lunkai
 */

#include "AP_BCM.h"

#if AP_BCM_ENABLED
#include <stdio.h>
#include <string.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/AP_Math.h>    
#include <RC_Channel/RC_Channel.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Notify/AP_Notify.h>

#include <GCS_MAVLink/GCS.h>

#define AP_BCM_DEBUG 1

extern const AP_HAL::HAL & hal;
// parameters
const AP_Param::GroupInfo AP_BCM::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Engine control
    // @Description: This enables internal combustion engine control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_BCM, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: GEAR_DZ
    // @DisplayName: gear control deadzone, range[0, 100]
    // @Description: protect motor gear switch from F/R to R/F
    AP_GROUPINFO("GEAR_DZ", 1, AP_BCM, _gear_dz, 5),

    AP_GROUPEND
};

// singleton instance
AP_BCM *AP_BCM::_singleton;

AP_BCM::AP_BCM()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_BCM must be singleton");
    }
#endif
    _singleton = this;
}


void AP_BCM::init()
{
    if (_driver != nullptr || !_enable) {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::BCM) {
            _driver = NEW_NOTHROW AP_BCM_Driver();
            return;
        }
    }
}

void AP_BCM::update()
{
    // exit if not enabled
    if (_driver == nullptr || !_enable) {
        return;
    }

      // enginer control
    _driver->update(_gear_dz);
}

// add interface for AP_SCU
void AP_BCM::set_control_commands(const float throttle, const uint8_t lift, const uint8_t sts)
{
	if (_driver == nullptr || !_enable) {
		return;
	}

	// throttle 
    uint16_t throttle_open = AP_BCM_Driver::BCM_TO_ECM_CMD_FRAME1_THR_IDEL;
    uint16_t gear_cmd = static_cast<uint16_t>(AP_BCM_Driver::Gear_Pos::N);
    uint16_t hand_pos = AP_BCM_Driver::BCM_TO_ECU_CMD_FRAME3_GEAR_IDEL;

    if (throttle > _gear_dz * 0.01f) {
        throttle_open = linear_interpolate(
        AP_BCM_Driver::BCM_TO_ECM_CMD_FRAME1_THR_IDEL,
        AP_BCM_Driver::BCM_TO_ECM_CMD_FRAME1_THR_FORWARD_MAX,
        throttle,
        _gear_dz * 0.01f,
        1.0f);

        gear_cmd = static_cast<uint16_t>(AP_BCM_Driver::Gear_Pos::F);

        hand_pos = AP_BCM_Driver::BCM_TO_ECU_CMD_FRAME3_GEAR_FORWARD;


    } else if (throttle < -_gear_dz * 0.01f) {
        throttle_open = linear_interpolate(
        AP_BCM_Driver::BCM_TO_ECM_CMD_FRAME1_THR_BACKWARD_MAX,
        AP_BCM_Driver::BCM_TO_ECM_CMD_FRAME1_THR_IDEL,
        throttle,
        -1.0f,
        -_gear_dz * 0.01f);

        gear_cmd = static_cast<uint16_t>(AP_BCM_Driver::Gear_Pos::R);

        hand_pos = AP_BCM_Driver::BCM_TO_ECU_CMD_FRAME3_GEAR_BACKWARD;

    } else {
        hand_pos = AP_BCM_Driver::BCM_TO_ECU_CMD_FRAME3_GEAR_IDEL;
    }
 

	// see doc
	uint16_t control = throttle_open | gear_cmd <<10 | lift << 12 | sts << 14;
	{
		WITH_SEMAPHORE(_driver->get_semaphore());
		_driver->_bcm_to_ecu_cmd.data1.control = ((control & 0xff) << 8) | ((control &0xff00) >> 8);
		_driver->_bcm_to_ecu_cmd.data1.control2 = ((control & 0xff) << 8) | ((control &0xff00) >> 8);
		_driver->_bcm_to_ecu_cmd.data1.control3 = ((control & 0xff) << 8) | ((control &0xff00) >> 8);
		_driver->_bcm_to_ecu_cmd3.data3.byte1_2 = ((hand_pos & 0xff) << 8) | ((hand_pos &0xff00) >> 8);
	}
}

namespace AP {
    AP_BCM *bcm()
    {
        return AP_BCM::get_singleton();
    }
};

/*
 * AP_BCM_Driver class 
*/

AP_BCM_Driver::AP_BCM_Driver() : CANSensor("BCM")
{
  register_driver(AP_CAN::Protocol::BCM);

  // init _bcm_to_ecu_cmd3
  _bcm_to_ecu_cmd3.data3.frame3_head = 0x08;
  _bcm_to_ecu_cmd3.data3.byte1_2 = 0x916e;
  _bcm_to_ecu_cmd3.data3.byte3 =0;
  _bcm_to_ecu_cmd3.data3.byte4 =0;
  _bcm_to_ecu_cmd3.data3.byte5 =0x00;
  _bcm_to_ecu_cmd3.data3.byte6 =0x61;
  _bcm_to_ecu_cmd3.data3.byte7 =0x08;

  // init _bcm_to_ecu_cmd2
  _bcm_to_ecu_cmd2.data2.frame2_head = 0x04;
  _bcm_to_ecu_cmd2.data2.control = 0xcd00;
  _bcm_to_ecu_cmd2.data2.reserved = 0x00;

  // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
  //hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_BCM_Driver::loop, void), "BCM", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);

  // register 1kHz timer callback
  hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_BCM_Driver::timer_update, void));   

}

void AP_BCM_Driver::update(const int16_t gear_dz)
{
     // 1. engine start and stop control
    // DISARM and ARMED
    uint16_t cvalue = 1500;
    uint16_t sts_cmd =  static_cast<uint16_t>(Engine_Cmd::NO_OPS);

    // start command
    RC_Channel *c_start = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARMDISARM);
    if (c_start != nullptr && rc().has_valid_input()) {
        // get starter control channel
        cvalue = c_start->get_radio_in();

        if (cvalue <1000 || cvalue > 2000) {
            sts_cmd = static_cast<uint16_t>(Engine_Cmd::NO_OPS);

            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Radio Failed For STS!");

        } else {
            if (cvalue >= RC_Channel::AUX_PWM_TRIGGER_HIGH && hal.util->get_soft_armed()) {
                sts_cmd = static_cast<uint16_t>(Engine_Cmd::REQ_START);
                        
                if (AP_BCM_DEBUG) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Staring Enginer");
                } 

            }else if (cvalue <= RC_Channel::AUX_PWM_TRIGGER_LOW) {
                sts_cmd = static_cast<uint16_t>(Engine_Cmd::REQ_STOP);

            if (AP_BCM_DEBUG) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Stoping Enginer");
                }

            } else {}
        }
    }

    // throttle 
    const float throttle = constrain_float(SRV_Channels::get_output_norm(SRV_Channel::k_throttle), -1.0f, 1.0f);
    uint16_t throttle_open = BCM_TO_ECM_CMD_FRAME1_THR_IDEL;
    uint16_t gear_cmd = static_cast<uint16_t>(Gear_Pos::N);
    uint16_t hand_pos = BCM_TO_ECU_CMD_FRAME3_GEAR_IDEL;

    if (throttle > gear_dz * 0.01f) {
        throttle_open = linear_interpolate(
        BCM_TO_ECM_CMD_FRAME1_THR_IDEL,
        BCM_TO_ECM_CMD_FRAME1_THR_FORWARD_MAX,
        throttle,
        gear_dz * 0.01f,
        1.0f);

        gear_cmd = static_cast<uint16_t>(Gear_Pos::F);

        hand_pos = BCM_TO_ECU_CMD_FRAME3_GEAR_FORWARD;


    } else if (throttle < -gear_dz * 0.01f) {
        throttle_open = linear_interpolate(
        BCM_TO_ECM_CMD_FRAME1_THR_BACKWARD_MAX,
        BCM_TO_ECM_CMD_FRAME1_THR_IDEL,
        throttle,
        -1.0f,
        -gear_dz * 0.01f);

        gear_cmd = static_cast<uint16_t>(Gear_Pos::R);

        hand_pos = BCM_TO_ECU_CMD_FRAME3_GEAR_BACKWARD;

    } else {
        hand_pos = BCM_TO_ECU_CMD_FRAME3_GEAR_IDEL;
    }
 
    // lift
    uint16_t lift_open = static_cast<uint16_t>(Lift_Pos::NORMAL);
    int16_t lift = 1500;

    if(!hal.util->get_soft_armed()) {
        RC_Channel *c = rc().channel(3);

        if (c != nullptr && rc().has_valid_input()) {
            // get starter control channel
            lift = c->get_radio_in();

            if (lift <1000 || lift > 2000) {
                lift_open = static_cast<uint16_t>(Lift_Pos::NORMAL);

                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Radio Failed For Lift!");

            } else {
                if (lift >= RC_Channel::AUX_SWITCH_PWM_TRIGGER_HIGH) {

                    lift_open = static_cast<uint16_t>(Lift_Pos::UP);
                                
                    if (AP_BCM_DEBUG) {
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Lift UP");
                    }

                } else if (lift <= RC_Channel::AUX_SWITCH_PWM_TRIGGER_LOW) {

                    lift_open = static_cast<uint16_t>(Lift_Pos::DOWN);

                    if (AP_BCM_DEBUG) {
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Lift DOWN");
                    }

                } else {}
            }
        }  
    }

    // copy this data 
    {
      WITH_SEMAPHORE(_rsem);
      uint16_t control = throttle_open | gear_cmd <<10 | lift_open << 12 | sts_cmd << 14;
      _bcm_to_ecu_cmd.data1.control = ((control & 0xff) << 8) | ((control &0xff00) >> 8);
      _bcm_to_ecu_cmd.data1.control2 = ((control & 0xff) << 8) | ((control &0xff00) >> 8);
      _bcm_to_ecu_cmd.data1.control3 = ((control & 0xff) << 8) | ((control &0xff00) >> 8);
      _bcm_to_ecu_cmd3.data3.byte1_2 = ((hand_pos & 0xff) << 8) | ((hand_pos &0xff00) >> 8);
    }

}

// handle for incoming frames
void AP_BCM_Driver::handle_frame(AP_HAL::CANFrame & frame)
{
  if (frame.isExtended()) {
    return;
  }

  uint32_t id = frame.id & AP_HAL::CANFrame::MaskStdID;

  switch (CAN_ID(id)) {

      case CAN_ID::ECU_TO_BCM_STS: // 0x303
      {
        memcpy(_ecu_to_bcm_sts.raw_data, frame.data, frame.dlc);
        _received_ecu_status = true;
        _update_ecu_status_ms =  AP_HAL::millis();

        // update rpm
        const float rpm = _ecu_to_bcm_sts.data.rpm * 4.0f;
        update_esc_rpm(0, rpm);

        break;
      }
      
      case CAN_ID::ECU_TO_BCM_REQ: // 0x7e3
      {
        memcpy(_ecu_to_bcm_req.raw_data, frame.data, frame.dlc);
        _received_ecu_request = true;

        break;
      }

      case CAN_ID::BCM_TO_ECU_ACK:
      {
        break;
      }

      case CAN_ID::BCM_TO_ECU_CMD:
      {
        break;
      }
  }

}

// loop to send output to ESCs in background thread
void AP_BCM_Driver::loop()
{
  // deal with control commands
  const uint32_t LOOP_INTERVAL_US = 1000;  // 1ms

  // handshake
  while (!_handshake_success) {
    if (_received_ecu_request && _received_ecu_status) {
      _handshake_success = true;
      break;
#if AP_BCM_DEBUG
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_BCM_Driver: handshare success!");
#endif
    }

    if (!_received_ecu_request) {
      hal.scheduler->delay_microseconds(1000);
    } else {
      // send ack
      hal.scheduler->delay_microseconds(5000);
      _bcm_to_ecu_ack.data.byte3 = _ecu_to_bcm_req.data.byte1;
      _bcm_to_ecu_ack.data.byte4 = _ecu_to_bcm_req.data.byte2;
      _bcm_to_ecu_ack.data.byte5 = _ecu_to_bcm_req.data.byte3;
      send_handshake_ack_messages(_bcm_to_ecu_ack);
    }
  }

  while (true) {

    // BCM_TO_ECU_CMD_FRAME1_LOOP_TIME_MS*t for sending frame1
    {
        WITH_SEMAPHORE(_rsem);
        if (_clock_counter % BCM_TO_ECU_CMD_FRAME1_LOOP_TIME_MS == 0) {
          send_control_cmd_messages(_bcm_to_ecu_cmd);
        }
      

      // BCM_TO_ECU_CMD_FRAME2_LOOP_TIME_MS *t + 1 for sending frame2
      if (_clock_counter % BCM_TO_ECU_CMD_FRAME2_LOOP_TIME_MS == 1) {
          send_control_cmd_messages(_bcm_to_ecu_cmd2);
      }

      // BCM_TO_ECU_CMD_FRAME2_LOOP_TIME_MS *t + 4 for sending frame3
      if (_clock_counter % BCM_TO_ECU_CMD_FRAME3_LOOP_TIME_MS == 4) {

        if (++_change_counter >= 5) {
          _change_counter = 0;
          _bcm_to_ecu_cmd3.data3.byte5 = (_bcm_to_ecu_cmd3.data3.byte5 == 0x00)?(0x04):0x00;
        }
        send_control_cmd_messages(_bcm_to_ecu_cmd3);
      }
    }

    _clock_counter++;

    // ensure fixed time
    hal.scheduler->delay_microseconds(LOOP_INTERVAL_US);
    
  } // end while
}

void AP_BCM_Driver::timer_update()
{
  // rehandshake when time out 3s for updating ecu status messages
  uint32_t now = AP_HAL::millis();
  if ((_update_ecu_status_ms == 0 || (now - _update_ecu_status_ms) > 3000) && _handshake_success) {
    _handshake_success = false;
    _received_ecu_request = false;
    _received_ecu_status = false;
  }

  // handshake
  if (!_handshake_success) {
    if (_received_ecu_request && _received_ecu_status) {
      _handshake_success = true;

      #if AP_BCM_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_BCM_Driver: handshare success!");
      #endif
    }
    // response ECU request
    if (_received_ecu_request && !_received_ecu_status && ++_counter >= 5) {
      _bcm_to_ecu_ack.data.byte3 = _ecu_to_bcm_req.data.byte1;
      _bcm_to_ecu_ack.data.byte4 = _ecu_to_bcm_req.data.byte2;
      _bcm_to_ecu_ack.data.byte5 = _ecu_to_bcm_req.data.byte3;
      send_handshake_ack_messages(_bcm_to_ecu_ack);
      _counter = 0;
    }
  } 
  
  {
      // BCM_TO_ECU_CMD_FRAME1_LOOP_TIME_MS*t for sending frame1
      {
        WITH_SEMAPHORE(_rsem);
        if (_clock_counter % BCM_TO_ECU_CMD_FRAME1_LOOP_TIME_MS == 0) {
          send_control_cmd_messages(_bcm_to_ecu_cmd);
        }
      

        // BCM_TO_ECU_CMD_FRAME2_LOOP_TIME_MS *t + 1 for sending frame2
        if (_clock_counter % BCM_TO_ECU_CMD_FRAME2_LOOP_TIME_MS == 1) {
            send_control_cmd_messages(_bcm_to_ecu_cmd2);
        }


        // BCM_TO_ECU_CMD_FRAME2_LOOP_TIME_MS *t + 4 for sending frame3
        if (_clock_counter % BCM_TO_ECU_CMD_FRAME3_LOOP_TIME_MS == 4) {

          if (++_change_counter >= 5) {
            _change_counter = 0;
            _bcm_to_ecu_cmd3.data3.byte5 = (_bcm_to_ecu_cmd3.data3.byte5 == 0x00)?(0x04):0x00;
          }

          send_control_cmd_messages(_bcm_to_ecu_cmd3);
        }
      }

    _clock_counter++;
  }

}

// update engine speed
void AP_BCM_Driver::update_esc_rpm(uint8_t telem_esc_index, float rpm)
{
  #if HAL_WITH_ESC_TELEM
    update_rpm(telem_esc_index, rpm);
  #endif
}

// send BCM to ECU ACK
void AP_BCM_Driver::send_handshake_ack_messages(const BCM_TO_ECU_ACK_Data & mode_select)
{
    AP_HAL::CANFrame txFrame;
    uint32_t id = (uint32_t)(CAN_ID::BCM_TO_ECU_ACK);
    txFrame = {id, mode_select.raw_data, sizeof(mode_select.raw_data)};
    write_frame(txFrame, 10000ULL);
}

// send an engine control commands over can
void AP_BCM_Driver::send_control_cmd_messages(const BCM_TO_ECU_CMD_Data & control_cmd)
{
    AP_HAL::CANFrame txFrame;
    uint32_t id = (uint32_t)(CAN_ID::BCM_TO_ECU_CMD);
    txFrame = {id, control_cmd.raw_data, sizeof(control_cmd.raw_data)};
    write_frame(txFrame, 1000ULL);
}


#endif