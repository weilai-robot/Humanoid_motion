/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:46
 */

#include <string>

// projects
#include "common_type.h"
#include "internal/common_utils.h"
#include "internal/power_flow.h"

using namespace xyber;

namespace xyber_power_flow {

void PowerFlowR::SetDataFiled(uint8_t* send, uint8_t* recv) {
  send_buf_ = send + ACTUATOR_FRAME_SIZE * (id_ - 1);
  recv_buf_ = recv + ACTUATOR_FRAME_SIZE * (id_ - 1);

  mit_cmd_ = (MitCmd*)send_buf_;
  state_data_ = (StateData*)recv_buf_;
}

int PowerFlowR::MitFloatToUint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float PowerFlowR::MitUintToFloat(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void PowerFlowR::RequestState(ActautorState state) {
  send_buf_[0] = CMD_REQUEST_STATE;
  send_buf_[1] = state;
};

ActautorState PowerFlowR::GetPowerState() { return (ActautorState)state_data_->state; }

void PowerFlowR::ClearError() { send_buf_[0] = CMD_CLEAR_ERROR; }

void PowerFlowR::SetHomingPosition() { send_buf_[0] = CMD_SET_HOMING_POSITON; }

void PowerFlowR::SaveConfig() {
  send_buf_[0] = CMD_SAVE_CONFIG;
  send_buf_[1] = 123;  // Magic number for PowerFlowR
}

std::string PowerFlowR::GetErrorString() { return "None"; }

void PowerFlowR::SetMode(ActautorMode mode) {
  send_buf_[0] = CMD_SET_MODE;
  send_buf_[1] = mode;
}

void PowerFlowR::SetEffort(float cur) {
  send_buf_[0] = CMD_SET_CUR;
  xyber_utils::FloatToBytes(cur, send_buf_ + 1);
}

float PowerFlowR::GetEffort() {
  uint16_t toq = (recv_buf_[3] & 0xF) << 8 | recv_buf_[4];
  return MitUintToFloat(toq, mit_param_.toq_min, mit_param_.toq_max, 12);
}

void PowerFlowR::SetVelocity(float vel) {
  send_buf_[0] = CMD_SET_VEL;
  xyber_utils::FloatToBytes(vel, send_buf_ + 1);
}

float PowerFlowR::GetVelocity() {
  uint16_t vel = recv_buf_[2] << 4 | recv_buf_[3] >> 4;
  return MitUintToFloat(vel, mit_param_.vel_min, mit_param_.vel_max, 12);
}

void PowerFlowR::SetPosition(float pos) {
  send_buf_[0] = CMD_SET_POS;
  xyber_utils::FloatToBytes(pos, send_buf_ + 1);
}

float PowerFlowR::GetPosition() {
  uint16_t pos = recv_buf_[0] << 8 | recv_buf_[1];
  return MitUintToFloat(pos, mit_param_.pos_min, mit_param_.pos_max, 16);
}

void PowerFlowR::SetMitCmd(float pos, float vel, float toq, float kp, float kd) {
  int pos_tmp = MitFloatToUint(pos, mit_param_.pos_min, mit_param_.pos_max, 16);
  int vel_tmp = MitFloatToUint(vel, mit_param_.vel_min, mit_param_.vel_max, 12);
  int tor_tmp = MitFloatToUint(toq, mit_param_.toq_min, mit_param_.toq_max, 12);
  int kp_tmp = MitFloatToUint(kp, mit_param_.kp_min, mit_param_.kp_max, 12);
  int kd_tmp = MitFloatToUint(kd, mit_param_.kd_min, mit_param_.kd_max, 12);

  send_buf_[0] = pos_tmp >> 8;
  send_buf_[1] = pos_tmp;
  send_buf_[2] = vel_tmp >> 4;
  send_buf_[3] = (vel_tmp & 0xF) << 4 | kp_tmp >> 8;
  send_buf_[4] = kp_tmp;
  send_buf_[5] = (kd_tmp >> 4);
  send_buf_[6] = (kd_tmp & 0xF) << 4 | tor_tmp >> 8;
  send_buf_[7] = tor_tmp;
}

}  // namespace xyber_power_flow