/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:59
 */

#pragma once

// projects
#include "internal/actuator_base.h"

#define PF_L28_MAX_CUR (2.5f)

namespace xyber_power_flow {

enum ActuatorCmd : uint8_t {
  CMD_REQUEST_STATE = 0x01,
  CMD_CLEAR_ERROR = 0x02,
  CMD_SET_HOMING_POSITON = 0x03,
  CMD_CLAC_CURRENT_GAIN = 0x04,
  CMD_SET_MODE = 0x0B,

  CMD_INVERT_DIRECTION = 0x1D,

  CMD_SET_MIT_MIN_POS = 0x2A,
  CMD_SET_MIT_MAX_POS = 0x2B,
  CMD_SET_MIT_MIN_VEL = 0x2C,
  CMD_SET_MIT_MAX_VEL = 0x2D,
  CMD_SET_MIT_MIN_TOR = 0x2E,
  CMD_SET_MIT_MAX_TOR = 0x2F,
  CMD_SET_MIT_MIN_KP = 0x30,
  CMD_SET_MIT_MAX_KP = 0x31,
  CMD_SET_MIT_MIN_KD = 0x32,
  CMD_SET_MIT_MAX_KD = 0x33,

  CMD_SET_POS = 0x65,
  CMD_SET_VEL = 0x66,
  CMD_SET_CUR = 0x67,

  CMD_SET_CAN_ID = 0xC9,
  CMD_SAVE_CONFIG = 0xCC,
};

class PowerFlowR : public xyber::Actuator {
 public:
  explicit PowerFlowR(xyber::ActuatorType type, std::string name, uint8_t id,
                      xyber::CtrlChannel ctrl_ch, xyber::MitParam param)
      : Actuator(name, type, id, ctrl_ch), mit_param_(param) {}
  ~PowerFlowR() {}

 public:
  virtual void RequestState(xyber::ActautorState state);
  virtual void ClearError() override;
  virtual void SaveConfig() override;
  virtual void SetHomingPosition() override;

  virtual void SetMode(xyber::ActautorMode mode) override;
  virtual std::string GetErrorString() override;
  virtual xyber::ActautorState GetPowerState() override;

  virtual void SetEffort(float cur) override;
  virtual float GetEffort() override;
  virtual void SetVelocity(float vel) override;
  virtual float GetVelocity() override;
  virtual void SetPosition(float pos) override;
  virtual float GetPosition() override;

  virtual void SetMitParam(xyber::MitParam param) override { mit_param_ = param; };
  virtual void SetMitCmd(float pos, float vel, float toq, float kp, float kd) override;

 private:
  virtual void SetDataFiled(uint8_t* send, uint8_t* recv) override;

  int MitFloatToUint(float x, float x_min, float x_max, int bits);
  float MitUintToFloat(int x_int, float x_min, float x_max, int bits);

 private:
  struct MitCmd {
    uint64_t pos : 16;
    uint64_t vel : 12;
    uint64_t kp : 12;
    uint64_t kd : 12;
    uint64_t tor : 12;
  };

#pragma pack(1)
  struct StateData {
    uint16_t pos;
    uint32_t vel : 12;
    uint32_t cur : 12;
    uint32_t heartbeat : 1;
    uint32_t state : 3;
    uint32_t error : 4;
    uint16_t temp;
  };
#pragma pack()

  xyber::MitParam mit_param_;
  MitCmd* mit_cmd_ = nullptr;
  StateData* state_data_ = nullptr;
};

class PowerFlowL : public xyber::Actuator {
 public:
  explicit PowerFlowL(std::string name, uint8_t id, xyber::CtrlChannel ctrl_ch)
      : Actuator(name, xyber::ActuatorType::POWER_FLOW_L28, id, ctrl_ch) {}
  ~PowerFlowL() {}

 public:
  virtual void SetEffort(float cur) override { cmd_data_->cur = cur; }
  virtual float GetEffort() override { return state_data_->cur; }
  virtual void SetPosition(float pos) override { cmd_data_->pos = pos; }
  virtual float GetPosition() override { return state_data_->pos; }
  virtual void SetMitCmd(float pos, float, float, float, float) override { cmd_data_->pos = pos; }

 private:
  virtual void SetDataFiled(uint8_t* send, uint8_t* recv) override {
    send_buf_ = send + ACTUATOR_FRAME_SIZE * (id_ - 1);
    recv_buf_ = recv + ACTUATOR_FRAME_SIZE * (id_ - 1);

    cmd_data_ = (L28Data*)send_buf_;
    state_data_ = (L28Data*)recv_buf_;

    cmd_data_->cur = PF_L28_MAX_CUR;  // Give a default cur
  }

 private:
  struct L28Data {
    float pos = 0;
    float cur = 0;
  };

  L28Data* cmd_data_ = nullptr;
  L28Data* state_data_ = nullptr;
};

}  // namespace xyber_power_flow