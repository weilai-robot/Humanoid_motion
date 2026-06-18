/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:52
 */

#pragma once

// projects
#include "internal/actuator_base.h"

namespace xyber_omni_picker {

class OmniPicker : public xyber::Actuator {
 public:
  explicit OmniPicker(std::string name, uint8_t id, xyber::CtrlChannel ctrl_ch)
      : Actuator(name, xyber::ActuatorType::OMNI_PICKER, id, ctrl_ch) {}
  ~OmniPicker() {}

 public:
  virtual float GetEffort() override { return state_data_->cur / 255.0f; }
  virtual float GetVelocity() override { return state_data_->vel / 255.0f; }
  virtual float GetPosition() override { return state_data_->pos / 255.0f; }
  virtual void SetMitCmd(float pos, float, float cur, float, float) override {
    if (pos > 1) {
      pos = 1;
    }
    if (cur > 1) {
      cur = 1;
    }
    cmd_data_->pos = 0xFF * pos;
    cmd_data_->cur = 0xFF * cur;
    cmd_data_->speed = 0xFF;  // TODO: hardcoding for claw, cause it` using different protocol
    cmd_data_->acc = 0xFF;
    cmd_data_->dcc = 0xFF;
  }

 private:
  virtual void SetDataFiled(uint8_t* send, uint8_t* recv) override {
    send_buf_ = send + ACTUATOR_FRAME_SIZE * (id_ - 1);
    recv_buf_ = recv + ACTUATOR_FRAME_SIZE * (id_ - 1);

    cmd_data_ = (CmdData*)send_buf_;
    state_data_ = (StateData*)recv_buf_;
  }

 private:
  struct CmdData {
    uint8_t reserved_1 = 0;
    uint8_t pos = 0;
    uint8_t cur = 0;
    uint8_t speed = 0;
    uint8_t acc = 0;
    uint8_t dcc = 0;
    uint16_t reserved_2 = 0;
  };

  struct StateData {
    uint8_t error_code = 0;
    uint8_t state = 0;
    uint8_t pos = 0;
    uint8_t vel = 0;
    uint8_t cur = 0;
    uint8_t reserved_1 = 0;
    uint16_t reserved_2 = 0;
  };

  CmdData* cmd_data_ = nullptr;
  StateData* state_data_ = nullptr;
};

}  // namespace xyber_omni_picker