/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:11
 */

#pragma once

// cpp
#include <string>

// projects
#include "common_type.h"

#define ACTUATOR_FRAME_SIZE 8

namespace xyber {

class Actuator {
 public:
  explicit Actuator(std::string name, ActuatorType type, uint8_t id, CtrlChannel ctrl_ch)
      : id_(id), type_(type), name_(name), ctrl_ch_(ctrl_ch) {}
  virtual ~Actuator() {}

  uint8_t GetId() { return id_; }
  std::string GetName() { return name_; }
  CtrlChannel GetCtrlChannel() { return ctrl_ch_; }
  ActuatorType GetType() { return type_; }
  virtual void SetDataFiled(uint8_t* send, uint8_t* recv) {
    send_buf_ = send + ACTUATOR_FRAME_SIZE * (id_ - 1);
    recv_buf_ = recv + ACTUATOR_FRAME_SIZE * (id_ - 1);
  }

 public:  // Actuator base function
  virtual void RequestState(ActautorState state) {}
  virtual void ClearError() {}
  virtual void SetHomingPosition() {}
  virtual void SaveConfig() {}
  virtual void SetMode(ActautorMode mode) {}
  virtual float GetTempure() { return 0.0f; }
  virtual std::string GetErrorString() { return "None"; }
  virtual ActautorMode GetMode() { return MODE_CURRENT; }
  virtual ActautorState GetPowerState() { return STATE_DISABLE; }

  virtual void SetEffort(float cur) {}
  virtual float GetEffort() { return 0.0f; }
  virtual void SetVelocity(float vel) {}
  virtual float GetVelocity() { return 0.0f; }
  virtual void SetPosition(float pos) {}
  virtual float GetPosition() { return 0.0f; }

  virtual void SetMitParam(MitParam param) {}
  virtual void SetMitCmd(float pos, float vel, float toq, float kp, float kd) {}

 protected:
  uint8_t id_;
  std::string name_;

  uint8_t* send_buf_ = nullptr;
  uint8_t* recv_buf_ = nullptr;

  CtrlChannel ctrl_ch_;
  ActuatorType type_;
};

}  // namespace xyber