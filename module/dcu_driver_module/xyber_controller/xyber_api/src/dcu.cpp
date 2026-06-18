/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:31
 */

#include "internal/dcu.h"

#include <cstring>
#include <typeinfo>

// projects
#include "internal/common_utils.h"

#define COM_TIME_UP_MS 1000
#define MAX_CHANNEL_DEVICES 8
#define CHANNEL_BROADCAST_ID 0xFF

using namespace ethercat_manager;
using namespace std::chrono_literals;

namespace xyber {

Dcu::Dcu(std::string name, int32_t ecat_id) : EthercatNode(ecat_id), name_(name) {
  LOG_DEBUG("Dcu %s id %u created.", name.c_str(), id_);
  memset(&send_buf_, 0, sizeof(send_buf_));
  memset(&recv_buf_, 0, sizeof(recv_buf_));
}

Dcu::~Dcu() {
  for (auto& it : actuator_map_) {
    delete it.second;
  }
}

bool Dcu::Init() {
  // check io size
  if (sizeof(DcuSendPacket) != send_size_) {
    LOG_ERROR("CmdType %s size does not match. target %lu, actual %u", typeid(DcuSendPacket).name(),
              sizeof(DcuSendPacket), send_size_);
    return false;
  }
  if (sizeof(DcuRecvPacket) != recv_size_) {
    LOG_ERROR("StateType %s size does not match. target %lu, actual %u",
              typeid(DcuRecvPacket).name(), sizeof(DcuRecvPacket), recv_size_);
    return false;
  }

  return true;
}

void Dcu::RegisterActuator(Actuator* actr) {
  CtrlChannel ch = actr->GetCtrlChannel();
  if (ctrl_channel_map_[ch].size() >= MAX_CHANNEL_DEVICES) {
    LOG_ERROR("Channel %d has too many devices, %s register failed.", (int)ch,
              actr->GetName().c_str());
    return;
  }

  actr->SetDataFiled(send_buf_.canfd[(size_t)ch].data, recv_buf_.canfd[(size_t)ch]);
  actuator_map_[actr->GetName()] = actr;
  ctrl_channel_map_[ch].push_back(actr);
}

Actuator* Dcu::GetActautor(const std::string& name) {
  auto it = actuator_map_.find(name);
  if (it != actuator_map_.end()) {
    return it->second;
  }
  return nullptr;
}

void Dcu::SetChannelId(CtrlChannel ch, uint8_t id) {
  send_buf_.canfd[(int)ch].ctrl = id == CHANNEL_BROADCAST_ID ? id : 1 << (id - 1);
}

DcuImu Dcu::GetImuData() {
  std::lock_guard<std::mutex> lock(recv_mtx_);

  // TODO: imu data in buf is big endian
  DcuImu data;
  for (size_t i = 0; i < 3; i++) {
    data.acc[i] = xyber_utils::BytesToFloat((uint8_t*)&recv_buf_.imu.acc[i]);
    data.gyro[i] = xyber_utils::BytesToFloat((uint8_t*)&recv_buf_.imu.gyro[i]);
  }
  for (size_t i = 0; i < 4; i++) {
    data.quat[i] = xyber_utils::BytesToFloat((uint8_t*)&recv_buf_.imu.quat[i]);
  }
  return data;
}

void Dcu::ImuAppleUserOffset() {
  send_buf_.imu_cmd = 1;
  std::this_thread::sleep_for(30ms);
  send_buf_.imu_cmd = 0;
}

void Dcu::OnStateChangeHook(NodeState next) {
  LOG_DEBUG("Dcu %s %d, state change: %s -> %s", name_.c_str(), id_,
            NodeStateStr(node_state_).c_str(), NodeStateStr(next).c_str());
}

bool Dcu::EnableAllActuator() {
  for (const auto& [name, actr] : actuator_map_) {
    if (!EnableActuator(name)) return false;
  }
  return true;
}

bool Dcu::EnableActuator(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return false;

  if (actr->GetType() == ActuatorType::POWER_FLOW_L28 ||
      actr->GetType() == ActuatorType::OMNI_PICKER) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    actr->RequestState(STATE_ENABLE);
    SetChannelId(actr->GetCtrlChannel(), actr->GetId());
  }
  ActautorState ret = STATE_DISABLE;
  for (size_t i = 0; i < COM_TIME_UP_MS / 10; i++) {
    std::this_thread::sleep_for(10ms);

    std::lock_guard<std::mutex> lock(recv_mtx_);
    ret = actr->GetPowerState();
    if (ret == STATE_ENABLE) break;
  }
  if (ret == STATE_ENABLE) {
    LOG_DEBUG("Actuator %s %d enable success.", name.c_str(), (int)actr->GetId());
  } else {
    LOG_ERROR("Actuator %s %d enable failed.", name.c_str(), (int)actr->GetId());
  }
  return ret == STATE_ENABLE;
}

bool Dcu::DisableAllActuator() {
  for (const auto& [name, actr] : actuator_map_) {
    if (!DisableActuator(name)) return false;
  }
  return true;
}

bool Dcu::DisableActuator(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return false;

  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    actr->RequestState(STATE_DISABLE);
    SetChannelId(actr->GetCtrlChannel(), actr->GetId());
  }
  ActautorState ret = STATE_DISABLE;
  for (size_t i = 0; i < COM_TIME_UP_MS / 10; i++) {
    std::this_thread::sleep_for(10ms);

    std::lock_guard<std::mutex> lock(recv_mtx_);
    ret = actr->GetPowerState();
    if (ret == STATE_DISABLE) break;
  }

  return ret == STATE_DISABLE;
}

void Dcu::ClearError(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->ClearError();
  SetChannelId(actr->GetCtrlChannel(), actr->GetId());
}

void Dcu::SetHomingPosition(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetHomingPosition();
  SetChannelId(actr->GetCtrlChannel(), actr->GetId());
}

void Dcu::SaveConfig(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SaveConfig();
  SetChannelId(actr->GetCtrlChannel(), actr->GetId());
}

float Dcu::GetTempure(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return 0;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetTempure();
}

std::string Dcu::GetErrorString(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return "";

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetErrorString();
}

ActautorState Dcu::GetPowerState(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return STATE_DISABLE;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetPowerState();
}

bool Dcu::SetMode(const std::string& name, ActautorMode mode) {
  auto actr = GetActautor(name);
  if (!actr) return 0;

  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    actr->SetMode(mode);
    SetChannelId(actr->GetCtrlChannel(), actr->GetId());
  }

  for (size_t i = 0; i < COM_TIME_UP_MS / 10; i++) {
    std::this_thread::sleep_for(10ms);

    std::lock_guard<std::mutex> lock(recv_mtx_);
    if (actr->GetMode() == mode) return true;
  }
  return false;
}

ActautorMode Dcu::GetMode(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return MODE_CURRENT;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetMode();
}

void Dcu::SetEffort(const std::string& name, float cur) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetEffort(cur);
  SetChannelId(actr->GetCtrlChannel(), actr->GetId());
}

float Dcu::GetEffort(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return 0;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetEffort();
}

void Dcu::SetVelocity(const std::string& name, float vel) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetVelocity(vel);
  SetChannelId(actr->GetCtrlChannel(), actr->GetId());
}

float Dcu::GetVelocity(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return 0;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetVelocity();
}

void Dcu::SetPosition(const std::string& name, float pos) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetPosition(pos);
  SetChannelId(actr->GetCtrlChannel(), actr->GetId());
}

float Dcu::GetPosition(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return 0;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetPosition();
}

void Dcu::SetMitParam(const std::string& name, MitParam param) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetMitParam(param);
}

void Dcu::SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp,
                    float kd) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetMitCmd(pos, vel, effort, kp, kd);
  SetChannelId(actr->GetCtrlChannel(), CHANNEL_BROADCAST_ID);
}

}  // namespace xyber