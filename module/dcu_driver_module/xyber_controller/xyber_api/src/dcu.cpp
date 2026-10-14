/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:31
 */

#include "internal/dcu.h"

#include <cstring>
#include <algorithm>
#include <chrono>
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

bool Dcu::RegisterActuator(Actuator* actr) {
  if (!actr || actr->GetId() == 0 || actr->GetId() > MAX_CHANNEL_DEVICES ||
      static_cast<unsigned>(actr->GetCtrlChannel()) > 2 ||
      actuator_map_.find(actr->GetName()) != actuator_map_.end()) return false;
  CtrlChannel ch = actr->GetCtrlChannel();
  if (ctrl_channel_map_[ch].size() >= MAX_CHANNEL_DEVICES) {
    LOG_ERROR("Channel %d has too many devices, %s register failed.", (int)ch,
              actr->GetName().c_str());
    return false;
  }
  for (auto* existing : ctrl_channel_map_[ch]) {
    if (existing->GetId() == actr->GetId()) return false;
  }

  actr->SetDataFiled(send_buf_.canfd[(size_t)ch].data, recv_buf_.canfd[(size_t)ch]);
  actuator_map_[actr->GetName()] = actr;
  ctrl_channel_map_[ch].push_back(actr);
  return true;
}

namespace {
int64_t SteadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
float Decode(unsigned value, float lo, float hi, unsigned max_code) {
  return lo + value * (hi-lo) / max_code;
}
}  // namespace

void Dcu::PushRecvData(const uint8_t* buf, uint16_t len) {
  if (!buf || len != sizeof(DcuRecvPacket)) return;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  const int64_t now = SteadyNs();
  for (const auto& [name, actr] : actuator_map_) {
    const auto ch = static_cast<size_t>(actr->GetCtrlChannel());
    const auto id = static_cast<size_t>(actr->GetId()-1);
    auto& tracker = rx_tracker_[ch][id];
    const auto* next = buf + ch*64 + id*8;
    const auto* previous = recv_buf_.canfd[ch] + id*8;
    if (!tracker.seen || std::memcmp(next, previous, 8) != 0) {
      ++tracker.content_seq;
      tracker.content_ns = now;
    }
    const size_t measurement_bytes =
        actr->GetType() == ActuatorType::POWER_FLOW_R86 ||
        actr->GetType() == ActuatorType::POWER_FLOW_R52 ? 5 : 8;
    if (!tracker.seen || std::memcmp(next, previous, measurement_bytes) != 0) {
      ++tracker.measurement_seq;
      tracker.measurement_ns = now;
    }
    tracker.seen = true;
  }
  std::memcpy(&recv_buf_, buf, sizeof(recv_buf_));
  ++rx_pdo_seq_;
  rx_pdo_ns_ = now;
}

void Dcu::PullSendData(uint8_t* buf, uint16_t len) {
  if (!buf || len != sizeof(DcuSendPacket)) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  std::memcpy(buf, &send_buf_, sizeof(send_buf_));
  last_sent_ = send_buf_;
  std::memcpy(last_command_seq_, command_seq_, sizeof(command_seq_));
  std::memcpy(last_command_format_, command_format_, sizeof(command_format_));
  ++tx_pdo_seq_;
  tx_pdo_ns_ = SteadyNs();
}

void Dcu::MarkCommand(Actuator* actr, uint8_t format) {
  const auto ch = static_cast<size_t>(actr->GetCtrlChannel());
  const auto id = static_cast<size_t>(actr->GetId()-1);
  ++command_seq_[ch][id];
  command_format_[ch][id] = format;
}

bool Dcu::GetActuatorState(const std::string& name, ActuatorStateSnapshot& state) {
  auto* actr = GetActautor(name);
  if (!actr) return false;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  state = {actr->GetPosition(), actr->GetVelocity(), actr->GetEffort(), rx_pdo_ns_};
  return true;
}

bool Dcu::GetActuatorDiagnostics(const std::string& name, ActuatorDiagnostics& data) {
  auto* actr = GetActautor(name);
  if (!actr) return false;
  std::scoped_lock lock(send_mtx_, recv_mtx_);
  data = ActuatorDiagnostics{};
  const auto ch = static_cast<size_t>(actr->GetCtrlChannel());
  const auto id = static_cast<size_t>(actr->GetId()-1);
  const auto& tracker = rx_tracker_[ch][id];
  data.ecat_id = id_; data.channel = ch+1; data.can_id = id+1; data.type = actr->GetType();
  data.rx_pdo_seq = rx_pdo_seq_; data.rx_pdo_ns = rx_pdo_ns_;
  data.rx_content_seq = tracker.content_seq; data.rx_content_ns = tracker.content_ns;
  data.rx_measurement_seq = tracker.measurement_seq; data.rx_measurement_ns = tracker.measurement_ns;
  data.rx_seen = tracker.seen;
  data.tx_pdo_seq = tx_pdo_seq_; data.tx_pdo_ns = tx_pdo_ns_;
  data.tx_command_seq = last_command_seq_[ch][id];
  data.tx_format = last_command_format_[ch][id]; data.tx_mask = last_sent_.canfd[ch].ctrl;
  std::copy_n(recv_buf_.canfd[ch]+id*8, 8, data.rx.begin());
  std::copy_n(last_sent_.canfd[ch].data+id*8, 8, data.tx.begin());
  data.state = {actr->GetPosition(), actr->GetVelocity(), actr->GetEffort(), rx_pdo_ns_};
  data.mit_param_valid = actr->GetMitParam(data.mit_param);
  if (data.tx_pdo_seq && data.tx_format == 1 && data.mit_param_valid) {
    const auto& p = data.mit_param; const auto& f = data.tx;
    data.cmd_position = Decode((f[0]<<8)|f[1], p.pos_min,p.pos_max,65535);
    data.cmd_velocity = Decode((f[2]<<4)|(f[3]>>4),p.vel_min,p.vel_max,4095);
    data.cmd_kp = Decode(((f[3]&15)<<8)|f[4],p.kp_min,p.kp_max,4095);
    data.cmd_kd = Decode((f[5]<<4)|(f[6]>>4),p.kd_min,p.kd_max,4095);
    data.cmd_effort = Decode(((f[6]&15)<<8)|f[7],p.toq_min,p.toq_max,4095);
  } else if (data.tx_pdo_seq && data.tx_format == 2) {
    std::memcpy(&data.cmd_position,data.tx.data(),sizeof(float));
    std::memcpy(&data.cmd_current_limit,data.tx.data()+4,sizeof(float));
  }
  return true;
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
    MarkCommand(actr);
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
    MarkCommand(actr);
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
  MarkCommand(actr);
  SetChannelId(actr->GetCtrlChannel(), actr->GetId());
}

void Dcu::SetHomingPosition(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetHomingPosition();
  MarkCommand(actr);
  SetChannelId(actr->GetCtrlChannel(), actr->GetId());
}

void Dcu::SaveConfig(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SaveConfig();
  MarkCommand(actr);
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
    MarkCommand(actr);
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
  MarkCommand(actr);
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
  MarkCommand(actr);
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
  MarkCommand(actr);
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

  std::scoped_lock lock(send_mtx_, recv_mtx_);
  actr->SetMitParam(param);
}

void Dcu::SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp,
                    float kd) {
  auto actr = GetActautor(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetMitCmd(pos, vel, effort, kp, kd);
  const auto type = actr->GetType();
  MarkCommand(actr, type == ActuatorType::POWER_FLOW_L28 ? 2 :
                   type == ActuatorType::POWER_FLOW_R86 || type == ActuatorType::POWER_FLOW_R52 ? 1 : 0);
  SetChannelId(actr->GetCtrlChannel(), CHANNEL_BROADCAST_ID);
}

}  // namespace xyber
