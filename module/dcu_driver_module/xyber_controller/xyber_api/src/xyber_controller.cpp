/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:50
 */

#include "xyber_controller.h"

#include <functional>
#include <map>
#include <stdexcept>
#include <vector>

#include "common_type.h"
#include "internal/dcu.h"
#include "internal/ethercat_manager.h"
#include "internal/omni_picker.h"
#include "internal/power_flow.h"
#include "internal/version.h"

using namespace ethercat_manager;

namespace xyber {

// global hidden variable
XyberController* XyberController::instance_ = nullptr;

static EthercatConfig ecat_config_;
static EthercatManager ecat_manager_;
static std::unordered_map<std::string, Dcu*> dcu_map_;
static std::unordered_map<std::string, Dcu*> actuator_dcu_map_;

XyberController::XyberController() {
  LOG_INFO("XyberController is created. Ver %d.%d.%d, Compiled in %s %s.", MAIN_VERSION,
           SUB_VERSION, PATCH_VERSION, __DATE__, __TIME__);
}

XyberController::~XyberController() {
  Stop();
  for (auto& dcu : dcu_map_) {
    delete dcu.second;
  }
}

XyberController* XyberController::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new XyberController();
  }
  return instance_;
}

std::string XyberController::GetVersion() {
  return std::to_string(MAIN_VERSION) + "." + std::to_string(SUB_VERSION) + "." +
         std::to_string(PATCH_VERSION);
}

bool XyberController::CreateDcu(std::string name, uint8_t id) {
  if (dcu_map_.find(name) != dcu_map_.end()) {
    LOG_ERROR("DCU %s is already created, add failed.", name.c_str());
    return false;
  }
  dcu_map_[name] = new Dcu(name, id);
  return true;
}

bool XyberController::AttachActuator(std::string dcu_name, CtrlChannel ch, ActuatorType type,
                                     std::string actuator_name, uint8_t id) {
  // check check
  if (id == 0) {
    LOG_ERROR("Actuator %s invalid id 0, attach failed.", actuator_name.c_str());
    return false;
  }
  if (dcu_map_.find(dcu_name) == dcu_map_.end()) {
    LOG_ERROR("Can`t find DCU %s, attach actuator %s failed.", dcu_name.c_str(),
              actuator_name.c_str());
    return false;
  }
  if (actuator_dcu_map_.find(actuator_name) != actuator_dcu_map_.end()) {
    LOG_ERROR("Actuator %s is already attached on DCU %s.", actuator_name.c_str(),
              actuator_dcu_map_[actuator_name]->GetName().c_str());
    return false;
  }
  // create actuator
  Actuator* atcr = nullptr;
  switch (type) {
    case ActuatorType::POWER_FLOW_R86:
      atcr = new xyber_power_flow::PowerFlowR(type, actuator_name, id, ch,
                                              PF_R86_MIT_MODE_DEFAULT_PARAM);
      break;
    case ActuatorType::POWER_FLOW_R52:
      atcr = new xyber_power_flow::PowerFlowR(type, actuator_name, id, ch,
                                              PF_R52_MIT_MODE_DEFAULT_PARAM);
      break;
    case ActuatorType::POWER_FLOW_L28:
      atcr = new xyber_power_flow::PowerFlowL(actuator_name, id, ch);
      break;
    case ActuatorType::OMNI_PICKER:
      atcr = new xyber_omni_picker::OmniPicker(actuator_name, id, ch);
      break;
    default:
      LOG_ERROR("Actuator %s type %d is not supported.", actuator_name.c_str(), (int)type);
      return false;
  }
  // attach to dcu
  LOG_DEBUG("Attach actuator %s %d to DCU %s Channel %d.", actuator_name.c_str(), (int)id,
            dcu_name.c_str(), (int)ch + 1);
  dcu_map_[dcu_name]->RegisterActuator(atcr);
  actuator_dcu_map_[actuator_name] = dcu_map_[dcu_name];
  return true;
}

bool XyberController::SetRealtime(int rt_priority, int bind_cpu) {
  if (is_running_) {
    LOG_WARN("XyberController is running, can not set realtime.");
    return false;
  }
  ecat_config_.bind_cpu = bind_cpu;
  ecat_config_.rt_priority = rt_priority;
  return true;
}

bool XyberController::Start(std::string ifname, uint64_t cycle_ns, bool enable_dc) {
  if (is_running_) {
    LOG_WARN("XyberController is already running.");
    return true;
  }
  // register dcu to ethercat manager
  for (const auto& dcu : dcu_map_) {
    ecat_manager_.RegisterNode(dcu.second);
  }
  ecat_config_.ifname = ifname;
  ecat_config_.enable_dc = enable_dc;
  ecat_config_.cycle_time_ns = cycle_ns;
  is_running_ = ecat_manager_.Start(ecat_config_);
  if (is_running_) {
    LOG_INFO("XyberController start successfully.");
  } else {
    LOG_ERROR("XyberController start failed.");
  }
  return is_running_;
}

void XyberController::Stop() {
  if (!is_running_) return;

  ecat_manager_.Stop();
  is_running_ = false;
  LOG_INFO("XyberController stop successfully.");
}

DcuImu XyberController::GetDcuImuData(const std::string& name) {
  auto it = dcu_map_.find(name);
  if (it == dcu_map_.end()) return DcuImu();

  return it->second->GetImuData();
}

void XyberController::ApplyDcuImuOffset(const std::string& name) {
  auto it = dcu_map_.find(name);
  if (it == dcu_map_.end()) return;
}

bool XyberController::EnableAllActuator() {
  for (const auto& [name, dcu] : dcu_map_) {
    if (!dcu->EnableAllActuator()) return false;
  }
  return true;
}

bool XyberController::EnableActuator(const std::string& name) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return false;

  return it->second->EnableActuator(name);
}

bool XyberController::DisableAllActuator() {
  for (const auto& [name, dcu] : dcu_map_) {
    if (!dcu->DisableAllActuator()) return false;
  }
  return true;
}

bool XyberController::DisableActuator(const std::string& name) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return false;

  return it->second->DisableActuator(name);
}

float XyberController::GetTempure(const std::string& name) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return 0;

  return it->second->GetTempure(name);
}

// std::string XyberController::GetErrorString(const std::string& name) {
//   auto it = actuator_dcu_map_.find(name);
//   if (it == actuator_dcu_map_.end()) return "";

//   return it->second->GetErrorString(name);
// }

ActautorState XyberController::GetPowerState(const std::string& name) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return STATE_DISABLE;

  return it->second->GetPowerState(name);
}

ActautorMode XyberController::GetMode(const std::string& name) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return MODE_CURRENT;

  return it->second->GetMode(name);
}

float XyberController::GetEffort(const std::string& name) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return 0;

  return it->second->GetEffort(name);
}

float XyberController::GetVelocity(const std::string& name) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return 0;

  return it->second->GetVelocity(name);
}

float XyberController::GetPosition(const std::string& name) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return 0;

  return it->second->GetPosition(name);
}

void XyberController::SetMitParam(const std::string& name, MitParam param) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return;

  return it->second->SetMitParam(name, param);
}

void XyberController::SetMitCmd(const std::string& name, float pos, float vel, float effort,
                                float kp, float kd) {
  auto it = actuator_dcu_map_.find(name);
  if (it == actuator_dcu_map_.end()) return;

  return it->second->SetMitCmd(name, pos, vel, effort, kp, kd);
}

}  // namespace xyber