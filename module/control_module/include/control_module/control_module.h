#pragma once

#include <chrono>
#include <memory>
#include <set>
#include <fstream>
#include <filesystem>
#include "aimrt_module_cpp_interface/module_base.h"
#include "control_module/pd_controller.h"
#include "control_module/rl_controller.h"
#include "control_module/state_machine.h"
// #include "control_module/joint_groups.h"

using namespace std::chrono;

namespace xyber_x1_infer::rl_control_module {

class ControlModule : public aimrt::ModuleBase {
 public:
  ControlModule() = default;
  ~ControlModule() override = default;
  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ControlModule"};
  }
  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  bool MainLoop();
  void UpdateRlLoggingState(const std::string& state_name);

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef executor_;

  std::vector<aimrt::channel::SubscriberRef> subs_;
  aimrt::channel::PublisherRef joint_cmd_pub_;

  StateMachine state_machine_;
  std::set<std::string> trigger_topics_;//// 添加重复的 trigger_topic 会报错，这里用 set 来存储
  std::map<std::string, std::shared_ptr<ControllerBase>> controller_map_;
  std::unordered_map<std::string, int> joint_state_index_map_;
  std::unordered_map<std::string, int> joint_cmd_index_map_;
  std::map<std::string, double> joint_offset_map_;

  bool use_sim_handles_;
  int32_t freq_;
  std::atomic_bool run_flag_{true};
  time_point<high_resolution_clock> last_trigger_time_;

  std::string last_state_name_;
};

}  // namespace xyber_x1_infer::rl_control_module
