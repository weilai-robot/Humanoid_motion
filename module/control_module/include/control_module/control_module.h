#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <set>
#include <fstream>
#include <filesystem>
#include <thread>
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
  void StatsWorkerLoop();

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
  int32_t rt_priority_{};  // rl_control_pub_thread 实时优先级（SCHED_FIFO）
  int32_t bind_cpu_{};     // rl_control_pub_thread 绑核
  std::atomic_bool run_flag_{true};
  time_point<high_resolution_clock> last_trigger_time_;

  std::string last_state_name_;

  // ── R5 控制周期统计：MainLoop(RT,1kHz) 写直方图双缓冲，StatsWorker(非RT) 每10s 读出落盘 ──
  struct PeriodHist {
    std::array<uint32_t, 32> late_hist_{};  // 唤醒延迟直方图, 桶宽 10us, 覆盖 0..320us
    std::array<uint32_t, 32> body_hist_{};  // 体耗时直方图, 桶宽 20us, 覆盖 0..640us
    uint32_t late_over_cnt_ = 0;            // late > 320us 样本数
    uint32_t body_over_cnt_ = 0;            // body > 640us 样本数
    uint32_t miss_cnt_ = 0;                 // late >= 100us（deadline miss, 阈值 S1 基线后校准）
    uint32_t late_max_us_ = 0;
    uint32_t body_max_us_ = 0;
    uint32_t sample_cnt_ = 0;
  };
  PeriodHist hist_buf_[2];
  std::atomic<uint32_t> hist_active_{0};    // MainLoop 只读, StatsWorker 翻转
  std::thread stats_worker_thread_;
  std::atomic_bool stats_running_{false};
  std::ofstream stats_file_;                // StatsWorker 独占
};

}  // namespace xyber_x1_infer::rl_control_module
