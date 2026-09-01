#include "control_module/control_module.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <pthread.h>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "control_module/global.h"
#include "internal/common_utils.h"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace xyber_x1_infer::rl_control_module {

namespace {
// R5 统计参数（桶宽/阈值均为初值，S1 基线后校准）
constexpr uint32_t kLateBucketUs = 10;
constexpr uint32_t kLateBucketCnt = 32;
constexpr uint32_t kBodyBucketUs = 20;
constexpr uint32_t kBodyBucketCnt = 32;
constexpr uint32_t kMissLateUs = 100;

std::string StatsTimestamp() {
  const std::time_t t = std::time(nullptr);
  std::tm tm_val{};
  localtime_r(&t, &tm_val);
  char buf[24];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_val);
  return std::string(buf);
}

// 由直方图+溢出计数估计分位数（粒度=桶宽；落在溢出区返回桶上界）
uint32_t HistPct(const std::array<uint32_t, 32>& hist, uint32_t n, double pct, uint32_t step) {
  if (n == 0) return 0;
  const uint64_t target = static_cast<uint64_t>(std::ceil(n * pct));
  uint64_t acc = 0;
  for (uint32_t i = 0; i < 32; ++i) {
    acc += hist[i];
    if (acc >= target) return i * step;
  }
  return 32 * step;
}
}  // namespace

bool ControlModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;
  SetLogger(core_.GetLogger());
  subs_.clear();

  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  try {
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path.data());
      freq_ = cfg_node["control_frequecy"].as<int32_t>();
      use_sim_handles_ = cfg_node["use_sim_handles"].as<bool>();
      rt_priority_ = cfg_node["rt_priority"] ? cfg_node["rt_priority"].as<int32_t>() : -1;
      bind_cpu_ = cfg_node["bind_cpu"] ? cfg_node["bind_cpu"].as<int32_t>() : -1;

      // 解析状态机
      last_trigger_time_ = high_resolution_clock::now();
      const std::string initial_state =
          cfg_node["initial_state"] ? cfg_node["initial_state"].as<std::string>() : "";
      state_machine_.Init(cfg_node["robot_states"], initial_state);
      last_state_name_ = state_machine_.GetCurrentState();
      for (auto iter = cfg_node["robot_states"].begin(); iter != cfg_node["robot_states"].end(); iter++) {
        auto trigger_topic = iter->second["trigger_topic"].as<std::string>();
        if (trigger_topics_.find(trigger_topic) != trigger_topics_.end()) {
          continue;
        }
        trigger_topics_.insert(trigger_topic);
        subs_.push_back(core_.GetChannelHandle().GetSubscriber(trigger_topic));
        bool ret = aimrt::channel::Subscribe<std_msgs::msg::Float32>(subs_.back(),
          [this, trigger_topic](const std::shared_ptr<const std_msgs::msg::Float32>& msg) {
            if (Throttler(high_resolution_clock::now(), last_trigger_time_, milliseconds(1000)) && state_machine_.OnEvent(trigger_topic)) {
              auto now_state = state_machine_.GetCurrentState();
              auto controller_names = state_machine_.GetCurrentControllerNames();
              for (auto name : controller_names) {
                // printf("RestartController: %s\n", name.c_str());
                controller_map_[name]->RestartController();
              }
              AIMRT_INFO("Trigger event: [{}] state '{}' -> '{}'", trigger_topic, last_state_name_, now_state);
              
              UpdateRlLoggingState(now_state);
              last_state_name_ = now_state;
            }
          });
        AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");
      }
      // auto controller_names = state_machine_.GetCurrentControllerNames();
      // for (auto name : controller_names) {
      //   printf("name: %s\n", name.c_str());
      // }

      // 解析控制器
      for (auto iter = cfg_node["controllers"].begin(); iter != cfg_node["controllers"].end(); iter++) {
        std::string controller_name = iter->first.as<std::string>();
        // printf("controller: %s\n", controller_name.c_str());

        if (controller_name.substr(0, 3) == "rl_") {
          controller_map_[controller_name] = std::make_shared<RLController>(use_sim_handles_);
        } else if (controller_name.substr(0, 3) == "pd_") {
          controller_map_[controller_name] = std::make_shared<PDController>(use_sim_handles_);
        } else {
          AIMRT_ERROR("Unknown controller type: {}", controller_name);
        }
        // 将顶层 joint_limits 注入 controller 子节点，供 RLController::Init() 使用
        if (cfg_node["joint_limits"]) {
          iter->second["joint_limits"] = cfg_node["joint_limits"];
        }
        controller_map_[controller_name]->Init(iter->second);

      }

      UpdateRlLoggingState(last_state_name_);

      // 设置 joint_xxx_index_map_ 的尺度
      for (const auto& joint : cfg_node["joint_list"]) {
        joint_state_index_map_[joint.as<std::string>()] = -1;
      }
      std::vector<std::string> joint_list = cfg_node["joint_list"].as<std::vector<std::string>>();
      for (size_t ii = 0; ii < joint_list.size(); ++ii) {
        joint_cmd_index_map_[joint_list[ii]] = ii;
      }
      joint_offset_map_ = cfg_node["joint_offset"].as<std::map<std::string, double>>();
      // printf("joint_cmd_index_map_: ");
      // for (const auto& pair : joint_cmd_index_map_) {
      //   printf("%s: %d, ", pair.first.c_str(), pair.second);
      // }
      // printf("\n");

      // 控制器订阅
      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_joy_vel_name"].as<std::string>()));
      bool ret = aimrt::channel::Subscribe<geometry_msgs::msg::Twist>(subs_.back(), 
        [this](const std::shared_ptr<const geometry_msgs::msg::Twist>& msg) {
          auto controller_names = state_machine_.GetCurrentControllerNames();
          for (const auto& name : controller_names) {
            controller_map_[name]->SetCmdData(*msg);
          }
        });

      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_imu_data_name"].as<std::string>()));
      ret &= aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(subs_.back(), 
        [this](const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
          // IMU 数据转发给所有控制器
          for (const auto& controller : controller_map_) {
            controller.second->SetImuData(*msg);
          }
        });

      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_joint_state_name"].as<std::string>()));
      ret &= aimrt::channel::Subscribe<sensor_msgs::msg::JointState>(subs_.back(), 
        [this](const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {
          // 仅初始化一次 joint_state_index_map_
          if (joint_state_index_map_.begin()->second == -1) {
            for (size_t i = 0; i < msg->name.size(); i++) {
              joint_state_index_map_[msg->name[i]] = i;
            }
          }

          // 新设置的 offset
          sensor_msgs::msg::JointState temp_msg = *msg;
          for (const auto& joint : joint_offset_map_) {
            temp_msg.position[joint_state_index_map_.at(joint.first)] -= joint.second;
          }

          for (const auto& controller : controller_map_) {
            controller.second->SetJointStateData(temp_msg, joint_state_index_map_);
          }
        });

      // 电机层指令/反馈（dcu_driver actuator_debug=true 时发布；用于 walk_diag 记录电机侧扭矩）
      const std::string actuator_cmd_topic =
          cfg_node["sub_actuator_cmd_name"]
              ? cfg_node["sub_actuator_cmd_name"].as<std::string>()
              : "/actuator_cmd";
      const std::string actuator_state_topic =
          cfg_node["sub_actuator_state_name"]
              ? cfg_node["sub_actuator_state_name"].as<std::string>()
              : "/actuator_states";

      subs_.push_back(core_.GetChannelHandle().GetSubscriber(actuator_cmd_topic));
      ret &= aimrt::channel::Subscribe<my_ros2_proto::msg::JointCommand>(
          subs_.back(),
          [this](const std::shared_ptr<const my_ros2_proto::msg::JointCommand>& msg) {
            for (const auto& controller : controller_map_) {
              controller.second->SetActuatorCmdData(*msg);
            }
          });

      subs_.push_back(core_.GetChannelHandle().GetSubscriber(actuator_state_topic));
      ret &= aimrt::channel::Subscribe<sensor_msgs::msg::JointState>(
          subs_.back(),
          [this](const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {
            for (const auto& controller : controller_map_) {
              controller.second->SetActuatorStateData(*msg);
            }
          });
      AIMRT_INFO("Subscribed actuator topics: {} , {}", actuator_cmd_topic, actuator_state_topic);

      AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");

      // 控制器发布
      joint_cmd_pub_ = core_.GetChannelHandle().GetPublisher(cfg_node["pub_joint_cmd_name"].as<std::string>());
      executor_ = core_.GetExecutorManager().GetExecutor("rl_control_pub_thread");
      AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor 'rl_control_pub_thread'.");
      aimrt::channel::RegisterPublishType<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");
  return true;
}

void ControlModule::UpdateRlLoggingState(const std::string& state_name) {
  const auto active_controller_names = state_machine_.GetCurrentControllerNames();
  int active_rl_count = 0;

  for (auto& [name, controller] : controller_map_) {
    auto rl_controller = std::dynamic_pointer_cast<RLController>(controller);
    if (!rl_controller) {
      continue;
    }

    const bool active =
        std::find(active_controller_names.begin(), active_controller_names.end(), name) !=
        active_controller_names.end();
    rl_controller->SetLoggingActive(active);
    if (active) {
      ++active_rl_count;
    }
  }

  AIMRT_INFO("[Diag Trigger] State '{}': {} active RL controller(s)", state_name, active_rl_count);
}

bool ControlModule::Start() {
  AIMRT_INFO("thread safe [{}]", executor_.ThreadSafe());
  try {
    executor_.Execute([this]() { MainLoop(); });
    // R5 统计线程（观测不阻塞控制：启动失败仅静默放弃统计）
    stats_running_.store(true);
    try {
      stats_worker_thread_ = std::thread([this]() { StatsWorkerLoop(); });
    } catch (...) {
      stats_running_.store(false);
    }
    AIMRT_INFO("Started succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }
  return true;
}

void ControlModule::Shutdown() {
  run_flag_.store(false);
  stats_running_.store(false);
  if (stats_worker_thread_.joinable()) {
    stats_worker_thread_.join();
  }
  for (auto& [name, controller] : controller_map_) {
    (void)name;
    auto rl_controller = std::dynamic_pointer_cast<RLController>(controller);
    if (rl_controller) {
      rl_controller->StopLoggingWorker();
    }
  }
}

bool ControlModule::MainLoop() {
  try {
    AIMRT_INFO("Start MainLoop.");
    // 设置线程实时属性（参数从 yaml 配置读取，与 dcu 模块同模式）
    if (!xyber_utils::SetRealTimeThread(pthread_self(), "rl_control_pub", rt_priority_, bind_cpu_)) {
      AIMRT_ERROR("rl_control_pub real-time setup failed, running without SCHED_FIFO/CPU affinity");
    }
    auto const period = nanoseconds(1'000'000'000 / freq_);
    time_point<high_resolution_clock, nanoseconds> next_iteration_time = high_resolution_clock::now();

    my_ros2_proto::msg::JointCommand cmd_msg;
    cmd_msg.name.resize(joint_cmd_index_map_.size(), "");
    cmd_msg.position.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.velocity.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.effort.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.damping.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.stiffness.resize(joint_cmd_index_map_.size(), 0.0);

    while (run_flag_) {
      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);

      // ── R5: 唤醒延迟（调度抖动真值）──
      const auto wake_tp = high_resolution_clock::now();
      const int64_t late_us =
          duration_cast<microseconds>(wake_tp - next_iteration_time).count();
      const auto body_t0 = high_resolution_clock::now();

      auto controller_names = state_machine_.GetCurrentControllerNames();
      for (const auto& name : controller_names) {
        controller_map_[name]->Update();
        my_ros2_proto::msg::JointCommand tmp_cmd = controller_map_[name]->GetJointCmdData();
        // 将 tmp_cmd 中的数据复制到 cmd_msg 中
        for (size_t ii = 0; ii < tmp_cmd.name.size(); ii++) {
          int index = joint_cmd_index_map_[tmp_cmd.name[ii].c_str()];
          cmd_msg.name[index] = tmp_cmd.name[ii];
          cmd_msg.position[index] = tmp_cmd.position[ii] + joint_offset_map_[tmp_cmd.name[ii]];
          cmd_msg.velocity[index] = tmp_cmd.velocity[ii];
          cmd_msg.effort[index] = tmp_cmd.effort[ii];
          cmd_msg.damping[index] = tmp_cmd.damping[ii];
          cmd_msg.stiffness[index] = tmp_cmd.stiffness[ii];
        }
      }
      aimrt::channel::Publish<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_, cmd_msg);

      // ── R5: 本周期体耗时 + 直方图更新（O(1)，无锁无 IO）──
      const int64_t body_us =
          duration_cast<microseconds>(high_resolution_clock::now() - body_t0).count();
      PeriodHist& h = hist_buf_[hist_active_.load(std::memory_order_relaxed)];
      const uint32_t lu = late_us > 0 ? static_cast<uint32_t>(late_us) : 0u;
      if (lu < kLateBucketCnt * kLateBucketUs) ++h.late_hist_[lu / kLateBucketUs];
      else ++h.late_over_cnt_;
      if (lu > h.late_max_us_) h.late_max_us_ = lu;
      if (lu >= kMissLateUs) ++h.miss_cnt_;
      const uint32_t bu = body_us > 0 ? static_cast<uint32_t>(body_us) : 0u;
      if (bu < kBodyBucketCnt * kBodyBucketUs) ++h.body_hist_[bu / kBodyBucketUs];
      else ++h.body_over_cnt_;
      if (bu > h.body_max_us_) h.body_max_us_ = bu;
      ++h.sample_cnt_;

    }
    AIMRT_INFO("Exit MainLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
    return false;
  }
  return true;
}

void ControlModule::StatsWorkerLoop() {
  pthread_setname_np(pthread_self(), "ctrl_stat_w");

  // 数据只落文件（与 walk_diag 同目录族），不打印终端；失败静默放弃统计
  std::error_code fs_ec;
  std::filesystem::create_directories("test_logs/data_csv", fs_ec);
  const std::string path = "test_logs/data_csv/ctrl_period_" + StatsTimestamp() + ".csv";
  stats_file_.open(path, std::ios::out | std::ios::app);
  if (stats_file_.is_open()) {
    stats_file_ << "wall_ts,steady_ns,samples,"
                   "late_us_p50,late_us_p95,late_us_p99,late_us_max,miss_cnt,"
                   "body_us_p50,body_us_p95,body_us_p99,body_us_max,"
                   "late_over_cnt,body_over_cnt\n";
  }

  auto next_emit = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (stats_running_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() < next_emit) continue;
    next_emit += std::chrono::seconds(10);
    if (!stats_file_.is_open()) continue;

    // 翻转双缓冲：取回旧半区快照并清零（MainLoop 此后写新半区；边界最多丢 1 个样本）
    const uint32_t idx = hist_active_.exchange(
        1u - hist_active_.load(std::memory_order_relaxed), std::memory_order_acq_rel);
    const PeriodHist snap = hist_buf_[idx];
    hist_buf_[idx] = PeriodHist{};

    const auto steady_ns = duration_cast<nanoseconds>(
        high_resolution_clock::now().time_since_epoch()).count();
    stats_file_ << StatsTimestamp() << ',' << steady_ns << ',' << snap.sample_cnt_ << ','
                << HistPct(snap.late_hist_, snap.sample_cnt_, 0.50, kLateBucketUs) << ','
                << HistPct(snap.late_hist_, snap.sample_cnt_, 0.95, kLateBucketUs) << ','
                << HistPct(snap.late_hist_, snap.sample_cnt_, 0.99, kLateBucketUs) << ','
                << snap.late_max_us_ << ','
                << snap.miss_cnt_ << ','
                << HistPct(snap.body_hist_, snap.sample_cnt_, 0.50, kBodyBucketUs) << ','
                << HistPct(snap.body_hist_, snap.sample_cnt_, 0.95, kBodyBucketUs) << ','
                << HistPct(snap.body_hist_, snap.sample_cnt_, 0.99, kBodyBucketUs) << ','
                << snap.body_max_us_ << ','
                << snap.late_over_cnt_ << ','
                << snap.body_over_cnt_ << '\n';
    stats_file_.flush();
  }

  if (stats_file_.is_open()) {
    stats_file_.flush();
    stats_file_.close();
  }
}

}  // namespace xyber_x1_infer::rl_control_module
