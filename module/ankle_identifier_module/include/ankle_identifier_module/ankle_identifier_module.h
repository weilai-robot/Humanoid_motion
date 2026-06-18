// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "aimrt_module_cpp_interface/module_base.h"
#include "my_ros2_proto/msg/joint_command.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace xyber_x1_infer::ankle_identifier_module {

class AnkleIdentifierModule : public aimrt::ModuleBase {
 public:
  AnkleIdentifierModule() = default;
  ~AnkleIdentifierModule() override = default;

  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "AnkleIdentifierModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  struct JointSnapshot {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
  };

  enum class TestMode { kStep, kSine };
  enum class StartupPoseMode { kCurrent, kZero, kStand };

  bool LoadConfig();
  void PrepareTargets();
  bool LoadStartupPoseTargets();
  void PrepareCsv();
  void MainLoop();
  void OnJointState(const std::shared_ptr<const sensor_msgs::msg::JointState>& msg);
  void OnImu(const std::shared_ptr<const sensor_msgs::msg::Imu>& msg);
  void StepControl(double elapsed_sec);
  bool TryCaptureStableBaseline();
  double DesiredPrimaryVelocity(double local_time) const;
  void PublishHoldCommand();
  void PublishStartupPoseHoldCommand();
  std::pair<double, double> GetHoldGains(const std::string& joint_name) const;
  void SetJointCmd(my_ros2_proto::msg::JointCommand& cmd, const std::string& joint_name,
                   double position, double velocity, double effort, double kp, double kd);
  double GetBaseline(const std::string& joint_name) const;
  void LogSample(double elapsed, const std::string& phase, int iteration, double target_primary,
                 double target_coupled);
  auto GetLogger() { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;
  aimrt::channel::SubscriberRef joint_state_sub_;
  aimrt::channel::SubscriberRef imu_sub_;
  aimrt::channel::PublisherRef joint_cmd_pub_;

  std::string joint_cmd_topic_ = "/joint_cmd";
  std::string joint_state_topic_ = "/joint_states";
  std::string imu_topic_ = "/imu/data";
  std::string test_side_ = "left";
  std::string test_axis_ = "pitch";
  std::string primary_joint_;
  std::string coupled_joint_;
  std::string csv_path_ = "ankle_identification.csv";
  std::string reference_control_cfg_path_ = "./cfg/control_module/rl_x1.yaml";

  double publish_rate_hz_ = 1000.0;
  double pre_hold_sec_ = 2.0;
  double active_sec_ = 1.0;
  double post_hold_sec_ = 2.0;
  double step_amplitude_rad_ = 0.005;
  double sine_amplitude_rad_ = 0.004;
  double sine_frequency_hz_ = 1.0;
  double test_kp_ = 35.0;
  double test_kd_ = 0.8;
  double hold_kp_ = 30.0;
  double hold_kd_ = 1.0;
  double torso_hold_kp_ = 500.0;
  double torso_hold_kd_ = 5.0;
  double arm_hold_kp_ = 80.0;
  double arm_hold_kd_ = 1.5;
  double leg_hold_kp_ = 300.0;
  double leg_hold_kd_ = 5.0;
  double startup_stable_sec_ = 1.0;
  double startup_joint_vel_threshold_ = 0.05;
  double startup_gyro_threshold_ = 0.2;
  int repeat_count_ = 3;
  bool use_imu_ = true;
  bool auto_stop_after_test_ = true;
  TestMode test_mode_ = TestMode::kStep;
  StartupPoseMode startup_pose_mode_ = StartupPoseMode::kZero;

  std::atomic_bool run_flag_{false};
  std::atomic_bool have_joint_index_{false};
  std::atomic_bool baseline_captured_{false};
  std::atomic_bool test_completed_{false};
  std::atomic_bool completion_logged_{false};
  std::atomic_bool startup_wait_logged_{false};
  std::thread main_thread_;

  mutable std::mutex data_mutex_;
  std::unordered_map<std::string, size_t> joint_index_;
  std::unordered_map<std::string, JointSnapshot> latest_joint_state_;
  std::unordered_map<std::string, double> startup_target_positions_;
  std::vector<std::string> joint_names_;
  my_ros2_proto::msg::JointCommand baseline_cmd_;
  sensor_msgs::msg::Imu latest_imu_;
  std::chrono::steady_clock::time_point start_time_;
  std::optional<std::chrono::steady_clock::time_point> startup_stable_since_;
  std::ofstream csv_;
};

}  // namespace xyber_x1_infer::ankle_identifier_module
