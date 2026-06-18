// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "ankle_identifier_module/ankle_identifier_module.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

namespace xyber_x1_infer::ankle_identifier_module {

using namespace std::chrono_literals;

bool AnkleIdentifierModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  try {
    if (!LoadConfig()) return false;
    PrepareTargets();
    if (!LoadStartupPoseTargets()) return false;
    PrepareCsv();

    joint_state_sub_ = core_.GetChannelHandle().GetSubscriber(joint_state_topic_);
    bool ret = aimrt::channel::Subscribe<sensor_msgs::msg::JointState>(
        joint_state_sub_,
        std::bind(&AnkleIdentifierModule::OnJointState, this, std::placeholders::_1));
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe {} failed.", joint_state_topic_);

    if (use_imu_) {
      imu_sub_ = core_.GetChannelHandle().GetSubscriber(imu_topic_);
      ret = aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(
          imu_sub_, std::bind(&AnkleIdentifierModule::OnImu, this, std::placeholders::_1));
      AIMRT_CHECK_ERROR_THROW(ret, "Subscribe {} failed.", imu_topic_);
    }

    joint_cmd_pub_ = core_.GetChannelHandle().GetPublisher(joint_cmd_topic_);
    aimrt::channel::RegisterPublishType<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_);

    AIMRT_WARN("AnkleIdentifierModule publishes directly to {}.", joint_cmd_topic_);
    AIMRT_WARN("Do not run it concurrently with another /joint_cmd publisher such as ControlModule.");
    AIMRT_INFO("Init succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  return true;
}

bool AnkleIdentifierModule::Start() {
  run_flag_.store(true);
  main_thread_ = std::thread(&AnkleIdentifierModule::MainLoop, this);
  AIMRT_INFO("Started succeeded.");
  return true;
}

void AnkleIdentifierModule::Shutdown() {
  run_flag_.store(false);
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
  if (csv_.is_open()) {
    csv_.flush();
    csv_.close();
  }
}

bool AnkleIdentifierModule::LoadConfig() {
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  if (file_path.empty()) {
    AIMRT_ERROR("Init failed, [file_path] Empty");
    return false;
  }

  YAML::Node cfg_node = YAML::LoadFile(file_path.data());
  joint_cmd_topic_ = cfg_node["joint_cmd_topic"].as<std::string>("/joint_cmd");
  joint_state_topic_ = cfg_node["joint_state_topic"].as<std::string>("/joint_states");
  imu_topic_ = cfg_node["imu_topic"].as<std::string>("/imu/data");
  reference_control_cfg_path_ =
      cfg_node["reference_control_cfg_path"].as<std::string>("./cfg/control_module/rl_x1.yaml");

  const auto mode = cfg_node["mode"].as<std::string>("step");
  if (mode == "step") {
    test_mode_ = TestMode::kStep;
  } else if (mode == "sine") {
    test_mode_ = TestMode::kSine;
  } else {
    AIMRT_ERROR("Unsupported mode {}", mode);
    return false;
  }

  const auto startup_pose_mode = cfg_node["startup_pose_mode"].as<std::string>("zero");
  if (startup_pose_mode == "current") {
    startup_pose_mode_ = StartupPoseMode::kCurrent;
  } else if (startup_pose_mode == "zero") {
    startup_pose_mode_ = StartupPoseMode::kZero;
  } else if (startup_pose_mode == "stand") {
    startup_pose_mode_ = StartupPoseMode::kStand;
  } else {
    AIMRT_ERROR("Unsupported startup_pose_mode {}", startup_pose_mode);
    return false;
  }

  test_side_ = cfg_node["test_side"].as<std::string>("left");
  test_axis_ = cfg_node["test_axis"].as<std::string>("pitch");
  publish_rate_hz_ = cfg_node["publish_rate_hz"].as<double>(1000.0);
  pre_hold_sec_ = cfg_node["pre_hold_sec"].as<double>(2.0);
  active_sec_ = cfg_node["active_sec"].as<double>(1.0);
  post_hold_sec_ = cfg_node["post_hold_sec"].as<double>(2.0);
  repeat_count_ = cfg_node["repeat_count"].as<int>(3);
  step_amplitude_rad_ = cfg_node["step_amplitude_rad"].as<double>(0.005);
  sine_amplitude_rad_ = cfg_node["sine_amplitude_rad"].as<double>(0.004);
  sine_frequency_hz_ = cfg_node["sine_frequency_hz"].as<double>(1.0);
  test_kp_ = cfg_node["test_kp"].as<double>(35.0);
  test_kd_ = cfg_node["test_kd"].as<double>(0.8);
  hold_kp_ = cfg_node["hold_kp"].as<double>(30.0);
  hold_kd_ = cfg_node["hold_kd"].as<double>(1.0);
  torso_hold_kp_ = cfg_node["torso_hold_kp"].as<double>(500.0);
  torso_hold_kd_ = cfg_node["torso_hold_kd"].as<double>(5.0);
  arm_hold_kp_ = cfg_node["arm_hold_kp"].as<double>(80.0);
  arm_hold_kd_ = cfg_node["arm_hold_kd"].as<double>(1.5);
  leg_hold_kp_ = cfg_node["leg_hold_kp"].as<double>(300.0);
  leg_hold_kd_ = cfg_node["leg_hold_kd"].as<double>(5.0);
  startup_stable_sec_ = cfg_node["startup_stable_sec"].as<double>(1.0);
  startup_joint_vel_threshold_ = cfg_node["startup_joint_vel_threshold"].as<double>(0.05);
  startup_gyro_threshold_ = cfg_node["startup_gyro_threshold"].as<double>(0.2);
  use_imu_ = cfg_node["use_imu"].as<bool>(true);
  auto_stop_after_test_ = cfg_node["auto_stop_after_test"].as<bool>(true);
  csv_path_ = cfg_node["csv_path"].as<std::string>("ankle_identification.csv");

  return true;
}

void AnkleIdentifierModule::PrepareTargets() {
  if (test_side_ != "left" && test_side_ != "right") {
    throw std::runtime_error("test_side must be left or right");
  }
  if (test_axis_ != "pitch" && test_axis_ != "roll") {
    throw std::runtime_error("test_axis must be pitch or roll");
  }

  if (test_side_ == "left") {
    primary_joint_ = test_axis_ == "pitch" ? "left_ankle_pitch_joint" : "left_ankle_roll_joint";
    coupled_joint_ = test_axis_ == "pitch" ? "left_ankle_roll_joint" : "left_ankle_pitch_joint";
  } else {
    primary_joint_ =
        test_axis_ == "pitch" ? "right_ankle_pitch_joint" : "right_ankle_roll_joint";
    coupled_joint_ =
        test_axis_ == "pitch" ? "right_ankle_roll_joint" : "right_ankle_pitch_joint";
  }
}

bool AnkleIdentifierModule::LoadStartupPoseTargets() {
  startup_target_positions_.clear();
  if (startup_pose_mode_ == StartupPoseMode::kCurrent) return true;

  YAML::Node cfg_node = YAML::LoadFile(reference_control_cfg_path_);
  const auto joint_list = cfg_node["joint_list"].as<std::vector<std::string>>();
  for (const auto& joint_name : joint_list) {
    startup_target_positions_[joint_name] = 0.0;
  }

  if (startup_pose_mode_ == StartupPoseMode::kZero) return true;

  const auto stand_cfg = cfg_node["controllers"]["pd_stand"];
  const auto stand_joints = stand_cfg["joint_list"].as<std::vector<std::string>>();
  const auto stand_init = stand_cfg["init_state"].as<std::vector<double>>();
  if (stand_joints.size() != stand_init.size()) {
    AIMRT_ERROR("pd_stand joint_list size {} != init_state size {}", stand_joints.size(),
                stand_init.size());
    return false;
  }

  for (size_t i = 0; i < stand_joints.size(); ++i) {
    startup_target_positions_[stand_joints[i]] = stand_init[i];
  }
  return true;
}

void AnkleIdentifierModule::PrepareCsv() {
  const auto path = std::filesystem::path(csv_path_);
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  csv_.open(csv_path_, std::ios::out | std::ios::trunc);
  AIMRT_CHECK_ERROR_THROW(csv_.is_open(), "Open csv {} failed.", csv_path_);
  csv_ << "time_sec,phase,iteration,primary_joint,coupled_joint,target_primary,target_coupled,"
          "actual_primary,actual_coupled,actual_primary_vel,actual_coupled_vel,actual_primary_effort,"
          "actual_coupled_effort,imu_w,imu_x,imu_y,imu_z,gyro_x,gyro_y,gyro_z\n";
}

void AnkleIdentifierModule::MainLoop() {
  const auto period =
      std::chrono::nanoseconds(static_cast<uint64_t>(1e9 / std::max(publish_rate_hz_, 1.0)));
  auto next_loop_time = std::chrono::steady_clock::now();

  while (run_flag_.load()) {
    next_loop_time += period;

    if (!have_joint_index_.load()) {
      std::this_thread::sleep_until(next_loop_time);
      continue;
    }

    if (!baseline_captured_.load()) {
      PublishStartupPoseHoldCommand();
      TryCaptureStableBaseline();
      std::this_thread::sleep_until(next_loop_time);
      continue;
    }

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    StepControl(elapsed);
    std::this_thread::sleep_until(next_loop_time);
  }
}

void AnkleIdentifierModule::OnJointState(
    const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!have_joint_index_.load()) {
    joint_names_ = msg->name;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_index_[msg->name[i]] = i;
    }
    if (!joint_index_.count(primary_joint_) || !joint_index_.count(coupled_joint_)) {
      AIMRT_ERROR("Target joints {} / {} not found in {}", primary_joint_, coupled_joint_,
                  joint_state_topic_);
      return;
    }

    baseline_cmd_.name = joint_names_;
    baseline_cmd_.position.resize(joint_names_.size(), 0.0);
    baseline_cmd_.velocity.resize(joint_names_.size(), 0.0);
    baseline_cmd_.effort.resize(joint_names_.size(), 0.0);
    baseline_cmd_.stiffness.resize(joint_names_.size(), hold_kp_);
    baseline_cmd_.damping.resize(joint_names_.size(), hold_kd_);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto [kp, kd] = GetHoldGains(joint_names_[i]);
      baseline_cmd_.stiffness[i] = kp;
      baseline_cmd_.damping[i] = kd;
    }
    have_joint_index_.store(true);
    AIMRT_INFO("Joint index initialized with {} joints.", joint_names_.size());
  }

  for (size_t i = 0; i < msg->name.size(); ++i) {
    latest_joint_state_[msg->name[i]] = JointSnapshot{
        .position = i < msg->position.size() ? msg->position[i] : 0.0,
        .velocity = i < msg->velocity.size() ? msg->velocity[i] : 0.0,
        .effort = i < msg->effort.size() ? msg->effort[i] : 0.0};
  }

}

void AnkleIdentifierModule::OnImu(const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
  if (!use_imu_) return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_imu_ = *msg;
}

void AnkleIdentifierModule::StepControl(double elapsed_sec) {
  const double cycle_sec = pre_hold_sec_ + active_sec_ + post_hold_sec_;
  const int iteration = static_cast<int>(std::floor(elapsed_sec / cycle_sec));

  if (iteration >= repeat_count_) {
    PublishHoldCommand();
    if (auto_stop_after_test_ && !completion_logged_.exchange(true)) {
      if (csv_.is_open()) csv_.flush();
      AIMRT_INFO("Test completed. CSV written to {}", csv_path_);
      test_completed_.store(true);
    }
    return;
  }

  const double local_time = elapsed_sec - iteration * cycle_sec;
  std::string phase = "pre_hold";
  double primary_target = 0.0;
  double coupled_target = 0.0;
  my_ros2_proto::msg::JointCommand cmd;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    primary_target = GetBaseline(primary_joint_);
    coupled_target = GetBaseline(coupled_joint_);

    if (local_time < pre_hold_sec_) {
      phase = "pre_hold";
    } else if (local_time < pre_hold_sec_ + active_sec_) {
      phase = "active";
      const double active_time = local_time - pre_hold_sec_;
      if (test_mode_ == TestMode::kStep) {
        primary_target += step_amplitude_rad_;
      } else {
        primary_target +=
            sine_amplitude_rad_ * std::sin(2.0 * M_PI * sine_frequency_hz_ * active_time);
      }
    } else {
      phase = "post_hold";
    }

    cmd = baseline_cmd_;
    SetJointCmd(cmd, primary_joint_, primary_target, DesiredPrimaryVelocity(local_time), 0.0,
                test_kp_, test_kd_);
    SetJointCmd(cmd, coupled_joint_, coupled_target, 0.0, 0.0, test_kp_, test_kd_);
  }

  aimrt::channel::Publish<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_, cmd);
  LogSample(elapsed_sec, phase, iteration + 1, primary_target, coupled_target);
}

bool AnkleIdentifierModule::TryCaptureStableBaseline() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (baseline_captured_.load()) return true;

  const auto primary = latest_joint_state_[primary_joint_];
  const auto coupled = latest_joint_state_[coupled_joint_];
  const double max_joint_vel = std::max(std::abs(primary.velocity), std::abs(coupled.velocity));
  const double gyro_norm =
      std::sqrt(latest_imu_.angular_velocity.x * latest_imu_.angular_velocity.x +
                latest_imu_.angular_velocity.y * latest_imu_.angular_velocity.y +
                latest_imu_.angular_velocity.z * latest_imu_.angular_velocity.z);
  const bool joint_stable = max_joint_vel <= startup_joint_vel_threshold_;
  const bool imu_stable = !use_imu_ || gyro_norm <= startup_gyro_threshold_;
  const bool stable_now = joint_stable && imu_stable;
  const auto now = std::chrono::steady_clock::now();

  if (!stable_now) {
    startup_stable_since_.reset();
    startup_wait_logged_.store(false);
    return false;
  }

  if (!startup_stable_since_.has_value()) {
    startup_stable_since_ = now;
    if (!startup_wait_logged_.exchange(true)) {
      AIMRT_INFO(
          "Startup settle entered. Waiting {:.3f}s stable window before baseline capture. "
          "joint_vel={:.5f}, gyro_norm={:.5f}",
          startup_stable_sec_, max_joint_vel, gyro_norm);
    }
    return false;
  }

  const double stable_elapsed =
      std::chrono::duration<double>(now - startup_stable_since_.value()).count();
  if (stable_elapsed < startup_stable_sec_) return false;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    baseline_cmd_.position[i] = latest_joint_state_[joint_names_[i]].position;
  }
  start_time_ = now;
  baseline_captured_.store(true);
  AIMRT_INFO(
      "Baseline captured after startup settle. Test joint: {}, coupled joint: {}, "
      "joint_vel={:.5f}, gyro_norm={:.5f}",
      primary_joint_, coupled_joint_, max_joint_vel, gyro_norm);
  return true;
}

double AnkleIdentifierModule::DesiredPrimaryVelocity(double local_time) const {
  if (test_mode_ == TestMode::kSine &&
      local_time >= pre_hold_sec_ && local_time < pre_hold_sec_ + active_sec_) {
    const double active_time = local_time - pre_hold_sec_;
    return 2.0 * M_PI * sine_frequency_hz_ * sine_amplitude_rad_ *
           std::cos(2.0 * M_PI * sine_frequency_hz_ * active_time);
  }
  return 0.0;
}

void AnkleIdentifierModule::PublishHoldCommand() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!baseline_captured_.load()) return;
  aimrt::channel::Publish<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_, baseline_cmd_);
}

void AnkleIdentifierModule::PublishStartupPoseHoldCommand() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!have_joint_index_.load()) return;

  auto cmd = baseline_cmd_;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint_name = joint_names_[i];
    const auto [kp, kd] = GetHoldGains(joint_name);
    double target_position = 0.0;
    if (startup_pose_mode_ == StartupPoseMode::kCurrent) {
      const auto snapshot_it = latest_joint_state_.find(joint_name);
      if (snapshot_it == latest_joint_state_.end()) continue;
      target_position = snapshot_it->second.position;
    } else {
      const auto target_it = startup_target_positions_.find(joint_name);
      if (target_it == startup_target_positions_.end()) continue;
      target_position = target_it->second;
    }
    cmd.position[i] = target_position;
    cmd.velocity[i] = 0.0;
    cmd.effort[i] = 0.0;
    cmd.stiffness[i] = kp;
    cmd.damping[i] = kd;
  }
  aimrt::channel::Publish<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_, cmd);
}

std::pair<double, double> AnkleIdentifierModule::GetHoldGains(const std::string& joint_name) const {
  if (joint_name.rfind("lumbar_", 0) == 0) {
    return {torso_hold_kp_, torso_hold_kd_};
  }
  if (joint_name.find("shoulder_") != std::string::npos ||
      joint_name.find("elbow_") != std::string::npos ||
      joint_name.find("wrist_") != std::string::npos) {
    return {arm_hold_kp_, arm_hold_kd_};
  }
  if (joint_name.find("hip_") != std::string::npos ||
      joint_name.find("knee_") != std::string::npos ||
      joint_name.find("ankle_") != std::string::npos) {
    return {leg_hold_kp_, leg_hold_kd_};
  }
  return {hold_kp_, hold_kd_};
}

void AnkleIdentifierModule::SetJointCmd(my_ros2_proto::msg::JointCommand& cmd,
                                        const std::string& joint_name, double position,
                                        double velocity, double effort, double kp, double kd) {
  const size_t idx = joint_index_.at(joint_name);
  cmd.position[idx] = position;
  cmd.velocity[idx] = velocity;
  cmd.effort[idx] = effort;
  cmd.stiffness[idx] = kp;
  cmd.damping[idx] = kd;
}

double AnkleIdentifierModule::GetBaseline(const std::string& joint_name) const {
  return baseline_cmd_.position[joint_index_.at(joint_name)];
}

void AnkleIdentifierModule::LogSample(double elapsed, const std::string& phase, int iteration,
                                      double target_primary, double target_coupled) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto primary = latest_joint_state_[primary_joint_];
  const auto coupled = latest_joint_state_[coupled_joint_];

  csv_ << std::fixed << std::setprecision(6) << elapsed << "," << phase << "," << iteration << ","
       << primary_joint_ << "," << coupled_joint_ << "," << target_primary << ","
       << target_coupled << "," << primary.position << "," << coupled.position << ","
       << primary.velocity << "," << coupled.velocity << "," << primary.effort << ","
       << coupled.effort << ",";

  if (use_imu_) {
    csv_ << latest_imu_.orientation.w << "," << latest_imu_.orientation.x << ","
         << latest_imu_.orientation.y << "," << latest_imu_.orientation.z << ","
         << latest_imu_.angular_velocity.x << "," << latest_imu_.angular_velocity.y << ","
         << latest_imu_.angular_velocity.z;
  } else {
    csv_ << "0,0,0,0,0,0,0";
  }
  csv_ << "\n";
}

}  // namespace xyber_x1_infer::ankle_identifier_module
