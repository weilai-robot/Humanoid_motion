// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "dcu_driver_module/dcu_driver_module.h"

#include <ctime>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "my_ros2_proto/msg/joy_stick_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace xyber;
using namespace std::chrono_literals;

namespace xyber_x1_infer::dcu_driver_module {

bool DcuDriverModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Load cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    YAML::Node cfg_node = YAML::LoadFile(file_path.data());
    imu_dcu_name_ = cfg_node["imu_dcu_name"].as<std::string>();
    actuator_debug_ = cfg_node["actuator_debug"].as<bool>();
    enable_actuator_ = cfg_node["enable_actuator"].as<bool>();
    publish_frequecy_ = cfg_node["publish_frequecy"].as<double>();
    joint_name_list_ = cfg_node["joint_list"].as<std::vector<std::string>>();
    actuator_name_list_ = cfg_node["actuator_list"].as<std::vector<std::string>>();
    if (const auto diag = cfg_node["diagnostics_conf"]) {
      diagnostics_enabled_ = diag["enabled"] && diag["enabled"].as<bool>();
      if (diag["frequency_hz"]) diagnostics_frequency_ = diag["frequency_hz"].as<double>();
      if (diag["duration_s"]) diagnostics_duration_s_ = diag["duration_s"].as<double>();
      if (diag["output_dir"]) diagnostics_dir_ = diag["output_dir"].as<std::string>();
      AIMRT_CHECK_ERROR_THROW(std::isfinite(diagnostics_frequency_) && diagnostics_frequency_ > 0 &&
                                 diagnostics_frequency_ <= 100 && std::isfinite(diagnostics_duration_s_) &&
                                 diagnostics_duration_s_ > 0 && diagnostics_duration_s_ <= 600,
                             "Invalid DCU diagnostics frequency/duration");
    }

    // Init DCU SDK
    AIMRT_CHECK_ERROR_THROW(InitDcu(cfg_node), "DCU initialization failed");

    // Init Transmission
    AIMRT_CHECK_ERROR_THROW(InitTransmission(cfg_node), "Transmission initialization failed");

    // Prepare publisher
    pub_imu_ = core_.GetChannelHandle().GetPublisher("/imu/data");
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::Imu>(pub_imu_);

    pub_joint_state_ = core_.GetChannelHandle().GetPublisher("/joint_states");
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::JointState>(pub_joint_state_);

    pub_actuator_cmd_ = core_.GetChannelHandle().GetPublisher("/actuator_cmd");
    aimrt::channel::RegisterPublishType<my_ros2_proto::msg::JointCommand>(pub_actuator_cmd_);

    pub_actuator_state_ = core_.GetChannelHandle().GetPublisher("/actuator_states");
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::JointState>(pub_actuator_state_);

    // Initialize subscriber
    sub_joint_cmd_ = core_.GetChannelHandle().GetSubscriber("/joint_cmd");
    aimrt::channel::Subscribe<my_ros2_proto::msg::JointCommand>(
        sub_joint_cmd_, std::bind(&DcuDriverModule::JointCmdCallback, this, std::placeholders::_1));

    AIMRT_INFO("Init succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  return true;
}

bool DcuDriverModule::Start() {
  is_running_ = true;
  AIMRT_INFO("DcuDriverModule::Start enter, publish_frequecy={}", publish_frequecy_);
  publish_thread_ = std::thread(&DcuDriverModule::PublishLoop, this);
  if (diagnostics_enabled_) {
    try { diagnostics_thread_ = std::thread(&DcuDriverModule::DiagnosticsLoop, this); }
    catch (const std::exception& e) { AIMRT_WARN("DCU diagnostic worker unavailable: {}", e.what()); }
  }
  AIMRT_INFO("DcuDriverModule::Start publish thread launched");

  return true;
}

void DcuDriverModule::Shutdown() {
  is_running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  if (diagnostics_thread_.joinable()) diagnostics_thread_.join();

  xyber_ctrl_->DisableAllActuator();
  xyber_ctrl_->Stop();
}

bool DcuDriverModule::InitDcu(YAML::Node& cfg_node) {
  // read config param
  ecat_cfg_ = cfg_node["ethercat"].as<YAML::EthercatConfig>();
  dcu_network_cfg_ = cfg_node["dcu_network"].as<YAML::DcuNetworkConfig>();

  // create XyberController
  xyber_ctrl_.reset(XyberController::GetInstance());

  // create dcu and attch actuators to it
  for (const auto& dcu_cfg : dcu_network_cfg_) {
    if (!dcu_cfg.enable) continue;
    bool ret = xyber_ctrl_->CreateDcu(dcu_cfg.name, dcu_cfg.ecat_id);
    AIMRT_CHECK_ERROR_THROW(ret, "DCU {} register failed.", dcu_cfg.name);

    for (size_t ch = 0; ch <= (size_t)CtrlChannel::CTRL_CH3; ch++) {
      for (const auto& actr : dcu_cfg.ch[ch]) {
        if (std::find(actuator_name_list_.begin(), actuator_name_list_.end(), actr.name) ==
            actuator_name_list_.end()) {
          AIMRT_WARN("Actuator {} not found in actuator_list. ignored.", actr.name);
          continue;
        }
        ret = xyber_ctrl_->AttachActuator(dcu_cfg.name, (CtrlChannel)ch, StringToType(actr.type),
                                          actr.name, actr.can_id);
        AIMRT_CHECK_ERROR_THROW(ret, "Actuator {} register failed.", actr.name);
        AIMRT_INFO("Actuator route: {} -> DCU {} (ECAT {}) / channel_{} / CAN {} / {}",
                   actr.name, dcu_cfg.name, dcu_cfg.ecat_id, ch + 1, actr.can_id, actr.type);
      }
    }
  }

  // setup ethercat stuff
  xyber_ctrl_->SetRealtime(ecat_cfg_.rt_priority, ecat_cfg_.bind_cpu);
  bool ret = xyber_ctrl_->Start(ecat_cfg_.ifname, ecat_cfg_.cycle_time_ns, ecat_cfg_.enable_dc);
  if (!ret) {
    AIMRT_ERROR_THROW("XyberController start failed.");
    return false;
  }

  // enable actuator and imu
  if (enable_actuator_) {
    for (const auto name : actuator_name_list_) {
      const bool enabled = xyber_ctrl_->EnableActuator(name);
      AIMRT_INFO("EnableActuator({}) returned {}", name, enabled);
      if (!enabled) {
        ret = false;
      }
    }
    if (!ret) {
      AIMRT_ERROR_THROW("EnableActuator failed.");
      return false;
    }

    // update actuator cmd cache
    for (auto& name : actuator_name_list_) {
      xyber_ctrl_->SetMitCmd(name, 0, 0, 0, 0, 0);
    }
  }
  for (const auto& dcu_cfg : dcu_network_cfg_) {
    if (!dcu_cfg.imu_enable) continue;
    AIMRT_DEBUG("DCU {} imu enabled.", dcu_cfg.name);
    xyber_ctrl_->ApplyDcuImuOffset(dcu_cfg.name);
  }

  return true;
}

bool DcuDriverModule::InitTransmission(YAML::Node& cfg_node) {
  auto trans_node = cfg_node["transmission"];

  // create data space
  for (const auto& name : joint_name_list_) {
    joint_data_space_[name];
  }
  for (const auto& name : actuator_name_list_) {
    actuator_data_space_[name];
  }

  // bind joint and actuator
  for (const auto& node : trans_node) {
    std::string name = node["name"].as<std::string>();
    std::string type = node["type"].as<std::string>();

    if (type == "SimpleTransmission") {
      double direction = node["direction"].as<double>();
      std::string joint = node["joint"].as<std::string>();
      std::string actuator = node["actuator"].as<std::string>();

      const auto& joint_it = joint_data_space_.find(joint);
      if (joint_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint);
        continue;
      }
      const auto& actuator_it = actuator_data_space_.find(actuator);
      if (actuator_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator);
        continue;
      }

      JointHandle joint_handle;
      joint_handle.handle = &joint_it->second;

      ActuatorHandle actuator_handle;
      actuator_handle.direction = direction;
      actuator_handle.handle = &actuator_it->second;

      auto trans = new SimpleTransmission(name, actuator_handle, joint_handle);
      transmission_.RegisterTransmission(trans);

      // AIMRT_DEBUG("Add Transmission {}, type {}, bind actuator {}, joint {} .", name, type,
      //             actuator, joint);

    } else if (type == "LeftAnkleParallelTransmission" ||
               type == "RightAnkleParallelTransmission") {
      double direction_left = node["direction_left"].as<double>();
      double direction_right = node["direction_right"].as<double>();
      std::string param_path = node["param_path"].as<std::string>();
      std::string joint_pitch = node["joint_pitch"].as<std::string>();
      std::string joint_roll = node["joint_roll"].as<std::string>();
      std::string actuator_left = node["actuator_left"].as<std::string>();
      std::string actuator_right = node["actuator_right"].as<std::string>();

      const auto& joint_pitch_it = joint_data_space_.find(joint_pitch);
      if (joint_pitch_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_pitch);
        continue;
      }
      const auto& joint_roll_it = joint_data_space_.find(joint_roll);
      if (joint_roll_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_roll);
        continue;
      }
      const auto& actuator_left_it = actuator_data_space_.find(actuator_left);
      if (actuator_left_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_left);
        continue;
      }
      const auto& actuator_right_it = actuator_data_space_.find(actuator_right);
      if (actuator_right_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_right);
        continue;
      }

      JointHandle joint_pitch_data;
      joint_pitch_data.handle = &joint_pitch_it->second;

      JointHandle joint_roll_data;
      joint_roll_data.handle = &joint_roll_it->second;

      ActuatorHandle actr_left_data;
      actr_left_data.direction = direction_left;
      actr_left_data.handle = &actuator_left_it->second;

      ActuatorHandle actr_right_data;
      actr_right_data.direction = direction_right;
      actr_right_data.handle = &actuator_right_it->second;

      if (type == "LeftAnkleParallelTransmission") {
        auto trans = new LeftAnkleParallelTransmission(
            name, param_path, actr_left_data, actr_right_data, joint_pitch_data, joint_roll_data);
        transmission_.RegisterTransmission(trans);
      } else {
        auto trans = new RightAnkleParallelTransmission(
            name, param_path, actr_left_data, actr_right_data, joint_pitch_data, joint_roll_data);
        transmission_.RegisterTransmission(trans);
      }
      // AIMRT_DEBUG(
      //     "Add Transmission {}, type {}, bind actr left {}, actr right {}, joint pitch {},
      //     joint roll {} .", name, type, actuator_left, actuator_right, joint_pitch,
      //     joint_roll);
    } else if (type == "LeftWristParallelTransmission" ||
               type == "RightWristParallelTransmission") {
      double direction_left = node["direction_left"].as<double>();
      double direction_right = node["direction_right"].as<double>();
      std::string param_path = node["param_path"].as<std::string>();
      std::string joint_pitch = node["joint_pitch"].as<std::string>();
      std::string joint_roll = node["joint_roll"].as<std::string>();
      std::string actuator_left = node["actuator_left"].as<std::string>();
      std::string actuator_right = node["actuator_right"].as<std::string>();

      const auto& joint_pitch_it = joint_data_space_.find(joint_pitch);
      if (joint_pitch_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_pitch);
        continue;
      }
      const auto& joint_roll_it = joint_data_space_.find(joint_roll);
      if (joint_roll_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_roll);
        continue;
      }
      const auto& actuator_left_it = actuator_data_space_.find(actuator_left);
      if (actuator_left_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_left);
        continue;
      }
      const auto& actuator_right_it = actuator_data_space_.find(actuator_right);
      if (actuator_right_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_right);
        continue;
      }

      JointHandle joint_pitch_data;
      joint_pitch_data.handle = &joint_pitch_it->second;

      JointHandle joint_roll_data;
      joint_roll_data.handle = &joint_roll_it->second;

      ActuatorHandle actr_left_data;
      actr_left_data.direction = direction_left;
      actr_left_data.handle = &actuator_left_it->second;

      ActuatorHandle actr_right_data;
      actr_right_data.direction = direction_right;
      actr_right_data.handle = &actuator_right_it->second;

      if (type == "LeftWristParallelTransmission") {
        auto trans = new LeftWristParallelTransmission(
            name, param_path, actr_left_data, actr_right_data, joint_pitch_data, joint_roll_data);
        transmission_.RegisterTransmission(trans);
      } else {
        auto trans = new RightWristParallelTransmission(
            name, param_path, actr_left_data, actr_right_data, joint_pitch_data, joint_roll_data);
        transmission_.RegisterTransmission(trans);
      }
    } else if (type == "LumbarParallelTransmission") {
      double direction_left = node["direction_left"].as<double>();
      double direction_right = node["direction_right"].as<double>();
      std::string param_path = node["param_path"].as<std::string>();
      std::string joint_pitch = node["joint_pitch"].as<std::string>();
      std::string joint_roll = node["joint_roll"].as<std::string>();
      std::string actuator_left = node["actuator_left"].as<std::string>();
      std::string actuator_right = node["actuator_right"].as<std::string>();

      const auto& joint_pitch_it = joint_data_space_.find(joint_pitch);
      if (joint_pitch_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_pitch);
        continue;
      }
      const auto& joint_roll_it = joint_data_space_.find(joint_roll);
      if (joint_roll_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_roll);
        continue;
      }
      const auto& actuator_left_it = actuator_data_space_.find(actuator_left);
      if (actuator_left_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_left);
        continue;
      }
      const auto& actuator_right_it = actuator_data_space_.find(actuator_right);
      if (actuator_right_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_right);
        continue;
      }

      JointHandle joint_pitch_data;
      joint_pitch_data.handle = &joint_pitch_it->second;

      JointHandle joint_roll_data;
      joint_roll_data.handle = &joint_roll_it->second;

      ActuatorHandle actr_left_data;
      actr_left_data.direction = direction_left;
      actr_left_data.handle = &actuator_left_it->second;

      ActuatorHandle actr_right_data;
      actr_right_data.direction = direction_right;
      actr_right_data.handle = &actuator_right_it->second;

      auto trans = new LumbarParallelTransmission(name, param_path, actr_left_data, actr_right_data,
                                                  joint_pitch_data, joint_roll_data);
      transmission_.RegisterTransmission(trans);
    } else {
      AIMRT_ERROR_THROW("Transmission {} type {} not support.", name, type);
      return false;
    }
  }

  return true;
}

void DcuDriverModule::PublishLoop() {
  AIMRT_INFO("PublishLoop started");
  // create publish proxy
  aimrt::channel::PublisherProxy<sensor_msgs::msg::Imu> pub_imu(pub_imu_);
  aimrt::channel::PublisherProxy<sensor_msgs::msg::JointState> pub_joint_state(pub_joint_state_);
  aimrt::channel::PublisherProxy<sensor_msgs::msg::JointState> pub_actuator_state(
      pub_actuator_state_);
  aimrt::channel::PublisherProxy<my_ros2_proto::msg::JointCommand> pub_actuator_cmd(
      pub_actuator_cmd_);

  sensor_msgs::msg::Imu imu_msg;
  sensor_msgs::msg::JointState js_msg;
  // for (const auto& [name, data] : joint_data_space_) {
  //   js_msg.name.push_back(name);
  // }
  js_msg.name = joint_name_list_;
  js_msg.effort.resize(js_msg.name.size());
  js_msg.velocity.resize(js_msg.name.size());
  js_msg.position.resize(js_msg.name.size());

  sensor_msgs::msg::JointState actr_state;
  actr_state.name = actuator_name_list_;
  actr_state.effort.resize(actr_state.name.size());
  actr_state.velocity.resize(actr_state.name.size());
  actr_state.position.resize(actr_state.name.size());

  my_ros2_proto::msg::JointCommand actr_cmd;
  actr_cmd.name = actuator_name_list_;
  actr_cmd.effort.resize(actr_cmd.name.size());
  actr_cmd.velocity.resize(actr_cmd.name.size());
  actr_cmd.position.resize(actr_cmd.name.size());
  actr_cmd.stiffness.resize(actr_cmd.name.size());
  actr_cmd.damping.resize(actr_cmd.name.size());

  auto period = std::chrono::nanoseconds((uint64_t)(1 / publish_frequecy_ * 1000000000));
  auto next_loop_time = std::chrono::steady_clock::now();
  uint64_t loop_count = 0;
  int64_t last_joint_pdo_ns = 0;
  while (is_running_) {
    // get time
    timeval now;
    gettimeofday(&now, NULL);

    builtin_interfaces::msg::Time stamp;
    stamp.sec = now.tv_sec;
    stamp.nanosec = now.tv_usec * 1000;

    // Read SDK feedback into thread-local messages before locking shared data.
    int64_t joint_pdo_ns = std::numeric_limits<int64_t>::max();
    bool all_pdo_seen = !actuator_name_list_.empty();
    for (size_t i = 0; i < actuator_name_list_.size(); ++i) {
      const auto& name = actuator_name_list_[i];
      ActuatorStateSnapshot state;
      if (xyber_ctrl_->GetActuatorState(name, state)) {
        actr_state.effort[i] = state.effort;
        actr_state.velocity[i] = state.velocity;
        actr_state.position[i] = state.position;
        all_pdo_seen = all_pdo_seen && state.pdo_received_ns > 0;
        joint_pdo_ns = std::min(joint_pdo_ns, state.pdo_received_ns);
      } else {
        all_pdo_seen = false;
      }
    }
    // Snapshot feedback and command caches under the same lock as JointCmdCallback.
    {
      std::lock_guard<std::mutex> lock(rw_mtx_);
      for (size_t i = 0; i < actuator_name_list_.size(); ++i) {
        auto& state = actuator_data_space_.at(actuator_name_list_[i]).state;
        state.effort = actr_state.effort[i];
        state.velocity = actr_state.velocity[i];
        state.position = actr_state.position[i];
      }
      transmission_.TransformActuatorToJoint();
      for (size_t i = 0; i < joint_name_list_.size(); i++) {
        const auto& state = joint_data_space_.at(js_msg.name[i]).state;
        js_msg.effort[i] = state.effort;
        js_msg.velocity[i] = state.velocity;
        js_msg.position[i] = state.position;
      }
      if (actuator_debug_) {
        for (size_t i = 0; i < actuator_name_list_.size(); i++) {
          const auto& cmd = actuator_data_space_.at(actuator_name_list_[i]).cmd;
          actr_cmd.effort[i] = cmd.effort;
          actr_cmd.velocity[i] = cmd.velocity;
          actr_cmd.position[i] = cmd.position;
          actr_cmd.stiffness[i] = cmd.kp;
          actr_cmd.damping[i] = cmd.kd;
        }
      }
    }
    js_msg.header.stamp = stamp;
    // Do not make a stopped PDO cache look fresh by stamping it every 1 ms.
    // Constant joint positions are fine when new valid PDOs actually arrive.
    if (all_pdo_seen && joint_pdo_ns > last_joint_pdo_ns) {
      pub_joint_state.Publish(js_msg);
      last_joint_pdo_ns = joint_pdo_ns;
    }
    if (actuator_debug_) {
      actr_state.header.stamp = stamp;
      actr_cmd.header.stamp = stamp;
      pub_actuator_cmd.Publish(actr_cmd);
      pub_actuator_state.Publish(actr_state);
    }

    // publish imu
    DcuImu imu = xyber_ctrl_->GetDcuImuData(imu_dcu_name_);
    imu_msg.angular_velocity.x = imu.gyro[0] / 180 * M_PI;
    imu_msg.angular_velocity.y = imu.gyro[1] / 180 * M_PI;
    imu_msg.angular_velocity.z = imu.gyro[2] / 180 * M_PI;
    imu_msg.linear_acceleration.x = imu.acc[0];
    imu_msg.linear_acceleration.y = imu.acc[1];
    imu_msg.linear_acceleration.z = imu.acc[2];
    imu_msg.orientation.w = imu.quat[0];
    imu_msg.orientation.x = imu.quat[1];
    imu_msg.orientation.y = imu.quat[2];
    imu_msg.orientation.z = imu.quat[3];
    imu_msg.header.stamp = stamp;
    pub_imu.Publish(imu_msg);
    if (++loop_count % 1000 == 0) {
      AIMRT_INFO("PublishLoop heartbeat count={}, joint0={}, imu_w={}", loop_count,
                 js_msg.position.empty() ? 0.0 : js_msg.position[0], imu_msg.orientation.w);
    }
    // yumx: 打印 IMU 数据（每100帧一次）
    /*
    static uint64_t imu_print_cnt = 0;
    if (++imu_print_cnt % 100 == 0) {
      AIMRT_INFO("[IMU] quat(w,x,y,z)=({:.3f}, {:.3f}, {:.3f}, {:.3f}) | "
                 "gyro(rad/s)=({:.3f}, {:.3f}, {:.3f}) | "
                 "acc(m/s2)=({:.3f}, {:.3f}, {:.3f})",
                 imu_msg.orientation.w, imu_msg.orientation.x,
                 imu_msg.orientation.y, imu_msg.orientation.z,
                 imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
                 imu_msg.angular_velocity.z,
                 imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                 imu_msg.linear_acceleration.z);
    }
    */
    next_loop_time += period;
    std::this_thread::sleep_until(next_loop_time);
  }

  AIMRT_INFO("PublishLoop exited");
}

void DcuDriverModule::JointCmdCallback(
    const std::shared_ptr<const my_ros2_proto::msg::JointCommand>& msg) {
  // AIMRT_DEBUG("Received joint cmd data: {}", my_ros2_proto::msg::to_yaml(*msg));
  if (!is_running_) return;
  const size_t n = msg->name.size();
  if (msg->position.size() < n || msg->velocity.size() < n || msg->effort.size() < n ||
      msg->stiffness.size() < n || msg->damping.size() < n) {
    AIMRT_WARN("Reject incomplete joint command");
    return;
  }
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(msg->position[i]) || !std::isfinite(msg->velocity[i]) ||
        !std::isfinite(msg->effort[i]) || !std::isfinite(msg->stiffness[i]) ||
        !std::isfinite(msg->damping[i])) {
      AIMRT_WARN("Reject non-finite joint command");
      return;
    }
  }

  // Keep command writes, transmission, and SDK dispatch consistent with snapshots.
  std::lock_guard<std::mutex> lock(rw_mtx_);
  // cache cmd data
  for (size_t i = 0; i < msg->name.size(); i++) {
    auto it = joint_data_space_.find(msg->name[i]);
    if (it == joint_data_space_.end()) {
      AIMRT_WARN("JointCmdCallback joint {} not found.", msg->name[i]);
      AIMRT_WARN("num {}", i);

      continue;
    }
    it->second.cmd.effort = msg->effort[i];
    it->second.cmd.velocity = msg->velocity[i];
    it->second.cmd.position = msg->position[i];
    it->second.cmd.kp = msg->stiffness[i];
    it->second.cmd.kd = msg->damping[i];
  }
  // transmission
  transmission_.TransformJointToActuator();
  // send to actuator
  for (const auto& [name, data] : actuator_data_space_) {
    // AIMRT_DEBUG("Send actuator {} pos {} vel {} tor {} kp {} kd {}.", name, data.cmd.position,
    //             data.cmd.velocity, data.cmd.effort, data.cmd.kp, data.cmd.kd);
    xyber_ctrl_->SetMitCmd(name, data.cmd.position, data.cmd.velocity, data.cmd.effort, data.cmd.kp,
                           data.cmd.kd);
  }
  last_joint_cmd_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_release);
}

void DcuDriverModule::DiagnosticsLoop() {
  // File formatting/IO stays on this bounded, non-RT worker. It never changes
  // commands or resets a motor. Do not hold rw_mtx_ while acquiring SDK locks.
  try {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "dcu_diag");
    sched_param scheduling{};
    const int sched_result = pthread_setschedparam(pthread_self(), SCHED_OTHER, &scheduling);
    if (sched_result != 0) {
      AIMRT_WARN("DCU diagnostics cannot select non-RT scheduling: {}", sched_result);
      return;
    }
#endif
    std::filesystem::create_directories(diagnostics_dir_);
    const auto wall = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&wall, &tm);
    std::ostringstream filename;
    filename << diagnostics_dir_ << "/dcu_diag_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << '_'
             << std::chrono::steady_clock::now().time_since_epoch().count() << ".csv";
    std::ofstream file(filename.str());
    if (!file) { AIMRT_WARN("Cannot open DCU diagnostics {}", filename.str()); return; }
    file.exceptions(std::ofstream::badbit | std::ofstream::failbit);
    file << "timestamp_ns,steady_ns,frame,actuator,dcu,ecat_id,channel,can_id,type,"
            "rx_seen,rx_pdo_seq,rx_age_ms,rx_content_seq,rx_content_age_ms,"
            "rx_measurement_seq,rx_measurement_age_ms,tx_pdo_seq,tx_age_ms,tx_command_seq,"
            "tx_format,tx_mask,tx_slot_enabled,position,velocity,effort,"
            "status_byte,heartbeat,power_state,error_nibble,"
            "cmd_position,cmd_velocity,cmd_effort,cmd_kp,cmd_kd,cmd_current_limit,"
            "host_kp_max,host_kd_max,host_effort_max,"
            "cache_cmd_position,cache_cmd_velocity,cache_cmd_effort,cache_cmd_kp,cache_cmd_kd,"
            "joint_cmd_age_ms,host_pd_effort,rx_hex,tx_hex\n";
    file << std::setprecision(10);
    const auto hex = [](const std::array<uint8_t,8>& bytes) {
      std::ostringstream out;
      for (auto b : bytes) out << std::hex << std::setfill('0') << std::setw(2) << unsigned(b);
      return out.str();
    };
    const auto text = [](const std::string& value) {
      std::string out = "\"";
      for (char c : value) { if (c == '"') out += '"'; out += c; }
      return out + '"';
    };
    const auto age = [](int64_t now, int64_t stamp) {
      return stamp > 0 && now >= stamp ? (now-stamp)/1e6 : -1.0;
    };
    std::vector<DataSpace> cache(actuator_name_list_.size());
    std::unordered_map<int,int64_t> last_repetition_warning;
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0/diagnostics_frequency_));
    uint64_t frame = 0;
    AIMRT_INFO("DCU diagnostics: {} at {} Hz, {} s", filename.str(),
               diagnostics_frequency_, diagnostics_duration_s_);
    while (is_running_ && std::chrono::duration<double>(
               std::chrono::steady_clock::now()-start).count() < diagnostics_duration_s_) {
      int64_t cmd_stamp;
      {
        std::lock_guard<std::mutex> lock(rw_mtx_);
        for (size_t i=0;i<cache.size();++i) cache[i] = actuator_data_space_.at(actuator_name_list_[i]);
        cmd_stamp = last_joint_cmd_ns_.load(std::memory_order_acquire);
      }
      std::unordered_map<int,int> repetition_counts;
      for (size_t i=0;i<cache.size();++i) {
        const auto& name = actuator_name_list_[i];
        ActuatorDiagnostics s;
        if (!xyber_ctrl_->GetActuatorDiagnostics(name,s)) continue;
        const auto now = std::chrono::steady_clock::now();
        const int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        const int64_t wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::string dcu;
        for (const auto& route : dcu_network_cfg_) if (route.ecat_id == s.ecat_id) dcu = route.name;
        const bool r = s.type == ActuatorType::POWER_FLOW_R86 || s.type == ActuatorType::POWER_FLOW_R52;
        const int status = r && s.rx_seen ? s.rx[5] : -1;
        if (r && status >= 0 && ((status>>1)&7) == STATE_ENABLE &&
            age(ns,s.rx_pdo_ns) >= 0 && age(ns,s.rx_pdo_ns) < 100 &&
            age(ns,s.rx_measurement_ns) >= 200 && std::abs(s.state.velocity) >= .1) {
          ++repetition_counts[s.ecat_id*4+s.channel];
        }
        const double pd = s.tx_format == 1 && s.tx_pdo_seq && s.rx_seen
            ? s.cmd_effort + s.cmd_kp*(s.cmd_position-s.state.position) +
              s.cmd_kd*(s.cmd_velocity-s.state.velocity) : std::numeric_limits<double>::quiet_NaN();
        const auto& c = cache[i].cmd;
        file << wall_ns << ',' << ns << ',' << frame << ',' << text(name) << ',' << text(dcu) << ','
             << s.ecat_id << ',' << s.channel << ',' << s.can_id << ',' << int(s.type) << ','
             << s.rx_seen << ',' << s.rx_pdo_seq << ',' << age(ns,s.rx_pdo_ns) << ','
             << s.rx_content_seq << ',' << age(ns,s.rx_content_ns) << ','
             << s.rx_measurement_seq << ',' << age(ns,s.rx_measurement_ns) << ','
             << s.tx_pdo_seq << ',' << age(ns,s.tx_pdo_ns) << ',' << s.tx_command_seq << ','
             << unsigned(s.tx_format) << ',' << unsigned(s.tx_mask) << ','
             << ((s.tx_mask & (1u << (s.can_id-1))) != 0) << ','
             << s.state.position << ',' << s.state.velocity << ',' << s.state.effort << ','
             << status << ',' << (status < 0 ? -1 : status&1) << ','
             << (status < 0 ? -1 : (status>>1)&7) << ',' << (status < 0 ? -1 : status>>4) << ','
             << s.cmd_position << ',' << s.cmd_velocity << ',' << s.cmd_effort << ','
             << s.cmd_kp << ',' << s.cmd_kd << ',' << s.cmd_current_limit << ','
             << s.mit_param.kp_max << ',' << s.mit_param.kd_max << ',' << s.mit_param.toq_max << ','
             << c.position << ',' << c.velocity << ',' << c.effort << ',' << c.kp << ',' << c.kd << ','
             << age(ns,cmd_stamp) << ',' << pd << ',' << hex(s.rx) << ',' << hex(s.tx) << '\n';
      }
      const int64_t warning_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      for (const auto& [group,count] : repetition_counts) {
        if (warning_ns-last_repetition_warning[group] >= 2'000'000'000LL) {
          AIMRT_WARN("DCU suspicious repeated moving feedback: ECAT {} channel_{} motors={}; "
                     "PDO still arriving, measurement bytes unchanged >=200ms with nonzero velocity. "
                     "Check dcu_diag; stop AMP testing if this persists.", group/4,group%4,count);
          last_repetition_warning[group] = warning_ns;
        }
      }
      if (++frame % 50 == 0) file.flush();
      next += period;
      // After IO/scheduling stalls, resume sampling instead of flooding catch-up rows.
      if (next < std::chrono::steady_clock::now()) next = std::chrono::steady_clock::now()+period;
      std::this_thread::sleep_until(next);
    }
    file.flush();
    AIMRT_INFO("DCU diagnostics completed: {}, {} frames", filename.str(), frame);
  } catch (const std::exception& e) {
    AIMRT_WARN("DCU diagnostics stopped: {}", e.what());
  }
}

}  // namespace xyber_x1_infer::dcu_driver_module
