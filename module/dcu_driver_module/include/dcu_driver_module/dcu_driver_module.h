// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

// cpp
#include <future>
#include <thread>
#include <unordered_map>

// projects
#include "aimrt_module_cpp_interface/module_base.h"
#include "dcu_driver_module/ankle_transmission.h"
#include "dcu_driver_module/wrist_transmission.h"
#include "dcu_driver_module/lumbar_transmission.h"
#include "dcu_driver_module/config_parse.h"
#include "dcu_driver_module/xyber_controller/xyber_api/include/xyber_controller.h"
#include "my_ros2_proto/msg/joint_command.hpp"

namespace xyber_x1_infer::dcu_driver_module {

class DcuDriverModule : public aimrt::ModuleBase {
 public:
  DcuDriverModule() = default;
  ~DcuDriverModule() override = default;

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "DcuDriverModule"};
  }

 private:
  bool InitDcu(YAML::Node& cfg_node);
  bool InitTransmission(YAML::Node& cfg_node);

  void PublishLoop();
  auto GetLogger() { return core_.GetLogger(); }

  void JointCmdCallback(const std::shared_ptr<const my_ros2_proto::msg::JointCommand>& msg);

 private:
  bool actuator_debug_ = false;
  bool enable_actuator_ = false;
  double publish_frequecy_ = 100.0f;
  std::atomic_bool is_running_ = false;
  std::string imu_dcu_name_;
  std::mutex rw_mtx_;
  std::thread publish_thread_;
  std::vector<std::string> joint_name_list_;
  std::vector<std::string> actuator_name_list_;
  std::unordered_map<std::string, DataSpace> joint_data_space_;
  std::unordered_map<std::string, DataSpace> actuator_data_space_;

  aimrt::CoreRef core_;
  aimrt::channel::PublisherRef pub_imu_;
  aimrt::channel::PublisherRef pub_joint_state_;
  aimrt::channel::PublisherRef pub_actuator_cmd_;
  aimrt::channel::PublisherRef pub_actuator_state_;
  aimrt::channel::SubscriberRef sub_joint_cmd_;

  YAML::EthercatConfig ecat_cfg_;
  YAML::DcuNetworkConfig dcu_network_cfg_;
  TransimissionManager transmission_;
  xyber::XyberControllerPtr xyber_ctrl_;
};

}  // namespace xyber_x1_infer::dcu_driver_module
