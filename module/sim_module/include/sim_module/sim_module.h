// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#pragma once
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <eigen3/Eigen/Core>
#include <mutex>
#include <thread>
#include <vector>
#include <eigen3/Eigen/Core>

#include "sim_module/simulate.h"
#include "sim_module/glfw_adapter.h"
#include "sim_module/simulate.h"
#include "sim_module/array_safety.h"

#include "aimrt_module_cpp_interface/module_base.h"
#include "my_ros2_proto/msg/joint_command.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>


extern "C" {
#include <sys/errno.h>
#include <unistd.h>
}

using array_t = Eigen::Array<double, Eigen::Dynamic, 1>;
using namespace std::chrono;
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

namespace xyber_x1_infer::sim_module {

class SimModule : public aimrt::ModuleBase {
 public:
  SimModule() = default;
  ~SimModule() override = default;

  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "SimModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }
  void CmdCallback(const std::shared_ptr<const my_ros2_proto::msg::JointCommand>& msg);
  void ReadSensorData(sensor_msgs::msg::Imu& imu_data, sensor_msgs::msg::JointState& joint_state);
  void WriteMotorCmd(my_ros2_proto::msg::JointCommand cmd);
  bool IsRobotGeom(int geom_id) const;
  void PublishGroundTruth();

 private:
  aimrt::CoreRef core_;
  aimrt::channel::SubscriberRef joint_cmd_sub_;
  aimrt::channel::PublisherRef imu_data_pub_;
  aimrt::channel::PublisherRef joint_state_pub_;
  aimrt::channel::PublisherRef lidar_imu_pub_;
  aimrt::channel::PublisherRef base_pose_pub_;
  aimrt::channel::PublisherRef ground_truth_pub_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, int> joint_state_index_map_;
  aimrt::executor::ExecutorRef render_executor_;
  std::atomic<bool> is_render_thread_running_ = false;

  // pid
  array_t target_q_;
  array_t target_dq_;
  array_t target_tq_;
  array_t kp_;
  array_t kd_;
  array_t motor_torque_;

  // model and data
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  mjtNum* ctrl_noise_ = nullptr;
  std::shared_ptr<mj::Simulate> sim_;
  std::atomic<bool> sim_ready_{false};
  std::string filename_;
  mjvCamera cam_;
  mjvOption opt_;
  mjvPerturb pert_;

  // lidar IMU sensor address cache
  int lidar_imu_quat_adr_ = -1;
  int lidar_imu_gyro_adr_ = -1;
  int lidar_imu_accel_adr_ = -1;

  // ground truth / metrics tracking
  int robot_root_body_id_ = -1;
  int floor_geom_id_ = -1;
  time_point<high_resolution_clock> last_rtf_wall_time_;
  double last_rtf_sim_time_ = -1.0;
  double rtf_smoothed_ = 1.0;
  double last_gt_x_ = 0.0;
  double last_gt_y_ = 0.0;
  double cum_distance_ = 0.0;
  bool first_gt_ = true;

  time_point<high_resolution_clock> start_time_;
};

}  // namespace xyber_x1_infer::sim_module
