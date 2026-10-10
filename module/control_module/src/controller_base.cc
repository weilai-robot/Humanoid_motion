#include "control_module/controller_base.h"

#include <cmath>

namespace {
int64_t SteadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

namespace xyber_x1_infer::rl_control_module {

ControllerBase::ControllerBase(const bool use_sim_handles): use_sim_handles_(use_sim_handles) {}

void ControllerBase::SetCmdData(const geometry_msgs::msg::Twist joy_data) {
  if (!std::isfinite(joy_data.linear.x) || !std::isfinite(joy_data.linear.y) ||
      !std::isfinite(joy_data.angular.z)) return;
  std::lock_guard<std::shared_mutex> lock(joy_mutex_);
  joy_data_ = joy_data;
  last_cmd_ns_.store(SteadyNowNs(), std::memory_order_release);
}

void ControllerBase::SetImuData(const sensor_msgs::msg::Imu imu_data) {
  const auto& q = imu_data.orientation;
  const auto& w = imu_data.angular_velocity;
  const double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!std::isfinite(norm_sq) || norm_sq < 0.25 || norm_sq > 2.25 ||
      !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) return;
  std::lock_guard<std::shared_mutex> lock(imu_mutex_);
  imu_data_ = imu_data;
  last_imu_ns_.store(SteadyNowNs(), std::memory_order_release);
}

void ControllerBase::SetJointStateData(const sensor_msgs::msg::JointState joint_state_data, const std::unordered_map<std::string, int> &joint_state_index_map_) {
  std::lock_guard<std::shared_mutex> lock(joint_state_mutex_);

  for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
    const auto it = joint_state_index_map_.find(joint_names_[ii]);
    if (it == joint_state_index_map_.end() || it->second < 0 ||
        static_cast<size_t>(it->second) >= joint_state_data.position.size() ||
        static_cast<size_t>(it->second) >= joint_state_data.velocity.size() ||
        static_cast<size_t>(it->second) >= joint_state_data.effort.size()) return;
    const size_t index = static_cast<size_t>(it->second);
    if (!std::isfinite(joint_state_data.position[index]) ||
        !std::isfinite(joint_state_data.velocity[index])) return;
  }
  for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
    const size_t index = static_cast<size_t>(joint_state_index_map_.at(joint_names_[ii]));
    joint_state_data_.position[ii] = joint_state_data.position[index];
    joint_state_data_.velocity[ii] = joint_state_data.velocity[index];
    joint_state_data_.effort[ii] = std::isfinite(joint_state_data.effort[index])
                                       ? joint_state_data.effort[index] : 0.0;
  }
  last_joint_state_ns_.store(SteadyNowNs(), std::memory_order_release);
}

void ControllerBase::SetActuatorCmdData(const my_ros2_proto::msg::JointCommand actuator_cmd_data) {
  std::lock_guard<std::shared_mutex> lock(actuator_state_mutex_);
  actuator_names_ = actuator_cmd_data.name;
  actuator_cmd_data_ = actuator_cmd_data;
}

void ControllerBase::SetActuatorStateData(const sensor_msgs::msg::JointState actuator_state_data) {
  std::lock_guard<std::shared_mutex> lock(actuator_state_mutex_);
  actuator_names_ = actuator_state_data.name;
  actuator_state_data_ = actuator_state_data;
}

std::vector<std::string> ControllerBase::GetJointList() {
  return joint_names_;
}

}// namespace xyber_x1_infer::rl_control_module
