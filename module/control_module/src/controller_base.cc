#include "control_module/controller_base.h"

namespace xyber_x1_infer::rl_control_module {

ControllerBase::ControllerBase(const bool use_sim_handles): use_sim_handles_(use_sim_handles) {}

void ControllerBase::SetCmdData(const geometry_msgs::msg::Twist joy_data) {
  std::lock_guard<std::shared_mutex> lock(joy_mutex_);
  joy_data_ = joy_data;
}

void ControllerBase::SetImuData(const sensor_msgs::msg::Imu imu_data) {
  std::lock_guard<std::shared_mutex> lock(imu_mutex_);
  imu_data_ = imu_data;
}

void ControllerBase::SetJointStateData(const sensor_msgs::msg::JointState joint_state_data, const std::unordered_map<std::string, int> &joint_state_index_map_) {
  std::lock_guard<std::shared_mutex> lock(joint_state_mutex_);

  for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
    joint_state_data_.position[ii] = joint_state_data.position[joint_state_index_map_.at(joint_names_[ii])];
    joint_state_data_.velocity[ii] = joint_state_data.velocity[joint_state_index_map_.at(joint_names_[ii])];
    joint_state_data_.effort[ii] = joint_state_data.effort[joint_state_index_map_.at(joint_names_[ii])];
  }
}

std::vector<std::string> ControllerBase::GetJointList() {
  return joint_names_;
}

}// namespace xyber_x1_infer::rl_control_module
