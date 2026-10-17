#include "control_module/pd_controller.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdexcept>

namespace xyber_x1_infer::rl_control_module {

PDController::PDController(const bool use_sim_handles)
    : ControllerBase(use_sim_handles) {}

void PDController::Init(const YAML::Node &cfg_node) {
  trans_mode_duration_s_ = cfg_node["transition_duration_s"]
      ? cfg_node["transition_duration_s"].as<double>() : 2.0;
  control_frequency_hz_ = cfg_node["control_frequency_hz"]
      ? cfg_node["control_frequency_hz"].as<int>() : 1000;
  feedback_max_tilt_deg_ = cfg_node["max_tilt_deg"] ? cfg_node["max_tilt_deg"].as<double>() : 90.0;
  if (!std::isfinite(trans_mode_duration_s_) || trans_mode_duration_s_ <= 0 ||
      control_frequency_hz_ <= 0 || !std::isfinite(feedback_max_tilt_deg_) ||
      feedback_max_tilt_deg_ <= 0 || feedback_max_tilt_deg_ >= 180) {
    throw std::runtime_error("invalid PD transition duration or control frequency");
  }
  // 初始化 joint_names_
  joint_names_.clear();
  joint_names_ = cfg_node["joint_list"].as<std::vector<std::string>>();
  // printf("joint_names_: ");
  // for (const auto &name : joint_names_) {
  //   printf("%s ", name.c_str());
  // }
  // printf("\n");
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);

  // 初始化 joint_conf_
  joint_conf_.init_state = Eigen::Map<vector_t>(cfg_node["init_state"].as<std::vector<double>>().data(), cfg_node["init_state"].as<std::vector<double>>().size());
  joint_conf_.stiffness = Eigen::Map<vector_t>(cfg_node["stiffness"].as<std::vector<double>>().data(), cfg_node["stiffness"].as<std::vector<double>>().size());
  joint_conf_.damping = Eigen::Map<vector_t>(cfg_node["damping"].as<std::vector<double>>().data(), cfg_node["damping"].as<std::vector<double>>().size());
  if (use_sim_handles_) {
    start_joint_angles_ = cfg_node["init_state"].as<std::vector<double>>();
  } else {
    start_joint_angles_.resize(joint_names_.size(), 0.0);
  }
  // std::cout << "init_state: " << joint_conf_.init_state.transpose() << std::endl;
  // std::cout << "stiffness: " << joint_conf_.stiffness.transpose() << std::endl;
  // std::cout << "damping: " << joint_conf_.damping.transpose() << std::endl;

  // 其他 PD 参数
  if (cfg_node["is_keep_controller"]) {
    is_keep_controller_ = cfg_node["is_keep_controller"].as<bool>();
  }

  if (cfg_node["plan_conf"]) {
    is_plan_controller_ = true;
    to_interpolate_data_ = cfg_node["plan_conf"]["trajectory_interpolator"].as<std::vector<std::vector<double>>>();
    to_interpolate_data_.insert(to_interpolate_data_.begin(), std::vector<double>{0.0}); // 在开头和结尾插入两个空的数组
    to_interpolate_data_.push_back(std::vector<double>{0.0});
  }
}

void PDController::RestartController() {
  {
    std::lock_guard<std::shared_mutex> lock(joint_state_mutex_);
    start_joint_angles_ = joint_state_data_.position;
  }
  trans_mode_percent_ = 0.0;

  if (is_plan_controller_) {
    // 在开头和结尾插入当前时刻的关节角度
    std::vector<double> temp_data = start_joint_angles_;
    temp_data.insert(temp_data.begin(), 1.0); // 1.0 表示插值时间
    to_interpolate_data_[0] = temp_data;
    to_interpolate_data_.back() = temp_data;
    trajectory_generator_.Init(to_interpolate_data_);
  }
}

void PDController::Update() {
  if (is_keep_controller_) {
    return;
  } else if (is_plan_controller_) {
    // trajectory_generator_.GetNextPoint(generated_joint_angles_);
    trajectory_generator_.GetNextPoint(start_joint_angles_);
    return;
  }

  trans_mode_percent_ += 1.0 / trans_mode_duration_s_ / control_frequency_hz_;
  trans_mode_percent_ = std::min(trans_mode_percent_, scalar_t(1));
}

bool PDController::FeedbackReady() const {
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto joint = last_joint_state_ns_.load(std::memory_order_acquire);
  const auto imu = last_imu_ns_.load(std::memory_order_acquire);
  if (joint <= 0 || imu <= 0 || now - joint > 80000000LL || now - imu > 80000000LL) return false;
  std::shared_lock<std::shared_mutex> lock(imu_mutex_);
  quaternion_t q(imu_data_.orientation.w, imu_data_.orientation.x,
                 imu_data_.orientation.y, imu_data_.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 0.5 || q.norm() > 1.5) return false;
  q.normalize();
  return (q * vector3_t::UnitZ()).z() >= std::cos(feedback_max_tilt_deg_ * std::acos(-1.0) / 180.0);
}

bool PDController::TargetReached(double max_error) const {
  if (!std::isfinite(max_error) || max_error < 0 || !FeedbackReady()) return false;
  std::shared_lock<std::shared_mutex> lock(joint_state_mutex_);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double measured = joint_state_data_.position[i];
    if (!std::isfinite(measured) || std::abs(measured - joint_conf_.init_state(i)) > max_error) return false;
  }
  return true;
}

my_ros2_proto::msg::JointCommand PDController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.resize(joint_names_.size());
  joint_cmd.velocity.resize(joint_names_.size());
  joint_cmd.effort.resize(joint_names_.size());
  joint_cmd.damping.resize(joint_names_.size());
  joint_cmd.stiffness.resize(joint_names_.size());

  for (int ii = 0; ii < joint_names_.size(); ii++) {
    double pos_des;
    if (is_keep_controller_) {
      pos_des = start_joint_angles_[ii];
    } else if (is_plan_controller_) {
      // pos_des = generated_joint_angles_[ii];
      pos_des = start_joint_angles_[ii];
    } else {
      pos_des = start_joint_angles_[ii] * (1 - trans_mode_percent_) + joint_conf_.init_state(ii) * trans_mode_percent_;
    }

    joint_cmd.position[ii] = pos_des;
    joint_cmd.velocity[ii] = 0.0;
    joint_cmd.effort[ii] = 0.0;
    joint_cmd.stiffness[ii] = joint_conf_.stiffness(ii);
    joint_cmd.damping[ii] = joint_conf_.damping(ii);
  }
  return joint_cmd;
}

}  // namespace xyber_x1_infer::rl_control_module
