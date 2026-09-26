#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstdint>
#include <shared_mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "control_module/controller_base.h"

namespace xyber_x1_infer::rl_control_module {

// BeyondMimic X1 motion tracking. The state estimator must publish the pose of
// lumbar_pitch_link in odom and the base_link linear velocity in body axes.
// Missing or stale estimates prevent entry and stop a running dance.
class DanceController final : public ControllerBase {
 public:
  explicit DanceController(bool use_sim_handles);
  void Init(const YAML::Node& cfg_node) override;
  void RestartController() override;
  void Update() override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;

  void SetImuData(const sensor_msgs::msg::Imu imu_data) override;
  void SetJointStateData(
      const sensor_msgs::msg::JointState joint_state_data,
      const std::unordered_map<std::string, int>& joint_state_index_map) override;
  void SetAnchorPose(const geometry_msgs::msg::TransformStamped& pose);
  void SetBaseVelocity(const geometry_msgs::msg::TwistStamped& velocity);

  bool Ready() const;
  bool Faulted() const { return faulted_; }
  const std::string& FaultReason() const { return fault_reason_; }

 private:
  static constexpr int kJoints = 29;
  static constexpr int kObs = 160;
  using Clock = std::chrono::steady_clock;
  using RowMajorMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  struct Layer {
    RowMajorMatrix weight;
    Eigen::VectorXf bias;
  };
  struct MotionFrame {
    std::array<float, kJoints> joint_pos{};
    std::array<float, kJoints> joint_vel{};
    Eigen::Vector3f anchor_pos{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf anchor_quat{Eigen::Quaternionf::Identity()};
  };
  struct Snapshot {
    std::array<double, kJoints> joint_pos{};
    std::array<double, kJoints> joint_vel{};
    Eigen::Quaternionf base_quat{Eigen::Quaternionf::Identity()};
    Eigen::Vector3f base_ang_vel{Eigen::Vector3f::Zero()};
    Eigen::Vector3f base_lin_vel{Eigen::Vector3f::Zero()};
    Eigen::Vector3f anchor_pos{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf anchor_quat{Eigen::Quaternionf::Identity()};
  };

  void LoadModel(const std::string& path);
  void LoadMotion(const std::string& path);
  bool ReadSnapshot(Snapshot& snapshot) const;
  void ComputeObservation(const Snapshot& snapshot, const MotionFrame& frame);
  void Infer();
  void SetFault(const char* reason);
  float ClipTarget(int index, float target) const;

  std::array<float, kJoints> default_pos_{};
  std::array<float, kJoints> kp_{};
  std::array<float, kJoints> kd_{};
  std::array<float, kJoints> effort_limit_{};
  std::array<float, kJoints> action_scale_{};
  // q_policy = sign * (q_dcu - joint_offset); the inverse maps commands to DCU.
  std::array<float, kJoints> joint_sign_{};
  std::array<float, kObs> mean_{};
  std::array<float, kObs> inv_std_{};
  std::array<float, kObs> observation_{};
  std::array<float, kJoints> last_action_{};
  std::array<float, kJoints> target_pos_{};
  std::array<float, kJoints> start_pos_{};
  Snapshot current_;
  std::array<Layer, 4> layers_;
  Eigen::VectorXf hidden1_, hidden2_, hidden3_, output_;
  std::vector<MotionFrame> motion_;

  mutable std::shared_mutex sensor_mutex_;
  mutable std::shared_mutex pose_mutex_;
  mutable std::shared_mutex velocity_mutex_;
  geometry_msgs::msg::TransformStamped anchor_pose_;
  geometry_msgs::msg::TwistStamped base_velocity_;
  Clock::time_point last_joint_{};
  Clock::time_point last_imu_{};
  Clock::time_point last_pose_{};
  Clock::time_point last_velocity_{};

  std::string odom_frame_;
  std::string anchor_frame_;
  std::string base_frame_;
  int decimation_{20};
  int prepare_ticks_{4000};
  int sensor_timeout_ms_{100};
  float max_tilt_deg_{45.0f};
  float max_start_speed_{0.2f};
  float max_tracking_error_{0.8f};
  float max_start_pose_error_{0.25f};
  float max_anchor_error_{0.4f};
  float joint_limit_margin_{0.03f};
  bool sign_calibration_verified_{false};
  uint64_t tick_{0};
  size_t frame_index_{0};
  bool active_{false};
  bool faulted_{false};
  std::string fault_reason_;
  Eigen::Quaternionf reference_alignment_quat_{Eigen::Quaternionf::Identity()};
  Eigen::Vector3f reference_alignment_pos_{Eigen::Vector3f::Zero()};
};

}  // namespace xyber_x1_infer::rl_control_module
