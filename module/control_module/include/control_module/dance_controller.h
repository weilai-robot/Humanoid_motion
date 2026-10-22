#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "control_module/controller_base.h"
#include "control_module/dance_state_adapter.h"
#include "control_module/dance_estimator_inputs.h"

namespace xyber_x1_infer::rl_control_module {

// BeyondMimic X1 motion tracking. Estimate body state from IMU/leg feedback or
// accept body odometry; compute the anchor with source-time waist FK.
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
  void SetOdometry(const nav_msgs::msg::Odometry& odometry);

  bool UsesOdometry() const { return use_odometry_; }
  bool UsesProprioceptiveEstimator() const { return use_proprioception_; }
  // Set during module initialization, before sensor subscriptions are started.
  void SetEstimatedOdometryCallback(std::function<void(const nav_msgs::msg::Odometry&)> callback) {
    estimated_odometry_callback_ = std::move(callback);
  }
  bool Ready(const char** reason = nullptr) const;
  std::vector<double> PreparationPositions() const;
  bool Faulted() const { return faulted_; }
  bool Finished() const { return finished_; }
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
  bool ReadSnapshot(Snapshot& snapshot, const char** reason = nullptr) const;
  void ComputeObservation(const Snapshot& snapshot, const MotionFrame& frame);
  void Infer();
  void SetFault(const char* reason);
  void AlignReference(const Snapshot& snapshot);
  float ClipTarget(int index, float target) const;
  void UpdateBodyEstimate();  // Caller holds estimator_mutex_, never sensor_mutex_.

  std::array<float, kJoints> default_pos_{};
  std::array<float, kJoints> preparation_pos_{};
  std::array<float, kJoints> kp_{};
  std::array<float, kJoints> kd_{};
  std::array<float, kJoints> effort_limit_{};
  std::array<float, kJoints> action_scale_{};
  std::array<float, kJoints> armature_{};  // Identified training dynamics; never an extra hardware torque.
  // q_policy = sign * (q_dcu - joint_offset); the inverse maps commands to DCU.
  std::array<float, kJoints> joint_sign_{};
  std::array<float, kObs> mean_{};
  std::array<float, kObs> inv_std_{};
  std::array<float, kObs> observation_{};
  std::array<float, kJoints> last_action_{};
  std::array<float, kJoints> target_pos_{};
  Snapshot current_;
  std::array<Layer, 4> layers_;
  Eigen::VectorXf hidden1_, hidden2_, hidden3_, output_;
  std::vector<MotionFrame> motion_;

  mutable std::shared_mutex sensor_mutex_;
  mutable std::shared_mutex pose_mutex_;
  mutable std::shared_mutex velocity_mutex_;
  mutable std::shared_mutex odometry_mutex_;
  struct BodyOdometry {
    builtin_interfaces::msg::Time stamp;
    std::array<double, 3> position{};
    std::array<double, 4> orientation{};
    Eigen::Vector3f linear_velocity{Eigen::Vector3f::Zero()};
    Clock::time_point received{};
  };
  static constexpr size_t kOdometryHistoryCapacity = 128;
  std::array<BodyOdometry, kOdometryHistoryCapacity> body_odometry_history_;
  size_t odometry_next_{};
  size_t odometry_count_{};
  const char* odometry_problem_{"no accepted body odometry; start the body state estimator"};
  bool use_odometry_{false};
  bool use_proprioception_{false};
  std::array<size_t, 12> leg_indices_{};
  std::mutex estimator_mutex_;
  DanceEstimatorInputs estimator_inputs_;
  DanceBodyStateEstimator body_estimator_;
  std::function<void(const nav_msgs::msg::Odometry&)> estimated_odometry_callback_;
  int64_t last_estimate_publication_ns_{};
  std::array<size_t, 3> waist_indices_{};
  DanceWaistHistory waist_history_;  // Protected by sensor_mutex_.
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
  int sensor_timeout_ms_{80};
  float max_tilt_deg_{35.0f};
  float max_start_speed_{0.2f};
  float max_start_pose_error_{0.12f};
  float max_tracking_error_{0.0f};
  float max_anchor_error_{0.6f};
  float joint_limit_margin_{0.0f};
  float joint_limit_tolerance_{0.01f};
  uint64_t tick_{0};
  size_t frame_index_{0};
  bool active_{false};
  bool faulted_{false};
  bool finished_{false};
  std::string fault_reason_;
  Eigen::Quaternionf reference_alignment_quat_{Eigen::Quaternionf::Identity()};
  Eigen::Vector3f reference_alignment_pos_{Eigen::Vector3f::Zero()};
};

}  // namespace xyber_x1_infer::rl_control_module
