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
#include "control_module/dance_diagnostics.h"

namespace xyber_x1_infer::rl_control_module {

// BeyondMimic X1 motion tracking. The 154-input actor uses IMU + waist FK;
// the 160-input actor additionally requires translation/velocity estimates.
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
  bool UsesImuOnly() const { return use_imu_only_; }
  // Set during module initialization, before sensor subscriptions are started.
  void SetEstimatedOdometryCallback(std::function<void(const nav_msgs::msg::Odometry&)> callback) {
    estimated_odometry_callback_ = std::move(callback);
  }
  bool Ready(const char** reason = nullptr) const;
  std::vector<double> PreparationPositions() const;
  std::vector<double> HandoffTargets() const;
  void ValidateFinisher(const YAML::Node& cfg) const;
  bool Faulted() const { return faulted_; }
  bool Finished() const { return finished_; }
  const std::string& FaultReason() const { return fault_reason_; }
  void SetDiagnosticState(const std::string& state);
  void SampleDiagnostics();  // called at 1 kHz; passive phases are sampled at 50 Hz
  void StopDiagnostics() { diagnostics_.Stop(); }
  const std::string& DiagnosticPath() const { return diagnostics_.Path(); }
  bool DiagnosticIoError() const { return diagnostics_.IoError(); }
  bool DiagnosticsEnabled() const { return diagnostics_.Enabled(); }

 private:
  static constexpr int kJoints = 29;
  static constexpr int kMaxObs = 160;  // Fixed storage capacity; never pad a 154-input actor.
  int observation_dim_{160};
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
    std::array<double, kJoints> joint_effort{};
    int64_t joint_stamp_ns{}, imu_stamp_ns{}, body_stamp_ns{}, anchor_stamp_ns{}, velocity_stamp_ns{};
    std::array<double, 3> body_position{};
    std::array<double, 4> body_orientation{};
    DanceBodyStateEstimator::Output estimate;
    bool estimator_valid{};
    Eigen::Vector3f imu_accel{Eigen::Vector3f::Zero()};
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
  void SetFault(const char* reason, const Snapshot* snapshot = nullptr,
                bool has_observation = false, bool has_action = false, const char* detail = nullptr);
  void AlignReference(const Snapshot& snapshot);
  float ClipTarget(int index, float target) const;
  void UpdateBodyEstimate();  // Caller holds estimator_mutex_, never sensor_mutex_.
  void StoreOdometry(const nav_msgs::msg::Odometry& odometry,
                     const DanceBodyStateEstimator::Output* estimate);
  DanceDiagnostics::Frame DiagnosticFrame(const Snapshot* snapshot, const char* event,
                                         const char* reason, bool has_observation, bool has_action) const;

  std::array<float, kJoints> default_pos_{};
  std::array<float, kJoints> preparation_pos_{};
  std::array<float, kJoints> kp_{};
  std::array<float, kJoints> kd_{};
  std::array<float, kJoints> effort_limit_{};
  std::array<float, kJoints> action_scale_{};
  std::array<float, kJoints> armature_{};  // Identified training dynamics; never an extra hardware torque.
  // q_policy = sign * (q_dcu - joint_offset); the inverse maps commands to DCU.
  std::array<float, kJoints> joint_sign_{};
  std::array<float, kMaxObs> mean_{};
  std::array<float, kMaxObs> inv_std_{};
  std::array<float, kMaxObs> observation_{};
  std::array<float, kMaxObs> raw_observation_{};
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
    DanceBodyStateEstimator::Output estimate;
    bool estimator_valid{};
  };
  static constexpr size_t kOdometryHistoryCapacity = 128;
  std::array<BodyOdometry, kOdometryHistoryCapacity> body_odometry_history_;
  size_t odometry_next_{};
  size_t odometry_count_{};
  const char* odometry_problem_{"no accepted body odometry; start the body state estimator"};
  bool use_odometry_{false};
  bool use_proprioception_{false};
  bool use_imu_only_{false};
  struct ImuSample {
    builtin_interfaces::msg::Time stamp;
    Eigen::Quaternionf orientation{Eigen::Quaternionf::Identity()};
    Eigen::Vector3f gyro{Eigen::Vector3f::Zero()}, acceleration{Eigen::Vector3f::Zero()};
    Clock::time_point received{};
  };
  static constexpr size_t kImuHistoryCapacity = 128;
  std::array<ImuSample, kImuHistoryCapacity> imu_history_;  // Protected by sensor_mutex_.
  size_t imu_next_{}, imu_count_{};
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
  float max_start_joint_speed_{0.5f};
  float max_start_angular_speed_{0.2f};
  float max_start_pose_error_{0.12f};
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
  DanceDiagnostics diagnostics_;
  DanceDiagnostics::Frame diagnostic_pending_frame_;
  bool diagnostic_pending_{false}, diagnostic_session_pending_{false};
  uint64_t diagnostic_session_{}, diagnostic_passive_tick_{};
  std::string diagnostic_state_{"idle"}, state_source_;
  Clock::time_point diagnostic_state_since_{};
};

}  // namespace xyber_x1_infer::rl_control_module
