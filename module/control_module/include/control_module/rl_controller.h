#pragma once

#include <onnxruntime/onnxruntime_cxx_api.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "control_module/controller_base.h"
#include "control_module/data_file_logger.h"
#include "control_module/rotation_tools.h"
#include "control_module/dance_finish_core.h"
#include "control_module/dance_diagnostics.h"

namespace xyber_x1_infer::rl_control_module {

class RLController : public ControllerBase {
 public:
  explicit RLController(bool use_sim_handles);
  ~RLController() override;

  void Init(const YAML::Node& cfg_node) override;
  void RestartController() override;

  void Update() override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;
  void SetImuData(const sensor_msgs::msg::Imu imu_data) override;
  void SetJointStateData(const sensor_msgs::msg::JointState joint_state_data,
                       const std::unordered_map<std::string, int>& index) override;
  bool IsDanceFinisher() const { return static_cast<bool>(finish_core_); }
  bool FinishStandingConfirmed() const { return finish_core_ && finish_core_->StandingConfirmed(); }
  bool FinishRecovering() const { return finish_core_ &&
      (finish_core_->GetPhase() == DanceFinishCore::Phase::kBrake ||
       finish_core_->GetPhase() == DanceFinishCore::Phase::kLand); }
  void PrepareDanceHandoff(const std::vector<double>& targets);
  void StopDanceFinishDiagnostics() { finish_diagnostics_.Stop(); }

  void SetLoggingActive(bool active);
  void StopLoggingWorker();
  bool IsAmpMode() const { return amp_mode_; }
  bool AmpReady() const;
  bool AmpFaulted() const { return amp_fault_.load(std::memory_order_acquire); }

 private:
  void LoadModel();
  void UpdateStateEstimation();
  void ComputeObservation();
  void ComputeActions();
  void UpdateDanceFinish();
  void LogDanceFinish(const my_ros2_proto::msg::JointCommand& command);
  void StartLoggingWorker();
  void LoggingWorkerLoop();
  void DrainDiagBuffer();
  void DrainTmBuffer();
  void FinalizeDiagLogging(const char* reason);
  void FinalizeTmLogging(const char* reason);
  bool EnqueueDiagFrame();
  bool EnqueueTmFrame();

 private:
  struct WalkStepConf {
    double action_scale;
    int decimation;
    double cycle_time;
    bool sw_mode;
    double cmd_threshold;
  } walk_step_conf_;

  struct ObsScales {
    double lin_vel;
    double ang_vel;
    double dof_pos;
    double dof_vel;
    double quat;
  } obs_scales_;

  struct OnnxConf {
    std::string policy_file;
    int actions_size;
    int observations_size;
    int num_hist;
    double observations_clip;
    double actions_clip;
    int inter_op_threads;   // 算子间并行线程数（顺序 MLP 用 1）
    int intra_op_threads;  // 算子内并行线程数（实时控制用 1，避免 fork worker 到隔离核外）
  } onnx_conf_;

  struct LPFConf {
    double wc;
    double ts;
    bool position_enabled;
    bool effort_enabled;
    std::set<std::string> paralle_list;
  } lpf_conf_;

  // AMP 全身策略部署适配（x1_29_AMP / soup59c_50 契约）：
  //   obs 每帧 96 维 = [ang_vel(3), projected_gravity(3), cmd(3),
  //                     joint_pos_rel(n), joint_vel(n), last_action(n)]，n=29
  //   历史按 term-major 展开（每项 num_hist 帧 oldest-first 拼接），
  //   归一化器融合在 ONNX 图内，喂原始值（不走 obs_scales）。
  struct AmpConf {
    int settle_ticks;      // 软启动拍数（策略拍，50Hz 下 50 拍 = 1.0s）
    double cmd_x_min;      // 指令限幅（训练范围）
    double cmd_x_max;
    double cmd_y_min;
    double cmd_y_max;
    double cmd_yaw_min;
    double cmd_yaw_max;
    bool use_constant_cmd{false};  // 验收复刻模式: 恒定指令, 忽略外部 /cmd_vel_limiter
    double const_cmd_x{0.0};
    double const_cmd_y{0.0};
    double const_cmd_z{0.0};
    int sensor_timeout_ms;
    int cmd_timeout_ms;
    double max_tilt_deg;
    double ankle_effort_limit;
  } amp_conf_;
  bool amp_mode_{false};
  int amp_settle_remaining_{0};
  int amp_transition_elapsed_{0};
  vector_t amp_transition_start_pos_;
  std::atomic_bool amp_fault_{false};
  std::unique_ptr<DanceFinishCore> finish_core_;
  DanceFinishCore::Joints finish_start_targets_{};
  bool finish_pending_{false};
  std::mutex finish_feedback_mutex_;
  int64_t finish_joint_source_ns_{}, finish_imu_source_ns_{};  // feedback mutex
  bool finish_effort_valid_{false};
  int64_t finish_joint_snapshot_ns_{}, finish_imu_snapshot_ns_{};  // control thread
  bool finish_effort_snapshot_valid_{false};
  DanceDiagnostics finish_diagnostics_;
  uint64_t finish_tick_{}, finish_session_{};
  DanceFinishCore::Phase finish_logged_phase_{DanceFinishCore::Phase::kInactive};
  bool finish_logged_confirmed_{};

  std::unique_ptr<Ort::Session> session_ptr_;
  Ort::MemoryInfo memory_info_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
  std::vector<std::string> input_name_storage_;
  std::vector<std::string> output_name_storage_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;

  std::vector<float> actions_;
  std::vector<float> observations_;
  vector_t last_actions_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> propri_history_buffer_;
  struct Proprioception {
    vector_t joint_pos;
    vector_t joint_vel;
    vector_t joint_effort;
    vector3_t base_ang_vel;
    vector3_t base_euler_xyz;
    vector3_t projected_gravity;
    quaternion_t imu_quat;
    vector3_t imu_accel;
  } propri_;

  int64_t loop_count_{0};
  std::vector<digital_lp_filter<double>> low_pass_filters_;
  std::atomic_bool is_first_frame_{true};
  std::thread log_worker_thread_;
  std::atomic_bool log_worker_running_{false};
  std::atomic_bool log_worker_started_{false};

  DataFileLogger diag_logger_;
  bool diag_logging_enabled_{false};
  bool diag_logging_triggered_{false};
  bool diag_pending_frame_{false};
  std::atomic_bool diag_logging_requested_{false};
  int diag_log_count_{0};
  int diag_log_max_count_{0};
  std::string diag_log_dir_;
  std::vector<std::string> diag_file_actuator_names_;  // 开文件时冻结的电机列名
  std::atomic_size_t diag_dropped_count_{0};
  std::atomic<int64_t> diag_last_enqueue_ns_{0};

  DataFileLogger tm_logger_;
  bool tm_logging_enabled_{false};
  bool tm_logging_triggered_{false};
  std::atomic_bool tm_logging_requested_{false};
  int tm_log_count_{0};
  int tm_log_max_count_{0};
  std::string tm_log_dir_;
  std::atomic_size_t tm_dropped_count_{0};
  std::atomic<int64_t> tm_last_enqueue_ns_{0};

  struct DiagFrame {
    int64_t timestamp_ns{0};
    double phase_sin{0.0};
    double phase_cos{0.0};
    double cmd_linear_x{0.0};
    double cmd_linear_y{0.0};
    double cmd_angular_z{0.0};
    double base_euler_x{0.0};
    double base_euler_y{0.0};
    double base_euler_z{0.0};
    double base_ang_vel_x{0.0};
    double base_ang_vel_y{0.0};
    double base_ang_vel_z{0.0};
    double imu_quat_w{0.0};
    double imu_quat_x{0.0};
    double imu_quat_y{0.0};
    double imu_quat_z{0.0};
    double imu_gyro_x{0.0};
    double imu_gyro_y{0.0};
    double imu_gyro_z{0.0};
    double imu_accel_x{0.0};
    double imu_accel_y{0.0};
    double imu_accel_z{0.0};
    int clip_count{0};
    std::vector<float> actions;
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    std::vector<double> joint_effort;
    std::vector<double> pos_des_raw;
    std::vector<double> pos_des_lpf;
    std::vector<double> tau_des_raw;
    std::vector<double> tau_des_lpf;
    std::vector<int> is_parallel;
    // 电机侧力矩（来自 /actuator_cmd、/actuator_states，需 dcu actuator_debug=true）
    std::vector<std::string> actuator_names;
    std::vector<double> actuator_cmd_effort;
    std::vector<double> actuator_state_effort;
  };

  struct TmFrame {
    std::vector<float> observations;
  };

  std::vector<DiagFrame> diag_ring_;
  std::vector<TmFrame> tm_ring_;
  std::atomic<size_t> diag_write_idx_{0};
  std::atomic<size_t> diag_read_idx_{0};
  std::atomic<size_t> tm_write_idx_{0};
  std::atomic<size_t> tm_read_idx_{0};

  std::vector<double> pd_pos_des_raw_;
  std::vector<double> pd_pos_des_lpf_;
  std::vector<double> pd_tau_des_raw_;
  std::vector<double> pd_tau_des_lpf_;
  std::vector<int> pd_is_parallel_;

  double obs_phase_sin_{0.0};
  double obs_phase_cos_{1.0};
  double obs_cmd_linear_x_{0.0};
  double obs_cmd_linear_y_{0.0};
  double obs_cmd_angular_z_{0.0};
};

}  // namespace xyber_x1_infer::rl_control_module
