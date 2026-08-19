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

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "control_module/controller_base.h"
#include "control_module/data_file_logger.h"
#include "control_module/rotation_tools.h"

namespace xyber_x1_infer::rl_control_module {

class RLController : public ControllerBase {
 public:
  explicit RLController(bool use_sim_handles);
  ~RLController() override = default;

  void Init(const YAML::Node& cfg_node) override;
  void RestartController() override;

  void Update() override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;

  void SetWalkLegEntered(bool entered);
  void SetLogExecutor(aimrt::executor::ExecutorRef executor);
  void StopLoggingWorker();

 private:
  void LoadModel();
  void UpdateStateEstimation();
  void ComputeObservation();
  void ComputeActions();
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
    double cycle_time;         // 固定步态周期（旧模型/固定周期模式）
    bool sw_mode;
    double cmd_threshold;
    bool adaptive_cycle{false};      // 自适应周期开关（对齐训练 exp1.2）
    double cycle_time_min{0.35};     // 静止/低速最短周期 [s]
    double cycle_time_max{0.7};      // cycle_speed_max 时的最长周期 [s]
    double cycle_speed_max{0.6};     // 平面指令速度上限 [m/s]
    double ema_tau{0.5};             // 速度 EMA 时间常数 [s]
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
  } onnx_conf_;

  struct LPFConf {
    double wc;
    double ts;
    std::set<std::string> paralle_list;
  } lpf_conf_;

  std::unique_ptr<Ort::Session> session_ptr_;
  Ort::MemoryInfo memory_info_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
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
  aimrt::executor::ExecutorRef log_executor_;
  std::atomic_bool log_worker_running_{false};
  std::atomic_bool log_worker_started_{false};
  std::atomic_bool log_worker_stopped_{true};

  DataFileLogger diag_logger_;
  bool diag_logging_enabled_{false};
  bool diag_logging_triggered_{false};
  bool diag_pending_frame_{false};
  std::atomic_bool diag_walk_entered_{false};
  int diag_log_count_{0};
  int diag_log_max_count_{0};
  std::string diag_log_dir_;
  std::atomic_size_t diag_dropped_count_{0};
  std::atomic<int64_t> diag_last_enqueue_ns_{0};

  DataFileLogger tm_logger_;
  bool tm_logging_enabled_{false};
  bool tm_logging_triggered_{false};
  std::atomic_bool tm_walk_entered_{false};
  int tm_log_count_{0};
  int tm_log_max_count_{0};
  std::string tm_log_dir_;
  std::atomic_size_t tm_dropped_count_{0};
  std::atomic<int64_t> tm_last_enqueue_ns_{0};

  struct DiagFrame {
    int64_t timestamp_ns{0};
    double phase_sin{0.0};
    double phase_cos{0.0};
    double cycle_time{0.0};
    double smoothed_speed{0.0};
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
  double policy_dt_{0.0};          // 推理周期 = decimation / 控制频率 [s]
  int64_t phase_step_count_{0};    // 相位累加步数（对齐训练 phase_length_buf）
  double smoothed_speed_{0.0};     // EMA 平滑后的平面指令速度 [m/s]
  double cur_cycle_time_{0.7};     // 当前生效的步态周期 [s]（日志用）
  double obs_cmd_linear_x_{0.0};
  double obs_cmd_linear_y_{0.0};
  double obs_cmd_angular_z_{0.0};
};

}  // namespace xyber_x1_infer::rl_control_module
