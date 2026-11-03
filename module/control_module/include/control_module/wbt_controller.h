#pragma once

#include <onnxruntime/onnxruntime_cxx_api.h>

#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "control_module/controller_base.h"
#include "control_module/rotation_tools.h"

namespace xyber_x1_infer::rl_control_module {

/// Whole-body motion tracking controller.
///
/// Consumes a BeyondMimic tracking policy exported by `export_motion_policy_as_onnx`
/// (inputs: obs, time_step; outputs: actions + reference motion streams).
class WBTController : public ControllerBase {
 public:
  explicit WBTController(bool use_sim_handles);
  ~WBTController() override = default;

  void Init(const YAML::Node& cfg_node) override;
  void RestartController() override;
  void SetBaseVelData(const geometry_msgs::msg::Twist base_vel_data) override;
  void SetBasePoseData(const geometry_msgs::msg::PoseStamped base_pose_data) override;

  void Update() override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;
  bool Finished() const override { return finished_; }

 private:
  void LoadModel();
  void UpdateStateEstimation();
  void ComputeActions();
  void ComputeObservation();

  struct OnnxConf {
    std::string policy_file;
    int actions_size;
    int observations_size;
    double observations_clip;
    double actions_clip;
    int inter_op_threads;
    int intra_op_threads;
  } onnx_conf_;

  struct MimicConf {
    double action_scale;
    int decimation;
    int anchor_index;
    bool loop;
    int reference_frames;
    std::string anchor_pos_mode;  // "frozen" | "ref" | "zero"
    std::vector<std::string> policy_joint_order;
    int num_ref_bodies;
  } mimic_conf_;

  struct LPFConf {
    double wc;
    double ts;
    std::set<std::string> paralle_list;
  } lpf_conf_;

  std::unique_ptr<Ort::Session> session_ptr_;
  Ort::MemoryInfo memory_info_;
  std::vector<std::string> input_names_owned_;
  std::vector<std::string> output_names_owned_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;

  //! policy index -> command index (joint_names_ order)
  std::vector<int> policy_to_cmd_;
  //! command index -> policy index
  std::vector<int> cmd_to_policy_;

  vector_t policy_default_;     // default joint pos in policy order
  vector_t obs_;                // observations_size
  vector_t ref_joint_pos_;      // actions_size (policy order)
  vector_t ref_joint_vel_;      // actions_size (policy order)
  std::vector<vector3_t> ref_body_pos_;
  std::vector<quaternion_t> ref_body_quat_;

  std::vector<float> actions_;  // actions_size, policy order
  vector_t last_actions_;       // policy order
  Eigen::Matrix<float, Eigen::Dynamic, 1> obs_buffer_;

  struct Proprioception {
    vector_t joint_pos;  // command order
    vector_t joint_vel;
    vector3_t base_ang_vel;
    quaternion_t imu_quat;
  } propri_;

  // motion frame alignment
  bool first_frame_{true};
  quaternion_t imu_quat0_;
  quaternion_t ref_quat0_;
  vector3_t ref_pos0_;
  vector3_t robot_pos_motion_;
  quaternion_t imu_quat_offset_;
  vector3_t base_pos0_;  // 进入 mimic 时的 base 世界位置（指标对齐基准）

  int64_t time_step_{0};
  int64_t loop_count_{0};

  // 仿真真值 base 线速度（IMU site 系），无该话题时保持为零
  mutable std::mutex base_vel_mutex_;
  vector3_t base_lin_vel_sim_;
  bool has_base_lin_vel_{false};

  // 仿真真值 base 位姿（用于跟踪指标记录），真机无该话题
  mutable std::mutex base_pose_mutex_;
  vector3_t base_pos_sim_;
  quaternion_t base_quat_sim_;
  bool has_base_pose_{false};
  bool wait_for_base_pose_{false};  // 收到 base 位姿后才开始推进策略

  // 跟踪指标 CSV
  std::ofstream metrics_file_;
  int metrics_row_{0};

  // 系统辨识用：保持进入 mimic 时的关节角（不跑策略）
  bool hold_pose_{false};
  vector_t hold_q_;
  // 系统辨识用：直接跟踪参考关节角（开环回放，不用策略输出）
  bool replay_reference_{false};

  // ONNX 参考流（joint_pos/joint_vel）的关节排列：Isaac 槽位 -> ONNX 数组下标
  std::vector<int> onnx_ref_order_;

  // 参考动作播完（time_step >= reference_frames）后置位，供 ControlModule 自动切回 stand
  bool finish_at_end_{false};
  bool finished_{false};

  std::vector<digital_lp_filter<double>> low_pass_filters_;
};

}  // namespace xyber_x1_infer::rl_control_module
