#pragma once
#include <memory>
// #include <sstream>

// #include "control_module/rotation_tools.h"
#include "control_module/controller_base.h"

namespace xyber_x1_infer::rl_control_module {

class PDController : public ControllerBase {
 public:
  PDController(const bool use_sim_handles);
  ~PDController() = default;

  void Init(const YAML::Node &cfg_node) override;
  void RestartController() override;

  void Update() override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;

 private:
  // trans mode
  double trans_mode_percent_ = 0.0;     // 0 ~ 1
  double trans_mode_duration_s_ = 2.0;  // hard code
  std::vector<double> start_joint_angles_;

  // keep controller config
  bool is_keep_controller_ = false;

  // plan controller config
  bool is_plan_controller_ = false;
  Interpolator trajectory_generator_;
  std::vector<std::vector<double>> to_interpolate_data_;
  std::vector<double> generated_joint_angles_;
};

}  // namespace xyber_x1_infer::rl_control_module
