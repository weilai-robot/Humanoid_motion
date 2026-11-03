#include "control_module/wbt_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>

#include "control_module/global.h"

namespace xyber_x1_infer::rl_control_module {

WBTController::WBTController(bool use_sim_handles)
    : ControllerBase(use_sim_handles),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {}

void WBTController::Init(const YAML::Node& cfg_node) {
  joint_names_ = cfg_node["joint_list"].as<std::vector<std::string>>();
  const size_t num_joints = joint_names_.size();
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(num_joints, 0.0);
  joint_state_data_.velocity.resize(num_joints, 0.0);
  joint_state_data_.effort.resize(num_joints, 0.0);

  joint_conf_.init_state = Eigen::Map<vector_t>(
      cfg_node["init_state"].as<std::vector<double>>().data(),
      cfg_node["init_state"].as<std::vector<double>>().size());
  joint_conf_.stiffness = Eigen::Map<vector_t>(
      cfg_node["stiffness"].as<std::vector<double>>().data(),
      cfg_node["stiffness"].as<std::vector<double>>().size());
  joint_conf_.damping = Eigen::Map<vector_t>(
      cfg_node["damping"].as<std::vector<double>>().data(),
      cfg_node["damping"].as<std::vector<double>>().size());

  {
    const auto& limits_node = cfg_node["joint_limits"];
    AIMRT_CHECK_ERROR_THROW(limits_node, "Missing joint_limits for WBTController.");
    joint_conf_.pos_limit_lower.resize(num_joints);
    joint_conf_.pos_limit_upper.resize(num_joints);
    for (size_t ii = 0; ii < num_joints; ++ii) {
      const std::string& name = joint_names_[ii];
      if (limits_node[name]) {
        joint_conf_.pos_limit_lower(ii) = limits_node[name]["lower"].as<double>();
        joint_conf_.pos_limit_upper(ii) = limits_node[name]["upper"].as<double>();
      } else {
        joint_conf_.pos_limit_lower(ii) = -std::numeric_limits<double>::infinity();
        joint_conf_.pos_limit_upper(ii) = std::numeric_limits<double>::infinity();
      }
    }
  }

  mimic_conf_.action_scale = cfg_node["action_scale"].as<double>();
  mimic_conf_.decimation = cfg_node["decimation"].as<int32_t>();
  mimic_conf_.anchor_index = cfg_node["anchor_index"].as<int32_t>();
  mimic_conf_.loop = cfg_node["loop"] ? cfg_node["loop"].as<bool>() : false;
  mimic_conf_.reference_frames = cfg_node["reference_frames"]
                                     ? cfg_node["reference_frames"].as<int32_t>()
                                     : 0;
  mimic_conf_.anchor_pos_mode =
      cfg_node["anchor_pos_mode"] ? cfg_node["anchor_pos_mode"].as<std::string>() : "frozen";
  mimic_conf_.policy_joint_order =
      cfg_node["policy_joint_order"].as<std::vector<std::string>>();
  AIMRT_CHECK_ERROR_THROW(mimic_conf_.policy_joint_order.size() == num_joints,
                         "policy_joint_order size must match joint_list size.");

  policy_to_cmd_.resize(num_joints);
  cmd_to_policy_.resize(num_joints);
  for (size_t pi = 0; pi < num_joints; ++pi) {
    const auto& name = mimic_conf_.policy_joint_order[pi];
    auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
    AIMRT_CHECK_ERROR_THROW(it != joint_names_.end(),
                            "Policy joint not found in joint_list: {}", name);
    const int ci = static_cast<int>(std::distance(joint_names_.begin(), it));
    policy_to_cmd_[pi] = ci;
    cmd_to_policy_[ci] = static_cast<int>(pi);
  }
  policy_default_.resize(num_joints);
  for (size_t pi = 0; pi < num_joints; ++pi) {
    policy_default_(pi) = joint_conf_.init_state(policy_to_cmd_[pi]);
  }

  onnx_conf_.policy_file = cfg_node["onnx_conf"]["policy_file"].as<std::string>();
  onnx_conf_.actions_size = cfg_node["onnx_conf"]["actions_size"].as<int32_t>();
  onnx_conf_.observations_size = cfg_node["onnx_conf"]["observations_size"].as<int32_t>();
  onnx_conf_.observations_clip = cfg_node["onnx_conf"]["observations_clip"].as<double>();
  onnx_conf_.actions_clip = cfg_node["onnx_conf"]["actions_clip"].as<double>();
  onnx_conf_.inter_op_threads = cfg_node["onnx_conf"]["inter_op_threads"]
                                   ? cfg_node["onnx_conf"]["inter_op_threads"].as<int32_t>()
                                   : 1;
  onnx_conf_.intra_op_threads = cfg_node["onnx_conf"]["intra_op_threads"]
                                   ? cfg_node["onnx_conf"]["intra_op_threads"].as<int32_t>()
                                   : 1;
  AIMRT_CHECK_ERROR_THROW(onnx_conf_.actions_size == static_cast<int>(num_joints),
                         "actions_size must match joint_list size.");

  lpf_conf_.wc = cfg_node["lpf_conf"]["wc"].as<double>();
  lpf_conf_.ts = cfg_node["lpf_conf"]["ts"].as<double>();
  auto paralle_list = cfg_node["lpf_conf"]["paralle_list"].as<std::vector<std::string>>();
  lpf_conf_.paralle_list = std::set<std::string>(paralle_list.begin(), paralle_list.end());

  LoadModel();

  const int64_t bodies_per_frame =
      output_shapes_[3].size() >= 2 ? output_shapes_[3][1] : 1;
  mimic_conf_.num_ref_bodies = static_cast<int>(bodies_per_frame);
  AIMRT_CHECK_ERROR_THROW(
      mimic_conf_.anchor_index >= 0 && mimic_conf_.anchor_index < mimic_conf_.num_ref_bodies,
      "anchor_index out of range, num_ref_bodies={}", mimic_conf_.num_ref_bodies);

  propri_.joint_pos.resize(num_joints);
  propri_.joint_vel.resize(num_joints);
  actions_.resize(onnx_conf_.actions_size);
  obs_.resize(onnx_conf_.observations_size);
  obs_buffer_.resize(onnx_conf_.observations_size);
  // 首次推理前 obs 必须已初始化，否则会用未初始化内存推理得到发散动作
  obs_.setZero();
  obs_buffer_.setZero();
  actions_.assign(onnx_conf_.actions_size, 0.0f);
  last_actions_.resize(onnx_conf_.actions_size);
  last_actions_.setZero();
  ref_joint_pos_.resize(onnx_conf_.actions_size);
  ref_joint_vel_.resize(onnx_conf_.actions_size);
  ref_joint_pos_.setZero();
  ref_joint_vel_.setZero();
  ref_body_pos_.resize(mimic_conf_.num_ref_bodies);
  ref_body_quat_.resize(mimic_conf_.num_ref_bodies);

  low_pass_filters_.clear();
  for (int i = 0; i < onnx_conf_.actions_size; ++i) {
    low_pass_filters_.emplace_back(lpf_conf_.wc, lpf_conf_.ts);
  }
  first_frame_ = true;
  time_step_ = 0;
  loop_count_ = 0;
  has_base_pose_ = false;

  // 跟踪指标 CSV（每个策略步一行，用于和 Isaac 回放指标对比；默认关闭）
  const bool log_metrics = cfg_node["log_metrics"] ? cfg_node["log_metrics"].as<bool>() : false;
  wait_for_base_pose_ = cfg_node["wait_for_base_pose"] ? cfg_node["wait_for_base_pose"].as<bool>() : false;
  hold_pose_ = cfg_node["hold_pose"] ? cfg_node["hold_pose"].as<bool>() : false;
  replay_reference_ = cfg_node["replay_reference"] ? cfg_node["replay_reference"].as<bool>() : false;
  finish_at_end_ = cfg_node["finish_at_end"] ? cfg_node["finish_at_end"].as<bool>() : false;
  finished_ = false;
  hold_q_.resize(num_joints);
  hold_q_.setZero();
  // ONNX 参考流重排（默认恒等）
  onnx_ref_order_.assign(onnx_conf_.actions_size, 0);
  for (int i = 0; i < onnx_conf_.actions_size; ++i) {
    onnx_ref_order_[i] = i;
  }
  if (cfg_node["onnx_ref_order"]) {
    auto order = cfg_node["onnx_ref_order"].as<std::vector<int>>();
    if (static_cast<int>(order.size()) == onnx_conf_.actions_size) {
      onnx_ref_order_ = order;
    } else {
      AIMRT_WARN("onnx_ref_order size {} != actions_size {}, ignored", order.size(),
                 onnx_conf_.actions_size);
    }
  }
  if (log_metrics) {
    std::error_code fs_ec;
    std::filesystem::create_directories("test_logs/data_csv", fs_ec);
    std::time_t now = std::time(nullptr);
    std::tm tm_val{};
    localtime_r(&now, &tm_val);
    char path[96];
    std::strftime(path, sizeof(path), "test_logs/data_csv/wbt_metrics_%Y%m%d_%H%M%S.csv", &tm_val);
    metrics_file_.open(path, std::ios::out | std::ios::trunc);
    if (metrics_file_.is_open()) {
      metrics_file_ << std::fixed << std::setprecision(6);
      metrics_file_ << "step,base_x,base_y,base_z,error_anchor_pos,error_anchor_rot,"
                       "error_joint_pos,error_joint_vel,action_abs_mean\n";
      AIMRT_INFO("WBT metrics log: {}", path);
    } else {
      AIMRT_WARN("WBT metrics log open failed: {}", path);
    }
    metrics_row_ = 0;
  }
}

void WBTController::RestartController() {
  first_frame_ = true;
  time_step_ = 0;
  loop_count_ = 0;
  finished_ = false;
  last_actions_.setZero();
}

void WBTController::SetBaseVelData(const geometry_msgs::msg::Twist base_vel_data) {
  std::lock_guard<std::mutex> lock(base_vel_mutex_);
  base_lin_vel_sim_ << base_vel_data.linear.x, base_vel_data.linear.y, base_vel_data.linear.z;
  has_base_lin_vel_ = true;
}

void WBTController::SetBasePoseData(const geometry_msgs::msg::PoseStamped base_pose_data) {
  std::lock_guard<std::mutex> lock(base_pose_mutex_);
  const auto& p = base_pose_data.pose.position;
  const auto& o = base_pose_data.pose.orientation;
  base_pos_sim_ << p.x, p.y, p.z;
  base_quat_sim_ = quaternion_t(o.w, o.x, o.y, o.z);
  base_quat_sim_.normalize();
  if (!has_base_pose_) {
    AIMRT_INFO("First base pose received: ({:.3f},{:.3f},{:.3f})", p.x, p.y, p.z);
  }
  has_base_pose_ = true;
}

void WBTController::LoadModel() {
  std::shared_ptr<Ort::Env> onnx_env(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "WBTOnnxController"));
  Ort::SessionOptions session_options;
  session_options.SetInterOpNumThreads(onnx_conf_.inter_op_threads);
  session_options.SetIntraOpNumThreads(onnx_conf_.intra_op_threads);
  session_options.AddConfigEntry("session.intra_op.allow_spinning", "0");
  session_ptr_ =
      std::make_unique<Ort::Session>(*onnx_env, onnx_conf_.policy_file.c_str(), session_options);

  input_names_owned_.clear();
  output_names_owned_.clear();
  input_names_.clear();
  output_names_.clear();
  input_shapes_.clear();
  output_shapes_.clear();

  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t ii = 0; ii < session_ptr_->GetInputCount(); ++ii) {
    input_names_owned_.emplace_back(session_ptr_->GetInputNameAllocated(ii, allocator).get());
    input_shapes_.push_back(
        session_ptr_->GetInputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (size_t ii = 0; ii < session_ptr_->GetOutputCount(); ++ii) {
    output_names_owned_.emplace_back(session_ptr_->GetOutputNameAllocated(ii, allocator).get());
    output_shapes_.push_back(
        session_ptr_->GetOutputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (const auto& name : input_names_owned_) {
    input_names_.push_back(name.c_str());
  }
  for (const auto& name : output_names_owned_) {
    output_names_.push_back(name.c_str());
  }
  AIMRT_CHECK_ERROR_THROW(input_names_.size() == 2 && output_names_.size() >= 7,
                         "WBT ONNX must have 2 inputs (obs, time_step) and 7 outputs.");
}

void WBTController::UpdateStateEstimation() {
  {
    std::shared_lock<std::shared_mutex> lock(joint_state_mutex_);
    for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
      propri_.joint_pos(ii) = joint_state_data_.position[ii];
      propri_.joint_vel(ii) = joint_state_data_.velocity[ii];
    }
  }
  {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    propri_.base_ang_vel(0) = imu_data_.angular_velocity.x;
    propri_.base_ang_vel(1) = imu_data_.angular_velocity.y;
    propri_.base_ang_vel(2) = imu_data_.angular_velocity.z;
    propri_.imu_quat = quaternion_t(imu_data_.orientation.w, imu_data_.orientation.x,
                                   imu_data_.orientation.y, imu_data_.orientation.z);
    propri_.imu_quat.normalize();
  }
}

void WBTController::ComputeActions() {
  int64_t time_step_value = time_step_;
  std::vector<Ort::Value> inputs;
  inputs.push_back(Ort::Value::CreateTensor<float>(memory_info_, obs_buffer_.data(),
                                                   obs_buffer_.size(), input_shapes_[0].data(),
                                                   input_shapes_[0].size()));
  inputs.push_back(Ort::Value::CreateTensor<int64_t>(memory_info_, &time_step_value, 1,
                                                     input_shapes_[1].data(),
                                                     input_shapes_[1].size()));

  std::vector<Ort::Value> outputs =
      session_ptr_->Run(Ort::RunOptions{}, input_names_.data(), inputs.data(), inputs.size(),
                        output_names_.data(), output_names_.size());

  const float* actions_ptr = outputs[0].GetTensorMutableData<float>();
  const float* joint_pos_ptr = outputs[1].GetTensorMutableData<float>();
  const float* joint_vel_ptr = outputs[2].GetTensorMutableData<float>();
  const float* body_pos_ptr = outputs[3].GetTensorMutableData<float>();
  const float* body_quat_ptr = outputs[4].GetTensorMutableData<float>();

  for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
    ref_joint_pos_(ii) = joint_pos_ptr[onnx_ref_order_[ii]];
    ref_joint_vel_(ii) = joint_vel_ptr[onnx_ref_order_[ii]];
    actions_[ii] = actions_ptr[ii];
  }
  for (int bi = 0; bi < mimic_conf_.num_ref_bodies; ++bi) {
    ref_body_pos_[bi] << body_pos_ptr[bi * 3 + 0], body_pos_ptr[bi * 3 + 1],
        body_pos_ptr[bi * 3 + 2];
    // Isaac Lab stores quaternions as wxyz
    ref_body_quat_[bi] = quaternion_t(body_quat_ptr[bi * 4 + 0], body_quat_ptr[bi * 4 + 1],
                                      body_quat_ptr[bi * 4 + 2], body_quat_ptr[bi * 4 + 3]);
    ref_body_quat_[bi].normalize();
  }

  if (first_frame_) {
    ref_quat0_ = ref_body_quat_[mimic_conf_.anchor_index];
    ref_pos0_ = ref_body_pos_[mimic_conf_.anchor_index];
    imu_quat0_ = propri_.imu_quat;
    imu_quat_offset_ = ref_quat0_ * imu_quat0_.inverse();
    robot_pos_motion_ = ref_pos0_;
    {
      std::lock_guard<std::mutex> lock(base_pose_mutex_);
      base_pos0_ = has_base_pose_ ? base_pos_sim_ : ref_pos0_;
    }
    if (hold_pose_) {
      hold_q_ = propri_.joint_pos;
    }
    for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
      if (lpf_conf_.paralle_list.find(joint_names_[ii]) == lpf_conf_.paralle_list.end()) {
        low_pass_filters_[ii].init(propri_.joint_pos[ii]);
      } else {
        low_pass_filters_[ii].init(0);
      }
    }
    first_frame_ = false;
  }
}

void WBTController::ComputeObservation() {
  const quaternion_t base_quat_motion = imu_quat_offset_ * propri_.imu_quat;
  Eigen::Matrix3d rot_base = base_quat_motion.toRotationMatrix();

  vector3_t robot_pos = robot_pos_motion_;
  if (mimic_conf_.anchor_pos_mode == "ref") {
    robot_pos = ref_body_pos_[mimic_conf_.anchor_index];
  } else if (mimic_conf_.anchor_pos_mode == "zero") {
    robot_pos = vector3_t::Zero();
  } else if (mimic_conf_.anchor_pos_mode == "sim") {
    // 用仿真真值：把机器人位置映射到参考动作的初始位姿基准，再与参考 anchor 比较
    std::lock_guard<std::mutex> lock(base_pose_mutex_);
    if (has_base_pose_) {
      robot_pos = ref_pos0_ + imu_quat_offset_.toRotationMatrix() * (base_pos_sim_ - base_pos0_);
    }
  }

  vector3_t anchor_pos_b =
      rot_base.transpose() * (ref_body_pos_[mimic_conf_.anchor_index] - robot_pos);
  Eigen::Matrix3d rot_ref = ref_body_quat_[mimic_conf_.anchor_index].toRotationMatrix();
  Eigen::Matrix3d rot_rel = rot_base.transpose() * rot_ref;

  int k = 0;
  // command: reference joint pos then reference joint vel (policy order)
  for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
    obs_(k++) = ref_joint_pos_(ii);
  }
  for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
    obs_(k++) = ref_joint_vel_(ii);
  }
  // motion anchor position in base frame
  obs_(k++) = anchor_pos_b(0);
  obs_(k++) = anchor_pos_b(1);
  obs_(k++) = anchor_pos_b(2);
  // motion anchor orientation: first two columns of the relative rotation (row-major)
  obs_(k++) = rot_rel(0, 0);
  obs_(k++) = rot_rel(0, 1);
  obs_(k++) = rot_rel(1, 0);
  obs_(k++) = rot_rel(1, 1);
  obs_(k++) = rot_rel(2, 0);
  obs_(k++) = rot_rel(2, 1);
  // base linear velocity: 优先用仿真真值（/mujoco/base_vel），无该话题时置零
  {
    vector3_t lin_vel = vector3_t::Zero();
    {
      std::lock_guard<std::mutex> lock(base_vel_mutex_);
      if (has_base_lin_vel_) lin_vel = base_lin_vel_sim_;
    }
    obs_(k++) = lin_vel(0);
    obs_(k++) = lin_vel(1);
    obs_(k++) = lin_vel(2);
  }
  // base angular velocity
  obs_(k++) = propri_.base_ang_vel(0);
  obs_(k++) = propri_.base_ang_vel(1);
  obs_(k++) = propri_.base_ang_vel(2);
  // joint positions relative to the training default pose (policy order)
  for (int pi = 0; pi < onnx_conf_.actions_size; ++pi) {
    obs_(k++) = propri_.joint_pos(policy_to_cmd_[pi]) - policy_default_(pi);
  }
  // joint velocities (policy order)
  for (int pi = 0; pi < onnx_conf_.actions_size; ++pi) {
    obs_(k++) = propri_.joint_vel(policy_to_cmd_[pi]);
  }
  // last actions (policy order)
  for (int pi = 0; pi < onnx_conf_.actions_size; ++pi) {
    obs_(k++) = last_actions_(pi);
  }
  AIMRT_CHECK_ERROR_THROW(k == onnx_conf_.observations_size,
                          "Observation size mismatch: built {} vs configured {}", k,
                          onnx_conf_.observations_size);

  const double clip = onnx_conf_.observations_clip;
  obs_ = obs_.cwiseMax(-clip).cwiseMin(clip);
  for (int ii = 0; ii < obs_.size(); ++ii) {
    obs_buffer_(ii) = static_cast<float>(obs_(ii));
  }
}

void WBTController::Update() {
  if (finished_) {
    return;
  }
  UpdateStateEstimation();
  if (wait_for_base_pose_) {
    std::lock_guard<std::mutex> lock(base_pose_mutex_);
    if (!has_base_pose_) {
      return;  // 仿真尚未就绪，保持 time_step=0，避免参考动作空转
    }
  }
  if (loop_count_ % mimic_conf_.decimation == 0) {
    // Reference outputs for the current time step, then the observation that consumes them.
    ComputeActions();
    if (replay_reference_) {
      // 开环回放：令 pos_des 等于参考关节角
      for (int pi = 0; pi < onnx_conf_.actions_size; ++pi) {
        actions_[pi] = static_cast<float>(
            (ref_joint_pos_(pi) - policy_default_(pi)) / mimic_conf_.action_scale);
      }
    }
    ComputeObservation();
    for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
      last_actions_(ii) = std::max(-onnx_conf_.actions_clip,
                                   std::min(onnx_conf_.actions_clip,
                                            static_cast<double>(actions_[ii])));
    }

    // 记录跟踪指标（列与 Isaac 回放 CSV 对齐）
    if (metrics_file_.is_open()) {
      vector3_t base_pos = vector3_t::Zero();
      quaternion_t base_quat = propri_.imu_quat;
      bool has_pose = false;
      {
        std::lock_guard<std::mutex> lock(base_pose_mutex_);
        has_pose = has_base_pose_;
        if (has_pose) {
          base_pos = base_pos_sim_;
          base_quat = base_quat_sim_;
        }
      }
      double e_ap = std::nan(""), e_ar = std::nan("");
      double bx = std::nan(""), by = std::nan(""), bz = std::nan("");
      if (has_pose) {
        const int ai = mimic_conf_.anchor_index;
        const vector3_t pos_motion =
            ref_pos0_ + imu_quat_offset_.toRotationMatrix() * (base_pos - base_pos0_);
        bx = base_pos(0);
        by = base_pos(1);
        bz = base_pos(2);
        e_ap = (ref_body_pos_[ai] - pos_motion).norm();
        const quaternion_t q_err =
            (imu_quat_offset_ * base_quat).conjugate() * ref_body_quat_[ai];
        e_ar = 2.0 * std::acos(std::min(1.0, std::abs(q_err.w())));
      }
      double e_jp = 0.0, e_jv = 0.0, act_mean = 0.0;
      for (int pi = 0; pi < onnx_conf_.actions_size; ++pi) {
        e_jp += std::abs(propri_.joint_pos(policy_to_cmd_[pi]) - ref_joint_pos_(pi));
        e_jv += std::abs(propri_.joint_vel(policy_to_cmd_[pi]) - ref_joint_vel_(pi));
        act_mean += std::abs(static_cast<double>(actions_[pi]));
      }
      const double n = static_cast<double>(onnx_conf_.actions_size);
      e_jp /= n;
      e_jv /= n;
      act_mean /= n;
      metrics_file_ << metrics_row_ << ',' << bx << ',' << by << ',' << bz << ',' << e_ap << ','
                    << e_ar << ',' << e_jp << ',' << e_jv << ',' << act_mean << '\n';
      ++metrics_row_;
      if (metrics_row_ % 50 == 0) {
        metrics_file_.flush();
      }
    }

    time_step_++;
    if (mimic_conf_.loop && mimic_conf_.reference_frames > 0 &&
        time_step_ >= mimic_conf_.reference_frames) {
      time_step_ = 0;
    }
    if (finish_at_end_ && mimic_conf_.reference_frames > 0 &&
        time_step_ >= mimic_conf_.reference_frames) {
      finished_ = true;
      AIMRT_INFO("WBT reference finished after {} steps", mimic_conf_.reference_frames);
    }
  }
  loop_count_++;
}

my_ros2_proto::msg::JointCommand WBTController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.assign(joint_names_.size(), 0.0);
  joint_cmd.velocity.assign(joint_names_.size(), 0.0);
  joint_cmd.effort.assign(joint_names_.size(), 0.0);
  joint_cmd.damping.assign(joint_names_.size(), 0.0);
  joint_cmd.stiffness.assign(joint_names_.size(), 0.0);

  // 等待仿真就绪时输出零力矩：避免用未初始化的状态把机器人从参考姿态拽走
  if (wait_for_base_pose_) {
    std::lock_guard<std::mutex> lock(base_pose_mutex_);
    if (!has_base_pose_) {
      return joint_cmd;
    }
  }

  for (size_t ci = 0; ci < joint_names_.size(); ++ci) {
    const int pi = cmd_to_policy_[ci];
    double action = actions_[pi];
    action = std::max(-onnx_conf_.actions_clip, std::min(onnx_conf_.actions_clip, action));
    const double pos_des =
        hold_pose_ ? hold_q_(ci) : action * mimic_conf_.action_scale + joint_conf_.init_state(ci);
    const double stiffness = joint_conf_.stiffness(ci);
    const double damping = joint_conf_.damping(ci);

    if (lpf_conf_.paralle_list.find(joint_names_[ci]) == lpf_conf_.paralle_list.end()) {
      low_pass_filters_[ci].input(pos_des);
      joint_cmd.position[ci] = low_pass_filters_[ci].output();
      joint_cmd.stiffness[ci] = stiffness;
      joint_cmd.damping[ci] = damping;
    } else {
      const double tau_des = stiffness * (pos_des - propri_.joint_pos[ci]) +
                             damping * (0.0 - propri_.joint_vel[ci]);
      low_pass_filters_[ci].input(tau_des);
      joint_cmd.effort[ci] = low_pass_filters_[ci].output();
    }
  }
  return joint_cmd;
}

}  // namespace xyber_x1_infer::rl_control_module
