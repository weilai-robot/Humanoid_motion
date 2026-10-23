#include "control_module/rl_controller.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <filesystem>
#include <limits>
#include <sstream>
#include <string_view>

#include "control_module/global.h"

namespace xyber_x1_infer::rl_control_module {

namespace {

constexpr size_t kDiagRingCapacity = 256;
constexpr size_t kTmRingCapacity = 512;
constexpr auto kLogWorkerPollInterval = std::chrono::milliseconds(1);
constexpr auto kLogIdleTimeout = std::chrono::milliseconds(500);
constexpr auto kLogFlushInterval = std::chrono::milliseconds(200);

int64_t FinishStampNs(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) return 0;
  return int64_t(stamp.sec)*1000000000LL+stamp.nanosec;
}
bool FinishStampFresh(int64_t source, bool simulation) {
  if (source <= 0) return false;
  if (simulation) return true;
  const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return now-source >= -20000000LL && now-source <= 80000000LL;
}

bool IsLegActuator(const std::string& name) {
  return name.find("_hip_") != std::string::npos ||
         name.find("_knee_") != std::string::npos ||
         name.find("_ankle_") != std::string::npos;
}

// 踝并联电机：软件下发 effort；髋/膝串联：下发位置，目标力矩记 0
bool IsParallelLegActuator(const std::string& name) {
  return name.find("_ankle_") != std::string::npos;
}

std::string MakeTimestampString() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now{};
#ifdef _WIN32
  localtime_s(&tm_now, &time_t_now);
#else
  localtime_r(&time_t_now, &tm_now);
#endif
  char time_buf[64];
  std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", &tm_now);
  return time_buf;
}

}  // namespace

RLController::RLController(bool use_sim_handles)
    : ControllerBase(use_sim_handles),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {}

RLController::~RLController() {
  finish_diagnostics_.Stop();
  StopLoggingWorker();
}

void RLController::Init(const YAML::Node& cfg_node) {
  joint_names_ = cfg_node["joint_list"].as<std::vector<std::string>>();
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);

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
    AIMRT_CHECK_ERROR_THROW(limits_node, "Missing joint_limits for RLController.");
    const size_t n = joint_names_.size();
    joint_conf_.pos_limit_lower.resize(n);
    joint_conf_.pos_limit_upper.resize(n);
    for (size_t ii = 0; ii < n; ++ii) {
      const std::string& name = joint_names_[ii];
      if (limits_node[name]) {
        joint_conf_.pos_limit_lower(ii) = limits_node[name]["lower"].as<double>();
        joint_conf_.pos_limit_upper(ii) = limits_node[name]["upper"].as<double>();
      } else {
        joint_conf_.pos_limit_lower(ii) = -std::numeric_limits<double>::infinity();
        joint_conf_.pos_limit_upper(ii) = std::numeric_limits<double>::infinity();
        AIMRT_WARN("No joint_limits found for '{}', skipping clamp.", name);
      }
    }
  }

  walk_step_conf_.action_scale = cfg_node["walk_step_conf"]["action_scale"].as<double>();
  walk_step_conf_.decimation = cfg_node["walk_step_conf"]["decimation"].as<int32_t>();
  walk_step_conf_.cycle_time = cfg_node["walk_step_conf"]["cycle_time"].as<double>();
  walk_step_conf_.sw_mode = cfg_node["walk_step_conf"]["sw_mode"].as<bool>();
  walk_step_conf_.cmd_threshold = cfg_node["walk_step_conf"]["cmd_threshold"].as<double>();
  obs_scales_.lin_vel = cfg_node["obs_scales"]["lin_vel"].as<double>();
  obs_scales_.ang_vel = cfg_node["obs_scales"]["ang_vel"].as<double>();
  obs_scales_.dof_pos = cfg_node["obs_scales"]["dof_pos"].as<double>();
  obs_scales_.dof_vel = cfg_node["obs_scales"]["dof_vel"].as<double>();
  obs_scales_.quat = cfg_node["obs_scales"]["quat"].as<double>();
  onnx_conf_.policy_file = cfg_node["onnx_conf"]["policy_file"].as<std::string>();
  onnx_conf_.actions_size = cfg_node["onnx_conf"]["actions_size"].as<int32_t>();
  onnx_conf_.observations_size = cfg_node["onnx_conf"]["observations_size"].as<int32_t>();
  onnx_conf_.num_hist = cfg_node["onnx_conf"]["num_hist"].as<int32_t>();
  onnx_conf_.observations_clip = cfg_node["onnx_conf"]["observations_clip"].as<double>();
  onnx_conf_.actions_clip = cfg_node["onnx_conf"]["actions_clip"].as<double>();
  // 线程数带默认值 1：兼容旧配置，且默认即单线程串行（避免 ORT 默认吃满全部核）
  onnx_conf_.inter_op_threads = cfg_node["onnx_conf"]["inter_op_threads"]
                                    ? cfg_node["onnx_conf"]["inter_op_threads"].as<int32_t>()
                                    : 1;
  onnx_conf_.intra_op_threads = cfg_node["onnx_conf"]["intra_op_threads"]
                                    ? cfg_node["onnx_conf"]["intra_op_threads"].as<int32_t>()
                                    : 1;
  lpf_conf_.wc = cfg_node["lpf_conf"]["wc"].as<double>();
  lpf_conf_.ts = cfg_node["lpf_conf"]["ts"].as<double>();
  auto paralle_list = cfg_node["lpf_conf"]["paralle_list"].as<std::vector<std::string>>();
  lpf_conf_.paralle_list = std::set<std::string>(paralle_list.begin(), paralle_list.end());

  // AMP 全身策略模式（可选，缺省 legacy 保持旧策略行为不变）。
  // 契约来源：x1_29_AMP sim2sim/deploy_onnx_rt.py + mujoco_rollout.py。
  {
    const std::string obs_layout =
        cfg_node["obs_layout"] ? cfg_node["obs_layout"].as<std::string>() : "legacy";
    amp_mode_ = (obs_layout == "amp");
    if (amp_mode_) {
      AIMRT_CHECK_ERROR_THROW(
          onnx_conf_.observations_size == 9 + 3 * onnx_conf_.actions_size,
          "amp obs_layout requires observations_size == 9 + 3*actions_size, got {} vs {}",
          onnx_conf_.observations_size, 9 + 3 * onnx_conf_.actions_size);
      const auto& amp_node = cfg_node["amp_conf"];
      amp_conf_.settle_ticks =
          amp_node["settle_ticks"] ? amp_node["settle_ticks"].as<int>() : 50;
      amp_conf_.cmd_x_min = amp_node["cmd_x_range"] ? amp_node["cmd_x_range"][0].as<double>() : -0.5;
      amp_conf_.cmd_x_max = amp_node["cmd_x_range"] ? amp_node["cmd_x_range"][1].as<double>() : 2.5;
      amp_conf_.cmd_y_min = amp_node["cmd_y_range"] ? amp_node["cmd_y_range"][0].as<double>() : -0.5;
      amp_conf_.cmd_y_max = amp_node["cmd_y_range"] ? amp_node["cmd_y_range"][1].as<double>() : 0.5;
      amp_conf_.cmd_yaw_min = amp_node["cmd_yaw_range"] ? amp_node["cmd_yaw_range"][0].as<double>() : -1.5;
      amp_conf_.cmd_yaw_max = amp_node["cmd_yaw_range"] ? amp_node["cmd_yaw_range"][1].as<double>() : 1.5;
      // 验收复刻模式（可选）: 恒定速度指令, 对齐 sim2sim mujoco_rollout 的
      // --cmd 常量语义(全程恒定, settle 期间不清零)。设置后外部速度输入被忽略。
      if (amp_node["constant_cmd"]) {
        const auto cc = amp_node["constant_cmd"].as<std::vector<double>>();
        AIMRT_CHECK_ERROR_THROW(cc.size() == 3, "amp_conf.constant_cmd must be [vx, vy, wz]");
        amp_conf_.use_constant_cmd = true;
        amp_conf_.const_cmd_x = cc[0];
        amp_conf_.const_cmd_y = cc[1];
        amp_conf_.const_cmd_z = cc[2];
      }
      amp_conf_.sensor_timeout_ms = amp_node["sensor_timeout_ms"] ? amp_node["sensor_timeout_ms"].as<int>() : 100;
      amp_conf_.cmd_timeout_ms = amp_node["cmd_timeout_ms"] ? amp_node["cmd_timeout_ms"].as<int>() : 250;
      amp_conf_.max_tilt_deg = amp_node["max_tilt_deg"] ? amp_node["max_tilt_deg"].as<double>() : 45.0;
      amp_conf_.ankle_effort_limit = amp_node["ankle_effort_limit"]
                                         ? amp_node["ankle_effort_limit"].as<double>() : 80.0;
      const int control_hz = cfg_node["control_frequency_hz"]
                                 ? cfg_node["control_frequency_hz"].as<int>() : 1000;
      AIMRT_CHECK_ERROR_THROW(walk_step_conf_.decimation > 0 && onnx_conf_.num_hist == 3 &&
                                  onnx_conf_.actions_size == 29 &&
                                  onnx_conf_.actions_size == static_cast<int>(joint_names_.size()) &&
                                  std::abs(walk_step_conf_.action_scale - 0.25) < 1e-9 &&
                                  joint_conf_.init_state.size() == onnx_conf_.actions_size &&
                                  joint_conf_.stiffness.size() == onnx_conf_.actions_size &&
                                  joint_conf_.damping.size() == onnx_conf_.actions_size &&
                                  amp_conf_.settle_ticks >= 0 && amp_conf_.sensor_timeout_ms > 0 &&
                                  amp_conf_.cmd_timeout_ms > 0 && amp_conf_.max_tilt_deg > 0 &&
                                  amp_conf_.max_tilt_deg < 90 && amp_conf_.ankle_effort_limit > 0 &&
                                  control_hz % walk_step_conf_.decimation == 0 &&
                                  control_hz / walk_step_conf_.decimation == 50,
                              "Invalid AMP dimensions, rate or safety parameters");
      for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
        AIMRT_CHECK_ERROR_THROW(std::isfinite(joint_conf_.pos_limit_lower(ii)) &&
                                    std::isfinite(joint_conf_.pos_limit_upper(ii)) &&
                                    joint_conf_.pos_limit_lower(ii) < joint_conf_.pos_limit_upper(ii) &&
                                    joint_conf_.init_state(ii) >= joint_conf_.pos_limit_lower(ii) &&
                                    joint_conf_.init_state(ii) <= joint_conf_.pos_limit_upper(ii) &&
                                    std::isfinite(joint_conf_.stiffness(ii)) &&
                                    std::isfinite(joint_conf_.damping(ii)) &&
                                    joint_conf_.stiffness(ii) > 0 && joint_conf_.damping(ii) >= 0,
                                "Invalid AMP joint limits or PD parameters for {}", joint_names_[ii]);
      }
      AIMRT_INFO(
          "RLController amp mode: obs {} x hist {}, act {}, action_scale {}, "
          "decimation {}, settle {} ticks, cmd x[{},{}] y[{},{}] yaw[{},{}], cmd source: {}",
          onnx_conf_.observations_size, onnx_conf_.num_hist, onnx_conf_.actions_size,
          walk_step_conf_.action_scale, walk_step_conf_.decimation, amp_conf_.settle_ticks,
          amp_conf_.cmd_x_min, amp_conf_.cmd_x_max,
          amp_conf_.cmd_y_min, amp_conf_.cmd_y_max,
          amp_conf_.cmd_yaw_min, amp_conf_.cmd_yaw_max,
          amp_conf_.use_constant_cmd ? "CONSTANT" : "external");
      if (amp_conf_.use_constant_cmd) {
        AIMRT_INFO("amp CONSTANT CMD = [{:.2f}, {:.2f}, {:.2f}] (external /cmd_vel_limiter ignored)",
                   amp_conf_.const_cmd_x, amp_conf_.const_cmd_y, amp_conf_.const_cmd_z);
      }
    }
  }
  const bool default_lpf = cfg_node["lpf_conf"]["enabled"]
                               ? cfg_node["lpf_conf"]["enabled"].as<bool>() : !amp_mode_;
  lpf_conf_.position_enabled = cfg_node["lpf_conf"]["position_enabled"]
                                   ? cfg_node["lpf_conf"]["position_enabled"].as<bool>() : default_lpf;
  lpf_conf_.effort_enabled = cfg_node["lpf_conf"]["effort_enabled"]
                                 ? cfg_node["lpf_conf"]["effort_enabled"].as<bool>() : default_lpf;

  if (const auto finish = cfg_node["dance_finish"]; finish && finish["enabled"].as<bool>()) {
    AIMRT_CHECK_ERROR_THROW(amp_mode_ && amp_conf_.use_constant_cmd && amp_conf_.const_cmd_x == 0 &&
        amp_conf_.const_cmd_y == 0 && amp_conf_.const_cmd_z == 0 && amp_conf_.settle_ticks == 0 &&
        amp_conf_.sensor_timeout_ms <= 80 && !lpf_conf_.position_enabled && !lpf_conf_.effort_enabled,
        "dance finisher requires zero-command AMP, no settle/LPF, and <=80 ms feedback timeout");
    DanceFinishCore::Config c;
    const std::vector<std::pair<const char*, double*>> fields{
      {"blend_s", &c.blend_s}, {"upper_delay_s", &c.upper_delay_s}, {"upper_blend_s", &c.upper_blend_s},
      {"landing_min_s", &c.landing_min_s}, {"landing_hold_s", &c.landing_hold_s},
      {"stand_blend_s", &c.stand_blend_s}, {"verify_s", &c.verify_s},
      {"max_tilt_deg", &c.max_tilt_deg}, {"quiet_tilt_deg", &c.quiet_tilt_deg},
      {"quiet_gyro", &c.quiet_gyro}, {"quiet_joint_speed", &c.quiet_joint_speed},
      {"quiet_base_speed", &c.quiet_base_speed}, {"contact_on", &c.contact_on}, {"contact_off", &c.contact_off},
      {"max_foot_height_delta", &c.max_foot_height_delta}, {"max_foot_velocity_delta", &c.max_foot_velocity_delta},
      {"recovery_tilt_deg", &c.recovery_tilt_deg}, {"recovery_gyro", &c.recovery_gyro},
      {"recovery_speed", &c.recovery_speed}, {"recovery_delay_s", &c.recovery_delay_s},
      {"upper_target_speed", &c.upper_target_speed}, {"force_filter_s", &c.force_filter_s},
      {"max_stand_pose_error", &c.max_stand_pose_error}};
    for (const auto& [key, value] : fields) if (finish[key]) *value = finish[key].as<double>();
    AIMRT_CHECK_ERROR_THROW(c.max_tilt_deg == amp_conf_.max_tilt_deg, "finish and AMP tilt limits must match");
    const auto p = finish["stand_stiffness"].as<std::vector<double>>();
    const auto d = finish["stand_damping"].as<std::vector<double>>();
    const auto pose = finish["stand_pose"].as<std::vector<double>>();
    AIMRT_CHECK_ERROR_THROW(p.size() == 29 && d.size() == 29 && pose.size() == 29,
                            "finish stand pose/gains must cover 29 joints");
    for (size_t i = 0; i < 29; ++i) {
      c.lower[i] = joint_conf_.pos_limit_lower(i); c.upper[i] = joint_conf_.pos_limit_upper(i);
      c.nominal[i] = joint_conf_.init_state(i); c.kp[i] = joint_conf_.stiffness(i); c.kd[i] = joint_conf_.damping(i);
      c.stand_kp[i] = p[i]; c.stand_kd[i] = d[i]; c.stand_pose[i] = pose[i];
    }
    for (size_t i = 0; i < 12; ++i) {
      const auto j = std::find(joint_names_.begin(), joint_names_.end(), kDanceLegJointNames[i]);
      AIMRT_CHECK_ERROR_THROW(j != joint_names_.end(), "finish is missing a leg joint");
      c.legs[i] = std::distance(joint_names_.begin(), j);
    }
    finish_core_ = std::make_unique<DanceFinishCore>(c);
    const auto diag = finish["diagnostics"];
    if (!diag || diag["enabled"].as<bool>(true)) {
      const auto directory = diag ? diag["directory"].as<std::string>("test_logs/data_csv") : "test_logs/data_csv";
      if (finish_diagnostics_.Start(directory, joint_names_, 100, 256, 0)) {
        std::ofstream snapshot(finish_diagnostics_.Path()+".yaml"); snapshot << cfg_node << '\n';
        AIMRT_INFO("Dance finish diagnostics: {} (observation_dim=0; AMP inputs use tm_obs_input)", finish_diagnostics_.Path());
      }
    }
  }

  LoadModel();

  diag_log_dir_ = "test_logs/data_csv";
  std::filesystem::create_directories(diag_log_dir_);
  diag_log_max_count_ = 20 * (1000 / walk_step_conf_.decimation);
  diag_logging_enabled_ = true;
  pd_pos_des_raw_.assign(onnx_conf_.actions_size, std::numeric_limits<double>::quiet_NaN());
  pd_pos_des_lpf_.assign(onnx_conf_.actions_size, std::numeric_limits<double>::quiet_NaN());
  pd_tau_des_raw_.assign(onnx_conf_.actions_size, std::numeric_limits<double>::quiet_NaN());
  pd_tau_des_lpf_.assign(onnx_conf_.actions_size, std::numeric_limits<double>::quiet_NaN());
  pd_is_parallel_.assign(onnx_conf_.actions_size, 0);

  tm_log_dir_ = "test_logs/data_csv/t_m";
  std::filesystem::create_directories(tm_log_dir_);
  tm_log_max_count_ = 10 * (1000 / walk_step_conf_.decimation);
  tm_logging_enabled_ = true;

  diag_ring_.resize(kDiagRingCapacity);
  for (auto& frame : diag_ring_) {
    frame.actions.resize(onnx_conf_.actions_size);
    frame.joint_pos.resize(onnx_conf_.actions_size);
    frame.joint_vel.resize(onnx_conf_.actions_size);
    frame.joint_effort.resize(onnx_conf_.actions_size);
    frame.pos_des_raw.resize(onnx_conf_.actions_size);
    frame.pos_des_lpf.resize(onnx_conf_.actions_size);
    frame.tau_des_raw.resize(onnx_conf_.actions_size);
    frame.tau_des_lpf.resize(onnx_conf_.actions_size);
    frame.is_parallel.resize(onnx_conf_.actions_size);
  }

  tm_ring_.resize(kTmRingCapacity);
  for (auto& frame : tm_ring_) {
    frame.observations.resize(onnx_conf_.observations_size * onnx_conf_.num_hist);
  }

  propri_.joint_pos.resize(onnx_conf_.actions_size);
  propri_.joint_vel.resize(onnx_conf_.actions_size);
  propri_.joint_effort.resize(onnx_conf_.actions_size);
  actions_.resize(onnx_conf_.actions_size);
  observations_.resize(onnx_conf_.observations_size * onnx_conf_.num_hist);
  last_actions_.resize(onnx_conf_.actions_size);
  last_actions_.setZero();
  amp_transition_start_pos_.resize(onnx_conf_.actions_size);
  propri_history_buffer_.resize(onnx_conf_.observations_size * onnx_conf_.num_hist);
  low_pass_filters_.clear();
  for (int i = 0; i < onnx_conf_.actions_size; ++i) {
    low_pass_filters_.emplace_back(lpf_conf_.wc, lpf_conf_.ts);
  }

  StartLoggingWorker();
}

void RLController::RestartController() {
  if (finish_core_ && !finish_pending_) {
    AIMRT_CHECK_ERROR_THROW(finish_core_->Active(), "dance finisher requires a normal dance handoff");
    return;  // finish <-> stand changes preserve the active controller and its captured stance.
  }
  is_first_frame_ = true;
  diag_pending_frame_ = false;
  if (amp_mode_) {
    // 50Hz 策略不作常态 LPF；只在模式切换时从实测姿态平滑进入 default_q。
    amp_settle_remaining_ = amp_conf_.settle_ticks;
    amp_transition_elapsed_ = 0;
    amp_fault_.store(false, std::memory_order_release);
    loop_count_ = 0;
    last_actions_.setZero();
    std::fill(actions_.begin(), actions_.end(), 0.0f);
    std::shared_lock<std::shared_mutex> lock(joint_state_mutex_);
    for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
      amp_transition_start_pos_(ii) = joint_state_data_.position[ii];
    }
  }
  if (finish_core_) {
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    finish_core_->Begin(finish_start_targets_, now);
    finish_pending_ = false; finish_tick_ = 0; ++finish_session_;
    finish_logged_phase_ = DanceFinishCore::Phase::kInactive; finish_logged_confirmed_ = false;
  }
}

void RLController::PrepareDanceHandoff(const std::vector<double>& targets) {
  AIMRT_CHECK_ERROR_THROW(finish_core_ && targets.size() == 29, "invalid dance finish handoff");
  std::copy(targets.begin(), targets.end(), finish_start_targets_.begin());
  finish_pending_ = true;
}

void RLController::SetImuData(const sensor_msgs::msg::Imu value) {
  if (!finish_core_) { ControllerBase::SetImuData(value); return; }
  std::lock_guard<std::mutex> guard(finish_feedback_mutex_);
  const auto stamp = FinishStampNs(value.header.stamp);
  const auto& q = value.orientation;
  const double norm = q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
  if (!FinishStampFresh(stamp, use_sim_handles_) || stamp <= finish_imu_source_ns_ ||
      !std::isfinite(norm) || norm < 0.81 || norm > 1.21 ||
      !std::isfinite(value.angular_velocity.x) || !std::isfinite(value.angular_velocity.y) ||
      !std::isfinite(value.angular_velocity.z)) return;
  ControllerBase::SetImuData(value); finish_imu_source_ns_ = stamp;
}

void RLController::SetJointStateData(sensor_msgs::msg::JointState value,
                                    const std::unordered_map<std::string, int>& index) {
  if (!finish_core_) { ControllerBase::SetJointStateData(value, index); return; }
  std::lock_guard<std::mutex> guard(finish_feedback_mutex_);
  const auto stamp = FinishStampNs(value.header.stamp);
  if (!FinishStampFresh(stamp, use_sim_handles_) || stamp <= finish_joint_source_ns_) return;
  bool effort_valid = true;
  for (const auto& name : joint_names_) {
    const auto it = index.find(name);
    if (it == index.end() || it->second < 0) return;
    const auto i = static_cast<size_t>(it->second);
    const bool leg = IsLegActuator(name);
    if (i >= value.name.size() || value.name[i] != name || i >= value.position.size() ||
        i >= value.velocity.size() || !std::isfinite(value.position[i]) || !std::isfinite(value.velocity[i])) return;
    if (leg && (i >= value.effort.size() || !std::isfinite(value.effort[i]))) effort_valid = false;
  }
  // Zero-command balance needs q/qd/IMU, not torque. Missing leg torque only
  // prevents quiet-support certification and static capture; never freeze a
  // moving robot merely because a contact cue is unavailable.
  value.effort.resize(value.name.size(), 0.0);
  ControllerBase::SetJointStateData(value, index);
  finish_joint_source_ns_ = stamp; finish_effort_valid_ = effort_valid;
}

bool RLController::AmpReady() const {
  if (!amp_mode_) return true;
  const int64_t now_ns = duration_cast<nanoseconds>(
      steady_clock::now().time_since_epoch()).count();
  const int64_t max_age_ns = static_cast<int64_t>(amp_conf_.sensor_timeout_ms) * 1'000'000;
  const int64_t joint_ns = last_joint_state_ns_.load(std::memory_order_acquire);
  const int64_t imu_ns = last_imu_ns_.load(std::memory_order_acquire);
  return joint_ns > 0 && imu_ns > 0 && now_ns >= joint_ns && now_ns >= imu_ns &&
         now_ns - joint_ns <= max_age_ns && now_ns - imu_ns <= max_age_ns;
}

void RLController::SetLoggingActive(bool active) {
  diag_logging_requested_.store(active, std::memory_order_release);
  tm_logging_requested_.store(active, std::memory_order_release);
}

void RLController::StopLoggingWorker() {
  if (!log_worker_started_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }

  log_worker_running_.store(false, std::memory_order_release);
  if (log_worker_thread_.joinable()) {
    log_worker_thread_.join();
  }
}

void RLController::StartLoggingWorker() {
  if (log_worker_started_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }

  log_worker_running_.store(true, std::memory_order_release);
  try {
    log_worker_thread_ = std::thread([this]() { LoggingWorkerLoop(); });
  } catch (...) {
    log_worker_running_.store(false, std::memory_order_release);
    log_worker_started_.store(false, std::memory_order_release);
    throw;
  }
}

void RLController::LoggingWorkerLoop() {
  auto last_diag_flush = std::chrono::steady_clock::now();
  auto last_tm_flush = std::chrono::steady_clock::now();

  while (log_worker_running_.load(std::memory_order_acquire)) {
    DrainDiagBuffer();
    DrainTmBuffer();

    const auto now = std::chrono::steady_clock::now();
    const auto now_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();

    if (diag_logging_triggered_) {
      const auto last_enqueue_ns = diag_last_enqueue_ns_.load(std::memory_order_acquire);
      if (last_enqueue_ns > 0 &&
          std::chrono::nanoseconds(now_ns - last_enqueue_ns) >= kLogIdleTimeout) {
        FinalizeDiagLogging("idle_timeout");
      } else if (now - last_diag_flush >= kLogFlushInterval) {
        if (!diag_logger_.Flush()) {
          FinalizeDiagLogging("flush_error");
        } else {
          last_diag_flush = now;
        }
      }
    }

    if (tm_logging_triggered_) {
      const auto last_enqueue_ns = tm_last_enqueue_ns_.load(std::memory_order_acquire);
      if (last_enqueue_ns > 0 &&
          std::chrono::nanoseconds(now_ns - last_enqueue_ns) >= kLogIdleTimeout) {
        FinalizeTmLogging("idle_timeout");
      } else if (now - last_tm_flush >= kLogFlushInterval) {
        if (!tm_logger_.Flush()) {
          FinalizeTmLogging("flush_error");
        } else {
          last_tm_flush = now;
        }
      }
    }

    std::this_thread::sleep_for(kLogWorkerPollInterval);
  }

  DrainDiagBuffer();
  DrainTmBuffer();
  FinalizeDiagLogging("worker_stop");
  FinalizeTmLogging("worker_stop");
}

void RLController::Update() {
  std::unique_lock<std::mutex> finish_lock;
  if (finish_core_) finish_lock = std::unique_lock<std::mutex>(finish_feedback_mutex_);
  if (amp_mode_ && !AmpReady()) {
    amp_fault_.store(true, std::memory_order_release);
    std::fill(actions_.begin(), actions_.end(), 0.0f);
    AIMRT_ERROR("AMP feedback stale or missing");
    return;
  }
  UpdateStateEstimation();
  if (finish_core_) {
    finish_joint_snapshot_ns_ = finish_joint_source_ns_; finish_imu_snapshot_ns_ = finish_imu_source_ns_;
    finish_effort_snapshot_valid_ = finish_effort_valid_;
    if (!finish_core_->Active() || !FinishStampFresh(finish_joint_snapshot_ns_, use_sim_handles_) ||
        !FinishStampFresh(finish_imu_snapshot_ns_, use_sim_handles_)) {
      amp_fault_.store(true, std::memory_order_release); return;
    }
    finish_lock.unlock();
  }
  if (amp_mode_ && propri_.projected_gravity.z() >
                       -std::cos(amp_conf_.max_tilt_deg * M_PI / 180.0)) {
    amp_fault_.store(true, std::memory_order_release);
    std::fill(actions_.begin(), actions_.end(), 0.0f);
    AIMRT_ERROR("AMP tilt exceeded {} deg", amp_conf_.max_tilt_deg);
    return;
  }

  if (loop_count_ % walk_step_conf_.decimation == 0) {
    ComputeObservation();
    if (amp_mode_) {
      try {
        ComputeActions();
      } catch (const std::exception& e) {
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        amp_fault_.store(true, std::memory_order_release);
        AIMRT_ERROR("AMP ONNX inference failed: {}", e.what());
        return;
      }
    } else {
      ComputeActions();
    }
    if (amp_mode_ && amp_settle_remaining_ > 0) {
      // 软启动期：目标角 = default_q，零动作（观测侧 cmd 同拍置零，见 ComputeObservation）
      std::fill(actions_.begin(), actions_.end(), 0.0f);
      --amp_settle_remaining_;
    }

    if (diag_logging_enabled_) {
      diag_pending_frame_ = true;
    }

    if (tm_logging_enabled_) {
      (void)EnqueueTmFrame();
    }
  }

  if (finish_core_ && !AmpFaulted()) UpdateDanceFinish();
  loop_count_++;
}

void RLController::UpdateDanceFinish() {
  DanceFinishCore::Input in;
  DanceFinishCore::Joints proposed;
  for (size_t i = 0; i < 29; ++i) {
    in.q[i] = propri_.joint_pos(i); in.qd[i] = propri_.joint_vel(i); in.effort[i] = propri_.joint_effort(i);
    proposed[i] = std::clamp(joint_conf_.init_state(i)+walk_step_conf_.action_scale*actions_[i],
                            joint_conf_.pos_limit_lower(i), joint_conf_.pos_limit_upper(i));
  }
  const auto& q = propri_.imu_quat;
  in.orientation = {q.w(), q.x(), q.y(), q.z()};
  in.gyro = {propri_.base_ang_vel.x(), propri_.base_ang_vel.y(), propri_.base_ang_vel.z()};
  in.source_ns = std::min(finish_joint_snapshot_ns_, finish_imu_snapshot_ns_);
  in.contact_input_valid = finish_effort_snapshot_valid_ &&
      std::abs(finish_joint_snapshot_ns_-finish_imu_snapshot_ns_) <= 10000000;
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  finish_core_->Update(in, proposed, now); ++finish_tick_;
  if (finish_core_->Faulted()) {
    amp_fault_.store(true, std::memory_order_release);
    AIMRT_ERROR("Dance finish fault: {}", finish_core_->Reason());
  }
}

void RLController::LogDanceFinish(const my_ros2_proto::msg::JointCommand& command) {
  const auto phase = finish_core_->GetPhase();
  const bool confirmed = finish_core_->StandingConfirmed();
  const bool changed = phase != finish_logged_phase_ || confirmed != finish_logged_confirmed_;
  if (changed) AIMRT_INFO("Dance finish phase={} confirmed={} quiet_s={:.3f} recoveries={} reason={}",
      finish_core_->PhaseName(), confirmed, finish_core_->QuietSeconds(), finish_core_->Recoveries(), finish_core_->Reason());
  if (finish_diagnostics_.Enabled() && (changed || finish_tick_%20 == 0)) {
    using D = DanceDiagnostics;
    D::Frame f; D::Stamp(f);
    f.tick = finish_tick_; f.session = finish_session_;
    f.joint_stamp_ns = finish_joint_snapshot_ns_; f.imu_stamp_ns = finish_imu_snapshot_ns_;
    f.velocity_stamp_ns = std::min(f.joint_stamp_ns, f.imu_stamp_ns);
    f.snapshot_valid = f.has_action = f.command_valid = true;
    D::Text(f.state, finish_core_->PhaseName()); D::Text(f.event, changed ? "finish_phase" : "finish_sample");
    D::Text(f.reason, finish_core_->Reason()); D::Text(f.state_source, "foot_kinematics");
    const auto& q = propri_.imu_quat;
    const auto v = finish_core_->Velocity();
    const vector3_t velocity = q.conjugate()*vector3_t(v[0], v[1], v[2]);
    const std::array<double, 4> quaternion{q.w(), q.x(), q.y(), q.z()};
    for (size_t i = 0; i < 4; ++i) f.values[D::kImuQuat+i] = quaternion[i];
    for (size_t i = 0; i < 3; ++i) {
      f.values[D::kImuGyro+i] = propri_.base_ang_vel(i);
      f.values[D::kImuAccel+i] = propri_.imu_accel(i);
      if (finish_core_->Contacts()[0] && finish_core_->Contacts()[1]) f.values[D::kBodyVelocity+i] = velocity(i);
    }
    for (size_t i = 0; i < 2; ++i) {
      f.values[D::kContactForce+i] = finish_core_->Forces()[i];
      f.values[D::kContact+i] = finish_core_->Contacts()[i];
    }
    for (size_t i = 0; i < 29; ++i) {
      f.values[D::kJointPosition+i] = propri_.joint_pos(i);
      f.values[D::kJointVelocity+i] = propri_.joint_vel(i);
      f.values[D::kJointEffort+i] = propri_.joint_effort(i);
      f.values[D::kAction+i] = actions_[i]; f.values[D::kTarget+i] = finish_core_->Targets()[i];
      f.values[D::kCommandPosition+i] = command.position[i];
      f.values[D::kCommandEffort+i] = command.effort[i];
      f.values[D::kCommandKp+i] = command.stiffness[i]; f.values[D::kCommandKd+i] = command.damping[i];
    }
    finish_diagnostics_.Enqueue(f);
  }
  finish_logged_phase_ = phase; finish_logged_confirmed_ = confirmed;
}

my_ros2_proto::msg::JointCommand RLController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.resize(joint_names_.size());
  joint_cmd.velocity.resize(joint_names_.size());
  joint_cmd.effort.resize(joint_names_.size());
  joint_cmd.damping.resize(joint_names_.size());
  joint_cmd.stiffness.resize(joint_names_.size());

  double settle_blend = 1.0;
  if (amp_mode_) {
    const int total = amp_conf_.settle_ticks * walk_step_conf_.decimation;
    if (total > 0 && amp_transition_elapsed_ < total) {
      const double t = static_cast<double>(++amp_transition_elapsed_) / total;
      settle_blend = t * t * (3.0 - 2.0 * t);
    }
  }

  for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
    const scalar_t pos_des_raw = actions_[ii] * walk_step_conf_.action_scale + joint_conf_.init_state(ii);
    scalar_t pos_des = pos_des_raw;
    if (amp_mode_ && settle_blend < 1.0) {
      pos_des = (1.0 - settle_blend) * amp_transition_start_pos_(ii) +
                settle_blend * joint_conf_.init_state(ii);
    }
    if (amp_mode_) {
      pos_des = std::clamp(pos_des, joint_conf_.pos_limit_lower(ii), joint_conf_.pos_limit_upper(ii));
    }
    if (finish_core_) pos_des = finish_core_->Targets()[ii];
    const double stiffness = finish_core_ ? finish_core_->Kp()[ii] : joint_conf_.stiffness(ii);
    const double damping = finish_core_ ? finish_core_->Kd()[ii] : joint_conf_.damping(ii);
    pd_pos_des_raw_[ii] = pos_des_raw;
    pd_pos_des_lpf_[ii] = std::numeric_limits<double>::quiet_NaN();
    pd_tau_des_raw_[ii] = std::numeric_limits<double>::quiet_NaN();
    pd_tau_des_lpf_[ii] = std::numeric_limits<double>::quiet_NaN();

    if (lpf_conf_.paralle_list.find(joint_names_[ii]) == lpf_conf_.paralle_list.end()) {
      pd_is_parallel_[ii] = 0;
      if (lpf_conf_.position_enabled) low_pass_filters_[ii].input(pos_des);
      const double pos_des_lp = lpf_conf_.position_enabled ? low_pass_filters_[ii].output() : pos_des;
      pd_pos_des_lpf_[ii] = pos_des_lp;
      joint_cmd.position[ii] = pos_des_lp;
      joint_cmd.velocity[ii] = 0.0;
      joint_cmd.effort[ii] = 0.0;
      joint_cmd.stiffness[ii] = stiffness;
      joint_cmd.damping[ii] = damping;
    } else {
      pd_is_parallel_[ii] = 1;
      const double tau_des_raw = stiffness * (pos_des - propri_.joint_pos[ii]) + damping * (0.0 - propri_.joint_vel[ii]);
      const double tau_des = amp_mode_
                                 ? std::clamp(tau_des_raw, -amp_conf_.ankle_effort_limit,
                                              amp_conf_.ankle_effort_limit)
                                 : tau_des_raw;
      pd_tau_des_raw_[ii] = tau_des_raw;
      if (lpf_conf_.effort_enabled) low_pass_filters_[ii].input(tau_des);
      const double tau_des_lp = lpf_conf_.effort_enabled ? low_pass_filters_[ii].output() : tau_des;
      pd_tau_des_lpf_[ii] = tau_des_lp;
      joint_cmd.position[ii] = 0.0;
      joint_cmd.velocity[ii] = 0.0;
      joint_cmd.effort[ii] = tau_des_lp;
      joint_cmd.stiffness[ii] = 0.0;
      joint_cmd.damping[ii] = 0.0;
    }

    last_actions_(ii, 0) = actions_[ii];
  }

  if (diag_pending_frame_) {
    (void)EnqueueDiagFrame();
    diag_pending_frame_ = false;
  }

  if (finish_core_) LogDanceFinish(joint_cmd);

  return joint_cmd;
}

void RLController::LoadModel() {
  static Ort::Env onnx_env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController");
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetInterOpNumThreads(onnx_conf_.inter_op_threads);
  sessionOptions.SetIntraOpNumThreads(onnx_conf_.intra_op_threads);
  // 禁用算子内空闲 worker 自旋，避免在管家核上持续 busy-wait 产生背景噪声
  sessionOptions.AddConfigEntry("session.intra_op.allow_spinning", "0");
  session_ptr_ = std::make_unique<Ort::Session>(onnx_env, onnx_conf_.policy_file.c_str(), sessionOptions);

  if (amp_mode_) {
    AIMRT_CHECK_ERROR_THROW(session_ptr_->GetInputCount() == 1 && session_ptr_->GetOutputCount() == 1,
                            "AMP policy must have one input and one output");
    // ORT C++ API 陷阱: ConstTensorTypeAndShapeInfo 是非拥有视图, 底层内存归 TypeInfo 所有。
    // 必须让 TypeInfo 活过使用点, 否则悬垂读取(free 后被堆元数据覆写)导致校验误报。
    const auto in_type_info = session_ptr_->GetInputTypeInfo(0);
    const auto out_type_info = session_ptr_->GetOutputTypeInfo(0);
    const auto in_info = in_type_info.GetTensorTypeAndShapeInfo();
    const auto out_info = out_type_info.GetTensorTypeAndShapeInfo();
    const std::vector<int64_t> expected_in{
        1, static_cast<int64_t>(onnx_conf_.observations_size) * onnx_conf_.num_hist};
    const std::vector<int64_t> expected_out{1, onnx_conf_.actions_size};
    const bool contract_ok = in_info.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
                             out_info.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
                             in_info.GetShape() == expected_in && out_info.GetShape() == expected_out;
    if (!contract_ok) {
      auto shape_str = [](const std::vector<int64_t>& v) {
        std::string s;
        for (size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + std::to_string(v[i]);
        return s.empty() ? std::string("<none>") : s;
      };
      AIMRT_ERROR("AMP ONNX contract mismatch detail: input type={} shape=[{}], output type={} shape=[{}], "
                  "expected FLOAT [1,{}] / [1,{}]",
                  static_cast<int>(in_info.GetElementType()), shape_str(in_info.GetShape()),
                  static_cast<int>(out_info.GetElementType()), shape_str(out_info.GetShape()),
                  onnx_conf_.observations_size * onnx_conf_.num_hist, onnx_conf_.actions_size);
      AIMRT_CHECK_ERROR_THROW(false, "ONNX model tensor contract differs from controller configuration");
    }
  }

  input_names_.clear();
  output_names_.clear();
  input_name_storage_.clear();
  output_name_storage_.clear();
  input_shapes_.clear();
  output_shapes_.clear();

  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t ii = 0; ii < session_ptr_->GetInputCount(); ++ii) {
    auto name = session_ptr_->GetInputNameAllocated(ii, allocator);
    input_name_storage_.emplace_back(name.get());
    input_shapes_.push_back(session_ptr_->GetInputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }

  for (size_t ii = 0; ii < session_ptr_->GetOutputCount(); ++ii) {
    auto name = session_ptr_->GetOutputNameAllocated(ii, allocator);
    output_name_storage_.emplace_back(name.get());
    output_shapes_.push_back(session_ptr_->GetOutputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (const auto& name : input_name_storage_) input_names_.push_back(name.c_str());
  for (const auto& name : output_name_storage_) output_names_.push_back(name.c_str());
  if (amp_mode_) {
    AIMRT_CHECK_ERROR_THROW(input_name_storage_[0] == "obs" && output_name_storage_[0] == "action",
                            "AMP ONNX tensor names must be obs/action");
  }
}

void RLController::UpdateStateEstimation() {
  {
    std::shared_lock<std::shared_mutex> lock(joint_state_mutex_);
    for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
      propri_.joint_pos(ii) = joint_state_data_.position[ii];
      propri_.joint_vel(ii) = joint_state_data_.velocity[ii];
      propri_.joint_effort(ii) = joint_state_data_.effort[ii];
    }
  }

  {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    propri_.base_ang_vel(0) = imu_data_.angular_velocity.x;
    propri_.base_ang_vel(1) = imu_data_.angular_velocity.y;
    propri_.base_ang_vel(2) = imu_data_.angular_velocity.z;

    vector3_t gravity_vector(0, 0, -1);
    quaternion_t quat;
    quat.x() = imu_data_.orientation.x;
    quat.y() = imu_data_.orientation.y;
    quat.z() = imu_data_.orientation.z;
    quat.w() = imu_data_.orientation.w;
    propri_.imu_accel(0) = imu_data_.linear_acceleration.x;
    propri_.imu_accel(1) = imu_data_.linear_acceleration.y;
    propri_.imu_accel(2) = imu_data_.linear_acceleration.z;
    quat.normalize();
    propri_.imu_quat = quat;
    propri_.projected_gravity = quat.conjugate() * gravity_vector;
    propri_.base_euler_xyz = QuatToXyz(quat);
  }
}

void RLController::ComputeObservation() {
  vector_t propri_obs(onnx_conf_.observations_size);
  if (amp_mode_) {
    // AMP 契约（sim2sim 验证）：每帧 [ang_vel(3), grav(3), cmd(3),
    // jpos_rel(n), jvel(n), last_act(n)]，原始值不缩放（归一化器融合在
    // ONNX 图内）；无相位时钟项；指令限幅到训练范围。
    // 对齐 sim2sim: settle 期间只清动作、不清指令（mujoco_rollout 的 cmd 全程恒定）。
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    if (amp_conf_.use_constant_cmd) {
      // 验收复刻模式: 恒定指令(如 [1.0,0,0]), 外部 /cmd_vel_limiter 与手柄速度忽略
      vx = amp_conf_.const_cmd_x;
      vy = amp_conf_.const_cmd_y;
      wz = amp_conf_.const_cmd_z;
    } else {
      std::shared_lock<std::shared_mutex> lock(joy_mutex_);
      vx = joy_data_.linear.x;
      vy = joy_data_.linear.y;
      wz = joy_data_.angular.z;
      const int64_t now_ns = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
      const int64_t cmd_ns = last_cmd_ns_.load(std::memory_order_acquire);
      if (cmd_ns == 0 ||
          now_ns - cmd_ns > static_cast<int64_t>(amp_conf_.cmd_timeout_ms) * 1'000'000) {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
      }
    }
    vx = std::max(amp_conf_.cmd_x_min, std::min(amp_conf_.cmd_x_max, vx));
    vy = std::max(amp_conf_.cmd_y_min, std::min(amp_conf_.cmd_y_max, vy));
    wz = std::max(amp_conf_.cmd_yaw_min, std::min(amp_conf_.cmd_yaw_max, wz));
    obs_cmd_linear_x_ = vx;
    obs_cmd_linear_y_ = vy;
    obs_cmd_angular_z_ = wz;

    propri_obs << propri_.base_ang_vel,
        propri_.projected_gravity,
        vx, vy, wz,
        (propri_.joint_pos - joint_conf_.init_state),
        propri_.joint_vel,
        last_actions_;
  } else {
    std::shared_lock<std::shared_mutex> lock(joy_mutex_);
    double phase = duration<double>(high_resolution_clock::now().time_since_epoch()).count();
    if (walk_step_conf_.sw_mode) {
      const double cmd_norm =
          std::sqrt(Square(joy_data_.linear.x) + Square(joy_data_.linear.y) + Square(joy_data_.angular.z));
      if (cmd_norm <= walk_step_conf_.cmd_threshold) {
        phase = 0;
      }
    }
    phase = phase / walk_step_conf_.cycle_time;

    obs_phase_sin_ = std::sin(2 * M_PI * phase);
    obs_phase_cos_ = std::cos(2 * M_PI * phase);
    obs_cmd_linear_x_ = joy_data_.linear.x;
    obs_cmd_linear_y_ = joy_data_.linear.y;
    obs_cmd_angular_z_ = joy_data_.angular.z;

    propri_obs << obs_phase_sin_,
        obs_phase_cos_,
        joy_data_.linear.x * obs_scales_.lin_vel,
        joy_data_.linear.y * obs_scales_.lin_vel,
        joy_data_.angular.z,
        (propri_.joint_pos - joint_conf_.init_state) * obs_scales_.dof_pos,
        propri_.joint_vel * obs_scales_.dof_vel,
        last_actions_,
        propri_.base_ang_vel * obs_scales_.ang_vel,
        propri_.base_euler_xyz * obs_scales_.quat;
  }

  if (is_first_frame_) {
    for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
      if (lpf_conf_.paralle_list.find(joint_names_[ii]) == lpf_conf_.paralle_list.end()) {
        if (lpf_conf_.position_enabled) {
          low_pass_filters_[ii].init(propri_.joint_pos[ii]);
        }
      } else {
        if (lpf_conf_.effort_enabled) {
          low_pass_filters_[ii].init(0);
        }
      }
    }

    if (amp_mode_) {
      // 参考实现（mujoco_rollout / deploy_onnx_rt）：历史零初始化，
      // 首帧仅入当前观测 —— 与 Isaac env reset 行为一致
      propri_history_buffer_.setZero();
    } else {
      for (int ii = 5 + onnx_conf_.actions_size * 2; ii < 5 + onnx_conf_.actions_size * 3; ++ii) {
        propri_obs(ii, 0) = 0.0;
      }

      for (int ii = 0; ii < onnx_conf_.num_hist; ++ii) {
        propri_history_buffer_.segment(ii * onnx_conf_.observations_size, onnx_conf_.observations_size) =
            propri_obs.cast<float>();
      }
    }
    is_first_frame_ = false;
  }

  propri_history_buffer_.head(propri_history_buffer_.size() - onnx_conf_.observations_size) =
      propri_history_buffer_.tail(propri_history_buffer_.size() - onnx_conf_.observations_size);
  propri_history_buffer_.tail(onnx_conf_.observations_size) = propri_obs.cast<float>();

  if (amp_mode_) {
    // term-major 展开（IsaacLab obs group 历史展平方式，x1_29_AMP v26
    // 根因修复点）：每个观测项的 num_hist 帧 oldest-first 拼接，项间顺序
    // = 帧内布局 [ang, grav, cmd, jpos, jvel, act]。帧存储仍是 frame-major
    // （上面整体移位），此处仅做重排拷贝。
    const int n = onnx_conf_.actions_size;
    const int frame = onnx_conf_.observations_size;
    const int term_bounds[7] = {0, 3, 6, 9, 9 + n, 9 + 2 * n, 9 + 3 * n};
    int idx = 0;
    for (int t = 0; t < 6; ++t) {
      for (int f = 0; f < onnx_conf_.num_hist; ++f) {
        for (int k = term_bounds[t]; k < term_bounds[t + 1]; ++k) {
          observations_[idx++] =
              static_cast<float>(propri_history_buffer_[static_cast<int64_t>(f) * frame + k]);
        }
      }
    }
  } else {
    for (int ii = 0; ii < onnx_conf_.observations_size * onnx_conf_.num_hist; ++ii) {
      observations_[ii] = static_cast<float>(propri_history_buffer_[ii]);
    }
  }

  const scalar_t obs_min = -onnx_conf_.observations_clip;
  const scalar_t obs_max = onnx_conf_.observations_clip;
  std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                 [obs_min, obs_max](scalar_t x) { return std::max(obs_min, std::min(obs_max, x)); });
}

void RLController::ComputeActions() {
  std::vector<Ort::Value> input_tensor;
  input_tensor.push_back(Ort::Value::CreateTensor<float>(
      memory_info_, observations_.data(), observations_.size(), input_shapes_[0].data(), input_shapes_[0].size()));

  std::vector<Ort::Value> output_values =
      session_ptr_->Run(Ort::RunOptions{}, input_names_.data(), input_tensor.data(), 1, output_names_.data(), 1);

  for (int i = 0; i < onnx_conf_.actions_size; ++i) {
    actions_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
    if (!std::isfinite(actions_[i])) {
      std::fill(actions_.begin(), actions_.end(), 0.0f);
      amp_fault_.store(true, std::memory_order_release);
      return;
    }
  }

  const scalar_t action_min = -onnx_conf_.actions_clip;
  const scalar_t action_max = onnx_conf_.actions_clip;
  std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                 [action_min, action_max](scalar_t x) { return std::max(action_min, std::min(action_max, x)); });
}

bool RLController::EnqueueDiagFrame() {
  const size_t write_idx = diag_write_idx_.load(std::memory_order_relaxed);
  const size_t next_idx = (write_idx + 1) % diag_ring_.size();
  if (next_idx == diag_read_idx_.load(std::memory_order_acquire)) {
    diag_dropped_count_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  DiagFrame& frame = diag_ring_[write_idx];
  frame.timestamp_ns = duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
  frame.phase_sin = obs_phase_sin_;
  frame.phase_cos = obs_phase_cos_;
  frame.cmd_linear_x = obs_cmd_linear_x_;
  frame.cmd_linear_y = obs_cmd_linear_y_;
  frame.cmd_angular_z = obs_cmd_angular_z_;
  frame.base_euler_x = propri_.base_euler_xyz(0);
  frame.base_euler_y = propri_.base_euler_xyz(1);
  frame.base_euler_z = propri_.base_euler_xyz(2);
  frame.base_ang_vel_x = propri_.base_ang_vel(0);
  frame.base_ang_vel_y = propri_.base_ang_vel(1);
  frame.base_ang_vel_z = propri_.base_ang_vel(2);
  frame.clip_count = 0;

  for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
    frame.actions[ii] = actions_[ii];
    frame.joint_pos[ii] = propri_.joint_pos(ii);
    frame.joint_vel[ii] = propri_.joint_vel(ii);
    frame.joint_effort[ii] = propri_.joint_effort(ii);
    frame.pos_des_raw[ii] = pd_pos_des_raw_[ii];
    frame.pos_des_lpf[ii] = pd_pos_des_lpf_[ii];
    frame.tau_des_raw[ii] = pd_tau_des_raw_[ii];
    frame.tau_des_lpf[ii] = pd_tau_des_lpf_[ii];
    frame.is_parallel[ii] = pd_is_parallel_[ii];
    if (std::abs(actions_[ii]) >= onnx_conf_.actions_clip - 1e-6) {
      ++frame.clip_count;
    }
  }

  frame.imu_quat_w = propri_.imu_quat.w();
  frame.imu_quat_x = propri_.imu_quat.x();
  frame.imu_quat_y = propri_.imu_quat.y();
  frame.imu_quat_z = propri_.imu_quat.z();
  frame.imu_gyro_x = propri_.base_ang_vel(0);
  frame.imu_gyro_y = propri_.base_ang_vel(1);
  frame.imu_gyro_z = propri_.base_ang_vel(2);
  frame.imu_accel_x = propri_.imu_accel(0);
  frame.imu_accel_y = propri_.imu_accel(1);
  frame.imu_accel_z = propri_.imu_accel(2);

  {
    std::shared_lock<std::shared_mutex> lock(actuator_state_mutex_);
    frame.actuator_names = actuator_names_;
    const size_t n = actuator_names_.size();
    frame.actuator_cmd_effort.assign(n, std::numeric_limits<double>::quiet_NaN());
    frame.actuator_state_effort.assign(n, std::numeric_limits<double>::quiet_NaN());
    for (size_t ii = 0; ii < n; ++ii) {
      if (ii < actuator_cmd_data_.effort.size()) {
        frame.actuator_cmd_effort[ii] = actuator_cmd_data_.effort[ii];
      }
      if (ii < actuator_state_data_.effort.size()) {
        frame.actuator_state_effort[ii] = actuator_state_data_.effort[ii];
      }
    }
  }

  diag_last_enqueue_ns_.store(frame.timestamp_ns, std::memory_order_release);
  diag_write_idx_.store(next_idx, std::memory_order_release);
  return true;
}

bool RLController::EnqueueTmFrame() {
  const size_t write_idx = tm_write_idx_.load(std::memory_order_relaxed);
  const size_t next_idx = (write_idx + 1) % tm_ring_.size();
  if (next_idx == tm_read_idx_.load(std::memory_order_acquire)) {
    tm_dropped_count_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  TmFrame& frame = tm_ring_[write_idx];
  std::copy(observations_.begin(), observations_.end(), frame.observations.begin());
  tm_last_enqueue_ns_.store(
      duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count(),
      std::memory_order_release);
  tm_write_idx_.store(next_idx, std::memory_order_release);
  return true;
}

void RLController::FinalizeDiagLogging(const char* reason) {
  if (!diag_logging_triggered_) {
    return;
  }

  const bool flush_ok = diag_logger_.Flush();
  const bool close_ok = diag_logger_.Close();
  diag_logging_triggered_ = false;
  diag_logging_requested_.store(false, std::memory_order_release);
  diag_file_actuator_names_.clear();
  const std::string_view reason_view(reason);
  const bool io_error = reason_view == "write_error" || reason_view == "flush_error";
  if (flush_ok && close_ok && !io_error) {
    AIMRT_INFO("walk_diag logging finished, reason={}, frames={}, dropped={}",
               reason,
               diag_log_count_,
               diag_dropped_count_.load(std::memory_order_relaxed));
  } else {
    AIMRT_ERROR("walk_diag logging incomplete, reason={}, frames={}, dropped={}, flush_ok={}, close_ok={}",
                reason,
                diag_log_count_,
                diag_dropped_count_.load(std::memory_order_relaxed),
                flush_ok,
                close_ok);
  }
}

void RLController::FinalizeTmLogging(const char* reason) {
  if (!tm_logging_triggered_) {
    return;
  }

  const bool flush_ok = tm_logger_.Flush();
  const bool close_ok = tm_logger_.Close();
  tm_logging_triggered_ = false;
  tm_logging_requested_.store(false, std::memory_order_release);
  const std::string_view reason_view(reason);
  const bool io_error = reason_view == "write_error" || reason_view == "flush_error";
  if (flush_ok && close_ok && !io_error) {
    AIMRT_INFO("tm_obs_input logging finished, reason={}, frames={}, dropped={}",
               reason,
               tm_log_count_,
               tm_dropped_count_.load(std::memory_order_relaxed));
  } else {
    AIMRT_ERROR("tm_obs_input logging incomplete, reason={}, frames={}, dropped={}, flush_ok={}, close_ok={}",
                reason,
                tm_log_count_,
                tm_dropped_count_.load(std::memory_order_relaxed),
                flush_ok,
                close_ok);
  }
}

void RLController::DrainDiagBuffer() {
  while (diag_read_idx_.load(std::memory_order_relaxed) != diag_write_idx_.load(std::memory_order_acquire)) {
    const size_t read_idx = diag_read_idx_.load(std::memory_order_relaxed);
    const DiagFrame& frame = diag_ring_[read_idx];

    const bool logging_requested = diag_logging_requested_.load(std::memory_order_acquire);
    if (logging_requested && !diag_logging_triggered_) {
      const std::string diag_path = diag_log_dir_ + "/walk_diag_" + MakeTimestampString() + ".csv";
      if (!diag_logger_.Open(diag_path, false, true)) {
        AIMRT_ERROR("Failed to open walk_diag log file: {}", diag_path);
        diag_logging_requested_.store(false, std::memory_order_release);
      } else {
        std::ostringstream header;
        header << "timestamp_ns,phase_sin,phase_cos"
               << ",cmd_linear_x,cmd_linear_y,cmd_angular_z"
               << ",base_euler_x,base_euler_y,base_euler_z"
               << ",base_ang_vel_x,base_ang_vel_y,base_ang_vel_z";
        for (const auto& name : joint_names_) {
          header << ",action_" << name
                 << ",pos_" << name
                 << ",vel_" << name
                 << ",effort_" << name
                 << ",pos_des_raw_" << name
                 << ",pos_des_lpf_" << name
                 << ",tau_des_raw_" << name
                 << ",tau_des_lpf_" << name
                 << ",is_parallel_" << name;
        }
        header << ",clip_count"
               << ",imu_quat_w,imu_quat_x,imu_quat_y,imu_quat_z"
               << ",imu_gyro_x,imu_gyro_y,imu_gyro_z"
               << ",imu_accel_x,imu_accel_y,imu_accel_z";
        {
          std::shared_lock<std::shared_mutex> lock(actuator_state_mutex_);
          diag_file_actuator_names_.clear();
          for (const auto& name : actuator_names_) {
            if (IsLegActuator(name)) {
              diag_file_actuator_names_.push_back(name);
            }
          }
        }
        for (const auto& name : diag_file_actuator_names_) {
          header << ",actuator_cmd_effort_" << name
                 << ",actuator_state_effort_" << name;
        }
        if (!diag_logger_.WriteTextLine(header.str())) {
          const bool close_ok = diag_logger_.Close();
          diag_logging_requested_.store(false, std::memory_order_release);
          diag_file_actuator_names_.clear();
          AIMRT_ERROR("Failed to write walk_diag header: {}, close_ok={}", diag_path, close_ok);
        } else {
          diag_logging_triggered_ = true;
          diag_log_count_ = 0;
          diag_dropped_count_.store(0, std::memory_order_relaxed);
          AIMRT_INFO("walk_diag logging triggered: {} ({} leg actuator torque cols)",
                     diag_path, diag_file_actuator_names_.size());
        }
      }
    }

    if (diag_logging_triggered_ && diag_log_count_ < diag_log_max_count_) {
      std::ostringstream row;
      row << frame.timestamp_ns
          << "," << frame.phase_sin
          << "," << frame.phase_cos
          << "," << frame.cmd_linear_x
          << "," << frame.cmd_linear_y
          << "," << frame.cmd_angular_z
          << "," << frame.base_euler_x
          << "," << frame.base_euler_y
          << "," << frame.base_euler_z
          << "," << frame.base_ang_vel_x
          << "," << frame.base_ang_vel_y
          << "," << frame.base_ang_vel_z;

      for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
        row << "," << frame.actions[ii]
            << "," << frame.joint_pos[ii]
            << "," << frame.joint_vel[ii]
            << "," << frame.joint_effort[ii]
            << "," << frame.pos_des_raw[ii]
            << "," << frame.pos_des_lpf[ii]
            << "," << frame.tau_des_raw[ii]
            << "," << frame.tau_des_lpf[ii]
            << "," << frame.is_parallel[ii];
      }

      row << "," << frame.clip_count
          << "," << frame.imu_quat_w
          << "," << frame.imu_quat_x
          << "," << frame.imu_quat_y
          << "," << frame.imu_quat_z
          << "," << frame.imu_gyro_x
          << "," << frame.imu_gyro_y
          << "," << frame.imu_gyro_z
          << "," << frame.imu_accel_x
          << "," << frame.imu_accel_y
          << "," << frame.imu_accel_z;
      for (const auto& name : diag_file_actuator_names_) {
        double cmd_effort = 0.0;  // 串联默认 0
        double state_effort = std::numeric_limits<double>::quiet_NaN();
        for (size_t ai = 0; ai < frame.actuator_names.size(); ++ai) {
          if (frame.actuator_names[ai] != name) {
            continue;
          }
          // 仅并联踝电机保留真实 cmd effort；串联强制 0
          if (IsParallelLegActuator(name) && ai < frame.actuator_cmd_effort.size()) {
            cmd_effort = frame.actuator_cmd_effort[ai];
          }
          if (ai < frame.actuator_state_effort.size()) {
            state_effort = frame.actuator_state_effort[ai];
          }
          break;
        }
        row << "," << cmd_effort << "," << state_effort;
      }
      if (!diag_logger_.WriteTextLine(row.str())) {
        FinalizeDiagLogging("write_error");
      } else {
        ++diag_log_count_;
      }

      if (diag_logging_triggered_ && diag_log_count_ >= diag_log_max_count_) {
        FinalizeDiagLogging("frame_limit");
      }
    }

    diag_read_idx_.store((read_idx + 1) % diag_ring_.size(), std::memory_order_release);
  }
}

void RLController::DrainTmBuffer() {
  while (tm_read_idx_.load(std::memory_order_relaxed) != tm_write_idx_.load(std::memory_order_acquire)) {
    const size_t read_idx = tm_read_idx_.load(std::memory_order_relaxed);
    const TmFrame& frame = tm_ring_[read_idx];

    const bool logging_requested = tm_logging_requested_.load(std::memory_order_acquire);
    if (logging_requested && !tm_logging_triggered_) {
      const std::string bin_path = tm_log_dir_ + "/tm_obs_input_" + MakeTimestampString() + ".bin";
      if (!tm_logger_.Open(bin_path, true, false)) {
        AIMRT_ERROR("Failed to open tm_obs_input log file: {}", bin_path);
        tm_logging_requested_.store(false, std::memory_order_release);
      } else {
        tm_logging_triggered_ = true;
        tm_log_count_ = 0;
        tm_dropped_count_.store(0, std::memory_order_relaxed);
        AIMRT_INFO("tm_obs_input logging triggered: {}", bin_path);
      }
    }

    if (tm_logging_triggered_ && tm_log_count_ < tm_log_max_count_) {
      if (!tm_logger_.WriteRaw(frame.observations.data(), frame.observations.size() * sizeof(float))) {
        FinalizeTmLogging("write_error");
      } else {
        ++tm_log_count_;
      }
      if (tm_logging_triggered_ && tm_log_count_ >= tm_log_max_count_) {
        FinalizeTmLogging("frame_limit");
      }
    }

    tm_read_idx_.store((read_idx + 1) % tm_ring_.size(), std::memory_order_release);
  }
}

}  // namespace xyber_x1_infer::rl_control_module
