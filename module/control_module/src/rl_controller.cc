#include "control_module/rl_controller.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <sstream>

#include "control_module/global.h"

namespace xyber_x1_infer::rl_control_module {

namespace {

constexpr size_t kDiagRingCapacity = 256;
constexpr size_t kTmRingCapacity = 512;
constexpr auto kLogWorkerPollInterval = std::chrono::milliseconds(1);
constexpr auto kLogIdleTimeout = std::chrono::milliseconds(500);
constexpr auto kLogFlushInterval = std::chrono::milliseconds(200);

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
  {
    const auto& ws = cfg_node["walk_step_conf"];
    walk_step_conf_.adaptive_cycle = ws["adaptive_cycle"] ? ws["adaptive_cycle"].as<bool>() : false;
    walk_step_conf_.cycle_time_min =
        ws["cycle_time_min"] ? ws["cycle_time_min"].as<double>() : walk_step_conf_.cycle_time;
    walk_step_conf_.cycle_time_max =
        ws["cycle_time_max"] ? ws["cycle_time_max"].as<double>() : walk_step_conf_.cycle_time;
    walk_step_conf_.cycle_speed_max = ws["cycle_speed_max"] ? ws["cycle_speed_max"].as<double>() : 0.6;
    walk_step_conf_.ema_tau = ws["cycle_speed_ema_tau"] ? ws["cycle_speed_ema_tau"].as<double>() : 0.5;
    AIMRT_CHECK_ERROR_THROW(walk_step_conf_.ema_tau > 0.0, "cycle_speed_ema_tau must be > 0.");
    AIMRT_CHECK_ERROR_THROW(walk_step_conf_.cycle_speed_max > 0.0, "cycle_speed_max must be > 0.");
    walk_step_conf_.cycle_time_min = std::min(walk_step_conf_.cycle_time_min, walk_step_conf_.cycle_time_max);
    cur_cycle_time_ = walk_step_conf_.adaptive_cycle ? walk_step_conf_.cycle_time_min : walk_step_conf_.cycle_time;
    // 推理周期 = decimation / 控制频率（本模块主循环按 1kHz 设计）
    policy_dt_ = walk_step_conf_.decimation / 1000.0;
  }
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

  // 分速度段模型（可选）：配置 stages 后启用多模型切换，忽略单模型字段
  const auto& stages_node = cfg_node["onnx_conf"]["stages"];
  if (stages_node && stages_node.IsSequence() && stages_node.size() > 0) {
    multi_stage_ = true;
    const auto& sw = cfg_node["onnx_conf"]["switch_conf"];
    if (sw) {
      switch_conf_.low_norm = sw["low_norm"] ? sw["low_norm"].as<double>() : switch_conf_.low_norm;
      switch_conf_.low_norm_hyst = sw["low_norm_hyst"] ? sw["low_norm_hyst"].as<double>() : switch_conf_.low_norm_hyst;
      switch_conf_.still_norm = sw["still_norm"] ? sw["still_norm"].as<double>() : switch_conf_.still_norm;
      switch_conf_.still_window_s = sw["still_window_s"] ? sw["still_window_s"].as<double>() : switch_conf_.still_window_s;
      switch_conf_.still_ratio = sw["still_ratio"] ? sw["still_ratio"].as<double>() : switch_conf_.still_ratio;
      switch_conf_.min_dwell_s = sw["min_dwell_s"] ? sw["min_dwell_s"].as<double>() : switch_conf_.min_dwell_s;
    }
    AIMRT_CHECK_ERROR_THROW(switch_conf_.low_norm > switch_conf_.low_norm_hyst,
                            "switch_conf.low_norm({}) must be > low_norm_hyst({}).",
                            switch_conf_.low_norm, switch_conf_.low_norm_hyst);
    AIMRT_CHECK_ERROR_THROW(switch_conf_.still_window_s > 0.0, "still_window_s must be > 0.");
    AIMRT_CHECK_ERROR_THROW(switch_conf_.still_ratio > 0.0 && switch_conf_.still_ratio <= 1.0,
                            "still_ratio must be in (0, 1].");

    for (const auto& st : stages_node) {
      OnnxStage stage;
      stage.name = st["name"].as<std::string>();
      stage.policy_file = st["policy_file"].as<std::string>();
      stage.actions_size = st["actions_size"].as<int32_t>();
      stage.observations_size = st["observations_size"].as<int32_t>();
      stage.num_hist = st["num_hist"].as<int32_t>();
      stage.observations_clip = st["observations_clip"] ? st["observations_clip"].as<double>() : onnx_conf_.observations_clip;
      stage.actions_clip = st["actions_clip"] ? st["actions_clip"].as<double>() : onnx_conf_.actions_clip;
      // stage 覆盖 PD 增益（未配置则用全局）
      if (st["stiffness"] && st["damping"]) {
        stage.stiffness = Eigen::Map<vector_t>(st["stiffness"].as<std::vector<double>>().data(),
                                               st["stiffness"].as<std::vector<double>>().size());
        stage.damping = Eigen::Map<vector_t>(st["damping"].as<std::vector<double>>().data(),
                                             st["damping"].as<std::vector<double>>().size());
        AIMRT_CHECK_ERROR_THROW(stage.stiffness.size() == static_cast<Eigen::Index>(stage.actions_size) &&
                                    stage.damping.size() == static_cast<Eigen::Index>(stage.actions_size),
                                "stage '{}' stiffness/damping size mismatch with actions_size.", stage.name);
        stage.has_pd = true;
      }
      // stage 覆盖自适应周期映射（未配置则用全局 walk_step_conf）
      if (st["cycle_time_min"] && st["cycle_time_max"]) {
        stage.cycle_time_min = st["cycle_time_min"].as<double>();
        stage.cycle_time_max = st["cycle_time_max"].as<double>();
        stage.cycle_time_min = std::min(stage.cycle_time_min, stage.cycle_time_max);
        stage.has_cycle = true;
      }
      // stage 覆盖站立冻结开关（低速模型 sw_switch=False：零命令相位连续推进=原地踏步）
      if (st["sw_mode"]) {
        stage.sw_mode = st["sw_mode"].as<bool>();
        stage.has_sw_mode = true;
      }
      // stage 命令限幅（观测用，对齐训练命令范围）
      if (st["cmd_limit_x"] && st["cmd_limit_y"]) {
        stage.cmd_limit_x = st["cmd_limit_x"].as<double>();
        stage.cmd_limit_y = st["cmd_limit_y"].as<double>();
        AIMRT_CHECK_ERROR_THROW(stage.cmd_limit_x > 0.0 && stage.cmd_limit_y > 0.0,
                                "stage '{}' cmd_limit_x/y must be > 0.", stage.name);
        stage.has_cmd_limit = true;
      }
      // 切换到该 stage 时的一次性相位偏移（0.5 = 半周期，纠正两模型迈脚语义差异）
      if (st["phase_offset"]) {
        stage.phase_offset = st["phase_offset"].as<double>();
      }
      stages_.push_back(std::move(stage));
    }
    AIMRT_CHECK_ERROR_THROW(stages_.size() >= 2, "onnx_conf.stages needs at least 2 stages.");
    // 校验 stage 间观测布局同构（切换时共享观测历史的前提）
    for (size_t ii = 1; ii < stages_.size(); ++ii) {
      AIMRT_CHECK_ERROR_THROW(
          stages_[ii].observations_size == stages_[0].observations_size &&
              stages_[ii].num_hist == stages_[0].num_hist &&
              stages_[ii].actions_size == stages_[0].actions_size,
          "stage '{}' obs layout differs from '{}': multi-stage requires identical "
          "observations_size/num_hist/actions_size (PD/cycle may differ).",
          stages_[ii].name, stages_[0].name);
    }
    // 静止窗口缓冲（推理拍 100Hz）
    const size_t window_n = std::max<size_t>(
        1, static_cast<size_t>(switch_conf_.still_window_s / policy_dt_ + 0.5));
    still_window_.assign(window_n, false);
    last_switch_time_ = high_resolution_clock::now() - std::chrono::duration_cast<nanoseconds>(
                                                          std::chrono::duration<double>(switch_conf_.min_dwell_s));
    AIMRT_INFO("RLController multi-stage enabled: {} stages, low_norm={}, hyst={}, still={}/{}s ratio={}, dwell={}s",
               stages_.size(), switch_conf_.low_norm, switch_conf_.low_norm_hyst, switch_conf_.still_norm,
               switch_conf_.still_window_s, switch_conf_.still_ratio, switch_conf_.min_dwell_s);
  }

  lpf_conf_.wc = cfg_node["lpf_conf"]["wc"].as<double>();
  lpf_conf_.ts = cfg_node["lpf_conf"]["ts"].as<double>();
  auto paralle_list = cfg_node["lpf_conf"]["paralle_list"].as<std::vector<std::string>>();
  lpf_conf_.paralle_list = std::set<std::string>(paralle_list.begin(), paralle_list.end());

  if (multi_stage_) {
    LoadStageModels();
    // 观测缓冲按首个 stage 布局分配（已校验同构）
    onnx_conf_.observations_size = stages_[0].observations_size;
    onnx_conf_.num_hist = stages_[0].num_hist;
    onnx_conf_.actions_size = stages_[0].actions_size;
  } else {
    LoadModel();
  }

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
  propri_history_buffer_.resize(onnx_conf_.observations_size * onnx_conf_.num_hist);
  low_pass_filters_.clear();
  for (int i = 0; i < onnx_conf_.actions_size; ++i) {
    low_pass_filters_.emplace_back(lpf_conf_.wc, lpf_conf_.ts);
  }
}

void RLController::RestartController() {
  is_first_frame_ = true;
  diag_pending_frame_ = false;
  phase_step_count_ = 0;
  phase_accum_ = 0.0;
  smoothed_speed_ = 0.0;
  if (multi_stage_) {
    active_stage_idx_ = 0;  // 重启回低速模型
    std::fill(still_window_.begin(), still_window_.end(), false);
    still_window_idx_ = 0;
    still_window_filled_ = false;
  }
}

void RLController::SetWalkLegEntered(bool entered) {
  diag_walk_entered_.store(entered, std::memory_order_release);
  tm_walk_entered_.store(entered, std::memory_order_release);
}

void RLController::SetLogExecutor(aimrt::executor::ExecutorRef executor) {
  log_executor_ = executor;
  StartLoggingWorker();
}

void RLController::StopLoggingWorker() {
  log_worker_running_.store(false, std::memory_order_release);
  while (!log_worker_stopped_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void RLController::StartLoggingWorker() {
  if (!log_executor_ || log_worker_started_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }

  log_worker_stopped_.store(false, std::memory_order_release);
  log_worker_running_.store(true, std::memory_order_release);
  log_executor_.Execute([this]() { LoggingWorkerLoop(); });
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
        diag_logger_.Flush();
        last_diag_flush = now;
      }
    }

    if (tm_logging_triggered_) {
      const auto last_enqueue_ns = tm_last_enqueue_ns_.load(std::memory_order_acquire);
      if (last_enqueue_ns > 0 &&
          std::chrono::nanoseconds(now_ns - last_enqueue_ns) >= kLogIdleTimeout) {
        FinalizeTmLogging("idle_timeout");
      } else if (now - last_tm_flush >= kLogFlushInterval) {
        tm_logger_.Flush();
        last_tm_flush = now;
      }
    }

    std::this_thread::sleep_for(kLogWorkerPollInterval);
  }

  DrainDiagBuffer();
  DrainTmBuffer();
  FinalizeDiagLogging("worker_stop");
  FinalizeTmLogging("worker_stop");
  log_worker_stopped_.store(true, std::memory_order_release);
}

void RLController::Update() {
  UpdateStateEstimation();

  if (loop_count_ % walk_step_conf_.decimation == 0) {
    ComputeObservation();
    ComputeActions();

    if (diag_logging_enabled_) {
      diag_pending_frame_ = true;
    }

    if (tm_logging_enabled_) {
      (void)EnqueueTmFrame();
    }
  }

  loop_count_++;
}

my_ros2_proto::msg::JointCommand RLController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.resize(joint_names_.size());
  joint_cmd.velocity.resize(joint_names_.size());
  joint_cmd.effort.resize(joint_names_.size());
  joint_cmd.damping.resize(joint_names_.size());
  joint_cmd.stiffness.resize(joint_names_.size());

  for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
    const scalar_t pos_des_raw = actions_[ii] * walk_step_conf_.action_scale + joint_conf_.init_state(ii);
    scalar_t pos_des = pos_des_raw;
    // PD 增益按 stage 覆盖（未配置则用全局）
    const OnnxStage* stage = multi_stage_ ? &stages_[active_stage_idx_] : nullptr;
    const double stiffness =
        (stage && stage->has_pd) ? stage->stiffness(ii) : joint_conf_.stiffness(ii);
    const double damping = (stage && stage->has_pd) ? stage->damping(ii) : joint_conf_.damping(ii);
    pd_pos_des_raw_[ii] = pos_des_raw;
    pd_pos_des_lpf_[ii] = std::numeric_limits<double>::quiet_NaN();
    pd_tau_des_raw_[ii] = std::numeric_limits<double>::quiet_NaN();
    pd_tau_des_lpf_[ii] = std::numeric_limits<double>::quiet_NaN();

    // pos_des = std::max(static_cast<scalar_t>(joint_conf_.pos_limit_lower(ii)),
    //                    std::min(static_cast<scalar_t>(joint_conf_.pos_limit_upper(ii)), pos_des));

    if (lpf_conf_.paralle_list.find(joint_names_[ii]) == lpf_conf_.paralle_list.end()) {
      pd_is_parallel_[ii] = 0;
      low_pass_filters_[ii].input(pos_des);
      const double pos_des_lp = low_pass_filters_[ii].output();
      pd_pos_des_lpf_[ii] = pos_des_lp;
      joint_cmd.position[ii] = pos_des_lp;
      joint_cmd.velocity[ii] = 0.0;
      joint_cmd.effort[ii] = 0.0;
      joint_cmd.stiffness[ii] = stiffness;
      joint_cmd.damping[ii] = damping;
    } else {
      pd_is_parallel_[ii] = 1;
      const double tau_des = stiffness * (pos_des - propri_.joint_pos[ii]) + damping * (0.0 - propri_.joint_vel[ii]);
      pd_tau_des_raw_[ii] = tau_des;
      low_pass_filters_[ii].input(tau_des);
      const double tau_des_lp = low_pass_filters_[ii].output();
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

  return joint_cmd;
}

void RLController::LoadModel() {
  std::shared_ptr<Ort::Env> onnxEnvPrt(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetInterOpNumThreads(1);
  session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt, onnx_conf_.policy_file.c_str(), sessionOptions);

  input_names_.clear();
  output_names_.clear();
  input_shapes_.clear();
  output_shapes_.clear();

  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t ii = 0; ii < session_ptr_->GetInputCount(); ++ii) {
    char* tempstring = new char[std::strlen(session_ptr_->GetInputNameAllocated(ii, allocator).get()) + 1];
    std::strcpy(tempstring, session_ptr_->GetInputNameAllocated(ii, allocator).get());
    input_names_.push_back(tempstring);
    input_shapes_.push_back(session_ptr_->GetInputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }

  for (size_t ii = 0; ii < session_ptr_->GetOutputCount(); ++ii) {
    char* tempstring = new char[std::strlen(session_ptr_->GetOutputNameAllocated(ii, allocator).get()) + 1];
    std::strcpy(tempstring, session_ptr_->GetOutputNameAllocated(ii, allocator).get());
    output_names_.push_back(tempstring);
    output_shapes_.push_back(session_ptr_->GetOutputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }
}

void RLController::LoadStageModels() {
  // Env 必须比所有 Session 长寿：提升为静态，随进程存活
  static Ort::Env onnx_env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxMultiStage");

  for (auto& stage : stages_) {
    Ort::SessionOptions session_options;
    session_options.SetInterOpNumThreads(1);
    session_options.SetIntraOpNumThreads(1);
    stage.session = std::make_unique<Ort::Session>(onnx_env, stage.policy_file.c_str(), session_options);

    stage.input_names.clear();
    stage.output_names.clear();
    stage.input_shapes.clear();
    stage.output_shapes.clear();

    Ort::AllocatorWithDefaultOptions allocator;
    for (size_t ii = 0; ii < stage.session->GetInputCount(); ++ii) {
      char* temp = new char[std::strlen(stage.session->GetInputNameAllocated(ii, allocator).get()) + 1];
      std::strcpy(temp, stage.session->GetInputNameAllocated(ii, allocator).get());
      stage.input_names.push_back(temp);
      stage.input_shapes.push_back(stage.session->GetInputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
    }
    for (size_t ii = 0; ii < stage.session->GetOutputCount(); ++ii) {
      char* temp = new char[std::strlen(stage.session->GetOutputNameAllocated(ii, allocator).get()) + 1];
      std::strcpy(temp, stage.session->GetOutputNameAllocated(ii, allocator).get());
      stage.output_names.push_back(temp);
      stage.output_shapes.push_back(stage.session->GetOutputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
    }

    stage.observations.assign(stage.observations_size * stage.num_hist, 0.0f);
    AIMRT_INFO("Stage '{}' loaded: {} (obs {}x{} act {})", stage.name, stage.policy_file,
               stage.observations_size, stage.num_hist, stage.actions_size);
  }
}

void RLController::UpdateActiveStage(double cmd_x, double cmd_y, double cmd_z) {
  const double ax = std::abs(cmd_x);
  const double ay = std::abs(cmd_y);
  const double az = std::abs(cmd_z);

  // (1) 静止窗口更新（含 ωz；滑窗累计占比）
  const bool still_now = (ax < switch_conf_.still_norm) && (ay < switch_conf_.still_norm) &&
                         (az < switch_conf_.still_norm);
  still_window_[still_window_idx_] = still_now;
  still_window_idx_ = (still_window_idx_ + 1) % still_window_.size();
  if (still_window_idx_ == 0) still_window_filled_ = true;
  const size_t filled_n = still_window_filled_ ? still_window_.size() : still_window_idx_;
  size_t still_cnt = 0;
  for (size_t ii = 0; ii < filled_n; ++ii) still_cnt += still_window_[ii] ? 1 : 0;
  const bool still_timeout =
      filled_n > 0 && static_cast<double>(still_cnt) / filled_n >= switch_conf_.still_ratio;

  // (2) 目标 stage 判定（优先级：静止超时 > 上切 > 下切保持）
  const bool is_low = (active_stage_idx_ == 0);
  size_t target = active_stage_idx_;
  if (still_timeout) {
    target = 1;  // 规则3：静止超时 → 高速模型（带 sw_mode 站立冻结，低速模型原地踏步不会站立）
  } else if (is_low) {
    if (ax > switch_conf_.low_norm || ay > switch_conf_.low_norm) {
      target = 1;  // 规则1：上切
    }
  } else {
    if (ax < switch_conf_.low_norm_hyst && ay < switch_conf_.low_norm_hyst) {
      target = 0;  // 规则2：下切（迟滞）
    }
  }

  // (3) 防抖：距上次切换不足 min_dwell_s 则不切
  if (target != active_stage_idx_) {
    const double since_switch = duration<double>(high_resolution_clock::now() - last_switch_time_).count();
    if (since_switch >= switch_conf_.min_dwell_s) {
      AIMRT_INFO("[RLStage] switch '{}' -> '{}' (cmd=({:.3f},{:.3f},{:.3f}), still={})",
                 stages_[active_stage_idx_].name, stages_[target].name, cmd_x, cmd_y, cmd_z, still_timeout);
      active_stage_idx_ = target;
      last_switch_time_ = high_resolution_clock::now();
      // 新 stage 配置了相位偏移则在下一观测帧生效（一次性）
      if (stages_[target].phase_offset != 0.0) {
        has_pending_phase_offset_ = true;
      }
    }
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
    propri_.imu_quat = quat;
    propri_.imu_accel(0) = imu_data_.linear_acceleration.x;
    propri_.imu_accel(1) = imu_data_.linear_acceleration.y;
    propri_.imu_accel(2) = imu_data_.linear_acceleration.z;
    matrix_t inverse_rot = GetRotationMatrixFromZyxEulerAngles(QuatToZyx(quat)).inverse();
    propri_.projected_gravity = inverse_rot * gravity_vector;
    propri_.base_euler_xyz = QuatToXyz(quat);
  }
}

void RLController::ComputeObservation() {
  vector_t propri_obs(onnx_conf_.observations_size);
  {
    std::shared_lock<std::shared_mutex> lock(joy_mutex_);
    // 对齐训练侧 exp1.2（x1_dh_stand_env.py / sim2sim.py）：
    // 平面指令限幅 -> EMA 平滑速度 -> 自适应周期 -> 相位积分（站立清零）
    double cmd_x = joy_data_.linear.x;
    double cmd_y = joy_data_.linear.y;
    const double cmd_z = joy_data_.angular.z;

    // 多 stage 模式：先更新活动 stage（判定用原始指令）
    if (multi_stage_) {
      UpdateActiveStage(joy_data_.linear.x, joy_data_.linear.y, joy_data_.angular.z);
    }
    const OnnxStage* stage = multi_stage_ ? &stages_[active_stage_idx_] : nullptr;

    // stage 级命令限幅（观测用，对齐该 stage 训练命令范围；未配置不限幅）
    if (stage && stage->has_cmd_limit) {
      cmd_x = std::clamp(cmd_x, -stage->cmd_limit_x, stage->cmd_limit_x);
      cmd_y = std::clamp(cmd_y, -stage->cmd_limit_y, stage->cmd_limit_y);
    }

    if (walk_step_conf_.adaptive_cycle) {
      // 平面指令限幅到 cycle_speed_max（训练命令范围上界）
      const double planar = std::sqrt(Square(cmd_x) + Square(cmd_y));
      if (planar > walk_step_conf_.cycle_speed_max && planar > 0.0) {
        const double scale = walk_step_conf_.cycle_speed_max / planar;
        cmd_x *= scale;
        cmd_y *= scale;
      }
    }

    // 站立冻结开关按 stage 覆盖（低速模型 sw_switch=False：零命令相位连续推进=原地踏步）
    const bool sw_mode =
        (stage && stage->has_sw_mode) ? stage->sw_mode : walk_step_conf_.sw_mode;
    // 站立判定（训练 sw_switch：含 yaw 的指令范数）
    const bool standing = sw_mode &&
        std::sqrt(Square(cmd_x) + Square(cmd_y) + Square(cmd_z)) <= walk_step_conf_.cmd_threshold;

    // 自适应周期参数按 stage 覆盖（未配置则用全局）
    const double cyc_min = (stage && stage->has_cycle) ? stage->cycle_time_min : walk_step_conf_.cycle_time_min;
    const double cyc_max = (stage && stage->has_cycle) ? stage->cycle_time_max : walk_step_conf_.cycle_time_max;

    if (walk_step_conf_.adaptive_cycle) {
      // EMA 平滑平面速度（仅线速度，原地转向不改变步频）
      const double target_speed = std::sqrt(Square(cmd_x) + Square(cmd_y));
      const double alpha = std::min(1.0, policy_dt_ / walk_step_conf_.ema_tau);
      smoothed_speed_ += alpha * (target_speed - smoothed_speed_);

      // 周期随速度线性伸缩：cyc_min(静止) ~ cyc_max(cycle_speed_max)
      const double ratio = std::clamp(smoothed_speed_, 0.0, walk_step_conf_.cycle_speed_max) /
                           walk_step_conf_.cycle_speed_max;
      cur_cycle_time_ = cyc_min + ratio * (cyc_max - cyc_min);

      // 相位积分累加（cycle_time 时变/跨 stage 切换均连续）
      if (standing) {
        phase_accum_ = 0.0;
        phase_step_count_ = 0;
      } else {
        phase_accum_ += policy_dt_ / cur_cycle_time_;
        ++phase_step_count_;
      }
    } else {
      // 固定周期模式：相位改为累加器（修复墙钟相位跳变），站立清零
      cur_cycle_time_ = (stage && stage->has_cycle) ? stage->cycle_time_max : walk_step_conf_.cycle_time;
      if (standing) {
        phase_accum_ = 0.0;
        phase_step_count_ = 0;
      } else {
        phase_accum_ += policy_dt_ / cur_cycle_time_;
        ++phase_step_count_;
      }
    }

    // 切换后一次性相位偏移（纠正两 stage 迈脚语义差异；0.5=半周期交换左右脚）
    if (multi_stage_ && has_pending_phase_offset_) {
      phase_accum_ += stages_[active_stage_idx_].phase_offset;
      has_pending_phase_offset_ = false;
      AIMRT_INFO("[RLStage] phase offset {} applied to '{}'", stages_[active_stage_idx_].phase_offset,
                 stages_[active_stage_idx_].name);
    }

    obs_phase_sin_ = std::sin(2 * M_PI * phase_accum_);
    obs_phase_cos_ = std::cos(2 * M_PI * phase_accum_);
    obs_cmd_linear_x_ = cmd_x;
    obs_cmd_linear_y_ = cmd_y;
    obs_cmd_angular_z_ = cmd_z;

    propri_obs << obs_phase_sin_,
        obs_phase_cos_,
        cmd_x * obs_scales_.lin_vel,
        cmd_y * obs_scales_.lin_vel,
        cmd_z,
        (propri_.joint_pos - joint_conf_.init_state) * obs_scales_.dof_pos,
        propri_.joint_vel * obs_scales_.dof_vel,
        last_actions_,
        propri_.base_ang_vel * obs_scales_.ang_vel,
        propri_.base_euler_xyz * obs_scales_.quat;
  }

  if (is_first_frame_) {
    for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
      if (lpf_conf_.paralle_list.find(joint_names_[ii]) == lpf_conf_.paralle_list.end()) {
        low_pass_filters_[ii].init(propri_.joint_pos[ii]);
      } else {
        low_pass_filters_[ii].init(0);
      }
    }

    for (int ii = 5 + onnx_conf_.actions_size * 2; ii < 5 + onnx_conf_.actions_size * 3; ++ii) {
      propri_obs(ii, 0) = 0.0;
    }

    for (int ii = 0; ii < onnx_conf_.num_hist; ++ii) {
      propri_history_buffer_.segment(ii * onnx_conf_.observations_size, onnx_conf_.observations_size) =
          propri_obs.cast<float>();
    }
    is_first_frame_ = false;
  }

  propri_history_buffer_.head(propri_history_buffer_.size() - onnx_conf_.observations_size) =
      propri_history_buffer_.tail(propri_history_buffer_.size() - onnx_conf_.observations_size);
  propri_history_buffer_.tail(onnx_conf_.observations_size) = propri_obs.cast<float>();

  if (multi_stage_) {
    // 各 stage 独立 clip 后写入各自观测缓冲（历史帧共享，clip 参数可不同）
    OnnxStage& stage = stages_[active_stage_idx_];
    for (int ii = 0; ii < onnx_conf_.observations_size * onnx_conf_.num_hist; ++ii) {
      stage.observations[ii] = static_cast<float>(propri_history_buffer_[ii]);
    }
    const float obs_min = -stage.observations_clip;
    const float obs_max = stage.observations_clip;
    std::transform(stage.observations.begin(), stage.observations.end(), stage.observations.begin(),
                   [obs_min, obs_max](float x) { return std::max(obs_min, std::min(obs_max, x)); });
  } else {
    for (int ii = 0; ii < onnx_conf_.observations_size * onnx_conf_.num_hist; ++ii) {
      observations_[ii] = static_cast<float>(propri_history_buffer_[ii]);
    }
    const scalar_t obs_min = -onnx_conf_.observations_clip;
    const scalar_t obs_max = onnx_conf_.observations_clip;
    std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                   [obs_min, obs_max](scalar_t x) { return std::max(obs_min, std::min(obs_max, x)); });
  }
}

void RLController::ComputeActions() {
  if (multi_stage_) {
    OnnxStage& stage = stages_[active_stage_idx_];
    std::vector<Ort::Value> input_tensor;
    input_tensor.push_back(Ort::Value::CreateTensor<float>(
        memory_info_, stage.observations.data(), stage.observations.size(), stage.input_shapes[0].data(),
        stage.input_shapes[0].size()));
    std::vector<Ort::Value> output_values =
        stage.session->Run(Ort::RunOptions{}, stage.input_names.data(), input_tensor.data(), 1,
                           stage.output_names.data(), 1);
    for (int i = 0; i < stage.actions_size; ++i) {
      actions_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
    }
    const float action_min = -stage.actions_clip;
    const float action_max = stage.actions_clip;
    std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                   [action_min, action_max](float x) { return std::max(action_min, std::min(action_max, x)); });
    return;
  }

  std::vector<Ort::Value> input_tensor;
  input_tensor.push_back(Ort::Value::CreateTensor<float>(
      memory_info_, observations_.data(), observations_.size(), input_shapes_[0].data(), input_shapes_[0].size()));

  std::vector<Ort::Value> output_values =
      session_ptr_->Run(Ort::RunOptions{}, input_names_.data(), input_tensor.data(), 1, output_names_.data(), 1);

  for (int i = 0; i < onnx_conf_.actions_size; ++i) {
    actions_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
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
  frame.cycle_time = cur_cycle_time_;
  frame.smoothed_speed = smoothed_speed_;
  frame.active_stage = multi_stage_ ? static_cast<int>(active_stage_idx_) : 0;
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

  diag_logger_.Flush();
  diag_logger_.Close();
  diag_logging_triggered_ = false;
  diag_walk_entered_.store(false, std::memory_order_release);
  AIMRT_INFO("walk_diag logging finished, reason={}, frames={}, dropped={}",
             reason,
             diag_log_count_,
             diag_dropped_count_.load(std::memory_order_relaxed));
}

void RLController::FinalizeTmLogging(const char* reason) {
  if (!tm_logging_triggered_) {
    return;
  }

  tm_logger_.Flush();
  tm_logger_.Close();
  tm_logging_triggered_ = false;
  tm_walk_entered_.store(false, std::memory_order_release);
  AIMRT_INFO("tm_obs_input logging finished, reason={}, frames={}, dropped={}",
             reason,
             tm_log_count_,
             tm_dropped_count_.load(std::memory_order_relaxed));
}

void RLController::DrainDiagBuffer() {
  while (diag_read_idx_.load(std::memory_order_relaxed) != diag_write_idx_.load(std::memory_order_acquire)) {
    const size_t read_idx = diag_read_idx_.load(std::memory_order_relaxed);
    const DiagFrame& frame = diag_ring_[read_idx];

    const bool walk_entered = diag_walk_entered_.load(std::memory_order_acquire);
    if (walk_entered && !diag_logging_triggered_) {
      const std::string diag_path = diag_log_dir_ + "/walk_diag_" + MakeTimestampString() + ".csv";
      if (!diag_logger_.Open(diag_path, false, true)) {
        AIMRT_ERROR("Failed to open walk_diag log file: {}", diag_path);
      } else {
        std::ostringstream header;
        header << "timestamp_ns,phase_sin,phase_cos,cycle_time,smoothed_speed,active_stage"
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
        diag_logger_.WriteTextLine(header.str());
        diag_logging_triggered_ = true;
        diag_log_count_ = 0;
        AIMRT_INFO("walk_diag logging triggered: {}", diag_path);
      }
    }

    if (diag_logging_triggered_ && diag_log_count_ < diag_log_max_count_) {
      std::ostringstream row;
      row << frame.timestamp_ns
          << "," << frame.phase_sin
          << "," << frame.phase_cos
          << "," << frame.cycle_time
          << "," << frame.smoothed_speed
          << "," << frame.active_stage
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
      diag_logger_.WriteTextLine(row.str());
      ++diag_log_count_;

      if (diag_log_count_ >= diag_log_max_count_) {
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

    const bool walk_entered = tm_walk_entered_.load(std::memory_order_acquire);
    if (walk_entered && !tm_logging_triggered_) {
      const std::string bin_path = tm_log_dir_ + "/tm_obs_input_" + MakeTimestampString() + ".bin";
      if (!tm_logger_.Open(bin_path, true, false)) {
        AIMRT_ERROR("Failed to open tm_obs_input log file: {}", bin_path);
      } else {
        tm_logging_triggered_ = true;
        tm_log_count_ = 0;
        AIMRT_INFO("tm_obs_input logging triggered: {}", bin_path);
      }
    }

    if (tm_logging_triggered_ && tm_log_count_ < tm_log_max_count_) {
      tm_logger_.WriteRaw(frame.observations.data(), frame.observations.size() * sizeof(float));
      ++tm_log_count_;
      if (tm_log_count_ >= tm_log_max_count_) {
        FinalizeTmLogging("frame_limit");
      }
    }

    tm_read_idx_.store((read_idx + 1) % tm_ring_.size(), std::memory_order_release);
  }
}

}  // namespace xyber_x1_infer::rl_control_module
