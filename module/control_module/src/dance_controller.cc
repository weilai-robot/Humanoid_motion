#include "control_module/dance_controller.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace xyber_x1_infer::rl_control_module {
namespace {
constexpr float kRadToDeg = 57.29577951308232f;

template <typename T>
void ReadExact(std::ifstream& file, T* data, size_t count = 1) {
  if (!file.read(reinterpret_cast<char*>(data), sizeof(T) * count)) {
    throw std::runtime_error("dance asset is truncated");
  }
}

std::string ReadName(std::ifstream& file) {
  std::array<char, 32> bytes{};
  ReadExact(file, bytes.data(), bytes.size());
  const auto end = std::find(bytes.begin(), bytes.end(), '\0');
  if (end == bytes.end()) throw std::runtime_error("dance asset has an invalid joint name");
  return std::string(bytes.begin(), end);
}

bool Finite(float value) { return std::isfinite(value); }

bool ValidQuat(const Eigen::Quaternionf& q) {
  return Finite(q.w()) && Finite(q.x()) && Finite(q.y()) && Finite(q.z()) &&
         q.norm() > 0.9f && q.norm() < 1.1f;
}

Eigen::Quaternionf ToQuat(const geometry_msgs::msg::Quaternion& q) {
  return {static_cast<float>(q.w), static_cast<float>(q.x),
          static_cast<float>(q.y), static_cast<float>(q.z)};
}

}  // namespace

DanceController::DanceController(bool use_sim_handles) : ControllerBase(use_sim_handles) {}

void DanceController::Init(const YAML::Node& cfg) {
  joint_names_ = cfg["joint_list"].as<std::vector<std::string>>();
  if (joint_names_.size() != kJoints) throw std::runtime_error("dance needs exactly 29 joints");
  decimation_ = cfg["decimation"].as<int>();
  prepare_ticks_ = cfg["prepare_ticks"].as<int>();
  sensor_timeout_ms_ = cfg["sensor_timeout_ms"].as<int>();
  max_tilt_deg_ = cfg["max_tilt_deg"].as<float>();
  max_start_speed_ = cfg["max_start_speed"].as<float>();
  max_tracking_error_ = cfg["max_tracking_error"].as<float>();
  max_start_pose_error_ = cfg["max_start_pose_error"].as<float>();
  max_anchor_error_ = cfg["max_anchor_error"].as<float>();
  joint_limit_margin_ = cfg["joint_limit_margin"].as<float>();
  sign_calibration_verified_ = cfg["sign_calibration_verified"].as<bool>();
  odom_frame_ = cfg["odom_frame"].as<std::string>();
  anchor_frame_ = cfg["anchor_frame"].as<std::string>();
  base_frame_ = cfg["base_frame"].as<std::string>();
  if (decimation_ != 20 || prepare_ticks_ < 1000 || sensor_timeout_ms_ <= 0 ||
      !Finite(max_tilt_deg_) || !Finite(max_start_speed_) ||
      !Finite(max_tracking_error_) || !Finite(max_start_pose_error_) ||
      !Finite(max_anchor_error_) || !Finite(joint_limit_margin_) ||
      max_tilt_deg_ <= 0 || max_start_speed_ <= 0 || max_tracking_error_ <= 0 ||
      max_start_pose_error_ <= 0 || max_anchor_error_ <= 0 || joint_limit_margin_ < 0) {
    throw std::runtime_error("invalid dance control timing or safety limits");
  }
  if (!cfg["joint_limits"]) throw std::runtime_error("dance joint_limits missing");
  joint_sign_.fill(1.0f);
  const auto sign_cfg = cfg["joint_sign"];
  if (sign_cfg) {
    if (!sign_cfg.IsMap()) throw std::runtime_error("dance joint_sign must be a map");
    for (const auto& item : sign_cfg) {
      const std::string name = item.first.as<std::string>();
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
      if (it == joint_names_.end()) throw std::runtime_error("unknown dance joint_sign: " + name);
      const float sign = item.second.as<float>();
      if (sign != -1.0f && sign != 1.0f) {
        throw std::runtime_error("dance joint_sign must be +1 or -1: " + name);
      }
      joint_sign_[std::distance(joint_names_.begin(), it)] = sign;
    }
  }
  joint_conf_.pos_limit_lower.resize(kJoints);
  joint_conf_.pos_limit_upper.resize(kJoints);
  for (int i = 0; i < kJoints; ++i) {
    const auto limit = cfg["joint_limits"][joint_names_[i]];
    if (!limit) throw std::runtime_error("dance joint limit missing: " + joint_names_[i]);
    const float lower = limit["lower"].as<float>();
    const float upper = limit["upper"].as<float>();
    if (!Finite(lower) || !Finite(upper) || lower + 2 * joint_limit_margin_ >= upper) {
      throw std::runtime_error("invalid dance joint limit: " + joint_names_[i]);
    }
    // The top-level limits and DCU joint feedback share hardware coordinates.
    // Convert limits into the policy coordinates used by the reference/actor.
    joint_conf_.pos_limit_lower(i) = joint_sign_[i] > 0 ? lower : -upper;
    joint_conf_.pos_limit_upper(i) = joint_sign_[i] > 0 ? upper : -lower;
  }

  LoadModel(cfg["model_file"].as<std::string>());
  LoadMotion(cfg["motion_file"].as<std::string>());
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.assign(kJoints, 0.0);
  joint_state_data_.velocity.assign(kJoints, 0.0);
  joint_state_data_.effort.assign(kJoints, 0.0);
  target_pos_ = default_pos_;
  hidden1_.resize(512);
  hidden2_.resize(256);
  hidden3_.resize(128);
  output_.resize(kJoints);
}

void DanceController::LoadModel(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("cannot open dance model: " + path);
  std::array<char, 8> magic{};
  ReadExact(file, magic.data(), magic.size());
  if (std::memcmp(magic.data(), "X1DNCE1", 7) != 0 || magic[7] != '\0') {
    throw std::runtime_error("invalid dance model magic");
  }
  std::array<uint32_t, 7> header{};
  ReadExact(file, header.data(), header.size());
  if (header != std::array<uint32_t, 7>{1, kJoints, kObs, 512, 256, 128, kJoints}) {
    throw std::runtime_error("unsupported dance model dimensions");
  }
  for (const auto& name : joint_names_) {
    if (ReadName(file) != name) throw std::runtime_error("dance model joint order mismatch");
  }
  for (auto* values : {&default_pos_, &kp_, &kd_, &effort_limit_, &action_scale_}) {
    ReadExact(file, values->data(), values->size());
  }
  ReadExact(file, mean_.data(), mean_.size());
  ReadExact(file, inv_std_.data(), inv_std_.size());
  for (int i = 0; i < kJoints; ++i) {
    if (!Finite(default_pos_[i]) || !Finite(kp_[i]) || !Finite(kd_[i]) ||
        !Finite(effort_limit_[i]) || !Finite(action_scale_[i]) ||
        kp_[i] <= 0 || kd_[i] <= 0 || effort_limit_[i] <= 0 || action_scale_[i] <= 0) {
      throw std::runtime_error("invalid dance joint parameters");
    }
  }
  for (int i = 0; i < kObs; ++i) {
    if (!Finite(mean_[i]) || !Finite(inv_std_[i]) || inv_std_[i] <= 0) {
      throw std::runtime_error("invalid dance observation normalizer");
    }
  }
  const std::array<int, 5> dims{kObs, 512, 256, 128, kJoints};
  for (int i = 0; i < 4; ++i) {
    layers_[i].weight.resize(dims[i + 1], dims[i]);
    layers_[i].bias.resize(dims[i + 1]);
    ReadExact(file, layers_[i].weight.data(), dims[i] * dims[i + 1]);
    ReadExact(file, layers_[i].bias.data(), dims[i + 1]);
    if (!layers_[i].weight.allFinite() || !layers_[i].bias.allFinite()) {
      throw std::runtime_error("nonfinite dance actor weights");
    }
  }
  if (file.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error("dance model has unexpected trailing data");
  }
}

void DanceController::LoadMotion(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("cannot open dance motion: " + path);
  std::array<char, 8> magic{};
  ReadExact(file, magic.data(), magic.size());
  if (std::memcmp(magic.data(), "X1MOTN1", 7) != 0 || magic[7] != '\0') {
    throw std::runtime_error("invalid dance motion magic");
  }
  std::array<uint32_t, 4> header{};
  ReadExact(file, header.data(), header.size());
  if (header[0] != 1 || header[1] < 2 || header[1] > 100000 ||
      header[2] != kJoints || header[3] != 50) {
    throw std::runtime_error("unsupported dance motion dimensions or fps");
  }
  for (const auto& name : joint_names_) {
    if (ReadName(file) != name) throw std::runtime_error("dance motion joint order mismatch");
  }
  motion_.resize(header[1]);
  std::array<float, 2 * kJoints + 7> raw{};
  for (size_t frame_index = 0; frame_index < motion_.size(); ++frame_index) {
    auto& frame = motion_[frame_index];
    ReadExact(file, raw.data(), raw.size());
    if (!std::all_of(raw.begin(), raw.end(), Finite)) {
      throw std::runtime_error("nonfinite dance motion frame");
    }
    std::copy_n(raw.begin(), kJoints, frame.joint_pos.begin());
    std::copy_n(raw.begin() + kJoints, kJoints, frame.joint_vel.begin());
    for (int joint = 0; joint < kJoints; ++joint) {
      constexpr float kRoundingTolerance = 1e-4f;
      const float lower = joint_conf_.pos_limit_lower(joint);
      const float upper = joint_conf_.pos_limit_upper(joint);
      if (frame.joint_pos[joint] < lower - kRoundingTolerance ||
          frame.joint_pos[joint] > upper + kRoundingTolerance) {
        throw std::runtime_error(
            "dance reference exceeds configured joint limit: " + joint_names_[joint] +
            " frame=" + std::to_string(frame_index) +
            " reference=" + std::to_string(frame.joint_pos[joint]) +
            " limits=[" + std::to_string(lower) + "," + std::to_string(upper) + "]");
      }
    }
    frame.anchor_pos = Eigen::Vector3f(raw[58], raw[59], raw[60]);
    frame.anchor_quat = Eigen::Quaternionf(raw[61], raw[62], raw[63], raw[64]);
    if (!ValidQuat(frame.anchor_quat)) throw std::runtime_error("invalid dance motion quaternion");
    frame.anchor_quat.normalize();
  }
  if (file.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error("dance motion has unexpected trailing data");
  }
}

void DanceController::SetImuData(const sensor_msgs::msg::Imu value) {
  std::unique_lock<std::shared_mutex> lock(sensor_mutex_);
  imu_data_ = value;
  last_imu_ = Clock::now();
}

void DanceController::SetJointStateData(
    const sensor_msgs::msg::JointState value,
    const std::unordered_map<std::string, int>& index) {
  if (value.position.size() < value.name.size() || value.velocity.size() < value.name.size()) return;
  std::unique_lock<std::shared_mutex> lock(sensor_mutex_);
  for (int i = 0; i < kJoints; ++i) {
    const auto it = index.find(joint_names_[i]);
    if (it == index.end() || it->second < 0 ||
        static_cast<size_t>(it->second) >= value.position.size()) return;
    const int slot = it->second;
    if (!std::isfinite(value.position[slot]) || !std::isfinite(value.velocity[slot])) return;
  }
  for (int i = 0; i < kJoints; ++i) {
    const int slot = index.at(joint_names_[i]);
    joint_state_data_.position[i] = joint_sign_[i] * value.position[slot];
    joint_state_data_.velocity[i] = joint_sign_[i] * value.velocity[slot];
  }
  last_joint_ = Clock::now();
}

void DanceController::SetAnchorPose(const geometry_msgs::msg::TransformStamped& pose) {
  if (pose.header.frame_id != odom_frame_ || pose.child_frame_id != anchor_frame_) return;
  const auto q = ToQuat(pose.transform.rotation);
  if (!ValidQuat(q) || !std::isfinite(pose.transform.translation.x) ||
      !std::isfinite(pose.transform.translation.y) || !std::isfinite(pose.transform.translation.z)) return;
  std::unique_lock<std::shared_mutex> lock(pose_mutex_);
  anchor_pose_ = pose;
  last_pose_ = Clock::now();
}

void DanceController::SetBaseVelocity(const geometry_msgs::msg::TwistStamped& velocity) {
  if (velocity.header.frame_id != base_frame_ ||
      !std::isfinite(velocity.twist.linear.x) || !std::isfinite(velocity.twist.linear.y) ||
      !std::isfinite(velocity.twist.linear.z)) return;
  std::unique_lock<std::shared_mutex> lock(velocity_mutex_);
  base_velocity_ = velocity;
  last_velocity_ = Clock::now();
}

bool DanceController::ReadSnapshot(Snapshot& out) const {
  const auto now = Clock::now();
  const auto max_age = std::chrono::milliseconds(sensor_timeout_ms_);
  {
    std::shared_lock<std::shared_mutex> lock(sensor_mutex_);
    if (now - last_joint_ > max_age || now - last_imu_ > max_age) return false;
    for (int i = 0; i < kJoints; ++i) {
      out.joint_pos[i] = joint_state_data_.position[i];
      out.joint_vel[i] = joint_state_data_.velocity[i];
    }
    out.base_quat = ToQuat(imu_data_.orientation);
    out.base_ang_vel = Eigen::Vector3f(imu_data_.angular_velocity.x,
                                       imu_data_.angular_velocity.y,
                                       imu_data_.angular_velocity.z);
  }
  {
    std::shared_lock<std::shared_mutex> lock(pose_mutex_);
    if (now - last_pose_ > max_age) return false;
    out.anchor_pos = Eigen::Vector3f(anchor_pose_.transform.translation.x,
                                      anchor_pose_.transform.translation.y,
                                      anchor_pose_.transform.translation.z);
    out.anchor_quat = ToQuat(anchor_pose_.transform.rotation);
  }
  {
    std::shared_lock<std::shared_mutex> lock(velocity_mutex_);
    if (now - last_velocity_ > max_age) return false;
    out.base_lin_vel = Eigen::Vector3f(base_velocity_.twist.linear.x,
                                        base_velocity_.twist.linear.y,
                                        base_velocity_.twist.linear.z);
  }
  if (!ValidQuat(out.base_quat) || !ValidQuat(out.anchor_quat) ||
      !out.base_lin_vel.allFinite() || !out.base_ang_vel.allFinite() ||
      !out.anchor_pos.allFinite()) return false;
  out.base_quat.normalize();
  out.anchor_quat.normalize();
  return true;
}

bool DanceController::Ready() const {
  Snapshot snapshot;
  if (!sign_calibration_verified_ || !ReadSnapshot(snapshot) || motion_.empty()) return false;
  if (snapshot.base_lin_vel.norm() > max_start_speed_) return false;
  const float up = (snapshot.base_quat * Eigen::Vector3f::UnitZ()).z();
  if (std::acos(std::clamp(up, -1.0f, 1.0f)) * kRadToDeg > max_tilt_deg_) return false;
  for (int i = 0; i < kJoints; ++i) {
    if (snapshot.joint_pos[i] < joint_conf_.pos_limit_lower(i) ||
        snapshot.joint_pos[i] > joint_conf_.pos_limit_upper(i)) return false;
  }
  return true;
}

void DanceController::RestartController() {
  if (!sign_calibration_verified_) {
    SetFault("dance joint direction calibration not verified");
    return;
  }
  Snapshot snapshot;
  if (!ReadSnapshot(snapshot)) {
    SetFault("required feedback missing at dance start");
    return;
  }
  current_ = snapshot;
  for (int i = 0; i < kJoints; ++i) {
    start_pos_[i] = static_cast<float>(snapshot.joint_pos[i]);
    target_pos_[i] = start_pos_[i];
  }
  last_action_.fill(0.0f);
  tick_ = 0;
  frame_index_ = 0;
  faulted_ = false;
  fault_reason_.clear();
  active_ = true;
  reference_alignment_quat_ = snapshot.anchor_quat * motion_[0].anchor_quat.conjugate();
  reference_alignment_quat_.normalize();
  reference_alignment_pos_ = snapshot.anchor_pos -
                             reference_alignment_quat_ * motion_[0].anchor_pos;
}

void DanceController::SetFault(const char* reason) {
  active_ = false;
  faulted_ = true;
  fault_reason_ = reason;
}

float DanceController::ClipTarget(int index, float target) const {
  const float lower = joint_conf_.pos_limit_lower(index) + joint_limit_margin_;
  const float upper = joint_conf_.pos_limit_upper(index) - joint_limit_margin_;
  return std::clamp(target, lower, upper);
}

void DanceController::ComputeObservation(const Snapshot& state, const MotionFrame& frame) {
  const Eigen::Vector3f ref_pos = reference_alignment_pos_ +
                                  reference_alignment_quat_ * frame.anchor_pos;
  const Eigen::Quaternionf ref_quat = reference_alignment_quat_ * frame.anchor_quat;
  const Eigen::Vector3f anchor_delta = state.anchor_quat.conjugate() *
                                        (ref_pos - state.anchor_pos);
  const Eigen::Matrix3f rotation =
      (state.anchor_quat.conjugate() * ref_quat).normalized().toRotationMatrix();
  int offset = 0;
  auto put = [this, &offset](float value) { observation_[offset++] = value; };
  for (float value : frame.joint_pos) put(value);
  for (float value : frame.joint_vel) put(value);
  for (int i = 0; i < 3; ++i) put(anchor_delta[i]);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 2; ++col) put(rotation(row, col));
  }
  for (int i = 0; i < 3; ++i) put(state.base_lin_vel[i]);
  for (int i = 0; i < 3; ++i) put(state.base_ang_vel[i]);
  for (int i = 0; i < kJoints; ++i) put(state.joint_pos[i] - default_pos_[i]);
  for (int i = 0; i < kJoints; ++i) put(state.joint_vel[i]);
  for (float value : last_action_) put(value);
  if (offset != kObs) throw std::runtime_error("dance observation layout mismatch");
  for (int i = 0; i < kObs; ++i) {
    observation_[i] = (observation_[i] - mean_[i]) * inv_std_[i];
  }
}

void DanceController::Infer() {
  const Eigen::Map<const Eigen::VectorXf> input(observation_.data(), kObs);
  hidden1_.noalias() = layers_[0].weight * input;
  hidden1_ += layers_[0].bias;
  for (int i = 0; i < hidden1_.size(); ++i) if (hidden1_[i] < 0) hidden1_[i] = std::expm1(hidden1_[i]);
  hidden2_.noalias() = layers_[1].weight * hidden1_;
  hidden2_ += layers_[1].bias;
  for (int i = 0; i < hidden2_.size(); ++i) if (hidden2_[i] < 0) hidden2_[i] = std::expm1(hidden2_[i]);
  hidden3_.noalias() = layers_[2].weight * hidden2_;
  hidden3_ += layers_[2].bias;
  for (int i = 0; i < hidden3_.size(); ++i) if (hidden3_[i] < 0) hidden3_[i] = std::expm1(hidden3_[i]);
  output_.noalias() = layers_[3].weight * hidden3_;
  output_ += layers_[3].bias;
  if (!output_.allFinite() || output_.cwiseAbs().maxCoeff() > 20.0f) {
    SetFault("nonfinite or excessive dance policy action");
    return;
  }
  for (int i = 0; i < kJoints; ++i) {
    last_action_[i] = output_[i];
    target_pos_[i] = ClipTarget(i, default_pos_[i] + action_scale_[i] * output_[i]);
  }
}

void DanceController::Update() {
  if (!active_ || faulted_) return;
  Snapshot snapshot;
  if (!ReadSnapshot(snapshot)) {
    SetFault("dance feedback or state estimate stale");
    return;
  }
  const float up = (snapshot.base_quat * Eigen::Vector3f::UnitZ()).z();
  if (std::acos(std::clamp(up, -1.0f, 1.0f)) * kRadToDeg > max_tilt_deg_) {
    SetFault("dance base tilt exceeded limit");
    return;
  }
  for (int i = 0; i < kJoints; ++i) {
    if (std::abs(snapshot.joint_pos[i] - target_pos_[i]) > max_tracking_error_ ||
        snapshot.joint_pos[i] < joint_conf_.pos_limit_lower(i) ||
        snapshot.joint_pos[i] > joint_conf_.pos_limit_upper(i)) {
      SetFault("dance joint tracking error or limit exceeded");
      return;
    }
  }
  const auto& reference = motion_[std::min(frame_index_, motion_.size() - 1)];
  const Eigen::Vector3f expected_anchor = reference_alignment_pos_ +
      reference_alignment_quat_ * reference.anchor_pos;
  if ((expected_anchor - snapshot.anchor_pos).norm() > max_anchor_error_) {
    SetFault("dance anchor tracking error exceeded limit");
    return;
  }
  current_ = snapshot;
  if (tick_ < static_cast<uint64_t>(prepare_ticks_)) {
    const float u = static_cast<float>(tick_ + 1) / prepare_ticks_;
    const float blend = u * u * (3.0f - 2.0f * u);
    for (int i = 0; i < kJoints; ++i) {
      target_pos_[i] = ClipTarget(i, start_pos_[i] + blend *
                                                (motion_[0].joint_pos[i] - start_pos_[i]));
    }
  } else if ((tick_ - prepare_ticks_) % decimation_ == 0) {
    if (tick_ == static_cast<uint64_t>(prepare_ticks_)) {
      for (int i = 0; i < kJoints; ++i) {
        if (std::abs(snapshot.joint_pos[i] - target_pos_[i]) > max_start_pose_error_) {
          SetFault("dance reference start pose not reached");
          return;
        }
      }
    }
    if (frame_index_ >= motion_.size()) {
      SetFault("dance motion completed");
      return;
    }
    ComputeObservation(snapshot, motion_[frame_index_]);
    if (!std::all_of(observation_.begin(), observation_.end(), Finite)) {
      SetFault("nonfinite dance observation");
      return;
    }
    Infer();
    ++frame_index_;
  }
  ++tick_;
}

my_ros2_proto::msg::JointCommand DanceController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand command;
  command.name = joint_names_;
  command.position.resize(kJoints);
  command.velocity.assign(kJoints, 0.0);
  command.effort.assign(kJoints, 0.0);
  command.stiffness.resize(kJoints);
  command.damping.resize(kJoints);
  for (int i = 0; i < kJoints; ++i) {
    if (joint_names_[i].find("_ankle_") != std::string::npos) {
      const float torque = kp_[i] * (target_pos_[i] - current_.joint_pos[i]) -
                           kd_[i] * current_.joint_vel[i];
      command.position[i] = 0.0;
      command.effort[i] = joint_sign_[i] *
          std::clamp(torque, -effort_limit_[i], effort_limit_[i]);
      command.stiffness[i] = 0.0;
      command.damping[i] = 0.0;
    } else {
      command.position[i] = joint_sign_[i] * target_pos_[i];
      command.stiffness[i] = kp_[i];
      command.damping[i] = kd_[i];
    }
  }
  return command;
}

}  // namespace xyber_x1_infer::rl_control_module
