#include "control_module/dance_controller.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <set>
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

int64_t StampNs(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) return 0;
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool FreshStamp(const builtin_interfaces::msg::Time& stamp, int timeout_ms, bool simulation) {
  const int64_t ns = StampNs(stamp);
  if (ns <= 0) return false;
  if (simulation) return true;  // ROS simulation clock can differ from system time.
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return now - ns >= -20000000LL && now - ns <= timeout_ms * 1000000LL;
}

}  // namespace

DanceController::DanceController(bool use_sim_handles) : ControllerBase(use_sim_handles) {}

void DanceController::Init(const YAML::Node& cfg) {
  joint_names_ = cfg["joint_list"].as<std::vector<std::string>>();
  if (joint_names_.size() != kJoints ||
      std::set<std::string>(joint_names_.begin(), joint_names_.end()).size() != kJoints) {
    throw std::runtime_error("dance needs exactly 29 unique joints");
  }
  decimation_ = cfg["decimation"].as<int>();
  const int prepare_ticks = cfg["prepare_ticks"].as<int>();
  sensor_timeout_ms_ = cfg["sensor_timeout_ms"].as<int>();
  max_tilt_deg_ = cfg["max_tilt_deg"].as<float>();
  max_start_speed_ = cfg["max_start_speed"].as<float>();
  max_start_pose_error_ = cfg["max_start_pose_error"].as<float>();
  max_tracking_error_ = cfg["max_tracking_error"].as<float>();
  max_anchor_error_ = cfg["max_anchor_error"].as<float>();
  joint_limit_margin_ = cfg["joint_limit_margin"].as<float>();
  joint_limit_tolerance_ = cfg["joint_limit_tolerance"].as<float>();
  odom_frame_ = cfg["odom_frame"].as<std::string>();
  anchor_frame_ = cfg["anchor_frame"].as<std::string>();
  base_frame_ = cfg["base_frame"].as<std::string>();
  const std::string state_source = cfg["state_source"] ? cfg["state_source"].as<std::string>() : "external_topics";
  if (state_source != "odometry" && state_source != "external_topics" && state_source != "proprioceptive") {
    throw std::runtime_error("dance state_source must be odometry, proprioceptive or external_topics");
  }
  use_proprioception_ = state_source == "proprioceptive";
  use_odometry_ = state_source == "odometry" || use_proprioception_;
  if (use_proprioception_) {
    DanceBodyStateEstimator::Config estimator_cfg;
    const auto c = cfg["state_estimator"];
    if (c) {
      const auto read = [&c](const char* key, double& value) {
        if (c[key]) value = c[key].as<double>();
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error(std::string("invalid state_estimator.")+key);
      };
      read("acceleration_scale", estimator_cfg.acceleration_scale);
      read("initialization_s", estimator_cfg.initialization_s);
      read("contact_on_force", estimator_cfg.contact_on_force);
      read("contact_off_force", estimator_cfg.contact_off_force);
      read("acceleration_noise", estimator_cfg.acceleration_noise);
      read("bias_noise", estimator_cfg.bias_noise);
      read("position_noise", estimator_cfg.position_noise);
      read("velocity_noise", estimator_cfg.velocity_noise);
      read("height_noise", estimator_cfg.height_noise);
      read("max_gap_s", estimator_cfg.max_gap_s);
      read("max_flight_s", estimator_cfg.max_flight_s);
      read("contact_velocity_limit", estimator_cfg.contact_velocity_limit);
      if (c["imu_position_in_base"]) {
        const auto offset = c["imu_position_in_base"].as<std::vector<double>>();
        if (offset.size() != 3 || !std::all_of(offset.begin(), offset.end(), [](double v) { return std::isfinite(v); }))
          throw std::runtime_error("state_estimator.imu_position_in_base must have three finite coordinates");
        std::copy(offset.begin(), offset.end(), estimator_cfg.imu_position_in_base.begin());
      }
    }
    if (estimator_cfg.contact_on_force <= estimator_cfg.contact_off_force || estimator_cfg.max_gap_s > 0.01)
      throw std::runtime_error("invalid estimator contact hysteresis or source-time gap");
    body_estimator_ = DanceBodyStateEstimator(estimator_cfg);
    for (size_t i = 0; i < leg_indices_.size(); ++i) {
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), kDanceLegJointNames[i]);
      if (it == joint_names_.end()) throw std::runtime_error("body estimator missing leg joint");
      leg_indices_[i] = std::distance(joint_names_.begin(), it);
    }
  }
  if (use_odometry_) {
    if (base_frame_ != "base_link" || anchor_frame_ != "lumbar_pitch_link") {
      throw std::runtime_error("X1 waist FK requires base_link and lumbar_pitch_link frames");
    }
    const std::array<std::string, 3> names{"lumbar_yaw_joint", "lumbar_roll_joint", "lumbar_pitch_joint"};
    for (size_t i = 0; i < names.size(); ++i) {
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), names[i]);
      if (it == joint_names_.end()) throw std::runtime_error("X1 waist FK missing joint: " + names[i]);
      waist_indices_[i] = std::distance(joint_names_.begin(), it);
    }
  }
  const int control_hz = cfg["control_frequency_hz"].as<int>();
  if (control_hz != 1000 || decimation_ != 20 || prepare_ticks != 0 ||
      sensor_timeout_ms_ <= 0 || sensor_timeout_ms_ > 80 ||
      !Finite(max_tilt_deg_) || !Finite(max_start_speed_) || !Finite(max_start_pose_error_) ||
      !Finite(max_tracking_error_) || !Finite(joint_limit_tolerance_) ||
      !Finite(max_anchor_error_) || !Finite(joint_limit_margin_) ||
      max_tilt_deg_ <= 0 || max_tilt_deg_ >= 90 || max_start_speed_ <= 0 ||
      max_start_pose_error_ <= 0 || max_tracking_error_ < 0 ||
      max_anchor_error_ <= 0 || joint_limit_margin_ < 0 ||
      joint_limit_tolerance_ < 0 || joint_limit_tolerance_ > 0.05f) {
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
  const auto start_pose = cfg["start_pose"].as<std::string>();
  if (start_pose == "default_q") preparation_pos_ = default_pos_;
  else if (start_pose == "motion_start") preparation_pos_ = motion_[0].joint_pos;
  else throw std::runtime_error("dance start_pose must be default_q or motion_start");
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
  if (std::memcmp(magic.data(), "X1DNCE2", 7) != 0 || magic[7] != '\0') {
    throw std::runtime_error("invalid dance model magic");
  }
  std::array<uint32_t, 7> header{};
  ReadExact(file, header.data(), header.size());
  if (header != std::array<uint32_t, 7>{2, kJoints, kObs, 512, 256, 128, kJoints}) {
    throw std::runtime_error("unsupported dance model dimensions");
  }
  for (const auto& name : joint_names_) {
    if (ReadName(file) != name) throw std::runtime_error("dance model joint order mismatch");
  }
  for (auto* values : {&default_pos_, &kp_, &kd_, &effort_limit_, &action_scale_, &armature_}) {
    ReadExact(file, values->data(), values->size());
  }
  ReadExact(file, mean_.data(), mean_.size());
  ReadExact(file, inv_std_.data(), inv_std_.size());
  for (int i = 0; i < kJoints; ++i) {
    if (!Finite(default_pos_[i]) || !Finite(kp_[i]) || !Finite(kd_[i]) ||
        !Finite(effort_limit_[i]) || !Finite(action_scale_[i]) || !Finite(armature_[i]) ||
        kp_[i] <= 0 || kd_[i] <= 0 || kd_[i] > 8 || effort_limit_[i] <= 0 ||
        armature_[i] < 0 || action_scale_[i] <= 0 ||
        std::abs(action_scale_[i] - 0.25f * effort_limit_[i] / kp_[i]) > 1e-6f) {
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
  if (!FreshStamp(value.header.stamp, sensor_timeout_ms_, use_sim_handles_) ||
      !ValidQuat(ToQuat(value.orientation)) ||
      !std::isfinite(value.angular_velocity.x) || !std::isfinite(value.angular_velocity.y) ||
      !std::isfinite(value.angular_velocity.z)) return;
  std::unique_lock<std::shared_mutex> lock(sensor_mutex_);
  if (StampNs(value.header.stamp) <= StampNs(imu_data_.header.stamp)) return;
  imu_data_ = value;
  last_imu_ = Clock::now();
  lock.unlock();
  if (use_proprioception_) {
    DanceBodyStateEstimator::Input input;
    input.stamp_ns = StampNs(value.header.stamp);
    input.orientation = {value.orientation.w, value.orientation.x, value.orientation.y, value.orientation.z};
    input.gyro = {value.angular_velocity.x, value.angular_velocity.y, value.angular_velocity.z};
    input.acceleration = {value.linear_acceleration.x, value.linear_acceleration.y, value.linear_acceleration.z};
    std::lock_guard<std::mutex> estimator_lock(estimator_mutex_);
    estimator_inputs_.PushImu(input);
    UpdateBodyEstimate();
  }
}

void DanceController::SetJointStateData(
    const sensor_msgs::msg::JointState value,
    const std::unordered_map<std::string, int>& index) {
  if (value.position.size() < value.name.size() || value.velocity.size() < value.name.size() ||
      (use_proprioception_ && value.effort.size() < value.name.size()) ||
      !FreshStamp(value.header.stamp, sensor_timeout_ms_, use_sim_handles_)) return;
  std::unique_lock<std::shared_mutex> lock(sensor_mutex_);
  if (StampNs(value.header.stamp) <= StampNs(joint_state_data_.header.stamp)) return;
  for (int i = 0; i < kJoints; ++i) {
    const auto it = index.find(joint_names_[i]);
    if (it == index.end() || it->second < 0 ||
        static_cast<size_t>(it->second) >= value.name.size() ||
        value.name[it->second] != joint_names_[i]) return;
    const int slot = it->second;
    if (!std::isfinite(value.position[slot]) || !std::isfinite(value.velocity[slot]) ||
        (use_proprioception_ && !std::isfinite(value.effort[slot]))) return;
  }
  for (int i = 0; i < kJoints; ++i) {
    const int slot = index.at(joint_names_[i]);
    joint_state_data_.position[i] = joint_sign_[i] * value.position[slot];
    joint_state_data_.velocity[i] = joint_sign_[i] * value.velocity[slot];
  }
  joint_state_data_.header = value.header;
  if (use_odometry_) {
    waist_history_.Push(StampNs(value.header.stamp),
        {joint_state_data_.position[waist_indices_[0]], joint_state_data_.position[waist_indices_[1]],
         joint_state_data_.position[waist_indices_[2]]});
  }
  last_joint_ = Clock::now();
  DanceEstimatorInputs::Joints legs{};
  if (use_proprioception_) {
    for (size_t i = 0; i < leg_indices_.size(); ++i) {
      const size_t j = leg_indices_[i];
      legs[i] = joint_state_data_.position[j];
      legs[12+i] = joint_state_data_.velocity[j];
      legs[24+i] = joint_sign_[j]*value.effort[index.at(joint_names_[j])];
    }
  }
  lock.unlock();
  if (use_proprioception_) {
    std::lock_guard<std::mutex> estimator_lock(estimator_mutex_);
    estimator_inputs_.PushJoints(StampNs(value.header.stamp), legs);
    UpdateBodyEstimate();
  }
}

void DanceController::UpdateBodyEstimate() {
  DanceBodyStateEstimator::Input input;
  while (estimator_inputs_.Next(input)) {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp.sec = static_cast<int32_t>(input.stamp_ns/1000000000LL);
    odometry.header.stamp.nanosec = static_cast<uint32_t>(input.stamp_ns%1000000000LL);
    if (!FreshStamp(odometry.header.stamp, sensor_timeout_ms_, use_sim_handles_)) continue;
    DanceBodyStateEstimator::Output state;
    if (!body_estimator_.Update(input, state)) {
      std::unique_lock<std::shared_mutex> lock(odometry_mutex_);
      odometry_problem_ = body_estimator_.Problem();
      continue;
    }
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    auto& p = odometry.pose.pose.position;
    auto& q = odometry.pose.pose.orientation;
    auto& v = odometry.twist.twist.linear;
    auto& w = odometry.twist.twist.angular;
    p.x = state.position[0]; p.y = state.position[1]; p.z = state.position[2];
    q.w = state.orientation[0]; q.x = state.orientation[1]; q.y = state.orientation[2]; q.z = state.orientation[3];
    v.x = state.velocity_body[0]; v.y = state.velocity_body[1]; v.z = state.velocity_body[2];
    w.x = input.gyro[0]; w.y = input.gyro[1]; w.z = input.gyro[2];
    const Eigen::Matrix3d rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    const Eigen::Vector3d variance(state.velocity_variance[0], state.velocity_variance[1], state.velocity_variance[2]);
    const Eigen::Matrix3d body_covariance = rotation.transpose()*variance.asDiagonal()*rotation;
    for (size_t i = 0; i < 3; ++i) {
      odometry.pose.covariance[7*i] = state.position_variance[i];
      odometry.pose.covariance[7*(i+3)] = 0.001;  // nominal IMU uncertainty; attitude is not estimated here
      odometry.twist.covariance[7*(i+3)] = 0.001;
      for (size_t j = 0; j < 3; ++j) odometry.twist.covariance[6*i+j] = body_covariance(i, j);
    }
    SetOdometry(odometry);
    if (estimated_odometry_callback_ && input.stamp_ns-last_estimate_publication_ns_ >= 20000000) {
      last_estimate_publication_ns_ = input.stamp_ns;
      estimated_odometry_callback_(odometry);  // 50 Hz inspection; internal feedback remains at sensor rate.
    }
  }
}

void DanceController::SetAnchorPose(const geometry_msgs::msg::TransformStamped& pose) {
  if (use_odometry_) return;
  if (pose.header.frame_id != odom_frame_ || pose.child_frame_id != anchor_frame_ ||
      !FreshStamp(pose.header.stamp, sensor_timeout_ms_, use_sim_handles_)) return;
  const auto q = ToQuat(pose.transform.rotation);
  if (!ValidQuat(q) || !std::isfinite(pose.transform.translation.x) ||
      !std::isfinite(pose.transform.translation.y) || !std::isfinite(pose.transform.translation.z)) return;
  std::unique_lock<std::shared_mutex> lock(pose_mutex_);
  if (StampNs(pose.header.stamp) <= StampNs(anchor_pose_.header.stamp)) return;
  anchor_pose_ = pose;
  last_pose_ = Clock::now();
}

void DanceController::SetBaseVelocity(const geometry_msgs::msg::TwistStamped& velocity) {
  if (use_odometry_) return;
  if (velocity.header.frame_id != base_frame_ ||
      !FreshStamp(velocity.header.stamp, sensor_timeout_ms_, use_sim_handles_) ||
      !std::isfinite(velocity.twist.linear.x) || !std::isfinite(velocity.twist.linear.y) ||
      !std::isfinite(velocity.twist.linear.z)) return;
  std::unique_lock<std::shared_mutex> lock(velocity_mutex_);
  if (StampNs(velocity.header.stamp) <= StampNs(base_velocity_.header.stamp)) return;
  base_velocity_ = velocity;
  last_velocity_ = Clock::now();
}

void DanceController::SetOdometry(const nav_msgs::msg::Odometry& odometry) {
  if (!use_odometry_) return;
  std::unique_lock<std::shared_mutex> lock(odometry_mutex_);
  if (odometry.header.frame_id != odom_frame_ || odometry.child_frame_id != base_frame_) {
    odometry_problem_ = "odometry frames mismatch: expected odom -> base_link, not Nav2 base_footprint";
    return;
  }
  if (!FreshStamp(odometry.header.stamp, sensor_timeout_ms_, use_sim_handles_)) {
    odometry_problem_ = "body odometry source timestamp is stale or invalid";
    return;
  }
  const auto& p = odometry.pose.pose.position;
  const auto& q = odometry.pose.pose.orientation;
  const auto& v = odometry.twist.twist.linear;
  if (!ValidQuat(ToQuat(q)) || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
    odometry_problem_ = "body odometry has invalid position, orientation or linear velocity";
    return;
  }
  if (odometry_count_ && StampNs(odometry.header.stamp) <=
      StampNs(body_odometry_history_[(odometry_next_ + kOdometryHistoryCapacity - 1) % kOdometryHistoryCapacity].stamp)) {
    odometry_problem_ = "body odometry source timestamp is not advancing";
    return;
  }
  auto& sample = body_odometry_history_[odometry_next_];
  sample.stamp = odometry.header.stamp;
  sample.position = {p.x, p.y, p.z};
  sample.orientation = {q.w, q.x, q.y, q.z};
  sample.linear_velocity = Eigen::Vector3f(v.x, v.y, v.z);
  sample.received = Clock::now();
  odometry_next_ = (odometry_next_ + 1) % kOdometryHistoryCapacity;
  if (odometry_count_ < kOdometryHistoryCapacity) ++odometry_count_;
  odometry_problem_ = "body odometry stream stopped or source timestamp expired";
}

bool DanceController::ReadSnapshot(Snapshot& out, const char** reason) const {
  const auto fail = [reason](const char* message) {
    if (reason) *reason = message;
    return false;
  };
  const auto now = Clock::now();
  const auto max_age = std::chrono::milliseconds(sensor_timeout_ms_);
  BodyOdometry odometry;
  // Fixed lock order: odometry then joint/IMU. Setters take only their own lock.
  std::shared_lock<std::shared_mutex> odometry_lock;
  if (use_odometry_) {
    odometry_lock = std::shared_lock<std::shared_mutex>(odometry_mutex_);
    if (!odometry_count_) return fail(odometry_problem_);
    const auto& newest = body_odometry_history_[(odometry_next_ + kOdometryHistoryCapacity - 1) % kOdometryHistoryCapacity];
    if (now - newest.received > max_age ||
        !FreshStamp(newest.stamp, sensor_timeout_ms_, use_sim_handles_)) {
      return fail(odometry_problem_);
    }
  }
  DanceWaistHistory::Positions waist{};
  {
    std::shared_lock<std::shared_mutex> lock(sensor_mutex_);
    if (now - last_joint_ > max_age ||
        !FreshStamp(joint_state_data_.header.stamp, sensor_timeout_ms_, use_sim_handles_)) {
      return fail("joint feedback missing, stale or source timestamp expired");
    }
    if (now - last_imu_ > max_age ||
        !FreshStamp(imu_data_.header.stamp, sensor_timeout_ms_, use_sim_handles_)) {
      return fail("IMU feedback missing, stale or source timestamp expired");
    }
    for (int i = 0; i < kJoints; ++i) {
      out.joint_pos[i] = joint_state_data_.position[i];
      out.joint_vel[i] = joint_state_data_.velocity[i];
    }
    out.base_quat = ToQuat(imu_data_.orientation);
    out.base_ang_vel = Eigen::Vector3f(imu_data_.angular_velocity.x,
                                       imu_data_.angular_velocity.y,
                                       imu_data_.angular_velocity.z);
    if (use_odometry_) {
      bool synchronized = false;
      // A new odometry sample may arrive just before its bracketing joint
      // sample. Keep using the newest complete, still-fresh pair in that gap;
      // arrival order must not reset the four-second readiness hold every time.
      for (size_t age = 0; age < odometry_count_; ++age) {
        const auto& candidate = body_odometry_history_[
            (odometry_next_ + kOdometryHistoryCapacity - 1 - age) % kOdometryHistoryCapacity];
        if (now - candidate.received > max_age ||
            !FreshStamp(candidate.stamp, sensor_timeout_ms_, use_sim_handles_)) continue;
        if (waist_history_.SampleAt(StampNs(candidate.stamp), waist)) {
          odometry = candidate;
          synchronized = true;
          break;
        }
      }
      if (!synchronized) {
        return fail("waist joint samples do not bracket odometry source time within a 10 ms interval");
      }
    }
  }
  if (odometry_lock.owns_lock()) odometry_lock.unlock();
  if (use_odometry_) {
    DanceAnchorState anchor;
    if (!ComputeX1DanceAnchor(odometry.position, odometry.orientation, waist, anchor)) {
      return fail("body odometry / waist FK produced an invalid anchor");
    }
    out.anchor_pos = Eigen::Vector3f(anchor.position[0], anchor.position[1], anchor.position[2]);
    out.anchor_quat = Eigen::Quaternionf(anchor.orientation[0], anchor.orientation[1],
                                         anchor.orientation[2], anchor.orientation[3]);
    out.base_lin_vel = odometry.linear_velocity;
  } else {
    {
      std::shared_lock<std::shared_mutex> lock(pose_mutex_);
      if (now - last_pose_ > max_age ||
          !FreshStamp(anchor_pose_.header.stamp, sensor_timeout_ms_, use_sim_handles_)) {
        return fail("/dance/anchor_pose missing, stale or source timestamp expired");
      }
      out.anchor_pos = Eigen::Vector3f(anchor_pose_.transform.translation.x,
                                        anchor_pose_.transform.translation.y,
                                        anchor_pose_.transform.translation.z);
      out.anchor_quat = ToQuat(anchor_pose_.transform.rotation);
    }
    {
      std::shared_lock<std::shared_mutex> lock(velocity_mutex_);
      if (now - last_velocity_ > max_age ||
          !FreshStamp(base_velocity_.header.stamp, sensor_timeout_ms_, use_sim_handles_)) {
        return fail("/dance/base_velocity missing, stale or source timestamp expired");
      }
      out.base_lin_vel = Eigen::Vector3f(base_velocity_.twist.linear.x,
                                          base_velocity_.twist.linear.y,
                                          base_velocity_.twist.linear.z);
    }
  }
  if (!ValidQuat(out.base_quat) || !ValidQuat(out.anchor_quat) ||
      !out.base_lin_vel.allFinite() || !out.base_ang_vel.allFinite() ||
      !out.anchor_pos.allFinite()) return fail("invalid body / anchor state values");
  out.base_quat.normalize();
  out.anchor_quat.normalize();
  return true;
}

bool DanceController::Ready(const char** reason) const {
  if (reason) *reason = "feedback ready; waiting for the continuous readiness hold";
  const auto fail = [reason](const char* message) {
    if (reason) *reason = message;
    return false;
  };
  Snapshot snapshot;
  if (!ReadSnapshot(snapshot, reason)) return false;
  if (motion_.empty()) return fail("dance reference motion is empty");
  if (snapshot.base_lin_vel.norm() > max_start_speed_) return fail("body linear speed exceeds max_start_speed");
  const float up = (snapshot.base_quat * Eigen::Vector3f::UnitZ()).z();
  if (std::acos(std::clamp(up, -1.0f, 1.0f)) * kRadToDeg > max_tilt_deg_) return fail("body tilt exceeds max_tilt_deg");
  for (int i = 0; i < kJoints; ++i) {
    if (std::abs(snapshot.joint_pos[i] - preparation_pos_[i]) > max_start_pose_error_) {
      return fail("a measured joint has not reached the preparation pose (max_start_pose_error)");
    }
    if (snapshot.joint_pos[i] < joint_conf_.pos_limit_lower(i) - joint_limit_tolerance_ ||
        snapshot.joint_pos[i] > joint_conf_.pos_limit_upper(i) + joint_limit_tolerance_) {
      return fail("a measured joint is outside its position limits");
    }
  }
  return true;
}

std::vector<double> DanceController::PreparationPositions() const {
  if (motion_.empty()) throw std::runtime_error("dance start pose is unavailable");
  return std::vector<double>(preparation_pos_.begin(), preparation_pos_.end());
}

void DanceController::RestartController() {
  Snapshot snapshot;
  if (!Ready() || !ReadSnapshot(snapshot)) {
    SetFault("required feedback missing at dance start");
    return;
  }
  current_ = snapshot;
  for (int i = 0; i < kJoints; ++i) {
    target_pos_[i] = static_cast<float>(snapshot.joint_pos[i]);
  }
  last_action_.fill(0.0f);
  tick_ = 0;
  frame_index_ = 0;
  faulted_ = false;
  finished_ = false;
  fault_reason_.clear();
  active_ = true;
}

void DanceController::AlignReference(const Snapshot& snapshot) {
  // Align world yaw only: a full 3-D alignment would rotate gravity and hide
  // the torso pitch/roll error from the actor.
  const auto yaw = [](const Eigen::Quaternionf& q) {
    const auto r = q.toRotationMatrix();
    return std::atan2(r(1, 0), r(0, 0));
  };
  reference_alignment_quat_ = Eigen::AngleAxisf(
      yaw(snapshot.anchor_quat) - yaw(motion_[0].anchor_quat), Eigen::Vector3f::UnitZ());
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
    if ((max_tracking_error_ > 0 &&
         std::abs(snapshot.joint_pos[i] - target_pos_[i]) > max_tracking_error_) ||
        snapshot.joint_pos[i] < joint_conf_.pos_limit_lower(i) - joint_limit_tolerance_ ||
        snapshot.joint_pos[i] > joint_conf_.pos_limit_upper(i) + joint_limit_tolerance_) {
      SetFault("dance joint tracking error or limit exceeded");
      return;
    }
  }
  const auto& reference = motion_[frame_index_ == 0 ? 0 : frame_index_ - 1];
  const Eigen::Vector3f expected_anchor = reference_alignment_pos_ +
      reference_alignment_quat_ * reference.anchor_pos;
  if (tick_ > 0 &&
      (expected_anchor - snapshot.anchor_pos).norm() > max_anchor_error_) {
    SetFault("dance anchor tracking error exceeded limit");
    return;
  }
  current_ = snapshot;
  // Guide hands over from the separately held dance pose with no settle phase;
  // target positions are held for 20 ticks, ankle PD is recomputed every tick.
  if (tick_ % decimation_ == 0) {
    if (tick_ == 0) {
      AlignReference(snapshot);
    }
    if (frame_index_ >= motion_.size()) {
      active_ = false;
      finished_ = true;
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
