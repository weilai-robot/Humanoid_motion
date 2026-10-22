#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace xyber_x1_infer::rl_control_module {

// One control-thread producer, one file worker. Start/Stop require a quiescent
// producer. No allocation, formatting, file I/O or waiting in Enqueue.
class DanceDiagnostics {
 public:
  static constexpr size_t kJoints = 29, kObservations = 160;
  enum Field : size_t {
    kImuQuat = 0, kImuGyro = kImuQuat+4, kImuAccel = kImuGyro+3,
    kBodyPosition = kImuAccel+3, kBodyQuat = kBodyPosition+3, kBodyVelocity = kBodyQuat+4,
    kAnchorPosition = kBodyVelocity+3, kAnchorQuat = kAnchorPosition+3,
    kReferencePosition = kAnchorQuat+4, kReferenceQuat = kReferencePosition+3,
    kAnchorError = kReferenceQuat+4, kAnchorErrorNorm = kAnchorError+3,
    kAccelerationWorld = kAnchorErrorNorm+1, kAccelBias = kAccelerationWorld+3,
    kPositionVariance = kAccelBias+3, kVelocityVariance = kPositionVariance+3,
    kContactForce = kVelocityVariance+3, kContact = kContactForce+2,
    kJointPosition = kContact+2, kJointVelocity = kJointPosition+kJoints,
    kJointEffort = kJointVelocity+kJoints, kObservationRaw = kJointEffort+kJoints,
    kObservationNormalized = kObservationRaw+kObservations,
    kAction = kObservationNormalized+kObservations, kTarget = kAction+kJoints,
    kCommandPosition = kTarget+kJoints, kCommandEffort = kCommandPosition+kJoints,
    kCommandKp = kCommandEffort+kJoints, kCommandKd = kCommandKp+kJoints,
    kValueCount = kCommandKd+kJoints
  };
  struct Frame {
    int64_t wall_ns{}, steady_ns{}, joint_stamp_ns{}, imu_stamp_ns{}, body_stamp_ns{};
    int64_t anchor_stamp_ns{}, velocity_stamp_ns{}, policy_frame{-1};
    uint64_t session{}, tick{}, sequence{}, dropped{};
    bool snapshot_valid{}, has_observation{}, has_action{}, command_valid{}, estimator_valid{};
    std::array<char, 24> state{}, event{}, state_source{};
    std::array<char, 160> reason{};
    std::array<double, kValueCount> values;
    Frame() { values.fill(std::numeric_limits<double>::quiet_NaN()); }
  };

  ~DanceDiagnostics() { Stop(); }
  DanceDiagnostics() = default;
  DanceDiagnostics(const DanceDiagnostics&) = delete;
  DanceDiagnostics& operator=(const DanceDiagnostics&) = delete;

  template <size_t N>
  static void Text(std::array<char, N>& out, std::string_view text) {
    const auto n = std::min(text.size(), N-1);
    std::copy_n(text.data(), n, out.data()); out[n] = '\0';
  }
  static void Stamp(Frame& frame) {
    frame.wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  bool Start(const std::string& directory, const std::vector<std::string>& joints,
             int flush_ms = 100, size_t capacity = 256) {
    Stop();
    if (joints.size() != kJoints || flush_ms < 10 || flush_ms > 1000 || capacity < 2) return false;
    try {
      std::filesystem::create_directories(directory);
      Frame stamp; Stamp(stamp);
      path_ = (std::filesystem::path(directory)/("dance_diag_"+std::to_string(stamp.wall_ns)+".csv")).string();
      frames_ = std::make_unique<std::vector<Frame>>(capacity);
      read_.store(0); write_.store(0); dropped_.store(0); sequence_ = 0; io_error_.store(false);
      file_.open(path_, std::ios::out | std::ios::trunc);
      file_ << std::setprecision(17);
      const auto columns = Columns(joints);
      for (size_t i = 0; i < columns.size(); ++i) {
        if (i) file_ << ',';
        Quoted(columns[i]);
      }
      file_ << '\n'; file_.flush();
      if (!file_) { file_.close(); return false; }
      flush_interval_ = std::chrono::milliseconds(flush_ms);
      running_.store(true, std::memory_order_release);
      worker_ = std::thread([this] { Work(); });
      return true;
    } catch (...) {
      running_.store(false);
      if (file_.is_open()) file_.close();
      return false;
    }
  }
  bool Enqueue(const Frame& frame) noexcept {
    if (!running_.load(std::memory_order_acquire)) return false;
    const auto sequence = sequence_++;
    const size_t write = write_.load(std::memory_order_relaxed);
    const size_t next = (write+1)%frames_->size();
    if (next == read_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed); return false;
    }
    auto& target = (*frames_)[write]; target = frame;
    target.sequence = sequence; target.dropped = dropped_.load(std::memory_order_relaxed);
    write_.store(next, std::memory_order_release);
    return true;
  }
  void Stop() {
    running_.store(false, std::memory_order_release);
    if (worker_.joinable()) worker_.join();
    if (file_.is_open()) file_.close();
  }
  bool Enabled() const { return running_.load(std::memory_order_acquire); }
  bool IoError() const { return io_error_.load(std::memory_order_acquire); }
  uint64_t Dropped() const { return dropped_.load(std::memory_order_relaxed); }
  const std::string& Path() const { return path_; }

  static std::vector<std::string> Columns(const std::vector<std::string>& joints) {
    std::vector<std::string> result{
      "schema_version", "wall_ns", "steady_ns", "session", "event", "state", "reason", "state_source",
      "control_tick", "policy_frame", "motion_time_s", "snapshot_valid", "has_observation", "has_action",
      "command_valid", "estimator_valid", "sequence", "dropped_total", "joint_stamp_ns", "imu_stamp_ns",
      "body_stamp_ns", "anchor_stamp_ns", "velocity_stamp_ns"};
    auto group = [&](std::string_view prefix, const auto& suffixes) {
      for (const auto& suffix : suffixes) result.emplace_back(std::string(prefix)+"_"+suffix);
    };
    const std::array<std::string, 3> xyz{"x", "y", "z"};
    const std::array<std::string, 4> quat{"w", "x", "y", "z"};
    const std::array<std::string, 2> sides{"left", "right"};
    group("imu_quat", quat); group("imu_gyro", xyz); group("imu_accel", xyz);
    group("body_position_world", xyz); group("body_quat", quat); group("base_velocity_body", xyz);
    group("anchor_position_world", xyz); group("anchor_quat", quat);
    group("reference_position_world", xyz); group("reference_quat", quat);
    group("anchor_error_world", xyz); result.emplace_back("anchor_error_norm");
    group("est_acceleration_world", xyz); group("est_accel_bias_world", xyz);
    group("est_position_variance_world", xyz); group("est_velocity_variance_world", xyz);
    group("est_vertical_force", sides); group("est_contact", sides);
    group("joint_position", joints); group("joint_velocity", joints); group("joint_effort", joints);
    for (const auto prefix : {"obs_raw_", "obs_normalized_"})
      for (size_t i = 0; i < kObservations; ++i) result.emplace_back(std::string(prefix)+std::to_string(i));
    group("action", joints); group("target_policy", joints);
    group("command_position", joints); group("command_effort", joints);
    group("command_kp", joints); group("command_kd", joints);
    return result;
  }

 private:
  void Quoted(std::string_view value) {
    file_ << '"';
    for (char c : value) { if (c == '"') file_ << '"'; file_ << c; }
    file_ << '"';
  }
  void Row(const Frame& f) {
    file_ << 1 << ',' << f.wall_ns << ',' << f.steady_ns << ',' << f.session << ',';
    Quoted(f.event.data()); file_ << ','; Quoted(f.state.data()); file_ << ',';
    Quoted(f.reason.data()); file_ << ','; Quoted(f.state_source.data());
    file_ << ',' << f.tick << ',' << f.policy_frame << ',';
    if (f.policy_frame >= 0) file_ << f.policy_frame/50.0; else file_ << "nan";
    file_ << ',' << f.snapshot_valid << ',' << f.has_observation << ',' << f.has_action
          << ',' << f.command_valid << ',' << f.estimator_valid << ',' << f.sequence << ',' << f.dropped
          << ',' << f.joint_stamp_ns << ',' << f.imu_stamp_ns << ',' << f.body_stamp_ns
          << ',' << f.anchor_stamp_ns << ',' << f.velocity_stamp_ns;
    for (double value : f.values) file_ << ',' << value;
    file_ << '\n';
  }
  void Work() noexcept {
    try {
      auto last_flush = std::chrono::steady_clock::now();
      while (running_.load(std::memory_order_acquire) ||
             read_.load(std::memory_order_relaxed) != write_.load(std::memory_order_acquire)) {
        size_t read = read_.load(std::memory_order_relaxed);
        const size_t limit = write_.load(std::memory_order_acquire);
        while (read != limit) {
          Row((*frames_)[read]);
          read = (read+1)%frames_->size();
          read_.store(read, std::memory_order_release);
        }
        if (std::chrono::steady_clock::now()-last_flush >= flush_interval_) {
          file_.flush(); last_flush = std::chrono::steady_clock::now();
          if (!file_) { io_error_.store(true); running_.store(false); return; }
        }
        if (running_.load(std::memory_order_acquire)) std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      Frame end; Stamp(end); Text(end.event, "logger_stop");
      end.sequence = sequence_; end.dropped = Dropped(); Row(end); file_.flush();
      if (!file_) io_error_.store(true);
    } catch (...) { io_error_.store(true); running_.store(false); }
  }
  std::unique_ptr<std::vector<Frame>> frames_;
  std::atomic<size_t> read_{0}, write_{0};
  std::atomic<uint64_t> dropped_{0};
  uint64_t sequence_{};  // producer-owned; worker reads only after producer quiesces
  std::atomic_bool running_{false}, io_error_{false};
  std::thread worker_;
  std::ofstream file_;
  std::string path_;
  std::chrono::milliseconds flush_interval_{100};
};

}  // namespace xyber_x1_infer::rl_control_module
