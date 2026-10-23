#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include "control_module/dance_state_estimator.h"

namespace xyber_x1_infer::rl_control_module {

// Shared by the ROS controller and the native MuJoCo probe. No simulator state,
// external odometry, heap allocation or file I/O enters Update().
class DanceFinishCore {
 public:
  static constexpr size_t kJoints = 29;
  using Joints = std::array<double, kJoints>;
  struct Config {
    double blend_s{0.1}, upper_delay_s{1.0}, upper_blend_s{2.0};
    double landing_min_s{2.0}, landing_hold_s{0.75}, stand_blend_s{4.0}, verify_s{2.0};
    double max_tilt_deg{35.0}, quiet_tilt_deg{10.0}, quiet_gyro{0.1};
    double quiet_joint_speed{0.3}, quiet_base_speed{0.025};
    double contact_on{35.0}, contact_off{15.0}, max_foot_height_delta{0.03};
    double max_foot_velocity_delta{0.15};
    double upper_target_speed{2.0};  // rad/s, waist/arm/wrist target slew during takeover
    double force_filter_s{0.1};  // quasi-static load inversion amplifies torque noise
    double recovery_tilt_deg{12.0}, recovery_gyro{0.3}, recovery_speed{0.12}, recovery_delay_s{0.05};
    double max_stand_pose_error{0.12};
    Joints lower{}, upper{}, nominal{}, kp{}, kd{}, stand_kp{}, stand_kd{}, stand_pose{};
    std::array<size_t, 12> legs{};
  };
  struct Input {
    Joints q{}, qd{}, effort{};
    std::array<double, 4> orientation{1, 0, 0, 0};
    std::array<double, 3> gyro{};
    bool contact_input_valid{true};  // finite efforts, joint/IMU source times within 10 ms
    int64_t source_ns{};            // newest complete joint/IMU pair; must advance for gates
  };
  enum class Phase { kInactive, kBrake, kLand, kStandBlend, kStanding, kFault };

  explicit DanceFinishCore(const Config& cfg) : cfg_(cfg) {
    for (double v : {cfg.blend_s, cfg.upper_blend_s, cfg.landing_min_s, cfg.landing_hold_s,
                     cfg.stand_blend_s, cfg.verify_s, cfg.max_tilt_deg, cfg.quiet_tilt_deg,
                     cfg.quiet_gyro, cfg.quiet_joint_speed, cfg.quiet_base_speed, cfg.contact_on,
                     cfg.contact_off, cfg.max_foot_height_delta, cfg.max_foot_velocity_delta,
                     cfg.recovery_tilt_deg, cfg.recovery_gyro, cfg.recovery_speed, cfg.recovery_delay_s,
                     cfg.upper_target_speed, cfg.force_filter_s, cfg.max_stand_pose_error})
      if (!std::isfinite(v) || v <= 0) throw std::runtime_error("invalid dance finish parameter");
    if (!std::isfinite(cfg.upper_delay_s) || cfg.upper_delay_s < 0 || cfg.blend_s > 0.4 ||
        cfg.landing_min_s < cfg.blend_s || cfg.contact_on <= cfg.contact_off ||
        cfg.quiet_tilt_deg >= cfg.recovery_tilt_deg || cfg.recovery_tilt_deg >= cfg.max_tilt_deg ||
        cfg.max_tilt_deg >= 90) throw std::runtime_error("invalid dance finish gates");
    std::array<bool, kJoints> seen{};
    for (size_t j : cfg.legs) {
      if (j >= kJoints || seen[j]) throw std::runtime_error("invalid dance finish leg order");
      seen[j] = true;
    }
    for (size_t i = 0; i < kJoints; ++i) {
      if (!std::isfinite(cfg.lower[i]) || !std::isfinite(cfg.upper[i]) || cfg.lower[i] >= cfg.upper[i] ||
          !std::isfinite(cfg.nominal[i]) || cfg.nominal[i] < cfg.lower[i] || cfg.nominal[i] > cfg.upper[i] ||
          !std::isfinite(cfg.kp[i]) || !std::isfinite(cfg.kd[i]) || cfg.kp[i] <= 0 || cfg.kd[i] <= 0 ||
          !std::isfinite(cfg.stand_kp[i]) || !std::isfinite(cfg.stand_kd[i]) ||
          cfg.stand_kp[i] <= 0 || cfg.stand_kd[i] <= 0 || cfg.stand_kd[i] > 8 ||
          !std::isfinite(cfg.stand_pose[i]) || cfg.stand_pose[i] < cfg.lower[i] || cfg.stand_pose[i] > cfg.upper[i])
        throw std::runtime_error("invalid dance finish joint parameters");
    }
  }
  void Begin(const Joints& previous_targets, int64_t now_ns) {
    for (size_t i = 0; i < kJoints; ++i) {
      if (!std::isfinite(previous_targets[i]) || previous_targets[i] < cfg_.lower[i]-1e-5 ||
          previous_targets[i] > cfg_.upper[i]+1e-5) throw std::runtime_error("invalid dance handoff target");
    }
    target_ = start_target_ = previous_targets;
    kp_ = start_kp_ = cfg_.kp; kd_ = start_kd_ = cfg_.kd;
    phase_ = Phase::kBrake; start_ns_ = phase_ns_ = previous_ns_ = now_ns;
    source_ns_ = 0; quiet_s_ = bad_s_ = 0; contacts_ = {}; forces_ = {};
    load_initialized_ = false; confirmed_ = false; recoveries_ = 0;
    reason_ = "braking with zero-command balance policy";
  }
  void Update(const Input& in, const Joints& balance_targets, int64_t now_ns) {
    if (phase_ == Phase::kInactive || phase_ == Phase::kFault) return;
    if (now_ns < previous_ns_ || now_ns-previous_ns_ > 80000000) {
      Fault("finish control clock stalled or went backwards"); return;
    }
    const double dt = (now_ns-previous_ns_)*1e-9;
    previous_ns_ = now_ns;
    auto orientation = in.orientation;
    if (!dance_state_detail::Normalize(orientation)) { Fault("invalid finish orientation"); return; }
    for (double v : in.gyro) if (!std::isfinite(v)) { Fault("invalid finish gyro"); return; }
    double joint_speed = 0, stand_error = 0;
    for (size_t j = 0; j < kJoints; ++j) {
      if (!std::isfinite(in.q[j]) || !std::isfinite(in.qd[j]) || !std::isfinite(balance_targets[j])) {
        Fault("invalid finish joint feedback or balance output"); return;
      }
      joint_speed = std::max(joint_speed, std::abs(in.qd[j]));
      stand_error = std::max(stand_error, std::abs(in.q[j]-cfg_.stand_pose[j]));
    }
    const auto up = dance_state_detail::Rotate(orientation, {0, 0, 1});
    tilt_deg_ = std::acos(std::clamp(up[2], -1.0, 1.0))*180/std::acos(-1.0);
    gyro_speed_ = dance_estimation_detail::Norm(in.gyro);
    if (tilt_deg_ > cfg_.max_tilt_deg) { Fault("finish tilt exceeded limit"); return; }
    bool contact_valid = in.contact_input_valid;
    std::array<double, 12> q{}, qd{}, effort{};
    for (size_t j = 0; j < 12; ++j) {
      const auto i = cfg_.legs[j]; q[j] = in.q[i]; qd[j] = in.qd[i]; effort[j] = in.effort[i];
      contact_valid = contact_valid && std::isfinite(effort[j]);
    }
    bool support = false;
    if (contact_valid) {
      const auto left = dance_estimation_detail::Kinematics(0, q, qd, effort, orientation, in.gyro);
      const auto right = dance_estimation_detail::Kinematics(1, q, qd, effort, orientation, in.gyro);
      const double alpha = load_initialized_ ? std::clamp(dt/cfg_.force_filter_s, 0.0, 1.0) : 1.0;
      const std::array<double, 2> raw{left.vertical_force, right.vertical_force};
      for (size_t side = 0; side < 2; ++side) {
        forces_[side] += alpha*(raw[side]-forces_[side]);
        contacts_[side] = forces_[side] >= (contacts_[side] ? cfg_.contact_off : cfg_.contact_on);
      }
      load_initialized_ = true;
      const auto velocity_delta = dance_estimation_detail::Sub(left.point_velocity, right.point_velocity);
      velocity_ = dance_estimation_detail::Scale(
          dance_estimation_detail::Add(left.point_velocity, right.point_velocity), -0.5);
      support = contacts_[0] && contacts_[1] &&
          std::abs(left.point[2]-left.radius-right.point[2]+right.radius) <= cfg_.max_foot_height_delta &&
          dance_estimation_detail::Norm(velocity_delta) <= cfg_.max_foot_velocity_delta;
    } else {
      contacts_ = {}; load_initialized_ = false; velocity_ = {};
    }
    const double base_speed = dance_estimation_detail::Norm(velocity_);
    quiet_ = support && tilt_deg_ <= cfg_.quiet_tilt_deg && gyro_speed_ <= cfg_.quiet_gyro &&
        joint_speed <= cfg_.quiet_joint_speed && base_speed <= cfg_.quiet_base_speed;
    if (phase_ == Phase::kStanding && stand_error > cfg_.max_stand_pose_error) quiet_ = false;
    double sensor_dt = 0;
    if (in.source_ns > source_ns_) {
      if (source_ns_ && in.source_ns-source_ns_ <= 10000000) sensor_dt = (in.source_ns-source_ns_)*1e-9;
      else quiet_s_ = 0;
      source_ns_ = in.source_ns;
    } else if (in.source_ns < source_ns_) quiet_s_ = 0;
    if (quiet_) quiet_s_ += sensor_dt; else quiet_s_ = 0;
    const double elapsed = (now_ns-start_ns_)*1e-9;
    if (phase_ == Phase::kBrake && (now_ns-phase_ns_)*1e-9 >= cfg_.blend_s) phase_ = Phase::kLand;
    if (phase_ == Phase::kLand && elapsed >= cfg_.landing_min_s &&
        elapsed >= cfg_.upper_delay_s+cfg_.upper_blend_s && quiet_s_ >= cfg_.landing_hold_s) {
      phase_ = Phase::kStandBlend; phase_ns_ = now_ns; quiet_s_ = 0;
    }
    if (phase_ == Phase::kStandBlend || phase_ == Phase::kStanding) {
      const bool disturbed = tilt_deg_ > cfg_.recovery_tilt_deg || gyro_speed_ > cfg_.recovery_gyro ||
          !support || base_speed > cfg_.recovery_speed;
      bad_s_ = disturbed ? bad_s_+dt : 0;
      if (bad_s_ >= cfg_.recovery_delay_s) {
        start_target_ = target_; start_kp_ = kp_; start_kd_ = kd_; start_ns_ = phase_ns_ = now_ns;
        phase_ = Phase::kBrake; quiet_s_ = bad_s_ = 0; confirmed_ = false; ++recoveries_;
      }
    }
    const double phase_elapsed = (now_ns-phase_ns_)*1e-9;
    const double upper_alpha = Smooth((now_ns-start_ns_)*1e-9-cfg_.upper_delay_s, cfg_.upper_blend_s);
    const double enter_alpha = phase_ == Phase::kBrake ? Smooth(phase_elapsed, cfg_.blend_s) : 1;
    const double stand_alpha = phase_ == Phase::kStandBlend ? Smooth(phase_elapsed, cfg_.stand_blend_s) :
                              phase_ == Phase::kStanding ? 1 : 0;
    for (size_t j = 0; j < kJoints; ++j) {
      const bool leg = std::find(cfg_.legs.begin(), cfg_.legs.end(), j) != cfg_.legs.end();
      double balance = std::clamp(balance_targets[j], cfg_.lower[j], cfg_.upper[j]);
      if (!leg) balance = (1-upper_alpha)*balance+upper_alpha*cfg_.nominal[j];
      const double entering = (1-enter_alpha)*start_target_[j]+enter_alpha*balance;
      // Retain feedback balance authority while gradually moving to the B-key
      // stand pose. A frozen start target removes stabilizing corrections too early.
      double desired = (1-stand_alpha)*entering+stand_alpha*cfg_.stand_pose[j];
      if (!leg) desired = std::clamp(desired, target_[j]-cfg_.upper_target_speed*dt,
                                             target_[j]+cfg_.upper_target_speed*dt);
      target_[j] = std::clamp(desired, cfg_.lower[j], cfg_.upper[j]);
      const double entering_kp = (1-enter_alpha)*start_kp_[j]+enter_alpha*cfg_.kp[j];
      const double entering_kd = (1-enter_alpha)*start_kd_[j]+enter_alpha*cfg_.kd[j];
      kp_[j] = (1-stand_alpha)*entering_kp+stand_alpha*cfg_.stand_kp[j];
      kd_[j] = (1-stand_alpha)*entering_kd+stand_alpha*cfg_.stand_kd[j];
    }
    if (phase_ == Phase::kStandBlend && stand_alpha >= 1) {
      phase_ = Phase::kStanding; phase_ns_ = now_ns; quiet_s_ = 0;
    }
    if (phase_ == Phase::kStanding) confirmed_ = quiet_s_ >= cfg_.verify_s;
    if (!contact_valid) reason_ = "waiting for synchronized finite leg torques; balance remains active";
    else if (!support) reason_ = "waiting for two consistent loaded feet";
    else if (phase_ == Phase::kStanding && stand_error > cfg_.max_stand_pose_error) reason_ = "waiting to reach configured stand pose";
    else if (!quiet_) reason_ = "waiting for low tilt and body/joint speeds";
    else if (confirmed_) reason_ = "standing continuously verified; active monitoring remains on";
    else reason_ = "continuous quiet support hold";
  }
  void Fault(const char* reason) { phase_ = Phase::kFault; confirmed_ = false; reason_ = reason; }
  Phase GetPhase() const { return phase_; }
  bool Active() const { return phase_ != Phase::kInactive; }
  bool Faulted() const { return phase_ == Phase::kFault; }
  bool StandingConfirmed() const { return confirmed_; }
  const char* Reason() const { return reason_; }
  const char* PhaseName() const {
    switch (phase_) {
      case Phase::kBrake: return "finish_brake";
      case Phase::kLand: return "finish_land";
      case Phase::kStandBlend: return "finish_stand_blend";
      case Phase::kStanding: return "finish_standing";
      case Phase::kFault: return "finish_fault";
      default: return "finish_inactive";
    }
  }
  const Joints& Targets() const { return target_; }
  const Joints& Kp() const { return kp_; }
  const Joints& Kd() const { return kd_; }
  const std::array<double, 2>& Forces() const { return forces_; }
  const std::array<bool, 2>& Contacts() const { return contacts_; }
  const std::array<double, 3>& Velocity() const { return velocity_; }
  double QuietSeconds() const { return quiet_s_; }
  double TiltDegrees() const { return tilt_deg_; }
  size_t Recoveries() const { return recoveries_; }
 private:
  static double Smooth(double t, double duration) {
    const double u = std::clamp(t/duration, 0.0, 1.0);
    return u*u*u*(10+u*(-15+6*u));
  }
  Config cfg_;
  Phase phase_{Phase::kInactive};
  Joints start_target_{}, start_kp_{}, start_kd_{}, target_{}, kp_{}, kd_{};
  std::array<double, 2> forces_{};
  std::array<bool, 2> contacts_{};
  std::array<double, 3> velocity_{};
  int64_t start_ns_{}, phase_ns_{}, previous_ns_{}, source_ns_{};
  double quiet_s_{}, bad_s_{}, tilt_deg_{}, gyro_speed_{};
  bool quiet_{}, load_initialized_{}, confirmed_{};
  size_t recoveries_{};
  const char* reason_{"inactive"};
};

}  // namespace xyber_x1_infer::rl_control_module
