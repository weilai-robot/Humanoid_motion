#pragma once
#include "control_module/dance_state_estimator.h"

namespace xyber_x1_infer::rl_control_module {
// Caller serializes access. Process IMU source times only when leg feedback
// brackets them. Arrival order is irrelevant; never extrapolate or reuse a stamp.
class DanceEstimatorInputs {
 public:
  using Input = DanceBodyStateEstimator::Input;
  using Joints = std::array<double, 36>;  // q12, qd12, effort12, all model coordinates
  bool PushJoints(int64_t stamp, const Joints& values) {
    if (stamp <= 0 || (joint_count_ && stamp <= joints_[(joint_next_+255)%256].stamp)) return false;
    for (double v : values) if (!std::isfinite(v)) return false;
    joints_[joint_next_] = {stamp, values};
    joint_next_ = (joint_next_+1)%256;
    joint_count_ = std::min(joint_count_+1, size_t{256});
    return true;
  }
  bool PushImu(const Input& input) {
    if (input.stamp_ns <= 0 || (imu_count_ && input.stamp_ns <= imu_[(imu_next_+127)%128].stamp_ns)) return false;
    imu_[imu_next_] = input;
    imu_next_ = (imu_next_+1)%128;
    imu_count_ = std::min(imu_count_+1, size_t{128});
    return true;
  }
  bool Next(Input& out) {
    for (size_t age = imu_count_; age > 0; --age) {
      const auto& sample = imu_[(imu_next_+128-age)%128];
      if (sample.stamp_ns <= consumed_) continue;
      const JointSample* newer = nullptr;
      for (size_t j = 0; j < joint_count_; ++j) {
        const auto& joint = joints_[(joint_next_+255-j)%256];
        if (joint.stamp > sample.stamp_ns) { newer = &joint; continue; }
        Joints values = joint.values;
        if (joint.stamp != sample.stamp_ns) {
          if (!newer) return false;  // wait for the right bracketing sample
          if (newer->stamp-joint.stamp > 10000000) break;
          const double alpha = double(sample.stamp_ns-joint.stamp)/double(newer->stamp-joint.stamp);
          for (size_t i = 0; i < values.size(); ++i) values[i] += alpha*(newer->values[i]-values[i]);
        }
        out = sample;
        std::copy_n(values.begin(), 12, out.position.begin());
        std::copy_n(values.begin()+12, 12, out.velocity.begin());
        std::copy_n(values.begin()+24, 12, out.effort.begin());
        consumed_ = sample.stamp_ns;
        return true;
      }
      // If the IMU predates the oldest retained joint sample, it cannot pair.
      if (joint_count_ && joints_[(joint_next_+256-joint_count_)%256].stamp > sample.stamp_ns) consumed_ = sample.stamp_ns;
    }
    return false;
  }
 private:
  struct JointSample { int64_t stamp{}; Joints values{}; };
  std::array<JointSample, 256> joints_{};
  std::array<Input, 128> imu_{};
  size_t joint_next_{}, joint_count_{}, imu_next_{}, imu_count_{};
  int64_t consumed_{};
};
}  // namespace xyber_x1_infer::rl_control_module
