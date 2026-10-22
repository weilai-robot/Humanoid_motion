#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace xyber_x1_infer::rl_control_module {

// Bounded, allocation-free history for FK at the estimator's source time.
// Callers serialize access with the same mutex used for joint feedback.
class DanceWaistHistory {
 public:
  using Positions = std::array<double, 3>;  // yaw, roll, pitch, in model coordinates

  bool Push(int64_t stamp_ns, const Positions& positions) {
    if (stamp_ns <= 0 || (count_ && stamp_ns <= samples_[(next_ + kCapacity - 1) % kCapacity].stamp_ns)) {
      return false;
    }
    for (double q : positions) if (!std::isfinite(q)) return false;
    samples_[next_] = {stamp_ns, positions};
    next_ = (next_ + 1) % kCapacity;
    if (count_ < kCapacity) ++count_;
    return true;
  }

  bool SampleAt(int64_t stamp_ns, Positions& positions) const {
    if (stamp_ns <= 0) return false;
    const Sample* newer = nullptr;
    for (size_t age = 0; age < count_; ++age) {
      const auto& sample = samples_[(next_ + kCapacity - 1 - age) % kCapacity];
      if (sample.stamp_ns == stamp_ns) {
        positions = sample.positions;
        return true;
      }
      if (sample.stamp_ns < stamp_ns) {
        if (!newer || newer->stamp_ns - sample.stamp_ns > kMaxInterpolationGapNs) return false;
        const double alpha = static_cast<double>(stamp_ns - sample.stamp_ns) /
                             static_cast<double>(newer->stamp_ns - sample.stamp_ns);
        for (size_t i = 0; i < positions.size(); ++i) {
          positions[i] = sample.positions[i] + alpha * (newer->positions[i] - sample.positions[i]);
        }
        return true;
      }
      newer = &sample;
    }
    return false;  // No extrapolation or filling missing joint samples with zero.
  }

 private:
  static constexpr size_t kCapacity = 256;
  static constexpr int64_t kMaxInterpolationGapNs = 10000000;  // 10 ms
  struct Sample { int64_t stamp_ns{}; Positions positions{}; };
  std::array<Sample, kCapacity> samples_{};
  size_t next_{};
  size_t count_{};
};

namespace dance_state_detail {
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w, x, y, z

inline Quat Multiply(const Quat& a, const Quat& b) {
  return {a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
          a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
          a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
          a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]};
}

inline bool Normalize(Quat& q) {
  double squared_norm = 0;
  for (double value : q) {
    if (!std::isfinite(value)) return false;
    squared_norm += value * value;
  }
  if (!std::isfinite(squared_norm) || squared_norm < 1e-12) return false;
  const double norm = std::sqrt(squared_norm);
  for (double& value : q) value /= norm;
  return true;
}

inline Vec3 Rotate(const Quat& q, const Vec3& v) {
  const Vec3 cross{q[2]*v[2] - q[3]*v[1], q[3]*v[0] - q[1]*v[2], q[1]*v[1] - q[2]*v[0]};
  const Vec3 twice_cross{q[2]*cross[2] - q[3]*cross[1],
                        q[3]*cross[0] - q[1]*cross[2], q[1]*cross[1] - q[2]*cross[0]};
  return {v[0] + 2*(q[0]*cross[0] + twice_cross[0]),
          v[1] + 2*(q[0]*cross[1] + twice_cross[1]),
          v[2] + 2*(q[0]*cross[2] + twice_cross[2])};
}

inline Quat RotationZ(double angle) { return {std::cos(angle/2), 0, 0, std::sin(angle/2)}; }
}  // namespace dance_state_detail

struct DanceAnchorState {
  std::array<double, 3> position{};
  std::array<double, 4> orientation{};  // w, x, y, z
};

// base_link -> lumbar_pitch_link from the v11 training X1 MJCF/URDF.
// Fixed rotations are essential: the three joint-local axes are Z, Z, -Z.
// Hardware direction/offsets have already been applied to these joint angles.
inline bool ComputeX1DanceAnchor(const std::array<double, 3>& base_position,
                                std::array<double, 4> base_orientation,
                                const DanceWaistHistory::Positions& waist,
                                DanceAnchorState& out) {
  using namespace dance_state_detail;
  for (double value : base_position) if (!std::isfinite(value)) return false;
  for (double value : waist) if (!std::isfinite(value)) return false;
  if (!Normalize(base_orientation)) return false;
  Quat yaw_origin{1, 0, 0, -2.274557e-05};
  Normalize(yaw_origin);
  const auto yaw = Multiply(Multiply(base_orientation, yaw_origin), RotationZ(waist[0]));
  const auto yaw_offset = Rotate(base_orientation, {0.00245, 0, 0.11553403});
  const auto roll_offset = Rotate(yaw, {0, 0, 0.0405});
  for (size_t i = 0; i < 3; ++i) out.position[i] = base_position[i] + yaw_offset[i] + roll_offset[i];
  const auto roll = Multiply(Multiply(yaw, {0.5, 0.5, -0.5, -0.5}), RotationZ(waist[1]));
  const double half_sqrt = std::sqrt(0.5);
  out.orientation = Multiply(Multiply(roll, {half_sqrt, 0, half_sqrt, 0}), RotationZ(-waist[2]));
  return Normalize(out.orientation);
}

}  // namespace xyber_x1_infer::rl_control_module
