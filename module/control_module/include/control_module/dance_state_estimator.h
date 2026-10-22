#pragma once

#include <algorithm>
#include <limits>
#include "control_module/dance_leg_model.h"

namespace xyber_x1_infer::rl_control_module {
namespace dance_estimation_detail {
using Vec = std::array<double, 3>;
using Quat = std::array<double, 4>;
inline Vec Add(const Vec& a, const Vec& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
inline Vec Sub(const Vec& a, const Vec& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
inline Vec Scale(const Vec& a, double s) { return {a[0]*s, a[1]*s, a[2]*s}; }
inline Vec Cross(const Vec& a, const Vec& b) {
  return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
inline double Norm(const Vec& a) { return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); }
inline bool Finite(const Vec& a) {
  return std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]);
}
using dance_state_detail::Multiply;
using dance_state_detail::Rotate;
using dance_state_detail::Normalize;

struct Leg {
  Vec origin{}, point{}, point_velocity{};
  std::array<Vec, 6> origins{}, axes{}, com{}, jacobian{};
  Quat orientation{1, 0, 0, 0};
  size_t corner{};
  double radius{}, vertical_force{};
};

// All outputs are relative to the base origin, with vectors expressed in world
// axes. Joint axes and fixed rotations come from the actual v11 training MJCF.
inline Leg Kinematics(size_t side, const std::array<double, 12>& q,
                      const std::array<double, 12>& qd, const std::array<double, 12>& torque,
                      const Quat& base_orientation, const Vec& gyro) {
  Leg leg;
  auto rotation = base_orientation;
  Vec position{};
  for (size_t j = 0; j < 6; ++j) {
    const auto& link = kDanceLegLinks[side][j];
    position = Add(position, Rotate(rotation, link.translation));
    auto fixed = link.rotation;
    Normalize(fixed);
    rotation = Multiply(rotation, fixed);
    leg.origins[j] = position;
    leg.axes[j] = Rotate(rotation, link.axis);
    const double half = q[side*6+j]/2;
    rotation = Multiply(rotation, {std::cos(half), link.axis[0]*std::sin(half),
                                  link.axis[1]*std::sin(half), link.axis[2]*std::sin(half)});
    leg.com[j] = Add(position, Rotate(rotation, link.com));
  }
  leg.origin = position;
  leg.orientation = rotation;
  double lowest = std::numeric_limits<double>::infinity();
  for (size_t c = 0; c < 4; ++c) {
    const auto& point = kDanceFootPoints[side][c];
    const auto p = Add(position, Rotate(rotation, point.position));
    if (p[2]-point.radius < lowest) {
      lowest = p[2]-point.radius;
      leg.point = p;
      leg.corner = c;
      leg.radius = point.radius;
    }
  }
  leg.point_velocity = Cross(Rotate(base_orientation, gyro), leg.point);
  // Quasi-static leg load observer: J^T f = gravity_torque - motor_torque.
  // This is a contact cue, not a measured foot force or a dynamics observer.
  std::array<std::array<double, 3>, 3> normal{};
  Vec rhs{};
  for (size_t j = 0; j < 6; ++j) {
    const auto jac = Cross(leg.axes[j], Sub(leg.point, leg.origins[j]));
    leg.jacobian[j] = jac;
    leg.point_velocity = Add(leg.point_velocity, Scale(jac, qd[side*6+j]));
    double gravity = 0;
    for (size_t k = j; k < 6; ++k) {
      gravity += kDanceLegLinks[side][k].mass * 9.81 * Cross(leg.axes[j], Sub(leg.com[k], leg.origins[j]))[2];
    }
    for (size_t a = 0; a < 3; ++a) {
      rhs[a] += jac[a] * (gravity-torque[side*6+j]);
      for (size_t b = 0; b < 3; ++b) normal[a][b] += jac[a]*jac[b];
    }
  }
  for (size_t i = 0; i < 3; ++i) normal[i][i] += 1e-4;  // damp near straight-leg singularities
  // SPD Cholesky solve; fixed sizes and no heap allocation.
  std::array<std::array<double, 3>, 3> l{};
  for (size_t i = 0; i < 3; ++i) for (size_t j = 0; j <= i; ++j) {
    double v = normal[i][j];
    for (size_t k = 0; k < j; ++k) v -= l[i][k]*l[j][k];
    l[i][j] = i == j ? std::sqrt(std::max(v, 1e-12)) : v/l[j][j];
  }
  Vec y{}, force{};
  for (size_t i = 0; i < 3; ++i) {
    y[i] = rhs[i];
    for (size_t j = 0; j < i; ++j) y[i] -= l[i][j]*y[j];
    y[i] /= l[i][i];
  }
  for (int i = 2; i >= 0; --i) {
    force[i] = y[i];
    for (int j = i+1; j < 3; ++j) force[i] -= l[j][i]*force[j];
    force[i] /= l[i][i];
  }
  leg.vertical_force = force[2];
  return leg;
}
}  // namespace dance_estimation_detail

class DanceBodyStateEstimator {
 public:
  using Vec = dance_estimation_detail::Vec;
  using Quat = dance_estimation_detail::Quat;
  struct Config {
    double acceleration_scale{1.0};  // input must be specific force, SI m/s^2
    Vec imu_position_in_base{0, 0, 0};
    double initialization_s{0.5};
    double contact_on_force{35}, contact_off_force{15};
    double acceleration_noise{0.1}, bias_noise{0.01};
    double position_noise{0.03}, velocity_noise{0.3}, height_noise{0.01};
    double max_gap_s{0.01}, max_flight_s{0.35};
    double contact_velocity_limit{0.25};
  };
  struct Input {
    int64_t stamp_ns{};
    Quat orientation{1, 0, 0, 0};  // base -> world, same yaw as the policy IMU
    Vec gyro{}, acceleration{};
    std::array<double, 12> position{}, velocity{}, effort{};
  };
  struct Output {
    int64_t stamp_ns{};
    Vec position{}, velocity_body{}, position_variance{}, velocity_variance{};
    Quat orientation{1, 0, 0, 0};
    std::array<double, 2> vertical_force{};
    std::array<bool, 2> contact{};
  };

  DanceBodyStateEstimator() = default;
  explicit DanceBodyStateEstimator(const Config& config) : config_(config) {}
  void Reset() {
    initialized_ = false;
    last_stamp_ = 0;
    initialization_elapsed_ = 0;
    initial_bias_ = {};
    contact_ = {};
    force_ = {};
    no_contact_s_ = 0;
    problem_ = "waiting for stationary loaded feet and IMU calibration";
  }
  const char* Problem() const { return problem_; }
  bool Initialized() const { return initialized_; }

  bool Update(Input input, Output& out) {
    using namespace dance_estimation_detail;
    if (input.stamp_ns <= last_stamp_ || input.stamp_ns <= 0) return false;
    if (!Normalize(input.orientation) || !Finite(input.gyro) || !Finite(input.acceleration)) return Invalid();
    for (size_t i = 0; i < 12; ++i) {
      if (!std::isfinite(input.position[i]) || !std::isfinite(input.velocity[i]) ||
          !std::isfinite(input.effort[i])) return Invalid();
    }
    double dt = last_stamp_ ? (input.stamp_ns-last_stamp_)*1e-9 : 0;
    if (dt > config_.max_gap_s) {
      Reset();  // a fresh stream must recalibrate; never integrate across a dropout
      problem_ = "sensor gap exceeded estimator limit; recalibrating";
      dt = 0;
    }
    last_stamp_ = input.stamp_ns;
    auto acceleration = Scale(input.acceleration, config_.acceleration_scale);
    const auto alpha = dt > 0 ? Scale(Sub(input.gyro, previous_gyro_), 1/dt) : Vec{};
    previous_gyro_ = input.gyro;
    acceleration = Sub(acceleration, Add(Cross(alpha, config_.imu_position_in_base),
                         Cross(input.gyro, Cross(input.gyro, config_.imu_position_in_base))));
    acceleration = Sub(Rotate(input.orientation, acceleration), {0, 0, 9.81});
    std::array<Leg, 2> legs;
    for (size_t side = 0; side < 2; ++side) {
      legs[side] = Kinematics(side, input.position, input.velocity, input.effort, input.orientation, input.gyro);
      const double a = dt/(0.015+dt);
      force_[side] += a*(legs[side].vertical_force-force_[side]);
    }
    if (!initialized_) {
      double max_speed = 0;
      for (double v : input.velocity) max_speed = std::max(max_speed, std::abs(v));
      if (Norm(acceleration) > 1.0 || Norm(input.gyro) > 0.2 || max_speed > 0.3 ||
          std::max(force_[0], force_[1]) < config_.contact_on_force) {
        initialization_elapsed_ = 0;
        initial_bias_ = {};
        problem_ = "calibration requires stationary joints, SI accelerometer and loaded feet";
        return false;
      }
      initial_bias_ = Add(initial_bias_, Scale(acceleration, dt));
      initialization_elapsed_ += dt;
      if (initialization_elapsed_ < config_.initialization_s) return false;
      const double z = -std::min(legs[0].point[2]-legs[0].radius, legs[1].point[2]-legs[1].radius);
      for (size_t axis = 0; axis < 3; ++axis) {
        filters_[axis] = {};
        auto& f = filters_[axis];
        f.x[0] = axis == 2 ? z : 0;
        f.x[2] = initial_bias_[axis]/initialization_elapsed_;
        f.p[0][0] = 1e-4;
        f.p[1][1] = 0.01;
        f.p[2][2] = 0.01;
        for (size_t side = 0; side < 2; ++side) {
          f.x[3+side] = f.x[0]+legs[side].point[axis];
          f.p[3+side][3+side] = 1e-4;
        }
      }
      initialized_ = true;
      for (size_t side = 0; side < 2; ++side) corners_[side] = legs[side].corner;
    }
    for (size_t side = 0; side < 2; ++side) {
      const double height = filters_[2].x[0]+legs[side].point[2]-legs[side].radius;
      Vec contact_velocity = legs[side].point_velocity;
      for (size_t axis = 0; axis < 3; ++axis)
        contact_velocity[axis] += filters_[axis].x[1]+(acceleration[axis]-filters_[axis].x[2])*dt;
      const bool contact = force_[side] > (contact_[side] ? config_.contact_off_force : config_.contact_on_force)
                           && height < 0.06 && Norm(contact_velocity) < config_.contact_velocity_limit;
      if (contact && (!contact_[side] || corners_[side] != legs[side].corner)) {
        for (size_t axis = 0; axis < 3; ++axis) {
          auto& f = filters_[axis];
          f.x[3+side] = f.x[0]+legs[side].point[axis];
          // Preserve common base/foot uncertainty at touchdown.
          for (size_t j = 0; j < 5; ++j) f.p[3+side][j] = f.p[j][3+side] = f.p[0][j];
          f.p[3+side][3+side] = f.p[0][0]+1e-4;
        }
      }
      contact_[side] = contact;
      corners_[side] = legs[side].corner;
    }
    no_contact_s_ = contact_[0] || contact_[1] ? 0 : no_contact_s_+dt;
    for (size_t axis = 0; axis < 3; ++axis) {
      auto& f = filters_[axis];
      f.Predict(dt, acceleration[axis], config_.acceleration_noise, config_.bias_noise, contact_);
      for (size_t side = 0; side < 2; ++side) {
        if (!contact_[side]) continue;
        const double trust = std::clamp(force_[side]/100.0, 0.1, 1.0);
        std::array<double, 5> h{};
        h[0] = 1;
        h[3+side] = -1;
        f.Measure(h, -legs[side].point[axis], config_.position_noise*config_.position_noise/trust);
        h = {0, 1, 0, 0, 0};
        f.Measure(h, -legs[side].point_velocity[axis], config_.velocity_noise*config_.velocity_noise/trust);
        if (axis == 2) {
          h = {};
          h[3+side] = 1;
          f.Measure(h, legs[side].radius, config_.height_noise*config_.height_noise/trust);
        }
      }
    }
    if (no_contact_s_ > config_.max_flight_s) {
      Reset();
      problem_ = "no trusted foot contact for too long; recalibrating";
      return false;
    }
    out.stamp_ns = input.stamp_ns;
    out.orientation = input.orientation;
    Vec velocity{};
    for (size_t axis = 0; axis < 3; ++axis) {
      out.position[axis] = filters_[axis].x[0];
      velocity[axis] = filters_[axis].x[1];
      out.position_variance[axis] = filters_[axis].p[0][0];
      out.velocity_variance[axis] = filters_[axis].p[1][1];
      if (!std::isfinite(out.position[axis]) || !std::isfinite(velocity[axis]) ||
          !std::isfinite(out.position_variance[axis]) || out.position_variance[axis] > 0.25) return Invalid();
    }
    const Quat inverse{input.orientation[0], -input.orientation[1], -input.orientation[2], -input.orientation[3]};
    out.velocity_body = Rotate(inverse, velocity);
    out.vertical_force = force_;
    out.contact = contact_;
    problem_ = "ready";
    return true;
  }

 private:
  struct Filter {
    // Three independent axes, each [base_position, velocity, accel_bias, foot_L, foot_R].
    std::array<double, 5> x{};
    std::array<std::array<double, 5>, 5> p{};
    void Predict(double dt, double a, double acceleration_noise, double bias_noise,
                 const std::array<bool, 2>& contact) {
      x[0] += x[1]*dt + 0.5*(a-x[2])*dt*dt;
      x[1] += (a-x[2])*dt;
      std::array<std::array<double, 5>, 5> f{}, tmp{}, next{};
      for (size_t i = 0; i < 5; ++i) f[i][i] = 1;
      f[0][1] = dt; f[0][2] = -0.5*dt*dt; f[1][2] = -dt;
      for (size_t i = 0; i < 5; ++i) for (size_t j = 0; j < 5; ++j)
        for (size_t k = 0; k < 5; ++k) tmp[i][j] += f[i][k]*p[k][j];
      for (size_t i = 0; i < 5; ++i) for (size_t j = 0; j < 5; ++j)
        for (size_t k = 0; k < 5; ++k) next[i][j] += tmp[i][k]*f[j][k];
      const double q = acceleration_noise*acceleration_noise;
      next[0][0] += q*dt*dt*dt/3;
      next[0][1] += q*dt*dt/2; next[1][0] += q*dt*dt/2;
      next[1][1] += q*dt;
      next[2][2] += bias_noise*bias_noise*dt;
      for (size_t side = 0; side < 2; ++side) next[3+side][3+side] += (contact[side] ? 1e-6 : 1.0)*dt;
      p = next;
    }
    void Measure(const std::array<double, 5>& h, double value, double variance) {
      std::array<double, 5> ph{}, k{};
      double innovation = value, s = variance;
      for (size_t i = 0; i < 5; ++i) {
        innovation -= h[i]*x[i];
        for (size_t j = 0; j < 5; ++j) ph[i] += p[i][j]*h[j];
        s += h[i]*ph[i];
      }
      for (size_t i = 0; i < 5; ++i) { k[i] = ph[i]/s; x[i] += k[i]*innovation; }
      // Joseph covariance update, retaining symmetry through long stand periods.
      auto old = p;
      for (size_t i = 0; i < 5; ++i) for (size_t j = 0; j < 5; ++j)
        p[i][j] = old[i][j]-k[i]*ph[j]-ph[i]*k[j]+k[i]*s*k[j];
    }
  };
  bool Invalid() { Reset(); problem_ = "invalid or unbounded estimator state"; return false; }
  Config config_;
  std::array<Filter, 3> filters_{};
  int64_t last_stamp_{};
  bool initialized_{};
  double initialization_elapsed_{}, no_contact_s_{};
  Vec initial_bias_{}, previous_gyro_{};
  std::array<bool, 2> contact_{};
  std::array<double, 2> force_{};
  std::array<size_t, 2> corners_{};
  const char* problem_{"waiting for stationary loaded feet and IMU calibration"};
};
}  // namespace xyber_x1_infer::rl_control_module
