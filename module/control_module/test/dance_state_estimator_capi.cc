// Offline probe only: drive the exact production estimator from MuJoCo/ctypes.
#include "control_module/dance_state_estimator.h"
#include <algorithm>

using namespace xyber_x1_infer::rl_control_module;
extern "C" {
void* dance_estimator_create(const double* parameters) {
  DanceBodyStateEstimator::Config c;
  if (parameters) {
    c.acceleration_scale = parameters[0]; c.initialization_s = parameters[1];
    c.contact_on_force = parameters[2]; c.contact_off_force = parameters[3];
    c.acceleration_noise = parameters[4]; c.bias_noise = parameters[5];
    c.position_noise = parameters[6]; c.velocity_noise = parameters[7]; c.height_noise = parameters[8];
    c.max_gap_s = parameters[9]; c.max_flight_s = parameters[10]; c.contact_velocity_limit = parameters[11];
    std::copy_n(parameters+12, 3, c.imu_position_in_base.begin());
  }
  return new DanceBodyStateEstimator(c);
}
void dance_estimator_destroy(void* handle) { delete static_cast<DanceBodyStateEstimator*>(handle); }
const char* dance_estimator_problem(void* handle) { return static_cast<DanceBodyStateEstimator*>(handle)->Problem(); }
int dance_estimator_update(void* handle, int64_t stamp, const double* values, double* result) {
  DanceBodyStateEstimator::Input in;
  in.stamp_ns = stamp;
  std::copy_n(values, 4, in.orientation.begin());
  std::copy_n(values+4, 3, in.gyro.begin());
  std::copy_n(values+7, 3, in.acceleration.begin());
  std::copy_n(values+10, 12, in.position.begin());
  std::copy_n(values+22, 12, in.velocity.begin());
  std::copy_n(values+34, 12, in.effort.begin());
  DanceBodyStateEstimator::Output out;
  if (!static_cast<DanceBodyStateEstimator*>(handle)->Update(in, out)) return 0;
  std::copy(out.position.begin(), out.position.end(), result);
  std::copy(out.velocity_body.begin(), out.velocity_body.end(), result+3);
  std::copy(out.orientation.begin(), out.orientation.end(), result+6);
  std::copy(out.position_variance.begin(), out.position_variance.end(), result+10);
  std::copy(out.velocity_variance.begin(), out.velocity_variance.end(), result+13);
  std::copy(out.vertical_force.begin(), out.vertical_force.end(), result+16);
  result[18] = out.contact[0]; result[19] = out.contact[1];
  return 1;
}
void dance_leg_kinematics(int side, const double* values, double* result) {
  DanceBodyStateEstimator::Input in;
  std::copy_n(values, 4, in.orientation.begin());
  std::copy_n(values+4, 3, in.gyro.begin());
  std::copy_n(values+10, 12, in.position.begin());
  std::copy_n(values+22, 12, in.velocity.begin());
  std::copy_n(values+34, 12, in.effort.begin());
  const auto leg = dance_estimation_detail::Kinematics(side, in.position, in.velocity, in.effort, in.orientation, in.gyro);
  std::copy(leg.origin.begin(), leg.origin.end(), result);
  std::copy(leg.point.begin(), leg.point.end(), result+3);
  std::copy(leg.point_velocity.begin(), leg.point_velocity.end(), result+6);
  std::copy(leg.orientation.begin(), leg.orientation.end(), result+9);
  for (size_t j = 0; j < 6; ++j) std::copy(leg.jacobian[j].begin(), leg.jacobian[j].end(), result+13+3*j);
  result[31] = leg.vertical_force;
  result[32] = leg.corner;
}
}
