#include "control_module/dance_estimator_inputs.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace xyber_x1_infer::rl_control_module;
namespace {
void Check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
std::array<double, 59> Fixture() {
  std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"data/dance_x1_estimator_static.csv");
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream stream(line);
    std::array<double, 59> row{};
    for (double& v : row) Check(bool(stream >> v), "invalid independent static fixture");
    return row;
  }
  throw std::runtime_error("missing independent static fixture");
}
DanceBodyStateEstimator::Input Input(const std::array<double, 59>& row) {
  DanceBodyStateEstimator::Input in;
  std::copy_n(row.begin(), 4, in.orientation.begin());
  std::copy_n(row.begin()+4, 3, in.gyro.begin());
  std::copy_n(row.begin()+7, 3, in.acceleration.begin());
  std::copy_n(row.begin()+10, 12, in.position.begin());
  std::copy_n(row.begin()+22, 12, in.velocity.begin());
  std::copy_n(row.begin()+34, 12, in.effort.begin());
  return in;
}
void CheckSynchronization() {
  DanceEstimatorInputs buffer;
  DanceEstimatorInputs::Joints left{}, right{};
  right.fill(2);
  DanceBodyStateEstimator::Input imu, paired;
  imu.stamp_ns = 1002000000;
  Check(buffer.PushImu(imu), "first IMU");
  Check(!buffer.Next(paired), "must wait for joint source samples");
  Check(buffer.PushJoints(1001000000, left), "left bracket");
  Check(!buffer.Next(paired), "must not extrapolate");
  Check(buffer.PushJoints(1003000000, right), "right bracket");
  Check(buffer.Next(paired) && paired.stamp_ns == imu.stamp_ns && paired.position[0] == 1 &&
        paired.velocity[0] == 1 && paired.effort[0] == 1, "source-time interpolation of all leg fields");
  Check(!buffer.Next(paired), "cannot process one IMU twice");
  Check(!buffer.PushImu(imu), "cannot refresh frozen IMU");
  Check(!buffer.PushJoints(1003000000, right), "cannot refresh frozen joints");
  imu.stamp_ns = 1010000000;
  buffer.PushImu(imu);
  buffer.PushJoints(1020000000, right);
  Check(!buffer.Next(paired), "must reject brackets over 10 ms");
  imu.stamp_ns = 1020000000;
  buffer.PushImu(imu);
  Check(buffer.Next(paired) && paired.position[0] == 2, "exact later pair recovers");
}
void CheckFilter(const std::array<double, 59>& row) {
  DanceBodyStateEstimator estimator;
  auto in = Input(row);
  // Static accelerometer bias is calibrated instead of becoming linear drift.
  in.acceleration[0] += 0.1; in.acceleration[1] -= 0.12; in.acceleration[2] += 0.2;
  DanceBodyStateEstimator::Output out;
  bool ready = false;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 1; i <= 10000; ++i) {
    in.stamp_ns = 1000000000LL+i*1000000LL;
    ready = estimator.Update(in, out);
    if (i < 500) Check(!ready, "must finish stationary calibration before publishing");
  }
  Check(ready, "loaded stationary fixture must initialize");
  Check(std::abs(out.position[2]-row[58]) < 0.002, "estimated base height must match independent foot geometry");
  for (double v : out.velocity_body) Check(std::abs(v) < 1e-7, "static bias must not cause velocity drift");
  const auto rotated = dance_estimation_detail::Rotate(in.orientation, in.acceleration);
  for (size_t axis = 0; axis < 3; ++axis) {
    const double expected = rotated[axis] - (axis == 2 ? 9.81 : 0.0);
    Check(std::abs(out.acceleration_world[axis]-expected) < 1e-9, "diagnostic acceleration is gravity removed in world axes");
    // The bias continues to adapt while foot-height measurements settle.
    Check(std::abs(out.acceleration_bias_world[axis]-expected) < 1e-5, "diagnostic bias stays near the injected static bias");
    Check(std::isfinite(out.position_variance[axis]) && out.position_variance[axis] > 0,
          "diagnostic covariance is finite and positive");
  }
  Check(std::hypot(out.position[0], out.position[1]) < 1e-7, "static base position must not drift");
  Check(!estimator.Update(in, out), "repeated timestamps must not advance filter");
  in.stamp_ns += 20000000;
  Check(!estimator.Update(in, out) && !estimator.Initialized(), "source dropout must reset calibration");
  for (int i = 0; i < 1000; ++i) { in.stamp_ns += 1000000; ready = estimator.Update(in, out); }
  Check(ready, "reinitialization after a dropout");
  in.stamp_ns += 1000000;
  in.acceleration[0] = std::numeric_limits<double>::quiet_NaN();
  Check(!estimator.Update(in, out) && !estimator.Initialized(), "invalid accelerometer must invalidate estimate");
  estimator.Reset();
  in = Input(row); in.acceleration = {0, 0, 1};
  for (int i = 1; i <= 1000; ++i) { in.stamp_ns = i*1000000LL; Check(!estimator.Update(in, out), "wrong accelerometer units must not initialize"); }
  estimator.Reset(); in = Input(row);
  std::copy_n(row.begin()+46, 12, in.effort.begin());  // gravity torque with no external load
  for (int i = 1; i <= 1000; ++i) { in.stamp_ns = i*1000000LL; Check(!estimator.Update(in, out), "unloaded/hanging fixture must not initialize"); }
  const auto us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-start).count();
  std::cout << "PASS: static bias/height, loaded-foot calibration, SI units, dropout/recovery, invalid/frozen inputs; "
            << us/13001 << " us/update including checks on this host\n";
}
void CheckFlightAndVelocityFrame(const std::array<double, 59>& row) {
  DanceBodyStateEstimator estimator;
  auto in = Input(row);
  DanceBodyStateEstimator::Output out;
  for (int i = 1; i <= 1000; ++i) { in.stamp_ns = i*1000000LL; estimator.Update(in, out); }
  std::copy_n(row.begin()+46, 12, in.effort.begin());
  for (int i = 0; i < 80; ++i) { in.stamp_ns += 1000000; Check(estimator.Update(in, out), "short flight must propagate"); }
  const auto before = out.position;
  const auto q = in.orientation;
  const std::array<double, 4> inverse{q[0], -q[1], -q[2], -q[3]};
  in.acceleration = dance_state_detail::Rotate(inverse, {0.5, 0, 9.81});
  for (int i = 0; i < 100; ++i) { in.stamp_ns += 1000000; Check(estimator.Update(in, out), "inertial propagation without contact"); }
  const auto expected = dance_state_detail::Rotate(inverse, {0.05, 0, 0});
  // The nominal left/right soles have a small height mismatch; their shared
  // plane constraint leaves micrometres/second of vertical settling velocity.
  for (size_t i = 0; i < 3; ++i)
    Check(std::abs(out.velocity_body[i]-expected[i]) < 1e-5, "world velocity must rotate into base frame");
  Check(std::abs(out.position[0]-before[0]-0.0025) < 1e-8, "gravity-compensated inertial position integration");
  for (int i = 0; i < 260; ++i) { in.stamp_ns += 1000000; estimator.Update(in, out); }
  Check(!estimator.Initialized(), "prolonged loss of support must invalidate estimate");
  std::cout << "PASS: inertial flight propagation, body-frame velocity, prolonged loss of support\n";
}
}
int main() {
  try { const auto fixture = Fixture(); CheckSynchronization(); CheckFilter(fixture); CheckFlightAndVelocityFrame(fixture); return 0; }
  catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
