#include "control_module/dance_state_adapter.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace xyber_x1_infer::rl_control_module;

namespace {
void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void CheckHistory() {
  DanceWaistHistory history;
  DanceWaistHistory::Positions q{};
  Check(!history.SampleAt(1000000, q), "empty history must reject");
  Check(!history.Push(0, {0, 0, 0}), "zero timestamp must reject");
  Check(history.Push(1000000, {0, 1, 2}), "first sample");
  Check(history.Push(3000000, {2, 3, 4}), "second sample");
  Check(!history.Push(3000000, {99, 99, 99}), "duplicate timestamp must reject");
  Check(!history.Push(2000000, {99, 99, 99}), "out of order sample must reject");
  Check(!history.Push(4000000, {0, std::numeric_limits<double>::quiet_NaN(), 0}), "NaN must reject");
  Check(!history.SampleAt(999999, q) && !history.SampleAt(3000001, q), "no extrapolation");
  Check(history.SampleAt(2000000, q) && q == DanceWaistHistory::Positions{1, 2, 3}, "source-time interpolation");
  Check(history.SampleAt(3000000, q) && q == DanceWaistHistory::Positions{2, 3, 4}, "exact source sample");
  Check(history.Push(14000000, {4, 5, 6}), "sample after gap");
  Check(!history.SampleAt(13000000, q), "interpolation over a gap exceeding 10 ms must reject");
  for (int i = 15; i <= 315; ++i) Check(history.Push(i * 1000000LL, {double(i), 0, 0}), "ring push");
  Check(!history.SampleAt(59000000, q), "evicted history must reject");
  Check(history.SampleAt(60000000, q) && q[0] == 60, "oldest retained sample");
  Check(history.SampleAt(314500000, q) && q[0] == 314.5, "interpolation after ring wraps");
}

size_t CheckKinematics(const std::filesystem::path& csv) {
  std::ifstream input(csv);
  Check(input.good(), "independent MuJoCo FK fixture missing");
  std::string line;
  size_t cases = 0;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#' || line.starts_with("base_x")) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    std::array<double, 17> row{};
    for (double& value : row) Check(bool(values >> value), "invalid FK fixture row");
    DanceAnchorState out;
    Check(ComputeX1DanceAnchor({row[0], row[1], row[2]}, {row[3], row[4], row[5], row[6]},
                              {row[7], row[8], row[9]}, out), "valid FK rejected");
    for (size_t i = 0; i < 3; ++i) Check(std::abs(out.position[i] - row[10+i]) < 1e-10, "FK position differs from MuJoCo");
    double dot = 0;
    for (size_t i = 0; i < 4; ++i) dot += out.orientation[i] * row[13+i];
    Check(std::abs(std::abs(dot) - 1) < 1e-12, "FK rotation differs from MuJoCo");
    ++cases;
  }
  Check(cases >= 70, "missing independent mixed-axis FK coverage");
  DanceAnchorState out;
  Check(!ComputeX1DanceAnchor({0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0}, out), "zero quaternion must reject");
  Check(!ComputeX1DanceAnchor({0, 0, 0}, {1, 0, 0, 0},
                            {std::numeric_limits<double>::infinity(), 0, 0}, out), "invalid joint must reject");
  return cases;
}
}  // namespace

int main() {
  try {
    CheckHistory();
    const auto cases = CheckKinematics(std::filesystem::path(__FILE__).parent_path() / "data/dance_x1_waist_fk.csv");
    std::cout << "PASS: " << cases << " independent MuJoCo FK cases; bounded source-time interpolation/rejection\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
