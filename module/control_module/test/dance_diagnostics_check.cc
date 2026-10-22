#include "control_module/dance_diagnostics.h"

#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace xyber_x1_infer::rl_control_module;
namespace {
void Check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> values;
  std::string value; bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i+1 < line.size() && line[i+1] == '"') { value += '"'; ++i; }
      else quoted = !quoted;
    } else if (c == ',' && !quoted) { values.push_back(value); value.clear(); }
    else value += c;
  }
  values.push_back(value); return values;
}
}  // namespace

int main() {
  using D = DanceDiagnostics;
  D::Frame frame; D::Stamp(frame);
  const auto root = std::filesystem::temp_directory_path()/
      ("dance_diagnostics_check_"+std::to_string(frame.wall_ns));
  std::vector<std::string> joints;
  for (size_t i = 0; i < D::kJoints; ++i) joints.push_back("joint_"+std::to_string(i));
  const auto columns = D::Columns(joints);
  Check(columns.size() == 23+D::kValueCount, "schema offsets/column count must agree");
  auto column = [&](const std::string& name) {
    const auto it = std::find(columns.begin(), columns.end(), name);
    Check(it != columns.end(), "missing expected column"); return size_t(it-columns.begin());
  };
  D logger;
  Check(logger.Start(root.string(), joints, 10, 16), "file worker must start");
  frame.session = 1; frame.policy_frame = 0; frame.has_observation = true;
  frame.joint_stamp_ns = 1792651749849536442LL;
  D::Text(frame.event, "policy"); D::Text(frame.state, "dance_x1");
  D::Text(frame.reason, "comma, quote \" round-trip");
  frame.values[D::kObservationRaw+159] = 1.23456789;
  Check(logger.Enqueue(frame), "first short-run frame must enqueue");
  bool visible = false;
  for (int i = 0; i < 100 && !visible; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    std::ifstream file(logger.Path()); std::string line;
    std::getline(file, line); visible = bool(std::getline(file, line));
  }
  Check(visible, "short run must flush before shutdown, not wait ten seconds");
  frame.session = 2; frame.policy_frame = 0;
  Check(logger.Enqueue(frame), "second session frame");
  logger.Stop();
  Check(!logger.IoError(), "writer must close successfully");
  Check(!logger.Enqueue(frame), "stopped logger rejects new frames");
  std::ifstream file(logger.Path()); std::string line;
  Check(bool(std::getline(file, line)) && Split(line) == columns, "header must match schema");
  for (int session = 1; session <= 2; ++session) {
    Check(bool(std::getline(file, line)), "stop must drain all queued sessions");
    const auto row = Split(line);
    Check(row.size() == columns.size(), "row/header must have same number of columns");
    Check(std::stoull(row[column("session")]) == uint64_t(session), "session identity retained");
    Check(std::stoll(row[column("joint_stamp_ns")]) == frame.joint_stamp_ns, "timestamps must not lose integer precision");
    Check(row[column("reason")] == frame.reason.data(), "CSV quoting must preserve reasons");
    Check(std::stod(row[column("obs_raw_159")]) == frame.values[D::kObservationRaw+159], "observation round-trip");
    Check(std::isnan(std::stod(row[column("est_vertical_force_left")])) , "unavailable data stays NaN");
  }
  Check(bool(std::getline(file, line)) && Split(line)[column("event")] == "logger_stop", "shutdown summary retained");
  Check(!std::getline(file, line), "no duplicate frame after stop");

  Check(logger.Start(root.string(), joints, 10, 2), "small bounded queue restart");
  for (int i = 0; i < 5000; ++i) logger.Enqueue(frame);
  logger.Stop();
  Check(logger.Dropped() > 0, "queue overload is counted instead of blocking producer");
  Check(!logger.IoError(), "overflow must not corrupt file");
  std::ofstream(root/"not_a_directory") << "test";
  Check(!logger.Start((root/"not_a_directory"/"child").string(), joints), "unwritable path must fail cleanly");
  Check(!logger.Start(root.string(), joints, 0), "invalid flush interval rejected");
  std::filesystem::remove_all(root);
  std::cout << "PASS: schema, precision, short-run flush, multi-session drain, bounded overflow, I/O failure\n";
}
