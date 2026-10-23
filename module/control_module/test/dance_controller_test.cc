#include "control_module/dance_controller.h"
#include "control_module/dance_start_gate.h"
#include "control_module/pd_controller.h"
#include "control_module/rl_controller.h"
#include "control_module/state_machine.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace xyber_x1_infer::rl_control_module {
namespace {

std::filesystem::path SourceDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path();
}

YAML::Node Config() {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance_v11.yaml").string());
  auto cfg = YAML::Clone(root["controllers"]["dance_x1"]);
  cfg["diagnostics"]["enabled"] = false;  // Tests opt in only when inspecting diagnostic output.
  cfg["observation_layout"] = "full";
  cfg["state_source"] = "external_topics";  // Preserve the independent golden external-state fixture.
  cfg["joint_limits"] = root["joint_limits"];
  cfg["control_frequency_hz"] = root["control_frequecy"];
  cfg["model_file"] = (SourceDir() / "policy/dance_x1_v11_4999.bin").string();
  cfg["motion_file"] = (SourceDir() / "policy/dance_x1_motion.bin").string();
  return cfg;
}

YAML::Node Config154() {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance.yaml").string());
  auto cfg = YAML::Clone(root["controllers"]["dance_x1"]);
  cfg["diagnostics"]["enabled"] = false;
  cfg["joint_limits"] = root["joint_limits"];
  cfg["control_frequency_hz"] = root["control_frequecy"];
  cfg["model_file"] = (SourceDir()/"policy"/std::filesystem::path(cfg["model_file"].as<std::string>()).filename()).string();
  cfg["motion_file"] = (SourceDir()/"policy/dance_x1_motion.bin").string();
  return cfg;
}

YAML::Node ConfigFinish() {
  const auto root = YAML::LoadFile((SourceDir()/"cfg/rl_x1_dance.yaml").string());
  auto cfg = YAML::Clone(root["controllers"]["rl_dance_finish"]);
  cfg["joint_limits"] = root["joint_limits"];
  cfg["control_frequency_hz"] = root["control_frequecy"];
  cfg["dance_finish"]["diagnostics"]["enabled"] = false;
  cfg["onnx_conf"]["policy_file"] = (SourceDir()/"policy"/
      std::filesystem::path(cfg["onnx_conf"]["policy_file"].as<std::string>()).filename()).string();
  return cfg;
}

// Ideal frame-0 snapshot, with reversed message ordering to exercise name mapping.
// Expected commands in the CSV were computed directly from the v11 PyTorch
// checkpoint (_std + .01), independently of the C++ inference implementation.
struct Sample {
  sensor_msgs::msg::JointState joint;
  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::TransformStamped anchor;
  geometry_msgs::msg::TwistStamped velocity;
  std::unordered_map<std::string, int> index;

  explicit Sample(const YAML::Node& cfg) {
    std::array<float, 65> frame{};
    std::ifstream motion(cfg["motion_file"].as<std::string>(), std::ios::binary);
    motion.seekg(8 + 4 * 4 + 29 * 32);
    if (!motion.read(reinterpret_cast<char*>(frame.data()), sizeof(frame))) {
      throw std::runtime_error("test motion missing");
    }
    auto names = cfg["joint_list"].as<std::vector<std::string>>();
    joint.name.assign(names.rbegin(), names.rend());
    joint.position.resize(29);
    joint.velocity.resize(29);
    joint.effort.assign(29, 0);
    for (int i = 0; i < 29; ++i) {
      index[joint.name[i]] = i;
      joint.position[i] = frame[28 - i];
      joint.velocity[i] = frame[29 + 28 - i];
    }
    if (cfg["start_pose"].as<std::string>() == "default_q") {
      std::array<float, 29> default_pos{};
      std::ifstream model(cfg["model_file"].as<std::string>(), std::ios::binary);
      model.seekg(8 + 7 * 4 + 29 * 32);
      if (!model.read(reinterpret_cast<char*>(default_pos.data()), sizeof(default_pos))) {
        throw std::runtime_error("test model missing");
      }
      for (int i = 0; i < 29; ++i) {
        joint.position[i] = default_pos[28 - i];
        joint.velocity[i] = 0;
      }
    }
    imu.orientation.w = 1;
    anchor.header.frame_id = "odom";
    anchor.child_frame_id = "lumbar_pitch_link";
    anchor.transform.translation.x = frame[58];
    anchor.transform.translation.y = frame[59];
    anchor.transform.translation.z = frame[60];
    anchor.transform.rotation.w = frame[61];
    anchor.transform.rotation.x = frame[62];
    anchor.transform.rotation.y = frame[63];
    anchor.transform.rotation.z = frame[64];
    velocity.header.frame_id = "base_link";
    Stamp();
  }

  void Stamp() {
    const int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    joint.header.stamp.sec = ns / 1000000000LL;
    joint.header.stamp.nanosec = ns % 1000000000LL;
    imu.header.stamp = anchor.header.stamp = velocity.header.stamp = joint.header.stamp;
  }

  void Send(DanceController& controller) const {
    controller.SetJointStateData(joint, index);
    controller.SetImuData(imu);
    controller.SetAnchorPose(anchor);
    controller.SetBaseVelocity(velocity);
  }
  void Send(RLController& controller) const {
    controller.SetJointStateData(joint, index);
    controller.SetImuData(imu);
  }
};

std::vector<std::string> DiagnosticCsvRow(const std::string& line) {
  std::vector<std::string> fields; std::string value; bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    if (line[i] == '"') {
      if (quoted && i+1 < line.size() && line[i+1] == '"') { value += '"'; ++i; }
      else quoted = !quoted;
    } else if (line[i] == ',' && !quoted) { fields.push_back(value); value.clear(); }
    else value += line[i];
  }
  fields.push_back(value); return fields;
}

TEST(DanceController, DiagnosticsRetainExactActorInputsCommandsAndDistinctAttempts) {
  auto cfg = Config();
  const auto dir = std::filesystem::temp_directory_path()/
      ("dance_controller_diag_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  cfg["diagnostics"]["enabled"] = true;
  cfg["diagnostics"]["directory"] = dir.string();
  cfg["diagnostics"]["flush_ms"] = 10;
  DanceController controller(true);
  controller.Init(cfg);
  ASSERT_TRUE(controller.DiagnosticsEnabled());
  Sample sample(cfg);
  sample.joint.effort.assign(29, 1.25);
  sample.Send(controller);
  controller.SetDiagnosticState("stand");
  for (int i = 0; i < 20; ++i) controller.SampleDiagnostics();
  controller.SetDiagnosticState("dance_ready");
  controller.RestartController();
  controller.SetDiagnosticState("dance_x1");
  controller.Update();
  ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();
  const auto first_command = controller.GetJointCmdData();
  controller.GetJointCmdData();  // Repeated command access must not duplicate a policy row.
  for (int tick = 1; tick <= 20; ++tick) {
    sample.Stamp(); sample.Send(controller); controller.Update(); controller.GetJointCmdData();
    ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();
  }
  controller.SetDiagnosticState("keep");
  controller.SetDiagnosticState("dance_ready");
  controller.RestartController();
  controller.SetDiagnosticState("dance_x1");
  controller.Update(); controller.GetJointCmdData();
  ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  controller.Update();
  ASSERT_TRUE(controller.Faulted());
  controller.StopDiagnostics();
  ASSERT_FALSE(controller.DiagnosticIoError());

  std::ifstream file(controller.DiagnosticPath()); std::string line;
  ASSERT_TRUE(bool(std::getline(file, line)));
  const auto columns = DiagnosticCsvRow(line);
  std::map<std::string, size_t> index;
  for (size_t i = 0; i < columns.size(); ++i) index[columns[i]] = i;
  std::array<float, 160> mean{}, inv_std{};
  std::ifstream model(cfg["model_file"].as<std::string>(), std::ios::binary);
  model.seekg(8+7*4+29*32+6*29*4);
  ASSERT_TRUE(bool(model.read(reinterpret_cast<char*>(mean.data()), sizeof(mean))));
  ASSERT_TRUE(bool(model.read(reinterpret_cast<char*>(inv_std.data()), sizeof(inv_std))));
  std::vector<std::pair<int, int>> frames;
  bool passive = false, fault = false;
  while (std::getline(file, line)) {
    const auto row = DiagnosticCsvRow(line);
    ASSERT_EQ(row.size(), columns.size());
    if (row[index.at("event")] == "passive") {
      passive = true;
      EXPECT_EQ(row[index.at("has_observation")], "0");
      EXPECT_TRUE(std::isnan(std::stod(row[index.at("obs_raw_0")])));
    }
    if (row[index.at("event")] == "fault") {
      fault = true;
      EXPECT_EQ(row[index.at("snapshot_valid")], "0");
      EXPECT_EQ(row[index.at("command_valid")], "0");
      EXPECT_NE(row[index.at("reason")].find("joint feedback"), std::string::npos);
    }
    if (row[index.at("event")] != "policy") continue;
    frames.emplace_back(std::stoi(row[index.at("session")]), std::stoi(row[index.at("policy_frame")]));
    EXPECT_EQ(row[index.at("has_observation")], "1");
    EXPECT_EQ(row[index.at("command_valid")], "1");
    EXPECT_EQ(row[index.at("estimator_valid")], "0");  // External fixtures must not fabricate estimator data.
    for (int i = 0; i < 160; ++i) {
      const double raw = std::stod(row[index.at("obs_raw_"+std::to_string(i))]);
      const double normalized = std::stod(row[index.at("obs_normalized_"+std::to_string(i))]);
      EXPECT_NEAR(normalized, (raw-mean[i])*inv_std[i], 1e-4);
    }
    if (frames.size() == 1) {
      for (size_t i = 0; i < first_command.name.size(); ++i) {
        EXPECT_DOUBLE_EQ(std::stod(row[index.at("command_position_"+first_command.name[i])]), first_command.position[i]);
        EXPECT_DOUBLE_EQ(std::stod(row[index.at("joint_effort_"+first_command.name[i])]), 1.25);
        EXPECT_DOUBLE_EQ(std::stod(row[index.at("obs_raw_"+std::to_string(131+i))]), 0);
      }
    }
  }
  EXPECT_TRUE(passive);
  EXPECT_TRUE(fault);
  const std::vector<std::pair<int, int>> expected{{1,0}, {1,1}, {2,0}};
  EXPECT_EQ(frames, expected);
  const auto saved = YAML::LoadFile(controller.DiagnosticPath()+".yaml");
  EXPECT_EQ(saved["state_source"].as<std::string>(), "external_topics");
  EXPECT_EQ(saved["joint_list"].as<std::vector<std::string>>(), cfg["joint_list"].as<std::vector<std::string>>());
  std::filesystem::remove_all(dir);
}

TEST(DanceController, V12RejectsMismatchedModelLayoutAndStateSource) {
  auto cfg = Config154();
  cfg["model_file"] = (SourceDir() / "policy/dance_x1_v11_4999.bin").string();
  DanceController wrong_model(true);
  EXPECT_THROW(wrong_model.Init(cfg), std::runtime_error);
  cfg = Config154(); cfg["observation_layout"] = "full";
  DanceController wrong_layout(true);
  EXPECT_THROW(wrong_layout.Init(cfg), std::runtime_error);
  cfg = Config154(); cfg["state_source"] = "proprioceptive";
  DanceController wrong_source(true);
  EXPECT_THROW(wrong_source.Init(cfg), std::runtime_error);
  cfg = Config(); cfg["model_file"] = (SourceDir() / "policy/dance_x1_v12_154_4999.bin").string();
  DanceController wrong_full_model(true);
  EXPECT_THROW(wrong_full_model.Init(cfg), std::runtime_error);
}

TEST(DanceController, FinisherRequiresMatchingDanceModelParametersAndZeroCommand) {
  auto cfg = ConfigFinish();
  DanceController dance(true); dance.Init(Config154());
  EXPECT_NO_THROW(dance.ValidateFinisher(cfg));
  cfg["stiffness"][0] = 1.0;
  EXPECT_THROW(dance.ValidateFinisher(cfg), std::runtime_error);
  cfg = ConfigFinish(); cfg["amp_conf"]["constant_cmd"][0] = 1.0;
  RLController moving(true);
  EXPECT_ANY_THROW(moving.Init(cfg));
  cfg = ConfigFinish(); cfg["amp_conf"]["settle_ticks"] = 50;
  RLController settling(true);
  EXPECT_ANY_THROW(settling.Init(cfg));
}

TEST(DanceController, FinisherRequiresHandoffPreservesStandingTransitionsAndRejectsFrozenSources) {
  const auto cfg = ConfigFinish();
  RLController finisher(true); finisher.Init(cfg);
  ASSERT_TRUE(finisher.IsDanceFinisher());
  EXPECT_ANY_THROW(finisher.RestartController());
  Sample sample(Config154()); sample.Send(finisher);
  ASSERT_TRUE(finisher.AmpReady());
  const auto names = cfg["joint_list"].as<std::vector<std::string>>();
  std::vector<double> previous;
  for (const auto& name : names) previous.push_back(sample.joint.position[sample.index.at(name)]);
  finisher.PrepareDanceHandoff(previous); finisher.RestartController(); finisher.Update();
  ASSERT_FALSE(finisher.AmpFaulted());
  const auto first = finisher.GetJointCmdData();
  ASSERT_EQ(first.name, names);
  EXPECT_TRUE(finisher.FinishRecovering());
  // State-only transitions must not reset balance history or command blending.
  finisher.RestartController();
  const auto continued = finisher.GetJointCmdData();
  EXPECT_EQ(first.position, continued.position);
  sample.joint.velocity[sample.index.at("left_ankle_pitch_joint")] = 100;
  sample.Stamp(); sample.Send(finisher); finisher.Update();
  ASSERT_FALSE(finisher.AmpFaulted());
  const auto cmd = finisher.GetJointCmdData();
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == "left_ankle_pitch_joint") {
      EXPECT_DOUBLE_EQ(cmd.stiffness[i], 0); EXPECT_DOUBLE_EQ(cmd.damping[i], 0);
      EXPECT_DOUBLE_EQ(cmd.effort[i], -80);
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  sample.Send(finisher);  // Identical source stamps cannot refresh receive age.
  EXPECT_FALSE(finisher.AmpReady());
  finisher.Update(); EXPECT_TRUE(finisher.AmpFaulted());
}

TEST(DanceController, V12NeedsOnlyImuAndEncodersAndMatchesIndependentCheckpoint) {
  auto cfg = Config154();
  const auto dir = std::filesystem::temp_directory_path()/
      ("dance_v12_golden_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  cfg["diagnostics"]["enabled"] = true;
  cfg["diagnostics"]["directory"] = dir.string();
  DanceController controller(true);
  controller.Init(cfg);
  ASSERT_TRUE(controller.UsesImuOnly());
  ASSERT_FALSE(controller.UsesOdometry());
  ASSERT_FALSE(controller.UsesProprioceptiveEstimator());
  Sample sample(cfg);
  sample.joint.effort.clear();  // No contact/torque calibration prerequisite.
  sample.imu.linear_acceleration.x = std::numeric_limits<double>::quiet_NaN();
  controller.SetImuData(sample.imu);  // Exercise IMU-before-joint callback order.
  EXPECT_FALSE(controller.Ready());
  controller.SetJointStateData(sample.joint, sample.index);
  ASSERT_TRUE(controller.Ready());
  controller.RestartController();
  controller.Update();
  ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();
  const auto cmd = controller.GetJointCmdData();
  std::ifstream golden(SourceDir() / "test/data/dance_x1_v12_154_first_frame.csv");
  ASSERT_TRUE(golden.good());
  std::string line;
  std::getline(golden, line);
  for (int i = 0; i < 29; ++i) {
    ASSERT_TRUE(bool(std::getline(golden, line)));
    std::istringstream row(line); std::array<std::string, 5> fields;
    for (auto& field : fields) ASSERT_TRUE(bool(std::getline(row, field, ',')));
    ASSERT_EQ(cmd.name[i], fields[0]);
    EXPECT_NEAR(cmd.position[i], std::stod(fields[1]), 2e-4) << cmd.name[i];
    EXPECT_NEAR(cmd.effort[i], std::stod(fields[2]), 5e-3) << cmd.name[i];
    EXPECT_DOUBLE_EQ(cmd.stiffness[i], std::stod(fields[3]));
    EXPECT_NEAR(cmd.damping[i], std::stod(fields[4]), 1e-6);
  }
  controller.StopDiagnostics();
  std::ifstream log(controller.DiagnosticPath());
  ASSERT_TRUE(bool(std::getline(log, line)));
  const auto columns = DiagnosticCsvRow(line);
  std::map<std::string, size_t> index;
  for (size_t i = 0; i < columns.size(); ++i) index[columns[i]] = i;
  EXPECT_EQ(index.count("obs_raw_154"), 0u);
  bool policy = false;
  while (std::getline(log, line)) {
    const auto row = DiagnosticCsvRow(line);
    ASSERT_EQ(row.size(), columns.size());
    if (row[index.at("event")] != "policy") continue;
    policy = true;
    EXPECT_EQ(row[index.at("observation_dim")], "154");
    EXPECT_EQ(row[index.at("estimator_valid")], "0");
    EXPECT_EQ(row[index.at("body_stamp_ns")], "0");
    EXPECT_TRUE(std::isnan(std::stod(row[index.at("base_velocity_body_x")])));
    EXPECT_TRUE(std::isnan(std::stod(row[index.at("anchor_error_norm")])));
    std::ifstream observations(SourceDir() / "test/data/dance_x1_v12_154_observation.csv");
    ASSERT_TRUE(bool(std::getline(observations, line)));
    for (int i = 0; i < 154; ++i) {
      ASSERT_TRUE(bool(std::getline(observations, line)));
      std::istringstream values(line); std::array<std::string, 3> fields;
      for (auto& field : fields) ASSERT_TRUE(bool(std::getline(values, field, ',')));
      EXPECT_NEAR(std::stod(row[index.at("obs_raw_"+std::to_string(i))]), std::stod(fields[1]), 2e-5) << i;
      EXPECT_NEAR(std::stod(row[index.at("obs_normalized_"+std::to_string(i))]), std::stod(fields[2]), 2e-5) << i;
    }
  }
  EXPECT_TRUE(policy);
  std::filesystem::remove_all(dir);
}

TEST(DanceController, V12ChecksReadinessSpeedsAndRejectsFrozenOrMissingFeedback) {
  auto cfg = Config154();
  DanceController controller(true); controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  ASSERT_TRUE(controller.Ready());
  sample.imu.angular_velocity.x = 0.3;
  sample.Stamp(); sample.Send(controller);
  EXPECT_FALSE(controller.Ready());
  sample.imu.angular_velocity.x = 0;
  sample.joint.velocity[0] = 0.6;
  sample.Stamp(); sample.Send(controller);
  EXPECT_FALSE(controller.Ready());
  sample.joint.velocity[0] = 0;
  sample.Stamp(); sample.Send(controller);
  ASSERT_TRUE(controller.Ready());
  controller.RestartController(); controller.Update();
  ASSERT_FALSE(controller.Faulted());
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  sample.Send(controller);  // Replaying the same timestamps cannot extend freshness.
  EXPECT_FALSE(controller.Ready());
  controller.Update();
  EXPECT_TRUE(controller.Faulted());
}

TEST(DanceController, V12UsesNewestCompleteSourceTimePairWithoutExtrapolating) {
  auto cfg = Config154();
  DanceController controller(true); controller.Init(cfg);
  Sample sample(cfg);
  sample.joint.header.stamp.sec = 10; sample.joint.header.stamp.nanosec = 0;
  sample.imu.header.stamp = sample.joint.header.stamp;
  controller.SetJointStateData(sample.joint, sample.index);
  sample.imu.header.stamp.nanosec = 1000000;
  controller.SetImuData(sample.imu);
  EXPECT_FALSE(controller.Ready());  // No bracketing sample yet.
  sample.joint.header.stamp.nanosec = 2000000;
  controller.SetJointStateData(sample.joint, sample.index);
  ASSERT_TRUE(controller.Ready());
  sample.imu.header.stamp.nanosec = 3000000;
  controller.SetImuData(sample.imu);
  EXPECT_TRUE(controller.Ready());  // Previous complete pair survives callback ordering.
  DanceController gap(true); gap.Init(cfg);
  sample.joint.header.stamp.nanosec = 0;
  gap.SetJointStateData(sample.joint, sample.index);
  sample.imu.header.stamp.nanosec = 15000000;
  gap.SetImuData(sample.imu);
  sample.joint.header.stamp.nanosec = 20000000;
  gap.SetJointStateData(sample.joint, sample.index);
  EXPECT_FALSE(gap.Ready());  // 20 ms interpolation interval is rejected.
}

TEST(DanceController, V12HoldsTargetsFor20TicksAndUpdatesSaturatedAnklePdEveryTick) {
  const auto cfg = Config154();
  DanceController controller(true); controller.Init(cfg);
  Sample sample(cfg); sample.Send(controller);
  controller.RestartController(); controller.Update();
  ASSERT_FALSE(controller.Faulted());
  const auto first = controller.GetJointCmdData();
  sample.joint.velocity[sample.index.at("left_ankle_pitch_joint")] = 100.0;
  for (int tick = 1; tick < 20; ++tick) {
    sample.Stamp(); sample.Send(controller); controller.Update();
    ASSERT_FALSE(controller.Faulted());
    const auto cmd = controller.GetJointCmdData();
    for (size_t i = 0; i < cmd.name.size(); ++i) {
      EXPECT_DOUBLE_EQ(cmd.position[i], first.position[i]);
      if (cmd.name[i] == "left_ankle_pitch_joint") {
        EXPECT_DOUBLE_EQ(cmd.effort[i], -80.0);
        EXPECT_DOUBLE_EQ(cmd.stiffness[i], 0.0);
        EXPECT_DOUBLE_EQ(cmd.damping[i], 0.0);
      }
    }
  }
}

TEST(DanceController, FirstFrameMatchesV11Checkpoint) {
  auto cfg = Config();
  cfg["start_pose"] = "motion_start";  // Preserve the independent frame-0 golden case.
  DanceController controller(false);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  ASSERT_TRUE(controller.Ready());
  controller.RestartController();
  controller.Update();
  ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();
  const auto cmd = controller.GetJointCmdData();
  std::ifstream golden(SourceDir() / "test/data/dance_x1_v11_first_frame.csv");
  ASSERT_TRUE(golden.good());
  std::string line;
  std::getline(golden, line);
  for (int i = 0; i < 29; ++i) {
    ASSERT_TRUE(static_cast<bool>(std::getline(golden, line)));
    std::istringstream row(line);
    std::array<std::string, 5> fields;
    for (auto& field : fields) ASSERT_TRUE(static_cast<bool>(std::getline(row, field, ',')));
    ASSERT_EQ(cmd.name[i], fields[0]);
    EXPECT_NEAR(cmd.position[i], std::stod(fields[1]), 2e-4) << cmd.name[i];
    EXPECT_NEAR(cmd.effort[i], std::stod(fields[2]), 5e-3) << cmd.name[i];
    EXPECT_DOUBLE_EQ(cmd.stiffness[i], std::stod(fields[3]));
    EXPECT_NEAR(cmd.damping[i], std::stod(fields[4]), 1e-6);
  }
}

TEST(DanceController, HoldsPositionFor20TicksButUpdatesAnkleTorque) {
  auto cfg = Config();
  cfg["start_pose"] = "motion_start";
  DanceController controller(false);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  controller.RestartController();
  controller.Update();
  ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();
  const auto first = controller.GetJointCmdData();
  const int slot = sample.index.at("left_ankle_pitch_joint");
  sample.joint.velocity[slot] += 0.2;
  for (int tick = 1; tick < 20; ++tick) {
    sample.Stamp();
    sample.Send(controller);
    controller.Update();
    ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();
    const auto cmd = controller.GetJointCmdData();
    EXPECT_EQ(cmd.position, first.position);
    const auto ankle = std::find(cmd.name.begin(), cmd.name.end(), "left_ankle_pitch_joint") - cmd.name.begin();
    EXPECT_NE(cmd.effort[ankle], first.effort[ankle]);
  }
}

TEST(DanceController, PlaybackIgnoresLegacyTrackingThresholdAndMeasuredJointExcursions) {
  auto cfg = Config();
  // Older installed YAML may still contain this retired setting.
  cfg["max_tracking_error"] = 1e-6;
  DanceController controller(true);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  ASSERT_TRUE(controller.Ready());
  controller.RestartController();
  controller.Update();
  ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();

  const double excursion = cfg["joint_limit_tolerance"].as<double>() + 0.01;
  sample.joint.position[sample.index.at("left_hip_roll_joint")] =
      cfg["joint_limits"]["left_hip_roll_joint"]["upper"].as<double>() + excursion;
  sample.joint.position[sample.index.at("right_hip_roll_joint")] =
      cfg["joint_limits"]["right_hip_roll_joint"]["lower"].as<double>() - excursion;
  // Exercise held targets and two subsequent policy inferences with both
  // upper/lower measured limit excursions and finite, fresh feedback.
  for (int tick = 1; tick <= 40; ++tick) {
    sample.Stamp();
    sample.Send(controller);
    controller.Update();
    ASSERT_FALSE(controller.Faulted()) << controller.FaultReason();
    EXPECT_FALSE(controller.Finished());
    const auto cmd = controller.GetJointCmdData();
    for (size_t i = 0; i < cmd.name.size(); ++i) {
      const auto limit = cfg["joint_limits"][cmd.name[i]];
      EXPECT_TRUE(std::isfinite(cmd.position[i])) << cmd.name[i];
      EXPECT_TRUE(std::isfinite(cmd.effort[i])) << cmd.name[i];
      EXPECT_GE(cmd.position[i], limit["lower"].as<double>() - 1e-6) << cmd.name[i];
      EXPECT_LE(cmd.position[i], limit["upper"].as<double>() + 1e-6) << cmd.name[i];
    }
  }
}

TEST(DanceController, PlaybackContinuesAcrossAnchorExcursionsWithoutRealigningReference) {
  auto cfg = Config();
  cfg["max_anchor_error"] = 0.6;  // Retired settings in older installed YAML are ignored.
  DanceController baseline(true);
  DanceController displaced(true);
  baseline.Init(cfg);
  displaced.Init(cfg);
  Sample sample(cfg);
  sample.Send(baseline);
  sample.Send(displaced);
  ASSERT_TRUE(baseline.Ready());
  ASSERT_TRUE(displaced.Ready());
  baseline.RestartController();
  displaced.RestartController();
  baseline.Update();
  displaced.Update();
  ASSERT_FALSE(baseline.Faulted()) << baseline.FaultReason();
  ASSERT_FALSE(displaced.Faulted()) << displaced.FaultReason();

  for (int tick = 1; tick <= 20; ++tick) {
    sample.Stamp();
    sample.Send(baseline);
    Sample offset = sample;
    offset.anchor.transform.translation.x += 0.65;
    offset.Send(displaced);
    baseline.Update();
    displaced.Update();
    ASSERT_FALSE(baseline.Faulted()) << baseline.FaultReason();
    ASSERT_FALSE(displaced.Faulted()) << displaced.FaultReason();
    EXPECT_FALSE(displaced.Finished());
  }
  // The actor must still receive the error and respond to it at the next
  // inference, rather than hiding the displacement by realigning the reference.
  const auto expected = baseline.GetJointCmdData();
  const auto actual = displaced.GetJointCmdData();
  bool changed = false;
  for (size_t i = 0; i < actual.name.size(); ++i) {
    changed = changed || std::abs(actual.position[i] - expected.position[i]) > 1e-5 ||
                        std::abs(actual.effort[i] - expected.effort[i]) > 1e-5;
    const auto limit = cfg["joint_limits"][actual.name[i]];
    EXPECT_TRUE(std::isfinite(actual.position[i])) << actual.name[i];
    EXPECT_TRUE(std::isfinite(actual.effort[i])) << actual.name[i];
    EXPECT_GE(actual.position[i], limit["lower"].as<double>() - 1e-6) << actual.name[i];
    EXPECT_LE(actual.position[i], limit["upper"].as<double>() + 1e-6) << actual.name[i];
  }
  EXPECT_TRUE(changed);
}

TEST(DanceController, MeasuredJointLimitsStillPreventEntry) {
  auto cfg = Config();
  cfg["max_start_pose_error"] = 0.5;  // Isolate the entry limit check from pose readiness.
  DanceController controller(true);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  ASSERT_TRUE(controller.Ready());
  sample.joint.position[sample.index.at("left_hip_roll_joint")] =
      cfg["joint_limits"]["left_hip_roll_joint"]["upper"].as<double>() +
      cfg["joint_limit_tolerance"].as<double>() + 0.01;
  sample.Stamp();
  sample.Send(controller);
  const char* reason = nullptr;
  EXPECT_FALSE(controller.Ready(&reason));
  ASSERT_NE(reason, nullptr);
  EXPECT_EQ(std::string(reason), "a measured joint is outside its position limits");
}

TEST(DanceController, MissingOrOldSourceFeedbackPreventsEntry) {
  const auto cfg = Config();
  DanceController controller(false);
  controller.Init(cfg);
  EXPECT_FALSE(controller.Ready());
  Sample sample(cfg);
  --sample.anchor.header.stamp.sec;
  sample.Send(controller);
  EXPECT_FALSE(controller.Ready());
  sample.Stamp();
  sample.Send(controller);
  ASSERT_TRUE(controller.Ready());
  controller.RestartController();
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  controller.Update();
  EXPECT_TRUE(controller.Faulted());
}

TEST(DanceController, ProprioceptiveFeedbackPublishesSourceTimedBodyStateWithoutExternalOdometry) {
  auto cfg = Config();
  cfg["state_source"] = "proprioceptive";
  DanceController controller(true);
  controller.Init(cfg);
  ASSERT_TRUE(controller.UsesProprioceptiveEstimator());
  Sample sample(cfg);
  std::ifstream fixture(SourceDir()/"test/data/dance_x1_estimator_static.csv");
  std::string line;
  std::array<double, 59> row{};
  while (std::getline(fixture, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream stream(line);
    for (double& v : row) ASSERT_TRUE(bool(stream >> v));
    break;
  }
  ASSERT_GT(row[58], 0.1);
  sample.imu.orientation.w = row[0]; sample.imu.orientation.x = row[1];
  sample.imu.orientation.y = row[2]; sample.imu.orientation.z = row[3];
  sample.imu.linear_acceleration.z = 9.81;
  for (size_t i = 0; i < 12; ++i) {
    const size_t j = sample.index.at(kDanceLegJointNames[i]);
    sample.joint.position[j] = row[10+i]; sample.joint.velocity[j] = 0;
    sample.joint.effort[j] = row[34+i];
  }
  int count = 0;
  nav_msgs::msg::Odometry last;
  controller.SetEstimatedOdometryCallback([&](const nav_msgs::msg::Odometry& value) {
    const auto ns = [](const builtin_interfaces::msg::Time& stamp) {
      return int64_t(stamp.sec)*1000000000LL+stamp.nanosec;
    };
    EXPECT_GT(ns(value.header.stamp), ns(last.header.stamp));
    last = value; ++count;
  });
  for (int i = 1; i <= 1000; ++i) {
    sample.joint.header.stamp.sec = 1+i/1000;
    sample.joint.header.stamp.nanosec = (i%1000)*1000000;
    sample.imu.header.stamp = sample.joint.header.stamp;
    if (i%2) {
      controller.SetImuData(sample.imu);
      controller.SetJointStateData(sample.joint, sample.index);
    } else {
      controller.SetJointStateData(sample.joint, sample.index);
      controller.SetImuData(sample.imu);
    }
    if (i < 500) EXPECT_FALSE(controller.Ready());
  }
  EXPECT_TRUE(controller.Ready());
  EXPECT_GT(count, 10);
  EXPECT_LE(count, 50);
  EXPECT_EQ(last.header.frame_id, "odom"); EXPECT_EQ(last.child_frame_id, "base_link");
  EXPECT_NEAR(last.pose.pose.position.z, row[58], 0.002);
  EXPECT_NEAR(last.twist.twist.linear.x, 0, 1e-7);
  EXPECT_NEAR(last.twist.twist.linear.y, 0, 1e-7);
  EXPECT_GT(last.pose.covariance[0], 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  controller.SetImuData(sample.imu);  // frozen source cannot keep the estimate alive
  controller.SetJointStateData(sample.joint, sample.index);
  EXPECT_FALSE(controller.Ready());
}

TEST(DanceController, ProprioceptiveEstimatorRejectsInvalidContactConfiguration) {
  auto cfg = Config();
  cfg["state_source"] = "proprioceptive";
  cfg["state_estimator"]["contact_on_force"] = 10.;
  cfg["state_estimator"]["contact_off_force"] = 20.;
  DanceController controller(true);
  EXPECT_THROW(controller.Init(cfg), std::runtime_error);
}

TEST(DanceController, RejectsRepeatedSourceStampsInSimulationToo) {
  const auto cfg = Config();
  DanceController controller(true);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  ASSERT_TRUE(controller.Ready());
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  sample.Send(controller);  // Replaying a frozen source must not refresh receipt time.
  EXPECT_FALSE(controller.Ready());
}

TEST(DanceController, RejectsWrongRateOldSettlePhaseAndJointOrder) {
  auto cfg = Config();
  cfg["control_frequency_hz"] = 500;
  EXPECT_THROW(DanceController(false).Init(cfg), std::runtime_error);
  cfg = Config();
  cfg["prepare_ticks"] = 4000;
  EXPECT_THROW(DanceController(false).Init(cfg), std::runtime_error);
  cfg = Config();
  cfg["start_pose"] = "unknown";
  EXPECT_THROW(DanceController(false).Init(cfg), std::runtime_error);
  cfg = Config();
  cfg["state_source"] = "unknown";
  EXPECT_THROW(DanceController(false).Init(cfg), std::runtime_error);
  cfg = Config();
  cfg["state_source"] = "odometry";
  cfg["base_frame"] = "base_footprint";
  EXPECT_THROW(DanceController(false).Init(cfg), std::runtime_error);
  cfg = Config();
  cfg["joint_list"][0] = "lumbar_yaw_joint";
  cfg["joint_list"][1] = "left_hip_pitch_joint";
  EXPECT_THROW(DanceController(false).Init(cfg), std::runtime_error);
}

TEST(DanceController, GuideRejectsAnUnpreparedJointPose) {
  const auto cfg = Config();
  DanceController controller(false);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.joint.position[sample.index.at("left_elbow_pitch_joint")] = 1.0;
  sample.Send(controller);
  EXPECT_FALSE(controller.Ready());
}

nav_msgs::msg::Odometry BodyOdometry(const Sample& sample) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = sample.joint.header.stamp;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.position.z = 0.8;
  odometry.pose.pose.orientation.w = 1;
  return odometry;
}

builtin_interfaces::msg::Time ShiftStamp(builtin_interfaces::msg::Time stamp, int64_t offset_ns) {
  const int64_t ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec + offset_ns;
  stamp.sec = ns / 1000000000LL;
  stamp.nanosec = ns % 1000000000LL;
  return stamp;
}

TEST(DanceController, DirectOdometryNeedsNoPythonBridgeAndRejectsNavigationFrames) {
  auto cfg = Config();
  cfg["state_source"] = "odometry";
  DanceController controller(false);
  controller.Init(cfg);
  ASSERT_TRUE(controller.UsesOdometry());
  Sample sample(cfg);
  sample.Send(controller);  // Legacy anchor/velocity topics must not satisfy native mode.
  const char* reason = nullptr;
  EXPECT_FALSE(controller.Ready(&reason));
  ASSERT_NE(reason, nullptr);
  EXPECT_NE(std::string(reason).find("no accepted body odometry"), std::string::npos);
  auto odometry = BodyOdometry(sample);
  odometry.child_frame_id = "base_footprint";
  controller.SetOdometry(odometry);
  EXPECT_FALSE(controller.Ready(&reason));
  EXPECT_NE(std::string(reason).find("frames mismatch"), std::string::npos);
  odometry.child_frame_id = "base_link";
  controller.SetOdometry(odometry);
  ASSERT_TRUE(controller.Ready(&reason)) << reason;
  controller.RestartController();
  controller.Update();
  EXPECT_FALSE(controller.Faulted()) << controller.FaultReason();
}

TEST(DanceController, DirectOdometryMatchesIndependentExternalAnchorInference) {
  auto cfg = Config();
  DanceController external(false);
  external.Init(cfg);
  cfg["state_source"] = "odometry";
  DanceController native(false);
  native.Init(cfg);
  Sample sample(cfg);
  // Use the first independent MuJoCo fixture (zero waist, base [0,0,.8]).
  std::ifstream fixture(SourceDir() / "test/data/dance_x1_waist_fk.csv");
  std::string line;
  do {
    ASSERT_TRUE(bool(std::getline(fixture, line)));
  } while (line.empty() || line[0] == '#' || line.starts_with("base_x"));
  std::replace(line.begin(), line.end(), ',', ' ');
  std::istringstream values(line);
  std::array<double, 17> row{};
  for (double& value : row) ASSERT_TRUE(bool(values >> value));
  sample.anchor.transform.translation.x = row[10];
  sample.anchor.transform.translation.y = row[11];
  sample.anchor.transform.translation.z = row[12];
  sample.anchor.transform.rotation.w = row[13];
  sample.anchor.transform.rotation.x = row[14];
  sample.anchor.transform.rotation.y = row[15];
  sample.anchor.transform.rotation.z = row[16];
  sample.Send(external);
  sample.Send(native);
  native.SetOdometry(BodyOdometry(sample));
  ASSERT_TRUE(external.Ready());
  ASSERT_TRUE(native.Ready());
  external.RestartController();
  native.RestartController();
  external.Update();
  native.Update();
  ASSERT_FALSE(external.Faulted());
  ASSERT_FALSE(native.Faulted());
  const auto expected = external.GetJointCmdData();
  const auto actual = native.GetJointCmdData();
  ASSERT_EQ(actual.name, expected.name);
  for (size_t i = 0; i < actual.name.size(); ++i) {
    EXPECT_NEAR(actual.position[i], expected.position[i], 1e-5) << actual.name[i];
    EXPECT_NEAR(actual.effort[i], expected.effort[i], 1e-3) << actual.name[i];
  }
}

TEST(DanceController, DirectOdometryWaitsForBracketingJointFeedback) {
  auto cfg = Config();
  cfg["state_source"] = "odometry";
  DanceController controller(false);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  auto odometry = BodyOdometry(sample);
  odometry.header.stamp = ShiftStamp(odometry.header.stamp, 500000);
  controller.SetOdometry(odometry);
  const char* reason = nullptr;
  EXPECT_FALSE(controller.Ready(&reason));
  EXPECT_NE(std::string(reason).find("bracket odometry source time"), std::string::npos);
  sample.joint.header.stamp = ShiftStamp(sample.joint.header.stamp, 1000000);
  sample.imu.header.stamp = sample.joint.header.stamp;
  sample.Send(controller);
  EXPECT_TRUE(controller.Ready(&reason)) << reason;
}

TEST(DanceController, DirectOdometryRejectsFrozenEstimatorEvenWithFreshJointsAndImu) {
  auto cfg = Config();
  cfg["state_source"] = "odometry";
  DanceController controller(true);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  const auto odometry = BodyOdometry(sample);
  controller.SetOdometry(odometry);
  ASSERT_TRUE(controller.Ready());
  controller.RestartController();
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  sample.Stamp();
  sample.Send(controller);
  controller.SetOdometry(odometry);  // Replays must not refresh the estimator receipt time.
  EXPECT_FALSE(controller.Ready());
  controller.Update();
  EXPECT_TRUE(controller.Faulted());
}

TEST(DanceController, NewOdometryBeforeItsJointSampleDoesNotResetReadiness) {
  auto cfg = Config();
  cfg["state_source"] = "odometry";
  DanceController controller(false);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  auto odometry = BodyOdometry(sample);
  controller.SetOdometry(odometry);
  ASSERT_TRUE(controller.Ready());
  odometry.header.stamp = ShiftStamp(odometry.header.stamp, 500000);
  controller.SetOdometry(odometry);
  EXPECT_TRUE(controller.Ready());  // Previous synchronized pair is still valid.
  sample.joint.header.stamp = ShiftStamp(sample.joint.header.stamp, 1000000);
  sample.imu.header.stamp = sample.joint.header.stamp;
  sample.Send(controller);
  EXPECT_TRUE(controller.Ready());  // New pair becomes available without a false gap.
}

TEST(DanceController, DirectOdometryRejectsStaleInvalidAndExcessiveSpeedInputs) {
  auto cfg = Config();
  cfg["state_source"] = "odometry";
  DanceController controller(false);
  controller.Init(cfg);
  Sample sample(cfg);
  sample.Send(controller);
  const auto valid = BodyOdometry(sample);
  auto invalid = valid;
  --invalid.header.stamp.sec;
  controller.SetOdometry(invalid);
  EXPECT_FALSE(controller.Ready());
  invalid = valid;
  invalid.pose.pose.orientation.w = 0;
  controller.SetOdometry(invalid);
  EXPECT_FALSE(controller.Ready());
  invalid = valid;
  invalid.twist.twist.linear.x = 1;
  controller.SetOdometry(invalid);
  const char* reason = nullptr;
  EXPECT_FALSE(controller.Ready(&reason));
  EXPECT_NE(std::string(reason).find("max_start_speed"), std::string::npos);
}

TEST(DanceController, DefaultPreparationDoesNotRequireTheDynamicFirstFramePose) {
  const auto cfg = Config();
  ASSERT_EQ(cfg["start_pose"].as<std::string>(), "default_q");
  DanceController controller(false);
  controller.Init(cfg);
  Sample prepared(cfg);
  prepared.Send(controller);
  ASSERT_TRUE(controller.Ready());
  auto reference_cfg = YAML::Clone(cfg);
  reference_cfg["start_pose"] = "motion_start";
  Sample reference(reference_cfg);
  reference.Send(controller);
  EXPECT_FALSE(controller.Ready());  // A different pose must not bypass the selected profile.
}

TEST(DanceController, PreparationTargetMatchesDeclaredProfileAndHoldsWithoutPlayback) {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance.yaml").string());
  auto cfg = YAML::Clone(root["controllers"]["pd_dance_pose"]);
  cfg["control_frequency_hz"] = root["control_frequecy"];
  DanceController dance(false);
  dance.Init(Config());
  ASSERT_EQ(cfg["joint_list"].as<std::vector<std::string>>(), dance.GetJointList());
  const auto expected = dance.PreparationPositions();
  const auto target = cfg["init_state"].as<std::vector<double>>();
  for (size_t i = 0; i < expected.size(); ++i) EXPECT_NEAR(target[i], expected[i], 1e-6);
  PDController pose(true);
  pose.Init(cfg);
  pose.RestartController();
  EXPECT_FALSE(pose.TransitionComplete());
  for (int tick = 0; tick < 4001; ++tick) pose.Update();
  ASSERT_TRUE(pose.TransitionComplete());
  const auto held = pose.GetJointCmdData();
  for (size_t i = 0; i < target.size(); ++i) EXPECT_NEAR(held.position[i], target[i], 1e-9);
  for (int tick = 0; tick < 20000; ++tick) pose.Update();
  EXPECT_EQ(pose.GetJointCmdData().position, held.position);
}

TEST(DanceController, ThreeButtonsSelectZeroStandAndGuidePreparationThenInternalPlayback) {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance.yaml").string());
  StateMachine machine;
  machine.Init(root["robot_states"], "idle");
  EXPECT_FALSE(machine.OnEvent("/dance_start"));
  EXPECT_FALSE(machine.OnEvent("/stand_mode"));
  EXPECT_FALSE(machine.OnEvent("/dance_play"));
  ASSERT_TRUE(machine.OnEvent("/zero_mode"));
  EXPECT_EQ(machine.GetCurrentState(), "zero");
  EXPECT_EQ(machine.GetCurrentControllerNames(), std::vector<std::string>{"pd_zero"});
  EXPECT_FALSE(machine.OnEvent("/zero_mode"));
  EXPECT_FALSE(machine.OnEvent("/dance_start"));
  ASSERT_TRUE(machine.OnEvent("/stand_mode"));
  EXPECT_EQ(machine.GetCurrentState(), "stand");
  EXPECT_EQ(machine.GetCurrentControllerNames(), std::vector<std::string>{"pd_stand"});
  EXPECT_FALSE(machine.OnEvent("/stand_mode"));
  EXPECT_FALSE(machine.OnEvent("/dance_play"));
  ASSERT_TRUE(machine.OnEvent("/dance_start"));
  EXPECT_EQ(machine.GetCurrentState(), "dance_ready");
  EXPECT_EQ(machine.GetCurrentControllerNames(), std::vector<std::string>{"pd_dance_pose"});
  EXPECT_FALSE(machine.OnEvent("/dance_start"));
  EXPECT_EQ(machine.GetCurrentState(), "dance_ready");
  // No fourth button: the control loop emits this only after the ready hold.
  ASSERT_TRUE(root["robot_states"]["dance_x1"]["internal_trigger"].as<bool>());
  ASSERT_TRUE(machine.OnEvent("/dance_play"));
  EXPECT_EQ(machine.GetCurrentState(), "dance_x1");
  EXPECT_FALSE(machine.OnEvent("/dance_start"));
  EXPECT_FALSE(machine.OnEvent("/dance_play"));
  ASSERT_TRUE(machine.OnEvent("/keep_mode"));
  EXPECT_FALSE(machine.OnEvent("/dance_start"));
}

TEST(DanceController, StopInterruptsEveryStageOfTheThreeButtonSequence) {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance.yaml").string());
  const std::vector<std::string> events{
      "/zero_mode", "/stand_mode", "/dance_start", "/dance_play", "/dance_finish", "/dance_stand"};
  for (const std::string stop : {"/keep_mode", "/idle_mode"}) {
    for (size_t stage = 1; stage <= events.size(); ++stage) {
      StateMachine machine;
      machine.Init(root["robot_states"], "idle");
      for (size_t i = 0; i < stage; ++i) ASSERT_TRUE(machine.OnEvent(events[i]));
      ASSERT_TRUE(machine.OnEvent(stop));
      EXPECT_FALSE(machine.OnEvent("/dance_play"));
      EXPECT_FALSE(machine.OnEvent("/dance_start"));
      EXPECT_FALSE(machine.OnEvent("/stand_mode"));
      ASSERT_TRUE(machine.OnEvent("/zero_mode"));
      EXPECT_EQ(machine.GetCurrentState(), "zero");
    }
  }
}

TEST(DanceController, CompletionUsesInternalFinishingAndMonitoredStandingWithRecovery) {
  const auto root = YAML::LoadFile((SourceDir()/"cfg/rl_x1_dance.yaml").string());
  for (const auto state : {"dance_finish", "dance_stand"})
    ASSERT_TRUE(root["robot_states"][state]["internal_trigger"].as<bool>());
  EXPECT_EQ(root["controllers"]["rl_dance_finish"]["amp_conf"]["constant_cmd"].as<std::vector<double>>(),
            (std::vector<double>{0, 0, 0}));
  StateMachine machine; machine.Init(root["robot_states"], "idle");
  EXPECT_FALSE(machine.OnEvent("/dance_finish")); EXPECT_FALSE(machine.OnEvent("/dance_stand"));
  for (const auto event : {"/zero_mode", "/stand_mode", "/dance_start", "/dance_play"}) ASSERT_TRUE(machine.OnEvent(event));
  EXPECT_FALSE(machine.OnEvent("/dance_stand"));
  ASSERT_TRUE(machine.OnEvent("/dance_finish"));
  const auto controller = machine.GetCurrentControllerNames();
  EXPECT_EQ(controller, (std::vector<std::string>{"rl_dance_finish"}));
  EXPECT_FALSE(machine.OnEvent("/dance_finish")); EXPECT_FALSE(machine.OnEvent("/dance_start"));
  ASSERT_TRUE(machine.OnEvent("/dance_stand")); EXPECT_EQ(controller, machine.GetCurrentControllerNames());
  ASSERT_TRUE(machine.OnEvent("/dance_finish")); EXPECT_EQ(controller, machine.GetCurrentControllerNames());
  ASSERT_TRUE(machine.OnEvent("/keep_mode"));
  EXPECT_FALSE(machine.OnEvent("/dance_stand")); EXPECT_FALSE(machine.OnEvent("/dance_finish"));
}

TEST(DanceController, AcceptedGuideStartsAfterHoldWithoutAnotherButtonAndStopCancels) {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance.yaml").string());
  for (const bool stop_before_start : {false, true}) {
    StateMachine machine;
    DanceStartGate gate;
    gate.Configure(root["dance_ready_hold_s"].as<double>());
    const int hold_ms = static_cast<int>(root["dance_ready_hold_s"].as<double>() * 1000);
    machine.Init(root["robot_states"], "idle");
    ASSERT_TRUE(machine.OnEvent("/zero_mode"));
    ASSERT_TRUE(machine.OnEvent("/stand_mode"));
    ASSERT_TRUE(machine.OnEvent("/dance_start"));  // The third and last operator event.
    const auto start = DanceStartGate::Clock::now();
    for (int ms = 0; ms < hold_ms; ms += 20) {
      gate.Update(true, start + std::chrono::milliseconds(ms));
      EXPECT_FALSE(gate.Ready(start + std::chrono::milliseconds(ms)));
      EXPECT_FALSE(machine.OnEvent("/dance_start"));  // Duplicate Guide never resets preparation.
    }
    if (stop_before_start) {
      ASSERT_TRUE(machine.OnEvent("/keep_mode"));
      gate.Reset();
      EXPECT_FALSE(gate.Ready(start + std::chrono::milliseconds(hold_ms)));
      EXPECT_FALSE(machine.OnEvent("/dance_play"));
      EXPECT_EQ(machine.GetCurrentState(), "keep");
    } else {
      gate.Update(true, start + std::chrono::milliseconds(hold_ms));
      ASSERT_TRUE(gate.Ready(start + std::chrono::milliseconds(hold_ms)));
      ASSERT_TRUE(machine.OnEvent("/dance_play"));  // Automatic internal event.
      EXPECT_EQ(machine.GetCurrentState(), "dance_x1");
    }
  }
}

TEST(DanceController, ZeroAndStandInterpolateFromFeedbackAndRequireMeasuredArrival) {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance.yaml").string());
  Sample sample(Config());
  for (const std::string name : {"pd_zero", "pd_stand"}) {
    auto cfg = YAML::Clone(root["controllers"][name]);
    cfg["control_frequency_hz"] = root["control_frequecy"];
    PDController pose(false);
    pose.Init(cfg);
    EXPECT_FALSE(pose.TargetReached(0.12));  // Missing feedback is never zero.
    pose.SetJointStateData(sample.joint, sample.index);
    pose.SetImuData(sample.imu);
    pose.RestartController();
    const auto first = pose.GetJointCmdData();
    for (size_t i = 0; i < first.name.size(); ++i) {
      EXPECT_DOUBLE_EQ(first.position[i], sample.joint.position[sample.index.at(first.name[i])]);
    }
    EXPECT_FALSE(pose.TransitionComplete());
    for (int tick = 0; tick < 2001; ++tick) pose.Update();
    ASSERT_TRUE(pose.TransitionComplete());
    const auto reached = pose.GetJointCmdData();
    const auto target = cfg["init_state"].as<std::vector<double>>();
    ASSERT_EQ(reached.name.size(), 29U);
    for (size_t i = 0; i < reached.name.size(); ++i) {
      EXPECT_NEAR(reached.position[i], target[i], 1e-9);
      sample.joint.position[sample.index.at(reached.name[i])] = target[i];
    }
    EXPECT_FALSE(pose.TargetReached(0.12));  // Command elapsed != measured arrival.
    pose.SetJointStateData(sample.joint, sample.index);
    pose.SetImuData(sample.imu);
    EXPECT_TRUE(pose.TargetReached(0.12));
  }
}

TEST(DanceController, StandTargetAndGainsMatchTheProvenAmpComposition) {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance.yaml").string());
  const auto amp = YAML::LoadFile((SourceDir() / "cfg/rl_x1.yaml").string());
  for (const std::string field : {"init_state", "stiffness", "damping"}) {
    std::map<std::string, double> expected;
    for (const std::string controller : {"pd_zero", "pd_stand"}) {
      const auto c = amp["controllers"][controller];
      const auto names = c["joint_list"].as<std::vector<std::string>>();
      const auto values = c[field].as<std::vector<double>>();
      for (size_t i = 0; i < names.size(); ++i) expected[names[i]] = values[i];
    }
    const auto stand = root["controllers"]["pd_stand"];
    const auto names = stand["joint_list"].as<std::vector<std::string>>();
    const auto values = stand[field].as<std::vector<double>>();
    ASSERT_EQ(names.size(), 29U);
    for (size_t i = 0; i < names.size(); ++i) EXPECT_DOUBLE_EQ(values[i], expected.at(names[i])) << names[i];
  }
}

TEST(DanceStartGate, RequiresContinuousReadinessAndResetsOnFeedbackLossOrExit) {
  DanceStartGate gate;
  gate.Configure(3.0);
  const auto start = DanceStartGate::Clock::now();
  EXPECT_FALSE(gate.Ready(start));
  for (int ms = 0; ms < 3000; ms += 20) {
    gate.Update(true, start + std::chrono::milliseconds(ms));
    EXPECT_FALSE(gate.Ready(start + std::chrono::milliseconds(ms)));
  }
  gate.Update(true, start + std::chrono::seconds(3));
  EXPECT_TRUE(gate.Ready(start + std::chrono::seconds(3)));
  gate.Update(false, start + std::chrono::milliseconds(3001));
  EXPECT_FALSE(gate.Ready(start + std::chrono::milliseconds(3001)));
  gate.Update(true, start + std::chrono::milliseconds(3020));
  EXPECT_FALSE(gate.Ready(start + std::chrono::milliseconds(3020)));
  for (int ms = 3040; ms <= 6020; ms += 20) gate.Update(true, start + std::chrono::milliseconds(ms));
  EXPECT_TRUE(gate.Ready(start + std::chrono::milliseconds(6020)));
  EXPECT_FALSE(gate.Ready(start + std::chrono::milliseconds(6200)));  // Stall cannot preserve readiness.
  gate.Update(true, start + std::chrono::milliseconds(6200));
  EXPECT_FALSE(gate.Ready(start + std::chrono::milliseconds(6200)));
  gate.Reset();
  EXPECT_FALSE(gate.Ready(start + std::chrono::milliseconds(6200)));
  EXPECT_THROW(gate.Configure(-1), std::runtime_error);
}

}  // namespace
}  // namespace xyber_x1_infer::rl_control_module
