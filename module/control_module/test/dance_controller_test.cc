#include "control_module/dance_controller.h"
#include "control_module/dance_start_gate.h"
#include "control_module/pd_controller.h"
#include "control_module/state_machine.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace xyber_x1_infer::rl_control_module {
namespace {

std::filesystem::path SourceDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path();
}

YAML::Node Config() {
  const auto root = YAML::LoadFile((SourceDir() / "cfg/rl_x1_dance.yaml").string());
  auto cfg = YAML::Clone(root["controllers"]["dance_x1"]);
  cfg["joint_limits"] = root["joint_limits"];
  cfg["control_frequency_hz"] = root["control_frequecy"];
  cfg["model_file"] = (SourceDir() / "policy/dance_x1_v11_4999.bin").string();
  cfg["motion_file"] = (SourceDir() / "policy/dance_x1_motion.bin").string();
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
};

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
      "/zero_mode", "/stand_mode", "/dance_start", "/dance_play"};
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
