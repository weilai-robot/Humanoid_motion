// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cmath>
#include <filesystem>
#include <string>

#include "gtest/gtest.h"

#include "dcu_driver_module/ankle_transmission.h"

namespace xyber_x1_infer::dcu_driver_module {
namespace {

constexpr double kPosTol = 5e-3;
constexpr double kVelTol = 5e-3;
constexpr double kEffortTol = 5e-3;

std::string GetAnkleYamlPath() {
  const std::filesystem::path test_path(__FILE__);
  return (test_path.parent_path().parent_path() / "cfg" / "ankle_trans_x1.yaml").string();
}

struct LeftAnkleRig {
  DataSpace act_left;
  DataSpace act_right;
  DataSpace joint_pitch;
  DataSpace joint_roll;
  LeftAnkleParallelTransmission transmission;

  LeftAnkleRig()
      : transmission("left_ankle_parallel_trans", GetAnkleYamlPath(),
                     ActuatorHandle{.direction = 1.0, .handle = &act_left},
                     ActuatorHandle{.direction = 1.0, .handle = &act_right},
                     JointHandle{.handle = &joint_pitch}, JointHandle{.handle = &joint_roll}) {}
};

struct RightAnkleRig {
  DataSpace act_left;
  DataSpace act_right;
  DataSpace joint_pitch;
  DataSpace joint_roll;
  RightAnkleParallelTransmission transmission;

  RightAnkleRig()
      : transmission("right_ankle_parallel_trans", GetAnkleYamlPath(),
                     ActuatorHandle{.direction = 1.0, .handle = &act_left},
                     ActuatorHandle{.direction = 1.0, .handle = &act_right},
                     JointHandle{.handle = &joint_pitch}, JointHandle{.handle = &joint_roll}) {}
};

template <typename Rig>
void PrimeJointStateFromCurrentActuatorState(Rig& rig) {
  rig.transmission.TransformActuatorToJoint();
}

template <typename Rig>
void ApplyActuatorCommandAsState(Rig& rig) {
  rig.act_left.state.position = rig.act_left.cmd.position;
  rig.act_left.state.velocity = rig.act_left.cmd.velocity;
  rig.act_left.state.effort = rig.act_left.cmd.effort;

  rig.act_right.state.position = rig.act_right.cmd.position;
  rig.act_right.state.velocity = rig.act_right.cmd.velocity;
  rig.act_right.state.effort = rig.act_right.cmd.effort;
}

template <typename Rig>
void ExpectRoundTrip(Rig& rig, double pitch_cmd, double roll_cmd, double pitch_vel = 0.0,
                     double roll_vel = 0.0, double pitch_effort = 0.0,
                     double roll_effort = 0.0) {
  rig.joint_pitch.cmd.position = pitch_cmd;
  rig.joint_roll.cmd.position = roll_cmd;
  rig.joint_pitch.cmd.velocity = pitch_vel;
  rig.joint_roll.cmd.velocity = roll_vel;
  rig.joint_pitch.cmd.effort = pitch_effort;
  rig.joint_roll.cmd.effort = roll_effort;

  rig.joint_pitch.cmd.kp = 35.0;
  rig.joint_pitch.cmd.kd = 0.8;
  rig.joint_roll.cmd.kp = 35.0;
  rig.joint_roll.cmd.kd = 0.8;

  rig.transmission.TransformJointToActuator();
  ApplyActuatorCommandAsState(rig);
  rig.transmission.TransformActuatorToJoint();

  EXPECT_NEAR(rig.joint_pitch.state.position, pitch_cmd, kPosTol);
  EXPECT_NEAR(rig.joint_roll.state.position, roll_cmd, kPosTol);
  EXPECT_NEAR(rig.joint_pitch.state.velocity, pitch_vel, kVelTol);
  EXPECT_NEAR(rig.joint_roll.state.velocity, roll_vel, kVelTol);
  EXPECT_NEAR(rig.joint_pitch.state.effort, pitch_effort, kEffortTol);
  EXPECT_NEAR(rig.joint_roll.state.effort, roll_effort, kEffortTol);
}

template <typename Rig>
void ExpectParallelCouplingForPitchStep(Rig& rig, double base_pitch, double base_roll,
                                        double delta_pitch) {
  rig.joint_pitch.cmd.position = base_pitch;
  rig.joint_roll.cmd.position = base_roll;
  rig.transmission.TransformJointToActuator();
  const double left_base = rig.act_left.cmd.position;
  const double right_base = rig.act_right.cmd.position;

  rig.joint_pitch.cmd.position = base_pitch + delta_pitch;
  rig.joint_roll.cmd.position = base_roll;
  rig.transmission.TransformJointToActuator();
  const double left_step = rig.act_left.cmd.position;
  const double right_step = rig.act_right.cmd.position;

  EXPECT_GT(std::abs(left_step - left_base), 1e-4);
  EXPECT_GT(std::abs(right_step - right_base), 1e-4);
}

template <typename Rig>
void ExpectParallelCouplingForRollStep(Rig& rig, double base_pitch, double base_roll,
                                       double delta_roll) {
  rig.joint_pitch.cmd.position = base_pitch;
  rig.joint_roll.cmd.position = base_roll;
  rig.transmission.TransformJointToActuator();
  const double left_base = rig.act_left.cmd.position;
  const double right_base = rig.act_right.cmd.position;

  rig.joint_pitch.cmd.position = base_pitch;
  rig.joint_roll.cmd.position = base_roll + delta_roll;
  rig.transmission.TransformJointToActuator();
  const double left_step = rig.act_left.cmd.position;
  const double right_step = rig.act_right.cmd.position;

  EXPECT_GT(std::abs(left_step - left_base), 1e-4);
  EXPECT_GT(std::abs(right_step - right_base), 1e-4);
}

}  // namespace

TEST(LeftAnkleTransmissionTest, StandPoseRoundTrip) {
  LeftAnkleRig rig;
  PrimeJointStateFromCurrentActuatorState(rig);
  ExpectRoundTrip(rig, -0.21, 0.0, 0.02, -0.01, 0.5, -0.2);
}

TEST(RightAnkleTransmissionTest, StandPoseRoundTrip) {
  RightAnkleRig rig;
  PrimeJointStateFromCurrentActuatorState(rig);
  ExpectRoundTrip(rig, -0.21, 0.0, 0.02, 0.01, 0.5, 0.2);
}

TEST(LeftAnkleTransmissionTest, PitchAndRollCommandsBothDriveParallelActuators) {
  LeftAnkleRig rig;
  PrimeJointStateFromCurrentActuatorState(rig);

  ExpectParallelCouplingForPitchStep(rig, -0.21, 0.0, 0.005);
  ExpectParallelCouplingForRollStep(rig, -0.21, 0.0, 0.005);
}

TEST(RightAnkleTransmissionTest, PitchAndRollCommandsBothDriveParallelActuators) {
  RightAnkleRig rig;
  PrimeJointStateFromCurrentActuatorState(rig);

  ExpectParallelCouplingForPitchStep(rig, -0.21, 0.0, 0.005);
  ExpectParallelCouplingForRollStep(rig, -0.21, 0.0, 0.005);
}

TEST(LeftAndRightAnkleTransmissionTest, SmallSignalSweepNearStandPose) {
  LeftAnkleRig left_rig;
  RightAnkleRig right_rig;
  PrimeJointStateFromCurrentActuatorState(left_rig);
  PrimeJointStateFromCurrentActuatorState(right_rig);

  for (double pitch = -0.22; pitch <= -0.19; pitch += 0.01) {
    for (double roll : {-0.01, 0.0, 0.01}) {
      ExpectRoundTrip(left_rig, pitch, roll);
      ExpectRoundTrip(right_rig, pitch, roll);
    }
  }
}

}  // namespace xyber_x1_infer::dcu_driver_module
