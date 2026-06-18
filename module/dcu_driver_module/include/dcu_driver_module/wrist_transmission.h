// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

// c++
#include <unordered_map>

// projects
#include "dcu_driver_module/transmission.h"
#include "util/log_util.h"

namespace xyber_x1_infer::dcu_driver_module {

class WristParallelTransmission : public Transmission {
 public:
  WristParallelTransmission(std::string name, std::string path);

 protected:
  aimrt::common::util::SimpleLogger logger_;
  static std::unordered_map<std::string, std::vector<double>> actuator_to_joint_data_;
};

class LeftWristParallelTransmission : public WristParallelTransmission {
 public:
  LeftWristParallelTransmission(std::string name, std::string param_path, ActuatorHandle actr_left,
                                ActuatorHandle actr_right, JointHandle joint_pitch,
                                JointHandle joint_roll);

  virtual void TransformActuatorToJoint() override;
  virtual void TransformJointToActuator() override;

 private:

  JointHandle joint_roll_;
  JointHandle joint_pitch_;
  ActuatorHandle actr_left_;
  ActuatorHandle actr_right_;
};

class RightWristParallelTransmission : public WristParallelTransmission {
 public:
  RightWristParallelTransmission(std::string name, std::string param_path, ActuatorHandle actr_left,
                                 ActuatorHandle actr_right, JointHandle joint_pitch,
                                 JointHandle joint_roll);

  virtual void TransformActuatorToJoint() override;
  virtual void TransformJointToActuator() override;

 private:

  JointHandle joint_roll_;
  JointHandle joint_pitch_;
  ActuatorHandle actr_left_;
  ActuatorHandle actr_right_;
};

}  // namespace xyber_x1_infer::dcu_driver_module