// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "dcu_driver_module/lumbar_transmission.h"

#include <iostream>

#include "yaml-cpp/yaml.h"
namespace xyber_x1_infer::dcu_driver_module {

LumbarParallelTransmission::LumbarParallelTransmission(
    std::string name, std::string param_path, ActuatorHandle actr_left, ActuatorHandle actr_right,
    JointHandle joint_pitch, JointHandle joint_roll)
    : Transmission(name),
      actr_left_(actr_left),
      actr_right_(actr_right),
      joint_roll_(joint_roll),
      joint_pitch_(joint_pitch) {}

void LumbarParallelTransmission::TransformActuatorToJoint() {
  // pos
  double qm5 = actr_right_.handle->state.position * actr_right_.direction;
  double qm6 = actr_left_.handle->state.position * actr_left_.direction;

  // torque
  double taum5 = actr_right_.handle->state.effort * actr_right_.direction;
  // double taum6 = actr_left_.handle->state.velocity * actr_left_.direction;
  // ----
  // yumx : fix bug
  double taum6 = actr_left_.handle->state.effort * actr_left_.direction;
  // ----x
  
  // vel
  double qdm5 = actr_right_.handle->state.velocity * actr_right_.direction;
  double qdm6 = actr_left_.handle->state.velocity * actr_left_.direction;

  joint_pitch_.handle->state.position = qm5;
  joint_pitch_.handle->state.velocity = qdm5;
  joint_pitch_.handle->state.effort = taum5;

  joint_roll_.handle->state.position = qm6;
  joint_roll_.handle->state.velocity = qdm6;
  joint_roll_.handle->state.effort = taum6;
}

void LumbarParallelTransmission::TransformJointToActuator() {
  // pos
  double q5Des = joint_pitch_.handle->cmd.position;
  double q6Des = joint_roll_.handle->cmd.position;

  
  double tau5Des = joint_pitch_.handle->cmd.effort;
  double tau6Des = joint_roll_.handle->cmd.effort;

  // vel
  double qd5Des = joint_pitch_.handle->cmd.velocity;
  double qd6Des = joint_roll_.handle->cmd.velocity;

  actr_right_.handle->cmd.position = q5Des * actr_right_.direction;
  actr_right_.handle->cmd.velocity = qd5Des * actr_right_.direction;
  actr_right_.handle->cmd.effort = tau5Des * actr_right_.direction;
  actr_right_.handle->cmd.kp = joint_pitch_.handle->cmd.kp;
  actr_right_.handle->cmd.kd = joint_pitch_.handle->cmd.kd;

  actr_left_.handle->cmd.position = q6Des * actr_left_.direction;
  actr_left_.handle->cmd.velocity = qd6Des * actr_left_.direction;
  actr_left_.handle->cmd.effort = tau6Des * actr_left_.direction;
  actr_left_.handle->cmd.kp = joint_roll_.handle->cmd.kp;
  actr_left_.handle->cmd.kd = joint_roll_.handle->cmd.kd;
}

}  // namespace xyber_x1_infer::dcu_driver_module