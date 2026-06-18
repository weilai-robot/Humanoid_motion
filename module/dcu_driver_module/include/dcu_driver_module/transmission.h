// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

// linux
#include <math.h>

// c++
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace xyber_x1_infer::dcu_driver_module {

struct DataSpace {
  struct {
    double effort = 0;
    double velocity = 0;
    double position = 0;
    double kp = 0;
    double kd = 0;
  } cmd, state;
};

/**
 * \brief Contains pointers to raw data representing the position, velocity and
 * acceleration of a transmission's actuators.
 */
struct ActuatorHandle {
  double direction = 0;
  DataSpace* handle = nullptr;
};

/**
 * \brief Contains pointers to raw data representing the position, velocity and
 * acceleration of a transmission's joints.
 */
struct JointHandle {
  DataSpace* handle = nullptr;
};

class Transmission {
 public:
  Transmission(std::string name) : name_(name) {}
  virtual ~Transmission() {}

  std::string GetName() { return name_; }

  virtual void ActuatorToJointEffort() {};
  virtual void JointToActuatorEffort() {};

  virtual void ActuatorToJointVelocity() {};
  virtual void JointToActuatorVelocity() {};

  virtual void ActuatorToJointPosition() {};
  virtual void JointToActuatorPosition() {};

  virtual void TransformActuatorToJoint() {
    ActuatorToJointEffort();
    ActuatorToJointVelocity();
    ActuatorToJointPosition();
  }
  virtual void TransformJointToActuator() {
    JointToActuatorEffort();
    JointToActuatorVelocity();
    JointToActuatorPosition();
  }

 protected:
  std::string name_;
};

class SimpleTransmission : public Transmission {
 public:
  SimpleTransmission(std::string name, ActuatorHandle actuator, JointHandle joint)
      : Transmission(name), actuator_(actuator), joint_(joint) {}

  virtual void TransformActuatorToJoint() override {
    joint_.handle->state.effort = actuator_.handle->state.effort * actuator_.direction;
    joint_.handle->state.velocity = actuator_.handle->state.velocity * actuator_.direction;
    joint_.handle->state.position = actuator_.handle->state.position * actuator_.direction;
  }

  virtual void TransformJointToActuator() override {
    actuator_.handle->cmd.effort = joint_.handle->cmd.effort * actuator_.direction;
    actuator_.handle->cmd.velocity = joint_.handle->cmd.velocity * actuator_.direction;
    actuator_.handle->cmd.position = joint_.handle->cmd.position * actuator_.direction;

    actuator_.handle->cmd.kp = joint_.handle->cmd.kp;
    actuator_.handle->cmd.kd = joint_.handle->cmd.kd;
  }

 private:
  JointHandle joint_;
  ActuatorHandle actuator_;
};

class TransimissionManager {
 public:
  ~TransimissionManager() {
    for (auto it : transmissions_) {
      delete it;
    }
  }

  void RegisterTransmission(Transmission* trans) { transmissions_.push_back(trans); }

  void TransformActuatorToJoint() {
    for (const auto& it : transmissions_) {
      it->TransformActuatorToJoint();
    }
  }
  void TransformJointToActuator() {
    for (const auto& it : transmissions_) {
      it->TransformJointToActuator();
    }
  }

 private:
  std::vector<Transmission*> transmissions_;
};

}  // namespace xyber_x1_infer::dcu_driver_module