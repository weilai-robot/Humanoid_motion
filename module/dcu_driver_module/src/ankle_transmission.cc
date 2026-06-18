// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "dcu_driver_module/ankle_transmission.h"

#include <iostream>

#include "yaml-cpp/yaml.h"

#define QM5_ANGLE_MAX 1
#define QM5_ANGLE_MIN -1.4
#define QM5_RESOLUTION_RATIO (0.4 / 180.0 * M_PI)
#define QM5_ANGLE_NUM ((QM5_ANGLE_MAX - QM5_ANGLE_MIN) / QM5_RESOLUTION_RATIO)

#define QM6_ANGLE_MAX 1.4
#define QM6_ANGLE_MIN -1
#define QM6_RESOLUTION_RATIO (0.4 / 180.0 * M_PI)
#define QM6_ANGLE_NUM ((QM6_ANGLE_MAX - QM6_ANGLE_MIN) / QM6_RESOLUTION_RATIO)

static constexpr double r = 25e-3;
static constexpr double l = (50.0) / 2. * 1e-3;
static constexpr double r_l = r / l;
static constexpr double l_r = l / r;

static constexpr double p_4p2_6_x = -0.041;
static constexpr double p_4p4_6_x = -0.041;

static constexpr double p_4p2_6_z = 0.009;
static constexpr double p_4p4_6_z = 0.009;

static constexpr double p_m5p1_m5_z = 0.025;
static constexpr double p_m6p3_m6_z = 0.025;

static constexpr double p_4m5_4_x = -0.041;
static constexpr double p_4m6_4_x = -0.041;

static constexpr double p_4m5_4_z = 0.195;
static constexpr double p_4m6_4_z = 0.14;

static constexpr double l_p1p2 = 0.195;
static constexpr double l_p3p4 = 0.14;

namespace xyber_x1_infer::dcu_driver_module {

std::unordered_map<std::string, std::vector<double>>
    AnkleParallelTransmission::actuator_to_joint_data_;

AnkleParallelTransmission::AnkleParallelTransmission(std::string name, std::string path)
    : Transmission(name) {
  logger_ = aimrt::common::util::SimpleLogger();
  if (actuator_to_joint_data_.empty()) {
    AIMRT_HL_INFO(logger_, "actuator_to_joint_data_ loading.");
    YAML::Node config = YAML::LoadFile(path);
    std::vector<double> parse_vec;
    auto iter = config.begin();
    for (size_t i = 1; i <= QM5_ANGLE_NUM; i++) {
      for (size_t j = 1; j <= QM6_ANGLE_NUM; j++) {
        std::vector<double>().swap(parse_vec);
        parse_vec.push_back((iter->second)["q5"].as<double>());
        parse_vec.push_back((iter->second)["q6"].as<double>());
        actuator_to_joint_data_.insert(std::make_pair(iter->first.as<std::string>(), parse_vec));
        if (iter != config.end()) {
          iter++;
        }
      }
    }
  }
}

LeftAnkleParallelTransmission::LeftAnkleParallelTransmission(
    std::string name, std::string param_path, ActuatorHandle actr_left, ActuatorHandle actr_right,
    JointHandle joint_pitch, JointHandle joint_roll)
    : AnkleParallelTransmission(name, param_path),
      actr_left_(actr_left),
      actr_right_(actr_right),
      joint_roll_(joint_roll),
      joint_pitch_(joint_pitch) {}

void LeftAnkleParallelTransmission::TransformActuatorToJoint() {
  // pos
  double qm5 = actr_right_.handle->state.position * actr_right_.direction;
  double qm6 = actr_left_.handle->state.position * actr_left_.direction;
  qm5 *= -1;
  qm6 *= -1;

  int qm5_num_int = (int)((qm5 - QM5_ANGLE_MIN) / QM5_RESOLUTION_RATIO);
  int qm6_num_int = (int)((qm6 - QM6_ANGLE_MIN) / QM6_RESOLUTION_RATIO);

  double qmLss_5_num_double = (double)((qm5 - QM5_ANGLE_MIN) / QM5_RESOLUTION_RATIO);
  double qmLss_6_num_double = (double)((qm6 - QM6_ANGLE_MIN) / QM6_RESOLUTION_RATIO);

  if (qm5_num_int < 0) {
    std::cout << "qm5_num_int is error" << std::endl;
  }

  if (qm6_num_int < 0) {
    std::cout << "qm6_num_int is error" << std::endl;
  }

  std::string param1 = "qm5qm6_" + std::to_string(qm5_num_int + 1);
  param1 += "_" + std::to_string(qm6_num_int + 1);

  std::string param2 = "qm5qm6_" + std::to_string(qm5_num_int + 2);
  param2 += "_" + std::to_string(qm6_num_int + 2);

  auto iter1 = actuator_to_joint_data_.find(param1);
  auto iter2 = actuator_to_joint_data_.find(param2);

  double qm5_lerp = qmLss_5_num_double - qm5_num_int;
  double qm6_lerp = qmLss_6_num_double - qm6_num_int;
  double lerp = std::sqrt((pow(qm5_lerp, 2) + pow(qm6_lerp, 2)) / 2);

  double q5 = 0;
  double q6 = 0;

  if (iter1 != actuator_to_joint_data_.end() && iter2 != actuator_to_joint_data_.end()) {
    lerp = std::clamp(lerp, 0.0, 1.0);

    q5 =
        qm5_lerp * (actuator_to_joint_data_[param2].at(0) - actuator_to_joint_data_[param1].at(0)) +
        actuator_to_joint_data_[param1].at(0);
    q6 =
        qm6_lerp * (actuator_to_joint_data_[param2].at(1) - actuator_to_joint_data_[param1].at(1)) +
        actuator_to_joint_data_[param1].at(1);
  } else {
    std::cout << "actuator_to_joint_data_ out of range" << std::endl;
  }

  q6 *= -1;
  qm5 *= -1;
  qm6 *= -1;

  // torque
  double taum5 = actr_right_.handle->state.effort * actr_right_.direction;
  // double taum6 = actr_left_.handle->state.velocity * actr_left_.direction;
  // -----
  // yumx : fix bug
  double taum6 = actr_left_.handle->state.effort * actr_left_.direction;
  // -----
  double cq5 = cos(q5);
  double sq5 = sin(q5);
  double cq6 = cos(q6);
  double sq6 = sin(q6);
  double cqm5 = 0;
  double sqm5 = 0;
  double cqm6 = 0;
  double sqm6 = 0;

  cqm5 = cos(qm5 + 1.2028);
  sqm5 = sin(qm5 + 1.2028);
  cqm6 = cos(qm6 - 1.2030);
  sqm6 = sin(qm6 - 1.2030);
  p_4p2_6_y = -0.025;
  p_4p4_6_y = 0.025;

  double delta14a = 9 * sq6 / 1000;
  double delta13a = 41 * sq5 / 1000;
  double delta12a = 9 * cq5 * cq6;
  double delta11a = 41 * cq5 / 1000;
  double delta10a = 9 * cq6 * sq5;
  double delta9a = sqm5 / 40 - delta14a + p_4p2_6_y * cq6;
  double delta8a =
      delta13a - cqm5 / 40 + delta12a / 1000 + p_4p2_6_y * cq5 * sq6 - double(39) / 200;
  double delta7a = delta10a / 1000 - delta11a + p_4p4_6_y * sq5 * sq6 + double(41) / 1000;
  double delta6a = delta13a - cqm6 / 40 + delta12a / 1000 + p_4p4_6_y * cq5 * sq6 - double(7) / 50;
  double delta5a =
      sqrt(pow(delta10a / 1000 - delta11a + p_4p2_6_y * sq5 * sq6 + double(41) / 1000, 2) +
           pow(delta8a, 2) + pow(delta9a, 2));
  double delta4a =
      1000 * (40 * sqm6 * delta8a / delta5a + 40 * cqm6 * delta9a / delta5a) *
      sqrt(pow(delta7a, 2) + pow(delta6a, 2) + pow(sqm6 / 40 - delta14a + p_4p4_6_y * cq6, 2));
  double delta3a = 1;
  double delta2a = 1600;
  double delta1a = 25 * (25 * cqm5 * sqm5 - 9 * cqm5 * sq6 - 195 * sqm5 - 25 * sqm5 * cqm5 +
                         41 * sqm5 * sq5 + 9 * sqm5 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm5 * cq6 +
                         1000 * p_4p2_6_y * sqm5 * cq5 * sq6);

  double f1a =
      -(41 * sq5 * delta3a * (delta10a - 41 * cq5 + 1000 * p_4p2_6_y * sq5 * sq6 + 41)) / delta1a -
      (41 * cq5 * delta3a *
       (41 * sq5 - 25 * cqm5 + delta12a + 1000 * p_4p2_6_y * cq5 * sq6 - 195)) /
          delta1a;
  double f1b = -(41 * sq5 * delta2a * delta7a) / delta4a - (41 * cq5 * delta2a * delta6a) / delta4a;

  double delta14b = 9 * sq6 / 1000;
  double delta13b = 41 * sq5 / 1000;
  double delta12b = 9 * cq5 * cq6 / 1000;
  double delta11b = 41 * cq5 / 1000;
  double delta10b = 9 * cq6 * sq5 / 1000;
  double delta9b = sqm6 / 40 - delta14b + p_4p4_6_y * cq6;
  double delta8b = sqm5 / 40 - delta14b + p_4p2_6_y * cq6;
  double delta7b = delta13b - cqm5 / 40 + delta12b + p_4p2_6_y * cq5 * sq6 - double(39) / 200;
  double delta6b =
      sqrt(pow(delta10b - delta11b + p_4p4_6_y * sq5 * sq6 + double(41) / 1000, 2) +
           pow(delta13b - cqm6 / 40 + delta12b + p_4p4_6_y * cq5 * sq6 - double(7) / 50, 2) +
           pow(delta9b, 2));
  double delta5b = sqrt(pow(delta10b - delta11b + p_4p2_6_y * sq5 * sq6 + double(41) / 1000, 2) +
                        pow(delta7b, 2) + pow(delta8b, 2));
  double delta3b = 1;
  double delta2b = 1;
  double delta1b =
      (25 * cqm5 * sqm5 - 9 * cqm5 * sq6 - 195 * sqm5 - 25 * sqm5 * cqm5 + 41 * sqm5 * sq5 +
       9 * sqm5 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm5 * cq6 + 1000 * p_4p2_6_y * sqm5 * cq5 * sq6);

  double f2a = sq6 * delta2b * (25 * sqm5 - 9 * sq6 + 1000 * p_4p2_6_y * cq6) / delta1b -
               cq6 * delta2b *
                   (9 * cq6 - 195 * cq5 + 41 * sq5 - 25 * cq5 * cqm5 + 1000 * p_4p2_6_y * sq6) /
                   delta1b;
  double f2b =
      -sq6 * delta3b * delta9b * delta5b / ((sqm6 * delta7b + cqm6 * delta8b) * delta6b) +
      cq6 * delta3b * delta5b *
          (9 * cq6 - 140 * cq5 + 41 * sq5 - 25 * cq5 * cqm6 + 1000 * p_4p4_6_y * sq6) /
          (delta6b * (25 * cqm6 * sqm5 - 9 * cqm6 * sq6 - 195 * sqm6 - 25 * sqm6 * cqm5 +
                      41 * sqm6 * sq5 + 9 * sqm6 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm6 * cq6 +
                      1000 * p_4p2_6_y * sqm6 * cq5 * sq6));

  f2a *= -1;
  f2b *= -1;

  double tauj5 = f1a * taum5 + f1b * taum6;
  double tauj6 = f2a * taum5 + f2b * taum6;

  // vel
  double qdm5 = actr_right_.handle->state.velocity * actr_right_.direction;
  double qdm6 = actr_left_.handle->state.velocity * actr_left_.direction;

  double det11 = f2b / (f2b * f1a - f1b * f2a);
  double det12 = -f1b / (f2b * f1a - f1b * f2a);
  double det21 = f2a / (f2a * f1b - f1a * f2b);
  double det22 = -f1a / (f2a * f1b - f1a * f2b);

  Eigen::Matrix<double, 2, 2> J_T;
  J_T << det11, det12, det21, det22;

  Eigen::Matrix<double, 2, 1> qdm56;
  qdm56 << qdm5, qdm6;
  Eigen::Matrix<double, 2, 1> qd56;
  qd56 = J_T.transpose() * qdm56;

  double qd5 = qd56[0];
  double qd6 = qd56[1];

  joint_pitch_.handle->state.position = q5;
  joint_pitch_.handle->state.velocity = qd5;
  joint_pitch_.handle->state.effort = tauj5;

  joint_roll_.handle->state.position = q6;
  joint_roll_.handle->state.velocity = qd6;
  joint_roll_.handle->state.effort = tauj6;
}

void LeftAnkleParallelTransmission::TransformJointToActuator() {
  // pos
  double q5Des = joint_pitch_.handle->cmd.position;
  double q6Des = joint_roll_.handle->cmd.position;

  p_4p2_6_y = -0.025;
  p_4p4_6_y = 0.025;

  double delta11 = -pow(l_p1p2, 2) +
                   pow((p_4p2_6_x * cos(q5Des) - p_4m5_4_x + p_4p2_6_z * cos(q6Des) * sin(q5Des) +
                        p_4p2_6_y * sin(q5Des) * sin(q6Des)),
                       2);
  double delta12 = p_4m5_4_z + p_4p2_6_x * sin(q5Des) - p_4p2_6_z * cos(q5Des) * cos(q6Des) -
                   p_4p2_6_y * cos(q5Des) * sin(q6Des);
  double delta13 = p_4p2_6_y * cos(q6Des) - p_4p2_6_z * sin(q6Des);

  double a1 = 2 * delta13 * p_m5p1_m5_z;
  double b1 = 2 * delta12 * p_m5p1_m5_z;
  double c1 = -(delta11 + pow(delta12, 2) + pow(delta13, 2) + pow(p_m5p1_m5_z, 2));

  double delta21 = -pow(l_p3p4, 2) +
                   pow((p_4p4_6_x * cos(q5Des) - p_4m6_4_x + p_4p4_6_z * cos(q6Des) * sin(q5Des) +
                        p_4p4_6_y * sin(q5Des) * sin(q6Des)),
                       2);
  double delta22 = p_4m6_4_z + p_4p4_6_x * sin(q5Des) - p_4p4_6_z * cos(q5Des) * cos(q6Des) -
                   p_4p4_6_y * cos(q5Des) * sin(q6Des);
  double delta23 = p_4p4_6_y * cos(q6Des) - p_4p4_6_z * sin(q6Des);

  double a2 = 2 * delta23 * p_m6p3_m6_z;
  double b2 = 2 * delta22 * p_m6p3_m6_z;
  double c2 = -(delta21 + pow(delta22, 2) + pow(delta23, 2) + pow(p_m6p3_m6_z, 2));

  double qm5Des = 0;
  double qm6Des = 0;

  qm5Des = acos(c1 / sqrt(pow(a1, 2) + pow(b1, 2))) + atan(a1 / b1) - 1.2028;
  qm6Des = -acos(c2 / sqrt(pow(a2, 2) + pow(b2, 2))) + atan(a2 / b2) + 1.2030;

  // torque
  double q5 = joint_pitch_.handle->state.position;
  double q6 = joint_roll_.handle->state.position;

  double qm5 = actr_right_.handle->state.position * actr_right_.direction;
  double qm6 = actr_left_.handle->state.position * actr_left_.direction;

  double tau5Des = joint_pitch_.handle->cmd.effort;
  double tau6Des = joint_roll_.handle->cmd.effort;

  double cq5 = cos(q5);
  double sq5 = sin(q5);
  double cq6 = cos(q6);
  double sq6 = sin(q6);
  double cqm5 = 0;
  double sqm5 = 0;
  double cqm6 = 0;
  double sqm6 = 0;

  cqm5 = cos(qm5 + 1.2028);
  sqm5 = sin(qm5 + 1.2028);
  cqm6 = cos(qm6 - 1.2030);
  sqm6 = sin(qm6 - 1.2030);
  p_4p2_6_y = -0.025;
  p_4p4_6_y = 0.025;

  double delta14a = 9 * sq6 / 1000;
  double delta13a = 41 * sq5 / 1000;
  double delta12a = 9 * cq5 * cq6;
  double delta11a = 41 * cq5 / 1000;
  double delta10a = 9 * cq6 * sq5;
  double delta9a = sqm5 / 40 - delta14a + p_4p2_6_y * cq6;
  double delta8a =
      delta13a - cqm5 / 40 + delta12a / 1000 + p_4p2_6_y * cq5 * sq6 - double(39) / 200;
  double delta7a = delta10a / 1000 - delta11a + p_4p4_6_y * sq5 * sq6 + double(41) / 1000;
  double delta6a = delta13a - cqm6 / 40 + delta12a / 1000 + p_4p4_6_y * cq5 * sq6 - double(7) / 50;
  double delta5a =
      sqrt(pow(delta10a / 1000 - delta11a + p_4p2_6_y * sq5 * sq6 + double(41) / 1000, 2) +
           pow(delta8a, 2) + pow(delta9a, 2));
  double delta4a =
      1000 * (40 * sqm6 * delta8a / delta5a + 40 * cqm6 * delta9a / delta5a) *
      sqrt(pow(delta7a, 2) + pow(delta6a, 2) + pow(sqm6 / 40 - delta14a + p_4p4_6_y * cq6, 2));
  double delta3a = 1;
  double delta2a = 1600;
  double delta1a = 25 * (25 * cqm5 * sqm5 - 9 * cqm5 * sq6 - 195 * sqm5 - 25 * sqm5 * cqm5 +
                         41 * sqm5 * sq5 + 9 * sqm5 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm5 * cq6 +
                         1000 * p_4p2_6_y * sqm5 * cq5 * sq6);

  double f1a =
      -(41 * sq5 * delta3a * (delta10a - 41 * cq5 + 1000 * p_4p2_6_y * sq5 * sq6 + 41)) / delta1a -
      (41 * cq5 * delta3a *
       (41 * sq5 - 25 * cqm5 + delta12a + 1000 * p_4p2_6_y * cq5 * sq6 - 195)) /
          delta1a;
  double f1b = -(41 * sq5 * delta2a * delta7a) / delta4a - (41 * cq5 * delta2a * delta6a) / delta4a;

  double delta14b = 9 * sq6 / 1000;
  double delta13b = 41 * sq5 / 1000;
  double delta12b = 9 * cq5 * cq6 / 1000;
  double delta11b = 41 * cq5 / 1000;
  double delta10b = 9 * cq6 * sq5 / 1000;
  double delta9b = sqm6 / 40 - delta14b + p_4p4_6_y * cq6;
  double delta8b = sqm5 / 40 - delta14b + p_4p2_6_y * cq6;
  double delta7b = delta13b - cqm5 / 40 + delta12b + p_4p2_6_y * cq5 * sq6 - double(39) / 200;
  double delta6b =
      sqrt(pow(delta10b - delta11b + p_4p4_6_y * sq5 * sq6 + double(41) / 1000, 2) +
           pow(delta13b - cqm6 / 40 + delta12b + p_4p4_6_y * cq5 * sq6 - double(7) / 50, 2) +
           pow(delta9b, 2));
  double delta5b = sqrt(pow(delta10b - delta11b + p_4p2_6_y * sq5 * sq6 + double(41) / 1000, 2) +
                        pow(delta7b, 2) + pow(delta8b, 2));
  double delta3b = 1;
  double delta2b = 1;
  double delta1b =
      (25 * cqm5 * sqm5 - 9 * cqm5 * sq6 - 195 * sqm5 - 25 * sqm5 * cqm5 + 41 * sqm5 * sq5 +
       9 * sqm5 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm5 * cq6 + 1000 * p_4p2_6_y * sqm5 * cq5 * sq6);

  double f2a = sq6 * delta2b * (25 * sqm5 - 9 * sq6 + 1000 * p_4p2_6_y * cq6) / delta1b -
               cq6 * delta2b *
                   (9 * cq6 - 195 * cq5 + 41 * sq5 - 25 * cq5 * cqm5 + 1000 * p_4p2_6_y * sq6) /
                   delta1b;
  double f2b =
      -sq6 * delta3b * delta9b * delta5b / ((sqm6 * delta7b + cqm6 * delta8b) * delta6b) +
      cq6 * delta3b * delta5b *
          (9 * cq6 - 140 * cq5 + 41 * sq5 - 25 * cq5 * cqm6 + 1000 * p_4p4_6_y * sq6) /
          (delta6b * (25 * cqm6 * sqm5 - 9 * cqm6 * sq6 - 195 * sqm6 - 25 * sqm6 * cqm5 +
                      41 * sqm6 * sq5 + 9 * sqm6 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm6 * cq6 +
                      1000 * p_4p2_6_y * sqm6 * cq5 * sq6));

  f2a *= -1;
  f2b *= -1;

  double taum5Des = (f2b * tau5Des - f1b * tau6Des) / (f2b * f1a - f1b * f2a);
  double taum6Des = (f2a * tau5Des - f1a * tau6Des) / (f2a * f1b - f1a * f2b);

  // vel
  double qd5_cmd = joint_pitch_.handle->cmd.velocity;
  double qd6_cmd = joint_roll_.handle->cmd.velocity;
  double qdm5_cmd = 0;
  double qdm6_cmd = 0;

  // vel
  double qd5Des = joint_pitch_.handle->cmd.velocity;
  double qd6Des = joint_roll_.handle->cmd.velocity;

  double det11 = f2b / (f2b * f1a - f1b * f2a);
  double det12 = -f1b / (f2b * f1a - f1b * f2a);
  double det21 = f2a / (f2a * f1b - f1a * f2b);
  double det22 = -f1a / (f2a * f1b - f1a * f2b);

  Eigen::Matrix<double, 2, 2> J_T;
  J_T << det11, det12, det21, det22;
  Eigen::Matrix<double, 2, 2> J;
  J = J_T.transpose();

  Eigen::Matrix<double, 2, 1> qd56Des;
  qd56Des << qd5Des, qd6Des;
  // double determinant = J_T.determinant();

  // // 检查行列式是否为零
  // if (determinant == 0) {
  //     std::cerr << "Error: Matrix is singular and cannot be inverted." << std::endl;
  //     return -1;
  // }
  Eigen::Matrix<double, 2, 1> qdm56Des;
  qdm56Des = J.inverse() * qd56Des;

  double qdm5Des = qdm56Des[0];
  double qdm6Des = qdm56Des[1];

  actr_right_.handle->cmd.position = qm5Des * actr_right_.direction;
  actr_right_.handle->cmd.velocity = qdm5Des * actr_right_.direction;
  actr_right_.handle->cmd.effort = taum5Des * actr_right_.direction;
  actr_right_.handle->cmd.kp = joint_pitch_.handle->cmd.kp;
  actr_right_.handle->cmd.kd = joint_pitch_.handle->cmd.kd;

  actr_left_.handle->cmd.position = qm6Des * actr_left_.direction;
  actr_left_.handle->cmd.velocity = qdm6Des * actr_left_.direction;
  actr_left_.handle->cmd.effort = taum6Des * actr_left_.direction;
  actr_left_.handle->cmd.kp = joint_roll_.handle->cmd.kp;
  actr_left_.handle->cmd.kd = joint_roll_.handle->cmd.kd;
}

RightAnkleParallelTransmission::RightAnkleParallelTransmission(
    std::string name, std::string param_path, ActuatorHandle actr_left, ActuatorHandle actr_right,
    JointHandle joint_pitch, JointHandle joint_roll)
    : AnkleParallelTransmission(name, param_path),
      actr_left_(actr_left),
      actr_right_(actr_right),
      joint_roll_(joint_roll),
      joint_pitch_(joint_pitch) {}

void RightAnkleParallelTransmission::TransformActuatorToJoint() {
  // pos
  double qm5 = actr_left_.handle->state.position * actr_left_.direction;
  double qm6 = actr_right_.handle->state.position * actr_right_.direction;

  int qm5_num_int = (int)((qm5 - QM5_ANGLE_MIN) / QM5_RESOLUTION_RATIO);
  int qm6_num_int = (int)((qm6 - QM6_ANGLE_MIN) / QM6_RESOLUTION_RATIO);

  double qmLss_5_num_double = (double)((qm5 - QM5_ANGLE_MIN) / QM5_RESOLUTION_RATIO);
  double qmLss_6_num_double = (double)((qm6 - QM6_ANGLE_MIN) / QM6_RESOLUTION_RATIO);

  if (qm5_num_int < 0) {
    std::cout << "qm5_num_int is error" << std::endl;
  }

  if (qm6_num_int < 0) {
    std::cout << "qm6_num_int is error" << std::endl;
  }

  std::string param1 = "qm5qm6_" + std::to_string(qm5_num_int + 1);
  param1 += "_" + std::to_string(qm6_num_int + 1);

  std::string param2 = "qm5qm6_" + std::to_string(qm5_num_int + 2);
  param2 += "_" + std::to_string(qm6_num_int + 2);

  auto iter1 = actuator_to_joint_data_.find(param1);
  auto iter2 = actuator_to_joint_data_.find(param2);

  double qm5_lerp = qmLss_5_num_double - qm5_num_int;
  double qm6_lerp = qmLss_6_num_double - qm6_num_int;
  double lerp = std::sqrt((pow(qm5_lerp, 2) + pow(qm6_lerp, 2)) / 2);

  double q5 = 0;
  double q6 = 0;

  if (iter1 != actuator_to_joint_data_.end() && iter2 != actuator_to_joint_data_.end()) {
    lerp = std::clamp(lerp, 0.0, 1.0);

    q5 =
        qm5_lerp * (actuator_to_joint_data_[param2].at(0) - actuator_to_joint_data_[param1].at(0)) +
        actuator_to_joint_data_[param1].at(0);
    q6 =
        qm6_lerp * (actuator_to_joint_data_[param2].at(1) - actuator_to_joint_data_[param1].at(1)) +
        actuator_to_joint_data_[param1].at(1);
  } else {
    std::cout << "actuator_to_joint_data_ out of range" << std::endl;
  }

  // torque
  double taum5 = actr_left_.handle->state.effort * actr_left_.direction;
  double taum6 = actr_right_.handle->state.effort * actr_right_.direction;

  double cq5 = cos(q5);
  double sq5 = sin(q5);
  double cq6 = cos(q6);
  double sq6 = sin(q6);
  double cqm5 = 0;
  double sqm5 = 0;
  double cqm6 = 0;
  double sqm6 = 0;

  cqm5 = cos(qm5 - 1.2028);
  sqm5 = sin(qm5 - 1.2028);
  cqm6 = cos(qm6 + 1.2030);
  sqm6 = sin(qm6 + 1.2030);
  p_4p2_6_y = 0.025;
  p_4p4_6_y = -0.025;

  double delta14a = 9 * sq6 / 1000;
  double delta13a = 41 * sq5 / 1000;
  double delta12a = 9 * cq5 * cq6;
  double delta11a = 41 * cq5 / 1000;
  double delta10a = 9 * cq6 * sq5;
  double delta9a = sqm5 / 40 - delta14a + p_4p2_6_y * cq6;
  double delta8a =
      delta13a - cqm5 / 40 + delta12a / 1000 + p_4p2_6_y * cq5 * sq6 - double(39) / 200;
  double delta7a = delta10a / 1000 - delta11a + p_4p4_6_y * sq5 * sq6 + double(41) / 1000;
  double delta6a = delta13a - cqm6 / 40 + delta12a / 1000 + p_4p4_6_y * cq5 * sq6 - double(7) / 50;
  double delta5a =
      sqrt(pow(delta10a / 1000 - delta11a + p_4p2_6_y * sq5 * sq6 + double(41) / 1000, 2) +
           pow(delta8a, 2) + pow(delta9a, 2));
  double delta4a =
      1000 * (40 * sqm6 * delta8a / delta5a + 40 * cqm6 * delta9a / delta5a) *
      sqrt(pow(delta7a, 2) + pow(delta6a, 2) + pow(sqm6 / 40 - delta14a + p_4p4_6_y * cq6, 2));
  double delta3a = 1;
  double delta2a = 1600;
  double delta1a = 25 * (25 * cqm5 * sqm5 - 9 * cqm5 * sq6 - 195 * sqm5 - 25 * sqm5 * cqm5 +
                         41 * sqm5 * sq5 + 9 * sqm5 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm5 * cq6 +
                         1000 * p_4p2_6_y * sqm5 * cq5 * sq6);

  double f1a =
      -(41 * sq5 * delta3a * (delta10a - 41 * cq5 + 1000 * p_4p2_6_y * sq5 * sq6 + 41)) / delta1a -
      (41 * cq5 * delta3a *
       (41 * sq5 - 25 * cqm5 + delta12a + 1000 * p_4p2_6_y * cq5 * sq6 - 195)) /
          delta1a;
  double f1b = -(41 * sq5 * delta2a * delta7a) / delta4a - (41 * cq5 * delta2a * delta6a) / delta4a;

  double delta14b = 9 * sq6 / 1000;
  double delta13b = 41 * sq5 / 1000;
  double delta12b = 9 * cq5 * cq6 / 1000;
  double delta11b = 41 * cq5 / 1000;
  double delta10b = 9 * cq6 * sq5 / 1000;
  double delta9b = sqm6 / 40 - delta14b + p_4p4_6_y * cq6;
  double delta8b = sqm5 / 40 - delta14b + p_4p2_6_y * cq6;
  double delta7b = delta13b - cqm5 / 40 + delta12b + p_4p2_6_y * cq5 * sq6 - double(39) / 200;
  double delta6b =
      sqrt(pow(delta10b - delta11b + p_4p4_6_y * sq5 * sq6 + double(41) / 1000, 2) +
           pow(delta13b - cqm6 / 40 + delta12b + p_4p4_6_y * cq5 * sq6 - double(7) / 50, 2) +
           pow(delta9b, 2));
  double delta5b = sqrt(pow(delta10b - delta11b + p_4p2_6_y * sq5 * sq6 + double(41) / 1000, 2) +
                        pow(delta7b, 2) + pow(delta8b, 2));
  double delta3b = 1;
  double delta2b = 1;
  double delta1b =
      (25 * cqm5 * sqm5 - 9 * cqm5 * sq6 - 195 * sqm5 - 25 * sqm5 * cqm5 + 41 * sqm5 * sq5 +
       9 * sqm5 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm5 * cq6 + 1000 * p_4p2_6_y * sqm5 * cq5 * sq6);

  double f2a = sq6 * delta2b * (25 * sqm5 - 9 * sq6 + 1000 * p_4p2_6_y * cq6) / delta1b -
               cq6 * delta2b *
                   (9 * cq6 - 195 * cq5 + 41 * sq5 - 25 * cq5 * cqm5 + 1000 * p_4p2_6_y * sq6) /
                   delta1b;
  double f2b =
      -sq6 * delta3b * delta9b * delta5b / ((sqm6 * delta7b + cqm6 * delta8b) * delta6b) +
      cq6 * delta3b * delta5b *
          (9 * cq6 - 140 * cq5 + 41 * sq5 - 25 * cq5 * cqm6 + 1000 * p_4p4_6_y * sq6) /
          (delta6b * (25 * cqm6 * sqm5 - 9 * cqm6 * sq6 - 195 * sqm6 - 25 * sqm6 * cqm5 +
                      41 * sqm6 * sq5 + 9 * sqm6 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm6 * cq6 +
                      1000 * p_4p2_6_y * sqm6 * cq5 * sq6));

  double tauj5 = f1a * taum5 + f1b * taum6;
  double tauj6 = f2a * taum5 + f2b * taum6;

  // vel
  double qdm5 = actr_left_.handle->state.velocity * actr_left_.direction;
  double qdm6 = actr_right_.handle->state.velocity * actr_right_.direction;

  double det11 = f2b / (f2b * f1a - f1b * f2a);
  double det12 = -f1b / (f2b * f1a - f1b * f2a);
  double det21 = f2a / (f2a * f1b - f1a * f2b);
  double det22 = -f1a / (f2a * f1b - f1a * f2b);

  Eigen::Matrix<double, 2, 2> J_T;
  J_T << det11, det12, det21, det22;

  Eigen::Matrix<double, 2, 1> qdm56;
  qdm56 << qdm5, qdm6;
  Eigen::Matrix<double, 2, 1> qd56;
  qd56 = J_T.transpose() * qdm56;

  double qd5 = qd56[0];
  double qd6 = qd56[1];

  joint_pitch_.handle->state.position = q5;
  joint_pitch_.handle->state.velocity = qd5;
  joint_pitch_.handle->state.effort = tauj5;

  joint_roll_.handle->state.position = q6;
  joint_roll_.handle->state.velocity = qd6;
  joint_roll_.handle->state.effort = tauj6;
}

void RightAnkleParallelTransmission::TransformJointToActuator() {
  // pos
  double q5Des = joint_pitch_.handle->cmd.position;
  double q6Des = joint_roll_.handle->cmd.position;

  p_4p2_6_y = 0.025;
  p_4p4_6_y = -0.025;

  double delta11 = -pow(l_p1p2, 2) +
                   pow((p_4p2_6_x * cos(q5Des) - p_4m5_4_x + p_4p2_6_z * cos(q6Des) * sin(q5Des) +
                        p_4p2_6_y * sin(q5Des) * sin(q6Des)),
                       2);
  double delta12 = p_4m5_4_z + p_4p2_6_x * sin(q5Des) - p_4p2_6_z * cos(q5Des) * cos(q6Des) -
                   p_4p2_6_y * cos(q5Des) * sin(q6Des);
  double delta13 = p_4p2_6_y * cos(q6Des) - p_4p2_6_z * sin(q6Des);

  double a1 = 2 * delta13 * p_m5p1_m5_z;
  double b1 = 2 * delta12 * p_m5p1_m5_z;
  double c1 = -(delta11 + pow(delta12, 2) + pow(delta13, 2) + pow(p_m5p1_m5_z, 2));

  double delta21 = -pow(l_p3p4, 2) +
                   pow((p_4p4_6_x * cos(q5Des) - p_4m6_4_x + p_4p4_6_z * cos(q6Des) * sin(q5Des) +
                        p_4p4_6_y * sin(q5Des) * sin(q6Des)),
                       2);
  double delta22 = p_4m6_4_z + p_4p4_6_x * sin(q5Des) - p_4p4_6_z * cos(q5Des) * cos(q6Des) -
                   p_4p4_6_y * cos(q5Des) * sin(q6Des);
  double delta23 = p_4p4_6_y * cos(q6Des) - p_4p4_6_z * sin(q6Des);

  double a2 = 2 * delta23 * p_m6p3_m6_z;
  double b2 = 2 * delta22 * p_m6p3_m6_z;
  double c2 = -(delta21 + pow(delta22, 2) + pow(delta23, 2) + pow(p_m6p3_m6_z, 2));

  double qm5Des = 0;
  double qm6Des = 0;

  qm5Des = -acos(c1 / sqrt(pow(a1, 2) + pow(b1, 2))) + atan(a1 / b1) + 1.2028;
  qm6Des = acos(c2 / sqrt(pow(a2, 2) + pow(b2, 2))) + atan(a2 / b2) - 1.2030;

  // torque
  double q5 = joint_pitch_.handle->state.position;
  double q6 = joint_roll_.handle->state.position;

  double qm5 = actr_left_.handle->state.position * actr_left_.direction;
  double qm6 = actr_right_.handle->state.position * actr_right_.direction;

  double tau5Des = joint_pitch_.handle->cmd.effort;
  double tau6Des = joint_roll_.handle->cmd.effort;

  double cq5 = cos(q5);
  double sq5 = sin(q5);
  double cq6 = cos(q6);
  double sq6 = sin(q6);
  double cqm5 = 0;
  double sqm5 = 0;
  double cqm6 = 0;
  double sqm6 = 0;

  cqm5 = cos(qm5 - 1.2028);
  sqm5 = sin(qm5 - 1.2028);
  cqm6 = cos(qm6 + 1.2030);
  sqm6 = sin(qm6 + 1.2030);
  p_4p2_6_y = 0.025;
  p_4p4_6_y = -0.025;

  double delta14a = 9 * sq6 / 1000;
  double delta13a = 41 * sq5 / 1000;
  double delta12a = 9 * cq5 * cq6;
  double delta11a = 41 * cq5 / 1000;
  double delta10a = 9 * cq6 * sq5;
  double delta9a = sqm5 / 40 - delta14a + p_4p2_6_y * cq6;
  double delta8a =
      delta13a - cqm5 / 40 + delta12a / 1000 + p_4p2_6_y * cq5 * sq6 - double(39) / 200;
  double delta7a = delta10a / 1000 - delta11a + p_4p4_6_y * sq5 * sq6 + double(41) / 1000;
  double delta6a = delta13a - cqm6 / 40 + delta12a / 1000 + p_4p4_6_y * cq5 * sq6 - double(7) / 50;
  double delta5a =
      sqrt(pow(delta10a / 1000 - delta11a + p_4p2_6_y * sq5 * sq6 + double(41) / 1000, 2) +
           pow(delta8a, 2) + pow(delta9a, 2));
  double delta4a =
      1000 * (40 * sqm6 * delta8a / delta5a + 40 * cqm6 * delta9a / delta5a) *
      sqrt(pow(delta7a, 2) + pow(delta6a, 2) + pow(sqm6 / 40 - delta14a + p_4p4_6_y * cq6, 2));
  double delta3a = 1;
  double delta2a = 1600;
  double delta1a = 25 * (25 * cqm5 * sqm5 - 9 * cqm5 * sq6 - 195 * sqm5 - 25 * sqm5 * cqm5 +
                         41 * sqm5 * sq5 + 9 * sqm5 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm5 * cq6 +
                         1000 * p_4p2_6_y * sqm5 * cq5 * sq6);

  double f1a =
      -(41 * sq5 * delta3a * (delta10a - 41 * cq5 + 1000 * p_4p2_6_y * sq5 * sq6 + 41)) / delta1a -
      (41 * cq5 * delta3a *
       (41 * sq5 - 25 * cqm5 + delta12a + 1000 * p_4p2_6_y * cq5 * sq6 - 195)) /
          delta1a;
  double f1b = -(41 * sq5 * delta2a * delta7a) / delta4a - (41 * cq5 * delta2a * delta6a) / delta4a;

  double delta14b = 9 * sq6 / 1000;
  double delta13b = 41 * sq5 / 1000;
  double delta12b = 9 * cq5 * cq6 / 1000;
  double delta11b = 41 * cq5 / 1000;
  double delta10b = 9 * cq6 * sq5 / 1000;
  double delta9b = sqm6 / 40 - delta14b + p_4p4_6_y * cq6;
  double delta8b = sqm5 / 40 - delta14b + p_4p2_6_y * cq6;
  double delta7b = delta13b - cqm5 / 40 + delta12b + p_4p2_6_y * cq5 * sq6 - double(39) / 200;
  double delta6b =
      sqrt(pow(delta10b - delta11b + p_4p4_6_y * sq5 * sq6 + double(41) / 1000, 2) +
           pow(delta13b - cqm6 / 40 + delta12b + p_4p4_6_y * cq5 * sq6 - double(7) / 50, 2) +
           pow(delta9b, 2));
  double delta5b = sqrt(pow(delta10b - delta11b + p_4p2_6_y * sq5 * sq6 + double(41) / 1000, 2) +
                        pow(delta7b, 2) + pow(delta8b, 2));
  double delta3b = 1;
  double delta2b = 1;
  double delta1b =
      (25 * cqm5 * sqm5 - 9 * cqm5 * sq6 - 195 * sqm5 - 25 * sqm5 * cqm5 + 41 * sqm5 * sq5 +
       9 * sqm5 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm5 * cq6 + 1000 * p_4p2_6_y * sqm5 * cq5 * sq6);

  double f2a = sq6 * delta2b * (25 * sqm5 - 9 * sq6 + 1000 * p_4p2_6_y * cq6) / delta1b -
               cq6 * delta2b *
                   (9 * cq6 - 195 * cq5 + 41 * sq5 - 25 * cq5 * cqm5 + 1000 * p_4p2_6_y * sq6) /
                   delta1b;
  double f2b =
      -sq6 * delta3b * delta9b * delta5b / ((sqm6 * delta7b + cqm6 * delta8b) * delta6b) +
      cq6 * delta3b * delta5b *
          (9 * cq6 - 140 * cq5 + 41 * sq5 - 25 * cq5 * cqm6 + 1000 * p_4p4_6_y * sq6) /
          (delta6b * (25 * cqm6 * sqm5 - 9 * cqm6 * sq6 - 195 * sqm6 - 25 * sqm6 * cqm5 +
                      41 * sqm6 * sq5 + 9 * sqm6 * cq5 * cq6 + 1000 * p_4p2_6_y * cqm6 * cq6 +
                      1000 * p_4p2_6_y * sqm6 * cq5 * sq6));

  double taum5Des = (f2b * tau5Des - f1b * tau6Des) / (f2b * f1a - f1b * f2a);
  double taum6Des = (f2a * tau5Des - f1a * tau6Des) / (f2a * f1b - f1a * f2b);

  // vel
  double qd5Des = joint_pitch_.handle->cmd.velocity;
  double qd6Des = joint_roll_.handle->cmd.velocity;

  double det11 = f2b / (f2b * f1a - f1b * f2a);
  double det12 = -f1b / (f2b * f1a - f1b * f2a);
  double det21 = f2a / (f2a * f1b - f1a * f2b);
  double det22 = -f1a / (f2a * f1b - f1a * f2b);

  Eigen::Matrix<double, 2, 2> J_T;
  J_T << det11, det12, det21, det22;
  Eigen::Matrix<double, 2, 2> J;
  J = J_T.transpose();

  Eigen::Matrix<double, 2, 1> qd56Des;
  qd56Des << qd5Des, qd6Des;
  // double determinant = J_T.determinant();

  // // 检查行列式是否为零
  // if (determinant == 0) {
  //     std::cerr << "Error: Matrix is singular and cannot be inverted." << std::endl;
  //     return -1;
  // }
  Eigen::Matrix<double, 2, 1> qdm56Des;
  qdm56Des = J.inverse() * qd56Des;

  double qdm5Des = qdm56Des[0];
  double qdm6Des = qdm56Des[1];

  actr_left_.handle->cmd.position = qm5Des * actr_left_.direction;
  actr_left_.handle->cmd.velocity = qdm5Des * actr_left_.direction;
  actr_left_.handle->cmd.effort = taum5Des * actr_left_.direction;
  actr_left_.handle->cmd.kp = joint_pitch_.handle->cmd.kp;
  actr_left_.handle->cmd.kd = joint_pitch_.handle->cmd.kd;

  actr_right_.handle->cmd.position = qm6Des * actr_right_.direction;
  actr_right_.handle->cmd.velocity = qdm6Des * actr_right_.direction;
  actr_right_.handle->cmd.effort = taum6Des * actr_right_.direction;
  actr_right_.handle->cmd.kp = joint_roll_.handle->cmd.kp;
  actr_right_.handle->cmd.kd = joint_roll_.handle->cmd.kd;
}

}  // namespace xyber_x1_infer::dcu_driver_module
