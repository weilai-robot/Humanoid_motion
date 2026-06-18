/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:09
 */

#pragma once

// CPP
#include <cmath>
#include <cstdint>
#include <string>

#define M_2PI (2.0f * M_PI)

namespace xyber {

enum class CtrlChannel : uint8_t {
  CTRL_CH1 = 0,
  CTRL_CH2 = 1,
  CTRL_CH3 = 2,
};

enum class ActuatorType {
  POWER_FLOW_R86,
  POWER_FLOW_R52,
  POWER_FLOW_L28,
  OMNI_PICKER,
  UNKOWN,
};

static ActuatorType StringToType(std::string type) {
  if (type == "POWER_FLOW_R86") return ActuatorType::POWER_FLOW_R86;
  if (type == "POWER_FLOW_R52") return ActuatorType::POWER_FLOW_R52;
  if (type == "POWER_FLOW_L28") return ActuatorType::POWER_FLOW_L28;
  if (type == "OMNI_PICKER") return ActuatorType::OMNI_PICKER;
  return ActuatorType::UNKOWN;
}

enum ActautorState : uint8_t {
  STATE_DISABLE = 0,
  STATE_ENABLE = 1,
  STATE_CALIBRATION = 2,
};

enum ActautorMode : uint8_t {
  MODE_CURRENT = 0,
  MODE_CURRENT_RAMP = 1,
  MODE_VELOCITY = 2,
  MODE_VELOCITY_RAMP = 3,
  MODE_POSITION = 4,
  MODE_POSITION_RAMP = 5,
  MODE_MIT = 6,
};

enum class DcuImuCmd : uint8_t {
  NONE = 0,
  IMU_APPLY_OFFSET = 1,
};

struct DcuImu {
  float acc[3];
  float gyro[3];
  float quat[4];
};

/**
 * @description: Mit参数
 */
struct MitParam {
  float pos_min = 0.0;
  float pos_max = 0.0;
  float vel_min = 0.0;
  float vel_max = 0.0;
  float toq_min = 0.0;
  float toq_max = 0.0;
  float kp_min = 0.0;
  float kp_max = 0.0;
  float kd_min = 0.0;
  float kd_max = 0.0;
};

#define PF_R52_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -1.0f * M_2PI,       \
      .pos_max = 1.0f * M_2PI,        \
      .vel_min = -2.0f * M_2PI,       \
      .vel_max = 2.0f * M_2PI,        \
      .toq_min = -50.0f,              \
      .toq_max = 50.0f,               \
      .kp_min = 0.0f,                 \
      .kp_max = 500.0f,               \
      .kd_min = 0.0f,                 \
      .kd_max = 8.0f,                 \
  }

#define PF_R86_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -1.0f * M_2PI,       \
      .pos_max = 1.0f * M_2PI,        \
      .vel_min = -2.0f * M_2PI,       \
      .vel_max = 2.0f * M_2PI,        \
      .toq_min = -100.0f,             \
      .toq_max = 100.0f,              \
      .kp_min = 0.0f,                 \
      .kp_max = 500.0f,               \
      .kd_min = 0.0f,                 \
      .kd_max = 8.0f,                 \
  }

}  // namespace xyber