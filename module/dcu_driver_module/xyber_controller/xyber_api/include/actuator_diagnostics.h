#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "common_type.h"

namespace xyber {

struct ActuatorStateSnapshot {
  float position = 0, velocity = 0, effort = 0;
  int64_t pdo_received_ns = 0;
};

// PDO arrival and payload changes are observable. The DCU protocol exposes no
// per-CAN receive timestamp/sequence; unchanged bytes do NOT prove a stale motor.
struct ActuatorDiagnostics {
  int ecat_id = 0, channel = 0, can_id = 0;
  ActuatorType type = ActuatorType::UNKOWN;
  uint64_t rx_pdo_seq = 0, tx_pdo_seq = 0, tx_command_seq = 0;
  int64_t rx_pdo_ns = 0, tx_pdo_ns = 0;
  uint64_t rx_content_seq = 0, rx_measurement_seq = 0;
  int64_t rx_content_ns = 0, rx_measurement_ns = 0;
  bool rx_seen = false;
  uint8_t tx_mask = 0;
  // 0=administrative/unknown, 1=R52/R86 MIT, 2=L28 position + current limit.
  uint8_t tx_format = 0;
  std::array<uint8_t, 8> rx{}, tx{};
  ActuatorStateSnapshot state;
  bool mit_param_valid = false;
  MitParam mit_param;
  float cmd_position = std::numeric_limits<float>::quiet_NaN();
  float cmd_velocity = std::numeric_limits<float>::quiet_NaN();
  float cmd_effort = std::numeric_limits<float>::quiet_NaN();
  float cmd_kp = std::numeric_limits<float>::quiet_NaN();
  float cmd_kd = std::numeric_limits<float>::quiet_NaN();
  float cmd_current_limit = std::numeric_limits<float>::quiet_NaN();
};

}  // namespace xyber
