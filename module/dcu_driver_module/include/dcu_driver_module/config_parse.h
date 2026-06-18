// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

// projects
#include "util/log_util.h"
#include "yaml-cpp/yaml.h"

namespace YAML {

struct EthercatConfig {
  std::string ifname;
  bool enable_dc;
  int bind_cpu;
  int rt_priority;
  uint64_t cycle_time_ns;
};

template <>
struct convert<EthercatConfig> {
  static bool decode(const Node& node, EthercatConfig& rhs) {
    try {
      rhs.ifname = node["ifname"].as<std::string>();
      rhs.enable_dc = node["enable_dc"].as<bool>();
      rhs.bind_cpu = node["bind_cpu"].as<int>();
      rhs.rt_priority = node["rt_priority"].as<int>();
      rhs.cycle_time_ns = node["cycle_time_ns"].as<uint64_t>();
      return true;
    } catch (const YAML::Exception& e) {
      auto lgr = aimrt::common::util::SimpleLogger();
      AIMRT_HL_ERROR(lgr, "Parse EtherConfig failed, {}", e.what());
      return false;
    }
  }
};

struct DcuConfig {
  std::string name;
  bool enable;
  bool imu_enable;
  uint32_t ecat_id;

  struct Actuator {
    std::string name;
    std::string type;
    uint32_t can_id;
  };
  std::vector<Actuator> ch[3];
};
using DcuNetworkConfig = std::vector<DcuConfig>;

template <>
struct convert<DcuNetworkConfig> {
  static bool decode(const Node& node, DcuNetworkConfig& rhs) {
    try {
      for (auto& dcu_node : node) {
        DcuConfig dcu_cfg;
        dcu_cfg.name = dcu_node["name"].as<std::string>();
        dcu_cfg.ecat_id = dcu_node["ecat_id"].as<uint32_t>();
        dcu_cfg.enable = dcu_node["enable"].as<bool>();
        dcu_cfg.imu_enable = dcu_node["imu_enable"].as<bool>();

        std::vector<std::string> ch_names{"channel_1", "channel_2", "channel_3"};
        for (size_t i = 0; i < ch_names.size(); i++) {
          if (dcu_node[ch_names[i]].IsDefined()) {
            for (auto& actr_node : dcu_node[ch_names[i]]) {
              DcuConfig::Actuator actr_cfg;
              actr_cfg.name = actr_node["name"].as<std::string>();
              actr_cfg.type = actr_node["type"].as<std::string>();
              actr_cfg.can_id = actr_node["can_id"].as<uint32_t>();
              dcu_cfg.ch[i].push_back(actr_cfg);
            }
          }
        }
        rhs.push_back(dcu_cfg);
      }
      return true;
    } catch (const YAML::Exception& e) {
      auto lgr = aimrt::common::util::SimpleLogger();
      AIMRT_HL_ERROR(lgr, "Parse EtherConfig failed, {}", e.what());
      return false;
    }
  }
};

}  // namespace YAML