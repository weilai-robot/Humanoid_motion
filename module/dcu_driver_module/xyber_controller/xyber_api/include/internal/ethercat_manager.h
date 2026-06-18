/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:32
 */

#pragma once

// cpp
#include <map>
#include <thread>
#include <unordered_map>
#include <vector>

// project
#include "internal/ethercat_node.h"

#define IO_MAP_SIZE 4096

namespace ethercat_manager {

struct EthercatConfig {
  std::string ifname = "eth0";
  bool enable_dc = true;
  int bind_cpu = -1;                 // -1: disable
  int rt_priority = -1;              // -1: disable, 0: max
  uint64_t cycle_time_ns = 1000000;  // 1ms - 1000hz
};

class EthercatManager {
 public:
  explicit EthercatManager();
  virtual ~EthercatManager();

  void RegisterNode(EthercatNode *node);
  bool Start(EthercatConfig cfg);
  void Stop();

 private:
  bool InitMaster();
  bool InitSlaveNodes();
  void UpdateSlaveState(int32_t state, int32_t id = 0);  // 0 for all

  void WorkLoop();
  void ErrorHandler();
  void DoubleToFixed(double f_input, int32_t *pValue, int32_t *pBase);
  int64_t CalcDcPiSync(int64_t refTime, int64_t cycle_time, int64_t shift_time);

 private:
  uint8_t io_map_[IO_MAP_SIZE] = {0};
  int current_wkc_ = 0;
  int expected_wkc_ = 0;
  bool wkc_error_ = false;
  int wkc_error_count_ = 0;

  EthercatConfig cfg_;

  std::mutex work_loop_mutex_;
  std::thread work_loop_thread_;
  std::thread error_handler_thread_;
  std::atomic_bool is_running_{false};
  std::unordered_map<int32_t, EthercatNode *> nodes_map_;

};  // class EthercatManager

}  // namespace ethercat_manager