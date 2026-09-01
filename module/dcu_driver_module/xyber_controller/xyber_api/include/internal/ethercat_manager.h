/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:32
 */

#pragma once

// cpp
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
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
  void WriteStatsRow();
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

  // ── R3/R4 周期统计：WorkLoop(RT,1kHz) relaxed 写，ErrorHandler(非RT) 每10s 读出清零 ──
  // R3: 每周期 DC 偏差 dev = (ec_DCtime 增量) - cycle_time_ns（瞬时 DC 同步误差）
  std::atomic<int64_t> st_dc_dev_sum_ns_{0};       // 有符号和（可看平均漂移方向）
  std::atomic<int64_t> st_dc_dev_abs_min_ns_{INT64_MIN};  // 哨兵: INT64_MIN=无样本
  std::atomic<int64_t> st_dc_dev_abs_max_ns_{0};
  std::atomic<uint64_t> st_dc_cnt_{0};
  // R4: 每周期调度迟到 late = 实际唤醒 - sleep_until 目标时刻；late > cycle/5 记 overrun(=1.2×周期)
  std::atomic<int64_t> st_cycle_late_sum_ns_{0};
  std::atomic<int64_t> st_cycle_late_max_ns_{0};
  std::atomic<uint64_t> st_cycle_cnt_{0};
  std::atomic<uint64_t> st_cycle_overruns_{0};

  // 统计文件（ErrorHandler 线程独占；落点随进程工作目录，与 walk_diag 同约定）
  std::ofstream st_file_;
  uint64_t st_wkc_bad_polls_ = 0;  // 仅 ErrorHandler 线程访问（100ms 轮询中 wkc 异常次数）

};  // class EthercatManager

}  // namespace ethercat_manager