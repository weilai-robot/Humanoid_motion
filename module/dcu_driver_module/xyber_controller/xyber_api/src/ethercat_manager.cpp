/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:40
 */

#include "internal/ethercat_manager.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <limits>

#include "ethercat.h"

#define WKC_ERROR_CNT 10
#define EC_TIMEOUTMON 500

namespace {
// 统计文件名时间戳（wall 时钟）
std::string StatsTimestamp() {
  const std::time_t t = std::time(nullptr);
  std::tm tm_val{};
  localtime_r(&t, &tm_val);
  char buf[24];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_val);
  return std::string(buf);
}
}  // namespace

using namespace std::chrono_literals;

namespace ethercat_manager {

EthercatManager::EthercatManager() { memset(io_map_, 0, IO_MAP_SIZE); }

EthercatManager::~EthercatManager() { Stop(); }

void EthercatManager::RegisterNode(EthercatNode* node) { nodes_map_[node->GetId()] = node; }

bool EthercatManager::Start(EthercatConfig cfg) {
  if (is_running_) {
    LOG_WARN("EtherCAT is running already.");
    return true;
  }

  cfg_ = cfg;
  LOG_DEBUG("EthercatManager ifname: %s, cycle_time: %lu, enable_dc: %d", cfg_.ifname.c_str(),
            cfg_.cycle_time_ns, cfg_.enable_dc);

  // check if there are any slave nodes registered
  if (nodes_map_.empty()) {
    LOG_ERROR("No slave nodes registered.");
    return false;
  }
  // init master, after this operation, the ec_slave[] will be valid
  bool ret = InitMaster();
  if (!ret) {
    // LOG_ERROR("InitMaster failed");
    return false;
  }

  // Start the cyclic thread immediately to avoid the actuator watchdog
  // expiring. Real-time setup result is reported by WorkLoop itself.
  is_running_ = true;
  work_loop_thread_ = std::thread(&EthercatManager::WorkLoop, this);
  error_handler_thread_ = std::thread(&EthercatManager::ErrorHandler, this);

  LOG_DEBUG("EthercatManager is running now.");
  return true;
}

void EthercatManager::Stop() {
  if (!is_running_) return;

  is_running_ = false;
  if (work_loop_thread_.joinable()) {
    work_loop_thread_.join();
  }
  if (error_handler_thread_.joinable()) {
    error_handler_thread_.join();
  }

  /* request INIT state for all slaves */
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_INIT, 50000);

  LOG_DEBUG("EthercatManager is stopped.");
}

bool EthercatManager::InitMaster() {
  /* initialise SOEM, bind socket to ifname */
  if (!ec_init(cfg_.ifname.c_str())) {
    LOG_ERROR("No socket connection on %s, Execute as root.", cfg_.ifname.c_str());
    return false;
  }
  LOG_INFO("EtherCAT init on %s succeeded.", cfg_.ifname.c_str());

  /* find and auto-config slaves, transfer all slaves to PRE_OP */
  for (size_t i = 0; i < 3; i++) {
    if (ec_config_init(FALSE) > 0) break;
    std::this_thread::sleep_for(100ms);
  }
  if (ec_slavecount <= 0) {
    LOG_ERROR("No slaves found!");
    ec_close();
    return false;
  } else {
    LOG_INFO("%d slaves found and configured", ec_slavecount);
  }

  // check registered slave and register pdo mapping
  for (const auto& [id, node] : nodes_map_) {
    if (id > ec_slavecount) {
      LOG_ERROR("Can`t find Registered slave node %s (%d).", node->GetName().c_str(), id);
      return false;
    }
    ec_slave[id].PO2SOconfig = node->GetPreOpConfigFunc();
  }
  // check statr transfer
  LOG_DEBUG("All %lu registered slaves matched, Enter Pre-OP state", nodes_map_.size());
  for (const auto& [id, node] : nodes_map_) {
    if (ec_statecheck(id, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_PRE_OP) {
      LOG_ERROR("Slave %s (%d) set EC_STATE_PRE_OP failed.", node->GetName().c_str(), id);
      return false;
    }
  }
  std::this_thread::sleep_for(100ms);
  UpdateSlaveState(EC_STATE_PRE_OP);

  // PDO Configuration is invoked here
  int32_t iosize = ec_config_map(io_map_);
  if (iosize > IO_MAP_SIZE) {
    LOG_ERROR("PDO data buf too small ! current %d need %d", IO_MAP_SIZE, iosize);
    return false;
  }

  // init slave nodes
  if (!InitSlaveNodes()) {
    LOG_ERROR("InitSlaveNodes failed");
    return false;
  }

  // find out the unregistered slave and calculate wkc
  std::vector<int32_t> unregistered_slave;
  for (int32_t i = 1; i <= ec_slavecount; i++) {
    if (nodes_map_.find(i) != nodes_map_.end()) continue;
    LOG_INFO("Slaves %d unregistered, excluded.", i);
    unregistered_slave.push_back(i);
  }
  expected_wkc_ =
      ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC - unregistered_slave.size() * 2;
  LOG_INFO("Slaves mapped, Process Data IO size: %d, expected_wkc %d", iosize, expected_wkc_);

  // enable dc clock
  if (cfg_.enable_dc) {
    ec_configdc();
    for (int id = 1; id <= ec_slavecount; id++) ec_dcsync0(id, TRUE, cfg_.cycle_time_ns, 0);
    LOG_INFO("EtherCAT DC clock enabled.");
  } else {
    LOG_INFO("EtherCAT DC clock disabled.");
  }

  /* wait for all slaves to reach SAFE_OP state */
  LOG_DEBUG("transfer all slaves to SAFE_OP.");
  for (const auto& [id, node] : nodes_map_) {
    if (ec_statecheck(id, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_SAFE_OP) {
      LOG_ERROR("Slave %s (%d) set EC_STATE_SAFE_OP failed.", node->GetName().c_str(), id);
      return false;
    }
  }
  std::this_thread::sleep_for(100ms);
  UpdateSlaveState(EC_STATE_SAFE_OP);

  // all config done, ready to enter OP mode
  LOG_DEBUG("Request operational state for all slaves");

  /* send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  /* request OP state for all slaves */
  for (const auto& [id, node] : nodes_map_) {
    ec_slave[id].state = EC_STATE_OPERATIONAL;
    ec_writestate(id);
  }

  /* wait for all slaves to reach OP state */
  bool op_state = true;
  int32_t chk = 200;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    op_state = true;
    for (const auto& [id, node] : nodes_map_) {
      if (ec_statecheck(id, EC_STATE_OPERATIONAL, 50000) != EC_STATE_OPERATIONAL) {
        op_state = false;
      }
    }
  } while (chk-- && !op_state && is_running_);

  if (!op_state) {
    LOG_ERROR("Not all slaves reached operational state.");
    ec_readstate();
    for (const auto& [id, node] : nodes_map_) {
      if (ec_slave[id].state != EC_STATE_OPERATIONAL) {
        LOG_ERROR("Slave %s (%d) State=0x%2.2x StatusCode=0x%4.4x : %s", node->GetName().c_str(),
                  id, ec_slave[id].state, ec_slave[id].ALstatuscode,
                  ec_ALstatuscode2string(ec_slave[id].ALstatuscode));
      }
    }
    return false;
  }
  //
  LOG_INFO("Operational state reached for all slaves.");
  UpdateSlaveState(EC_STATE_OPERATIONAL);
  return true;
}

bool EthercatManager::InitSlaveNodes() {
  // init each slave node
  for (auto& node : nodes_map_) {
    node.second->SetNodeName(ec_slave[node.first].name);
    node.second->SetIoSize(ec_slave[node.first].Ibytes, ec_slave[node.first].Obytes);
    if (!node.second->Init()) return false;
  }
  return true;
}

void EthercatManager::UpdateSlaveState(int32_t state, int32_t id) {
  if (id) {
    auto node = nodes_map_.find(id);
    if (state == (int32_t)node->second->GetNodeState()) return;
    node->second->OnStateChangeHook((NodeState)state);
    node->second->SetState((NodeState)state);
  } else {
    for (const auto& [id, node] : nodes_map_) {
      if (state == (int32_t)node->GetNodeState()) continue;
      node->OnStateChangeHook((NodeState)state);
      node->SetState((NodeState)state);
    }
  }
}

void EthercatManager::DoubleToFixed(double f_input, int32_t* pValue, int32_t* pBase) {
  if (f_input < 1.0) {
    (*pBase) = 15;
    (*pValue) = (int32_t)(32768.0 * f_input);
  } else if (f_input < 2.0) {
    (*pBase) = 14;
    (*pValue) = (int32_t)(16384.0 * f_input);
  } else if (f_input < 4.0) {
    (*pBase) = 13;
    (*pValue) = (int32_t)(8192.0 * f_input);
  } else if (f_input < 8.0) {
    (*pBase) = 12;
    (*pValue) = (int32_t)(4096.0 * f_input);
  } else if (f_input < 16.0) {
    (*pBase) = 11;
    (*pValue) = (int32_t)(2048.0 * f_input);
  } else if (f_input < 32.0) {
    (*pBase) = 10;
    (*pValue) = (int32_t)(1024.0 * f_input);
  } else if (f_input < 64.0) {
    (*pBase) = 9;
    (*pValue) = (int32_t)(512.0 * f_input);
  } else if (f_input < 128.0) {
    (*pBase) = 8;
    (*pValue) = (int32_t)(256.0 * f_input);
  } else if (f_input < 256.0) {
    (*pBase) = 7;
    (*pValue) = (int32_t)(128.0 * f_input);
  } else if (f_input < 512.0) {
    (*pBase) = 6;
    (*pValue) = (int32_t)(64.0 * f_input);
  } else if (f_input < 1024.0) {
    (*pBase) = 5;
    (*pValue) = (int32_t)(32.0 * f_input);
  } else if (f_input < 2048.0) {
    (*pBase) = 4;
    (*pValue) = (int32_t)(16.0 * f_input);
  } else if (f_input < 4096.0) {
    (*pBase) = 3;
    (*pValue) = (int32_t)(8.0 * f_input);
  } else if (f_input < 81928.0) {
    (*pBase) = 2;
    (*pValue) = (int32_t)(4.0 * f_input);
  } else if (f_input < 16384.0) {
    (*pBase) = 1;
    (*pValue) = (int32_t)(2.0 * f_input);
  } else if (f_input < 32768.0) {
    (*pBase) = 0;
    (*pValue) = (int32_t)(1.0 * f_input);
  }
}

int64_t EthercatManager::CalcDcPiSync(int64_t refTime, int64_t cycle_time, int64_t shift_time) {
  static double kP = 0.05, kI = 0.01;
  static int64_t iTerm = 0;
  int64_t adjTime;
  int32_t iKp = 0, iKpBase, iKi = 0, iKiBase;
  DoubleToFixed(kP, &iKp, &iKpBase);
  DoubleToFixed(kI, &iKi, &iKiBase);

  int64_t error = (refTime - shift_time) % cycle_time;
  if (error > (cycle_time / 2)) error = error - cycle_time;

  int64_t pTerm = error * iKp;
  iTerm += (error * iKi);

  adjTime = -(pTerm >> iKpBase) - (iTerm >> iKiBase);

  // if (adjTime > cycle_time / 2) adjTime = cycle_time / 2;
  // if (adjTime < -cycle_time / 2) adjTime = -cycle_time / 2;

  return adjTime;
}

void EthercatManager::WorkLoop() {
  LOG_DEBUG("Workloop is running now...");
  if (!xyber_utils::SetRealTimeThread(pthread_self(), "ecat_io_loop", cfg_.rt_priority, cfg_.bind_cpu)) {
    LOG_ERROR("ecat_io_loop real-time setup failed, running without SCHED_FIFO/CPU affinity");
  }

  // For DC computation
  int32_t shift_time = cfg_.cycle_time_ns / 2 - 20000;  // 20us for SM-IRQ
  int64_t dc_ref_time = 0;
  int64_t last_dc_time = 0;
  int64_t timer_offset = 0;
  auto now_time = std::chrono::steady_clock::now();

  if (cfg_.enable_dc) {
    // Update DC time for first time
    ec_send_overlap_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // Align to DC clock edge
    now_time +=
        std::chrono::nanoseconds(10 * cfg_.cycle_time_ns - (ec_DCtime % cfg_.cycle_time_ns));
    std::this_thread::sleep_until(now_time);
  }

  /* cyclic loop */
  while (is_running_) {
    // update slave state data here
    current_wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
    for (const auto& [id, node] : nodes_map_) {
      node->PushRecvData(ec_slave[id].inputs, ec_slave[id].Ibytes);
      node->OnDataSyncHook();
    }

    // update slave cmd data here
    for (const auto& [id, node] : nodes_map_) {
      node->PullSendData(ec_slave[id].outputs, ec_slave[id].Obytes);
    }
    ec_send_processdata();

    // loop time stuff
    if (cfg_.enable_dc) {
      now_time += std::chrono::nanoseconds(cfg_.cycle_time_ns + timer_offset);
      std::this_thread::sleep_until(now_time);

      const int64_t dc_elapsed = ec_DCtime - last_dc_time;
      dc_ref_time += dc_elapsed;
      last_dc_time = ec_DCtime;
      timer_offset = CalcDcPiSync(dc_ref_time, cfg_.cycle_time_ns, shift_time);

      // R3: 瞬时 DC 偏差（本次 DC 时钟增量与名义周期的差）
      const int64_t dc_dev = dc_elapsed - static_cast<int64_t>(cfg_.cycle_time_ns);
      const int64_t dc_abs = dc_dev < 0 ? -dc_dev : dc_dev;
      st_dc_dev_sum_ns_.fetch_add(dc_dev, std::memory_order_relaxed);
      int64_t cur_max = st_dc_dev_abs_max_ns_.load(std::memory_order_relaxed);
      if (dc_abs > cur_max) st_dc_dev_abs_max_ns_.store(dc_abs, std::memory_order_relaxed);
      int64_t cur_min = st_dc_dev_abs_min_ns_.load(std::memory_order_relaxed);
      if (dc_abs < cur_min) st_dc_dev_abs_min_ns_.store(dc_abs, std::memory_order_relaxed);
      st_dc_cnt_.fetch_add(1, std::memory_order_relaxed);
    } else {
      now_time += std::chrono::nanoseconds(cfg_.cycle_time_ns);
      std::this_thread::sleep_until(now_time);
    }

    // R4: 本周期调度迟到（唤醒时刻 - 目标时刻；迟到超过周期 20% 记一次 overrun）
    const int64_t late_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now() - now_time).count();
    if (late_ns > 0) {
      st_cycle_late_sum_ns_.fetch_add(late_ns, std::memory_order_relaxed);
      int64_t l_max = st_cycle_late_max_ns_.load(std::memory_order_relaxed);
      if (late_ns > l_max) st_cycle_late_max_ns_.store(late_ns, std::memory_order_relaxed);
      if (late_ns > static_cast<int64_t>(cfg_.cycle_time_ns) / 5)
        st_cycle_overruns_.fetch_add(1, std::memory_order_relaxed);
    }
    st_cycle_cnt_.fetch_add(1, std::memory_order_relaxed);
  }
}

void EthercatManager::ErrorHandler() {
  LOG_DEBUG("ErrorHandler is running now...");
  pthread_setname_np(pthread_self(), "ecat_er_loop");

  std::this_thread::sleep_for(std::chrono::nanoseconds(cfg_.cycle_time_ns * 20));

  // R3/R4 统计文件（数据只落文件，不打印终端；失败则静默关闭统计，不影响总线）
  std::error_code fs_ec;
  std::filesystem::create_directories("test_logs/ecat_stats", fs_ec);
  const std::string stats_path = "test_logs/ecat_stats/ecat_stats_" + StatsTimestamp() + ".csv";
  st_file_.open(stats_path, std::ios::out | std::ios::app);
  if (st_file_.is_open()) {
    st_file_ << "wall_ts,steady_ns,dc_cnt,dc_dev_avg_ns,dc_dev_abs_min_ns,dc_dev_abs_max_ns,"
                "cycle_cnt,cycle_late_avg_ns,cycle_late_max_ns,cycle_overruns,wkc_bad_polls\n";
  }
  uint64_t poll_cnt = 0;

  while (is_running_) {
    std::this_thread::sleep_for(100ms);

    // 每 100 次 × 100ms = 10s 落一行统计
    if (st_file_.is_open() && ++poll_cnt >= 100) {
      poll_cnt = 0;
      WriteStatsRow();
    }

    // check error resume
    if (wkc_error_) {
      if (current_wkc_ != expected_wkc_) {
        continue;
      }
      LOG_WARN("Ecat link resmed.");
      wkc_error_ = false;
      wkc_error_count_ = 0;
    }
    // check wkc
    if (current_wkc_ < expected_wkc_ || ec_group[0].docheckstate) {
      ++st_wkc_bad_polls_;
      LOG_WARN("wkc < expected_wkc , wkc: %d, expected_wkc: %d", current_wkc_, expected_wkc_);
      ec_group[0].docheckstate = FALSE;
      ec_readstate();
      for (const auto& [id, node] : nodes_map_) {
        if (!is_running_) break;
        if ((ec_slave[id].group == 0) && (ec_slave[id].state != EC_STATE_OPERATIONAL)) {
          ec_group[0].docheckstate = TRUE;
          if (ec_slave[id].state == EC_STATE_SAFE_OP + EC_STATE_ERROR) {
            LOG_ERROR("ERROR : node %s (%d) is in SAFE_OP + ERROR, attempting ack.",
                      node->GetName().c_str(), id);
            ec_slave[id].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ec_writestate(id);
          } else if (ec_slave[id].state == EC_STATE_SAFE_OP) {
            LOG_ERROR("ERROR : node %s (%d) is in SAFE_OP, change to OPERATIONAL.",
                      node->GetName().c_str(), id);
            ec_slave[id].state = EC_STATE_OPERATIONAL;
            ec_writestate(id);
          } else if (ec_slave[id].state > EC_STATE_NONE) {
            LOG_WARN("MESSAGE : node %s (%d)  reconfigured. current state: 0x%04X",
                     node->GetName().c_str(), id, ec_slave[id].state);
            if (ec_reconfig_slave(id, EC_TIMEOUTMON)) {
              ec_slave[id].islost = FALSE;
              LOG_WARN(
                  "MESSAGE : node %s (%d) reconfigured done. current state: "
                  "0x%04X",
                  node->GetName().c_str(), id, ec_slave[id].state);
            }
          } else if (!ec_slave[id].islost) {
            LOG_WARN("WARN : node %s (%d) re-check state. current state: 0x%04X",
                     node->GetName().c_str(), id, ec_slave[id].state);
            /* re-check state */
            ec_statecheck(id, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (!ec_slave[id].state) {
              ec_slave[id].islost = TRUE;
              LOG_ERROR("ERROR : node %s (%d) lost. current state: 0x%04X", node->GetName().c_str(),
                        id, ec_slave[id].state);
            }
          }
        }
        if (ec_slave[id].islost) {
          LOG_ERROR("WARN : node %s (%d) lost. current state: 0x%04X", node->GetName().c_str(), id,
                    ec_slave[id].state);
          if (!ec_slave[id].state) {
            if (ec_recover_slave(id, EC_TIMEOUTMON)) {
              ec_slave[id].islost = FALSE;
              LOG_WARN("MESSAGE : node %s (%d) recovered", node->GetName().c_str(), id);
            }
          } else {
            ec_slave[id].islost = FALSE;
            LOG_WARN("MESSAGE : node %s (%d) found. current state: 0x%04X", node->GetName().c_str(),
                     id, ec_slave[id].state);
          }
        }
        UpdateSlaveState(ec_slave[id].state, id);
      }

      if (!ec_group[0].docheckstate) {
        LOG_INFO("OK : all slaves resumed OPERATIONAL.");
      } else {
        // if (OnBusError) OnBusError();
      }

      // count error
      if (++wkc_error_count_ >= WKC_ERROR_CNT) {
        LOG_ERROR("Ecat link lost.");
        wkc_error_ = true;
      }
    } else {
      wkc_error_count_ = 0;
    }
  }

  if (st_file_.is_open()) {
    st_file_.flush();
    st_file_.close();
  }
}

void EthercatManager::WriteStatsRow() {
  // 读出并清零（relaxed：与 WorkLoop 的写入以窗口为单位对齐，边界样本最多差 1 周期，统计可接受）
  const int64_t dc_sum = st_dc_dev_sum_ns_.exchange(0, std::memory_order_relaxed);
  const int64_t dc_min = st_dc_dev_abs_min_ns_.exchange(INT64_MIN, std::memory_order_relaxed);
  const int64_t dc_max = st_dc_dev_abs_max_ns_.exchange(0, std::memory_order_relaxed);
  const uint64_t dc_cnt = st_dc_cnt_.exchange(0, std::memory_order_relaxed);
  const int64_t late_sum = st_cycle_late_sum_ns_.exchange(0, std::memory_order_relaxed);
  const int64_t late_max = st_cycle_late_max_ns_.exchange(0, std::memory_order_relaxed);
  const uint64_t cyc_cnt = st_cycle_cnt_.exchange(0, std::memory_order_relaxed);
  const uint64_t overruns = st_cycle_overruns_.exchange(0, std::memory_order_relaxed);
  const uint64_t wkc_bad = st_wkc_bad_polls_;
  st_wkc_bad_polls_ = 0;

  const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch()).count();

  st_file_ << StatsTimestamp() << ',' << steady_ns << ',' << dc_cnt << ',';
  if (dc_cnt > 0)
    st_file_ << (dc_sum / static_cast<int64_t>(dc_cnt)) << ',' << dc_min << ',' << dc_max << ',';
  else
    st_file_ << ",,,";
  st_file_ << cyc_cnt << ',';
  if (cyc_cnt > 0)
    st_file_ << (late_sum / static_cast<int64_t>(cyc_cnt)) << ',' << late_max << ',';
  else
    st_file_ << ",,";
  st_file_ << overruns << ',' << wkc_bad << '\n';
  st_file_.flush();
}

}  // namespace ethercat_manager
