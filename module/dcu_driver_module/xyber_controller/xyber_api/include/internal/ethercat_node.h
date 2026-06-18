/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:46
 */

#pragma once

// cpp
#include <cstdint>
#include <memory>
#include <mutex>

// projects
#include "internal/common_utils.h"

namespace ethercat_manager {

using PreOpConfigFunc = int (*)(uint16_t /*id*/);

enum class NodeState : int32_t {  // same as SOEM
  NONE = 0x00,                    // means lost
  INIT = 0x01,
  PRE_OP = 0x02,
  BOOT = 0x03,
  SAFE_OP = 0x04,
  OP = 0x08,
  ACK = 0x10,
  ERROR = 0x10,
};

class EthercatNode {
 public:
  explicit EthercatNode(int32_t id) : id_(id) {}
  virtual ~EthercatNode() {}

  virtual bool Init() { return true; }
  virtual void OnDataSyncHook() {}
  virtual void OnStateChangeHook(NodeState next) {}

  virtual uint8_t* GetSendBuf() = 0;
  virtual uint8_t* GetRecvBuf() = 0;
  virtual PreOpConfigFunc GetPreOpConfigFunc() { return nullptr; }

  int32_t GetId() { return id_; }
  NodeState GetNodeState() { return node_state_; }
  virtual std::string GetName() { return node_name_; }

  void SetState(NodeState state) { node_state_ = state; }
  void SetIoSize(uint16_t input, uint16_t output) {
    recv_size_ = input;
    send_size_ = output;
  }
  void SetNodeName(const std::string& name) { node_name_ = name; }

  virtual void PullSendData(uint8_t* buf, uint16_t len) {
    std::lock_guard<std::mutex> lock(send_mtx_);
    // if (!send_mtx_.try_lock()) return;
    uint8_t* ptr = GetSendBuf();
    std::copy(ptr, ptr + len, buf);
    // send_mtx_.unlock();
  }

  virtual void PushRecvData(const uint8_t* buf, uint16_t len) {
    std::lock_guard<std::mutex> lock(recv_mtx_);
    // if (!recv_mtx_.try_lock()) return;
    std::copy(buf, buf + len, GetRecvBuf());
    // recv_mtx_.unlock();
  }

  static std::string NodeStateStr(NodeState state) {
    switch (state) {
      case NodeState::NONE:
        return "NONE";
      case NodeState::INIT:
        return "INIT";
      case NodeState::PRE_OP:
        return "PRE_OP";
      case NodeState::BOOT:
        return "BOOT";
      case NodeState::SAFE_OP:
        return "SAFE_OP";
      case NodeState::OP:
        return "OP";
      case NodeState::ACK:
        return "ACK_OR_ERROR";
      default:
        return "UNKNOWN";
    }
    return "UNKNOWN";
  }

 protected:
  const int32_t id_;
  std::string node_name_;
  std::mutex send_mtx_, recv_mtx_;

  uint16_t send_size_ = 0;
  uint16_t recv_size_ = 0;

  NodeState node_state_ = NodeState::NONE;
};

}  // namespace ethercat_manager