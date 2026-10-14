/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:25
 */

#pragma once

// cpp
#include <unordered_map>
#include <vector>

// projects
#include "common_type.h"
#include "actuator_diagnostics.h"
#include "internal/actuator_base.h"
#include "internal/ethercat_node.h"

namespace xyber {

#pragma pack(1)

struct DcuSendPacket {
  struct {
    uint8_t ctrl;
    uint8_t data[64];
  } canfd[3];
  uint8_t imu_cmd;
  uint8_t reserved[44];
};

struct DcuRecvPacket {
  uint8_t canfd[3][64];
  struct {
    uint32_t acc[3];
    uint32_t gyro[3];
    uint32_t quat[4];
  } imu;
  uint8_t reserved[8];
};

#pragma pack()

class Dcu : public ethercat_manager::EthercatNode {
 public:
  explicit Dcu(std::string name, int32_t ecat_id);
  virtual ~Dcu();

  bool RegisterActuator(Actuator* actr);
  Actuator* GetActautor(const std::string& name);
  virtual std::string GetName() override { return name_; }
  void SetChannelId(CtrlChannel ch, uint8_t id);

  DcuImu GetImuData();
  void ImuAppleUserOffset();

 public:  // Actuator Stuff
  bool EnableAllActuator();
  bool EnableActuator(const std::string& name);
  bool DisableAllActuator();
  bool DisableActuator(const std::string& name);
  void ClearError(const std::string& name);
  void SetHomingPosition(const std::string& name);
  void SaveConfig(const std::string& name);
  float GetTempure(const std::string& name);
  std::string GetErrorString(const std::string& name);
  ActautorState GetPowerState(const std::string& name);

  bool SetMode(const std::string& name, ActautorMode mode);
  ActautorMode GetMode(const std::string& name);

  void SetEffort(const std::string& name, float cur);
  float GetEffort(const std::string& name);
  void SetVelocity(const std::string& name, float vel);
  float GetVelocity(const std::string& name);
  void SetPosition(const std::string& name, float pos);
  float GetPosition(const std::string& name);

  void SetMitParam(const std::string& name, MitParam param);
  void SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp, float kd);
  bool GetActuatorState(const std::string& name, ActuatorStateSnapshot& state);
  bool GetActuatorDiagnostics(const std::string& name, ActuatorDiagnostics& data);
  void PushRecvData(const uint8_t* buf, uint16_t len) override;
  void PullSendData(uint8_t* buf, uint16_t len) override;

 private:
  virtual bool Init() override;
  virtual void OnStateChangeHook(ethercat_manager::NodeState state) override;
  virtual uint8_t* GetSendBuf() override { return (uint8_t*)&send_buf_; }
  virtual uint8_t* GetRecvBuf() override { return (uint8_t*)&recv_buf_; }

 private:
  std::string name_;
  std::unordered_map<std::string, Actuator*> actuator_map_;
  std::unordered_map<CtrlChannel, std::vector<Actuator*>> ctrl_channel_map_;

  DcuSendPacket send_buf_;
  DcuRecvPacket recv_buf_;
  DcuSendPacket last_sent_{};
  struct SlotTracker {
    bool seen = false;
    uint64_t content_seq = 0, measurement_seq = 0;
    int64_t content_ns = 0, measurement_ns = 0;
  } rx_tracker_[3][8];  // protected by recv_mtx_
  uint64_t rx_pdo_seq_ = 0, tx_pdo_seq_ = 0;
  int64_t rx_pdo_ns_ = 0, tx_pdo_ns_ = 0;
  uint64_t command_seq_[3][8]{}, last_command_seq_[3][8]{};
  uint8_t command_format_[3][8]{}, last_command_format_[3][8]{};
  void MarkCommand(Actuator* actr, uint8_t format = 0);  // caller owns send_mtx_
};

}  // namespace xyber
