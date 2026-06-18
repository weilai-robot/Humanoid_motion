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

  void RegisterActuator(Actuator* actr);
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
};

}  // namespace xyber