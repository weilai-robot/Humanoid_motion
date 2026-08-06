// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#pragma once
#include <chrono>
#include <future>
#include <shared_mutex>

#include "aimrt_module_cpp_interface/module_base.h"
#include "joy_stick_module/joy.h"
#include "joy_stick_module/joy_vel_limiter.h"

#include "sensor_msgs/msg/imu.hpp"

namespace xyber_x1_infer::joy_stick_module {

struct FloatPub {
  std::string topic_name;
  std::vector<uint8_t> buttons;
  aimrt::channel::PublisherRef pub;
};

struct TwistPub {
  std::string topic_name;
  std::vector<uint8_t> buttons;
  std::map<std::string, uint8_t> axis;
  aimrt::channel::PublisherRef pub;
  aimrt::channel::PublisherRef pub_limiter;
  // 固定速度（若设置则忽略摇杆轴输入）
  bool use_constant_velocity = false;
  std::map<std::string, double> constant_velocity;
};

struct ServiceClient {
  std::string service_name;
  std::string interface_type;
  std::vector<uint8_t> buttons;
};

class JoyStickModule : public aimrt::ModuleBase {
 public:
  JoyStickModule() = default;
  ~JoyStickModule() override = default;

  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "JoyStickModule"};
  }
  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }
  void MainLoop();

 private:
  aimrt::CoreRef core_;

  std::shared_ptr<Joy> joy_;
  std::atomic_bool run_flag_ = true;
  std::promise<void> stop_sig_;

  aimrt::executor::ExecutorRef executor_;
  std::vector<FloatPub> float_pubs_;
  std::vector<TwistPub> twist_pubs_;
  std::vector<ServiceClient> srv_clients_;
  std::shared_ptr<JoyVelLimiter> limiter_ = nullptr;

  uint32_t freq_{};

  // 行走模式状态：按一次切换，激活后自动持续发送 /cmd_vel
  bool walk_mode_active_ = false;
  std::vector<bool> prev_walk_buttons_;  // 用于上升沿检测

  // LT 自动行走状态机（走内部通道，和手柄一样的发布路径）
  bool prev_lt_pressed_ = false;
  bool auto_walk_active_ = false;
  std::chrono::steady_clock::time_point auto_walk_t0_;

  // IMU 订阅（用于 RT 偏航闭环）
  std::shared_mutex imu_mutex_;
  sensor_msgs::msg::Imu latest_imu_;
  bool imu_received_ = false;

  // RT 直线行走状态机（IMU 偏航闭环）
  bool prev_rt_pressed_ = false;
  bool rt_walk_active_ = false;
  double rt_yaw_target_ = 0.0;
  double rt_yaw_integral_ = 0.0;       // 积分项累积

  // RT 行走参数（从配置读取）
  double rt_linear_x_ = 0.4;
  double rt_yaw_kp_ = 0.8;
  double rt_yaw_ki_ = 0.3;
  double rt_yaw_kd_ = 0.2;
  double rt_max_angular_z_ = 0.5;
  double rt_i_limit_ = 0.3;            // 积分项限幅 rad/s
};

}  // namespace xyber_x1_infer::joy_stick_module
