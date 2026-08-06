// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#include "joy_stick_module/joy_stick_module.h"

#include <yaml-cpp/yaml.h>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

// #include "Empty.aimrt_rpc.srv.h"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>

namespace xyber_x1_infer::joy_stick_module {

bool JoyStickModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;
  joy_ = std::make_shared<Joy>();

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path.data());
      freq_ = cfg_node["freq"].as<uint32_t>();

      // prepare executor
      executor_ = core_.GetExecutorManager().GetExecutor("joy_stick_pub_thread");
      AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor 'joy_stick_pub_thread'.");

      // 订阅 IMU 数据（用于 RT 偏航闭环）
      if (cfg_node["sub_imu_topic"]) {
        auto imu_topic = cfg_node["sub_imu_topic"].as<std::string>();
        auto sub = core_.GetChannelHandle().GetSubscriber(imu_topic);
        aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(sub,
          [this](const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
            std::unique_lock<std::shared_mutex> lock(imu_mutex_);
            latest_imu_ = *msg;
            imu_received_ = true;
          });
        AIMRT_INFO("Subscribed IMU topic: {}", imu_topic);
      }

      // 读取 RT 直线行走参数
      if (cfg_node["rt_auto_walk"] && cfg_node["rt_auto_walk"]["enabled"]) {
        auto rt = cfg_node["rt_auto_walk"];
        rt_linear_x_      = rt["linear_x"].as<double>();
        rt_yaw_kp_        = rt["yaw_kp"].as<double>();
        rt_yaw_ki_        = rt["yaw_ki"].as<double>();
        rt_yaw_kd_        = rt["yaw_kd"].as<double>();
        rt_max_angular_z_ = rt["max_angular_z"].as<double>();
        rt_i_limit_       = rt["i_limit"].as<double>();
        AIMRT_INFO("RT auto-walk enabled: vx={}, Kp={}, Ki={}, Kd={}, max_wz={}, i_limit={}",
                   rt_linear_x_, rt_yaw_kp_, rt_yaw_ki_, rt_yaw_kd_, rt_max_angular_z_, rt_i_limit_);
      }

      if (cfg_node["float_pubs"]) {
        for (const auto& pub : cfg_node["float_pubs"]) {
          FloatPub publisher;
          publisher.topic_name = pub["topic_name"].as<std::string>();
          publisher.buttons = pub["buttons"].as<std::vector<uint8_t>>();
          publisher.pub = core_.GetChannelHandle().GetPublisher(publisher.topic_name);
          aimrt::channel::RegisterPublishType<std_msgs::msg::Float32>(publisher.pub);
          float_pubs_.push_back(std::move(publisher));
        }
      }
      if (cfg_node["twist_pubs"]) {
        for (const auto& pub : cfg_node["twist_pubs"]) {
          TwistPub publisher;
          publisher.topic_name = pub["topic_name"].as<std::string>();
          publisher.buttons = pub["buttons"].as<std::vector<uint8_t>>();
          publisher.axis = pub["axis"].as<std::map<std::string, uint8_t>>();
          publisher.pub = core_.GetChannelHandle().GetPublisher(publisher.topic_name);
          aimrt::channel::RegisterPublishType<geometry_msgs::msg::Twist>(publisher.pub);
          if (pub["velocity_limit_lb"] && pub["velocity_limit_ub"]) {
            publisher.pub_limiter = core_.GetChannelHandle().GetPublisher(publisher.topic_name + "_limiter");
            aimrt::channel::RegisterPublishType<geometry_msgs::msg::Twist>(publisher.pub_limiter);

            auto lb = pub["velocity_limit_lb"].as<std::vector<double>>();
            auto ub = pub["velocity_limit_ub"].as<std::vector<double>>();
            array_t lb_array = Eigen::Map<array_t>(lb.data(), lb.size());
            array_t ub_array = Eigen::Map<array_t>(ub.data(), ub.size());
            limiter_ = std::make_shared<JoyVelLimiter>(pub["axis"].size(), 1.0 / freq_, lb_array, ub_array);
          }
          // 读取固定速度配置（可选）：若设置则忽略摇杆轴，使用固定值
          if (pub["constant_velocity"]) {
            publisher.use_constant_velocity = true;
            for (const auto& kv : pub["constant_velocity"]) {
              publisher.constant_velocity[kv.first.as<std::string>()] =
                  kv.second.as<double>();
            }
          }
          twist_pubs_.push_back(std::move(publisher));
        }
      }
      if (cfg_node["rpc_clients"]) {
        for (const auto& rpc : cfg_node["rpc_clients"]) {
          ServiceClient clent;
          clent.service_name = rpc["service_name"].as<std::string>();
          clent.buttons = rpc["buttons"].as<std::vector<uint8_t>>();
          clent.interface_type = rpc["interface_type"].as<std::string>();
          srv_clients_.push_back(clent);
          // TODO: prepare rpc
          // std_srvs::srv::RegisterEmptyClientFunc(core_.GetRpcHandle(), clent.service_name);
        }
      }
    }
    AIMRT_INFO("Init succeeded.");
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }
}

bool JoyStickModule::Start() {
  executor_.Execute([this]() { MainLoop(); });

  AIMRT_INFO("Started succeeded.");
  return true;
}

void JoyStickModule::Shutdown() {
  run_flag_ = false;
  stop_sig_.get_future().wait();

  AIMRT_INFO("Shutdown succeeded.");
}

void JoyStickModule::MainLoop() {
  // TODO: prepare rpc
  // std_srvs::srv::EmptySyncProxy rpc_proxy(core_.GetRpcHandle());
  std_msgs::msg::Float32 button_msgs;
  geometry_msgs::msg::Twist vel_msgs;
  int32_t log_cnt = 0;
  while (run_flag_.load()) {
    JoyStruct joy_data;
    ++log_cnt;
    joy_->GetJoyData(joy_data);

    // 初始化 prev_walk_buttons_（首次循环）
    if (prev_walk_buttons_.size() != joy_data.buttons.size()) {
      prev_walk_buttons_.assign(joy_data.buttons.size(), false);
    }

    // 无手柄时跳过按钮相关发布逻辑
    if (!joy_data.buttons.empty()) {
      for (auto& float_pub : float_pubs_) {
        bool ret = true;
        for (auto button : float_pub.buttons) {
          if (button >= joy_data.buttons.size()) { ret = false; break; }
          ret &= joy_data.buttons[button];
        }
        if (ret) {
          aimrt::channel::Publish<std_msgs::msg::Float32>(float_pub.pub, button_msgs);
        }

        // 检测 /walk_mode 按钮上升沿，切换行走模式
        if (float_pub.topic_name == "/walk_mode") {
          bool all_pressed = true;
          bool all_pressed_prev = true;
          for (auto button : float_pub.buttons) {
            if (button >= joy_data.buttons.size() || button >= prev_walk_buttons_.size()) {
              all_pressed = false; all_pressed_prev = false; break;
            }
            all_pressed &= static_cast<bool>(joy_data.buttons[button]);
            all_pressed_prev &= prev_walk_buttons_[button];
          }
          if (all_pressed && !all_pressed_prev) {
            walk_mode_active_ = !walk_mode_active_;
            AIMRT_INFO("[JoyStick] Walk mode {}", walk_mode_active_ ? "ACTIVATED" : "DEACTIVATED");
          }
        }
      }

      // 更新上一帧按钮状态
      for (size_t i = 0; i < joy_data.buttons.size(); ++i) {
        prev_walk_buttons_[i] = static_cast<bool>(joy_data.buttons[i]);
      }
    }

    // ── LT 刹车检测（在 RT 或 walk_mode 行走时按 LT 减速到 0）──
    bool lt_pressed = false;
    if (joy_data.axis.size() > 2) {
      lt_pressed = joy_data.axis[2] < -0.5;  // LT 全按约 -1.0
    }
    if (lt_pressed && !prev_lt_pressed_ && !lt_brake_active_) {
      if (rt_walk_active_) {
        // 从 RT 行走刹车
        lt_brake_active_ = true;
        lt_brake_from_rt_ = true;
        lt_brake_vel_ = rt_linear_x_;
        rt_walk_active_ = false;  // 停止 RT 闭环
        AIMRT_INFO("[JoyStick] LT brake from RT, decelerating from {:.3f} m/s", lt_brake_vel_);
      } else if (walk_mode_active_) {
        // 从 walk_mode（摇杆/固定速度）刹车
        lt_brake_active_ = true;
        lt_brake_from_rt_ = false;
        lt_brake_vel_ = 0.4;  // walk_mode 固定速度
        AIMRT_INFO("[JoyStick] LT brake from walk_mode, decelerating from {:.3f} m/s", lt_brake_vel_);
      }
    }
    prev_lt_pressed_ = lt_pressed;

    // LT 刹车减速
    if (lt_brake_active_) {
      constexpr double LT_BRAKE_RAMP = 0.1;  // 减速变化率 m/s²
      lt_brake_vel_ -= LT_BRAKE_RAMP * (1.0 / freq_);
      if (lt_brake_vel_ <= 0.0) {
        lt_brake_vel_ = 0.0;
        lt_brake_active_ = false;
        // 回到 stand
        for (auto& fp : float_pubs_)
          if (fp.topic_name == "/stand_mode")
            aimrt::channel::Publish<std_msgs::msg::Float32>(fp.pub, button_msgs);
        AIMRT_INFO("[JoyStick] LT brake COMPLETE, back to stand");
      }

      // 发布减速速度
      vel_msgs.linear.x = lt_brake_vel_;
      vel_msgs.linear.y = 0.0;
      vel_msgs.linear.z = 0.0;
      vel_msgs.angular.x = 0.0;
      vel_msgs.angular.y = 0.0;
      vel_msgs.angular.z = 0.0;
      for (auto& tp : twist_pubs_) {
        aimrt::channel::Publish<geometry_msgs::msg::Twist>(tp.pub, vel_msgs);
        if (tp.pub_limiter)
          aimrt::channel::Publish<geometry_msgs::msg::Twist>(tp.pub_limiter, vel_msgs);
      }
      if (log_cnt % 20 == 0)
        AIMRT_INFO("[JoyStick] LT brake: vx={:.3f} m/s", lt_brake_vel_);
    }

    // ── RT 直线行走检测（IMU 偏航闭环）──
    bool rt_pressed = false;
    if (joy_data.axis.size() > 5) {
      rt_pressed = joy_data.axis[5] < -0.5;  // RT 全按约 -1.0
    }
    // 上升沿触发：切换 RT 行走开/关
    if (rt_pressed && !prev_rt_pressed_) {
      if (!rt_walk_active_ && !lt_brake_active_ && imu_received_) {
        // ── 开启：读取当前 yaw 作为目标航向 ──
        {
          std::shared_lock<std::shared_mutex> lock(imu_mutex_);
          double qw = latest_imu_.orientation.w;
          double qx = latest_imu_.orientation.x;
          double qy = latest_imu_.orientation.y;
          double qz = latest_imu_.orientation.z;
          double siny_cosp = 2.0 * (qw * qz + qx * qy);
          double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
          rt_yaw_target_ = std::atan2(siny_cosp, cosy_cosp);
        }
        rt_yaw_integral_ = 0.0;  // 重置积分项
        rt_walk_active_ = true;
        rt_t0_ = std::chrono::steady_clock::now();
        AIMRT_INFO("[JoyStick] RT auto-walk STARTED, target_yaw={:.3f} rad", rt_yaw_target_);
      } else if (rt_walk_active_) {
        // ── 关闭：发布零速度 + 回 stand ──
        rt_walk_active_ = false;
        vel_msgs.linear.x = 0.0;
        vel_msgs.angular.z = 0.0;
        for (auto& tp : twist_pubs_) {
          aimrt::channel::Publish<geometry_msgs::msg::Twist>(tp.pub, vel_msgs);
          if (tp.pub_limiter)
            aimrt::channel::Publish<geometry_msgs::msg::Twist>(tp.pub_limiter, vel_msgs);
        }
        // 发 /stand_mode 回到站立
        for (auto& fp : float_pubs_)
          if (fp.topic_name == "/stand_mode")
            aimrt::channel::Publish<std_msgs::msg::Float32>(fp.pub, button_msgs);
        AIMRT_INFO("[JoyStick] RT auto-walk STOPPED, back to stand");
      }
    }
    prev_rt_pressed_ = rt_pressed;

    // ── RT 偏航闭环控制 ──
    if (rt_walk_active_ && !lt_brake_active_) {
      double rt_t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - rt_t0_).count();
      constexpr double RT_MODE_RETRY = 1.0;  // walk_mode 重复发布时长 s
      constexpr double RT_SETTLE = 1.0;      // 进入 walk_leg 后等待稳定 s

      // 启动阶段：发 /walk_mode 切换到 walk_leg
      if (rt_t < RT_MODE_RETRY) {
        for (auto& fp : float_pubs_)
          if (fp.topic_name == "/walk_mode")
            aimrt::channel::Publish<std_msgs::msg::Float32>(fp.pub, button_msgs);
      }

      // 等待步态稳定期间不发速度
      if (rt_t < RT_MODE_RETRY + RT_SETTLE) {
        if (log_cnt % 20 == 0)
          AIMRT_INFO("[JoyStick] RT walk starting... t={:.2f}s", rt_t);
      } else {
      double yaw_current = 0.0;
      double gyro_z = 0.0;
      {
        std::shared_lock<std::shared_mutex> lock(imu_mutex_);
        if (imu_received_) {
          double qw = latest_imu_.orientation.w;
          double qx = latest_imu_.orientation.x;
          double qy = latest_imu_.orientation.y;
          double qz = latest_imu_.orientation.z;
          double siny_cosp = 2.0 * (qw * qz + qx * qy);
          double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
          yaw_current = std::atan2(siny_cosp, cosy_cosp);
          gyro_z = latest_imu_.angular_velocity.z;
        }
      }

      // 偏航误差（角度归一化到 [-π, π]）
      double yaw_err = rt_yaw_target_ - yaw_current;
      while (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
      while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

      // 积分项累积（带限幅防饱和）
      double dt = 1.0 / freq_;
      rt_yaw_integral_ += yaw_err * dt;
      rt_yaw_integral_ = std::clamp(rt_yaw_integral_, -rt_i_limit_ / rt_yaw_ki_, rt_i_limit_ / rt_yaw_ki_);

      // PID 控制：angular.z = Kp·err + Ki·∫err + Kd·(-gyro_z)
      double cmd_angular_z = rt_yaw_kp_ * yaw_err + rt_yaw_ki_ * rt_yaw_integral_ + rt_yaw_kd_ * (-gyro_z);
      cmd_angular_z = std::clamp(cmd_angular_z, -rt_max_angular_z_, rt_max_angular_z_);

      // 发布速度（和手柄完全相同的通道）
      vel_msgs.linear.x = rt_linear_x_;
      vel_msgs.linear.y = 0.0;
      vel_msgs.linear.z = 0.0;
      vel_msgs.angular.x = 0.0;
      vel_msgs.angular.y = 0.0;
      vel_msgs.angular.z = cmd_angular_z;
      for (auto& tp : twist_pubs_) {
        aimrt::channel::Publish<geometry_msgs::msg::Twist>(tp.pub, vel_msgs);
        if (tp.pub_limiter)
          aimrt::channel::Publish<geometry_msgs::msg::Twist>(tp.pub_limiter, vel_msgs);
      }

      if (log_cnt % 20 == 0)
        AIMRT_INFO("[JoyStick] RT walk: yaw_err={:.3f} rad, cmd_wz={:.3f} rad/s",
                   yaw_err, cmd_angular_z);
      }  // end else (after settle)
    }

    if (!lt_brake_active_ && !rt_walk_active_) {
    for (auto twist_pub : twist_pubs_) {
      // 行走模式激活时绕过按钮检查，直接发布；否则需按住对应按钮
      bool ret = walk_mode_active_;
      if (!ret) {
        ret = !twist_pub.buttons.empty() && !joy_data.buttons.empty();
        for (auto button : twist_pub.buttons) {
          if (button >= joy_data.buttons.size()) { ret = false; break; }
          ret &= joy_data.buttons[button];
        }
      }
      if (ret) {
        // 固定速度模式：忽略摇杆轴，直接发布预设速度
        if (twist_pub.use_constant_velocity) {
          vel_msgs.linear.x = 0.0;
          vel_msgs.linear.y = 0.0;
          vel_msgs.linear.z = 0.0;
          vel_msgs.angular.x = 0.0;
          vel_msgs.angular.y = 0.0;
          vel_msgs.angular.z = 0.0;
          auto& cv = twist_pub.constant_velocity;
          if (cv.count("linear-x"))  vel_msgs.linear.x  = cv.at("linear-x");
          if (cv.count("linear-y"))  vel_msgs.linear.y  = cv.at("linear-y");
          if (cv.count("linear-z"))  vel_msgs.linear.z  = cv.at("linear-z");
          if (cv.count("angular-x")) vel_msgs.angular.x = cv.at("angular-x");
          if (cv.count("angular-y")) vel_msgs.angular.y = cv.at("angular-y");
          if (cv.count("angular-z")) vel_msgs.angular.z = cv.at("angular-z");
          if (log_cnt % 50 == 0) {
            AIMRT_INFO("[JoyStick] Constant vel -> linear=[{:.3f}, {:.3f}, {:.3f}] angular=[{:.3f}, {:.3f}, {:.3f}]",
                vel_msgs.linear.x, vel_msgs.linear.y, vel_msgs.linear.z,
                vel_msgs.angular.x, vel_msgs.angular.y, vel_msgs.angular.z);
          }
          aimrt::channel::Publish<geometry_msgs::msg::Twist>(twist_pub.pub, vel_msgs);
          // 同时发布到 /cmd_vel_limiter，control_module 监听的是这个 topic
          if (twist_pub.pub_limiter) {
            aimrt::channel::Publish<geometry_msgs::msg::Twist>(twist_pub.pub_limiter, vel_msgs);
          }
        } else if (limiter_) {
          // 原有摇杆轴控制逻辑
          array_t target_pos;
          target_pos.resize(joy_data.axis.size());
          int32_t idx = 0;

          if (twist_pub.axis.find("linear-x") != twist_pub.axis.end()) {
            vel_msgs.linear.x = joy_data.axis[twist_pub.axis["linear-x"]];
            target_pos[idx++] = joy_data.axis[twist_pub.axis["linear-x"]];
          }
          if (twist_pub.axis.find("linear-y") != twist_pub.axis.end()) {
            vel_msgs.linear.y = joy_data.axis[twist_pub.axis["linear-y"]];
            target_pos[idx++] = joy_data.axis[twist_pub.axis["linear-y"]];
          }
          if (twist_pub.axis.find("linear-z") != twist_pub.axis.end()) {
            vel_msgs.linear.z = joy_data.axis[twist_pub.axis["linear-z"]];
            target_pos[idx++] = joy_data.axis[twist_pub.axis["linear-z"]];
          }
          if (twist_pub.axis.find("angular-x") != twist_pub.axis.end()) {
            vel_msgs.angular.x = joy_data.axis[twist_pub.axis["angular-x"]];
            target_pos[idx++] = joy_data.axis[twist_pub.axis["angular-x"]];
          }
          if (twist_pub.axis.find("angular-y") != twist_pub.axis.end()) {
            vel_msgs.angular.y = joy_data.axis[twist_pub.axis["angular-y"]];
            target_pos[idx++] = joy_data.axis[twist_pub.axis["angular-y"]];
          }
          if (twist_pub.axis.find("angular-z") != twist_pub.axis.end()) {
            vel_msgs.angular.z = joy_data.axis[twist_pub.axis["angular-z"]];
            target_pos[idx++] = joy_data.axis[twist_pub.axis["angular-z"]];
          }
          if (log_cnt % 50 == 0) {
            AIMRT_INFO("[JoyStick] Joystick vel -> linear=[{:.3f}, {:.3f}, {:.3f}] angular=[{:.3f}, {:.3f}, {:.3f}]",
                vel_msgs.linear.x, vel_msgs.linear.y, vel_msgs.linear.z,
                vel_msgs.angular.x, vel_msgs.angular.y, vel_msgs.angular.z);
          }
          aimrt::channel::Publish<geometry_msgs::msg::Twist>(twist_pub.pub, vel_msgs);

          array_t state = limiter_->update(target_pos);
          idx = 0;
          if (twist_pub.axis.find("linear-x") != twist_pub.axis.end()) {
            vel_msgs.linear.x = state[idx++];
          }
          if (twist_pub.axis.find("linear-y") != twist_pub.axis.end()) {
            vel_msgs.linear.y = state[idx++];
          }
          if (twist_pub.axis.find("linear-z") != twist_pub.axis.end()) {
            vel_msgs.linear.z = state[idx++];
          }
          if (twist_pub.axis.find("angular-x") != twist_pub.axis.end()) {
            vel_msgs.angular.x = state[idx++];
          }
          if (twist_pub.axis.find("angular-y") != twist_pub.axis.end()) {
            vel_msgs.angular.y = state[idx++];
          }
          if (twist_pub.axis.find("angular-z") != twist_pub.axis.end()) {
            vel_msgs.angular.z = state[idx++];
          }
          aimrt::channel::Publish<geometry_msgs::msg::Twist>(twist_pub.pub_limiter, vel_msgs);
        }
      }
    }
    }  // end if (!lt_brake_active_ && !rt_walk_active_)

    for (auto srv_client : srv_clients_) {
      bool ret = true;
      for (auto button : srv_client.buttons) {
        ret &= joy_data.buttons[button];
      }
      if (ret) {
        std::string cmd = "ros2 service call /" + srv_client.service_name + " " +
                          srv_client.interface_type + " > /dev/null &";
        int ret = system(cmd.data());
        AIMRT_INFO("Call /reset_word");
        // TODO: call rpc
        // rpc_proxy.SetServiceName("reset_world");
        // auto status = rpc_proxy.Empty(req, rsp);
        // if (status.OK()) {
        //   AIMRT_INFO("GetFooData success rsp: {}", std_srvs::srv::to_yaml(rsp));
        // } else {
        //   AIMRT_WARN("Call GetFooData failed, status: {}", status.ToString());
        // }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / freq_));
  }

  stop_sig_.set_value();
}

}  // namespace xyber_x1_infer::joy_stick_module
