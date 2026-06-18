// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#include "sim_module/sim_module.h"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

namespace xyber_x1_infer::sim_module {

bool SimModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  start_time_ = high_resolution_clock::now();

  core_ = core;
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  if (file_path.empty()) {
    AIMRT_ERROR("Init failed, [file_path] Empty");
    return false;
  }
  try {
    YAML::Node cfg_node = YAML::LoadFile(file_path.data());
    filename_ = cfg_node["model_file"].as<std::string>();

    joint_cmd_sub_ = core_.GetChannelHandle().GetSubscriber(cfg_node["sub_joint_cmd_topic"].as<std::string>());
    aimrt::channel::Subscribe<my_ros2_proto::msg::JointCommand>(joint_cmd_sub_, std::bind(&SimModule::CmdCallback, this, std::placeholders::_1));
    imu_data_pub_ = core_.GetChannelHandle().GetPublisher(cfg_node["pub_imu_data_topic"].as<std::string>());
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::Imu>(imu_data_pub_);
    joint_state_pub_ =core_.GetChannelHandle().GetPublisher(cfg_node["pub_joint_state_topic"].as<std::string>());
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::JointState>(joint_state_pub_);

    // lidar IMU publisher (optional, for navigation sim)
    if (cfg_node["pub_lidar_imu_topic"]) {
      lidar_imu_pub_ = core_.GetChannelHandle().GetPublisher(cfg_node["pub_lidar_imu_topic"].as<std::string>());
      aimrt::channel::RegisterPublishType<sensor_msgs::msg::Imu>(lidar_imu_pub_);
    }
    // base pose publisher (optional, for navigation sim)
    if (cfg_node["pub_base_pose_topic"]) {
      base_pose_pub_ = core_.GetChannelHandle().GetPublisher(cfg_node["pub_base_pose_topic"].as<std::string>());
      aimrt::channel::RegisterPublishType<geometry_msgs::msg::PoseStamped>(base_pose_pub_);
    }
    // ground truth publisher (optional, for nav testing metrics)
    if (cfg_node["pub_ground_truth_topic"]) {
      ground_truth_pub_ = core_.GetChannelHandle().GetPublisher(cfg_node["pub_ground_truth_topic"].as<std::string>());
      aimrt::channel::RegisterPublishType<std_msgs::msg::Float64MultiArray>(ground_truth_pub_);
    }

    render_executor_ = core_.GetExecutorManager().GetExecutor("sim_render_thread");

    AIMRT_INFO("Init succeeded.");
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
    return false;
  }
}

bool SimModule::Start() {
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);

  // Render thread
  render_executor_.Execute([this]() {
    sim_ = std::make_shared<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam_, &opt_, &pert_, false);
    sim_->LoadMessage(filename_.data());
    const int kErrorLength = 1024;
    char loadError[kErrorLength] = "";
    m_ = mj_loadXML(filename_.data(), nullptr, loadError, kErrorLength);
    mju::strcpy_arr(sim_->load_error, loadError);
    if (m_) {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      d_ = mj_makeData(m_);
    }
    is_render_thread_running_ = true;
    sim_->RenderLoop();
  });

  while (!is_render_thread_running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (d_) {
    sim_->Load(m_, d_, filename_.data());
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    mj_forward(m_, d_);
    free(ctrl_noise_);
    ctrl_noise_ = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m_->nu));
    mju_zero(ctrl_noise_, m_->nu);
  } else {
    sim_->LoadMessageClear();
  }

  joint_names_.clear();
  for (int i = 0; i < m_->njnt; ++i) {
    if (m_->jnt_type[i] == mjJNT_FREE) {
      continue;
    }
    const char* joint_name = mj_id2name(m_, mjOBJ_JOINT, i);
    joint_names_.push_back(std::string(joint_name));
  }

  // init pid
  target_q_.resize(joint_names_.size());
  target_dq_.resize(joint_names_.size());
  target_tq_.resize(joint_names_.size());
  kp_.resize(joint_names_.size());
  kd_.resize(joint_names_.size());
  motor_torque_.resize(joint_names_.size());

  // Cache lidar IMU sensor addresses (by name, robust to index changes)
  {
    int sid;
    sid = mj_name2id(m_, mjOBJ_SENSOR, "lidar-orientation");
    lidar_imu_quat_adr_ = (sid >= 0) ? m_->sensor_adr[sid] : -1;
    sid = mj_name2id(m_, mjOBJ_SENSOR, "lidar-angular-velocity");
    lidar_imu_gyro_adr_ = (sid >= 0) ? m_->sensor_adr[sid] : -1;
    sid = mj_name2id(m_, mjOBJ_SENSOR, "lidar-linear-acceleration");
    lidar_imu_accel_adr_ = (sid >= 0) ? m_->sensor_adr[sid] : -1;
  }

  // Cache body/geom IDs for ground truth metrics
  {
    robot_root_body_id_ = mj_name2id(m_, mjOBJ_BODY, "x1-body");
    floor_geom_id_ = mj_name2id(m_, mjOBJ_GEOM, "floor");
    if (robot_root_body_id_ >= 0) {
      AIMRT_INFO("Ground truth: robot_root_body_id={}, floor_geom_id={}",
                 robot_root_body_id_, floor_geom_id_);
    }
  }

  AIMRT_INFO("Started succeeded.");
  return true;
}

void SimModule::Shutdown() {
  free(ctrl_noise_);
  mj_deleteData(d_);
  mj_deleteModel(m_);
  AIMRT_INFO("Shutdown succeeded.");
}

void SimModule::CmdCallback(const std::shared_ptr<const my_ros2_proto::msg::JointCommand>& msg) {
  sensor_msgs::msg::Imu imu_data_msg;
  sensor_msgs::msg::JointState joint_states_msg;

  auto elapsed = high_resolution_clock::now() - start_time_;
  if (elapsed <= milliseconds(3000)) {
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  WriteMotorCmd(*msg);
  mj_step(m_, d_);
  ReadSensorData(imu_data_msg, joint_states_msg);

  aimrt::channel::Publish<sensor_msgs::msg::Imu>(imu_data_pub_, imu_data_msg);
  aimrt::channel::Publish<sensor_msgs::msg::JointState>(joint_state_pub_, joint_states_msg);

  // Publish lidar IMU (for FastLIO2 in navigation sim)
  if (lidar_imu_pub_ && lidar_imu_quat_adr_ >= 0) {
    sensor_msgs::msg::Imu lidar_imu_msg;
    double sim_time = d_->time;
    lidar_imu_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
    lidar_imu_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - lidar_imu_msg.header.stamp.sec) * 1e9);
    lidar_imu_msg.header.frame_id = "lidar_link";
    lidar_imu_msg.orientation.w = d_->sensordata[lidar_imu_quat_adr_];
    lidar_imu_msg.orientation.x = d_->sensordata[lidar_imu_quat_adr_ + 1];
    lidar_imu_msg.orientation.y = d_->sensordata[lidar_imu_quat_adr_ + 2];
    lidar_imu_msg.orientation.z = d_->sensordata[lidar_imu_quat_adr_ + 3];
    lidar_imu_msg.angular_velocity.x = d_->sensordata[lidar_imu_gyro_adr_];
    lidar_imu_msg.angular_velocity.y = d_->sensordata[lidar_imu_gyro_adr_ + 1];
    lidar_imu_msg.angular_velocity.z = d_->sensordata[lidar_imu_gyro_adr_ + 2];
    lidar_imu_msg.linear_acceleration.x = d_->sensordata[lidar_imu_accel_adr_];
    lidar_imu_msg.linear_acceleration.y = d_->sensordata[lidar_imu_accel_adr_ + 1];
    lidar_imu_msg.linear_acceleration.z = d_->sensordata[lidar_imu_accel_adr_ + 2];
    aimrt::channel::Publish<sensor_msgs::msg::Imu>(lidar_imu_pub_, lidar_imu_msg);
  }

  // Publish base pose (ground truth for LiDAR bridge)
  if (base_pose_pub_) {
    geometry_msgs::msg::PoseStamped pose_msg;
    double sim_time = d_->time;
    pose_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
    pose_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - pose_msg.header.stamp.sec) * 1e9);
    pose_msg.header.frame_id = "odom";
    pose_msg.pose.position.x = d_->qpos[0];
    pose_msg.pose.position.y = d_->qpos[1];
    pose_msg.pose.position.z = d_->qpos[2];
    pose_msg.pose.orientation.w = d_->qpos[3];
    pose_msg.pose.orientation.x = d_->qpos[4];
    pose_msg.pose.orientation.y = d_->qpos[5];
    pose_msg.pose.orientation.z = d_->qpos[6];
    aimrt::channel::Publish<geometry_msgs::msg::PoseStamped>(base_pose_pub_, pose_msg);
  }

  // Publish ground truth metrics (RTF, contacts, trajectory)
  if (ground_truth_pub_) {
    PublishGroundTruth();
  }
}

void SimModule::ReadSensorData(sensor_msgs::msg::Imu& imu_data, sensor_msgs::msg::JointState& joint_state) {
  double sim_time = d_->time;
  auto sec = static_cast<int64_t>(sim_time);
  auto nanosec = static_cast<int64_t>((sim_time - sec) * 1e9);

  imu_data.orientation.w = d_->sensordata[0];
  imu_data.orientation.x = d_->sensordata[1];
  imu_data.orientation.y = d_->sensordata[2];
  imu_data.orientation.z = d_->sensordata[3];
  imu_data.angular_velocity.x = d_->sensordata[4];
  imu_data.angular_velocity.y = d_->sensordata[5];
  imu_data.angular_velocity.z = d_->sensordata[6];
  imu_data.linear_acceleration.x = d_->sensordata[13];
  imu_data.linear_acceleration.y = d_->sensordata[14];
  imu_data.linear_acceleration.z = d_->sensordata[15];
  imu_data.header.stamp.sec = static_cast<int32_t>(sec);
  imu_data.header.stamp.nanosec = static_cast<uint32_t>(nanosec);

  joint_state.name = joint_names_;
  joint_state.position.resize(joint_names_.size(), 0.0);
  joint_state.velocity.resize(joint_names_.size(), 0.0);
  joint_state.effort.resize(joint_names_.size(), 0.0);
  memcpy((void*)joint_state.position.data(), d_->qpos+7, joint_names_.size() * sizeof(double));
  memcpy((void*)joint_state.velocity.data(), d_->qvel+6, joint_names_.size() * sizeof(double));
  memcpy((void*)joint_state.effort.data(), d_->qfrc_actuator+6, joint_names_.size() * sizeof(double));
  joint_state.header.stamp.sec = static_cast<int32_t>(sec);
  joint_state.header.stamp.nanosec = static_cast<uint32_t>(nanosec);
}

void SimModule::WriteMotorCmd(my_ros2_proto::msg::JointCommand cmd) {
  for (size_t ii = 0; ii < cmd.name.size(); ii++) {
    joint_state_index_map_[cmd.name[ii]] = ii;
  }

  for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
    int index = joint_state_index_map_[joint_names_[ii]];
    target_q_(ii) = cmd.position[index];
    target_dq_(ii) = cmd.velocity[index];
    target_tq_(ii) = cmd.effort[index];
    kp_(ii) = cmd.stiffness[index];
    kd_(ii) = cmd.damping[index];
  }
  array_t q = Eigen::Map<array_t>(d_->qpos + 7, joint_names_.size());
  array_t dq = Eigen::Map<array_t>(d_->qvel + 6, joint_names_.size());
  motor_torque_ = target_tq_ + (target_q_ - q) * kp_ + (target_dq_ - dq) * kd_;
  d_->ctrl = motor_torque_.data();

  // 添加控制噪声
  if (sim_->ctrl_noise_std) {
    mjtNum rate = mju_exp(-m_->opt.timestep / mju_max(sim_->ctrl_noise_rate, mjMINVAL));
    mjtNum scale = sim_->ctrl_noise_std * mju_sqrt(1-rate*rate);
    for (int i=0; i<m_->nu; i++) {
      ctrl_noise_[i] = rate * ctrl_noise_[i] + scale * mju_standardNormal(nullptr);
      d_->ctrl[i] += ctrl_noise_[i];
    }
  }
}

bool SimModule::IsRobotGeom(int geom_id) const {
  if (robot_root_body_id_ < 0) return false;
  int body_id = m_->geom_bodyid[geom_id];
  while (body_id > 0) {
    if (body_id == robot_root_body_id_) return true;
    body_id = m_->body_parentid[body_id];
  }
  return false;
}

void SimModule::PublishGroundTruth() {
  // === RTF computation (every step, EMA smoothed) ===
  auto wall_now = high_resolution_clock::now();
  double sim_time = d_->time;

  if (last_rtf_sim_time_ < 0.0) {
    last_rtf_sim_time_ = sim_time;
    last_rtf_wall_time_ = wall_now;
  } else {
    double d_sim = sim_time - last_rtf_sim_time_;
    double d_wall = duration<double>(wall_now - last_rtf_wall_time_).count();
    if (d_wall > 1e-6 && d_sim > 0) {
      double rtf_instant = d_sim / d_wall;
      rtf_smoothed_ = 0.95 * rtf_smoothed_ + 0.05 * rtf_instant;
    }
    last_rtf_sim_time_ = sim_time;
    last_rtf_wall_time_ = wall_now;
  }

  // === Base pose from free joint ===
  double x = d_->qpos[0];
  double y = d_->qpos[1];
  double z = d_->qpos[2];
  // Quaternion: MuJoCo stores [w, x, y, z]
  double qw = d_->qpos[3];
  double qx = d_->qpos[4];
  double qy = d_->qpos[5];
  double qz = d_->qpos[6];

  // Euler angles (XYZ sequence)
  double roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
  double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0*(qw*qy - qz*qx))));
  double yaw   = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));

  // === Distance traveled ===
  if (first_gt_) {
    last_gt_x_ = x;
    last_gt_y_ = y;
    first_gt_ = false;
  } else {
    double dx = x - last_gt_x_;
    double dy = y - last_gt_y_;
    cum_distance_ += std::sqrt(dx*dx + dy*dy);
    last_gt_x_ = x;
    last_gt_y_ = y;
  }

  // === Contact detection (robot vs environment, excluding floor) ===
  int collision_count = 0;
  for (int i = 0; i < d_->ncon; i++) {
    int g1 = d_->contact[i].geom1;
    int g2 = d_->contact[i].geom2;
    bool r1 = IsRobotGeom(g1);
    bool r2 = IsRobotGeom(g2);
    bool f1 = (g1 == floor_geom_id_);
    bool f2 = (g2 == floor_geom_id_);

    // Count: robot touching non-floor environment geom
    if ((r1 && !r2 && !f2) || (r2 && !r1 && !f1)) {
      collision_count++;
    }
  }

  // === Publish Float64MultiArray ===
  // Layout: [sim_time, x, y, z, roll, pitch, yaw, rtf, collisions, cum_distance]
  std_msgs::msg::Float64MultiArray gt_msg;
  gt_msg.data = {
    sim_time,
    x, y, z,
    roll, pitch, yaw,
    rtf_smoothed_,
    static_cast<double>(collision_count),
    cum_distance_
  };
  aimrt::channel::Publish<std_msgs::msg::Float64MultiArray>(ground_truth_pub_, gt_msg);
}

}  // namespace xyber_x1_infer::sim_module
