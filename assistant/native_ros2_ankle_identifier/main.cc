// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "my_ros2_proto/msg/joint_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class NativeRos2AnkleIdentifier : public rclcpp::Node {
 public:
  NativeRos2AnkleIdentifier() : Node("native_ros2_ankle_identifier") {
    DeclareParameters();
    LoadParameters();
    PrepareTargets();
    PrepareCsv();

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, rclcpp::QoS(100),
        std::bind(&NativeRos2AnkleIdentifier::OnJointState, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::QoS(100),
        std::bind(&NativeRos2AnkleIdentifier::OnImu, this, std::placeholders::_1));
    joint_cmd_pub_ =
        create_publisher<my_ros2_proto::msg::JointCommand>(joint_cmd_topic_, rclcpp::QoS(100));

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&NativeRos2AnkleIdentifier::OnTimer, this));

    RCLCPP_WARN(
        get_logger(),
        "This node publishes directly to %s. Do not run it concurrently with another /joint_cmd publisher such as ControlModule.",
        joint_cmd_topic_.c_str());
  }

 private:
  struct JointSnapshot {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
  };

  enum class TestMode { kStep, kSine };

  void DeclareParameters() {
    declare_parameter<std::string>("joint_cmd_topic", "/joint_cmd");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("mode", "step");
    declare_parameter<std::string>("test_side", "left");
    declare_parameter<std::string>("test_axis", "pitch");
    declare_parameter<double>("publish_rate_hz", 1000.0);
    declare_parameter<double>("pre_hold_sec", 2.0);
    declare_parameter<double>("active_sec", 1.0);
    declare_parameter<double>("post_hold_sec", 2.0);
    declare_parameter<int>("repeat_count", 3);
    declare_parameter<double>("step_amplitude_rad", 0.005);
    declare_parameter<double>("sine_amplitude_rad", 0.004);
    declare_parameter<double>("sine_frequency_hz", 1.0);
    declare_parameter<double>("test_kp", 35.0);
    declare_parameter<double>("test_kd", 0.8);
    declare_parameter<double>("hold_kp", 30.0);
    declare_parameter<double>("hold_kd", 1.0);
    declare_parameter<bool>("use_imu", true);
    declare_parameter<bool>("auto_stop_after_test", true);
    declare_parameter<std::string>("csv_path", "ankle_identification.csv");
  }

  void LoadParameters() {
    joint_cmd_topic_ = get_parameter("joint_cmd_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    pre_hold_sec_ = get_parameter("pre_hold_sec").as_double();
    active_sec_ = get_parameter("active_sec").as_double();
    post_hold_sec_ = get_parameter("post_hold_sec").as_double();
    repeat_count_ = get_parameter("repeat_count").as_int();
    step_amplitude_rad_ = get_parameter("step_amplitude_rad").as_double();
    sine_amplitude_rad_ = get_parameter("sine_amplitude_rad").as_double();
    sine_frequency_hz_ = get_parameter("sine_frequency_hz").as_double();
    test_kp_ = get_parameter("test_kp").as_double();
    test_kd_ = get_parameter("test_kd").as_double();
    hold_kp_ = get_parameter("hold_kp").as_double();
    hold_kd_ = get_parameter("hold_kd").as_double();
    use_imu_ = get_parameter("use_imu").as_bool();
    auto_stop_after_test_ = get_parameter("auto_stop_after_test").as_bool();
    csv_path_ = get_parameter("csv_path").as_string();

    const auto mode = get_parameter("mode").as_string();
    if (mode == "step") {
      test_mode_ = TestMode::kStep;
    } else if (mode == "sine") {
      test_mode_ = TestMode::kSine;
    } else {
      throw std::runtime_error("Unsupported mode: " + mode);
    }

    test_side_ = get_parameter("test_side").as_string();
    test_axis_ = get_parameter("test_axis").as_string();
  }

  void PrepareTargets() {
    if (test_side_ != "left" && test_side_ != "right") {
      throw std::runtime_error("test_side must be left or right");
    }
    if (test_axis_ != "pitch" && test_axis_ != "roll") {
      throw std::runtime_error("test_axis must be pitch or roll");
    }

    if (test_side_ == "left") {
      primary_joint_ = test_axis_ == "pitch" ? "left_ankle_pitch_joint" : "left_ankle_roll_joint";
      coupled_joint_ = test_axis_ == "pitch" ? "left_ankle_roll_joint" : "left_ankle_pitch_joint";
    } else {
      primary_joint_ =
          test_axis_ == "pitch" ? "right_ankle_pitch_joint" : "right_ankle_roll_joint";
      coupled_joint_ =
          test_axis_ == "pitch" ? "right_ankle_roll_joint" : "right_ankle_pitch_joint";
    }
  }

  void PrepareCsv() {
    const auto path = std::filesystem::path(csv_path_);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    csv_ << "time_sec,phase,iteration,primary_joint,coupled_joint,target_primary,target_coupled,"
            "actual_primary,actual_coupled,actual_primary_vel,actual_coupled_vel,actual_primary_effort,"
            "actual_coupled_effort,imu_w,imu_x,imu_y,imu_z,gyro_x,gyro_y,gyro_z\n";
  }

  void OnJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!have_joint_index_) {
      joint_names_ = msg->name;
      for (size_t i = 0; i < msg->name.size(); ++i) {
        joint_index_[msg->name[i]] = i;
      }
      if (!joint_index_.count(primary_joint_) || !joint_index_.count(coupled_joint_)) {
        RCLCPP_FATAL(get_logger(), "Target joints not found in /joint_states.");
        throw std::runtime_error("Target joints not found in /joint_states");
      }
      baseline_cmd_name_ = joint_names_;
      baseline_cmd_.name = joint_names_;
      baseline_cmd_.position.resize(joint_names_.size(), 0.0);
      baseline_cmd_.velocity.resize(joint_names_.size(), 0.0);
      baseline_cmd_.effort.resize(joint_names_.size(), 0.0);
      baseline_cmd_.stiffness.resize(joint_names_.size(), hold_kp_);
      baseline_cmd_.damping.resize(joint_names_.size(), hold_kd_);
      have_joint_index_ = true;
      RCLCPP_INFO(get_logger(), "Joint index initialized with %zu joints.", joint_names_.size());
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
      latest_joint_state_[msg->name[i]] = JointSnapshot{
          .position = i < msg->position.size() ? msg->position[i] : 0.0,
          .velocity = i < msg->velocity.size() ? msg->velocity[i] : 0.0,
          .effort = i < msg->effort.size() ? msg->effort[i] : 0.0};
    }

    if (!baseline_captured_) {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        baseline_cmd_.position[i] = latest_joint_state_[joint_names_[i]].position;
      }
      baseline_captured_ = true;
      start_time_ = now();
      RCLCPP_INFO(get_logger(), "Baseline captured. Test joint: %s, coupled joint: %s",
                  primary_joint_.c_str(), coupled_joint_.c_str());
    }
  }

  void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!use_imu_) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_ = *msg;
  }

  void OnTimer() {
    if (!baseline_captured_ || !have_joint_index_) {
      return;
    }

    const auto current_time = now();
    const double elapsed = (current_time - start_time_).seconds();
    const double cycle_sec = pre_hold_sec_ + active_sec_ + post_hold_sec_;
    const int iteration = static_cast<int>(std::floor(elapsed / cycle_sec));

    if (iteration >= repeat_count_) {
      PublishHoldCommand();
      if (auto_stop_after_test_) {
        RCLCPP_INFO(get_logger(), "Test completed. CSV written to %s", csv_path_.c_str());
        rclcpp::shutdown();
      }
      return;
    }

    const double local_time = elapsed - iteration * cycle_sec;
    std::string phase = "pre_hold";
    double primary_target = GetBaseline(primary_joint_);

    if (local_time < pre_hold_sec_) {
      phase = "pre_hold";
    } else if (local_time < pre_hold_sec_ + active_sec_) {
      phase = "active";
      const double active_time = local_time - pre_hold_sec_;
      if (test_mode_ == TestMode::kStep) {
        primary_target += step_amplitude_rad_;
      } else {
        primary_target +=
            sine_amplitude_rad_ * std::sin(2.0 * M_PI * sine_frequency_hz_ * active_time);
      }
    } else {
      phase = "post_hold";
    }

    my_ros2_proto::msg::JointCommand cmd = baseline_cmd_;
    SetJointCmd(cmd, primary_joint_, primary_target, DesiredPrimaryVelocity(local_time), 0.0,
                test_kp_, test_kd_);
    SetJointCmd(cmd, coupled_joint_, GetBaseline(coupled_joint_), 0.0, 0.0, test_kp_, test_kd_);

    joint_cmd_pub_->publish(cmd);
    LogSample(elapsed, phase, iteration + 1, primary_target, GetBaseline(coupled_joint_));
  }

  double DesiredPrimaryVelocity(double local_time) const {
    if (test_mode_ == TestMode::kSine &&
        local_time >= pre_hold_sec_ && local_time < pre_hold_sec_ + active_sec_) {
      const double active_time = local_time - pre_hold_sec_;
      return 2.0 * M_PI * sine_frequency_hz_ * sine_amplitude_rad_ *
             std::cos(2.0 * M_PI * sine_frequency_hz_ * active_time);
    }
    return 0.0;
  }

  void PublishHoldCommand() {
    if (!baseline_captured_) return;
    joint_cmd_pub_->publish(baseline_cmd_);
  }

  void SetJointCmd(my_ros2_proto::msg::JointCommand& cmd, const std::string& joint_name,
                   double position, double velocity, double effort, double kp, double kd) {
    const size_t idx = joint_index_.at(joint_name);
    cmd.position[idx] = position;
    cmd.velocity[idx] = velocity;
    cmd.effort[idx] = effort;
    cmd.stiffness[idx] = kp;
    cmd.damping[idx] = kd;
  }

  double GetBaseline(const std::string& joint_name) const {
    return baseline_cmd_.position[joint_index_.at(joint_name)];
  }

  void LogSample(double elapsed, const std::string& phase, int iteration, double target_primary,
                 double target_coupled) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto primary = latest_joint_state_[primary_joint_];
    const auto coupled = latest_joint_state_[coupled_joint_];

    csv_ << std::fixed << std::setprecision(6) << elapsed << "," << phase << "," << iteration
         << "," << primary_joint_ << "," << coupled_joint_ << "," << target_primary << ","
         << target_coupled << "," << primary.position << "," << coupled.position << ","
         << primary.velocity << "," << coupled.velocity << "," << primary.effort << ","
         << coupled.effort << ",";

    if (use_imu_) {
      csv_ << latest_imu_.orientation.w << "," << latest_imu_.orientation.x << ","
           << latest_imu_.orientation.y << "," << latest_imu_.orientation.z << ","
           << latest_imu_.angular_velocity.x << "," << latest_imu_.angular_velocity.y << ","
           << latest_imu_.angular_velocity.z;
    } else {
      csv_ << "0,0,0,0,0,0,0";
    }
    csv_ << "\n";
  }

  std::string joint_cmd_topic_;
  std::string joint_state_topic_;
  std::string imu_topic_;
  std::string test_side_;
  std::string test_axis_;
  std::string primary_joint_;
  std::string coupled_joint_;
  std::string csv_path_;

  double publish_rate_hz_ = 1000.0;
  double pre_hold_sec_ = 2.0;
  double active_sec_ = 1.0;
  double post_hold_sec_ = 2.0;
  double step_amplitude_rad_ = 0.005;
  double sine_amplitude_rad_ = 0.004;
  double sine_frequency_hz_ = 1.0;
  double test_kp_ = 35.0;
  double test_kd_ = 0.8;
  double hold_kp_ = 30.0;
  double hold_kd_ = 1.0;
  int repeat_count_ = 3;
  bool use_imu_ = true;
  bool auto_stop_after_test_ = true;

  TestMode test_mode_ = TestMode::kStep;
  bool have_joint_index_ = false;
  bool baseline_captured_ = false;

  rclcpp::Time start_time_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> baseline_cmd_name_;
  std::unordered_map<std::string, size_t> joint_index_;
  std::unordered_map<std::string, JointSnapshot> latest_joint_state_;
  sensor_msgs::msg::Imu latest_imu_;
  my_ros2_proto::msg::JointCommand baseline_cmd_;

  std::mutex data_mutex_;
  std::ofstream csv_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<my_ros2_proto::msg::JointCommand>::SharedPtr joint_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NativeRos2AnkleIdentifier>());
  rclcpp::shutdown();
  return 0;
}
