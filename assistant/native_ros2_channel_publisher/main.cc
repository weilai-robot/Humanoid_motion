// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <chrono>
#include <memory>
#include <string>

#include "my_ros2_proto/msg/joy_stick_data.hpp"
#include "rclcpp/rclcpp.hpp"

class RosTestChannelPublisher : public rclcpp::Node {
 public:
  RosTestChannelPublisher()
      : Node("native_ros2_channel_publisher") {
    using namespace std::chrono_literals;

    publisher_ =
        this->create_publisher<my_ros2_proto::msg::JoyStickData>("test_topic", 10);
    timer_ = this->create_wall_timer(
        500ms,
        [this]() -> void {
          ++count_;
          my_ros2_proto::msg::JoyStickData msg;
          msg.name = "joy_stick_module_data";
          msg.x = count_;
          msg.y = count_;
          msg.z = count_;

          RCLCPP_INFO(get_logger(), "Publishing msg:\n%s",
                      my_ros2_proto::msg::to_yaml(msg).c_str());
          publisher_->publish(msg);
        });
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_ros2_proto::msg::JoyStickData>::SharedPtr publisher_;
  size_t count_ = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosTestChannelPublisher>());
  rclcpp::shutdown();
  return 0;
}
