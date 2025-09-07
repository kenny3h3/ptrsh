#pragma once

#include <rclcpp/rclcpp.hpp>
#include <map>

#include "ros2_i2cpwmboard/msg/servo_array.hpp"
#include "ros2_i2cpwmboard/msg/servo_config.hpp"
#include "ros2_i2cpwmboard/srv/drive_mode.hpp"
#include "ros2_i2cpwmboard/srv/servos_config.hpp"
#include "ros2_i2cpwmboard/srv/stop_servos.hpp"

class I2CPWMController : public rclcpp::Node {
public:
  I2CPWMController();

private:
  // Services
  void handle_drive_mode(
      const std::shared_ptr<ros2_i2cpwmboard::srv::DriveMode::Request> request,
      std::shared_ptr<ros2_i2cpwmboard::srv::DriveMode::Response> response);

  void handle_servos_config(
      const std::shared_ptr<ros2_i2cpwmboard::srv::ServosConfig::Request> request,
      std::shared_ptr<ros2_i2cpwmboard::srv::ServosConfig::Response> response);

  void handle_stop_servos(
      const std::shared_ptr<ros2_i2cpwmboard::srv::StopServos::Request> request,
      std::shared_ptr<ros2_i2cpwmboard::srv::StopServos::Response> response);

  // Subscriptions
  void handle_servo_absolute(
      const std::shared_ptr<ros2_i2cpwmboard::msg::ServoArray> msg);

  void handle_servo_proportional(
      const std::shared_ptr<ros2_i2cpwmboard::msg::ServoArray> msg);

  // Hardware abstraction (Stub – implementiere I2C hier)
  void setPWM(int channel, int value);

  // Members
  rclcpp::Service<ros2_i2cpwmboard::srv::DriveMode>::SharedPtr   srv_drive_mode_;
  rclcpp::Service<ros2_i2cpwmboard::srv::ServosConfig>::SharedPtr srv_servos_config_;
  rclcpp::Service<ros2_i2cpwmboard::srv::StopServos>::SharedPtr   srv_stop_servos_;

  rclcpp::Subscription<ros2_i2cpwmboard::msg::ServoArray>::SharedPtr sub_servo_absolute_;
  rclcpp::Subscription<ros2_i2cpwmboard::msg::ServoArray>::SharedPtr sub_servo_proportional_;

  std::map<int, ros2_i2cpwmboard::msg::ServoConfig> servo_configs_;
};
