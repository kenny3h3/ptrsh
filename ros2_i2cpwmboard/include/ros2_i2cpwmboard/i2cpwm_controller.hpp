#ifndef I2CPWM_CONTROLLER_HPP_
#define I2CPWM_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ros2_i2cpwmboard/msg/servo_config.hpp>
#include <ros2_i2cpwmboard/srv/int_value.hpp>
#include <ros2_i2cpwmboard/srv/drive_mode.hpp>
#include <ros2_i2cpwmboard/srv/servos_config.hpp>
#include <ros2_i2cpwmboard/srv/stop_servos.hpp>
#include <ros2_i2cpwmboard/msg/servo_array.hpp>

class I2CPWMController : public rclcpp::Node {
public:
    I2CPWMController();

private:
    std::map<int, ros2_i2cpwmboard::msg::ServoConfig> servo_configs_;
    rclcpp::Service<ros2_i2cpwmboard::srv::IntValue>::SharedPtr srv_set_value_;
    rclcpp::Service<ros2_i2cpwmboard::srv::DriveMode>::SharedPtr srv_drive_mode_;
    rclcpp::Service<ros2_i2cpwmboard::srv::ServosConfig>::SharedPtr srv_servos_config_;
    rclcpp::Service<ros2_i2cpwmboard::srv::StopServos>::SharedPtr srv_stop_servos_;
    rclcpp::Subscription<ros2_i2cpwmboard::msg::ServoArray>::SharedPtr servo_absolute_sub_;
    rclcpp::Subscription<ros2_i2cpwmboard::msg::ServoArray>::SharedPtr servo_proportional_sub_;

    void handle_set_value(const std::shared_ptr<ros2_i2cpwmboard::srv::IntValue::Request> request,
                          std::shared_ptr<ros2_i2cpwmboard::srv::IntValue::Response> response);
    void handle_drive_mode(const std::shared_ptr<ros2_i2cpwmboard::srv::DriveMode::Request> request,
                           std::shared_ptr<ros2_i2cpwmboard::srv::DriveMode::Response> response);
    void handle_servos_config(const std::shared_ptr<ros2_i2cpwmboard::srv::ServosConfig::Request> request,
                              std::shared_ptr<ros2_i2cpwmboard::srv::ServosConfig::Response> response);
    void handle_stop_servos(const std::shared_ptr<ros2_i2cpwmboard::srv::StopServos::Request> request,
                            std::shared_ptr<ros2_i2cpwmboard::srv::StopServos::Response> response);
    void handle_servo_absolute(const std::shared_ptr<ros2_i2cpwmboard::msg::ServoArray> msg);
    void handle_servo_proportional(const std::shared_ptr<ros2_i2cpwmboard::msg::ServoArray> msg);

    void setPWM(int channel, int on_time, int off_time);
};

#endif