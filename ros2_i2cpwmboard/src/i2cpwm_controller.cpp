#include "ros2_i2cpwmboard/i2cpwm_controller.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

I2CPWMController::I2CPWMController() : rclcpp::Node("i2cpwm_controller") {
  // Services
  srv_drive_mode_ = this->create_service<ros2_i2cpwmboard::srv::DriveMode>(
      "drive_mode",
      std::bind(&I2CPWMController::handle_drive_mode, this, _1, _2));

  srv_servos_config_ = this->create_service<ros2_i2cpwmboard::srv::ServosConfig>(
      "servos_config",
      std::bind(&I2CPWMController::handle_servos_config, this, _1, _2));

  srv_stop_servos_ = this->create_service<ros2_i2cpwmboard::srv::StopServos>(
      "stop_servos",
      std::bind(&I2CPWMController::handle_stop_servos, this, _1, _2));

  // Subscriptions
  sub_servo_absolute_ = this->create_subscription<ros2_i2cpwmboard::msg::ServoArray>(
      "servo_absolute", rclcpp::QoS(10),
      std::bind(&I2CPWMController::handle_servo_absolute, this, _1));

  sub_servo_proportional_ = this->create_subscription<ros2_i2cpwmboard::msg::ServoArray>(
      "servo_proportional", rclcpp::QoS(10),
      std::bind(&I2CPWMController::handle_servo_proportional, this, _1));

  RCLCPP_INFO(this->get_logger(), "i2cpwm_controller node started");
}

// Services
void I2CPWMController::handle_drive_mode(
    const std::shared_ptr<ros2_i2cpwmboard::srv::DriveMode::Request> request,
    std::shared_ptr<ros2_i2cpwmboard::srv::DriveMode::Response> /*response*/) {

  RCLCPP_INFO(this->get_logger(),
              "DriveMode: mode=%s, rpm=%.2f, radius=%.2f, track=%.2f, scale=%.2f",
              request->mode.c_str(), request->rpm, request->radius,
              request->track, request->scale);

  // TODO: Fahr-/Antriebslogik
}

void I2CPWMController::handle_servos_config(
    const std::shared_ptr<ros2_i2cpwmboard::srv::ServosConfig::Request> request,
    std::shared_ptr<ros2_i2cpwmboard::srv::ServosConfig::Response> /*response*/) {

  for (const auto & cfg : request->servos) {
    servo_configs_[cfg.servo] = cfg;  // center, range, direction, scale
    RCLCPP_INFO(this->get_logger(),
                "Configured servo %d: center=%d, range=%d, direction=%d, scale=%d",
                cfg.servo, cfg.center, cfg.range, cfg.direction, cfg.scale);
  }
}

void I2CPWMController::handle_stop_servos(
    const std::shared_ptr<ros2_i2cpwmboard::srv::StopServos::Request> /*request*/,
    std::shared_ptr<ros2_i2cpwmboard::srv::StopServos::Response> /*response*/) {

  for (auto & kv : servo_configs_) {
    setPWM(kv.first, 0);
  }
  RCLCPP_INFO(this->get_logger(), "All servos stopped.");
}

// Subscriptions
void I2CPWMController::handle_servo_absolute(
    const std::shared_ptr<ros2_i2cpwmboard::msg::ServoArray> msg) {
  for (const auto & s : msg->servos) {
    setPWM(s.servo, s.value);
  }
}

void I2CPWMController::handle_servo_proportional(
    const std::shared_ptr<ros2_i2cpwmboard::msg::ServoArray> msg) {
  for (const auto & s : msg->servos) {
    auto it = servo_configs_.find(s.servo);
    if (it != servo_configs_.end()) {
      int adjusted = static_cast<int>(static_cast<long long>(s.value) * it->second.scale / 100);
      setPWM(s.servo, adjusted);
    } else {
      setPWM(s.servo, s.value);
    }
  }
}

// Hardware-Abstraktion (Stub)
void I2CPWMController::setPWM(int channel, int value) {
  // TODO: konkrete I2C/PCA9685-Ansteuerung hier implementieren
  RCLCPP_DEBUG(this->get_logger(), "setPWM(channel=%d, value=%d)", channel, value);
}

// main
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<I2CPWMController>());
  rclcpp::shutdown();
  return 0;
}
