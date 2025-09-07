#include "rclcpp/rclcpp.hpp"
#include "ros2_i2cpwmboard/msg/servo_config.hpp"
#include "ros2_i2cpwmboard/srv/int_value.hpp"
#include "ros2_i2cpwmboard/srv/drive_mode.hpp"
#include "ros2_i2cpwmboard/srv/servos_config.hpp"
#include "ros2_i2cpwmboard/srv/stop_servos.hpp"
#include "ros2_i2cpwmboard/msg/servo_array.hpp"
#include "i2cpwm_controller.hpp"

I2CPWMController::I2CPWMController() : Node("i2cpwm_controller") {
    servo_configs_ = std::map<int, ros2_i2cpwmboard::msg::ServoConfig>();

    srv_set_value_ = this->create_service<ros2_i2cpwmboard::srv::IntValue>(
        "set_value",
        std::bind(&I2CPWMController::handle_set_value, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_drive_mode_ = this->create_service<ros2_i2cpwmboard::srv::DriveMode>(
        "drive_mode",
        std::bind(&I2CPWMController::handle_drive_mode, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_servos_config_ = this->create_service<ros2_i2cpwmboard::srv::ServosConfig>(
        "servos_config",
        std::bind(&I2CPWMController::handle_servos_config, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_stop_servos_ = this->create_service<ros2_i2cpwmboard::srv::StopServos>(
        "stop_servos",
        std::bind(&I2CPWMController::handle_stop_servos, this, std::placeholders::_1, std::placeholders::_2)
    );

    servo_absolute_sub_ = this->create_subscription<ros2_i2cpwmboard::msg::ServoArray>(
        "servos_absolute", 10, std::bind(&I2CPWMController::handle_servo_absolute, this, std::placeholders::_1)
    );

    servo_proportional_sub_ = this->create_subscription<ros2_i2cpwmboard::msg::ServoArray>(
        "servos_proportional", 10, std::bind(&I2CPWMController::handle_servo_proportional, this, std::placeholders::_1)
    );
}

void I2CPWMController::handle_set_value(const std::shared_ptr<ros2_i2cpwmboard::srv::IntValue::Request> request,
                                       std::shared_ptr<ros2_i2cpwmboard::srv::IntValue::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Got IntValue request: %d", request->value);
    setPWM(1, 0, request->value); // Channel 1, on_time=0, off_time=value
    response->error = 0; // Erfolg anzeigen
}

void I2CPWMController::handle_drive_mode(const std::shared_ptr<ros2_i2cpwmboard::srv::DriveMode::Request> request,
                                        std::shared_ptr<ros2_i2cpwmboard::srv::DriveMode::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Got DriveMode request: mode=%s, rpm=%d, radius=%d, track=%d, scale=%d",
                request->mode.c_str(), request->rpm, request->radius, request->track, request->scale);
    response->error = 0; // Erfolg anzeigen
}

void I2CPWMController::handle_servos_config(const std::shared_ptr<ros2_i2cpwmboard::srv::ServosConfig::Request> request,
                                           std::shared_ptr<ros2_i2cpwmboard::srv::ServosConfig::Response> response) {
    for (const auto& servo : request->servos) {
        if (servo_configs_.find(servo.servo) != servo_configs_.end()) {
            servo_configs_[servo.servo] = servo;
            setPWM(servo.servo, servo.on_time, servo.off_time);
        }
    }
    response->error = 0; // Erfolg anzeigen
}

void I2CPWMController::handle_stop_servos(const std::shared_ptr<ros2_i2cpwmboard::srv::StopServos::Request>,
                                         std::shared_ptr<ros2_i2cpwmboard::srv::StopServos::Response> response) {
    for (const auto& config : servo_configs_) {
        setPWM(config.first, 0, 0); // Stoppe alle Servos
    }
    response->error = 0; // Erfolg anzeigen
}

void I2CPWMController::handle_servo_absolute(const std::shared_ptr<ros2_i2cpwmboard::msg::ServoArray> msg) {
    for (const auto& servo : msg->servos) {
        if (servo_configs_.find(servo.servo) != servo_configs_.end()) {
            setPWM(servo.servo, servo.on_time, servo.off_time);
        }
    }
}

void I2CPWMController::handle_servo_proportional(const std::shared_ptr<ros2_i2cpwmboard::msg::ServoArray> msg) {
    for (const auto& servo : msg->servos) {
        if (servo_configs_.find(servo.servo) != servo_configs_.end()) {
            // Proportionale Anpassung (Platzhalter-Logik)
            int adjusted_off_time = servo.off_time * servo_configs_[servo.servo].scale / 100;
            setPWM(servo.servo, servo.on_time, adjusted_off_time);
        }
    }
}

void I2CPWMController::setPWM(int channel, int on_time, int off_time) {
    // Dummy-Implementierung: Ersetze dies mit der tatsächlichen I2C-Logik
    RCLCPP_INFO(this->get_logger(), "setPWM called: channel=%d, on_time=%d, off_time=%d", channel, on_time, off_time);
}
