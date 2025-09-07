#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ros2_i2cpwmboard/srv/int_value.hpp>
#include <ros2_i2cpwmboard/srv/drive_mode.hpp>
#include <ros2_i2cpwmboard/srv/servos_config.hpp>
#include <ros2_i2cpwmboard/srv/stop_servos.hpp>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

using namespace std::chrono_literals;

class I2CPWMController : public rclcpp::Node
{
public:
  I2CPWMController() : Node("i2cpwm_controller")
  {
    // Declare and get I2C bus parameter
    this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
    std::string i2c_bus;
    this->get_parameter("i2c_bus", i2c_bus);

    // Open I2C bus
    i2c_fd_ = open(i2c_bus.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus %s: %s", i2c_bus.c_str(), std::strerror(errno));
      throw std::runtime_error("Failed to open I2C bus");
    }

    // Set PCA9685 address (default 0x40)
    if (ioctl(i2c_fd_, I2C_SLAVE, 0x40) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address: %s", std::strerror(errno));
      close(i2c_fd_);
      throw std::runtime_error("Failed to set I2C slave address");
    }

    // Initialize PCA9685
    initializePCA9685();

    RCLCPP_INFO(this->get_logger(), "I2C PWM Controller started on %s.", i2c_bus.c_str());

    // Services
    srv_set_value_ = this->create_service<i2cpwm_board_ros2::srv::IntValue>(
      "set_value",
      std::bind(&I2CPWMController::handle_set_value, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_drive_mode_ = this->create_service<i2cpwm_board_ros2::srv::DriveMode>(
      "drive_mode",
      std::bind(&I2CPWMController::handle_drive_mode, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_servos_config_ = this->create_service<i2cpwm_board_ros2::srv::ServosConfig>(
      "servos_config",
      std::bind(&I2CPWMController::handle_servos_config, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_stop_servos_ = this->create_service<i2cpwm_board_ros2::srv::StopServos>(
      "stop_servos",
      std::bind(&I2CPWMController::handle_stop_servos, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Subscribers for servo commands
    servo_absolute_sub_ = this->create_subscription<i2cpwm_board_ros2::msg::ServoArray>(
      "servos_absolute", 10, std::bind(&I2CPWMController::handle_servo_absolute, this, std::placeholders::_1)
    );

    servo_proportional_sub_ = this->create_subscription<i2cpwm_board_ros2::msg::ServoArray>(
      "servos_proportional", 10, std::bind(&I2CPWMController::handle_servo_proportional, this, std::placeholders::_1)
    );
  }

  ~I2CPWMController()
  {
    if (i2c_fd_ >= 0) {
      close(i2c_fd_);
    }
  }

private:
  int i2c_fd_ = -1;
  std::map<int, i2cpwm_board_ros2::msg::ServoConfig> servo_configs_;

  void initializePCA9685()
  {
    // Reset PCA9685
    writeByte(0x00, 0x00); // MODE1 register, reset
    usleep(1000);

    // Set frequency (50Hz for servos)
    uint8_t prescale = 121; // For 50Hz with 25MHz oscillator
    writeByte(0x00, 0x10); // Sleep mode
    writeByte(0xFE, prescale); // Set prescaler
    writeByte(0x00, 0x00); // Wake up
    usleep(1000);
    writeByte(0x00, 0x80); // Restart
  }

  void writeByte(uint8_t reg, uint8_t value)
  {
    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd_, buffer, 2) != 2) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to I2C: %s", std::strerror(errno));
    }
  }

  void setPWM(int channel, uint16_t on_time, uint16_t off_time)
  {
    uint8_t reg = 6 + 4 * (channel - 1); // LEDn_ON_L
    writeByte(reg, on_time & 0xFF);
    writeByte(reg + 1, (on_time >> 8) & 0xFF);
    writeByte(reg + 2, off_time & 0xFF);
    writeByte(reg + 3, (off_time >> 8) & 0xFF);
  }

  void handle_set_value(
    const std::shared_ptr<i2cpwm_board_ros2::srv::IntValue::Request> request,
    std::shared_ptr<i2cpwm_board_ros2::srv::IntValue::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Got IntValue request: %d", request->data);
    setPWM(1, 0, request->data); // Channel 1, on_time=0, off_time=data
    response->success = true;
  }

  void handle_drive_mode(
    const std::shared_ptr<i2cpwm_board_ros2::srv::DriveMode::Request> request,
    std::shared_ptr<i2cpwm_board_ros2::srv::DriveMode::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Drive mode: %s, RPM: %f, Radius: %f, Track: %f, Scale: %f",
                request->mode.c_str(), request->rpm, request->radius, request->track, request->scale);
    // TODO: Implement drive mode logic
    response->error = 0;
  }

  void handle_servos_config(
    const std::shared_ptr<i2cpwm_board_ros2::srv::ServosConfig::Request> request,
    std::shared_ptr<i2cpwm_board_ros2::srv::ServosConfig::Response> response)
  {
    for (const auto& servo : request->servos) {
      RCLCPP_INFO(this->get_logger(), "Configuring servo %d: center=%d, range=%d, direction=%d, center_angle=%f, max_angle=%f",
                  servo.servo, servo.center, servo.range, servo.direction, servo.center_angle_deg, servo.max_angle_deg);
      servo_configs_[servo.servo] = servo;
      // Initialize servo to center position
      int pwm_value = servo.center;
      setPWM(servo.servo, 0, pwm_value);
    }
    response->success = true;
  }

  void handle_stop_servos(
    const std::shared_ptr<i2cpwm_board_ros2::srv::StopServos::Request> request,
    std::shared_ptr<i2cpwm_board_ros2::srv::StopServos::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Stopping all servos");
    for (int i = 1; i <= 16; ++i) {
      setPWM(i, 0, 0); // Turn off PWM for all channels
    }
    response->success = true;
  }

  void handle_servo_absolute(
    const std::shared_ptr<i2cpwm_board_ros2::msg::ServoArray> msg)
  {
    for (const auto& servo : msg->servos) {
      if (servo_configs_.find(servo.servo) != servo_configs_.end()) {
        float max_angle = servo_configs_[servo.servo].max_angle_deg > 0 ? servo_configs_[servo.servo].max_angle_deg : 180.0;
        int pwm_value = static_cast<int>(servo_configs_[servo.servo].center +
                                         servo.value * servo_configs_[servo.servo].range / max_angle *
                                         servo_configs_[servo.servo].direction);
        setPWM(servo.servo, 0, pwm_value);
      }
    }
  }

  void handle_servo_proportional(
    const std::shared_ptr<i2cpwm_board_ros2::msg::ServoArray> msg)
  {
    for (const auto& servo : msg->servos) {
      if (servo_configs_.find(servo.servo) != servo_configs_.end()) {
        float max_angle = servo_configs_[servo.servo].max_angle_deg > 0 ? servo_configs_[servo.servo].max_angle_deg : 180.0;
        int pwm_value = static_cast<int>(servo_configs_[servo.servo].center +
                                         servo.value * servo_configs_[servo.servo].range / max_angle *
                                         servo_configs_[servo.servo].direction);
        setPWM(servo.servo, 0, pwm_value);
      }
    }
  }

  rclcpp::Service<i2cpwm_board_ros2::srv::IntValue>::SharedPtr srv_set_value_;
  rclcpp::Service<i2cpwm_board_ros2::srv::DriveMode>::SharedPtr srv_drive_mode_;
  rclcpp::Service<i2cpwm_board_ros2::srv::ServosConfig>::SharedPtr srv_servos_config_;
  rclcpp::Service<i2cpwm_board_ros2::srv::StopServos>::SharedPtr srv_stop_servos_;
  rclcpp::Subscription<i2cpwm_board_ros2::msg::ServoArray>::SharedPtr servo_absolute_sub_;
  rclcpp::Subscription<i2cpwm_board_ros2::msg::ServoArray>::SharedPtr servo_proportional_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<I2CPWMController>());
  rclcpp::shutdown();
  return 0;
}
