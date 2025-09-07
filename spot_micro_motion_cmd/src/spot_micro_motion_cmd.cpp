#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "ros2_i2cpwmboard/msg/servo.hpp"
#include "ros2_i2cpwmboard/msg/servo_array.hpp"
#include "ros2_i2cpwmboard/srv/servos_config.hpp"
#include "ros2_i2cpwmboard/srv/stop_servos.hpp"

#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "spot_micro_state.h"
#include "utils.h"

using namespace smk;
using namespace rclcpp;
using namespace std_msgs::msg;
using namespace geometry_msgs::msg;
using namespace ros2_i2cpwmboard::msg;
using namespace ros2_i2cpwmboard::srv;

SpotMicroMotionCmd::SpotMicroMotionCmd(Node::SharedPtr node) : node_(node) {
  state_ = std::make_unique<SpotMicroIdleState>();
  cmd_ = Command();
  readInConfigParameters();

  // Publishers
  servos_absolute_pub_ = node_->create_publisher<ServoArray>("servos_absolute", 10);
  servos_proportional_pub_ = node_->create_publisher<ServoArray>("servos_proportional", 10);
  body_state_pub_ = node_->create_publisher<Float32MultiArray>("body_state", 10);
  lcd_state_string_pub_ = node_->create_publisher<String>("lcd_state_string", 10);
  lcd_vel_cmd_pub_ = node_->create_publisher<Twist>("lcd_vel_cmd", 10);
  lcd_angle_cmd_pub_ = node_->create_publisher<Vector3>("lcd_angle_cmd", 10);

  // Subscribers
  stand_sub_ = node_->create_subscription<Bool>("/stand_cmd", 10, std::bind(&SpotMicroMotionCmd::standCommandCallback, this, std::placeholders::_1));
  idle_sub_ = node_->create_subscription<Bool>("/idle_cmd", 10, std::bind(&SpotMicroMotionCmd::idleCommandCallback, this, std::placeholders::_1));
  walk_sub_ = node_->create_subscription<Bool>("/walk_cmd", 10, std::bind(&SpotMicroMotionCmd::walkCommandCallback, this, std::placeholders::_1));
  angle_sub_ = node_->create_subscription<Vector3>("/angle_cmd", 10, std::bind(&SpotMicroMotionCmd::angleCommandCallback, this, std::placeholders::_1));
  vel_sub_ = node_->create_subscription<Twist>("/vel_cmd", 10, std::bind(&SpotMicroMotionCmd::velCommandCallback, this, std::placeholders::_1));

  // TF Broadcasters
  transform_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  static_transform_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  // Timer
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(static_cast<int>(smnc_.dt * 1000)), std::bind(&SpotMicroMotionCmd::runOnce, this));

  // Initialize static transforms
  publishStaticTransforms();

  // Servo config
  publishServoConfiguration();
}

SpotMicroMotionCmd::~SpotMicroMotionCmd() {
  // Cleanup
}

void SpotMicroMotionCmd::runOnce() {
  handleInputCommands();
  publishBodyState();
  publishLcdMonitorData();
  publishDynamicTransforms();
  integrateOdometry();
  rclcpp::spin_some(node_);
}

void SpotMicroMotionCmd::readInConfigParameters() {
  // Use rclcpp to load from YAML
  auto param_desc = rclcpp::parameter::ParameterDescriptor();
  node_->declare_parameter("hip_link_length", 0.055, param_desc);
  node_->get_parameter("hip_link_length", smnc_.smc.hip_link_length);
  // ... (Declare and get all other parameters from the YAML file)
  // Servo parameters
  node_->declare_parameter("num_servos", 12, param_desc);
  node_->get_parameter("num_servos", smnc_.num_servos);
  node_->declare_parameter("servo_max_angle_deg", 270.0, param_desc);
  node_->get_parameter("servo_max_angle_deg", smnc_.servo_max_angle_deg);
  // Add other parameters as needed
}

bool SpotMicroMotionCmd::publishServoConfiguration() {
  auto client = node_->create_client<ServosConfig>("servos_config");
  auto request = std::make_shared<ServosConfig::Request>();
  for (const auto& servo : smnc_.servo_config) {
    ServoConfig config;
    config.servo = servo.second.at("num");
    config.center = servo.second.at("center");
    config.range = servo.second.at("range");
    config.direction = servo.second.at("direction");
    config.center_angle_deg = servo.second.at("center_angle_deg");
    config.max_angle_deg = servo.second.at("max_angle_deg");
    request->servos.push_back(config);
  }
  if (!client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "Servo config service not available");
    return false;
  }
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
    return result.get()->success;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call servo config service");
    return false;
  }
}

void SpotMicroMotionCmd::publishZeroServoAbsoluteCommand() {
  servos_absolute_cmd_.servos.clear();
  for (int i = 1; i <= smnc_.num_servos; ++i) {
    Servo servo;
    servo.servo = i;
    servo.value = 0.0f;
    servos_absolute_cmd_.servos.push_back(servo);
  }
  servos_absolute_pub_->publish(servos_absolute_cmd_);
}

void SpotMicroMotionCmd::publishServoProportionalCommand() {
  servos_proportional_cmd_.servos.clear();
  LegsJointAngles joint_angles = smk_.getLegsJointAngles();
  std::map<std::string, JointAngles> legs = {
    {"RF", joint_angles.right_front},
    {"RB", joint_angles.right_back},
    {"LF", joint_angles.left_front},
    {"LB", joint_angles.left_back}
  };
  for (const auto& leg : legs) {
    Servo servo1, servo2, servo3;
    servo1.servo = smnc_.servo_config[leg.first + "_1"]["num"];
    servo1.value = (leg.second.ang1 * 180.0 / M_PI) * smnc_.servo_config[leg.first + "_1"]["direction"];
    servo2.servo = smnc_.servo_config[leg.first + "_2"]["num"];
    servo2.value = (leg.second.ang2 * 180.0 / M_PI) * smnc_.servo_config[leg.first + "_2"]["direction"];
    servo3.servo = smnc_.servo_config[leg.first + "_3"]["num"];
    servo3.value = (leg.second.ang3 * 180.0 / M_PI) * smnc_.servo_config[leg.first + "_3"]["direction"];
    servos_proportional_cmd_.servos.push_back(servo1);
    servos_proportional_cmd_.servos.push_back(servo2);
    servos_proportional_cmd_.servos.push_back(servo3);
  }
  servos_proportional_pub_->publish(servos_proportional_cmd_);
}

void SpotMicroMotionCmd::standCommandCallback(const Bool::SharedPtr msg) {
  cmd_.stand_cmd_ = msg->data;
}

void SpotMicroMotionCmd::idleCommandCallback(const Bool::SharedPtr msg) {
  cmd_.idle_cmd_ = msg->data;
}

void SpotMicroMotionCmd::walkCommandCallback(const Bool::SharedPtr msg) {
  cmd_.walk_cmd_ = msg->data;
}

void SpotMicroMotionCmd::angleCommandCallback(const Vector3::SharedPtr msg) {
  cmd_.phi_cmd_ = msg->x;
  cmd_.theta_cmd_ = msg->y;
  cmd_.psi_cmd_ = msg->z;
}

void SpotMicroMotionCmd::velCommandCallback(const Twist::SharedPtr msg) {
  cmd_.x_vel_cmd_mps_ = msg->linear.x;
  cmd_.y_vel_cmd_mps_ = msg->linear.y;
  cmd_.yaw_rate_cmd_rps_ = msg->angular.z;
}

void SpotMicroMotionCmd::resetEventCommands() {
  cmd_.resetEventCmds();
}

void SpotMicroMotionCmd::handleInputCommands() {
  smk::BodyState body_state_cmd;
  state_->handleInputCommands(smk::BodyState(), smnc_, cmd_, this, &body_state_cmd);
}

void SpotMicroMotionCmd::changeState(std::unique_ptr<SpotMicroState> sms) {
  state_ = std::move(sms);
}

void SpotMicroMotionCmd::publishBodyState() {
  body_state_msg_.data.clear();
  body_state_pub_->publish(body_state_msg_);
}

void SpotMicroMotionCmd::publishLcdMonitorData() {
  lcd_state_string_msg_.data = state_->getCurrentStateName();
  lcd_state_string_pub_->publish(lcd_state_string_msg_);
  lcd_vel_cmd_msg_.linear.x = cmd_.x_vel_cmd_mps_;
  lcd_vel_cmd_msg_.linear.y = cmd_.y_vel_cmd_mps_;
  lcd_vel_cmd_msg_.angular.z = cmd_.yaw_rate_cmd_rps_;
  lcd_vel_cmd_pub_->publish(lcd_vel_cmd_msg_);
  lcd_angle_cmd_msg_.x = cmd_.phi_cmd_;
  lcd_angle_cmd_msg_.y = cmd_.theta_cmd_;
  lcd_angle_cmd_msg_.z = cmd_.psi_cmd_;
  lcd_angle_cmd_pub_->publish(lcd_angle_cmd_msg_);
}

void SpotMicroMotionCmd::publishStaticTransforms() {
  auto transform = createTransform("base_link", "lidar", smnc_.lidar_x_pos, smnc_.lidar_y_pos, smnc_.lidar_z_pos, 0.0, 0.0, smnc_.lidar_yaw_angle * M_PI / 180.0);
  static_transform_br_->sendTransform(transform);
}

void SpotMicroMotionCmd::publishDynamicTransforms() {
  AllRobotRelativeTransforms transforms = smk_.getRobotTransforms();
  auto body_tf = eigAndFramesToTrans(matrix4fToAffine3d(transforms.bodyCenter), "world", "base_link");
  transform_br_->sendTransform(body_tf);
}

void SpotMicroMotionCmd::integrateOdometry() {
  odom_tf_.matrix()(0,3) += cmd_.x_vel_cmd_mps_ * smnc_.dt;
  odom_tf_.matrix()(1,3) += cmd_.y_vel_cmd_mps_ * smnc_.dt;
  tf2::Quaternion q;
  q.setRPY(0, 0, cmd_.yaw_rate_cmd_rps_ * smnc_.dt);
  odom_tf_ = odom_tf_ * tf2::toEigen(q);
}

Eigen::Affine3d SpotMicroMotionCmd::getOdometryTransform() {
  return odom_tf_;
}
