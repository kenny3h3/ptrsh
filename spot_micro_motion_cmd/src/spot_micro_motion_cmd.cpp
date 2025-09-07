#include "spot_micro_motion_cmd/spot_micro_motion_cmd.h"
#include <algorithm>
#include <chrono>
#include <cmath>

using i2c_pwm_board_msgs::msg::Servo;
using i2c_pwm_board_msgs::msg::ServoArray;

namespace smmc {

SpotMicroMotionCmd::SpotMicroMotionCmd(rclcpp::Node* node)
: node_(node)
{
  // Parameter
  node_->declare_parameter<int>("servo_neutral", map_.neutral);
  node_->declare_parameter<double>("servo_counts_per_rad", map_.counts_per_rad);
  node_->declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  node_->declare_parameter<double>("body_length", 0.26);
  node_->declare_parameter<double>("body_width", 0.12);
  node_->declare_parameter<double>("link_l2", links_.l2);
  node_->declare_parameter<double>("link_l3", links_.l3);
  node_->declare_parameter<double>("stance_z", -0.15);

  map_.neutral = node_->get_parameter("servo_neutral").as_int();
  map_.counts_per_rad = static_cast<float>(node_->get_parameter("servo_counts_per_rad").as_double());
  publish_rate_hz_ = node_->get_parameter("publish_rate_hz").as_double();

  links_.l2 = static_cast<float>(node_->get_parameter("link_l2").as_double());
  links_.l3 = static_cast<float>(node_->get_parameter("link_l3").as_double());
  const float body_L = static_cast<float>(node_->get_parameter("body_length").as_double());
  const float body_W = static_cast<float>(node_->get_parameter("body_width").as_double());

  kin_ = smk::SpotMicroKinematics(0.f,0.f,0.f, 0.f,0.f,0.f, body_L, body_W, links_);

  // FSM-Kontext
  fsm_ctx_.logger = node_->get_logger();
  fsm_ctx_.kin = &kin_;
  fsm_ctx_.target_feet = default_feet_;
  fsm_ctx_.setState(std::make_unique<Stand>());

  // Publisher / Subscriber
  pub_servos_ = node_->create_publisher<ServoArray>("servo_absolute", 10);
  sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&SpotMicroMotionCmd::onCmdVel, this, std::placeholders::_1));

  // Anfangshöhe
  command_.stance_z = static_cast<float>(node_->get_parameter("stance_z").as_double());
}

void SpotMicroMotionCmd::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  command_.twist = *msg;

  // einfacher Zustandswechsel:
  const double speed = std::hypot(msg->linear.x, msg->linear.y);
  if(speed > 1e-3 || std::abs(msg->angular.z) > 1e-3){
    if(dynamic_cast<Walk*>(fsm_ctx_.get()) == nullptr){
      fsm_ctx_.setState(std::make_unique<Walk>());
    }
  } else {
    if(dynamic_cast<Stand*>(fsm_ctx_.get()) == nullptr){
      fsm_ctx_.setState(std::make_unique<Stand>());
    }
  }
}

i2c_pwm_board_msgs::msg::ServoArray
SpotMicroMotionCmd::anglesToServos(const smk::LegsJointAngles& a) const
{
  std::array<float,12> q = {
    a.right_back.q1,  a.right_back.q2,  a.right_back.q3,
    a.right_front.q1, a.right_front.q2, a.right_front.q3,
    a.left_front.q1,  a.left_front.q2,  a.left_front.q3,
    a.left_back.q1,   a.left_back.q2,   a.left_back.q3
  };

  ServoArray arr;
  arr.servos.reserve(12);
  for (size_t i=0;i<q.size();++i) {
    Servo s;
    s.servo = map_.channels[i];
    s.value = map_.neutral + static_cast<int>(map_.counts_per_rad * q[i]);
    arr.servos.push_back(s);
  }
  return arr;
}

void SpotMicroMotionCmd::tick(float dt)
{
  // 1) FSM updaten -> setzt BodyState/Feet im Kin-Objekt
  if (fsm_ctx_.get()){
    fsm_ctx_.get()->update(fsm_ctx_, command_, dt);
  }

  // 2) IK & Servoausgabe
  kin_.setFeetPosGlobalCoordinates( kin_.getBodyState().leg_feet_pos );
  auto angs = kin_.getLegsJointAngles();
  auto msg  = anglesToServos(angs);
  pub_servos_->publish(msg);
}

} // namespace smmc
