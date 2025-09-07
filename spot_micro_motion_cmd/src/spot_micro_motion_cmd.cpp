#include "spot_micro_motion_cmd/spot_micro_motion_cmd.h"
#include <algorithm>
#include <cmath>

using i2c_pwm_board_msgs::msg::Servo;
using i2c_pwm_board_msgs::msg::ServoArray;

namespace smmc {

SpotMicroMotionCmd::SpotMicroMotionCmd(rclcpp::Node* node)
: node_(node)
{
  // Parameter deklarieren/lesen
  node_->declare_parameter<int>("servo_neutral", map_.neutral);
  node_->declare_parameter<double>("servo_counts_per_rad", map_.counts_per_rad);
  node_->declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  node_->declare_parameter<double>("body_length", 0.26);
  node_->declare_parameter<double>("body_width", 0.12);
  node_->declare_parameter<double>("link_l2", links_.l2);
  node_->declare_parameter<double>("link_l3", links_.l3);

  map_.neutral = node_->get_parameter("servo_neutral").as_int();
  map_.counts_per_rad = static_cast<float>(node_->get_parameter("servo_counts_per_rad").as_double());
  publish_rate_hz_ = node_->get_parameter("publish_rate_hz").as_double();
  kin_ = smk::SpotMicroKinematics(
      0.f,0.f,0.f, 0.f,0.f,0.f,
      static_cast<float>(node_->get_parameter("body_length").as_double()),
      static_cast<float>(node_->get_parameter("body_width").as_double()),
      smk::LinkLengths{0.f,
        static_cast<float>(node_->get_parameter("link_l2").as_double()),
        static_cast<float>(node_->get_parameter("link_l3").as_double())
      });

  pub_servos_ = node_->create_publisher<ServoArray>("servo_absolute", 10);

  sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&SpotMicroMotionCmd::onCmdVel, this, std::placeholders::_1));
}

void SpotMicroMotionCmd::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // sehr einfache Body-Pose aus cmd_vel ableiten (Demo):
  // - vx/vy → kleine Translation
  // - yaw rate → kleine Roll um z
  const float dt = 1.0f / static_cast<float>(publish_rate_hz_);
  const float dx = static_cast<float>(msg->linear.x) * dt * 0.01f;
  const float dy = static_cast<float>(msg->linear.y) * dt * 0.01f;
  const float dpsi = static_cast<float>(msg->angular.z) * dt * 0.02f;

  // aktuelle Pose holen und updaten
  auto s = kin_.getBodyState();
  s.xyz_pos.x   += dx;
  s.xyz_pos.y   += dy;
  s.euler_angs.psi += dpsi;

  // Füße bleiben (quasi-Stand) an target_feet_
  s.leg_feet_pos = target_feet_;
  kin_.setBodyState(s);
}

ServoArray SpotMicroMotionCmd::anglesToServos(const smk::LegsJointAngles& a) const
{
  // Reihenfolge: RB(q1,q2,q3), RF, LF, LB
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

void SpotMicroMotionCmd::tick()
{
  // 1) gewünschte Fußpunkte → IK → Gelenkwinkel
  kin_.setFeetPosGlobalCoordinates(target_feet_);
  smk::LegsJointAngles angs = kin_.getLegsJointAngles();

  // 2) map auf Servos und publish
  auto msg = anglesToServos(angs);
  pub_servos_->publish(msg);
}

} // namespace smmc
