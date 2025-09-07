#pragma once

#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <i2c_pwm_board_msgs/msg/servo.hpp>
#include <i2c_pwm_board_msgs/msg/servo_array.hpp>

#include <spot_micro_kinematics/spot_micro_kinematics.h>
#include <spot_micro_kinematics/spot_micro_leg.h>
#include <spot_micro_kinematics/utils.h>

#include "spot_micro_motion_cmd/command.h"
#include "spot_micro_motion_cmd/spot_micro_state.h"

namespace smmc {

struct ServoMap {
  int   neutral = 300;
  float counts_per_rad = 500.0f;
  std::array<int, 12> channels{0,1,2, 3,4,5, 6,7,8, 9,10,11};
};

class SpotMicroMotionCmd {
public:
  explicit SpotMicroMotionCmd(rclcpp::Node* node);

  void tick(float dt);
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
  i2c_pwm_board_msgs::msg::ServoArray anglesToServos(const smk::LegsJointAngles& a) const;

  rclcpp::Node* node_;  // nicht-owning
  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr pub_servos_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  // Kinematics
  smk::LinkLengths links_{ .l1 = 0.0f, .l2 = 0.08f, .l3 = 0.08f };
  smk::SpotMicroKinematics kin_{
      0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f,
      0.26f, 0.12f,
      links_
  };

  // FSM
  Context fsm_ctx_;
  Command command_;

  // Default-Zielfüße (Stehpose)
  smk::LegsFootPos default_feet_{
    smk::Point{ +0.10f, -0.06f, -0.15f}, // RB
    smk::Point{ -0.10f, -0.06f, -0.15f}, // RF
    smk::Point{ -0.10f, +0.06f, -0.15f}, // LF
    smk::Point{ +0.10f, +0.06f, -0.15f}  // LB
  };

  // Parameter
  ServoMap map_;
  double publish_rate_hz_{50.0};
};

} // namespace smmc
