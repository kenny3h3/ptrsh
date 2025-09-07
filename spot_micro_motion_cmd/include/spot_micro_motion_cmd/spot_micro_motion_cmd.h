#pragma once

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <i2c_pwm_board_msgs/msg/servo.hpp>
#include <i2c_pwm_board_msgs/msg/servo_array.hpp>

// Kinematics
#include <spot_micro_kinematics/spot_micro_kinematics.h>
#include <spot_micro_kinematics/spot_micro_leg.h>
#include <spot_micro_kinematics/utils.h>

namespace smmc {

// Einfache Parameter-Struktur für Mapping Gelenkwinkel -> Servo-PWM
struct ServoMap {
  // PWM "value" (z.B. Off-Time) bei Winkel 0 rad
  int neutral = 300;
  // Skala Counts pro Radiant (linear)
  float counts_per_rad = 500.0f;
  // Kanalnummern der 12 Servos (RB hip/knee/ankle, RF..., LF..., LB...)
  std::array<int, 12> channels{
      0,1,2,  3,4,5,  6,7,8,  9,10,11
  };
};

// Hauptklasse: verwaltet Kinematics + ROS2 I/O
class SpotMicroMotionCmd {
public:
  explicit SpotMicroMotionCmd(rclcpp::Node* node);

  // Periodische Veröffentlichung
  void tick();

  // Callback von cmd_vel
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
  // Hilfen
  i2c_pwm_board_msgs::msg::ServoArray anglesToServos(const smk::LegsJointAngles& a) const;

  rclcpp::Node* node_;  // nicht-owning
  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr pub_servos_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  // Kinematics-Zustand
  smk::LinkLengths links_{ .l1 = 0.0f, .l2 = 0.08f, .l3 = 0.08f }; // Beispielwerte anpassen
  smk::SpotMicroKinematics kin_{
      0.0f, 0.0f, 0.0f,   // x,y,z
      0.0f, 0.0f, 0.0f,   // phi,theta,psi
      0.26f, 0.12f,       // body_length, body_width (Beispiel)
      links_
  };

  // Ziel-Fußpunkte (vereinfachter Stehpose)
  smk::LegsFootPos target_feet_{
    smk::Point{ 0.10f,-0.06f,-0.15f}, // RB
    smk::Point{-0.10f,-0.06f,-0.15f}, // RF
    smk::Point{-0.10f, 0.06f,-0.15f}, // LF
    smk::Point{ 0.10f, 0.06f,-0.15f}  // LB
  };

  // Parameter
  ServoMap map_;
  double publish_rate_hz_{50.0};
};

} // namespace smmc
