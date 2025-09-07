#pragma once
#include <geometry_msgs/msg/twist.hpp>

namespace smmc {

// Abstrakte Befehlsdarstellung für die FSM
struct Command {
  // Ziel-Geschwindigkeit (aus cmd_vel)
  geometry_msgs::msg::Twist twist;

  // Ziel-Standhöhe (negativ in Z, Meter)
  float stance_z = -0.15f;

  // einfache Skalierung für Schrittlänge
  float step_scale = 1.0f;
};

} // namespace smmc
