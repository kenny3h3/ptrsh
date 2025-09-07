#pragma once

// ROS 2 message headers
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

// ---- Kompatibilitäts-Aliasse (ROS1-Namen -> ROS2-Typen) ----
// So kann bestehender Code weiterhin std_msgs::Float32 / geometry_msgs::Twist verwenden.
namespace std_msgs {
  using Float32 = msg::Float32;
}
namespace geometry_msgs {
  using Twist = msg::Twist;
}

// Falls du hier eigene Befehlsstrukturen brauchst, kannst du sie ergänzen.
// Beispiel (optional, nur wenn vom Code genutzt):
// struct Command {
//   float x{0.f}, y{0.f}, z{0.f};
//   float yaw_rate{0.f};
// };
