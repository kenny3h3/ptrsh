// Node file to create object and initializing the ROS node
#include "spot_micro_motion_cmd.h"
#include <iostream>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("spot_micro_motion_cmd_node");
  SpotMicroMotionCmd node_obj(node);

  auto dt = node_obj.getNodeConfig().dt;
  rclcpp::WallRate rate(1.0 / dt);

  if (node_obj.publishServoConfiguration()) {
    bool debug_mode = node_obj.getNodeConfig().debug_mode;
    while (rclcpp::ok()) {
      if (debug_mode) {
        auto begin = node->now();
        node_obj.runOnce();
        RCLCPP_INFO(node->get_logger(), "Loop time: %f s", (node->now() - begin).seconds());
      } else {
        node_obj.runOnce();
      }
      rate.sleep();
    }
  }
  rclcpp::shutdown();
  return 0;
}
