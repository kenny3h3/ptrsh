#include <rclcpp/rclcpp.hpp>
#include "spot_micro_motion_cmd/spot_micro_motion_cmd.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("spot_micro_motion_cmd");

  smmc::SpotMicroMotionCmd app(node.get());

  // Publish-Rate einlesen oder Default 50Hz
  double hz = 50.0;
  if (node->has_parameter("publish_rate_hz"))
    hz = node->get_parameter("publish_rate_hz").as_double();
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0/hz));

  auto last = node->now();
  auto timer = node->create_wall_timer(period, [&, node, period, &last](){
    auto now = node->now();
    const float dt = static_cast<float>((now - last).seconds());
    last = now;
    app.tick(dt > 0.f ? dt : 1.f/static_cast<float>(hz));
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
