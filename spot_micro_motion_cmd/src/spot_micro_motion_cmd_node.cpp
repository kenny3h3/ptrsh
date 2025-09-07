#include <rclcpp/rclcpp.hpp>
#include "spot_micro_motion_cmd/spot_micro_motion_cmd.h"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("spot_micro_motion_cmd");

  smmc::SpotMicroMotionCmd app(node.get());

  // Timer für periodisches Publizieren
  const double hz = node->get_parameter("publish_rate_hz").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET
                      ? node->get_parameter("publish_rate_hz").as_double()
                      : 50.0;
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));

  auto timer = node->create_wall_timer(period, [&app]() {
    app.tick();
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
